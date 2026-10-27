#!/usr/bin/env python3
"""Publish every declared stage-validation attempt, including falls and failures."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            value.update(block)
    return value.hexdigest()


def summarize(run):
    path=run/'result.json'
    result=json.loads(path.read_text()) if path.exists() else dict(success=False,error='result missing')
    manifest=json.loads((run/'manifest.json').read_text())
    outcomes=Counter()
    strategic=Counter()
    first_pincer=None
    strategic_search_ms=[]
    search_reasons=Counter()
    revalidation_ms=[]
    latest_worker={}
    proposal_margins=[]
    primitive_commands={}
    supervisor=Counter()
    supervisor_modes=Counter()
    phase='unknown'
    milestone_seen=False
    primitive_periods=Counter()
    primitive_phases=Counter()
    task_end=result.get('task_end_monotonic_s')
    continuations=Counter()
    forecast_sources=Counter()
    minimum=math.inf
    overlaps=0
    if (run/'trace.jsonl').exists():
        with (run/'trace.jsonl').open() as stream:
            for line in stream:
                row=json.loads(line)
                data=row['data']
                if row['topic']=='transport_state':
                    phase=data.get('phase','unknown')
                    milestone_seen |= phase in ('complete','survey_complete','pickup_complete','probe_complete')
                if row['topic']=='planner_debug':
                    forecast_sources[data.get('forecast_source','unrecorded')]+=1
                    outcomes[(data.get('kinodynamic_status'),data.get('selected'))]+=1
                    sequence=data.get('command_sequence',{})
                    if data.get('selected')=='escape_primitive' and sequence.get('publisher_id') is not None:
                        primitive_commands[(sequence['publisher_id'],sequence['sequence_id'])]=sequence['first']
                        after=(row.get('receipt_monotonic_s',math.inf)>=task_end) if task_end is not None else milestone_seen
                        primitive_periods['after_task' if after else 'before_task_end']+=1
                        primitive_phases[phase]+=1
                    if data.get('strategic_avoidance',{}).get('active'):
                        strategic['active_ticks']+=1
                        strategic['commitment_ticks']+=bool(data['strategic_avoidance'].get('continuation_active'))
                        strategic['closing_corridor_ticks']+=data['strategic_avoidance'].get('reason')=='closing_corridor'
                        first_pincer=row['stamp'] if first_pincer is None else first_pincer
                        refuges=data['strategic_avoidance'].get('refuges',[])
                        strategic['clear_coarse_refuge_ticks']+=bool(refuges and refuges[0]['minimum_margin_m']>=0.)
                        strategic['complete_plan_proposals']+=data.get('kinodynamic_status')=='proposed'
                        strategic['kinodynamic_selections']+=data.get('selected')=='kinodynamic'
                        strategic['primitive_selections']+=data.get('selected')=='escape_primitive'
                        cmd=data.get('selected_cmd') or {}
                        strategic['primitive_translation_selections']+=(data.get('selected')=='escape_primitive' and
                            math.hypot(cmd.get('x',0.),cmd.get('y',0.))>.05)
                        details=data.get('kinodynamic_details',{})
                        if details.get('continuation'):
                            continuations[details['continuation']]+=1
                        returned=details.get('worker_result',{})
                        if returned.get('reason')=='complete' and returned.get('minimum_margin_m') is not None:
                            proposal_margins.append(returned['minimum_margin_m'])
                        strategic['delay_checked_proposals']+=(returned.get('reason')=='complete' and returned.get('stage')=='admission')
                        for search_phase in ('worker_result','revalidation','beam'):
                            if details.get(search_phase,{}).get('reason'):
                                search_reasons[search_phase+':'+details[search_phase]['reason']]+=1
                        if details.get('reason'):
                            search_reasons[details['reason']]+=1
                        if details.get('revalidation',{}).get('elapsed_ms') is not None:
                            revalidation_ms.append(details['revalidation']['elapsed_ms'])
                        if details.get('worker'):
                            latest_worker=details['worker']
                        elapsed=data.get('planning_phases_ms',{}).get('kinodynamic_ms')
                        if elapsed is not None:
                            strategic_search_ms.append(elapsed)
                elif row['topic']=='evaluation_aligned':
                    for p in data['people']:
                        distance=math.hypot(data['robot']['x']-p['x'],data['robot']['y']-p['y'])-.35-p['radius_m']
                        minimum=min(minimum,distance)
                        overlaps+=distance<0.
        if primitive_commands:
            with (run/'trace.jsonl').open() as stream:
                for line in stream:
                    row=json.loads(line)
                    if row['topic']!='safety_debug':
                        continue
                    data=row['data']
                    expected=primitive_commands.get((data.get('command_publisher_id'),data.get('command_sequence_id')))
                    if expected is None:
                        continue
                    output=data.get('output_cmd',{})
                    supervisor['samples']+=1
                    supervisor_modes[data.get('mode','unknown')]+=1
                    preserved=(data.get('command_envelope_valid') is True and
                        all(abs(output.get(axis,math.inf)-value)<1e-3 for axis,value in zip(('x','y','z'),expected)))
                    supervisor['preserved_translation']+=bool(preserved and math.hypot(*expected[:2])>.05)
                    supervisor['zero_translation']+=math.hypot(output.get('x',math.inf),output.get('y',math.inf))<1e-6
    events=result.get('events',[])
    return dict(run_id=run.name,configuration={key:manifest.get(key) for key in
        ('people','seed','duration_s','test_stage','contact_force_capture','build_reused','strategic_avoidance',
         'record_invalidation_audit','forecast_change_guard')},
        result={key:value for key,value in result.items() if key!='events'},
        phase_counts=dict(Counter(event['phase'] for event in events)),
        recovery_events=[event for event in events if event['phase'] in
                         ('survey_yield','survey_reobserve','survey_resume','turn_displaced','body_instability')],
        full_logger_tail=dict(overlap_samples=overlaps,minimum_clearance_m=minimum if math.isfinite(minimum) else None),
        planner_outcomes=[dict(status=k[0],selected=k[1],count=v) for k,v in outcomes.items()],
        strategic_summary=dict(counters=dict(strategic),first_active_ros_s=first_pincer,
            search_max_ms=max(strategic_search_ms,default=None),
            search_mean_ms=sum(strategic_search_ms)/len(strategic_search_ms) if strategic_search_ms else None,
            search_reasons=dict(search_reasons),latest_worker=latest_worker,
            revalidation_max_ms=max(revalidation_ms,default=None),
            revalidation_mean_ms=sum(revalidation_ms)/len(revalidation_ms) if revalidation_ms else None,
            minimum_returned_margin_m=min(proposal_margins,default=None)),
        continuation_outcomes=dict(continuations),
        forecast_sources=dict(forecast_sources),
        primitive_supervision=dict(counters=dict(supervisor),modes=dict(supervisor_modes),
            scope='matched planner/supervisor publications, not motor-application acknowledgement',
            planner_periods=dict(primitive_periods),planner_phases=dict(primitive_phases),
            period_scope='logger receipt relative to explicit task end' if task_end is not None else
                'logger phase relative to completion milestone; failures without a milestone cannot isolate shutdown tail'),
        provenance={name:digest(run/name) for name in
                    ('result.json','manifest.json','source_snapshot.zip','effective_parameters.json','trace.jsonl') if (run/name).exists()})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='layered development validation; diagnostic completion is not delivery success',
                source_scope='each attempt retains its own source snapshot; development configurations may differ',
                attempts=[summarize(run) for run in args.runs])
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for attempt in report['attempts']:
        print(json.dumps(dict(run_id=attempt['run_id'],result={key:attempt['result'].get(key) for key in
              ('success','diagnostic_complete','fallen','contact_samples','error')})))


if __name__=='__main__':
    main()
