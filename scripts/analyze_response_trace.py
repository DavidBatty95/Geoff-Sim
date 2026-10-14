#!/usr/bin/env python3
"""R1 timing/coverage report from a retained trace; never infer missing causal links."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def distribution(values):
    if not values:
        return dict(count=0,min=None,median=None,p95=None,p99=None,max=None)
    ordered = sorted(values)
    def quantile(fraction):
        index = (len(ordered)-1)*fraction
        lo = int(index)
        return ordered[lo]+(ordered[min(lo+1,len(ordered)-1)]-ordered[lo])*(index-lo)
    return dict(count=len(values),min=ordered[0],median=quantile(.5),p95=quantile(.95),p99=quantile(.99),max=ordered[-1])


def analyze(rows):
    samples,invalid = defaultdict(list),Counter()
    topics,states,safety_modes = Counter(),Counter(),Counter()
    seen = set()
    duplicates = clock_regressions = counter_resets = 0
    last_receipt = None
    latest_counters = {}
    latest_counter_stamp = None
    interruption_counters = {}
    policy_holds = {}
    budget_samples = budget_overruns = period_samples = period_overruns = 0
    active_period_samples = active_period_overruns = 0
    active_plans = no_feasible = 0
    rtf_anchor = None
    rtf_clock = None
    planner_envelopes = {}
    supervisor_envelopes = []
    escape_states = Counter()
    escape_worker = None
    worker_completed = worker_counter_resets = unobserved_worker_completions = 0

    def add(name,value):
        if finite(value) and value>=0.:
            samples[name].append(float(value))
        elif value is not None:
            invalid[name] += 1

    for row in rows:
        if not isinstance(row,dict) or not isinstance(row.get('topic'),str):
            invalid['record_shape'] += 1
            continue
        topic,data = row['topic'],row.get('data')
        topics[topic] += 1
        receipt = row.get('stamp')
        if finite(receipt):
            if last_receipt is not None and receipt<last_receipt:
                clock_regressions += 1
                rtf_anchor = None
            last_receipt = receipt
        if not isinstance(data,dict):
            continue
        if topic == 'planner_debug':
            envelope = data.get('command_sequence',{})
            if isinstance(envelope,dict) and isinstance(envelope.get('publisher_id'),str) and type(envelope.get('sequence_id')) is int:
                planner_envelopes[(envelope['publisher_id'],envelope['sequence_id'])] = envelope
            states[data.get('state','unknown')] += 1
            add('planner_wall_ms',data.get('planning_ms'))
            add('planner_thread_ms',data.get('planning_thread_ms'))
            for phase,value in data.get('scoring_phases_ms',{}).items():
                add('scoring_'+phase,value)
            for source in ('scan','tracks','prediction'):
                add(source+'_age_at_command_publish_s',data.get(source+'_age_at_command_publish_s'))
                # Legacy planning-snapshot ages are kept distinct from publish ages.
                add(source+'_age_at_planning_snapshot_s',data.get(source+'_age_s'))
            elapsed,budget,period = data.get('planning_ms'),data.get('planning_budget_ms'),data.get('planner_period_s')
            if finite(elapsed) and finite(budget) and elapsed>=0. and budget>0.:
                budget_samples += 1
                budget_overruns += elapsed>budget
            if finite(elapsed) and finite(period) and elapsed>=0. and period>0.:
                period_samples += 1
                period_overruns += elapsed>period*1000.
            if data.get('mission_active') is True:
                active_plans += 1
                no_feasible += data.get('selected') == 'none'
                add('planner_wall_active_ms',data.get('planning_ms'))
                if finite(elapsed) and finite(period) and elapsed>=0. and period>0.:
                    active_period_samples += 1
                    active_period_overruns += elapsed>period*1000.
        elif topic == 'safety_debug':
            if data.get('async_escape_search') is True:
                escape_states[data.get('async_escape_state','unknown')] += 1
                if isinstance(data.get('escape_worker'),dict):
                    escape_worker = data['escape_worker']
                    completed = escape_worker.get('completed')
                    if type(completed) is int and completed>=0:
                        if completed<worker_completed:
                            worker_counter_resets += 1
                            worker_completed = 0
                        if completed>worker_completed:
                            unobserved_worker_completions += completed-worker_completed-1
                            add('escape_worker_submit_to_complete_ms',escape_worker.get('last_search_ms'))
                        worker_completed = completed
            add('supervisor_tick_to_publication_ms',data.get('tick_to_publication_ms'))
            add('supervisor_tick_interval_ms',data.get('tick_interval_ms'))
            add('supervisor_predictive_search_ms',data.get('predictive_search_ms'))
            add('supervisor_escape_control_path_ms',data.get('escape_control_path_ms'))
            if data.get('command_envelope_valid') is True:
                supervisor_envelopes.append((data.get('command_publisher_id'),data.get('command_sequence_id'),data.get('input_cmd')))
            safety_modes[data.get('mode','unknown')] += 1
            add('supervisor_input_age_s',data.get('input_age_s'))
        elif topic == 'policy_timing':
            identity = (data.get('generation'),data.get('sequence'),data.get('submitted_monotonic_s'))
            if any(not finite(part) for part in identity):
                invalid['policy_identity'] += 1
                continue
            if identity in seen:
                duplicates += 1
                continue
            seen.add(identity)
            if data.get('timing_valid') is not True:
                invalid['policy_timing'] += 1
                continue
            for name in ('submit_to_complete_ms','complete_to_publish_ms','submit_to_publish_ms'):
                add('policy_'+name,data.get(name))
            if data.get('context_matched') is True:
                for name in ('lowstate_dds_source_to_callback_ms','lowstate_dds_receive_to_callback_ms',
                             'lowstate_dds_source_to_target_publish_ms'):
                    add(name,data.get(name))
                for name in ('command_receipt_age_at_target_publish_s','lowstate_receipt_age_at_target_publish_s'):
                    add(name,data.get(name))
            else:
                invalid['policy_context'] += 1
        elif topic == 'policy_diagnostics':
            events = data.get('policy_hold_events',[])
            if not isinstance(events,list):
                invalid['policy_hold_event'] += 1
                events = []
            for event in events:
                if not isinstance(event,dict) or not finite(event.get('stamp')) or type(event.get('sequence')) is not int:
                    invalid['policy_hold_event'] += 1
                    continue
                policy_holds[(event['stamp'],event['sequence'])] = event
            counters = data.get('counters',{})
            if isinstance(counters,dict) and all(finite(v) and v>=0 for v in counters.values()):
                if any(value<latest_counters.get(key,0) for key,value in counters.items()):
                    counter_resets += 1
                latest_counters = counters
                latest_counter_stamp = data.get('stamp')
                interruption_counters = {key:data[key] for key in
                    ('stop_request_invalidations','stale_result_rejections') if key in data}
        elif topic in ('odom','policy_cmd') and finite(receipt) and finite(data.get('source_stamp')):
            add(topic+'_source_age_at_logger_receipt_s',receipt-data['source_stamp'])
        elif topic == 'evaluation_aligned' and finite(data.get('sim_time')):
            clock = 'monotonic' if finite(row.get('receipt_monotonic_s')) else 'ros_legacy'
            wall = row['receipt_monotonic_s'] if clock == 'monotonic' else receipt
            sim = data['sim_time']
            if not finite(wall):
                continue
            if rtf_anchor is None or clock!=rtf_clock or wall<rtf_anchor[0] or sim<rtf_anchor[1]:
                rtf_anchor,rtf_clock = (wall,sim),clock
            elif wall-rtf_anchor[0]>=1.:
                add('rtf_'+clock+'_receipt_windows',(sim-rtf_anchor[1])/(wall-rtf_anchor[0]))
                rtf_anchor = (wall,sim)
    metric_names = ('planner_wall_ms','planner_thread_ms','scan_age_at_command_publish_s',
        'tracks_age_at_command_publish_s','prediction_age_at_command_publish_s',
        'policy_submit_to_complete_ms','policy_complete_to_publish_ms','policy_submit_to_publish_ms',
        'command_receipt_age_at_target_publish_s','lowstate_receipt_age_at_target_publish_s',
        'lowstate_dds_source_to_callback_ms','lowstate_dds_receive_to_callback_ms',
        'lowstate_dds_source_to_target_publish_ms',
        'odom_source_age_at_logger_receipt_s','supervisor_input_age_s','planner_wall_active_ms',
        'supervisor_tick_to_publication_ms','supervisor_tick_interval_ms',
        'supervisor_predictive_search_ms','supervisor_escape_control_path_ms',
        'escape_worker_submit_to_complete_ms',
        'rtf_monotonic_receipt_windows','rtf_ros_legacy_receipt_windows')
    metrics = {name:distribution(samples[name]) for name in sorted(set(samples)|set(metric_names))}
    matched = mismatched = missing = 0
    for publisher,sequence,command in supervisor_envelopes:
        envelope = planner_envelopes.get((publisher,sequence))
        if envelope is None:
            missing += 1
        elif (not isinstance(command,dict) or not isinstance(envelope.get('first'),list) or len(envelope['first'])!=3 or
              any(not finite(command.get(axis)) or not finite(value) or abs(command[axis]-value)>1e-4
                  for axis,value in zip(('x','y','z'),envelope['first']))):
            mismatched += 1
        else:
            matched += 1
    return dict(schema_version=1,role='response_observability_diagnostic_not_acceptance',
        topics=dict(topics),metrics=metrics,invalid_samples=dict(invalid),
        ros_receipt_clock_regressions=clock_regressions,duplicate_policy_events=duplicates,
        planner_budget=dict(samples=budget_samples,overruns=budget_overruns,
                            fraction=budget_overruns/budget_samples if budget_samples else None),
        planner_period=dict(samples=period_samples,overruns=period_overruns,
                            fraction=period_overruns/period_samples if period_samples else None),
        planner_active_period=dict(samples=active_period_samples,overruns=active_period_overruns,
                                   fraction=active_period_overruns/active_period_samples if active_period_samples else None),
        active_plans=active_plans,no_feasible_plans=no_feasible,
        planner_states=dict(states),supervisor_modes=dict(safety_modes),
        policy_counters_latest=latest_counters,policy_counter_resets=counter_resets,
        policy_counters_last_ros_s=latest_counter_stamp,
        policy_interruption_counters_latest=interruption_counters,
        policy_hold_events=[policy_holds[key] for key in sorted(policy_holds)],
        planner_supervisor_lineage=dict(eligible=len(supervisor_envelopes),matched=matched,
            missing_planner_record=missing,command_mismatch=mismatched,
            scope='atomic planner request considered by supervisor, not proof of physical execution'),
        async_escape_states=dict(escape_states),escape_worker_latest=escape_worker,
        escape_worker_counter_resets=worker_counter_resets,
        unobserved_worker_completion_durations=unobserved_worker_completions,
        limitations=[
            'Input ages are not causal end-to-end sensor-to-actuator latency.',
            'Policy publication is upstream of arm composition and bridge/DDS; motor application is unmeasured.',
            'LowState age is receipt-based; source acquisition time is unavailable.',
            'Command-to-motion onset and stopping require the separate fixed-command measurement protocol.',
            'RTF windows use logger receipt timing and include delivery jitter.',
            'Periodic cumulative counters can precede the final inference event; their timestamp is retained.',
            'Missing metrics remain null; legacy traces do not gain new measurements retroactively.'])


def write_report(trace,output):
    trace,output = Path(trace),Path(output)
    digest = hashlib.sha256()
    malformed = [0]
    def rows():
        with trace.open('rb') as stream:
            for line in stream:
                digest.update(line)
                try:
                    yield json.loads(line)
                except (ValueError,UnicodeDecodeError):
                    malformed[0] += 1
    result = analyze(rows())
    result.update(trace_name=trace.name,trace_sha256=digest.hexdigest(),malformed_lines=malformed[0],
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result = write_report(args.trace,args.output)
    print(json.dumps(dict(output=str(args.output),topics=result['topics'],
        planner_period=result['planner_period'],malformed_lines=result['malformed_lines'])))


if __name__ == '__main__':
    main()
