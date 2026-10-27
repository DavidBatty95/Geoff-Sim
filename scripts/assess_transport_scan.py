#!/usr/bin/env python3
"""Summarize retained survey failures without feeding evaluator truth to control."""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def planar(data):
    return math.hypot(data['x'], data['y'])


def nearest(rows, stamp):
    return min(rows, key=lambda row: abs(row['stamp'] - stamp))


def assess(directory):
    result = json.loads((directory / 'result.json').read_text())
    manifest = json.loads((directory / 'manifest.json').read_text())
    timing = json.loads((directory / 'response_diagnostics.json').read_text())
    rows = [json.loads(line) for line in (directory / 'trace.jsonl').read_text().splitlines()]
    topics = {}
    for row in rows:
        topics.setdefault(row['topic'], []).append(row)
    aligned = topics['evaluation_aligned']
    policy = topics['policy_cmd']
    planner = topics['planner_debug']
    translating = [row for row in policy if planar(row['data']) > .05]
    first = translating[0] if translating else None
    overlaps = []
    minimum = math.inf
    for row in aligned:
        data = row['data']
        for person in data['people']:
            clearance = math.hypot(data['robot']['x']-person['x'], data['robot']['y']-person['y']) - .35 - person['radius_m']
            minimum = min(minimum, clearance)
            if clearance < 0:
                overlaps.append(dict(stamp=row['stamp'], sim_time=data['sim_time'],
                                     person_id=person['id'], clearance_m=clearance))
    # These streams use the same native acquisition timestamp. Avoid comparing
    # delayed logger receipts or assuming world/body velocity components match.
    truth = sorted(topics['evaluation_odom'], key=lambda row: row['data']['source_stamp'])
    stamps = [row['data']['source_stamp'] for row in truth]
    errors = []
    for row in topics['odom']:
        stamp = row['data']['source_stamp']
        index = bisect_left(stamps, stamp)
        candidates = truth[max(0,index-1):index+1]
        if not candidates:
            continue
        reference = min(candidates, key=lambda r: abs(r['data']['source_stamp']-stamp))
        if abs(reference['data']['source_stamp']-stamp) <= .001:
            errors.append(math.hypot(row['data']['x']-reference['data']['x'],row['data']['y']-reference['data']['y']))
    # Segment actual planner mode/authority, not just the mission's phase label.
    segments = []
    for row in planner:
        data = row['data']
        key = (data.get('navigation_mode'), data.get('mission_active'))
        if not segments or segments[-1]['key'] != key:
            segments.append(dict(key=key, rows=[]))
        segments[-1]['rows'].append(row)
    windows = []
    for segment in segments:
        mode, active = segment['key']
        if mode not in ('scan', 'brake'):
            continue
        group = segment['rows']
        start, end = group[0]['stamp'], group[-1]['stamp']
        commands = [r['data'] for r in policy if start <= r['stamp'] <= end]
        a, b = nearest(aligned,start)['data']['robot'], nearest(aligned,end)['data']['robot']
        windows.append(dict(mode=mode, active=active, start_ros_s=start, end_ros_s=end,
            measured_world_displacement_m=math.hypot(b['x']-a['x'],b['y']-a['y']),
            max_policy_planar_command_mps=max(map(planar,commands),default=0.),
            selected_reasons=dict(Counter(r['data'].get('selected_reason') for r in group))))
    onset = None
    if first:
        p = nearest(planner,first['stamp'])
        a = nearest(aligned,first['stamp'])
        onset = dict(policy=first, planner={key:p['data'].get(key) for key in
            ('navigation_mode','reference_cmd','selected_cmd','selected','selected_reason',
             'closest_track','tracks_age_at_command_publish_s','candidate_results')},
            evaluation=a)
    metrics = ('planner_wall_active_ms','tracks_age_at_command_publish_s',
               'policy_observation_interval_ms','policy_submit_to_publish_ms',
               'odom_source_age_at_logger_receipt_s','rtf_monotonic_receipt_windows')
    return dict(run_id=directory.name, people=manifest['people'], seed=manifest['seed'],
        pedestrian_start_delay_s=manifest['pedestrian_start_delay_s'],
        reported_result={key:result.get(key) for key in
            ('success','mission_complete','error','fallen','data_valid','contact_samples','minimum_person_clearance_m')},
        events=result['events'], first_translation=onset, mode_windows=windows,
        full_trace_clearance=dict(minimum_m=minimum, overlapping_person_samples=len(overlaps),
            first=overlaps[0] if overlaps else None, last=overlaps[-1] if overlaps else None,
            scope='geometric envelopes, including logger tail after mission evaluation ends; not force contacts'),
        odom_truth_matched_xy=dict(samples=len(errors),max_error_m=max(errors,default=None),
                                  mean_error_m=sum(errors)/len(errors) if errors else None),
        timing={key:timing['metrics'].get(key) for key in metrics},
        planner_active_period=timing['planner_active_period'],
        planner_supervisor_lineage=timing['planner_supervisor_lineage'],
        policy_hold_events=timing['policy_hold_events'],
        source_sha256={name:digest(directory/name) for name in
            ('manifest.json','result.json','trace.jsonl','effective_parameters.json','source_snapshot.zip')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve prior assessments')
    runs = [assess(path) for path in args.runs]
    report = dict(scope='offline survey failure assessment, not acceptance or a causal replay',
        script_sha256=digest(Path(__file__)), runs=runs,
        limitations=['Commands are upstream of motor application; these runs did not record full actuator traces.',
                     'Geometric clearance overlap is not a measured force or a unique collision count.',
                     'Logger tails extend past mission result evaluation; retain both scopes.',
                     'Temporal association with pedestrians does not by itself prove a physical push.'])
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for run in runs:
        print(json.dumps({key:run[key] for key in ('run_id','reported_result','full_trace_clearance','odom_truth_matched_xy','mode_windows')}))


if __name__ == '__main__':
    main()
