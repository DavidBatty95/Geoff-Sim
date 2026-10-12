#!/usr/bin/env python3
"""Offline incident chronology and phase accounting from retained demo traces."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from evaluate_onboard_perception import evaluate


def review(run):
    result = json.loads((run/'result.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text()) if (run/'manifest.json').exists() else {}
    first_stamp = None
    first_contact = first_fall = None
    contact_snapshot = None
    planner = {}
    planner_stamp = None
    timing_before,timing_after = [],[]
    contact_before_fall = contact_after_fall = 0
    mission_end = 0.
    max_tracks = 0
    tracker_rejections = Counter()
    active_track_counts = []
    with (run/'trace.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            stamp = row['receipt_monotonic_s']
            if first_stamp is None:
                first_stamp = stamp
            elapsed = stamp-first_stamp
            data = row['data']
            topic = row['topic']
            if not isinstance(data,dict):
                continue
            if topic=='transport_state':
                mission_end = max(mission_end,float(data.get('elapsed',0.)))
            elif topic=='planner_debug':
                planner,planner_stamp = data,elapsed
                max_tracks = max(max_tracks,int(data.get('track_count',0)))
                if first_fall is None:
                    active_track_counts.append(int(data.get('track_count',0)))
                value = data.get('planning_ms')
                if isinstance(value,(float,int)) and math.isfinite(value):
                    (timing_before if first_fall is None else timing_after).append(value)
            elif topic=='tracker_debug':
                tracker_rejections.update(event.get('reason','unknown') for event in data.get('events',[])
                                          if event.get('state')=='rejected')
            elif topic=='evaluation_odom':
                x,y,z,w = (data[key] for key in ('qx','qy','qz','qw'))
                roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                pitch = math.asin(max(-1.,min(1.,2*(w*y-z*x))))
                if first_fall is None and (data['z']<.5 or max(abs(roll),abs(pitch))>.6):
                    first_fall = elapsed
            elif topic=='evaluation_aligned':
                robot = data['robot']
                overlaps = [person for person in data['people'] if
                    math.hypot(robot['x']-person['x'],robot['y']-person['y'])-.35-person['radius_m']<0.]
                if overlaps:
                    if first_contact is None:
                        first_contact = elapsed
                        contact_snapshot = dict(planner_state=planner.get('state'),
                            planner_snapshot_age_s=None if planner_stamp is None else elapsed-planner_stamp,
                            closest_observed_track=planner.get('closest_track'),
                            selected_cmd=planner.get('selected_cmd'),policy_command=planner.get('policy_command'),
                            evaluated_candidates=planner.get('evaluated_candidates'),
                            rejection_counts=dict(Counter(item.get('reason') or 'feasible' for item in planner.get('candidate_results',[]))),
                            tracks_age_s=planner.get('tracks_age_s'),prediction_age_s=planner.get('prediction_age_s'))
                    if first_fall is None:
                        contact_before_fall += len(overlaps)
                    else:
                        contact_after_fall += len(overlaps)
    phases = Counter()
    events = result.get('events',[])
    for index,event in enumerate(events):
        end = events[index+1]['elapsed_s'] if index+1<len(events) else mission_end
        phases[event['phase']] += max(0.,end-event['elapsed_s'])
    def timing(values):
        return dict(samples=len(values),median_ms=float(np.median(values)) if values else None,
                    p95_ms=float(np.quantile(values,.95)) if values else None)
    return dict(run=run.name,world=manifest.get('world'),people_requested=manifest.get('people'),
        tracker_wall_evidence=manifest.get('tracker_wall_evidence'),
        early_overtaking_avoidance=manifest.get('early_overtaking_avoidance'),
        success=result['success'],data_valid=result.get('data_valid'),fallen=result['fallen'],error=result.get('error'),
        chronology_clock='logger receipt monotonic, relative to first trace row; topic delivery can introduce skew',
        first_geometric_person_overlap_s=first_contact,first_fall_threshold_s=first_fall,
        overlap_to_fall_s=None if first_contact is None or first_fall is None else first_fall-first_contact,
        overlap_samples_before_first_fall=contact_before_fall,overlap_samples_after_first_fall=contact_after_fall,
        overlap_scope='aligned robot .35 m disk against person radii; excludes carried-box contacts',
        first_overlap_planner_snapshot=contact_snapshot,max_observed_tracks=max_tracks,
        tracker_rejection_counts=dict(tracker_rejections),
        mean_tracks_before_first_fall=float(np.mean(active_track_counts)) if active_track_counts else None,
        fraction_samples_with_tracks_before_first_fall=float(np.mean(np.asarray(active_track_counts)>0)) if active_track_counts else None,
        planner_before_first_fall=timing(timing_before),planner_after_first_fall=timing(timing_after),
        phase_time_s=dict(sorted(phases.items(),key=lambda item:-item[1])),
        phase_time_scope='mission events through last logged heartbeat; includes waiting, travel and computation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--perception',action='store_true',help='Include existing offline frame-aligned perception diagnostics')
    args = parser.parse_args()
    records = []
    for run in args.runs:
        record = review(run)
        if args.perception:
            record['onboard_perception'] = evaluate(run/'trace.jsonl')
        records.append(record)
    report = json.dumps(records,indent=2)+'\n'
    if args.output:
        args.output.write_text(report)
    print(report,end='')


if __name__=='__main__':
    main()
