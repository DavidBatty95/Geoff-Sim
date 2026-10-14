#!/usr/bin/env python3
"""Compact time-aligned command/state snapshots from retained traces (offline)."""
import argparse
from collections import Counter
import json
from pathlib import Path


def summarize(path,start=0.,end=30.,period=1.):
    latest,states = {},Counter()
    origin = next_sample = None
    samples = []
    for line in path.open():
        row = json.loads(line)
        stamp = row.get('receipt_monotonic_s',row['stamp'])
        if origin is None:
            origin = stamp
            next_sample = start
        elapsed = stamp-origin
        topic,data = row['topic'],row['data']
        if elapsed>end:
            break
        if topic in ('policy_cmd','evaluation_odom','odom','policy_diagnostics','planner_debug','tracker_debug'):
            latest[topic] = data
        if elapsed<start:
            continue
        if topic=='safety_debug':
            states[data['mode']] += 1
        if elapsed>=next_sample:
            planner = latest.get('planner_debug',{})
            samples.append(dict(time=round(elapsed,2),command=latest.get('policy_cmd'),
                odom=latest.get('odom'),evaluation=latest.get('evaluation_odom'),
                planner={key:planner.get(key) for key in ('state','track_count','selected','navigation_mode','mission_active',
                                                         'nearest_static','static_error','reference_cmd','selected_reason')},
                tracks=latest.get('tracker_debug',{}).get('tracks',[]),
                policy=latest.get('policy_diagnostics',{})))
            next_sample += period
    return dict(source=str(path),scope='offline receipt-aligned diagnostic, not causal actuator timing',
                supervisor_modes=dict(states),samples=samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--start',type=float,default=0.)
    parser.add_argument('--end',type=float,default=30.)
    parser.add_argument('--period',type=float,default=2.)
    args = parser.parse_args()
    if args.period<=0 or args.end<=args.start:
        parser.error('positive period and end after start required')
    print(json.dumps(summarize(args.trace,args.start,args.end,args.period),indent=2))


if __name__=='__main__':
    main()
