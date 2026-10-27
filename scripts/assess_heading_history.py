#!/usr/bin/env python3
"""Compare logged heading increments with offline acquisition-stamped truth."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from analyze_invalidation_heading import heading_at, wrap
from publish_transport_stage_validation import digest


def summarize(run):
    odometry, truth, states = {}, [], []
    with (run/'trace.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']=='odom':
                odometry[row['data']['source_stamp']]=row['data']
            elif row['topic']=='evaluation_odom':
                truth.append(row['data'])
            elif row['topic']=='planner_debug':
                states.append(row['data'].get('heading_history',{}))
    truth.sort(key=lambda row:row['source_stamp'])
    counts=Counter(s.get('reason','unrecorded') for s in states)
    errors=[]
    for state in states:
        if not state.get('valid'):
            continue
        source,query=state['source_stamp'],state['query_stamp']
        odom=odometry.get(source)
        before,after=heading_at(truth,source),heading_at(truth,query)
        if odom is None or before is None or after is None:
            continue
        actual=wrap(after-before)
        errors.append((abs(wrap(state['heading_delta_rad']-actual)),
                       abs(wrap(odom['wz']*(query-source)-actual))))
    def stats(values):
        return dict(mean_rad=float(np.mean(values)),p95_rad=float(np.quantile(values,.95)),max_rad=float(max(values)))
    return dict(run_id=run.name,trace_sha256=digest(run/'trace.jsonl'),heading_states=dict(counts),
        matched_samples=len(errors),
        history_error=None if not errors else stats([e[0] for e in errors]),
        old_projection_counterfactual_error=None if not errors else stats([e[1] for e in errors]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='offline same-sample heading increment comparison; full logger interval including teardown; not a closed-loop ablation',
                script_sha256=digest(Path(__file__)),runs=[summarize(run) for run in args.runs])
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
