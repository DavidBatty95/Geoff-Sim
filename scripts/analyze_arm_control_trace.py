#!/usr/bin/env python3
"""Summarize an isolated ROS arm trace; never feed evaluator state to control."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def analyze(path):
    counts = Counter()
    phases = defaultdict(lambda:defaultdict(list))
    phase = 'startup'
    malformed = 0
    with path.open() as stream:
        for line in stream:
            if not line.endswith('\n'):
                continue  # The live logger may still be appending its final row.
            try:
                row = json.loads(line)
                topic,data,stamp = row['topic'],row['data'],row['receipt_monotonic_s']
            except (ValueError,KeyError,TypeError):
                malformed += 1
                continue
            counts[topic] += 1
            if topic=='transport_state':
                phase = data['phase']
            if topic in ('lowcmd_control','lowstate_control','actor_targets','composed_targets','evaluation_odom','odom'):
                phases[phase][topic].append((stamp,data))
    summary = {}
    for name,topics in phases.items():
        if not name.startswith('probe_'):
            continue
        item = {}
        for topic in ('evaluation_odom','odom'):
            rows = topics[topic]
            if rows:
                xy = np.array([[d['x'],d['y']] for _,d in rows])
                yaw = np.unwrap([math.atan2(2*(d['qw']*d['qz']+d['qx']*d['qy']),1-2*(d['qy']**2+d['qz']**2)) for _,d in rows])
                item[topic] = dict(samples=len(rows),duration_s=rows[-1][0]-rows[0][0],
                    max_receipt_gap_s=float(np.max(np.diff([t for t,_ in rows]))) if len(rows)>1 else None,
                    translation_m=float(np.max(np.linalg.norm(xy-xy[0],axis=1))),
                    yaw_excursion_rad=float(np.max(np.abs(yaw-yaw[0]))))
        for topic in ('lowstate_control','lowcmd_control','actor_targets','composed_targets'):
            rows = topics[topic]
            if rows:
                times = np.array([t for t,_ in rows])
                item[topic] = dict(samples=len(rows),duration_s=float(times[-1]-times[0]),
                    max_receipt_gap_s=float(np.max(np.diff(times))) if len(times)>1 else None)
                if topic=='lowstate_control':
                    ticks = np.array([d['device_tick_ms'] for _,d in rows])
                    item[topic].update(device_elapsed_s=float((ticks[-1]-ticks[0])*.001),
                        reordered_ticks=int(np.sum(np.diff(ticks)<0)),
                        median_q=np.median([d['q'] for _,d in rows],axis=0).tolist())
                elif topic=='lowcmd_control':
                    profiles = {(tuple(m['kp'] for m in d['motors']),tuple(m['kd'] for m in d['motors'])) for _,d in rows}
                    item[topic].update(gain_profiles=[dict(kp=kp,kd=kd) for kp,kd in sorted(profiles)],
                        median_q=np.median([[m['q'] for m in d['motors']] for _,d in rows],axis=0).tolist())
        summary[name] = item
    required = ('lowstate_control','lowcmd_control','actor_targets','composed_targets','evaluation_odom')
    hold = summary.get('probe_extension_hold',{})
    complete = all(hold.get(topic,{}).get('duration_s',0.)>=9. and
                   hold.get(topic,{}).get('samples',0)>=10 and
                   (hold.get(topic,{}).get('max_receipt_gap_s') or 0.)<=.5 for topic in required)
    complete = complete and hold.get('lowstate_control',{}).get('device_elapsed_s',0.)>0. and hold.get('lowstate_control',{}).get('reordered_ticks',1)==0
    return dict(scope='offline observed-publication and measured-state diagnostic; not actuator acknowledgement or delivery qualification',
        capture_valid=bool(complete and not malformed),malformed_rows=malformed,topic_counts=dict(counts),phases=summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    path = args.run/'trace.jsonl'
    result = analyze(path)
    digest = hashlib.sha256()
    hashed_bytes = 0
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
            hashed_bytes += len(block)
    result.update(trace_sha256=digest.hexdigest(),trace_hashed_bytes=hashed_bytes,trace=str(path),hash_scope='trace prefix observed during analysis; live logger may append later')
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(capture_valid=result['capture_valid'],phases=list(result['phases']),topic_counts=result['topic_counts'])))
    return int(not result['capture_valid'])


if __name__=='__main__':
    raise SystemExit(main())
