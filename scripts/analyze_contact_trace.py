#!/usr/bin/env python3
"""Associate native contact-force intervals with received policy/LowState data."""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def analyze(run):
    contacts=[json.loads(line) for line in (run/'native_contacts.jsonl').read_text().splitlines()]
    states,commands=[],[]
    counts=Counter()
    planner=Counter()
    with (run/'trace.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            counts[row['topic']]+=1
            if row['topic']=='lowstate_control':
                states.append((row['data']['device_tick_ms'],row['receipt_monotonic_s']))
            elif row['topic']=='policy_cmd':
                commands.append((row['receipt_monotonic_s'],row['data']))
            elif row['topic']=='planner_debug':
                planner[(row['data'].get('kinodynamic_status'),row['data'].get('selected'))]+=1
    regressions=sum(b[0]<a[0] for a,b in zip(states,states[1:]))
    states.sort()
    commands.sort(key=lambda pair:pair[0])
    ticks=[tick for tick,_ in states]
    stamps=[stamp for stamp,_ in commands]
    windows=[]
    for item in contacts:
        if item['peak_normal_force_n']<=0.:
            continue
        record=dict(item)
        index=bisect_left(ticks,item['device_tick_ms'])
        nearby=states[max(0,index-1):index+1]
        if nearby:
            tick,receipt=min(nearby,key=lambda pair:abs(pair[0]-item['device_tick_ms']))
            ci=bisect_right(stamps,receipt)-1
            record['state_tick_error_ms']=abs(tick-item['device_tick_ms'])
            if ci>=0 and receipt-stamps[ci]<=.1 and record['state_tick_error_ms']<=25:
                record['policy_command']={key:commands[ci][1][key] for key in ('x','y','z')}
                record['command_receipt_age_s']=receipt-stamps[ci]
        windows.append(record)
    gaps=[b['sim_time']-a['sim_time'] for a,b in zip(contacts,contacts[1:])]
    valid=bool(len(contacts)>20 and gaps and min(gaps)>0 and max(gaps)<.1 and
               all(counts[key]>20 for key in ('lowstate_control','lowcmd_control','policy_cmd')) and
               not regressions and ticks and ticks[0]<=contacts[0]['device_tick_ms']+100 and
               ticks[-1]>=contacts[-1]['device_tick_ms']-100 and ticks[-1]-ticks[0]>1000 and
               all(math.isfinite(row[key]) and row[key]>=0 for row in contacts
                   for key in ('peak_normal_force_n','normal_impulse_ns')))
    return dict(scope='actual native robot/person normal contact forces; received command association is not motor acknowledgement',
        capture_valid=valid,sim_time_start=contacts[0]['sim_time'] if contacts else None,
        sim_time_end=contacts[-1]['sim_time'] if contacts else None,
        max_sample_gap_s=max(gaps,default=None),windows=len(contacts),force_windows=windows,
        peak_normal_force_n=max((row['peak_normal_force_n'] for row in contacts),default=0.),
        normal_impulse_ns=sum(row['normal_impulse_ns'] for row in contacts),
        topic_counts=dict(counts),planner_outcomes=[dict(status=key[0],selected=key[1],count=value) for key,value in planner.items()],
        source_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in ('native_contacts.jsonl','trace.jsonl')},
        limitations=['Forces and impulses aggregate every physics substep into approximately 50 ms windows.',
                     'Only the first 35 simulator seconds are recorded; later contact is outside capture scope.',
                     'Command association uses nearest device tick then logger receipt time; uncertainty is recorded.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    result=analyze(args.run)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({key:value for key,value in result.items() if key not in ('force_windows','source_sha256','limitations','topic_counts')}))
    print(json.dumps(dict(force_windows=len(result['force_windows']),
        force_with_zero_translation=sum(math.hypot(row.get('policy_command',{}).get('x',math.inf),row.get('policy_command',{}).get('y',math.inf))<1e-6 for row in result['force_windows']))))
    return int(not result['capture_valid'])


if __name__=='__main__':
    raise SystemExit(main())
