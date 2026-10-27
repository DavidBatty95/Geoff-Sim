#!/usr/bin/env python3
"""Retained sensed-state comparison of beam search and reusable escape templates."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.escape_primitives import templates,check_programs
from unitree_g1_mujoco_bridge.kinodynamic_search import search_sequence
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.strategic_avoidance import crowd_forecast,refuge_candidates


def replay(run,primitive_budget=12.,beam_budget=20.):
    rows=[json.loads(line) for line in (run/'trace.jsonl').read_text().splitlines()]
    onset=next(row['stamp'] for row in rows if row['topic']=='policy_cmd' and
               math.hypot(row['data']['x'],row['data']['y'])>.05)
    model=MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6,accel_x=2.4,accel_y=2.4,accel_yaw=.9,
        forward_gain=.96,lateral_gain=.8,yaw_gain=.32,walking_yaw_gain=.55,lateral_yaw_gain=.1,
        negative_lateral_gain=.9,positive_lateral_time_constant=.3,linear_time_constant=.4,
        max_planar_speed=.45,response_profile='native_contact_cycle05')
    programs=templates(model,4.5,.65,.45,.45)
    controls=[(vx,vy,0.) for vx in (-.45,0.,.325,.65) for vy in (-.45,0.,.45)]
    results=[]
    for offset in (-1.,-.5,0.,.5,1.):
        stamp=onset+offset
        def latest(topic):
            return [row['data'] for row in rows if row['topic']==topic and row['stamp']<=stamp][-1]
        tracking,odom,feedback=latest('tracker_debug'),latest('odom'),latest('policy_cmd')
        age=stamp-tracking['stamp']
        tracks=[SimpleNamespace(**{**p,'x':p['x']+p['vx']*age,'y':p['y']+p['vy']*age}) for p in tracking['tracks']]
        pose=(odom['x'],odom['y'],2*math.atan2(odom['qz'],odom['qw']))
        velocity=(odom['vx'],odom['vy'],odom['wz'])
        filtered=(feedback['x'],feedback['y'],feedback['z'])
        forecast,r=crowd_forecast(tracks,np.linspace(0.,7.,36),.48,.35)
        goal=refuge_candidates(pose,forecast,r,np.linspace(0.,7.,36))[0]['goal']
        people,r=crowd_forecast(tracks,np.linspace(0.,4.5,31),.48,.35)
        radii=np.tile(r,(31,1))
        for method in ('beam','primitives'):
            details={}
            if method=='beam':
                plan=search_sequence(model,pose,velocity,filtered,controls,people,radii,goal,
                    4.5,.15,.1,goal_weight=2.,beam_width=16,budget_ms=beam_budget,enclosure_weight=8.,diagnostics=details)
            else:
                plan=check_programs(model,pose,velocity,filtered,programs,people,radii,goal,
                    4.5,.15,.1,budget_ms=primitive_budget,diagnostics=details)
            results.append(dict(offset_s=offset,method=method,complete=plan is not None,
                                diagnostics=details,schedule=plan))
    return dict(run_id=run.name,trace_sha256=hashlib.sha256((run/'trace.jsonl').read_bytes()).hexdigest(),results=results)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--primitive-budget-ms',type=float,default=12.)
    parser.add_argument('--beam-budget-ms',type=float,default=20.)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='sensed constant-velocity forecast isolation; no static-map or physical qualification',
        primitive_budget_ms=args.primitive_budget_ms,beam_budget_ms=args.beam_budget_ms,
        runs=[replay(path,args.primitive_budget_ms,args.beam_budget_ms) for path in args.runs],source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__),ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/escape_primitives.py',
                     ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/kinodynamic_search.py']})
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for run in report['runs']:
        print(json.dumps(dict(run_id=run['run_id'],results=[{k:v for k,v in row.items() if k!='schedule'} for row in run['results']])))


if __name__=='__main__':
    main()
