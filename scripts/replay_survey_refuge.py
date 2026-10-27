#!/usr/bin/env python3
"""Replay sensed crowd snapshots to compare endpoint-only and safe-hold searches.

This isolates forecast feasibility; full static geometry and physical execution
are validated by live survey runs, not by this offline replay.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.kinodynamic_search import search_sequence, rollout_schedule
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def replay(path,horizon=3.,beam_width=28):
    rows=[json.loads(line) for line in (path/'trace.jsonl').read_text().splitlines()]
    onset=next(row['stamp'] for row in rows if row['topic']=='policy_cmd' and
               math.hypot(row['data']['x'],row['data']['y'])>.05)
    envelope=MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6,accel_x=2.4,accel_y=2.4,accel_yaw=.9,
        forward_gain=.96,lateral_gain=.8,yaw_gain=.32,walking_yaw_gain=.55,lateral_yaw_gain=.1,
        negative_lateral_gain=.9,positive_lateral_time_constant=.3,linear_time_constant=.4,
        max_planar_speed=.45,response_profile='native_contact_cycle05')
    outcomes=[]
    for offset in (-1.,-.5,0.,.5,1.):
        stamp=onset+offset
        def latest(topic):
            return [row['data'] for row in rows if row['topic']==topic and row['stamp']<=stamp][-1]
        odom,feedback,tracking=latest('odom'),latest('policy_cmd'),latest('tracker_debug')
        tracks=tracking['tracks']
        age=stamp-tracking['stamp']
        pose=(odom['x'],odom['y'],2*math.atan2(odom['qz'],odom['qw']))
        times=np.linspace(0.,horizon,math.ceil(horizon/.15)+1)
        people=np.array([[[p['x']+p['vx']*(age+t),p['y']+p['vy']*(age+t)] for p in tracks] for t in times])
        radii=np.array([[.48+p['body_radius']+.35 for p in tracks] for _ in times])
        closest=min(tracks,key=lambda p:math.hypot(p['x']-pose[0],p['y']-pose[1]))
        speed=max(.01,math.hypot(closest['vx'],closest['vy']))
        normal=(-closest['vy']/speed,closest['vx']/speed)
        side=1 if (pose[0]-closest['x'])*normal[0]+(pose[1]-closest['y'])*normal[1]>=0 else -1
        goal=(pose[0]+1.5*side*normal[0],pose[1]+1.5*side*normal[1])
        velocity=(odom['vx'],odom['vy'],odom['wz'])
        filtered=(feedback['x'],feedback['y'],feedback['z'])
        actions=[(vx,vy,0.) for vx in (-.45,0.,.325,.65) for vy in (-.45,0.,.45)]
        for hold in (0.,2.):
            started=time.perf_counter()
            schedule=search_sequence(envelope,pose,velocity,filtered,actions,people,radii,goal,
                                     horizon,.15,.1,goal_weight=2.,budget_ms=0.,terminal_hold_s=hold,beam_width=beam_width)
            elapsed=(time.perf_counter()-started)*1000
            clearance=None
            if schedule:
                trajectory=np.array(rollout_schedule(envelope,pose,velocity,filtered,schedule,horizon+2.,.15,.1))
                future=np.array([[[p['x']+p['vx']*(age+t),p['y']+p['vy']*(age+t)] for p in tracks] for t in trajectory[:,3]])
                relative=trajectory[:,None,:2]-future
                delta=np.diff(relative,axis=0)
                fraction=np.clip(-np.sum(relative[:-1]*delta,axis=2)/np.maximum(1e-12,np.sum(delta*delta,axis=2)),0.,1.)
                clearance=float(np.min(np.linalg.norm(relative[:-1]+fraction[:,:,None]*delta,axis=2)-radii[0]))
            outcomes.append(dict(offset_s=offset,terminal_hold_s=hold,complete_plan=schedule is not None,
                search_ms=elapsed,hold_checked_swept_margin_m=clearance,schedule=schedule))
    return dict(run_id=path.name,trace_sha256=hashlib.sha256((path/'trace.jsonl').read_bytes()).hexdigest(),outcomes=outcomes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--horizon',type=float,default=3.)
    parser.add_argument('--beam-width',type=int,default=28)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='sensed constant-velocity crowd forecast isolation, not static-map or physical qualification',
        comparison='same search with zero versus two-second terminal hold; not a replay of the historical executable',
        horizon_s=args.horizon,beam_width=args.beam_width,
        runs=[replay(path,args.horizon,args.beam_width) for path in args.runs])
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for run in report['runs']:
        print(json.dumps(dict(run_id=run['run_id'],outcomes=[{k:v for k,v in row.items() if k!='schedule'} for row in run['outcomes']])))


if __name__=='__main__':
    main()
