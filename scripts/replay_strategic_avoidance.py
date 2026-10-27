#!/usr/bin/env python3
"""Compare closest-person and all-person refuge hints on retained sensed tracks."""
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
from unitree_g1_mujoco_bridge.strategic_avoidance import crowd_forecast,predict_pincer,refuge_candidates


def replay(run):
    rows=[json.loads(line) for line in (run/'trace.jsonl').read_text().splitlines()]
    onset=next(row['stamp'] for row in rows if row['topic']=='policy_cmd' and
               math.hypot(row['data']['x'],row['data']['y'])>.05)
    cases=[]
    for offset in (-2.,-1.,-.5,0.,.5,1.):
        stamp=onset+offset
        def latest(topic):
            return [row['data'] for row in rows if row['topic']==topic and row['stamp']<=stamp][-1]
        tracking,odom=latest('tracker_debug'),latest('odom')
        age=stamp-tracking['stamp']
        tracks=[SimpleNamespace(**{**p,'x':p['x']+p['vx']*age,'y':p['y']+p['vy']*age}) for p in tracking['tracks']]
        pose=(odom['x'],odom['y'],2*math.atan2(odom['qz'],odom['qw']))
        times=np.linspace(0.,7.,36)
        people,radii=crowd_forecast(tracks,times,.48,.35)
        risk=predict_pincer(np.tile(pose[:2],(len(times),1)),people,radii,times)
        ranked=refuge_candidates(pose,people,radii,times)
        moving=[p for p in tracks if math.hypot(p.vx,p.vy)>.25]
        baseline=None
        if moving:
            p=min(moving,key=lambda p:math.hypot(p.x-pose[0],p.y-pose[1]))
            speed=math.hypot(p.vx,p.vy)
            normal=np.array((-p.vy,p.vx))/speed
            sign=1 if (np.asarray(pose[:2])-(p.x,p.y))@normal>=0 else -1
            goal=np.asarray(pose[:2])+1.5*sign*normal
            candidates=refuge_candidates(pose,people,radii,times,previous=goal)
            baseline=next((row for row in candidates if np.linalg.norm(np.asarray(row['goal'])-goal)<1e-6),None)
        cases.append(dict(offset_from_first_translation_s=offset,track_age_s=age,pincer=risk,
                          closest_person_refuge=baseline,all_person_refuge=ranked[0] if ranked else None))
    return dict(run_id=run.name,trace_sha256=hashlib.sha256((run/'trace.jsonl').read_bytes()).hexdigest(),cases=cases)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='stationary-reference sensed constant-velocity forecast comparison, not execution qualification',
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        strategy_sha256=hashlib.sha256((ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/strategic_avoidance.py').read_bytes()).hexdigest(),
        limitations=['Static geometry is not replayed; live search rechecks it.',
                     'A coarse refuge is a direction hint, not an executable or certified-safe route.',
                     'Earlier detection in these samples does not prove that physical avoidance would succeed.'],
        runs=[replay(run) for run in args.runs])
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for run in report['runs']:
        print(json.dumps(run))


if __name__=='__main__':
    main()
