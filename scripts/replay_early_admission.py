#!/usr/bin/env python3
"""Compare earlier triggers and delayed-admission reserve on sensed snapshots."""
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
from unitree_g1_mujoco_bridge.escape_primitives import templates,check_programs,snapshot,search_snapshot
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.strategic_avoidance import crowd_forecast,predict_strategic_risk,refuge_candidates


def replay(run):
    rows=[json.loads(line) for line in (run/'trace.jsonl').read_text().splitlines()]
    onset=next(r['stamp'] for r in rows if r['topic']=='policy_cmd' and math.hypot(r['data']['x'],r['data']['y'])>.05)
    model=MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6,accel_x=2.4,accel_y=2.4,accel_yaw=.9,
        forward_gain=.96,lateral_gain=.8,yaw_gain=.32,walking_yaw_gain=.55,lateral_yaw_gain=.1,
        negative_lateral_gain=.9,positive_lateral_time_constant=.3,linear_time_constant=.4,
        max_planar_speed=.45,response_profile='native_contact_cycle05')
    programs=templates(model,4.5,.65,.45,.45)
    def state(stamp):
        def latest(topic):
            return [r['data'] for r in rows if r['topic']==topic and r['stamp']<=stamp][-1]
        tracking,odom,cmd,planner=map(latest,('tracker_debug','odom','policy_cmd','planner_debug'))
        age=stamp-tracking['stamp']
        tracks=[SimpleNamespace(**{**p,'x':p['x']+p['vx']*age,'y':p['y']+p['vy']*age}) for p in tracking['tracks']]
        pose=(odom['x'],odom['y'],2*math.atan2(odom['qz'],odom['qw']))
        people,radii=crowd_forecast(tracks,np.linspace(0.,4.5,31),.48,.35)
        goal_key=planner.get('command_sequence',{}).get('goal')
        if goal_key:
            goal_key=[goal_key[0],*goal_key[3:]]  # Exclude envelope source timestamp.
        context=(planner.get('navigation_mode'),planner.get('mission_active'),goal_key)
        inputs=dict(envelope=model,pose=pose,velocity=(odom['vx'],odom['vy'],odom['wz']),
            filtered=(cmd['x'],cmd['y'],cmd['z']),programs=programs,people=people,
            radii=np.tile(radii,(31,1)),horizon=4.5,dt=.15,delay=.1,budget_ms=0.)
        return inputs,tracks,context
    cases=[]
    for offset in (-1.,-.5,-.25,0.,.25,.5):
        current,tracks,context=state(onset+offset)
        future,_,future_context=state(onset+offset+.15)
        times=np.linspace(0.,7.,36)
        people,radii=crowd_forecast(tracks,times,.48,.35)
        risk=predict_strategic_risk(np.tile(current['pose'][:2],(len(times),1)),people,radii,times)
        goal=refuge_candidates(current['pose'],people,radii,times)[0]['goal']
        outcomes=[]
        for reserved in (False,True):
            extra=dict(minimum_margin=.1,admission_delays=(0.,.1,.2)) if reserved else {}
            request=snapshot({**current,'goal':goal,**extra},())
            plan=search_snapshot(request)
            detail={}
            admitted=False
            if plan is not None and context==future_context:
                admitted=check_programs(**{**future,'programs':[plan],'goal':goal},diagnostics=detail) is not None
            outcomes.append(dict(reserved=reserved,complete=plan is not None,
                context_unchanged=context==future_context,admitted_after_150ms=admitted,
                search=request.diagnostics,revalidation=detail))
        cases.append(dict(offset_s=offset,trigger=risk,outcomes=outcomes))
    return dict(run_id=run.name,trace_sha256=hashlib.sha256((run/'trace.jsonl').read_bytes()).hexdigest(),cases=cases)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    report=dict(scope='offline sensed constant-velocity forecast isolation with unlimited compute; no static-map or timing qualification',
        runs=[replay(run) for run in args.runs],source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__),ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/escape_primitives.py',
                      ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/strategic_avoidance.py')})
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for run in report['runs']:
        print(json.dumps(dict(run_id=run['run_id'],cases=[dict(offset_s=c['offset_s'],reason=c['trigger']['reason'],
            pincer=c['trigger']['pincer_active'],outcomes=[{k:v for k,v in o.items() if k not in ('search','revalidation')} for o in c['outcomes']]) for c in run['cases']])))


if __name__=='__main__':
    main()
