#!/usr/bin/env python3
"""Reproduce captured invalidations and separate robot-state/forecast changes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.escape_primitives import check_programs
from unitree_g1_mujoco_bridge.kinodynamic_search import response_step
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def advance_robot(audit,elapsed):
    model=MotionEnvelope(**audit['envelope'])
    state=np.array([[*audit['pose'],*audit['velocity'],*audit['filtered']]])
    program=audit['program']
    boundaries=audit['delay']+np.cumsum([p[0] for p in program])
    steps=max(1,math.ceil(elapsed/audit['dt']))
    for i in range(steps):
        start,end=i*elapsed/steps,(i+1)*elapsed/steps
        cuts=[start]+sorted({float(t) for t in (audit['delay'],*boundaries) if start<t<end})+[end]
        for a,b in zip(cuts,cuts[1:]):
            index=int(np.searchsorted(boundaries,a,side='right'))
            cmd=model.normalize(program[index][1:]) if index<len(program) else (0.,0.,0.)
            state=response_step(state,np.array([cmd]),model,b-a,b-a if a>=audit['delay'] else 0.)
    return state[0]


def shift_forecast(values,old_horizon,queries):
    values=np.asarray(values)
    times=np.linspace(0.,old_horizon,len(values))
    flat=values.reshape(len(values),-1)
    out=np.column_stack([np.interp(queries,times,flat[:,i]) for i in range(flat.shape[1])]).reshape(len(queries),*values.shape[1:])
    velocity=(values[-1]-values[-2])/(times[-1]-times[-2])
    return out+np.maximum(0.,queries-old_horizon).reshape((-1,)+(1,)*(values.ndim-1))*velocity


def check(current,robot,people,radii):
    details={}
    plan=check_programs(MotionEnvelope(**current['envelope']),robot[:3],robot[3:6],robot[6:9],
        [tuple(tuple(part) for part in current['program'])],people,radii,current['goal'],current['horizon'],current['dt'],current['delay'],
        budget_ms=0.,diagnostics=details)
    return dict(passed=plan is not None,details=details)


def analyze_pair(pair):
    old,new=pair['previous'],pair['current']
    elapsed=new['forecast_epoch']-old['forecast_epoch']
    if elapsed<0.:
        return dict(error='forecast clock regressed',inputs=pair)
    predicted=advance_robot(old,elapsed)
    measured=np.array([*new['pose'],*new['velocity'],*new['filtered']])
    old_ids=[p['track_id'] for p in old['tracks']]
    new_ids=[p['track_id'] for p in new['tracks']]
    people,radii=np.array(new['people']),np.array(new['radii'])
    actual=check(new,measured,people,radii)
    report=dict(elapsed_s=elapsed,recorded_rejection=pair['rejection'],reproduced=actual,
        robot_xy_error_m=float(np.linalg.norm(measured[:2]-predicted[:2])),
        robot_yaw_error_rad=math.atan2(math.sin(measured[2]-predicted[2]),math.cos(measured[2]-predicted[2])),
        robot_velocity_error=(measured[3:6]-predicted[3:6]).tolist(),
        robot_pose_age_s=[old['forecast_epoch']-old['odom_stamp'],new['forecast_epoch']-new['odom_stamp']],
        new_track_ids=sorted(set(new_ids)-set(old_ids)),lost_track_ids=sorted(set(old_ids)-set(new_ids)),
        stream_epoch_skew_s=[None if a['prediction_epoch'] is None else a['tracks_epoch']-a['prediction_epoch'] for a in (old,new)],
        rejecting_tracks=[],inputs=pair)
    for i,track in enumerate(new['tracks']):
        if not check(new,measured,people[:,i:i+1],radii[:,i:i+1])['passed']:
            report['rejecting_tracks'].append(track['track_id'])
    if set(old_ids)!=set(new_ids):
        report['factorial_unavailable']='identity set changed; do not compare forecasts by array index'
        return report
    queries=np.linspace(0.,new['horizon'],len(people))+elapsed
    order=[old_ids.index(i) for i in new_ids]
    old_people=shift_forecast(old['people'],old['horizon'],queries)[:,order]
    old_radii=shift_forecast(old['radii'],old['horizon'],queries)[:,order]
    report['counterfactuals']=dict(
        predicted_robot_old_forecast=check(new,predicted,old_people,old_radii),
        measured_robot_old_forecast=check(new,measured,old_people,old_radii),
        predicted_robot_new_forecast=check(new,predicted,people,radii),
        measured_robot_new_forecast=actual)
    report['forecast_changes']=[]
    for i,track in enumerate(new['tracks']):
        previous=old['tracks'][old_ids.index(track['track_id'])]
        report['forecast_changes'].append(dict(track_id=track['track_id'],
            velocity_delta=[track['vx']-previous['vx'],track['vy']-previous['vy']],
            maximum_forecast_shift_m=float(np.max(np.linalg.norm(people[:,i]-old_people[:,i],axis=1))),
            maximum_radius_change_m=float(np.max(np.abs(radii[:,i]-old_radii[:,i]))),
            initial_forecast_shift_m=float(np.linalg.norm(people[0,i]-old_people[0,i]))))
    return report


def truth_comparison(pair,aligned,odometry,evaluation):
    """Offline only: align world truth to the measured odom frame at one epoch."""
    old,new=pair['previous'],pair['current']
    stamp=new['forecast_epoch']
    def nearest(rows,t,key):
        return min(rows,key=lambda row:abs(row[key]-t)) if rows else None
    odom=nearest(odometry,new['odom_stamp'],'source_stamp')
    truth=nearest(evaluation,new['odom_stamp'],'source_stamp')
    if odom is None or truth is None or abs(odom['source_stamp']-truth['source_stamp'])>.001:
        return dict(valid=False,reason='timestamp-matched frame alignment unavailable')
    def yaw(d):
        return math.atan2(2*(d['qw']*d['qz']+d['qx']*d['qy']),1-2*(d['qy']**2+d['qz']**2))
    angle=yaw(truth)-yaw(odom)
    rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    translation=np.array([truth['x'],truth['y']])-rotation@np.array([odom['x'],odom['y']])
    def people_at(t):
        before=[d for d in aligned if d['stamp']<=t]
        after=[d for d in aligned if d['stamp']>=t]
        if not before or not after:
            return {}
        a,b=before[-1],after[0]
        if b['stamp']-a['stamp']>.25:
            return {}
        fraction=(t-a['stamp'])/max(1e-12,b['stamp']-a['stamp'])
        right={p['id']:p for p in b['people']}
        return {p['id']:np.array([p['x'],p['y']])+fraction*(np.array([right[p['id']]['x'],right[p['id']]['y']])-np.array([p['x'],p['y']]))
                for p in a['people'] if p['id'] in right}
    current=people_at(stamp)
    edges=sorted((float(np.linalg.norm(rotation@np.array([t['x'],t['y']])+translation-position)),i,identity)
                 for i,t in enumerate(new['tracks']) for identity,position in current.items())
    matches={}; used=set()
    for distance,i,identity in edges:
        if distance<=.75 and i not in matches and identity not in used:
            matches[i]=(identity,distance);used.add(identity)
    old_ids=[t['track_id'] for t in old['tracks']]
    report=[]
    for i,(identity,distance) in matches.items():
        track=new['tracks'][i]
        previous=old_ids.index(track['track_id']) if track['track_id'] in old_ids else None
        errors=[]
        for offset in (0.,.5,1.,2.,3.):
            actual=people_at(stamp+offset).get(identity)
            if actual is None:
                continue
            new_position=shift_forecast(new['people'],new['horizon'],np.array([offset]))[0,i]
            old_position=None if previous is None else shift_forecast(old['people'],old['horizon'],np.array([stamp-old['forecast_epoch']+offset]))[0,previous]
            errors.append(dict(offset_s=offset,new_error_m=float(np.linalg.norm(rotation@new_position+translation-actual)),
                old_error_m=None if old_position is None else float(np.linalg.norm(rotation@old_position+translation-actual))))
        a,b=people_at(stamp-.1).get(identity),people_at(stamp+.1).get(identity)
        velocity=None if a is None or b is None else (rotation.T@(b-a)/.2).tolist()
        report.append(dict(track_id=track['track_id'],truth_id=identity,association_distance_m=distance,
                           observed_velocity=[track['vx'],track['vy']],truth_wall_velocity=velocity,errors=errors))
    return dict(valid=True,scope='offline world-truth comparison with timestamp-matched planar frame correction',tracks=report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    pairs=[]
    aligned=[];odometry=[];evaluation=[]
    with (args.run/'trace.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']=='evaluation_aligned':
                aligned.append(row['data'])
            elif row['topic']=='odom':
                odometry.append(row['data'])
            elif row['topic']=='evaluation_odom':
                evaluation.append(row['data'])
            if row['topic']=='planner_debug':
                pair=row['data'].get('kinodynamic_details',{}).get('invalidation_audit')
                if pair:
                    pairs.append(analyze_pair(pair))
    for pair in pairs:
        pair['truth_comparison']=truth_comparison(pair['inputs'],aligned,odometry,evaluation)
    report=dict(scope='exact captured occupancy-forecast/state factorial diagnostic; static geometry and motor application not replayed',
        run_id=args.run.name,trace_sha256=hashlib.sha256((args.run/'trace.jsonl').read_bytes()).hexdigest(),pairs=pairs,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for pair in pairs:
        print(json.dumps({k:v for k,v in pair.items() if k!='inputs'}))
    if not pairs:
        print('No captured invalidation pairs; do not infer a cause from this run.')
    return int(not pairs)


if __name__=='__main__':
    raise SystemExit(main())
