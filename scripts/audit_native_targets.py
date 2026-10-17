#!/usr/bin/env python3
"""Audit deployed soft-limit clipping against unbounded training action targets."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
from unitree_g1_mujoco_bridge.policy_safety import clamp_joint_targets
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS


def audit(data):
    if not data.get('record_actions') or data.get('outage_at_s') is not None:
        raise ValueError('recorded actions without outage overrides required')
    adapter = UnitreeRLLabG1PolicyAdapter()
    counts,maxima = [0]*15,[0.]*15
    affected = checked = 0
    max_reconstruction_error = 0.
    phases = {name:dict(checked_samples=0,clipped_samples=0,max_target_delta_rad=0.) for name in ('standing','moving','post_stop')}
    start,stop = data['actual_command_start_s'],data['actual_command_end_s']
    if (type(start) not in (int,float) or type(stop) not in (int,float) or
            not math.isfinite(start) or not math.isfinite(stop) or not 0.<=start<stop):
        raise ValueError('recorded start and stop transitions required')
    for row in data['samples']:
        if row['applied_sequence']<0:
            continue
        if len(row['composed_joint_target'])!=29:
            raise ValueError('incomplete composed joint target')
        raw = adapter.action_targets(row['applied_action'])
        clipped = clamp_joint_targets(raw)
        differences = [abs(a-b) for a,b in zip(raw[:15],clipped[:15])]
        phase = phases['standing' if row['time']<start else 'moving' if row['time']<stop else 'post_stop']
        phase['checked_samples'] += 1
        phase['clipped_samples'] += any(value>1e-12 for value in differences)
        phase['max_target_delta_rad'] = max(phase['max_target_delta_rad'],max(differences))
        affected += any(value>1e-12 for value in differences)
        for i,value in enumerate(differences):
            counts[i] += value>1e-12
            maxima[i] = max(maxima[i],value)
        max_reconstruction_error = max(max_reconstruction_error,max(abs(a-b) for a,b in
            zip(clipped[:15],row['composed_joint_target'][:15])))
        checked += 1
    if not checked or max_reconstruction_error>1e-12:
        raise ValueError('recorded targets do not match the declared deployment transform')
    return dict(checked_samples=checked,clipped_samples=affected,clipped_fraction=affected/checked,
        max_reconstruction_error_rad=max_reconstruction_error,phases=phases,
        joints=[dict(joint=name,clipped_samples=count,max_target_delta_rad=maximum)
                for name,count,maximum in zip(G1_29DOF_JOINTS[:15],counts,maxima) if count])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    report = dict(scope='Leg/waist target reconstruction only; composed arms excluded; not force or mission qualification',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),attempts=[])
    for path in args.inputs:
        try:
            raw = path.read_bytes()
            data = json.loads(raw)
            json.dumps(data,allow_nan=False)
            row = dict(source=str(path),sha256=hashlib.sha256(raw).hexdigest(),policy_sha256=data['policy_sha256'],audit=audit(data))
        except (OSError,ValueError,TypeError,KeyError) as error:
            row = dict(source=str(path),error=str(error))
        report['attempts'].append(row)
        print(json.dumps(row))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 1 if any('error' in row for row in report['attempts']) else 0


if __name__=='__main__':
    raise SystemExit(main())
