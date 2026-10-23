#!/usr/bin/env python3
"""Publish all retained Dex3 calibration, DDS checks and mobile attempts."""
import argparse
import hashlib
import json
import re
from pathlib import Path

from publish_performance_evidence import attempt

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(data):
    if isinstance(data,dict):
        return {key:summary(value) for key,value in data.items() if key!='observations'}
    if isinstance(data,list):
        return [summary(value) for value in data]
    return data


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--date',default='20261020',help='Retained run date, YYYYMMDD')
    parser.add_argument('--include-stability-date',action='append',default=[],help='Also retain arm-stability probes from this YYYYMMDD directory')
    args=parser.parse_args()
    if any(not re.fullmatch(r'\d{8}',date) for date in [args.date,*args.include_stability_date]):
        parser.error('date must be YYYYMMDD')
    if args.output.exists():
        parser.error('output exists')
    bench=RESULTS/f'dex3_contact_{args.date}'
    records=[]
    for path in sorted(bench.glob('*.json')):
        records.append(dict(path=str(path.relative_to(ROOT)),sha256=digest(path),data=summary(json.loads(path.read_text()))))
    checks=[]
    for directory in sorted(bench.glob('ros_dds_*')):
        path=directory/'result.json'
        checks.append(dict(path=str(directory.relative_to(ROOT)),complete=path.is_file(),
            sha256=digest(path) if path.is_file() else None,
            data=json.loads(path.read_text()) if path.is_file() else {'success':False,'error':'setup failed before result creation'}))
    source_paths=['scripts/dex3_hand_model.py','scripts/measure_dex3_contact_grasp.py',
        'scripts/sweep_dex3_retention.py',
        'scripts/probe_dex3_arm_stability.py',
        'scripts/analyze_arm_control_trace.py','scripts/run_transport_demo.py',
        'scripts/verify_dex3_services.py','scripts/transport_worlds.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/dex3_grasp.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/manipulation_heading.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_run_logger.py',
        'ros2_ws/src/unitree_g1_sim_bringup/launch/g1_sim.launch.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/transport_demo.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/arm_kinematics.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/transport_perception.py',
        'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_motion_planner.py',
        'ros2_ws/src/unitree_mujoco/simulate/src/dex3_hand.h',
        'ros2_ws/src/unitree_mujoco/simulate/test/dex3_contact_bench.cc',
        'ros2_ws/src/unitree_g1_description/dex3_sources/manifest.json']
    result=dict(scope='Contact-only hand implementation; mobile delivery not yet qualified',
        source_hash_scope='Publisher-time working-tree snapshot, not per-run source provenance; retained runs may predate these sources.',
        source_sha256={name:digest(ROOT/name) for name in source_paths},
        calibration_notes='All selected-date calibration failures and sweeps retained. Contact bench uses actuated wrist slides; bench success does not qualify mobile delivery.',
        bench_records=records,ros_dds_checks=checks,
        ros_arm_diagnostics=[dict(attempt=attempt(path),capture_check=json.loads((path/'control_capture_check.json').read_text())
            if (path/'control_capture_check.json').exists() else dict(capture_valid=False,error='capture check missing'),
            final_capture_verification=json.loads((path/'control_capture_verified.json').read_text())
            if (path/'control_capture_verified.json').exists() else None)
            for path in sorted((RESULTS/'transport_demo').glob(f'dex3_ros_arm_*_{args.date}'))],
        stability_probes=[dict(path=str(path.relative_to(ROOT)),sha256=digest(path),data=summary(json.loads(path.read_text())))
            for date in sorted(set([args.date,*args.include_stability_date]))
            for path in sorted((RESULTS/f'dex3_arm_stability_{date}').glob('*.json'))],
        evaluation_trajectories=[dict(path=str(path.relative_to(ROOT)),sha256=digest(path))
            for path in sorted((RESULTS/'transport_demo').glob(f'dex3_contact_delivery_*_{args.date}/evaluation_object_history.json'))],
        mobile_attempts=[attempt(path) for path in sorted((RESULTS/'transport_demo').glob(f'dex3_contact_delivery_*_{args.date}'))])
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(bench_files=len(records),ros_dds_checks=len(checks),stability_probes=len(result['stability_probes']),ros_arm_diagnostics=len(result['ros_arm_diagnostics']),mobile_attempts=len(result['mobile_attempts']))))


if __name__=='__main__':
    main()
