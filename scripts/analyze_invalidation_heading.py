#!/usr/bin/env python3
"""Offline source-pose, extrapolation and physical-heading invalidation audit."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from analyze_primitive_invalidation import advance_robot, check
from unitree_g1_mujoco_bridge.motion_envelope import advance_planar_pose


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(row):
    return math.atan2(2*(row['qw']*row['qz']+row['qx']*row['qy']),
                      1-2*(row['qy']**2+row['qz']**2))


def heading_at(rows, stamp, max_gap=.15):
    """Interpolate wrapped yaw by acquisition epoch, never extrapolate truth."""
    index = bisect.bisect_left([r['source_stamp'] for r in rows], stamp)
    if index < len(rows) and abs(rows[index]['source_stamp']-stamp) < 1e-6:
        return yaw(rows[index])
    if index == 0 or index == len(rows):
        return None
    a, b = rows[index-1:index+1]
    gap = b['source_stamp']-a['source_stamp']
    if not 0 < gap <= max_gap:
        return None
    return wrap(yaw(a)+(stamp-a['source_stamp'])/gap*wrap(yaw(b)-yaw(a)))


def diagnose(pair, odometry, evaluation):
    states = [pair['previous'], pair['current']]
    samples = []
    for state in states:
        source = min(odometry, key=lambda row: abs(row['source_stamp']-state['odom_stamp']), default=None)
        if source is None or abs(source['source_stamp']-state['odom_stamp']) > 1e-6:
            return dict(valid=False, reason='exact source odometry missing')
        age = state['forecast_epoch']-source['source_stamp']
        projected = advance_planar_pose((source['x'], source['y'], yaw(source)), state['velocity'], age)
        truth_source = heading_at(evaluation, source['source_stamp'])
        truth_now = heading_at(evaluation, state['forecast_epoch'])
        physical = None if truth_source is None or truth_now is None else wrap(truth_now-truth_source)
        samples.append(dict(source_stamp=source['source_stamp'], forecast_epoch=state['forecast_epoch'],
            pose_age_s=age, source_yaw_rad=yaw(source), source_wz_rad_s=source['wz'],
            captured_velocity=state['velocity'], extrapolated_yaw_increment_rad=age*state['velocity'][2],
            reconstructed_pose=list(projected), captured_pose=state['pose'],
            reconstruction_yaw_residual_rad=wrap(state['pose'][2]-projected[2]),
            reconstruction_xy_residual_m=math.hypot(state['pose'][0]-projected[0],state['pose'][1]-projected[1]),
            truth_source_yaw_rad=truth_source, truth_now_yaw_rad=truth_now,
            physical_yaw_during_pose_age_rad=physical,
            extrapolation_increment_error_rad=None if physical is None else wrap(age*state['velocity'][2]-physical)))
    a, b = samples
    elapsed = states[1]['forecast_epoch']-states[0]['forecast_epoch']
    predicted = advance_robot(states[0], elapsed)
    physical_delta = None if any(s['truth_now_yaw_rad'] is None for s in samples) else wrap(b['truth_now_yaw_rad']-a['truth_now_yaw_rad'])
    captured_delta = wrap(states[1]['pose'][2]-states[0]['pose'][2])
    model_delta = wrap(predicted[2]-states[0]['pose'][2])
    current = states[1]
    measured = np.array([*current['pose'], *current['velocity'], *current['filtered']])
    components = {}
    for name, indices in [('xy', [0, 1]), ('yaw', [2]), ('linear_velocity', [3, 4]),
                          ('yaw_rate', [5]), ('yaw_and_rate', [2, 5]),
                          ('filtered_command', [6, 7, 8]), ('all', list(range(9)))]:
        substituted = predicted.copy()
        substituted[indices] = measured[indices]
        components[name] = check(current, substituted, np.array(current['people']), np.array(current['radii']))
    reverse = {}
    for name, indices in [('yaw', [2]), ('yaw_rate', [5]), ('yaw_and_rate', [2, 5])]:
        substituted = measured.copy()
        substituted[indices] = predicted[indices]
        reverse[name] = check(current, substituted, np.array(current['people']), np.array(current['radii']))
    return dict(valid=True, elapsed_s=elapsed, samples=samples,
        component_substitutions=dict(scope='one measured component group substituted into predicted state; current occupancy only, no static geometry', results=components),
        reverse_substitutions=dict(scope='predicted component group substituted into measured state; diagnostic, not a control recommendation', results=reverse),
        source_yaw_delta_rad=wrap(b['source_yaw_rad']-a['source_yaw_rad']),
        extrapolation_increment_delta_rad=b['extrapolated_yaw_increment_rad']-a['extrapolated_yaw_increment_rad'],
        captured_yaw_delta_rad=captured_delta, response_predicted_yaw_delta_rad=model_delta,
        physical_yaw_delta_rad=physical_delta,
        captured_minus_physical_delta_rad=None if physical_delta is None else wrap(captured_delta-physical_delta),
        response_minus_physical_delta_rad=None if physical_delta is None else wrap(model_delta-physical_delta),
        source_frame_offset_change_rad=None if any(s['truth_source_yaw_rad'] is None for s in samples) else
            wrap((b['source_yaw_rad']-b['truth_source_yaw_rad'])-(a['source_yaw_rad']-a['truth_source_yaw_rad'])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    pairs, odometry, evaluation = [], [], []
    digest = hashlib.sha256()
    with (args.run/'trace.jsonl').open('rb') as stream:
        for line in stream:
            digest.update(line)
            row = json.loads(line)
            if row['topic'] == 'odom':
                odometry.append(row['data'])
            elif row['topic'] == 'evaluation_odom':
                evaluation.append(row['data'])
            elif row['topic'] == 'planner_debug':
                pair = row['data'].get('kinodynamic_details', {}).get('invalidation_audit')
                if pair:
                    pairs.append(pair)
    evaluation.sort(key=lambda row: row['source_stamp'])
    results = [diagnose(pair, odometry, evaluation) for pair in pairs]
    report = dict(scope='offline heading decomposition; truth never used for control; no motor-application attribution',
        run_id=args.run.name, trace_sha256=digest.hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Logged odometry is rounded; reconstruction uses forecast epoch, which can differ from pose-query time.',
                     'Truth yaw interpolated only across acquisition gaps at most 150 ms; no extrapolation.',
                     'Response rollout assumes the admitted program; command publication is not actuator acknowledgement.'],
        pairs=results)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    for index, result in enumerate(results):
        print(json.dumps(dict(pair=index, **{k:v for k,v in result.items() if k not in ('samples','component_substitutions','reverse_substitutions')},
            component_passes={k:v['passed'] for k,v in result.get('component_substitutions',{}).get('results',{}).items()},
            reverse_passes={k:v['passed'] for k,v in result.get('reverse_substitutions',{}).get('results',{}).items()})))
    return int(not results or any(not result['valid'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
