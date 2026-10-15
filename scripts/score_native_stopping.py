#!/usr/bin/env python3
"""Score retained fixed-command stop probes; upright alone is not a passed stop."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def score_probe(data,*,max_travel=.30,max_speed=.08,max_yaw_rate=.12,dwell=1.,
                settle_deadline=3.,minimum_observation=5.,max_sample_gap=.05):
    limits = dict(max_travel_m=max_travel,max_speed_mps=max_speed,max_yaw_rate_rps=max_yaw_rate,
                  dwell_s=dwell,settle_deadline_s=settle_deadline,
                  minimum_observation_s=minimum_observation,max_sample_gap_s=max_sample_gap)
    if not all(math.isfinite(v) and v>0. for v in limits.values()):
        raise ValueError('stopping limits must be finite and positive')
    reasons = []
    duration = data.get('command_duration_s')
    if (data.get('replay_trace') or data.get('ramp') or type(duration) not in (int,float) or
            not math.isfinite(duration) or duration<=0.):
        return dict(passed=False,limits=limits,reasons=['fixed_command_stop_required'])
    command_end = 5.+duration  # Explicit native-probe standing/command contract.
    samples = []
    previous = None
    try:
        for row in data.get('samples',[]):
            t = row['time']
            values = (t,*row['xy'],*row['body_velocity'],row['yaw_rate'])
            if (len(row['xy'])!=2 or len(row['body_velocity'])!=2 or
                    not all(type(v) in (int,float) and math.isfinite(v) for v in values) or t<0.):
                raise ValueError('invalid sample geometry')
            if previous is not None and t<=previous:
                raise ValueError('nonincreasing sample time')
            previous = t
            # Match the probe's strict command switch. A float just below the
            # boundary can legitimately still carry the preceding command.
            if t>=command_end:
                if len(row['command'])!=3 or any(row['command']):
                    reasons.append('nonzero_post_stop_command')
                samples.append(row)
    except (KeyError,TypeError,ValueError):
        return dict(passed=False,limits=limits,reasons=['invalid_or_reordered_samples'])
    if not samples:
        return dict(passed=False,limits=limits,reasons=['missing_stop_samples'])
    first,last = samples[0]['time'],samples[-1]['time']
    requested_end = data.get('requested_duration_s')
    if type(requested_end) not in (int,float) or not math.isfinite(requested_end) or requested_end<=command_end:
        reasons.append('missing_requested_probe_duration')
    elif last+max_sample_gap+1e-9<requested_end:
        reasons.append('incomplete_requested_probe')
    gaps = [first-command_end]+[b['time']-a['time'] for a,b in zip(samples,samples[1:])]
    if max(gaps)>max_sample_gap+1e-9:
        reasons.append('sample_coverage_gap')
    if last-command_end+1e-9<minimum_observation:
        reasons.append('insufficient_post_stop_observation')
    travel = max(math.dist(row['xy'],samples[0]['xy']) for row in samples)
    if travel>max_travel:
        reasons.append('post_stop_travel_exceeded')
    stable_since = settled_at = None
    resumed = False
    for row in samples:
        stationary = math.hypot(*row['body_velocity'])<=max_speed and abs(row['yaw_rate'])<=max_yaw_rate
        if not stationary:
            resumed |= settled_at is not None
            stable_since = None
        else:
            stable_since = row['time'] if stable_since is None else stable_since
            if settled_at is None and row['time']-stable_since+1e-9>=dwell:
                settled_at = row['time']-command_end
    if settled_at is None or settled_at>settle_deadline+1e-9:
        reasons.append('settling_deadline_exceeded')
    if resumed:
        reasons.append('motion_resumed_after_settling')
    if data.get('fallen') is not False:
        reasons.append('fall_or_missing_fall_status')
    if data.get('sampled_nonfloor_robot_contacts')!={}:
        reasons.append('scene_contact_or_missing_contact_coverage')
    return dict(passed=not reasons,limits=limits,reasons=sorted(set(reasons)),
        stop_samples=len(samples),command_end_s=command_end,first_stop_sample_s=first,
        observation_s=last-command_end,max_sample_gap_s=max(gaps),
        sampled_max_travel_m=travel,settled_at_s=settled_at,
        terminal_speed_mps=math.hypot(*samples[-1]['body_velocity']),
        terminal_yaw_rate_rps=samples[-1]['yaw_rate'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',nargs='+',type=Path,help='Probe JSON files or directories of probe JSONs')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--require-pass',action='store_true',help='Return nonzero if any selected probe fails the declared stopping gate')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; retain each report separately')
    paths = sorted({file for path in args.inputs for file in (path.glob('*.json') if path.is_dir() else [path])})
    if not paths:
        parser.error('no probe files selected')
    records = []
    for path in paths:
        try:
            raw = path.read_bytes()
            data = json.loads(raw)
            if not isinstance(data,dict):
                raise ValueError('probe must be a JSON object')
            record = dict(source=str(path),sha256=hashlib.sha256(raw).hexdigest(),
                stopping=score_probe(data),metadata={key:value for key,value in data.items() if key!='samples'})
        except (OSError,ValueError) as error:
            record = dict(source=str(path),stopping=dict(passed=False,reasons=['unreadable_probe']),error=str(error))
        records.append(record)
    report = dict(schema_version=1,scorer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Declared sampled stopping gate, not hardware/crowd safety or policy promotion',
        limitations=['Displacement starts at the first post-command sample, not exact actuator stop time.',
                     'Sampled non-floor contacts do not establish continuous contact coverage.',
                     'Applied-feedback probes have simulator-known action history; deployment has no such acknowledgement.'],
        attempts=records,passed=sum(row['stopping']['passed'] for row in records))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),attempts=len(records),passed=report['passed'])))
    return 1 if args.require_pass and report['passed']!=len(records) else 0


if __name__=='__main__':
    raise SystemExit(main())
