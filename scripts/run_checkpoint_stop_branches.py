#!/usr/bin/env python3
"""Stop-time counterfactuals on shared native trajectories; correlated diagnostic branches."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import subprocess
import sys

from publish_performance_evidence import training_attempt
from score_native_stopping import score_probe

HERE = Path(__file__).resolve().parent
STATE_FIELDS = ('time','xy','height','tilt','yaw','body_velocity','yaw_rate','foot_normal_force_n',
                'ankle_body_height_m','leg_joint_pos_rad','payload_attached','payload_position_error_m')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare_branch(parent,branch):
    prefix = branch['branch_prefix_samples']
    if type(prefix) is not int or prefix<1 or prefix>=min(len(parent['samples']),len(branch['samples'])):
        raise ValueError('invalid branch prefix')
    if branch['samples'][:prefix]!=parent['samples'][:prefix]:
        raise ValueError('branch did not preserve the parent prefix')
    before,after = parent['samples'][prefix],branch['samples'][prefix]
    state = {key:before[key] for key in STATE_FIELDS}
    if state!={key:after[key] for key in STATE_FIELDS}:
        raise ValueError('physical state differs at branch command transition')
    if not any(before['command']) or any(after['command']) or after['time']!=branch['branch_stop_time_s']:
        raise ValueError('branch did not change the command at the declared transition')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',action='append',help='NAME=PATH, repeated per policy')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--times',nargs='+',type=float,default=[6.49,6.74,6.99,7.24,7.49,7.74,7.99])
    parser.add_argument('--delay-steps',type=int,default=3)
    parser.add_argument('--export-summary',type=Path)
    parser.add_argument('--report',type=Path)
    args = parser.parse_args()
    if args.export_summary:
        if args.report is None or args.report.exists() or args.policy or args.output:
            parser.error('export requires a new report and no execution arguments')
        report = json.loads(args.export_summary.read_text())
        report['source_summary_sha256'] = digest(args.export_summary)
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        return 0
    if not args.policy or args.output is None or args.output.exists():
        parser.error('execution requires policies and a new output directory')
    policies = {}
    for item in args.policy:
        name,sep,path = item.partition('=')
        if not sep or not re.fullmatch(r'[A-Za-z0-9_-]+',name) or name in policies or not Path(path).is_file():
            parser.error('unique plain policy name and existing file required')
        policies[name] = Path(path).resolve()
    if args.delay_steps<1 or len(set(args.times))!=len(args.times) or any(not math.isfinite(t) or not 5.<t<8.9 for t in args.times):
        parser.error('positive action delay and unique checkpoint times inside (5, 8.9) required')
    args.output.mkdir(parents=True)
    plan = [dict(policy=name,checkpoint_at=t,case=f'{name}-stop-{t:g}') for name,t in itertools.product(policies,args.times)]
    report = dict(schema_version=1,complete=False,scope='Correlated checkpoint stop branches, not independent acceptance trials',
        configuration=dict(delay_steps=args.delay_steps,command_duration_s=4.,duration_s=15.,linear_y=-.45,payload=True),
        sources={name:digest(HERE/name) for name in ('run_checkpoint_stop_branches.py','probe_native_policy.py','score_native_stopping.py')},
        policies={name:dict(sha256=digest(path),training=training_attempt(path.parent)) for name,path in policies.items()},
        plan=plan,attempts=[])
    reference = {}
    def save():
        report['by_policy'] = {name:dict(planned=len(args.times),attempted=sum(r['policy']==name for r in report['attempts']),
            gate_passes=sum(r['policy']==name and r['passed'] for r in report['attempts'])) for name in policies}
        temporary = args.output/'summary.tmp'
        temporary.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        temporary.replace(args.output/'summary.json')
    save()
    try:
        for case in plan:
            record = dict(**case,passed=False)
            report['attempts'].append(record)
            save()
            parent,branch,checkpoint = [args.output/(case['case']+'_'+suffix+'.json') for suffix in ('parent','branch','checkpoint')]
            base = [sys.executable,str(HERE/'probe_native_policy.py'),'--policy',str(policies[case['policy']]),
                    '--duration','15','--delay-steps',str(args.delay_steps),'--payload','--record-actions','--record-support',
                    '--require-mujoco-version','3.3.6','--linear-y','-.45','--command-duration','4','--action-feedback','published']
            try:
                for target,options in ((parent,['--checkpoint-at',str(case['checkpoint_at']),'--checkpoint-output',str(checkpoint)]),
                                       (branch,['--resume-checkpoint',str(checkpoint),'--branch-stop'])):
                    with target.with_suffix('.log').open('x') as stream:
                        subprocess.run([*base,'--output',str(target),*options],stdout=stream,stderr=subprocess.STDOUT,timeout=120,check=True)
                p,b = json.loads(parent.read_text()),json.loads(branch.read_text())
                for data in (p,b):
                    json.dumps(data,allow_nan=False)
                    if data['policy_sha256']!=report['policies'][case['policy']]['sha256']:
                        raise ValueError('policy changed during experiment')
                signature = hashlib.sha256(json.dumps(p['samples'],sort_keys=True,allow_nan=False).encode()).hexdigest()
                if signature!=reference.setdefault(case['policy'],signature):
                    raise ValueError('parent trajectory changed between checkpoint captures')
                if not b['branch_stop'] or b['resumed_checkpoint_sha256']!=digest(checkpoint):
                    raise ValueError('incorrect checkpoint branch provenance')
                record.update(initial_state=compare_branch(p,b),parent_trajectory_sha256=signature,
                    parent_source=str(parent),parent_sha256=digest(parent),branch_source=str(branch),branch_sha256=digest(branch),
                    checkpoint_sha256=digest(checkpoint),parent_outcome={k:v for k,v in p.items() if k!='samples'},
                    branch_outcome={k:v for k,v in b.items() if k!='samples'},stopping=score_probe(b))
                record['passed'] = record['stopping']['passed']
            except (OSError,ValueError,TypeError,KeyError,subprocess.SubprocessError) as error:
                record['error'] = str(error)
            save()
            print(json.dumps(dict(case=case['case'],passed=record['passed'],error=record.get('error'),
                                  initial_state=record.get('initial_state'),stopping=record.get('stopping'))),flush=True)
        report['complete'] = True
    finally:
        save()
    return 0 if all(row['passed'] for row in report['attempts']) else 1


if __name__=='__main__':
    raise SystemExit(main())
