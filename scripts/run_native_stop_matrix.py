#!/usr/bin/env python3
"""Serial, predeclared native stopping comparisons in the pinned probe environment."""
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


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cases(policies,loads,delays,durations):
    return [dict(case_id=f'{name}-{load}-delay{delay}-y{lateral:+g}-duration{duration:g}',
                 policy=name,load=load,delay_steps=delay,linear_y=lateral,command_duration_s=duration)
            for name,load,delay,lateral,duration in itertools.product(policies,loads,delays,(.45,-.45),durations)]


def update_counts(report):
    report['passed'] = sum(row['passed'] for row in report['attempts'])
    report['by_policy'] = {}
    for name in report['policies']:
        rows = [row for row in report['attempts'] if row['policy']==name]
        report['by_policy'][name] = dict(planned=sum(row['policy']==name for row in report['plan']),
            attempted=len(rows),passed=sum(row['passed'] for row in rows),
            fallen=sum(row.get('metadata',{}).get('fallen') is True for row in rows),
            process_or_protocol_errors=sum('error' in row for row in rows))
    fields = ('policy','load','delay_steps','linear_y')
    groups = {}
    for row in report['plan']:
        groups.setdefault(tuple(row[key] for key in fields),[]).append(row)
    report['timing_neighborhoods'] = []
    for key,planned in groups.items():
        rows = [row for row in report['attempts'] if tuple(row[field] for field in fields)==key]
        complete = (len(rows)==len(planned) and {row['case_id'] for row in rows}=={row['case_id'] for row in planned})
        travel = [row['stopping']['sampled_max_travel_m'] for row in rows if 'sampled_max_travel_m' in row.get('stopping',{})]
        report['timing_neighborhoods'].append(dict(zip(fields,key),
            command_durations_s=sorted(row['command_duration_s'] for row in planned),
            planned=len(planned),attempted=len(rows),passed=sum(row['passed'] for row in rows),
            complete=complete,all_passed=complete and all(row['passed'] for row in rows),
            worst_sampled_travel_m=max(travel) if travel else None))


def run_case(case,policy,policy_sha256,output,duration,timeout,command_clock='legacy_float'):
    result_path = output/(case['case_id']+'.json')
    log_path = output/(case['case_id']+'.log')
    command = [sys.executable,str(HERE/'probe_native_policy.py'),'--policy',str(policy),
        '--output',str(result_path),'--duration',str(duration),'--delay-steps',str(case['delay_steps']),
        '--'+case['load'],'--record-actions','--require-mujoco-version','3.3.6',
        '--linear-y',str(case['linear_y']),'--command-duration',str(case['command_duration_s']),
        '--action-feedback','published','--command-clock',command_clock]
    record = dict(**case,command=command,source=str(result_path),log=str(log_path),passed=False)
    try:
        with log_path.open('x') as stream:
            result = subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=timeout,check=False)
        record['returncode'] = result.returncode
        if result.returncode:
            record['error'] = 'probe_process_failed'
        elif not result_path.is_file():
            record['error'] = 'missing_probe_result'
        else:
            record['source_sha256'] = digest(result_path)
            data = json.loads(result_path.read_text())
            if not isinstance(data,dict):
                raise ValueError('probe result must be an object')
            json.dumps(data,allow_nan=False)  # Reject nonfinite evidence before retaining metadata.
            record['metadata'] = {key:value for key,value in data.items() if key!='samples'}
            record['stopping'] = score_probe(data)
            # Pairing must not silently survive a changed policy or probe protocol.
            expected = dict(policy_sha256=policy_sha256,requested_duration_s=duration,
                delay_steps=case['delay_steps'],requested_command=[0.,case['linear_y'],0.],
                command_duration_s=case['command_duration_s'],action_feedback='published',
                payload=case['load']=='payload',held_empty=case['load']=='held-empty',
                observation_steps=[10],sensor_delay_steps=0,mujoco_version='3.3.6',record_actions=True,
                ramp=False,outage_at_s=None,resumed_checkpoint_sha256=None,command_clock=command_clock)
            mismatches = [key for key,value in expected.items() if data.get(key)!=value]
            history = record['stopping'].get('action_history')
            if history and history.get('valid') and any(abs(history[key]-.02*case['delay_steps'])>1e-8
                    for key in ('min_application_age_s','max_application_age_s')):
                mismatches.append('measured_application_age')
            if mismatches:
                record.update(error='probe_protocol_mismatch',mismatches=mismatches)
            else:
                record['passed'] = record['stopping']['passed']
    except subprocess.TimeoutExpired:
        record['error'] = 'probe_timeout'
    except (OSError,ValueError,TypeError,KeyError) as error:
        record.update(error='unreadable_or_invalid_probe',detail=str(error))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',action='append',help='Unique NAME=ONNX_PATH; repeated per comparison arm')
    parser.add_argument('--output',type=Path,help='New artifact directory; never overwrite')
    parser.add_argument('--report',type=Path,help='Optional new portable summary path, updated after every attempt')
    parser.add_argument('--export-summary',type=Path,help='Export a retained summary on the host, without running probes')
    parser.add_argument('--loads',nargs='+',choices=('payload','held-empty'),default=['payload','held-empty'])
    parser.add_argument('--delays',nargs='+',type=int,default=[2,3])
    parser.add_argument('--command-durations',nargs='+',type=float,default=[2.])
    parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--timeout',type=float,default=120.)
    parser.add_argument('--command-clock',choices=('legacy_float','physics_step'),default='legacy_float')
    args = parser.parse_args()
    if args.export_summary is not None:
        if args.policy or args.output or args.report is None or args.report.exists():
            parser.error('export requires a new --report path and no execution arguments')
        report = json.loads(args.export_summary.read_text())
        update_counts(report)
        report['source_summary'] = dict(path=str(args.export_summary),sha256=digest(args.export_summary))
        report['exporter_sha256'] = digest(Path(__file__))
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        return 0
    if not args.policy or args.output is None:
        parser.error('execution requires --policy and --output')
    policies = {}
    for item in args.policy:
        name,separator,path = item.partition('=')
        if not separator or not re.fullmatch(r'[A-Za-z0-9_-]+',name) or name in policies or not Path(path).is_file():
            parser.error('each policy requires a unique plain name and existing ONNX file')
        policies[name] = Path(path).resolve()
    if (any(delay<0 for delay in args.delays) or
            not all(math.isfinite(value) and value>0. for value in (*args.command_durations,args.duration,args.timeout)) or
            args.duration<10.+max(args.command_durations)):
        parser.error('positive finite durations, nonnegative delays and >=5 s post-stop observation required')
    plan = cases(policies,args.loads,args.delays,args.command_durations)
    if len({row['case_id'] for row in plan})!=len(plan):
        parser.error('duplicate matrix conditions')
    if args.output.exists():
        parser.error('output already exists')
    if args.report is not None and args.report.exists():
        parser.error('report already exists')
    args.output.mkdir(parents=True)
    report = dict(schema_version=1,complete=False,scope='Predeclared serial native probes; not mission or hardware acceptance',
        sources={name:digest(HERE/name) for name in ('run_native_stop_matrix.py','probe_native_policy.py','score_native_stopping.py')},
        policies={name:dict(path=str(path),sha256=digest(path),training=training_attempt(path.parent)) for name,path in policies.items()},
        plan=plan,attempts=[],duration_s=args.duration,timeout_s=args.timeout,command_clock=args.command_clock)
    destination = args.output/'summary.json'
    def save():
        update_counts(report)
        temporary = args.output/'summary.tmp'
        temporary.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        temporary.replace(destination)
        if args.report is not None:
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    try:
        for case in plan:
            record = run_case(case,policies[case['policy']],report['policies'][case['policy']]['sha256'],
                              args.output,args.duration,args.timeout,args.command_clock)
            report['attempts'].append(record)
            save()
            print(json.dumps(dict(case=case['case_id'],passed=record['passed'],error=record.get('error'),
                                  stopping=record.get('stopping'))),flush=True)
        report['complete'] = True
    finally:
        save()
    return 0 if report['complete'] and report['passed']==len(plan) else 1


if __name__=='__main__':
    raise SystemExit(main())
