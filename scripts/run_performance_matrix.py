#!/usr/bin/env python3
"""Serial, retained exploration/transport validation across colour tasks/layouts.

Every attempt, including startup failures and incomplete deliveries, remains in
the summary. This deliberately does not turn a verified placement into mission
success when cleanup or the original deadline failed.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import fcntl
import math
import os
import signal
import uuid

from performance_outcome import strict_success

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'


def checkpoint(path, summary):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(summary, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def collect_result(record, run):
    path = run/'result.json'
    try:
        raw = path.read_bytes()
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError('result must be a JSON object')
    except (OSError, ValueError) as error:
        record.update(success=False, artifact_error=str(error))
        return
    record.update(result_sha256=hashlib.sha256(raw).hexdigest(),
        **{key:data.get(key) for key in ('mission_complete','data_valid','fallen',
                                       'contact_samples','error','deliveries','search_summary')})
    record['success'] = record['returncode']==0 and strict_success(data)


def execute(command, timeout, log):
    """Allow wrapper cleanup on interruption; never continue after an outer timeout."""
    process = subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        return process.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        try:
            os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--tasks',nargs='+',default=['red:red','red:blue','blue:red','blue:blue'])
    parser.add_argument('--layouts',type=int,nargs='+',default=[0,1])
    parser.add_argument('--seeds',type=int,nargs='+',default=[48])
    parser.add_argument('--world',default='transport_medium')
    parser.add_argument('--people',type=int,default=0)
    parser.add_argument('--duration',type=float,default=600.)
    parser.add_argument('--fast-start',action='store_true')
    parser.add_argument('--policy-device-clock',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--detach',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--startup-cleanup-budget',type=float,default=900.,
                        help='Wall seconds added to each mission deadline for build/startup/cleanup')
    args = parser.parse_args()
    if (not args.policy.is_file() or not math.isfinite(args.duration) or args.duration<=0 or
            not math.isfinite(args.startup_cleanup_budget) or args.startup_cleanup_budget<=0):
        parser.error('existing policy and finite positive duration/startup budget required')
    output = args.output or ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/performance_matrix'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    if args.detach:
        output.mkdir(parents=True,exist_ok=False)
        command = [sys.executable,str(Path(__file__)),*[arg for arg in sys.argv[1:] if arg!='--detach']]
        if args.output is None:
            command += ['--output',str(output)]
        with (output/'matrix.log').open('x') as log:
            process = subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print(json.dumps(dict(pid=process.pid,output=str(output))))
        return 0
    output.mkdir(parents=True,exist_ok=True)
    RUNS.mkdir(parents=True,exist_ok=True)
    lock = (RUNS.parent/'.performance_matrix.lock').open('a')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        parser.error('another performance matrix owns the simulator')
    # A detached parent may have created matrix.log, but never reuse a cohort.
    try:
        with (output/'summary.json').open('x'):
            pass
    except OSError:
        lock.close()
        raise
    summary = dict(role='native_performance_development_matrix',world=args.world,people=args.people,
                   schema_version=2,duration_s=args.duration,
                   startup_cleanup_budget_s=args.startup_cleanup_budget,
                   policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),attempts=[],complete=False)
    for seed in args.seeds:
        for layout in args.layouts:
            for task in args.tasks:
                run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex
                command = [sys.executable,str(ROOT/'scripts/run_exploration_transport_demo.py'),
                    '--world',args.world,'--people',str(args.people),'--seed',str(seed),'--layout-seed',str(layout),
                    '--task',task,'--duration',str(args.duration),'--policy',str(args.policy.resolve()),'--no-rviz',
                    '--run-id',run_id,
                    '--policy-device-clock' if args.policy_device_clock else '--no-policy-device-clock',
                    *(['--fast-start'] if args.fast_start else [])]
                summary['attempts'].append(dict(task=task,layout=layout,seed=seed,returncode=None,
                    command=command,runs=[run_id],success=False,status='planned'))
    checkpoint(output/'summary.json',summary)
    interrupted = False
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    old_handler = signal.signal(signal.SIGTERM,interrupt)
    try:
        for record in summary['attempts']:
            record['status'] = 'running'
            checkpoint(output/'summary.json',summary)
            print('RUN',record['command'],flush=True)
            try:
                with (output/(record['runs'][0]+'.log')).open('x') as log:
                    record['returncode'] = execute(record['command'],args.duration+args.startup_cleanup_budget,log)
                record['status'] = 'finished'
            except OSError as error:
                record.update(status='startup_failed',error=str(error))
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                record.update(status='timeout' if isinstance(error,subprocess.TimeoutExpired) else 'interrupted',
                              error='Outer execution interrupted; verify simulator cleanup before restarting')
                interrupted = True
            collect_result(record,RUNS/record['runs'][0])
            checkpoint(output/'summary.json',summary)
            if interrupted:
                break
        summary['complete'] = all(row['status'] not in ('planned','running') for row in summary['attempts']) and not interrupted
    finally:
        checkpoint(output/'summary.json',summary)
        lock.close()
        signal.signal(signal.SIGTERM,old_handler)
    return 0 if all(item['success'] and item['returncode']==0 for item in summary['attempts']) else 1


if __name__=='__main__':
    raise SystemExit(main())
