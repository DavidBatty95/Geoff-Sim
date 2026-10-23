"""Serial cumulative native-suite search with explicit incomplete-screen evidence.

No runtime policy is changed. Every accepted incumbent must complete the whole
current suite without falls and with fewer failures. Zero-failure incumbents
must also pass a newly declared validation batch before this worker stops.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import random
import shutil
import signal
import subprocess
import sys

from run_native_stop_matrix import cases,digest,run_case,update_counts

HERE = Path(__file__).resolve().parent
FIELDS = ('load','delay_steps','linear_y','command_duration_s')


def key(row):
    return tuple(row[field] for field in FIELDS)


def screen_reason(attempts, incumbent_failures):
    if any(row.get('error') for row in attempts):
        return 'process_or_protocol_error'
    if any(row.get('metadata',{}).get('fallen') is not False for row in attempts):
        return 'fall_or_missing_fall_status'
    if sum(not row['passed'] for row in attempts)>=incumbent_failures:
        return 'cannot_improve_failure_count'
    return None


def proposals(weights, step):
    for layer in range(4):
        for sign in (-1,1):
            candidate = weights.copy()
            candidate[layer] = round(min(1.,max(0.,candidate[layer]+sign*step)),6)
            if candidate!=weights:
                yield candidate


def initial_weights(policy, left, right):
    manifest = json.loads(policy.with_name('manifest.json').read_text())
    exported = json.loads(policy.with_suffix('.manifest.json').read_text())
    if (manifest['left']['sha256']!=digest(left) or manifest['right']['sha256']!=digest(right)
            or exported['policy_sha256']!=digest(policy)):
        raise ValueError('initial actor or interpolation endpoints do not match provenance')
    weights = manifest.get('layer_weights',[manifest['right_weight']]*4)
    if len(weights)!=4 or not all(type(v) in (float,int) and math.isfinite(v) and 0<=v<=1 for v in weights):
        raise ValueError('invalid initial layer weights')
    return weights


def proposal_batch(weights, step, plateau_sweep, rng):
    if step>=.0003125:
        return list(proposals(weights,step)),None
    sigma = (.0005,.001,.002,.005,.01)[plateau_sweep%5]
    return [[round(min(1.,max(0.,v+rng.gauss(0.,sigma))),6) for v in weights]
            for _ in range(8)],sigma


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left',type=Path,required=True)
    parser.add_argument('--right',type=Path,required=True)
    parser.add_argument('--initial-policy',type=Path,required=True)
    parser.add_argument('--initial-name',default='blend95')
    parser.add_argument('--initial-reports',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--artifacts',type=Path,required=True)
    parser.add_argument('--max-proposals',type=int,default=0,help='0 continues until qualified, interrupted or resource-blocked')
    args = parser.parse_args()
    weights = initial_weights(args.initial_policy,args.left,args.right)
    initial_hash = digest(args.initial_policy)
    baseline = {}
    sources = []
    for path in args.initial_reports:
        report = json.loads(path.read_text())
        if not report['complete'] or report['policies'][args.initial_name]['sha256']!=initial_hash:
            raise ValueError('initial evidence incomplete or policy mismatch')
        for field,value in dict(require_motion_progress=True,command_clock='physics_step',
                                joint_feedback='sensors',pd_feedback='sensors',duration_s=15.).items():
            if report.get(field)!=value:
                raise ValueError('initial protocol mismatch')
        sources.append(dict(path=str(path),sha256=digest(path)))
        for row in report['attempts']:
            if row['policy']==args.initial_name:
                if key(row) in baseline:
                    raise ValueError('duplicate initial condition')
                baseline[key(row)] = row
    if len(baseline)!=208 or any(row.get('error') or row['metadata']['fallen'] for row in baseline.values()):
        raise ValueError('initial suite must contain 208 valid no-fall outcomes')
    args.output.mkdir(parents=True,exist_ok=False)
    args.artifacts.mkdir(parents=True,exist_ok=False)
    (args.output/'worker_source.py').write_bytes(Path(__file__).read_bytes())
    state = dict(status='running',initial_reports=sources,proposals=0,accepted=[],
        best_policy=str(args.initial_policy),best_weights=weights,
        best_failures=sum(not row['passed'] for row in baseline.values()),suite_cases=len(baseline),
        current_suite=list(baseline.values()),qualified=False,auto_deployment=False,
        source_sha256={name:digest(HERE/name) for name in ('iterate_native_layers.py','interpolate_payload_actor.py',
            'export_payload_actor.py','probe_native_policy.py','score_native_stopping.py','run_native_stop_matrix.py')})
    def save():
        destination = args.output/'status.json'
        temporary = args.output/'status.tmp'
        temporary.write_text(json.dumps(state,indent=2)+'\n')
        temporary.replace(destination)
    save()
    rng = random.Random(20261019)
    seen = {tuple(state['best_weights'])}
    step = .01
    plateau_sweep = 0

    def evaluate(weights, validation=False, durations=None):
        identifier = f'proposal_{state["proposals"]:05d}' if not validation else f'validation_{len(state["accepted"]):05d}'
        output = args.output/identifier
        output.mkdir(exist_ok=False)
        if validation:
            policy = Path(state['best_policy'])
            plan = cases(['candidate'],['payload','held-empty'],[2,3],durations)
        else:
            artifact = args.artifacts/identifier
            command = [sys.executable,str(HERE/'interpolate_payload_actor.py'),'--left',str(args.left),
                '--right',str(args.right),'--right-weight','.95','--layer-weights',*map(str,weights),'--output',str(artifact)]
            with (output/'export.log').open('w') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
            policy = artifact/'policy.onnx'
            ordered = sorted(baseline.values(),key=lambda row:row['passed'])
            plan = [dict(policy='candidate',case_id=f'candidate-{i:03d}',**{field:row[field] for field in FIELDS})
                    for i,row in enumerate(ordered)]
        report = dict(complete=False,plan=plan,attempts=[],policies={'candidate':dict(path=str(policy),sha256=digest(policy))},
            weights=weights,incumbent_failure_count=state['best_failures'],validation=validation,
            duration_s=15.,require_motion_progress=True,command_clock='physics_step',joint_feedback='sensors',pd_feedback='sensors',payload_offset_x_m=None)
        def save_report():
            update_counts(report)
            (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        save_report()
        for case in plan:
            record = run_case(case,policy,report['policies']['candidate']['sha256'],output,15.,120.,
                              'physics_step','sensors','sensors',None,True)
            report['attempts'].append(record)
            reason = screen_reason(report['attempts'],state['best_failures']) if not validation else None
            if reason:
                report['screen_rejection'] = reason
            save_report()
            if record.get('error'):
                raise RuntimeError('probe setup/protocol failure; retained for diagnosis')
            if reason:
                break
        report['complete'] = len(report['attempts'])==len(plan)
        save_report()
        return policy,report

    try:
        while not args.max_proposals or state['proposals']<args.max_proposals:
            if any(digest(HERE/name)!=value for name,value in state['source_sha256'].items()):
                state['status'] = 'blocked_source_changed'
                break
            if shutil.disk_usage(args.output).free<10*1024**3:
                state['status'] = 'blocked_low_disk_space'
                break
            improved = False
            batch,sigma = proposal_batch(state['best_weights'],step,plateau_sweep,rng)
            state['proposal_sigma'] = sigma
            if sigma is not None:
                plateau_sweep += 1
            for weights in batch:
                if args.max_proposals and state['proposals']>=args.max_proposals:
                    break
                if tuple(weights) in seen:
                    continue
                seen.add(tuple(weights))
                state['proposals'] += 1
                state['current_weights'] = weights
                save()
                policy,report = evaluate(weights)
                failures = sum(not row['passed'] for row in report['attempts'])
                print(json.dumps(dict(event='screened',proposal=state['proposals'],weights=weights,
                    complete=report['complete'],attempted=len(report['attempts']),failures=failures,
                    rejection=report.get('screen_rejection'))),flush=True)
                if report['complete'] and not report.get('screen_rejection') and failures<state['best_failures']:
                    baseline = {key(row):row for row in report['attempts']}
                    state.update(best_policy=str(policy),best_weights=weights,best_failures=failures,
                                 current_suite=list(baseline.values()))
                    state['accepted'].append(dict(proposal=state['proposals'],failures=failures,weights=weights))
                    improved = True
                    print(json.dumps(dict(event='improved',proposal=state['proposals'],failures=failures)),flush=True)
                    save()
                    if failures==0:
                        used = {condition[-1] for condition in baseline}
                        fresh = rng.sample([round(i/50,2) for i in range(55,201) if round(i/50,2) not in used],6)
                        # Declare the batch on disk before looking at its outcomes.
                        state['next_validation_durations'] = fresh
                        save()
                        _,validation = evaluate(weights,True,fresh)
                        baseline.update({key(row):row for row in validation['attempts']})
                        state.update(current_suite=list(baseline.values()),suite_cases=len(baseline),
                                     best_failures=sum(not row['passed'] for row in baseline.values()))
                        if not state['best_failures']:
                            state.update(status='native_suite_passed',qualified=True)
                            save()
                            return 0
                        if any(row['metadata']['fallen'] for row in validation['attempts']):
                            state['status'] = 'blocked_validation_fall_requires_review'
                            save()
                            return 1
                    # Refine around each newly accepted incumbent immediately,
                    # rather than continuing an obsolete sweep or wide plateau.
                    step,plateau_sweep = .01,0
                    save()
                    break
                save()
            if not improved:
                step = max(.00015625,step/2)
            state['step'] = step
            save()
        if state['status']=='running':
            state['status'] = 'proposal_budget_reached'
    except KeyboardInterrupt:
        state['status'] = 'interrupted'
    except Exception as error:
        state.update(status='blocked_error',error=repr(error))
        raise
    finally:
        save()
    return 1


if __name__=='__main__':
    raise SystemExit(main())
