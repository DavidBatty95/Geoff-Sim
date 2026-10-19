"""Fill missing cumulative stopping conditions while retaining earlier evidence."""
import argparse
import copy
import json
import math
from pathlib import Path

from run_native_stop_matrix import cases,digest,run_case,update_counts
from publish_performance_evidence import training_attempt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',action='append',required=True)
    parser.add_argument('--sources',nargs='+',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--command-durations',nargs='+',type=float)
    args = parser.parse_args()
    policies = {item.split('=',1)[0]:Path(item.split('=',1)[1]) for item in args.policy}
    if len(policies)!=len(args.policy):
        parser.error('duplicate policy names')
    hashes = {name:digest(path) for name,path in policies.items()}
    if None in hashes.values():
        parser.error('policy missing')
    durations = [1.4,1.5,1.6,1.7,1.8,1.96,1.98,2.,2.02,2.04,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3.,3.2]
    if args.command_durations is not None:
        durations = args.command_durations
    if len(set(durations))!=len(durations) or not all(math.isfinite(v) and 1.<=v<=5. for v in durations):
        parser.error('unique finite durations in [1,5] required')
    plan = cases(policies,['payload','held-empty'],[2,3],durations)
    fields = ('load','delay_steps','linear_y','command_duration_s')
    retained = {}
    for path in args.sources:
        source = json.loads(path.read_text())
        if not source.get('complete') or not source.get('require_motion_progress'):
            continue
        if any(source.get(key)!=value for key,value in dict(command_clock='physics_step',
                joint_feedback='sensors',pd_feedback='sensors',duration_s=15.,payload_offset_x_m=None).items()):
            continue
        for row in source['attempts']:
            if row.get('error') or 'motion_progress' not in row:
                continue
            sha = source['policies'][row['policy']]['sha256']
            if sha not in hashes.values():
                continue
            key = (sha,*(row[field] for field in fields))
            if key in retained and retained[key]['passed']!=row['passed']:
                raise ValueError(f'contradictory prior verdict: {key}')
            record = copy.deepcopy(row)
            record['reused_evidence'] = dict(report=str(path),sha256=digest(path),case_id=row['case_id'])
            retained.setdefault(key,record)
    reused = sum((hashes[row['policy']],*(row[field] for field in fields)) in retained for row in plan)
    print(json.dumps(dict(planned=len(plan),reused=reused,new_probes=len(plan)-reused)),flush=True)
    if args.plan_only:
        return 0
    if args.output is None:
        parser.error('output required')
    args.output.mkdir(parents=True,exist_ok=False)
    report = dict(complete=False,scope='Cumulative inventory; reused evidence is not an independent trial',
        plan=plan,attempts=[],policies={name:dict(path=str(path),sha256=hashes[name],training=training_attempt(path.parent)) for name,path in policies.items()},
        command_clock='physics_step',joint_feedback='sensors',pd_feedback='sensors',duration_s=15.,
        payload_offset_x_m=None,require_motion_progress=True,runner_sha256=digest(Path(__file__)))
    def save():
        update_counts(report)
        (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    for case in plan:
        key = (hashes[case['policy']],*(case[field] for field in fields))
        if key in retained:
            record = dict(retained[key],**case)
        else:
            record = run_case(case,policies[case['policy']],hashes[case['policy']],args.output,15.,120.,
                              'physics_step','sensors','sensors',None,True)
            print(json.dumps(dict(case=case['case_id'],passed=record['passed'],error=record.get('error'))),flush=True)
        report['attempts'].append(record)
        save()
    report['complete'] = True
    save()
    return int(not all(row['passed'] for row in report['attempts']))


if __name__=='__main__':
    raise SystemExit(main())
