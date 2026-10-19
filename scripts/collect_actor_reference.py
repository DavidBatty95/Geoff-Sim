"""Re-capture every declared passing condition, retaining failed captures."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from score_native_stopping import motion_progress, score_probe

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs='+', type=Path, required=True)
    parser.add_argument('--policy-name', required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--expected-sources', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    policy_hash = hashlib.sha256(args.policy.read_bytes()).hexdigest()
    plan, identities = [], set()
    for path in args.reports:
        report = json.loads(path.read_text())
        if not report['complete'] or report['policies'][args.policy_name]['sha256'] != policy_hash:
            raise ValueError('incomplete report or policy mismatch')
        for case in report['attempts']:
            if case['policy'] != args.policy_name or not case['passed']:
                continue
            identity = (case['load'],case['delay_steps'],case['linear_y'],case['command_duration_s'])
            if identity in identities:
                raise ValueError('duplicate source condition')
            identities.add(identity)
            plan.append(dict(case_id=case['case_id'],load=case['load'],delay_steps=case['delay_steps'],
                linear_y=case['linear_y'],command_duration_s=case['command_duration_s'],
                report=str(path),report_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    if len(plan) != args.expected_sources:
        raise ValueError('source count differs from declaration')
    args.output.mkdir(parents=True,exist_ok=False)
    summary = dict(plan=plan,attempts=[],complete=False,policy_sha256=policy_hash)
    def save():
        (args.output/'collection.json').write_text(json.dumps(summary,indent=2)+'\n')
    save()
    outputs = []
    for i,case in enumerate(plan):
        output = args.output/f'capture_{i:03d}.json'
        command = [sys.executable,str(HERE/'probe_native_policy.py'),'--policy',str(args.policy),
            '--output',str(output),'--'+case['load'],'--delay-steps',str(case['delay_steps']),
            '--linear-y',str(case['linear_y']),'--command-duration',str(case['command_duration_s']),
            '--duration','15','--command-clock','physics_step','--joint-feedback','sensors',
            '--pd-feedback','sensors','--action-feedback','published','--record-actions',
            '--record-observations','--require-mujoco-version','3.3.6']
        record = dict(case_id=case['case_id'],command=command,passed=False)
        try:
            with (args.output/f'capture_{i:03d}.log').open('w') as log:
                result = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=120)
            record['returncode'] = result.returncode
            if result.returncode == 0:
                data = json.loads(output.read_text())
                record['stopping'] = score_probe(data)
                record['motion_progress'] = motion_progress(data)
                record['passed'] = (data['policy_sha256']==policy_hash and
                    record['stopping']['passed'] and record['motion_progress']['passed'])
        except (OSError,ValueError,KeyError,subprocess.TimeoutExpired) as error:
            record['error'] = str(error)
        summary['attempts'].append(record)
        outputs.append(output)
        save()
        print(json.dumps(dict(case=case['case_id'],passed=record['passed'])),flush=True)
    summary['complete'] = True
    save()
    if not all(row['passed'] for row in summary['attempts']):
        return 1
    return subprocess.call([sys.executable,str(HERE/'build_actor_reference.py'),
                            *map(str,outputs),'--output',str(args.output/'corpus.json')])


if __name__ == '__main__':
    raise SystemExit(main())
