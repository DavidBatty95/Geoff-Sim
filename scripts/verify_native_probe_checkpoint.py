#!/usr/bin/env python3
"""Compare uninterrupted and restored native probe samples, including actions."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def first_difference(before,after):
    for index,(a,b) in enumerate(zip(before,after)):
        if a!=b:
            return dict(index=index,fields={key:dict(original=value,resumed=b.get(key))
                for key,value in a.items() if value!=b.get(key)})
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--inspect',action='store_true',help='Read existing parity artifacts and print the first differing sample')
    parser.add_argument('--export-report',type=Path,help='Export existing parity summary and physical outcomes on the host')
    args = parser.parse_args()
    if args.export_report is not None:
        if args.export_report.exists():
            parser.error('report already exists')
        source = args.output/'summary.json'
        report = json.loads(source.read_text())
        report['source_summary_sha256'] = digest(source)
        for row in report['attempts']:
            original = json.loads((args.output/(row['case']+'_original.json')).read_text())
            row['original_outcome'] = {key:value for key,value in original.items() if key!='samples'}
        args.export_report.parent.mkdir(parents=True,exist_ok=True)
        args.export_report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        return 0
    if args.inspect:
        for row in json.loads((args.output/'summary.json').read_text())['attempts']:
            before,after = [json.loads((args.output/(row['case']+'_'+suffix+'.json')).read_text())
                            for suffix in ('original','resumed')]
            print(json.dumps(dict(case=row['case'],difference=first_difference(before['samples'],after['samples']))))
        return 0
    if args.policy is None:
        parser.error('--policy is required for execution')
    if args.output.exists():
        parser.error('output already exists')
    args.output.mkdir(parents=True)
    report = dict(scope='Synchronous native-probe continuation parity, not independent mission qualification',
                  policy_sha256=digest(args.policy),complete=False,attempts=[])
    summary = args.output/'summary.json'
    def save():
        summary.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    cases = [('loaded',['--payload'],6.987),
             ('native_sensors',['--payload','--joint-feedback','sensors','--pd-feedback','sensors'],6.987),
             ('sensor_cadence_outage',['--held-empty','--sensor-delay-steps','3','--observation-steps','7','13',
                                     '--outage-at','7.5','--outage-measured-hold'],6.991)]
    for name,options,at in cases:
        checkpoint = args.output/(name+'_checkpoint.json')
        original,resumed = [args.output/(name+'_'+suffix+'.json') for suffix in ('original','resumed')]
        record = dict(case=name,checkpoint_at_requested_s=at,passed=False)
        report['attempts'].append(record)
        save()
        base = [sys.executable,str(Path(__file__).with_name('probe_native_policy.py')),
                '--policy',str(args.policy),'--duration','15','--delay-steps','2','--record-actions',
                '--require-mujoco-version','3.3.6','--linear-y','-.45','--command-duration','2',
                '--action-feedback','published',*options]
        try:
            for target,extra in ((original,['--checkpoint-at',str(at),'--checkpoint-output',str(checkpoint)]),
                                 (resumed,['--resume-checkpoint',str(checkpoint)])):
                with target.with_suffix('.log').open('x') as stream:
                    subprocess.run([*base,'--output',str(target),*extra],stdout=stream,stderr=subprocess.STDOUT,
                                   timeout=120,check=True)
            before,after = json.loads(original.read_text()),json.loads(resumed.read_text())
            saved = json.loads(checkpoint.read_text())
            prefix = len(saved['controller']['rows'])
            record.update(checkpoint_sha256=digest(checkpoint),original_sha256=digest(original),
                          resumed_sha256=digest(resumed),checkpoint_time_s=saved['state'][0],
                          samples=len(before['samples']),recomputed_samples=len(after['samples'])-prefix,
                          first_mismatch=next((i for i,(a,b) in enumerate(zip(before['samples'],after['samples'])) if a!=b),None))
            exclusions = {'checkpoint_saved','resumed_checkpoint_sha256','checkpoint_time_s','branch_prefix_samples'}
            record['passed'] = (record['recomputed_samples']>0 and
                {k:v for k,v in before.items() if k not in exclusions}==
                {k:v for k,v in after.items() if k not in exclusions})
        except (OSError,ValueError,subprocess.SubprocessError) as error:
            record['error'] = str(error)
        save()
        print(json.dumps(record),flush=True)
    report['complete'] = True
    save()
    return 0 if all(row['passed'] for row in report['attempts']) else 1


if __name__=='__main__':
    raise SystemExit(main())
