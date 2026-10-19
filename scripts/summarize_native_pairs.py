"""Retain individual fixes and regressions in complete matched stopping matrices."""
import argparse
import hashlib
import json
from pathlib import Path

FIELDS = ('load','delay_steps','linear_y','command_duration_s')


def compare(report, candidate, baseline):
    if not report['complete']:
        raise ValueError('incomplete matrix')
    def index(rows, policy):
        selected = [row for row in rows if row['policy']==policy]
        result = {tuple(row[key] for key in FIELDS):row for row in selected}
        if not result or len(result)!=len(selected):
            raise ValueError('missing or duplicate conditions')
        return result
    left,right = index(report['attempts'],candidate),index(report['attempts'],baseline)
    if left.keys()!=right.keys() or left.keys()!=index(report['plan'],candidate).keys() or right.keys()!=index(report['plan'],baseline).keys():
        raise ValueError('unmatched or incomplete conditions')
    changes = []
    for key,a in left.items():
        b = right[key]
        if type(a['passed']) is not bool or type(b['passed']) is not bool:
            raise ValueError('invalid verdict')
        if a['passed']!=b['passed']:
            changes.append(dict(zip(FIELDS,key),kind='fix' if a['passed'] else 'regression',
                candidate_case=a['case_id'],baseline_case=b['case_id'],
                candidate_stopping=a.get('stopping'),baseline_stopping=b.get('stopping'),
                candidate_progress=a.get('motion_progress'),baseline_progress=b.get('motion_progress'),
                candidate_error=a.get('error'),baseline_error=b.get('error')))
    return dict(candidate=candidate,baseline=baseline,cases=len(left),
        fixes=sum(row['kind']=='fix' for row in changes),
        regressions=sum(row['kind']=='regression' for row in changes),changes=changes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports',nargs='+',type=Path,required=True)
    parser.add_argument('--pair',action='append',required=True,help='CANDIDATE=BASELINE')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    results = []
    for path in args.reports:
        report = json.loads(path.read_text())
        pairs = [compare(report,*pair.split('=')) for pair in args.pair]
        results.append(dict(report=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),pairs=pairs))
        print(path.name,[(p['candidate'],p['baseline'],p['fixes'],p['regressions']) for p in pairs])
    args.output.write_text(json.dumps(dict(results=results),indent=2)+'\n')


if __name__=='__main__':
    main()
