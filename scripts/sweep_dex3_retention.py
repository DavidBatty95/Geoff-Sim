#!/usr/bin/env python3
"""Retain a declared thumb-opposition sweep at the marginal raised grasp."""
import argparse
import hashlib
import json
from pathlib import Path

from measure_dex3_contact_grasp import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--validate',action='store_true',help='Compare baseline and selected closure across four geometries, including calibration repeat')
    parser.add_argument('--roll-sweep',action='store_true',help='Test wrist orientations with return to level carry and original release orientation')
    args = parser.parse_args()
    if args.validate and args.roll_sweep:
        parser.error('choose one study mode')
    if args.output.exists():
        parser.error('output exists')
    attempts = []
    if args.roll_sweep:
        cases = [('roll',0.,-.7,.136,.044,.03,.06,roll,0.)
                 for roll in (0.,.3,.6,1.,1.5,-.3,-.6,-1.,-1.5)]
    elif args.validate:
        cases = [(label,opposition,-.7,x,y,height,pitch) for label,x,y,height,pitch in (
            ('nominal',.135,.04,.02,0.),('marginal',.136,.044,.03,.06),
            ('opposite_pitch',.132,.038,.025,-.06),('higher',.138,.042,.035,.06))
            for opposition in (0.,.05)]
        cases = [(*case,0.,None) for case in cases]
    else:
        cases = [('calibration',opposition,tip,.136,.044,.03,.06,0.,None)
                 for opposition in (-.15,-.05,.05,.15) for tip in (-.5,-.7,-1.)]
    for label,opposition,thumb_tip,x,y,height,pitch,roll,carry_roll in cases:
        result = run('right',x,y,.02,.08,
            grip=[opposition,-1.,thumb_tip,1.5,1.5,1.5,1.5],support='box',
            hold_seconds=40.,sway_m=.01,grasp_height_offset=height,turns=True,wrist_pitch=pitch,
            release_rate=.3,release_from_measured=True,pitch_about_tool=True,wrist_roll=roll,carry_roll=carry_roll)
        result['condition'] = label
        attempts.append(result)
        print(json.dumps({k:v for k,v in result.items() if k!='observations'}),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(scope='raised rotational retention study; all outcomes retained; not mobile qualification',
        validation=args.validate,roll_sweep=args.roll_sweep,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),attempts=attempts),indent=2,allow_nan=False)+'\n')
    passed = all(row['success'] for row in attempts) if args.validate else any(row['success'] for row in attempts)
    return int(not passed)


if __name__=='__main__':
    raise SystemExit(main())
