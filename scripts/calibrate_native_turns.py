#!/usr/bin/env python3
"""Matched signed native turn probes, retaining every failure and policy identity.

Run with scripts/run_mjlab_docker.sh --no-sync --with mujoco==3.3.6 python.
The fitted response is diagnostic until separately verified through ROS.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--magnitudes',type=float,nargs='+',default=[.35,.6,.9])
    parser.add_argument('--delay-steps',type=int,default=2)
    parser.add_argument('--held-empty',action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    results = []
    for magnitude in args.magnitudes:
        if not 0.<magnitude<=.9:
            parser.error('probe magnitudes must be in (0, .9]')
        for sign in (-1,1):
            path = args.output/f'yaw_{sign*magnitude:+.2f}.json'
            subprocess.run([sys.executable,str(Path(__file__).with_name('probe_native_policy.py')),
                '--policy',str(args.policy),'--output',str(path),'--duration','14',
                '--command-duration','6','--yaw',str(sign*magnitude),
                '--delay-steps',str(args.delay_steps),'--require-mujoco-version','3.3.6',
                *(['--held-empty'] if args.held_empty else [])],check=True)
            data = json.loads(path.read_text())
            results.append({key:value for key,value in data.items() if key!='samples'})
            (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
