#!/usr/bin/env python3
"""Matched signed native turn probes, retaining every failure and policy identity.

Run with scripts/run_mjlab_docker.sh --no-sync --with mujoco==3.3.6 --with onnxruntime python.
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
    parser.add_argument('--duration',type=float,default=14.)
    parser.add_argument('--command-duration',type=float,default=6.)
    parser.add_argument('--ramp',action='store_true',help='Use the probe command ramp for both start and stop')
    parser.add_argument('--action-feedback',choices=('applied','published'),default='applied')
    args = parser.parse_args()
    if not 0.<args.command_duration<args.duration-5.:
        parser.error('duration must include five seconds standing, a positive command, and settling')
    args.output.mkdir(parents=True,exist_ok=True)
    results = []
    for magnitude in args.magnitudes:
        if not 0.<magnitude<=.9:
            parser.error('probe magnitudes must be in (0, .9]')
        for sign in (-1,1):
            path = args.output/f'yaw_{sign*magnitude:+.2f}.json'
            subprocess.run([sys.executable,str(Path(__file__).with_name('probe_native_policy.py')),
                '--policy',str(args.policy),'--output',str(path),'--duration',str(args.duration),
                '--command-duration',str(args.command_duration),'--yaw',str(sign*magnitude),
                '--delay-steps',str(args.delay_steps),'--require-mujoco-version','3.3.6',
                '--action-feedback',args.action_feedback,
                *(['--held-empty'] if args.held_empty else []),
                *(['--ramp'] if args.ramp else [])],check=True)
            data = json.loads(path.read_text())
            results.append({key:value for key,value in data.items() if key!='samples'})
            (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
