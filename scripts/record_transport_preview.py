#!/usr/bin/env python3
"""Capture actual MuJoCo window frames alongside a retained transport attempt.

Frames are visual evidence, not control inputs or an acceptance measurement.
Run only with exclusive simulator access. Export edited GIFs from the timestamped
frames after inspecting the result; never infer success from a recording alone.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from record_demo_preview import decode_xwd,activate

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--fps',type=float,default=3.)
    parser.add_argument('--width',type=int,default=960)
    parser.add_argument('transport_args',nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}',args.run_id):
        parser.error('invalid run ID')
    if not 1<=args.fps<=6 or not 480<=args.width<=1280:
        parser.error('use 1..6 fps and 480..1280 width')
    run = RUNS/args.run_id
    recording = RUNS.parent/'transport_recordings'/args.run_id
    if run.exists():
        parser.error('run ID already exists')
    recording.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('DISPLAY',':0')
    os.environ.setdefault('XAUTHORITY','/tmp/.docker.xauth')
    forwarded = args.transport_args[1:] if args.transport_args[:1]==['--'] else args.transport_args
    command = [sys.executable,str(ROOT/'scripts/run_transport_demo.py'),'--run-id',args.run_id,*forwarded]
    metadata = dict(recorded_at=datetime.now(timezone.utc).isoformat(),run=str(run.relative_to(ROOT)),
                    kind='live MuJoCo window capture',command=command,requested_fps=args.fps,
                    frames=[],capture_errors=0)
    trace = None
    phase = 'startup'
    window = None
    started = time.monotonic()
    with (recording/'runner.log').open('w') as log:
        process = subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while process.poll() is None and time.monotonic()-started<780:
                tick = time.monotonic()
                if trace is None and (run/'trace.jsonl').exists():
                    trace = (run/'trace.jsonl').open()
                if trace is not None:
                    while True:
                        offset = trace.tell()
                        line = trace.readline()
                        if not line.endswith('\n'):
                            trace.seek(offset)
                            break
                        row = json.loads(line)
                        if row.get('topic')=='transport_state':
                            phase = row['data'].get('phase',phase)
                if window is None:
                    tree = subprocess.check_output(['xwininfo','-root','-tree'],text=True)
                    matches = [re.search(r'0x[0-9a-f]+',line).group() for line in tree.splitlines() if 'MuJoCo :' in line]
                    if len(matches)==1:
                        window = matches[0]
                        activate(window)
                if window is not None:
                    try:
                        raw = subprocess.check_output(['xwd','-silent','-id',window],stderr=subprocess.DEVNULL,timeout=5)
                        image = decode_xwd(raw)
                        # Retain the full window for transparent, editable crops.
                        image.thumbnail((args.width,args.width))
                        name = f'frame-{len(metadata["frames"]):05d}.jpg'
                        image.save(recording/name,quality=88)
                        metadata['frames'].append(dict(file=name,monotonic_s=tick,phase=phase))
                    except (subprocess.SubprocessError,ValueError):
                        metadata['capture_errors'] += 1
                        window = None
                time.sleep(max(0.,1/args.fps-(time.monotonic()-tick)))
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            if trace is not None:
                trace.close()
            metadata['returncode'] = process.returncode
            for name in ('result.json','manifest.json','source_provenance.json'):
                path = run/name
                metadata[name+'_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
            (recording/'capture.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(recording=str(recording),frames=len(metadata['frames']),returncode=process.returncode)))
    return process.returncode


if __name__=='__main__':
    raise SystemExit(main())
