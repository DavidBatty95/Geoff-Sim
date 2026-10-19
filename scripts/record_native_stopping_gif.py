"""Render a complete native stopping probe; publish GIF only after qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image,ImageDraw

import probe_native_policy
from score_native_stopping import score_probe,motion_progress


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path,help='New raw probe JSON')
    parser.add_argument('--gif',required=True,type=Path)
    parser.add_argument('--linear-y',required=True,type=float)
    parser.add_argument('--label',required=True)
    args = parser.parse_args()
    if args.output.exists() or args.gif.exists() or args.gif.with_suffix('.json').exists():
        parser.error('refusing overwrite')
    frames,times = [],[]
    renderer = None
    original_step = mujoco.mj_step
    camera = mujoco.MjvCamera()
    camera.distance,camera.azimuth,camera.elevation = 3.2,135.,-15.

    def capture(model,data):
        nonlocal renderer
        if renderer is None:
            renderer = mujoco.Renderer(model,height=400,width=640)
        camera.lookat[:] = data.qpos[:3]
        renderer.update_scene(data,camera=camera)
        image = Image.fromarray(renderer.render()).convert('RGB')
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,640,42),fill=(18,24,32))
        draw.text((10,6),args.label,fill='white')
        phase = 'STAND' if data.time<5.-1e-8 else ('MOVE' if data.time<7.-1e-8 else 'BRAKE / HOLD')
        draw.text((10,23),f'{data.time:05.2f} / 15.00 s | {phase} | 150 g | delay 60 ms | full run, 1x',fill='white')
        frames.append(image)
        times.append(float(data.time))

    def rendered_step(model,data):
        if not frames:
            capture(model,data)
        original_step(model,data)
        if round(data.time/model.opt.timestep)%50==0:
            capture(model,data)

    command = ['--policy',str(args.policy),'--output',str(args.output),'--payload',
        '--linear-y',str(args.linear_y),'--command-duration','2','--duration','15',
        '--delay-steps','3','--action-feedback','published','--record-actions',
        '--command-clock','physics_step','--joint-feedback','sensors','--pd-feedback','sensors',
        '--require-mujoco-version','3.3.6']
    sys.argv = ['probe_native_policy.py',*command]
    mujoco.mj_step = rendered_step
    try:
        probe_native_policy.main()
    finally:
        mujoco.mj_step = original_step
        if renderer is not None:
            renderer.close()
    data = json.loads(args.output.read_text())
    stopping,progress = score_probe(data),motion_progress(data)
    if not stopping['passed'] or not progress['passed']:
        raise ValueError('probe failed; raw result retained, no gallery GIF published')
    if len(times)!=151 or abs(times[0])>1e-8 or abs(times[-1]-15)>1e-8 or not np.allclose(np.diff(times),.1):
        raise ValueError('incomplete frame coverage')
    draw = ImageDraw.Draw(frames[-1])
    draw.rectangle((0,360,640,400),fill=(18,70,40))
    draw.text((10,374),'RUN COMPLETE | movement + strict stopping PASS',fill='white')
    args.gif.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(args.gif,save_all=True,append_images=frames[1:],duration=[100]*150+[1000],loop=0,optimize=False)
    with Image.open(args.gif) as check:
        count = check.n_frames
        duration = 0
        for i in range(count):
            check.seek(i)
            duration += check.info['duration']
    if count!=151 or duration!=16000:
        raise ValueError('encoded GIF coverage mismatch')
    metadata = dict(kind='Offscreen rendering of actual native probe states; no cuts',
        label=args.label,command=command,policy_sha256=data['policy_sha256'],
        probe_sha256=data['probe_sha256'],recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        result_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        gif_sha256=hashlib.sha256(args.gif.read_bytes()).hexdigest(),
        simulation_start_s=times[0],simulation_end_s=times[-1],frame_count=count,
        frame_period_s=.1,playback_duration_ms=duration,final_frame_hold_ms=1000,
        stopping=stopping,motion_progress=progress,fallen=data['fallen'])
    args.gif.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(gif=str(args.gif),frames=count,complete=True)))


if __name__=='__main__':
    main()
