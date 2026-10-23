#!/usr/bin/env python3
"""Dex3 contact-only tabletop pickup, lift, hold, lower and release bench."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from dex3_hand_model import ROOT,add_hand,joint_names

sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.dex3_grasp import close_positions,GRIP_KP,GRIP_KD
from unitree_g1_mujoco_bridge.arm_kinematics import rpy_matrix


def fixture(side='right',x=.135,y=.04,radius=.02,mass=.08,as_xml=False,support='plane',grasp_height_offset=0.,rotational=False):
    root = ET.Element('mujoco',model='dex3_contact_bench')
    ET.SubElement(root,'compiler',angle='radian')
    ET.SubElement(root,'option',timestep='.002',integrator='Euler',iterations='100')
    ET.SubElement(root,'asset')
    world = ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -1 3')
    table_z = 1.-(.025+radius)
    if support not in ('plane','box'):
        raise ValueError('support must be plane or box')
    ET.SubElement(world,'geom',name='table',type=support,size='.55 .50 .04' if support=='box' else '.5 .5 .01',
                  pos=f'0 0 {table_z-(.04 if support=="box" else 0.)}',rgba='.45 .4 .3 1',
                  friction='1 .01 .001' if support=='box' else '1 .005 .0001')
    wrist = ET.SubElement(world,'body',name=side+'_wrist_yaw_link',pos=f'-.04 0 {1.+grasp_height_offset}')
    ET.SubElement(wrist,'joint',name='bench_x',type='slide',axis='1 0 0',damping='2')
    ET.SubElement(wrist,'joint',name='bench_z',type='slide',axis='0 0 1',damping='2')
    if rotational:
        ET.SubElement(wrist,'joint',name='bench_y',type='slide',axis='0 1 0',damping='2')
        for name,axis in (('yaw','0 0 1'),('pitch','0 1 0'),('roll','1 0 0')):
            ET.SubElement(wrist,'joint',name='bench_'+name,type='hinge',axis=axis,damping='.1',armature='.005')
    add_hand(root,wrist,side)
    actuators = ET.SubElement(root,'actuator')
    for axis in ('x','z'):
        ET.SubElement(actuators,'position',name='bench_'+axis,joint='bench_'+axis,kp='2000',kv='50',forcerange='-100 100')
    if rotational:
        for axis in ('yaw','pitch','roll'):
            ET.SubElement(actuators,'position',name='bench_'+axis,joint='bench_'+axis,kp='100',kv='2',forcerange='-10 10')
        ET.SubElement(actuators,'position',name='bench_y',joint='bench_y',kp='2000',kv='50',forcerange='-100 100')
    object_body = ET.SubElement(world,'body',name='object',pos=f'{x} {y if side=="right" else -y} 1')
    ET.SubElement(object_body,'freejoint',name='object_free')
    ET.SubElement(object_body,'geom',name='object',type='cylinder',size=f'{radius} {.025+radius}',mass=str(mass),rgba='.9 .1 .1 1',friction='1 .005 .0001',condim='4')
    xml = ET.tostring(root,encoding='unicode')
    return xml if as_xml else mujoco.MjModel.from_xml_string(xml)


def run(side,x,y,radius,mass,preview=None,grip=None,gif=None,hold_seconds=1.,sway_m=0.,release_gap=0.,release_rate=1.5,support='plane',release_from_measured=False,grasp_height_offset=0.,turns=False,wrist_pitch=0.,closure_wait=1.5,lift_retract=0.,pitch_about_tool=False,wrist_roll=0.,carry_roll=None):
    if not np.isfinite(hold_seconds) or not 1.<=hold_seconds<=120. or not np.isfinite(sway_m) or not 0.<=sway_m<=.03:
        raise ValueError('hold must be 1..120 s and wrist sway 0..0.03 m')
    if not np.isfinite(release_gap) or not 0.<=release_gap<=.05 or not np.isfinite(release_rate) or not .2<=release_rate<=1.5:
        raise ValueError('release gap must be 0..0.05 m and opening rate 0.2..1.5 rad/s')
    if not np.isfinite(wrist_pitch) or abs(wrist_pitch)>.25 or (turns and hold_seconds<20.):
        raise ValueError('pitch must be within +/-0.25 rad; turns need at least 20 seconds holding')
    if not np.isfinite(wrist_roll) or abs(wrist_roll)>1.57 or (carry_roll is not None and (not np.isfinite(carry_roll) or abs(carry_roll)>1.57)):
        raise ValueError('wrist roll must be within +/-1.57 rad')
    if not np.isfinite(closure_wait) or not .3<=closure_wait<=2.5:
        raise ValueError('closure wait must be 0.3..2.5 s')
    if not np.isfinite(lift_retract) or not 0.<=lift_retract<=.12:
        raise ValueError('lift retraction must be 0..0.12 m')
    if (preview or gif) and (hold_seconds!=1. or sway_m!=0. or release_gap!=0. or release_rate!=1.5 or release_from_measured or turns or wrist_pitch or wrist_roll or carry_roll is not None or closure_wait!=1.5 or lift_retract):
        raise ValueError('media capture requires the standard ten-second sequence')
    if not np.isfinite(grasp_height_offset) or not 0.<=grasp_height_offset<=.04:
        raise ValueError('grasp height offset must be 0..0.04 m')
    rotational = turns or bool(wrist_pitch) or bool(wrist_roll) or carry_roll is not None
    model = fixture(side,x,y,radius,mass,support=support,grasp_height_offset=grasp_height_offset,rotational=rotational)
    data = mujoco.MjData(model)
    assert model.neq==0
    joints = [model.joint(name).id for name in joint_names(side)]
    qids,dofs = model.jnt_qposadr[joints],model.jnt_dofadr[joints]
    limits = np.array([2.45,1.4,1.4,1.4,1.4,1.4,1.4])
    reference = np.zeros(7)
    close = np.array(grip if grip is not None else close_positions(side))
    if side=='left' and grip is not None:
        close *= -1
    obj = model.body('object').id
    wrist = model.body(side+'_wrist_yaw_link').id
    tool_offset = np.array([.135,.04 if side=='right' else -.04,0.])
    wrist_angle_qids = [model.joint('bench_'+axis).qposadr[0] for axis in ('yaw','pitch','roll')] if rotational else []
    object_dof = model.jnt_dofadr[model.joint('object_free').id]
    rows = []
    images = []
    gif_frames = []
    renderer = mujoco.Renderer(model,height=400,width=640) if preview or gif else None
    def render_frame(lift):
        from PIL import Image,ImageDraw
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.12,.035 if side=='right' else -.035,1.+lift]
        camera.distance,camera.azimuth,camera.elevation = .4,90. if side=='right' else -90.,-40.
        renderer.update_scene(data,camera=camera)
        image = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,640,43),fill=(15,22,32))
        phase = ('APPROACH / OPEN' if data.time<1.5 else 'CLOSE' if data.time<3 else
                 'LIFT' if data.time<5 else 'HOLD' if data.time<6 else 'LOWER' if data.time<8 else 'RELEASE')
        draw.text((10,6),f'Dex3 {side} | physical contacts only | actuated wrist test rig',fill='white')
        draw.text((10,24),f'{data.time:05.2f} / 10.00 s | {phase} | {mass*1000:.0f} g cylinder | no object weld',fill='white')
        return image
    if gif:
        mujoco.mj_forward(model,data)
        gif_frames.append(render_frame(0.))
    dt = model.opt.timestep
    lift_at = 1.5+closure_wait
    hold_at = lift_at+2.
    lower_at = hold_at+hold_seconds
    release_at = lower_at+2.
    end_at = release_at+max(2.,1.5/release_rate+1.)
    for step in range(round(end_at/dt)):
        t = step*dt
        if release_from_measured and step==round(release_at/dt):
            reference[:] = data.qpos[qids]
        desired = close if 1.5<=t<lower_at+2. else np.zeros(7)
        rate = release_rate if t>=release_at else 1.5
        reference += np.clip(desired-reference,-rate*dt,rate*dt)
        data.qfrc_applied[dofs] = np.clip(GRIP_KP*(reference-data.qpos[qids])-GRIP_KD*data.qvel[dofs],-limits,limits)
        data.ctrl[0] = .04*min(1.,t)
        retract_fraction = min(1.,max(0.,(t-lift_at)/2.)) if t<lower_at else max(0.,1.-(t-lower_at)/2.)
        data.ctrl[0] -= lift_retract*retract_fraction
        if hold_at<=t<lower_at:
            data.ctrl[0] += sway_m*np.sin(2*np.pi*1.5*(t-hold_at))
        lift = min(.15,max(0.,(t-lift_at)*.075)) if t<lower_at else max(release_gap,.15-(t-lower_at)*.075)
        data.ctrl[1] = lift
        if rotational:
            elapsed = min(hold_seconds,max(0.,t-hold_at))
            data.ctrl[2] = .5*np.pi*(1.-np.cos(2*np.pi*elapsed/20.)) if turns else 0.
            data.ctrl[3] = wrist_pitch*min(1.,t)
            data.ctrl[4] = wrist_roll*min(1.,t)
            if carry_roll is not None:
                data.ctrl[4] += (carry_roll-wrist_roll)*retract_fraction
            if pitch_about_tool:
                displacement = tool_offset-rpy_matrix((data.ctrl[4],data.ctrl[3],data.ctrl[2]))@tool_offset
                data.ctrl[0] += displacement[0]
                data.ctrl[1] += displacement[2]
                data.ctrl[5] = displacement[1]
        assert np.all(data.qfrc_applied[object_dof:object_dof+6]==0) and np.all(data.xfrc_applied==0)
        mujoco.mj_step(model,data)
        if preview and step in (round(1/dt),round(2/dt),round(3/dt)-1,round(5.5/dt)):
            images.append(render_frame(lift))
        if gif and (step+1)%round(.1/dt)==0:
            gif_frames.append(render_frame(lift))
        if step%round(.02/dt)==0:
            contacts = []
            table_force = 0.
            force = np.zeros(6)
            for i,contact in enumerate(data.contact):
                bodies = [int(model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]
                if 0 in bodies and obj not in bodies:
                    mujoco.mj_contactForce(model,data,i,force)
                    table_force += max(0.,float(force[0]))
                if obj in bodies:
                    other = bodies[1] if bodies[0]==obj else bodies[0]
                    mujoco.mj_contactForce(model,data,i,force)
                    if force[0]>.05:
                        contacts.append(model.body(other).name or 'world')
            rows.append(dict(time=float(data.time),position=data.xpos[obj].tolist(),contacts=contacts,
                             upright_cosine=float(data.xmat[obj].reshape(3,3)[2,2]),
                             object_axis_in_hand=(data.xmat[wrist].reshape(3,3).T@data.xmat[obj].reshape(3,3)[:,2]).tolist(),
                             hand_table_force_n=table_force,
                             wrist_angles_rad=data.qpos[wrist_angle_qids].tolist() if rotational else None,
                             tool_position=(data.xpos[wrist]+data.xmat[wrist].reshape(3,3)@tool_offset).tolist(),
                             joints=data.qpos[qids].tolist()))
    if renderer is not None:
        renderer.close()
    if preview:
        from PIL import Image
        sheet = Image.new('RGB',(1280,800))
        for i,image in enumerate(images):
            sheet.paste(image,((i%2)*640,(i//2)*400))
        preview.parent.mkdir(parents=True,exist_ok=True)
        sheet.save(preview)
    held = [r for r in rows if hold_at+.2<=r['time']<=lower_at-.1]
    initial = np.array(rows[0]['position'])
    opposed = all(any('thumb' in c for c in r['contacts']) and any('index' in c or 'middle' in c for c in r['contacts']) for r in held)
    min_lift = min(r['position'][2]-initial[2] for r in held)
    released = rows[-1]['contacts']==['world'] or ('world' in rows[-1]['contacts'] and not any('_hand_' in c for c in rows[-1]['contacts']))
    final_position = np.array(rows[-1]['position'])
    stable_release = all(np.linalg.norm(np.array(row['position'])-final_position)<.01
                         for row in rows if row['time']>=end_at-1.)
    upright_release = rows[-1]['upright_cosine']>.98
    success = bool(min_lift>.1 and opposed and released and abs(final_position[2]-initial[2])<.025
                   and upright_release and stable_release)
    if gif and success:
        from PIL import ImageDraw
        assert len(gif_frames)==101 and abs(data.time-10.)<1e-8
        draw = ImageDraw.Draw(gif_frames[-1])
        draw.rectangle((0,360,640,400),fill=(15,70,35))
        draw.text((10,374),'COMPLETE | grasp, lift, hold and release PASS',fill='white')
        gif.parent.mkdir(parents=True,exist_ok=True)
        gif_frames[0].save(gif,save_all=True,append_images=gif_frames[1:],duration=[100]*100+[1000],loop=0,optimize=False)
    return dict(success=success,scope='prescribed wrist contact-only bench, not mobile qualification',
        mujoco_version=mujoco.__version__,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        wrist_actuator_count=int(model.nu),
        release_gap_m=release_gap,release_rate_rad_s=release_rate,
        release_from_measured=release_from_measured,
        support_geometry=support,
        grasp_height_offset_m=grasp_height_offset,
        wrist_pitch_rad=wrist_pitch,turns=turns,turn_period_s=20. if turns else None,
        wrist_roll_rad=wrist_roll,carry_roll_rad=carry_roll,
        max_hand_table_force_before_lift_n=max(r['hand_table_force_n'] for r in rows if r['time']<lift_at),
        closure_wait_s=closure_wait,
        lift_retract_m=lift_retract,
        pitch_about_tool=pitch_about_tool,
        max_held_tilt_rad=float(max(np.arccos(np.clip(r['upright_cosine'],-1.,1.)) for r in held)),
        final_upright_cosine=rows[-1]['upright_cosine'],
        upright_release=upright_release,stable_release=bool(stable_release),
        hold_seconds=hold_seconds,sway_m=sway_m,sway_hz=1.5,grip_kp=GRIP_KP,grip_kd=GRIP_KD,
        side=side,x=x,y=y,radius=radius,mass=mass,grip=close.tolist(),object_shape='cylinder',object_constraints=0,object_external_force=False,
        minimum_held_lift_m=min_lift,opposed_contact=opposed,released=released,
        held_joints=held[-1]['joints'],held_contacts=held[-1]['contacts'],prelift=min(rows,key=lambda r:abs(r['time']-(lift_at-.1))),
        final_position=rows[-1]['position'],observations=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side',choices=('left','right'),default='right')
    parser.add_argument('--x',type=float,default=.135)
    parser.add_argument('--y',type=float,default=.04)
    parser.add_argument('--radius',type=float,default=.02)
    parser.add_argument('--mass',type=float,default=.08)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sweep',action='store_true',help='Retain a predeclared palm-space position sweep')
    parser.add_argument('--preview',type=Path)
    parser.add_argument('--grip-sweep',action='store_true')
    parser.add_argument('--opposition-sweep',action='store_true',help='Sweep thumb opposition and fingertip closure for sustained holding')
    parser.add_argument('--release-sweep',action='store_true',help='Retain all nine release gap/rate combinations')
    parser.add_argument('--support',choices=('plane','box'),default='plane')
    parser.add_argument('--release-from-measured',action='store_true')
    parser.add_argument('--grasp-height-offset',type=float,default=0.)
    parser.add_argument('--turns',action='store_true',help='Actuated 0-to-pi-to-zero yaw cycles, 20 s period, during hold')
    parser.add_argument('--wrist-pitch',type=float,default=0.,help='Fixed actuated wrist pitch, radians')
    parser.add_argument('--closure-wait',type=float,default=1.5,help='Seconds between close request and lift onset')
    parser.add_argument('--lift-retract',type=float,default=0.,help='Backward wrist travel during lift, 0..0.12 m')
    parser.add_argument('--pitch-about-tool',action='store_true',help='Preserve the calibrated tool point with actuated translations during wrist rotation')
    parser.add_argument('--wrist-roll',type=float,default=0.)
    parser.add_argument('--carry-roll',type=float)
    parser.add_argument('--release-rate',type=float,default=1.5)
    parser.add_argument('--release-gap',type=float,default=0.)
    parser.add_argument('--fixture-output',type=Path)
    parser.add_argument('--gif',type=Path)
    parser.add_argument('--hold-seconds',type=float,default=1.)
    parser.add_argument('--sway-m',type=float,default=0.)
    args = parser.parse_args()
    if (args.turns or args.wrist_pitch or args.wrist_roll or args.carry_roll is not None or args.pitch_about_tool or args.closure_wait!=1.5 or args.lift_retract or args.release_rate!=1.5 or args.release_gap) and (args.sweep or args.grip_sweep or args.opposition_sweep or args.release_sweep or args.fixture_output):
        parser.error('rotational calibration requires a single declared case')
    if args.grasp_height_offset and (args.sweep or args.grip_sweep or args.opposition_sweep or args.release_sweep or args.gif):
        parser.error('height-offset calibration requires a single case without historical GIF capture')
    if args.output.exists():
        parser.error('output exists')
    if args.preview is not None and args.preview.exists():
        parser.error('preview output exists')
    if args.gif is not None and (args.sweep or args.grip_sweep or args.opposition_sweep or args.release_sweep):
        parser.error('GIF recording requires a single declared case')
    if args.gif is not None and (args.gif.exists() or args.gif.with_suffix('.json').exists()):
        parser.error('GIF output exists')
    if args.fixture_output is not None:
        if args.fixture_output.exists():
            parser.error('fixture output exists')
        args.fixture_output.parent.mkdir(parents=True,exist_ok=True)
        args.fixture_output.write_text(fixture(args.side,args.x,args.y,args.radius,args.mass,True,support=args.support,grasp_height_offset=args.grasp_height_offset))
    if args.release_sweep:
        results = []
        for gap in (0.,.015,.03):
            for rate in (.3,.75,1.5):
                result = run(args.side,args.x,args.y,args.radius,args.mass,hold_seconds=args.hold_seconds,
                             sway_m=args.sway_m,release_gap=gap,release_rate=rate,support=args.support,
                             release_from_measured=args.release_from_measured)
                results.append(result)
                print(json.dumps({k:v for k,v in result.items() if k!='observations'}),flush=True)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(scope='release calibration; all9 conditions retained',attempts=results),indent=2)+'\n')
        return int(not all(r['success'] for r in results))
    if args.opposition_sweep:
        results = []
        for opposition in (-.15,-.05,.05,.15):
            for tip in (-.5,-.7,-1.):
                result = run(args.side,args.x,args.y,args.radius,args.mass,
                             grip=[opposition,-1.,tip,1.5,1.5,1.5,1.5],
                             hold_seconds=args.hold_seconds,sway_m=args.sway_m,support=args.support,
                             release_from_measured=args.release_from_measured)
                results.append(result)
                print(json.dumps({k:v for k,v in result.items() if k!='observations'}),flush=True)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(scope='opposition calibration; all12 conditions retained',attempts=results),indent=2)+'\n')
        return int(not any(r['success'] for r in results))
    if args.grip_sweep:
        results = []
        for thumb in (-.5,-.75,-1.):
            for tip in (-.7,-1.,-1.3):
                for fingers in (.8,1.2,1.5):
                    result = run(args.side,args.x,args.y,args.radius,args.mass,grip=[0.,thumb,tip,fingers,fingers,fingers,fingers],
                                 hold_seconds=args.hold_seconds,sway_m=args.sway_m,support=args.support,
                                 release_from_measured=args.release_from_measured)
                    results.append(result)
                    print(json.dumps({k:v for k,v in result.items() if k!='observations'}),flush=True)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(scope='closure calibration; all27 conditions retained',attempts=results),indent=2)+'\n')
        return int(not any(r['success'] for r in results))
    if args.sweep:
        results = []
        for x in (.105,.12,.135,.15):
            for y in (.025,.04,.055,.07):
                result = run(args.side,x,y,args.radius,args.mass,hold_seconds=args.hold_seconds,sway_m=args.sway_m,
                             support=args.support,release_from_measured=args.release_from_measured)
                results.append(result)
                print(json.dumps({k:v for k,v in result.items() if k!='observations'}),flush=True)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(scope='position calibration; all16 conditions retained',attempts=results),indent=2)+'\n')
        return int(not any(r['success'] for r in results))
    result = run(args.side,args.x,args.y,args.radius,args.mass,args.preview,gif=args.gif,
                 hold_seconds=args.hold_seconds,sway_m=args.sway_m,support=args.support,
                 release_from_measured=args.release_from_measured,grasp_height_offset=args.grasp_height_offset,
                 turns=args.turns,wrist_pitch=args.wrist_pitch,closure_wait=args.closure_wait,lift_retract=args.lift_retract,
                 pitch_about_tool=args.pitch_about_tool,wrist_roll=args.wrist_roll,carry_roll=args.carry_roll,
                 release_rate=args.release_rate,release_gap=args.release_gap)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    if args.gif and result['success']:
        metadata = dict(kind='dex3_contact_bench',simulation_start_s=0.,simulation_end_s=10.,
            frame_count=101,frame_period_s=.1,playback_duration_ms=11000,final_frame_hold_ms=1000,
            physical_grasp={k:v for k,v in result.items() if k!='observations'},
            result_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
            recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            gif_sha256=hashlib.sha256(args.gif.read_bytes()).hexdigest())
        args.gif.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='observations'}))
    return int(not result['success'])


if __name__=='__main__':
    raise SystemExit(main())
