#!/usr/bin/env python3
"""Free-base Dex3 arm-extension diagnostic, with optional near-table exposure.

This synchronous native probe is not mobile delivery qualification. Optional
heading feedback uses the IMU; root positions are used only for offline scoring.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnxruntime as ort

from native_probe_feedback import joint_sensor_addresses,joint_feedback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics
from unitree_g1_mujoco_bridge.dex3_grasp import joint_names,GRIP_KP,GRIP_KD
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD
from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES
from unitree_g1_mujoco_bridge.policy_safety import clamp_joint_targets
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
from unitree_g1_mujoco_bridge.manipulation_heading import heading_hold_reference,HEADING_KP,HEADING_KD
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.policy_safety import filter_command


def yaw_of(quat):
    w,x,y,z = quat
    return float(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def heading_command(reference,yaw,rate):
    return heading_hold_reference(reference-yaw,rate)


def hold_metrics(rows):
    """Score a whole observed hold window, including its settling motion."""
    xy = np.asarray([row['xy'] for row in rows])
    yaw = np.unwrap([row['yaw'] for row in rows])
    return dict(duration_s=rows[-1]['time']-rows[0]['time'],
        translation_m=float(np.max(np.linalg.norm(xy-xy[0],axis=1))),
        yaw_excursion_rad=float(np.max(np.abs(yaw-yaw[0]))),
        max_tilt_rad=max(row['tilt'] for row in rows),
        max_hand_drift_m=float(np.max(np.linalg.norm(np.asarray([row['hand_world'] for row in rows])-rows[0]['hand_world'],axis=1))))


def run(policy,mode,stabilization,initial_yaw=0.,preamble=False,filtered_heading=False,staged=False,extension_x=.42,extension_joints=None,initial_x=0.,initial_carry=False,extension_z=.20):
    scene = ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.neq or model.nu!=29 or abs(model.opt.timestep-.002)>1e-10:
        raise ValueError('expected unpinned contact-only Dex3 model, 29 actuators, 2 ms')
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'transport_unitree_rl_lab_stand')
    if key<0:
        raise ValueError('native stand keyframe missing')
    mujoco.mj_resetDataKeyframe(model,data,key)
    data.qpos[0] = initial_x
    data.qpos[3:7] = [np.cos(initial_yaw/2),0.,0.,np.sin(initial_yaw/2)]
    mujoco.mj_forward(model,data)
    ids = np.array([model.joint(name).id for name in G1_29DOF_JOINTS])
    qids,dofs = model.jnt_qposadr[ids],model.jnt_dofadr[ids]
    if initial_carry:
        data.qpos[qids[22:]] = PAYLOAD_V1_POSTURES[0]
        mujoco.mj_forward(model,data)
    aids = np.array([np.flatnonzero(model.actuator_trnid[:,0]==joint)[0] for joint in ids])
    addresses = joint_sensor_addresses(model,G1_29DOF_JOINTS)
    finger_ids = np.array([model.joint(name).id for side in ('left','right') for name in joint_names(side)])
    finger_q,finger_dof = model.jnt_qposadr[finger_ids],model.jnt_dofadr[finger_ids]
    finger_limits = np.tile([2.45,1.4,1.4,1.4,1.4,1.4,1.4],2)
    kp,kd = np.array(G1_UNITREE_RL_LAB_BODY_KP),np.array(G1_UNITREE_RL_LAB_BODY_KD)
    kin = ArmKinematics((ROOT/'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_dex3.urdf').read_text(),tool_offset=(.135,.04,0.))
    if extension_joints is not None and (np.any(np.asarray(extension_joints)<kin.limits[:,0]) or
                                        np.any(np.asarray(extension_joints)>kin.limits[:,1])):
        raise ValueError('prescribed extension exceeds robot joint limits')
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(policy),sess_options=options,providers=['CPUExecutionProvider'])
    adapter = UnitreeRLLabG1PolicyAdapter()
    action = np.zeros(29)
    target = np.asarray(adapter.sdk_default_positions)
    phase,phase_start = 'actor_stand',0.
    rows,events = [],[]
    arm_start = arm_goal = None
    authority = False
    heading = initial_yaw
    waist_yaw = 0.
    waist_index = G1_29DOF_JOINTS.index('waist_yaw_joint')
    trajectory_duration = 0.
    fallen = False
    planning_error = None
    root = model.body('pelvis').id
    wrist = model.body('right_wrist_yaw_link').id
    external_contacts = set()
    contact_details = {}
    arm_start_time = 15. if preamble else 5.
    filtered = [0.,0.,0.]
    envelope = MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6)
    if initial_carry:
        phase,authority = 'carry_hold',True
        arm_start = data.qpos[qids[15:]].copy()
        arm_goal = arm_start.copy()
        trajectory_duration = 1.
        kp[15:],kd[15:] = 160.,4.
        events.append(dict(phase=phase,time=0.,initial_carry_fixture=True))
    # Transitions depend on elapsed simulation time, never root/object truth.
    for step in range(25000):  # Hard 50-second diagnostic budget.
        q,dq = joint_feedback(data,qids,dofs,addresses,'sensors')
        if step%10==0:
            t = step*.002
            quat,gyro = data.sensor('imu_quat').data.copy(),data.sensor('imu_gyro').data.copy()
            yaw = yaw_of(quat)
            if phase=='actor_stand' and t>=arm_start_time:
                heading = yaw
                waist_yaw = float(q[waist_index])
                authority = mode!='free'
                arm_start = q[15:].copy()
                arm_goal = arm_start.copy()
                arm_goal[7:] = PAYLOAD_V1_POSTURES[0]
                trajectory_duration = max(2.,float(np.max(np.abs(arm_goal-arm_start)))/.35)
                phase,phase_start = ('carry_transition' if authority else 'free_hold'),t
                if authority:
                    kp[15:],kd[15:] = 160.,4.
                events.append(dict(phase=phase,time=t))
            elif phase=='carry_transition' and t-phase_start>=trajectory_duration:
                phase,phase_start = 'carry_hold',t
                events.append(dict(phase=phase,time=t))
            elif ((phase=='carry_hold' and t-phase_start>=5. and mode=='extend') or
                  (phase=='stage_hold' and t-phase_start>=3.)):
                joints = dict(zip(G1_29DOF_JOINTS,q))
                staging = staged and phase=='carry_hold'
                solution,error = kin.solve(np.array([.28 if staging else extension_x,-.18,extension_z]),joints,np.eye(3))
                if not staging and extension_joints is not None:
                    solution,error = dict(zip(kin.names,extension_joints)),0.
                if error>.012:
                    planning_error = f'extension IK error {error}'
                    break
                arm_start = q[15:].copy()
                arm_goal = arm_start.copy()
                arm_goal[7:] = [solution[name] for name in kin.names]
                trajectory_duration = max(2.,float(np.max(np.abs(arm_goal-arm_start)))/.35)
                phase,phase_start = ('stage_transition' if staging else 'extension_transition'),t
                events.append(dict(phase=phase,time=t,ik_error_m=error,joint_goal=arm_goal.tolist()))
            elif phase=='stage_transition' and t-phase_start>=trajectory_duration:
                phase,phase_start = 'stage_hold',t
                events.append(dict(phase=phase,time=t))
            elif phase=='extension_transition' and t-phase_start>=trajectory_duration:
                phase,phase_start = 'extension_hold',t
                events.append(dict(phase=phase,time=t))
            if ((phase in ('free_hold','extension_hold') and t-phase_start>=10.) or
                    (phase=='carry_hold' and mode=='carry' and t-phase_start>=10.)):
                break
            command = [0.,0.,heading_command(heading,yaw,float(gyro[2])) if authority and stabilization=='heading' else 0.]
            if preamble and not authority:
                if 5.<=t<7.:
                    command = [0.,0.,.6]
                elif 7.<=t<10.:
                    command = [.45,0.,0.]
            if filtered_heading and authority:
                requested = envelope.normalize(command,stopped=max(abs(v) for v in filtered)<1e-6)
                filtered = filter_command(filtered,requested,(.8,.8,.7),.02,False)
                command = list(filtered)
            observation = adapter.build_observation(command,q,dq,gyro,quat,action)
            action = session.run(None,{session.get_inputs()[0].name:observation})[0].reshape(-1)
            target = np.asarray(clamp_joint_targets(adapter.action_targets(action)))
            if authority:
                blend = min(1.,(t-phase_start)/trajectory_duration) if phase.endswith('transition') else 1.
                target[15:] = arm_start+(arm_goal-arm_start)*blend
                if stabilization=='waist_yaw':
                    target[waist_index] = waist_yaw
            tilt = float(np.arccos(np.clip(1-2*(quat[1]**2+quat[2]**2),-1.,1.)))
            hand = data.xpos[wrist]+data.xmat[wrist].reshape(3,3)@kin.tool_offset
            rows.append(dict(time=t,phase=phase,xy=data.qpos[:2].tolist(),height=float(data.qpos[2]),
                yaw=yaw,tilt=tilt,yaw_rate=float(gyro[2]),command=command,hand_world=hand.tolist(),
                arm_joints=q[15:].tolist(),waist_joints=q[12:15].tolist()))
            for contact_index,contact in enumerate(data.contact):
                a,b = contact.geom1,contact.geom2
                ar,br = model.body_rootid[model.geom_bodyid[a]]==root,model.body_rootid[model.geom_bodyid[b]]==root
                if ar!=br:
                    other = b if ar else a
                    if model.geom_type[other]!=mujoco.mjtGeom.mjGEOM_PLANE:
                        force = np.zeros(6)
                        mujoco.mj_contactForce(model,data,contact_index,force)
                        if force[0]>.05:
                            name = model.geom(other).name or str(other)
                            external_contacts.add(name)
                            robot_geom = a if ar else b
                            pair = phase+':'+model.body(model.geom_bodyid[robot_geom]).name+':'+name
                            item = contact_details.setdefault(pair,dict(samples=0,max_force_n=0.,min_distance_m=0.))
                            item['samples'] += 1
                            item['max_force_n'] = max(item['max_force_n'],float(force[0]))
                            item['min_distance_m'] = min(item['min_distance_m'],float(contact.dist))
            if tilt>.6 or data.qpos[2]<.5:
                fallen = True
                break
        data.ctrl[aids] = kp*(target-q)-kd*dq
        data.qfrc_applied[finger_dof] = np.clip(-GRIP_KP*data.qpos[finger_q]-GRIP_KD*data.qvel[finger_dof],-finger_limits,finger_limits)
        mujoco.mj_step(model,data)
    holds = {name:hold_metrics([r for r in rows if r['phase']==name]) for name in
             ('free_hold','carry_hold','extension_hold') if any(r['phase']==name for r in rows)}
    expected = 'extension_hold' if mode=='extend' else mode+'_hold'
    complete = expected in holds and holds[expected]['duration_s']>=9.9
    return dict(scope='synchronous free-base Dex3 arm stability diagnostic; not delivery qualification',
        mode=mode,stabilization=stabilization,initial_yaw_rad=initial_yaw,
        initial_robot_x_m=initial_x,contact_details=contact_details,
        initial_carry=initial_carry,extension_z_m=extension_z,
        maneuver_preamble=preamble,
        filtered_heading=filtered_heading,
        heading_gains=[HEADING_KP,HEADING_KD],
        staged=staged,extension_x_m=extension_x,
        prescribed_extension_joints=extension_joints,
        policy=str(policy),policy_sha256=hashlib.sha256(policy.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in (
            ROOT/'scripts/native_probe_feedback.py',
            ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/manipulation_heading.py',
            ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/arm_kinematics.py',
            ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/unitree_rl_lab_policy.py',
            ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/g1_constants.py')},
        model_dynamics_sha256=hashlib.sha256(b''.join(getattr(model,key).tobytes() for key in
            ('body_mass','body_inertia','geom_type','geom_size','geom_friction','jnt_range','dof_damping','dof_armature','mesh_vert'))).hexdigest(),
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),mujoco_version=mujoco.__version__,
        equality_constraints=model.neq,object_forces=False,root_pose_control=False,
        action_feedback='published_actor_action',joint_and_pd_feedback='native_sensors',
        completed=complete,fallen=fallen,planning_error=planning_error,external_contacts=sorted(external_contacts),
        holds=holds,phase_metrics={name:hold_metrics([r for r in rows if r['phase']==name])
            for name in sorted({r['phase'] for r in rows})},events=events,observations=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=('free','carry','extend'),default='extend')
    parser.add_argument('--stabilization',choices=('none','heading','waist_yaw'),default='none')
    parser.add_argument('--initial-yaw',type=float,default=0.)
    parser.add_argument('--preamble',action='store_true',help='Prescribed turn/walk, then five seconds stopped before arm authority')
    parser.add_argument('--filtered-heading',action='store_true',help='Apply 0.05/0.01 deadband/hysteresis and 0.7 rad/s² yaw slew')
    parser.add_argument('--staged',action='store_true',help='Retract/lower to x=0.28 m before final extension')
    parser.add_argument('--extension-x',type=float,default=.42)
    parser.add_argument('--extension-joints',type=float,nargs=7,help='Replay a recorded seven-joint right-arm target instead of solving the extension endpoint')
    parser.add_argument('--initial-x',type=float,default=0.,help='Offline fixture initial x, e.g. 1.62 for near-table exposure')
    parser.add_argument('--initial-carry',action='store_true',help='Offline carry-start fixture; avoids raising a hanging arm beside the table')
    parser.add_argument('--extension-z',type=float,default=.20)
    args = parser.parse_args()
    if args.output.exists() or not np.isfinite(args.initial_yaw) or not np.isfinite(args.initial_x) or not .3<=args.extension_x<=.5:
        parser.error('requires new output and finite initial yaw')
    if mujoco.__version__!='3.3.6':
        parser.error('requires native MuJoCo 3.3.6')
    if args.extension_joints is not None and not all(np.isfinite(args.extension_joints)):
        parser.error('extension joints must be finite')
    if not .15<=args.extension_z<=.4 or (args.initial_carry and (args.preamble or args.mode=='free')):
        parser.error('invalid extension height or incompatible carry-start fixture')
    result = run(args.policy,args.mode,args.stabilization,args.initial_yaw,args.preamble,args.filtered_heading,args.staged,args.extension_x,args.extension_joints,args.initial_x,args.initial_carry,args.extension_z)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='observations'},indent=2))
    return int(not result['completed'] or result['fallen'] or bool(result['external_contacts']))


if __name__=='__main__':
    raise SystemExit(main())
