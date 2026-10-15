#!/usr/bin/env python3
"""Isolate ONNX deployment on native MuJoCo geometry, without ROS scheduling.

This is a dynamics diagnostic, not a sensor-only mission acceptance run.
"""
import argparse
from collections import Counter,deque
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnxruntime as ort
from policy_command_replay import load_commands,command_at
from native_probe_payload import attach_payload,payload_sample

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
from unitree_g1_mujoco_bridge.policy_safety import clamp_joint_targets
from unitree_g1_mujoco_bridge.policy_safety import filter_command
from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--delay-steps',type=int,default=0,help='Application delay in observation intervals (20 ms each by default)')
    parser.add_argument('--action-feedback',choices=('applied','published'),default='applied',
                        help='Previous-action observation: legacy motor-applied value or deployment publisher value')
    parser.add_argument('--yaw',type=float,default=0.,help='Yaw command after five seconds standing')
    parser.add_argument('--linear-x',type=float,default=0.)
    parser.add_argument('--linear-y',type=float,default=0.)
    parser.add_argument('--command-duration',type=float,help='Stop after this many seconds of command; standing starts at t=0, command at t=5')
    parser.add_argument('--held-empty',action='store_true',help='Externally compose payload_v1 arms, without an attached load')
    parser.add_argument('--payload',action='store_true',help='Offline loaded-start fixture: native red object welded to right wrist; compose carry arms')
    parser.add_argument('--record-actions',action='store_true',help='Retain published/applied action vectors and publication/application sequence IDs')
    parser.add_argument('--ramp',action='store_true',help='Apply nominal acceleration-limited start/stop commands for transition comparison')
    parser.add_argument('--outage-at',type=float,help='Inject one inference outage at this simulation time')
    parser.add_argument('--outage-duration',type=float,default=.06)
    parser.add_argument('--outage-reset-history',action='store_true',help='Model history/action reset on inference timeout')
    parser.add_argument('--outage-measured-hold',action='store_true',help='Hold measured joints rather than the last validated target during the outage')
    parser.add_argument('--observation-steps',type=int,nargs='+',default=[10],
                        help='Repeating observation intervals in 2 ms physics steps (default 10 = 50 Hz)')
    parser.add_argument('--sensor-delay-steps',type=int,default=0,
                        help='Joint/IMU measurement delay in 2 ms physics steps, separate from action delay')
    parser.add_argument('--require-mujoco-version',help='Refuse calibration with a different engine version')
    parser.add_argument('--scene',type=Path,help='Native fixture to use instead of transport_dual')
    parser.add_argument('--replay-trace',type=Path,help='Replay empty-hand filtered command samples after 5 s standing; actors parked')
    parser.add_argument('--replay-start',type=float,default=0.,help='Seconds after the first moving command to begin command replay')
    args = parser.parse_args()
    if args.payload and args.held_empty:
        parser.error('--payload and --held-empty are mutually exclusive')
    compose_arms = args.held_empty or args.payload
    if args.output.exists():
        parser.error('output already exists; retain each probe separately')
    if not np.isfinite(args.replay_start) or args.replay_start<0. or (args.replay_start and not args.replay_trace):
        parser.error('nonnegative finite replay start requires a replay trace')
    if args.sensor_delay_steps<0:
        parser.error('sensor delay must be nonnegative')
    if any(step<=0 for step in args.observation_steps):
        parser.error('observation intervals must be positive')
    if args.outage_at is not None and (not np.isfinite(args.outage_at) or not 0.<=args.outage_at<args.duration or
            not np.isfinite(args.outage_duration) or args.outage_duration<=0.):
        parser.error('finite in-range outage time and positive outage duration required')
    if args.outage_at is None and (args.outage_reset_history or args.outage_measured_hold):
        parser.error('outage recovery options require --outage-at')
    if args.require_mujoco_version and mujoco.__version__!=args.require_mujoco_version:
        parser.error(f'expected MuJoCo {args.require_mujoco_version}, got {mujoco.__version__}')
    if args.delay_steps<0 or args.duration<=0:
        parser.error('nonnegative delay and positive duration required')
    if not np.all(np.isfinite((args.duration,args.linear_x,args.linear_y,args.yaw))) or (args.command_duration is not None and
            (not np.isfinite(args.command_duration) or args.command_duration<=0.)):
        parser.error('finite commands/duration and positive command duration required')
    command_end = 5.+args.command_duration if args.command_duration is not None else args.duration
    replay = None
    if args.replay_trace:
        if compose_arms or args.linear_x or args.linear_y or args.yaw or args.command_duration is not None or args.ramp:
            parser.error('replay cannot be combined with fixed commands or arm composition')
        try:
            replay = load_commands(args.replay_trace)
            if args.replay_start>replay[0][-1]:
                parser.error('replay start is beyond the recorded command timeline')
        except ValueError as exc:
            parser.error(str(exc))
    scene = args.scene or ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = .002
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'transport_unitree_rl_lab_stand')
    if key<0:
        raise ValueError('native stand keyframe missing')
    mujoco.mj_resetDataKeyframe(model,data,key)
    if replay is not None:
        for body in range(model.nbody):
            name = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,body) or ''
            if name.startswith('ped') and model.body_mocapid[body]>=0:
                data.mocap_pos[model.body_mocapid[body]] = (0.,0.,-50.)
    mujoco.mj_forward(model,data)
    payload = attach_payload(model,data) if args.payload else None
    joint_ids = np.array([model.joint(name).id for name in G1_29DOF_JOINTS])
    qids,dids = model.jnt_qposadr[joint_ids],model.jnt_dofadr[joint_ids]
    aids = np.array([np.flatnonzero(model.actuator_trnid[:,0]==index)[0] for index in joint_ids])
    kp,kd = np.asarray(G1_UNITREE_RL_LAB_BODY_KP),np.asarray(G1_UNITREE_RL_LAB_BODY_KD)
    kp,kd = kp.copy(),kd.copy()
    if compose_arms:
        kp[15:],kd[15:] = 160.,4.
    adapter = UnitreeRLLabG1PolicyAdapter()
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.policy),sess_options=options,providers=['CPUExecutionProvider'])
    target = np.asarray(adapter.sdk_default_positions)
    arm_target = target.copy()
    arm_goal = target.copy()
    arm_goal[22:] = PAYLOAD_V1_POSTURES[0]
    action = np.zeros(29)
    published_action = action.copy()
    queue = deque()
    rows = []
    external_contacts = Counter()
    payload_contacts = Counter()
    payload_robot_contacts = Counter()
    published_sequence = applied_sequence = -1
    robot_root = model.body('pelvis').id
    start = data.qpos[:2].copy()
    fallen = False
    filtered = [0.,0.,0.]
    outage_started = False
    next_observation = observation_index = 0
    last_observation = -.02
    measurements = deque(maxlen=args.sensor_delay_steps+1)
    for step in range(round(args.duration/.002)):
        measurements.append((data.qpos[qids].copy(),data.qvel[dids].copy(),
                             data.sensor('imu_gyro').data.copy(),data.sensor('imu_quat').data.copy()))
        if step>=next_observation:
            # Offline diagnostics only: distinguish a gait failure from a
            # collision with this probe's unchanged scene geometry.
            for contact in data.contact:
                a,b = int(contact.geom1),int(contact.geom2)
                robot_a = model.body_rootid[model.geom_bodyid[a]]==robot_root
                robot_b = model.body_rootid[model.geom_bodyid[b]]==robot_root
                if payload is not None:
                    payload_a = model.geom_bodyid[a]==payload['body']
                    payload_b = model.geom_bodyid[b]==payload['body']
                    if payload_a != payload_b:
                        other = b if payload_a else a
                        counter = payload_robot_contacts if (robot_b if payload_a else robot_a) else payload_contacts
                        counter[model.geom(other).name or f'geom_{other}'] += 1
                if robot_a != robot_b:
                    other = b if robot_a else a
                    if model.geom_type[other]!=mujoco.mjtGeom.mjGEOM_PLANE:
                        external_contacts[model.geom(other).name or f'geom_{other}'] += 1
            observation_dt = data.time-last_observation
            last_observation = data.time
            next_observation += args.observation_steps[observation_index%len(args.observation_steps)]
            observation_index += 1
            command = [args.linear_x,args.linear_y,args.yaw] if 5.<=data.time<command_end else [0.,0.,0.]
            if replay is not None:
                command = list(command_at(*replay,data.time-5.+args.replay_start)) if data.time>=5. else [0.,0.,0.]
            if args.ramp:
                filtered = filter_command(filtered,command,(1.,.8,.9),observation_dt,False)
                command = filtered
            outage = args.outage_at is not None and args.outage_at<=data.time<args.outage_at+args.outage_duration
            if outage:
                if not outage_started:
                    outage_started = True
                    queue.clear()  # No pre-timeout inference can be applied later.
                    if args.outage_reset_history:
                        adapter.reset()
                        action = np.zeros(29)
                        published_action = action.copy()
                if args.outage_measured_hold:
                    target = data.qpos[qids].copy()
            else:
                feedback = published_action if args.action_feedback=='published' else action
                observation = adapter.build_observation(command,*measurements[0],feedback)
                proposed = session.run(None,{session.get_inputs()[0].name:observation})[0].reshape(-1)
                published_action = proposed.copy()
                published_sequence += 1
                queue.append((published_sequence,proposed))
                if len(queue)>args.delay_steps:
                    applied_sequence,action = queue.popleft()
                    target = np.asarray(clamp_joint_targets(adapter.action_targets(action)))
            if compose_arms:
                arm_target += np.clip(arm_goal-arm_target,-.35*observation_dt,.35*observation_dt)
                target[15:] = arm_target[15:]
            quat = data.sensor('imu_quat').data
            tilt = float(np.arccos(np.clip(1.-2.*(quat[1]**2+quat[2]**2),-1.,1.)))
            yaw = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1.-2*(quat[2]**2+quat[3]**2)))
            c,s = np.cos(yaw),np.sin(yaw)
            rows.append(dict(time=float(data.time),xy=data.qpos[:2].tolist(),height=float(data.qpos[2]),
                             tilt=tilt,yaw=yaw,body_velocity=[float(c*data.qvel[0]+s*data.qvel[1]),float(-s*data.qvel[0]+c*data.qvel[1])],
                              yaw_rate=float(data.sensor('imu_gyro').data[2]),
                                command=command,inference_outage=outage,action_norm=float(np.linalg.norm(action))))
            if payload is not None:
                rows[-1].update(payload_sample(model,data,payload))
            if args.record_actions:
                rows[-1].update(published_action=published_action.tolist(),applied_action=action.tolist(),
                                published_sequence=published_sequence,applied_sequence=applied_sequence,
                                composed_joint_target=target.tolist())
            if tilt>.9 or data.qpos[2]<.45:
                fallen = True
                break
        data.ctrl[aids] = kp*(target-data.qpos[qids])-kd*data.qvel[dids]
        mujoco.mj_step(model,data)
    steady = [row['body_velocity'] for row in rows if 5.5<=row['time']<command_end] if replay is None else []
    stopped = [row for row in rows if row['time']>=command_end] if replay is None else []
    turning = [row['yaw_rate'] for row in rows if 6.<=row['time']<command_end] if replay is None else []
    stop_travel = [float(np.linalg.norm(np.asarray(row['xy'])-stopped[0]['xy'])) for row in stopped] if stopped else []
    result = dict(policy=str(args.policy),policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
                  scene=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                    mujoco_version=mujoco.__version__,ramp=args.ramp,scope='native geometry, synchronous CPU MuJoCo; no ROS or camera; optional prescribed carry arms and offline loaded-start magnetic fixture; not pickup qualification',
                    payload=args.payload,payload_mass_kg=payload['mass_kg'] if payload else None,
                    sampled_payload_scene_contacts=dict(payload_contacts),
                    sampled_payload_robot_contacts=dict(payload_robot_contacts),
                    record_actions=args.record_actions,probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    payload_fixture_sha256=hashlib.sha256((ROOT/'scripts/native_probe_payload.py').read_bytes()).hexdigest(),
                   delay_steps=args.delay_steps,yaw=args.yaw,held_empty=args.held_empty,requested_duration_s=args.duration,
                   outage_at_s=args.outage_at,outage_duration_s=args.outage_duration if args.outage_at is not None else None,
                   outage_reset_history=args.outage_reset_history,outage_measured_hold=args.outage_measured_hold,
                   outage_injected=outage_started,
                   observation_steps=args.observation_steps,
                   action_feedback=args.action_feedback,
                   sensor_delay_steps=args.sensor_delay_steps,sensor_delay_s=args.sensor_delay_steps*.002,
                   sampled_nonfloor_robot_contacts=dict(external_contacts),
                   requested_command=[args.linear_x,args.linear_y,args.yaw],command_duration_s=args.command_duration,
                   response_samples=len(steady),stop_samples=len(stopped),
                    mean_body_velocity_after_half_second=np.mean(steady,axis=0).tolist() if steady else None,
                    mean_yaw_rate_after_one_second=float(np.mean(turning)) if turning else None,
                    yaw_rate_std_after_one_second=float(np.std(turning)) if turning else None,
                   max_displacement_after_stop_m=max(stop_travel) if stop_travel else None,
                  yaw_displacement_rad=float(np.unwrap([row['yaw'] for row in rows])[-1]-rows[0]['yaw']),
                  elapsed_s=float(data.time),fallen=fallen,displacement_m=float(np.linalg.norm(data.qpos[:2]-start)),
                   max_tilt_rad=max(row['tilt'] for row in rows),samples=rows)
    if replay is not None:
        result.update(replay_trace=str(args.replay_trace),replay_trace_sha256=hashlib.sha256(args.replay_trace.read_bytes()).hexdigest(),
                       replay_samples=len(replay[0]),replay_duration_s=replay[0][-1],requested_command=None,
                       replay_start_s=args.replay_start,
                       replay_scope='filtered command timeline only; neutral initial state, configured cadence and delay, no ROS or moving actors; not an exact incident replay')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))


if __name__=='__main__':
    main()
