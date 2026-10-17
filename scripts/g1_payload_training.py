#!/usr/bin/env python3
"""Fine-tune the deployed 480/29 G1 actor for externally controlled arms/load.

Run through scripts/run_mjlab_docker.sh (uv), with MJLAB_WITH_GPUS=1.
Training truth is confined to rewards/critic/evaluation. Actor inputs retain the
deployed IMU, joint encoders, command and action-history contract.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnx
from onnx import numpy_helper
import torch
import warp as wp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import (
    G1_29DOF_JOINTS, G1_UNITREE_RL_LAB_BODY_KP, G1_UNITREE_RL_LAB_BODY_KD,
)
from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import (
    UNITREE_RL_LAB_G1_JOINT_IDS_MAP, UNITREE_RL_LAB_G1_DEFAULT_POS,
)
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics
from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.asset_zoo.robots.unitree_g1.g1_constants import get_spec
from mjlab.envs import ManagerBasedRlEnv
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg, RecomputeLevel, requires_model_fields
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.config.g1.env_cfgs import unitree_g1_flat_env_cfg
from mjlab.tasks.velocity.config.g1.rl_cfg import unitree_g1_ppo_runner_cfg
from mjlab.tasks.velocity.mdp import StandCreepStopVelocityCommandCfg
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
from mjlab.utils.os import dump_yaml
from payload_smoothness import FluidCarryPenalty,PayloadMotion,LOADED_TURN_SCHEDULE
from payload_sensor_delay import DelayedSensorTerm
from payload_transition_commands import TransitionVelocityCommandCfg

POLICY_NAMES = tuple(G1_29DOF_JOINTS[i] for i in UNITREE_RL_LAB_G1_JOINT_IDS_MAP)
ARM_POLICY_IDS = tuple(i for i, sdk in enumerate(UNITREE_RL_LAB_G1_JOINT_IDS_MAP) if sdk >= 15)
SOURCE = ROOT / 'vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx'
URDF = ROOT / 'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf'
SIGNED_STOP_SCHEDULE = (('settle',5.,(0.,0.,0.)),)+tuple(
    phase for direction in (1.,-1.) for duration in (2.,3.) for phase in (
        (f'lateral_{direction:+g}_{duration:g}',duration,(0.,direction*.45,0.)),
        (f'stop_{direction:+g}_{duration:g}',6.,(0.,0.,0.))))


class ConsoleTee:
    def __init__(self,terminal,path):
        self.terminal = terminal
        self.stream = path.open('w',buffering=1)

    def write(self,text):
        self.stream.write(text)
        return self.terminal.write(text)

    def flush(self):
        self.stream.flush()
        self.terminal.flush()

    def __getattr__(self,name):
        return getattr(self.terminal,name)


def arm_pose_bank(include_reaches=False):
    poses = [list(pose) for pose in PAYLOAD_V1_POSTURES]
    if not include_reaches:
        return poses
    kin = ArmKinematics(URDF.read_text(), tool_offset=(.12,.01,0.))
    rest = dict(zip(POLICY_NAMES, UNITREE_RL_LAB_G1_DEFAULT_POS))
    targets = [(.30,-.13,.36)] + [(x,y,z) for x in (.30,.36,.42,.48)
               for y in (-.08,-.15,-.22) for z in (.20,.28,.36,.40)]
    for target in targets:
        solution,error = kin.solve(target, rest, np.eye(3))
        if error < .012:
            poses.append([solution[name] for name in G1_29DOF_JOINTS[22:]])
    return poses


def cup_up_axis():
    """Virtual cup mounting: upright at the fixed cycle-01 carry posture."""
    kin = ArmKinematics(URDF.read_text(),tool_offset=(.12,.01,0.))
    joints = dict(zip(POLICY_NAMES,UNITREE_RL_LAB_G1_DEFAULT_POS))
    joints.update(dict(zip(kin.names,PAYLOAD_V1_POSTURES[0])))
    return tuple(map(float,kin.forward(joints)[1].T@np.array((0.,0.,1.))))


def native_robot_model():
    return mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml'))


def payload_spec(native_inertial=False):
    spec = get_spec()
    if native_inertial:
        native = native_robot_model()
        for body in spec.bodies:
            index = mujoco.mj_name2id(native,mujoco.mjtObj.mjOBJ_BODY,body.name)
            if index<=0:
                continue
            body.mass = float(native.body_mass[index])
            body.inertia = native.body_inertia[index].copy()
            body.ipos = native.body_ipos[index].copy()
            body.iquat = native.body_iquat[index].copy()
            body.pos = native.body_pos[index].copy()
            body.quat = native.body_quat[index].copy()
    for joint in spec.joints:
        if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            joint.damping = [.05, 0., 0.]
            joint.frictionloss = .1 if 'wrist' in joint.name else .2
    payload = spec.body('right_wrist_yaw_link').add_body(name='carried_payload', pos=[.19, .01, 0.])
    payload.add_geom(name='payload_collision', type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[.055, .055, .055], mass=.15, rgba=[.85, .08, .06, 1.])
    return spec


def native_inertial_payload_spec():
    return payload_spec(native_inertial=True)


def native_contact_payload_spec():
    """Also match deployment's four sphere contacts per foot, not seven capsules.

    Other training collision assets still differ; this is a declared foot-contact
    fidelity profile, not a claim that the two simulators are identical.
    """
    spec = native_inertial_payload_spec()
    native = native_robot_model()
    for side in ('left','right'):
        body = native.body(f'{side}_ankle_roll_link').id
        contacts = [index for index in range(native.ngeom) if native.geom_bodyid[index]==body and
                    (native.geom_contype[index] or native.geom_conaffinity[index])]
        for index in range(1,8):
            geom = spec.geom(f'{side}_foot{index}_collision')
            if index>len(contacts):
                spec.delete(geom)
                continue
            source = contacts[index-1]
            geom.type = mujoco.mjtGeom(int(native.geom_type[source]))
            geom.fromto = [math.nan]*6
            for target,name in (('pos','geom_pos'),('quat','geom_quat'),('size','geom_size'),
                                ('friction','geom_friction'),('solref','geom_solref'),('solimp','geom_solimp')):
                setattr(geom,target,getattr(native,name)[source].copy())
            geom.condim = int(native.geom_condim[source])
            geom.priority = int(native.geom_priority[source])
    return spec


def turn_drift_penalty(env):
    command = env.command_manager.get_command('twist')
    pivot = (torch.linalg.vector_norm(command[:,:2],dim=1)<.05)&(command[:,2].abs()>.2)
    velocity = env.scene['robot'].data.root_link_lin_vel_b[:,:2]
    return pivot*torch.sum(velocity*velocity,dim=1)


def stopping_velocity_penalty(env):
    """Critic/reward truth only: residual motion after an explicit zero command."""
    command = env.command_manager.get_command('twist')
    stopped = torch.all(command==0.,dim=1)
    data = env.scene['robot'].data
    return stopped*(torch.sum(data.root_link_lin_vel_b[:,:2]**2,dim=1)+
                    .25*data.root_link_ang_vel_b[:,2]**2)


@dataclass(kw_only=True)
class PayloadActionCfg(JointPositionActionCfg):
    max_payload: float = .5
    profile: str = 'mixed'
    pose_index: int = 0
    pose_bank: str = 'carry'
    arm_transitions: bool = False
    max_delay_steps: int = 0
    action_hold_probability: float = 0.
    max_sensor_delay_steps: int = 0
    action_feedback: str = 'applied'

    def build(self, env):
        return PayloadAction(self, env)


class PayloadAction(JointPositionAction):
    def _find_targets(self, cfg):
        return self._entity.find_joints(POLICY_NAMES, preserve_order=True)

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.held = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.mass = torch.zeros(self.num_envs, device=self.device)
        self.arm_commands = self._offset.expand(self.num_envs, -1).clone()
        self.arm_targets = self.arm_commands.clone()
        self.arm_ids = torch.tensor(ARM_POLICY_IDS, device=self.device)
        self.joint_ids = self._target_ids
        self.postures = torch.tensor(arm_pose_bank(cfg.pose_bank == 'reach'), device=self.device)
        self.next_pose_step = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.delay_steps = torch.zeros(self.num_envs,dtype=torch.long,device=self.device)
        self.sensor_delay_steps = torch.zeros(self.num_envs,dtype=torch.long,device=self.device)
        self.published_action = torch.zeros(self.num_envs,self.action_dim,device=self.device)
        self.action_history = torch.zeros(self.num_envs,cfg.max_delay_steps+1,self.action_dim,device=self.device)
        self.right_ids = torch.tensor([POLICY_NAMES.index(name) for name in G1_29DOF_JOINTS[22:]], device=self.device)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if not hasattr(self, 'held'):
            return
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        self.arm_commands[env_ids] = self._offset.expand(self.num_envs, -1)[env_ids]
        self.action_history[env_ids] = 0.
        self.published_action[env_ids] = 0.

    def sample_episode(self, env_ids):
        count = len(env_ids)
        if self.cfg.max_delay_steps:
            self.delay_steps[env_ids] = torch.randint(self.cfg.max_delay_steps+1,(count,),device=self.device)
        if self.cfg.max_sensor_delay_steps:
            self.sensor_delay_steps[env_ids] = torch.randint(self.cfg.max_sensor_delay_steps+1,(count,),device=self.device)
        random = torch.rand(count, device=self.device)
        if self.cfg.profile == 'mixed':
            self.held[env_ids] = random >= .2
            self.mass[env_ids] = torch.where(random >= .4,
                .05 + torch.rand(count, device=self.device) * (self.cfg.max_payload - .05), 0.)
        else:
            self.held[env_ids] = self.cfg.profile != 'unloaded'
            self.mass[env_ids] = {'unloaded': 0., 'held_empty': 0., 'loaded': .15, 'heavy': .35}[self.cfg.profile]
        self.arm_commands[env_ids] = self._offset.expand(self.num_envs, -1)[env_ids]
        self.arm_targets[env_ids] = self.arm_commands[env_ids]
        poses = self.postures[torch.randint(len(self.postures), (count,), device=self.device)]
        if self.cfg.profile != 'mixed':
            poses = self.postures[self.cfg.pose_index].expand(count, -1)
        noise = (torch.rand_like(poses) - .5) * .12 if self.cfg.profile == 'mixed' else 0.
        self.arm_targets[env_ids[:, None], self.right_ids[None, :]] = poses + noise
        self.next_pose_step[env_ids] = round(8. / self._env.step_dt)

    def process_actions(self, actions):
        self.published_action.copy_(actions)
        self.action_history = torch.roll(self.action_history,1,dims=1)
        self.action_history[:,0] = actions
        delayed = self.action_history[torch.arange(self.num_envs,device=self.device),self.delay_steps]
        if self.cfg.action_hold_probability:
            held = torch.rand(self.num_envs,device=self.device)<self.cfg.action_hold_probability
            delayed = torch.where(held[:,None],self.raw_action,delayed)
        super().process_actions(delayed)
        if self.cfg.profile == 'mixed' and self.cfg.arm_transitions:
            ids = torch.where(self.held & (self._env.episode_length_buf >= self.next_pose_step))[0]
            if len(ids):
                poses = self.postures[torch.randint(len(self.postures), (len(ids),), device=self.device)]
                self.arm_targets[ids[:, None], self.right_ids[None, :]] = poses
                self.next_pose_step[ids] += round(8. / self._env.step_dt)
        self.arm_commands += torch.clamp(self.arm_targets - self.arm_commands,
                                          -.35 * self._env.step_dt, .35 * self._env.step_dt)
        self._processed_actions[:, self.arm_ids] = torch.where(self.held[:, None],
            self.arm_commands[:, self.arm_ids], self._processed_actions[:, self.arm_ids])


@requires_model_fields('body_mass', 'body_inertia', 'actuator_gainprm', 'actuator_biasprm', recompute=RecomputeLevel.set_const)
def reset_payload(env, env_ids):
    action = env.action_manager.get_term('joint_pos')
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    # Reset events run before ActionManager.reset; sample here, not there.
    action.sample_episode(env_ids)
    robot = env.scene['robot']
    local_ids, _ = robot.find_bodies('carried_payload')
    body = robot.indexing.body_ids[local_ids[0]]
    mass = action.mass[env_ids].clamp_min(1e-6)
    env.sim.model.body_mass[env_ids, body] = mass
    env.sim.model.body_inertia[env_ids, body] = mass[:, None] * (.11**2 / 6.)
    for sdk, name in enumerate(G1_29DOF_JOINTS[15:], start=15):
        local = next(i for i, actuator in enumerate(robot.actuator_names) if name in actuator)
        actuator = robot.indexing.ctrl_ids[local]
        kp = torch.where(action.held[env_ids], 160., G1_UNITREE_RL_LAB_BODY_KP[sdk])
        kd = torch.where(action.held[env_ids], 4., G1_UNITREE_RL_LAB_BODY_KD[sdk])
        env.sim.model.actuator_gainprm[env_ids, actuator, 0] = kp
        env.sim.model.actuator_biasprm[env_ids, actuator, 1] = -kp
        env.sim.model.actuator_biasprm[env_ids, actuator, 2] = -kd


def joint_positions(env):
    action = env.action_manager.get_term('joint_pos')
    return env.scene['robot'].data.joint_pos[:, action.joint_ids] - action._offset


def joint_velocities(env):
    action = env.action_manager.get_term('joint_pos')
    return env.scene['robot'].data.joint_vel[:, action.joint_ids]


def applied_policy_action(env):
    action = env.action_manager.get_term('joint_pos')
    # Preserve the historical applied-action mode for reproducible baselines.
    # Deployment instead reports the last publisher output, before downstream
    # application delay and external arm composition.
    return action.published_action if action.cfg.action_feedback=='published' else action.raw_action


def build_env(num_envs, seed, profile='mixed', pose_index=0, pose_bank='carry', arm_transitions=False,
              objective='tracking',smoothness_weight=.2,dynamics='historical',max_delay_steps=0,action_hold_probability=0.,
              max_sensor_delay_steps=0,action_feedback='applied',command_profile='mixed'):
    cfg = unitree_g1_flat_env_cfg()
    cfg.seed = seed
    cfg.scene.num_envs = num_envs
    cfg.episode_length_s = 30.
    cfg.decimation = 10
    cfg.sim.mujoco.timestep = .002
    robot = cfg.scene.entities['robot']
    robot.spec_fn = {'historical':payload_spec,'native_inertial':native_inertial_payload_spec,
                     'native_contact':native_contact_payload_spec}[dynamics]
    if dynamics=='native_contact':
        for collision in robot.collisions:
            collision.friction = {r'^(left|right)_foot[1-7]_collision$':(1.,.005,.0001)}
            collision.priority = {r'^(left|right)_foot[1-7]_collision$':0}
    robot.init_state.pos = (0., 0., .793)
    robot.init_state.joint_pos = dict(zip(POLICY_NAMES, UNITREE_RL_LAB_G1_DEFAULT_POS))
    actuators = []
    native = native_robot_model() if dynamics in ('native_inertial','native_contact') else None
    for sdk, name in enumerate(G1_29DOF_JOINTS):
        effort = 5. if 'wrist_pitch' in name or 'wrist_yaw' in name else 25. if sdk >= 15 else 139. if 'knee' in name or 'hip_roll' in name else 50. if 'ankle' in name or 'waist' in name else 88.
        if native is not None:
            actuator = np.flatnonzero(native.actuator_trnid[:,0]==native.joint(name).id)[0]
            effort = float(native.actuator_ctrlrange[actuator,1])
        actuators.append(BuiltinPositionActuatorCfg(target_names_expr=(name,),
            stiffness=160. if sdk >= 15 and profile != 'unloaded' else G1_UNITREE_RL_LAB_BODY_KP[sdk],
            damping=4. if sdk >= 15 and profile != 'unloaded' else G1_UNITREE_RL_LAB_BODY_KD[sdk],
            effort_limit=effort, armature=.01))
    robot.articulation.actuators = tuple(actuators)
    cfg.actions = {'joint_pos': PayloadActionCfg(entity_name='robot', actuator_names=('.*',),
                    scale=.25, use_default_offset=True, profile=profile, pose_index=pose_index,
                    pose_bank=pose_bank, arm_transitions=arm_transitions,max_delay_steps=max_delay_steps,
                    action_hold_probability=action_hold_probability,max_sensor_delay_steps=max_sensor_delay_steps,
                    action_feedback=action_feedback)}
    actor_terms = {
        'base_ang_vel': ObservationTermCfg(func=mdp.builtin_sensor, params={'sensor_name': 'robot/imu_ang_vel'}, scale=.2),
        'projected_gravity': ObservationTermCfg(func=mdp.projected_gravity),
        'velocity_commands': ObservationTermCfg(func=mdp.generated_commands, params={'command_name': 'twist'}),
        'joint_pos_rel': ObservationTermCfg(func=joint_positions),
        'joint_vel_rel': ObservationTermCfg(func=joint_velocities, scale=.05),
        'last_action': ObservationTermCfg(func=applied_policy_action),
    }
    critic_terms = copy.deepcopy(actor_terms)
    if max_sensor_delay_steps:
        for name in ('base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel'):
            term = actor_terms[name]
            term.func,term.params = DelayedSensorTerm,dict(source=term.func,source_params=term.params,
                                                        max_lag=max_sensor_delay_steps)
    cfg.observations = {
        'actor': ObservationGroupCfg(terms=actor_terms, concatenate_terms=True, history_length=5, enable_corruption=False),
        'critic': ObservationGroupCfg(terms={**critic_terms, 'base_lin_vel': ObservationTermCfg(func=mdp.base_lin_vel)}, concatenate_terms=True, enable_corruption=False),
    }
    cfg.commands['twist'] = StandCreepStopVelocityCommandCfg(entity_name='robot',
        resampling_time_range=(2., 5.), heading_command=False, debug_vis=False,
        low_speed_probability=.15, moderate_speed_probability=.30,
        lateral_probability=.15, arc_probability=.10, diagonal_probability=.10,
        reverse_probability=.25, low_speed_range=(.12, .25), moderate_speed_range=(.25, .65),
        lateral_speed_min=.15, lateral_speed_max=.45, yaw_speed_min=.3, yaw_speed_max=.65,
        diagonal_speed_range=(.15, .4), initial_standing_probability=.25,
        moving_stop_probability=.3, standing_hold_probability=.15,
        ranges=StandCreepStopVelocityCommandCfg.Ranges(lin_vel_x=(-.45, .65), lin_vel_y=(-.45, .45), ang_vel_z=(-.65, .65)))
    cfg.events = {name: cfg.events[name] for name in ('reset_base', 'reset_robot_joints')}
    cfg.events['reset_base'].params['pose_range'] = {'x': (0., 0.), 'y': (0., 0.), 'z': (0., 0.), 'yaw': (-math.pi, math.pi)}
    cfg.events['payload'] = EventTermCfg(func=reset_payload, mode='reset')
    cfg.curriculum = {}
    cfg.rewards.pop('pose', None)  # External hand poses must not be penalized as bad posture.
    cfg.rewards['track_linear_velocity'].params['std'] = .25
    cfg.rewards['track_angular_velocity'].params['std'] = .35
    cfg.rewards['action_rate_l2'].weight = -.02
    if objective in ('fluid','reactive','stopping'):
        cfg.rewards['fluid_carry'] = RewardTermCfg(func=FluidCarryPenalty,weight=-smoothness_weight,
                                                  params={'up_axis':cup_up_axis()})
        # Match native clockwise pivots up to 0.9 rad/s, which cycle 01 did not
        # cover. Retain translations/stops so stillness cannot replace the task.
        command = cfg.commands['twist']
        command.low_speed_probability = .10
        command.moderate_speed_probability = .15
        command.lateral_probability = .10
        command.arc_probability = .20
        command.diagonal_probability = .10
        command.yaw_speed_max = .90
        command.ranges.ang_vel_z = (-.90,.90)
        command.moving_stop_probability = .40
    if objective in ('reactive','stopping'):
        # Tracking must not be traded away for the smoother nearly-stationary
        # pivots seen in cycle-03. Evaluate both progress and uprightness.
        cfg.rewards['track_angular_velocity'].weight *= 2.5
        cfg.rewards['track_linear_velocity'].weight *= 1.25
        cfg.rewards['turn_drift'] = RewardTermCfg(func=turn_drift_penalty,weight=-3.)
    if objective=='stopping':
        cfg.rewards['stop_velocity'] = RewardTermCfg(func=stopping_velocity_penalty,weight=-4.)
    if command_profile=='transitions':
        cfg.commands['twist'] = TransitionVelocityCommandCfg(**dict(vars(cfg.commands['twist']),
            initial_standing_probability=1.,moving_stop_probability=1.,standing_hold_probability=0.,
            low_speed_probability=.1,moderate_speed_probability=.1,lateral_probability=.5,
            arc_probability=.1,diagonal_probability=.1,lateral_speed_min=.3,lateral_speed_max=.45))
    elif command_profile!='mixed':
        raise ValueError('unknown command profile')
    return cfg


def load_onnx_actor(actor, path):
    model = onnx.load(str(path))
    state = {item.name.removeprefix('actor.'): torch.from_numpy(numpy_helper.to_array(item).copy())
             for item in model.graph.initializer}
    actor.mlp.load_state_dict(state, strict=True)
    # A conversion parity check prevents a silent architecture/order conversion.
    from onnx.reference import ReferenceEvaluator
    samples = np.random.default_rng(0).normal(0., .2, (8, 480)).astype(np.float32)
    expected = ReferenceEvaluator(model).run(None, {model.graph.input[0].name: samples})[0]
    with torch.no_grad():
        actual = actor.mlp(torch.as_tensor(samples, device=next(actor.parameters()).device)).cpu().numpy()
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=2e-5)
    return float(np.max(np.abs(actual - expected)))


def initialize_arm_inputs(actor):
    """Start from the nominal-arm ablation while retaining trainable arm inputs.

    Unlike a runtime observation mask, these columns receive gradients and learn
    from the real measured arm state during PPO. Record this as a separate
    warm-start baseline so its effect is not attributed to RL training.
    """
    columns = [start + history*29 + joint for start in (45, 190, 335)
               for history in range(5) for joint in ARM_POLICY_IDS]
    with torch.no_grad():
        actor.mlp[0].weight[:, columns] = 0.


def evaluate(runner, wrapped, profile, output,probe='tracking'):
    env = wrapped.unwrapped
    profiles = ('unloaded', 'held_empty', 'loaded', 'heavy') if profile == 'mixed' else (profile,)
    schedule = (
        ('settle', 5., (0., 0., 0.)), ('forward', 4., (.4, 0., 0.)),
        ('stop_forward', 2., (0., 0., 0.)), ('turn_left', 3., (0., 0., .5)),
        ('turn_right', 3., (0., 0., -.5)), ('reverse', 3., (-.3, 0., 0.)),
        ('lateral', 3., (0., .3, 0.)), ('diagonal', 3., (.3, -.2, 0.)),
        ('final_stop', 2., (0., 0., 0.)),
    )
    if probe == 'loaded_turns':
        schedule = LOADED_TURN_SCHEDULE
    elif probe == 'signed_stops':
        schedule = SIGNED_STOP_SCHEDULE
    reports = []
    runner.alg.actor.eval()
    for case in profiles:
        action = env.action_manager.get_term('joint_pos')
        action.cfg.profile = case
        observations, _ = wrapped.reset()
        command = env.command_manager.get_term('twist')
        alive = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
        body = env.scene['robot'].find_bodies('carried_payload')[0][0]
        motion = PayloadMotion(env.num_envs,env.device,env.step_dt,up_axis=cup_up_axis())
        phases = []
        for name, duration, values in schedule:
            count, error_xy, error_yaw, speed_peak = 0, 0., 0., 0.
            payload_samples = {}
            yaw_displacement = torch.zeros(env.num_envs,device=env.device)
            for _ in range(round(duration / env.step_dt)):
                command.time_left.fill_(1000.)
                command.is_standing_env[:] = not any(values)
                command.vel_command_b[:] = torch.tensor(values, device=env.device)
                with torch.no_grad():
                    controls = runner.alg.actor(observations)
                    observations, _, dones, _ = wrapped.step(controls)
                alive &= ~dones.bool()
                data = env.scene['robot'].data
                velocity = data.root_link_lin_vel_b[:, :2]
                yaw_rate = data.root_link_ang_vel_b[:, 2]
                motion.reset(torch.where(dones.bool())[0])
                measured = motion.update(data.body_link_lin_vel_w[:,body],data.body_link_ang_vel_w[:,body],data.body_link_quat_w[:,body])
                valid = alive & measured['derivative_valid']
                yaw_displacement += torch.where(alive,yaw_rate*env.step_dt,0.)
                if valid.any():
                    for key,value in measured.items():
                        if key != 'derivative_valid':
                            payload_samples.setdefault(key,[]).append(value[valid].cpu().numpy())
                if alive.any():
                    error_xy += torch.linalg.vector_norm(velocity[alive] - torch.tensor(values[:2], device=env.device), dim=-1).sum().item()
                    error_yaw += (yaw_rate[alive] - values[2]).abs().sum().item()
                    speed_peak = max(speed_peak, torch.linalg.vector_norm(velocity[alive], dim=-1).max().item())
                    count += int(alive.sum())
            distributions = {}
            for key,chunks in payload_samples.items():
                sample = np.concatenate(chunks)
                distributions[key] = dict(count=len(sample),mean=float(np.mean(sample)),min=float(np.min(sample)),
                    p95=float(np.quantile(sample,.95)),p99=float(np.quantile(sample,.99)),max=float(np.max(sample)))
            effective = np.concatenate(payload_samples['effective_tilt_rad']) if 'effective_tilt_rad' in payload_samples else np.array([])
            normal_force = np.concatenate(payload_samples['normal_specific_force_m_s2']) if len(effective) else np.array([])
            phases.append(dict(phase=name, command=values, survivors=int(alive.sum()),payload_motion=distributions,
                                spill_proxy_exceedance_fraction=float(np.mean((effective>math.atan2(.02,.04)) | (normal_force<1.))) if len(effective) else None,
                                mean_yaw_displacement_rad=float(yaw_displacement[alive].mean()) if alive.any() else None,
                                mean_planar_error_m_s=error_xy/count if count else None,
                               mean_yaw_error_rad_s=error_yaw/count if count else None,
                               peak_planar_speed_m_s=speed_peak))
        report = dict(profile=case, environments=env.num_envs, survivors=int(alive.sum()),
                       probe=probe,fluid_model='rigid virtual cup proxy, not liquid simulation',
                       cup_up_axis_local=cup_up_axis(),cup_radius_m=.04,freeboard_m=.02,acceleration_filter_tau_s=.08,
                      pose_index=action.cfg.pose_index,
                      failures=int((~alive).sum()), phases=phases,
                      payload_mass_kg={'unloaded': 0., 'held_empty': 0., 'loaded': .15, 'heavy': .35}[case])
        reports.append(report)
        output.write_text(json.dumps(reports, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'train', 'evaluate'), default='smoke')
    parser.add_argument('--num-envs', type=int, default=64)
    parser.add_argument('--iterations', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--arm-init', choices=('pretrained', 'zero'), default='pretrained')
    parser.add_argument('--pose-index', type=int, default=0, help='Fixed evaluation pose (3 is the close pregrasp reach)')
    parser.add_argument('--pose-bank', choices=('carry','reach'), default='carry')
    parser.add_argument('--arm-transitions', action='store_true', help='Resample held-arm targets every eight seconds during training')
    parser.add_argument('--profile', choices=('mixed', 'unloaded', 'held_empty', 'loaded', 'heavy'), default='mixed')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--objective',choices=('tracking','fluid','reactive','stopping'),default='tracking')
    parser.add_argument('--smoothness-weight',type=float,default=.2)
    parser.add_argument('--probe',choices=('tracking','loaded_turns','signed_stops'),default='tracking')
    parser.add_argument('--dynamics',choices=('historical','native_inertial','native_contact'),default='historical')
    parser.add_argument('--max-delay-steps',type=int,default=0,help='Per-episode policy application delay sampled from 0..N periods')
    parser.add_argument('--action-hold-probability',type=float,default=0.,help='Per-tick missed-update exposure, retaining the previously applied action')
    parser.add_argument('--max-sensor-delay-steps',type=int,default=0,help='Shared per-episode IMU/joint observation lag, in 20 ms control periods')
    parser.add_argument('--action-feedback',choices=('applied','published'),default='applied')
    parser.add_argument('--command-profile',choices=('mixed','transitions'),default='mixed')
    args = parser.parse_args()
    if args.num_envs <= 0 or args.iterations <= 0 or args.seed < 0 or args.pose_index < 0:
        parser.error('Environment/iteration counts must be positive and seed/pose index nonnegative')
    if not 0<=args.max_delay_steps<=5:
        parser.error('--max-delay-steps must be between 0 and 5')
    if not 0<=args.max_sensor_delay_steps<=5:
        parser.error('--max-sensor-delay-steps must be between 0 and 5')
    if not 0.<=args.action_hold_probability<=.2:
        parser.error('--action-hold-probability must be in [0, .2]')
    if args.pose_bank == 'carry' and args.pose_index > 2:
        parser.error('Pose indices above 2 require --pose-bank reach')
    if not math.isfinite(args.smoothness_weight) or args.smoothness_weight<=0.:
        parser.error('--smoothness-weight must be finite and positive')
    torch.set_num_threads(4)
    wp.config.kernel_cache_dir = str(ROOT / 'vendor/mjlab/logs/.warp_payload_cache')
    torch.manual_seed(args.seed)
    # vendor/ is a writable host bind mount; /workspace/logs is ephemeral.
    output = args.output or ROOT / 'vendor/mjlab/logs/payload_training' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True, exist_ok=False)
    sys.stdout = ConsoleTee(sys.stdout,output/'console.stdout.log')
    sys.stderr = ConsoleTee(sys.stderr,output/'console.stderr.log')
    cfg = build_env(args.num_envs, args.seed, args.profile, args.pose_index, args.pose_bank, args.arm_transitions,
                    args.objective,args.smoothness_weight,args.dynamics,args.max_delay_steps,args.action_hold_probability,
                    args.max_sensor_delay_steps,args.action_feedback,args.command_profile)
    if args.mode == 'evaluate' and args.probe in ('loaded_turns','signed_stops'):
        schedule = LOADED_TURN_SCHEDULE if args.probe=='loaded_turns' else SIGNED_STOP_SCHEDULE
        cfg.episode_length_s = sum(item[1] for item in schedule)+2.
    agent = unitree_g1_ppo_runner_cfg()
    agent.actor.obs_normalization = False
    agent.critic.obs_normalization = True
    agent.actor.distribution_cfg['init_std'] = .1
    agent.algorithm.learning_rate = 1e-5
    agent.algorithm.schedule = 'fixed'
    agent.algorithm.entropy_coef = .002
    agent.save_interval = 25
    agent.logger = 'tensorboard'
    agent.seed = args.seed
    agent.experiment_name = 'g1_payload'
    manifest = dict(mode=args.mode, seed=args.seed, num_envs=args.num_envs, iterations=args.iterations,
                    dynamics=args.dynamics,max_delay_steps=args.max_delay_steps,action_hold_probability=args.action_hold_probability,
                    max_sensor_delay_steps=args.max_sensor_delay_steps,action_feedback=args.action_feedback,
                     control_period_s=.02,
                     command_profile=args.command_profile,
                     transition_configuration=dict(moving_ticks=[75,175],standing_ticks=[250,300],lateral_probability=.5,
                        lateral_endpoint_probability=.5,deadline_margin_ticks=.5) if args.command_profile=='transitions' else None,
                    sensor_delay_scope='coherent IMU/joint packet, per-environment per-episode; commands/action feedback remain current',
                    profile=args.profile, source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                    arm_input_initialization=args.arm_init,
                    evaluation_pose_index=args.pose_index,
                    pose_bank=args.pose_bank, arm_transitions=args.arm_transitions,
                    optimizer_learning_rates=dict(actor=1e-5, critic=3e-4),
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                     checkpoint=None if args.checkpoint is None else str(args.checkpoint),
                     checkpoint_sha256=None if args.checkpoint is None else hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                     objective=args.objective,smoothness_weight=args.smoothness_weight,probe=args.probe,
                     cup_up_axis_local=cup_up_axis(),
                    actor_contract='unitree_rl_lab_g1_29dof: 480 observations / 29 actions; measured arms',
                    payload_mass_range_kg=[0., .5], hand_offset_m=[.19, .01, 0.], base_pinning=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output / 'training_source.py').write_bytes(Path(__file__).read_bytes())
    (output / 'payload_smoothness.py').write_bytes((ROOT/'scripts/payload_smoothness.py').read_bytes())
    (output / 'payload_sensor_delay.py').write_bytes((ROOT/'scripts/payload_sensor_delay.py').read_bytes())
    (output / 'payload_transition_commands.py').write_bytes((ROOT/'scripts/payload_transition_commands.py').read_bytes())
    if args.dynamics in ('native_inertial','native_contact'):
        native_source = ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml'
        (output/'native_robot_source.xml').write_bytes(native_source.read_bytes())
    for source in (URDF, *[ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge'/name
                          for name in ('arm_kinematics.py','payload_postures.py')]):
        (output / source.name).write_bytes(source.read_bytes())
    dump_yaml(output / 'env.yaml', asdict(cfg))
    dump_yaml(output / 'agent.yaml', asdict(agent))
    env = ManagerBasedRlEnv(cfg, device='cuda:0')
    wrapped = RslRlVecEnvWrapper(env, clip_actions=None)
    runner = VelocityOnPolicyRunner(wrapped, asdict(agent), str(output), device='cuda:0')
    parity = load_onnx_actor(runner.alg.actor, SOURCE)
    manifest['onnx_import_max_error'] = parity
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'ONNX conversion maximum error: {parity}', flush=True)
    if args.checkpoint:
        runner.load(str(args.checkpoint), load_cfg={'actor': True, 'critic': True,
                    'optimizer': False, 'iteration': True, 'rnd': False})
    elif args.arm_init == 'zero':
        initialize_arm_inputs(runner.alg.actor)
    # The actor is pretrained, but ONNX has no critic checkpoint. Fit that
    # fresh value function faster without applying its learning rate to actor.
    runner.alg.optimizer = torch.optim.Adam([
        {'params': runner.alg.actor.parameters(), 'lr': 1e-5},
        {'params': runner.alg.critic.parameters(), 'lr': 3e-4},
    ])
    action = env.action_manager.get_term('joint_pos')
    print('Payload reset audit:', dict(held=int(action.held.sum()), mass_min=float(action.mass.min()), mass_max=float(action.mass.max()),
        application_delay_counts=torch.bincount(action.delay_steps,minlength=args.max_delay_steps+1).tolist(),
        sensor_delay_counts=torch.bincount(action.sensor_delay_steps,minlength=args.max_sensor_delay_steps+1).tolist()), flush=True)
    try:
        if args.mode in ('smoke', 'train'):
            runner.learn(num_learning_iterations=2 if args.mode == 'smoke' else args.iterations,
                         init_at_random_ep_len=False)
            runner.save(str(output / 'final.pt'))
            if args.command_profile=='transitions':
                command = env.command_manager.get_term('twist')
                audit = dict(positive_lateral_requests=int(command.transition_counts[0]),
                    negative_lateral_requests=int(command.transition_counts[1]),
                    move_to_stop_requests=int(command.transition_counts[2]),
                    duration_tick_counts=command.duration_counts.cpu().tolist(),
                    scope='Sampled command requests across resets, not successful completed transitions')
                (output/'transition_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
            actor = copy.deepcopy(runner.alg.actor.mlp).cpu().eval()
            torch.onnx.export(actor, torch.zeros(1, 480), str(output / 'policy.onnx'),
                              input_names=['obs'], output_names=['actions'], opset_version=17, dynamo=False)
            print(f'Checkpoint: {output / "final.pt"}', flush=True)
        else:
            report = evaluate(runner, wrapped, args.profile, output / 'evaluation.json',args.probe)
            (output / 'evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    finally:
        env.close()


if __name__ == '__main__':
    main()
