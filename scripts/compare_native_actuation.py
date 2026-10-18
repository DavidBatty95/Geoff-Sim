#!/usr/bin/env python3
"""Isolate actuator/integrator semantics on identical captured native states.

The plant is held fixed. Position actuators use MJLab's built-in PD coefficients;
this is not a full CPU/Warp or training-geometry equivalence test.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from native_probe_checkpoint import model_digest,restore_array
from native_probe_payload import attach_payload

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.g1_constants import G1_29DOF_JOINTS,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD


def position_actuators(model,actuators,kp,kd):
    model.actuator_gaintype[actuators] = mujoco.mjtGain.mjGAIN_FIXED
    model.actuator_biastype[actuators] = mujoco.mjtBias.mjBIAS_AFFINE
    model.actuator_gainprm[actuators] = 0.
    model.actuator_biasprm[actuators] = 0.
    model.actuator_gainprm[actuators,0] = kp
    model.actuator_biasprm[actuators,1] = -kp
    model.actuator_biasprm[actuators,2] = -kd
    model.actuator_forcerange[actuators] = model.actuator_ctrlrange[actuators]
    model.actuator_forcelimited[actuators] = True
    model.actuator_ctrllimited[actuators] = False


def step_variant(base,state,target,qids,dids,aids,kp,kd,position=False,implicit=False):
    model = copy.copy(base)
    if implicit:
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if position:
        position_actuators(model,aids,kp,kd)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model,data,state,mujoco.mjtState.mjSTATE_INTEGRATION)
    data.ctrl[aids] = target if position else kp*(target-data.qpos[qids])-kd*data.qvel[dids]
    mujoco.mj_forward(model,data)
    force = data.qfrc_actuator.copy()
    acceleration = data.qacc.copy()
    mujoco.mj_step(model,data)
    return dict(force=force,acceleration=acceleration,qpos=data.qpos.copy(),qvel=data.qvel.copy())


def compare_checkpoint(path,scene):
    record = json.loads(path.read_text())
    json.dumps(record,allow_nan=False)
    if record['mujoco_version']!=mujoco.__version__ or record['boundary']!='before_mj_step':
        raise ValueError('checkpoint engine/boundary mismatch')
    config = record['identity']['configuration']
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = .002
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('transport_unitree_rl_lab_stand').id)
    mujoco.mj_forward(model,data)
    payload = attach_payload(model,data,config.get('payload_offset_x')) if config['payload'] else None
    if model_digest(model)!=record['model_sha256']:
        raise ValueError('compiled native model differs from captured model')
    state = np.asarray(record['state'],dtype=float)
    if state.shape!=(mujoco.mj_stateSize(model,mujoco.mjtState.mjSTATE_INTEGRATION),) or not np.all(np.isfinite(state)):
        raise ValueError('invalid integration state')
    joints = np.array([model.joint(name).id for name in G1_29DOF_JOINTS])
    qids,dids = model.jnt_qposadr[joints],model.jnt_dofadr[joints]
    aids = np.array([np.flatnonzero(model.actuator_trnid[:,0]==joint)[0] for joint in joints])
    kp,kd = np.asarray(G1_UNITREE_RL_LAB_BODY_KP).copy(),np.asarray(G1_UNITREE_RL_LAB_BODY_KD).copy()
    if config['held_empty'] or config['payload']:
        kp[15:],kd[15:] = 160.,4.
    target = restore_array(record['controller']['target'])
    mujoco.mj_setState(model,data,state,mujoco.mjtState.mjSTATE_INTEGRATION)
    portable = dict(root_qpos=data.qpos[:7].tolist(),root_qvel=data.qvel[:6].tolist(),
                    joint_qpos_sdk=data.qpos[qids].tolist(),joint_qvel_sdk=data.qvel[dids].tolist())
    if payload:
        joint = model.body_jntadr[payload['body']]
        q,d = model.jnt_qposadr[joint],model.jnt_dofadr[joint]
        portable.update(payload_qpos=data.qpos[q:q+7].tolist(),payload_qvel=data.qvel[d:d+6].tolist())
    variants = {name:step_variant(model,state,target,qids,dids,aids,kp,kd,position,implicit)
                for name,position,implicit in (('motor_native',False,False),('position_native',True,False),
                                               ('motor_implicitfast',False,True),('position_implicitfast',True,True))}
    reference = variants['motor_native']
    metrics = {}
    for name,values in variants.items():
        error = values['qvel'][dids]-reference['qvel'][dids]
        metrics[name] = dict(force_max_abs_delta_nm=float(np.max(np.abs(values['force']-reference['force']))),
            acceleration_max_abs_delta=float(np.max(np.abs(values['acceleration']-reference['acceleration']))),
            joint_velocity_max_abs_delta_rps=float(np.max(np.abs(error))),
            joint_velocity_delta_rps=error.tolist(),worst_joint=G1_29DOF_JOINTS[int(np.argmax(np.abs(error)))],
            joint_position_max_abs_delta_rad=float(np.max(np.abs(values['qpos'][qids]-reference['qpos'][qids]))),
            root_velocity_delta=(values['qvel'][:6]-reference['qvel'][:6]).tolist())
    return dict(checkpoint=str(path),checkpoint_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        checkpoint_time_s=float(state[0]),policy_sha256=record['identity']['policy_sha256'],
        model_sha256=record['model_sha256'],native_integrator=str(mujoco.mjtIntegrator(model.opt.integrator)),
        timestep_s=model.opt.timestep,payload_mass_kg=payload['mass_kg'] if payload else None,
        payload_offset_m=payload['offset'].tolist() if payload else None,
        target_sdk=target.tolist(),portable_state=portable,metrics=metrics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,nargs='+',required=True)
    parser.add_argument('--scene',type=Path,default=ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    report = dict(schema_version=1,scope=__doc__,mujoco_version=mujoco.__version__,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (ROOT/'scripts/g1_payload_training.py',ROOT/'vendor/mjlab/src/mjlab/utils/spec.py',ROOT/'vendor/mjlab/src/mjlab/sim/sim.py')},
        attempts=[])
    for path in args.checkpoint:
        try:
            row = compare_checkpoint(path,args.scene)
        except (OSError,ValueError,TypeError,KeyError) as error:
            row = dict(checkpoint=str(path),error=str(error))
        report['attempts'].append(row)
        print(json.dumps(row),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 1 if any('error' in row for row in report['attempts']) else 0


if __name__=='__main__':
    raise SystemExit(main())
