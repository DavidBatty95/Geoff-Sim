#!/usr/bin/env python3
"""Compare final compiled training/native plants on mapped physical states (CPU)."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from mjlab.scene import Scene
from g1_payload_training import ROOT,build_env,G1_29DOF_JOINTS,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD
from native_probe_payload import attach_payload
from native_probe_checkpoint import model_digest


def indexes(model,prefix=''):
    joints = np.array([model.joint(prefix+name).id for name in G1_29DOF_JOINTS])
    actuators = np.array([np.flatnonzero(model.actuator_trnid[:,0]==joint)[0] for joint in joints])
    root = model.body_jntadr[model.body(prefix+'pelvis').id]
    return joints,actuators,root


def run_state(model,record,prefix='',position=False,warp_backend=False):
    model = copy.copy(model)
    joints,aids,root = indexes(model,prefix)
    qids,dids = model.jnt_qposadr[joints],model.jnt_dofadr[joints]
    data = mujoco.MjData(model)
    if not prefix:
        mujoco.mj_resetDataKeyframe(model,data,model.key('transport_unitree_rl_lab_stand').id)
        mujoco.mj_forward(model,data)
        offset = record.get('payload_offset_m')
        payload = attach_payload(model,data,offset[0] if offset else None)
    source = record['portable_state']
    q,d = model.jnt_qposadr[root],model.jnt_dofadr[root]
    data.qpos[q:q+7],data.qvel[d:d+6] = source['root_qpos'],source['root_qvel']
    data.qpos[qids],data.qvel[dids] = source['joint_qpos_sdk'],source['joint_qvel_sdk']
    if not prefix:
        joint = model.body_jntadr[payload['body']]
        pq,pd = model.jnt_qposadr[joint],model.jnt_dofadr[joint]
        data.qpos[pq:pq+7],data.qvel[pd:pd+6] = source['payload_qpos'],source['payload_qvel']
    target = np.asarray(record['target_sdk'])
    kp,kd = np.array(G1_UNITREE_RL_LAB_BODY_KP),np.array(G1_UNITREE_RL_LAB_BODY_KD)
    kp[15:],kd[15:] = 160.,4.
    data.ctrl[aids] = target if position else kp*(target-data.qpos[qids])-kd*data.qvel[dids]
    mujoco.mj_forward(model,data)
    if warp_backend:
        import warp as wp
        import mujoco_warp as mjwarp
        wp.config.kernel_cache_dir = str(ROOT/'vendor/mjlab/logs/.warp_payload_cache')
        with wp.ScopedDevice('cuda:0'):
            wp_model = mjwarp.put_model(model)
            wp_data = mjwarp.put_data(model,data,nworld=1)
            mjwarp.forward(wp_model,wp_data)
            joint_forces = wp_data.qfrc_actuator.numpy()[0,dids].copy()
            mjwarp.step(wp_model,wp_data)
            return dict(joint_qpos=wp_data.qpos.numpy()[0,qids].copy(),
                        joint_qvel=wp_data.qvel.numpy()[0,dids].copy(),joint_force=joint_forces,
                        root_qvel=wp_data.qvel.numpy()[0,d:d+6].copy(),contacts=[],solver_iterations=[])
    joint_forces = data.qfrc_actuator[dids].copy()
    contacts = []
    for contact in data.contact:
        contacts.append(dict(geoms=[model.geom(int(g)).name for g in (contact.geom1,contact.geom2)],
                             distance=float(contact.dist),position=contact.pos.tolist(),dimension=int(contact.dim)))
    mujoco.mj_step(model,data)
    return dict(joint_qpos=data.qpos[qids].copy(),joint_qvel=data.qvel[dids].copy(),joint_force=joint_forces,
                root_qvel=data.qvel[d:d+6].copy(),contacts=contacts,solver_iterations=data.solver_niter.tolist())


def static_report(native,training):
    fields = ('integrator','cone','impratio','iterations','tolerance','ls_iterations','ls_tolerance','disableflags','enableflags')
    options = {name:dict(native=float(getattr(native.opt,name)),training=float(getattr(training.opt,name))) for name in fields}
    differences = []
    for name in G1_29DOF_JOINTS:
        a,b = native.joint(name).id,training.joint('robot/'+name).id
        ba,bb = native.jnt_bodyid[a],training.jnt_bodyid[b]
        da,db = native.jnt_dofadr[a],training.jnt_dofadr[b]
        for field,x,y in [('range',native.jnt_range[a],training.jnt_range[b]),
                          ('axis',native.jnt_axis[a],training.jnt_axis[b]),
                          ('joint_pos',native.jnt_pos[a],training.jnt_pos[b]),
                          ('mass',native.body_mass[ba],training.body_mass[bb]),
                          ('inertia',native.body_inertia[ba],training.body_inertia[bb]),
                          ('body_pos',native.body_pos[ba],training.body_pos[bb]),
                          ('body_quat',native.body_quat[ba],training.body_quat[bb]),
                          ('inertial_pos',native.body_ipos[ba],training.body_ipos[bb]),
                          ('inertial_quat',native.body_iquat[ba],training.body_iquat[bb]),
                          ('armature',native.dof_armature[da],training.dof_armature[db]),
                          ('damping',native.dof_damping[da],training.dof_damping[db]),
                          ('frictionloss',native.dof_frictionloss[da],training.dof_frictionloss[db])]:
            if not np.allclose(x,y,rtol=1e-6,atol=1e-8):
                differences.append(dict(joint=name,field=field,native=np.asarray(x).tolist(),training=np.asarray(y).tolist()))
    geometries = {}
    robot_bodies = {}
    for label,model,prefix in (('native',native,''),('training',training,'robot/')):
        root = model.body(prefix+'pelvis').id
        robot_bodies[label] = {model.body(i).name.removeprefix(prefix):float(model.body_mass[i])
                              for i in range(model.nbody) if model.body_rootid[i]==root}
        feet = [model.body(prefix+side+'_ankle_roll_link').id for side in ('left','right')]
        geometries[label] = [dict(name=model.geom(i).name,body=model.body(int(model.geom_bodyid[i])).name,
            type=int(model.geom_type[i]),pos=model.geom_pos[i].tolist(),size=model.geom_size[i].tolist(),
            friction=model.geom_friction[i].tolist(),condim=int(model.geom_condim[i]),priority=int(model.geom_priority[i]),
            solref=model.geom_solref[i].tolist(),solimp=model.geom_solimp[i].tolist()) for i in range(model.ngeom)
            if (model.geom_bodyid[i] in feet or model.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE) and
               (model.geom_contype[i] or model.geom_conaffinity[i])]
    unmatched = {label:{name:mass for name,mass in bodies.items() if name not in robot_bodies['training' if label=='native' else 'native']}
                 for label,bodies in robot_bodies.items()}
    return dict(options=options,joint_body_differences=differences,foot_and_floor_geometries=geometries,
                robot_body_masses_kg=robot_bodies,unmatched_robot_bodies=unmatched,
                payload_offset_m=dict(native_fixture=native.numeric('transport_hand_capture').data[:3].tolist(),
                                      training=training.body('robot/carried_payload').pos.tolist()),
                root_subtree_mass_kg={label:sum(bodies.values()) for label,bodies in robot_bodies.items()},
                mass_scope='Training subtree includes rigid payload; native magnetic payload is a separate free body')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--states',required=True,type=Path,help='Portable states exported by compare_native_actuation.py')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--native-joint-friction',action='store_true')
    parser.add_argument('--match-native-solver',action='store_true',help='Diagnostic iteration-budget comparison, not a training default')
    parser.add_argument('--match-payload-offset',action='store_true',help='Diagnostic rigid-load placement at native fixture offset')
    parser.add_argument('--warp',action='store_true',help='Also compare the same training model on the CUDA Warp backend')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    cfg = build_env(1,77,profile='loaded',dynamics='native_contact',integrator='euler',target_soft_limit_factor=.9,
                    native_joint_friction=args.native_joint_friction)
    scene = Scene(cfg.scene,device='cpu')
    training = scene.compile()
    cfg.sim.mujoco.apply(training)
    native = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'))
    native.opt.timestep = .002
    records = json.loads(args.states.read_text())['attempts']
    if args.match_native_solver:
        training.opt.iterations = native.opt.iterations
        training.opt.ls_iterations = native.opt.ls_iterations
    if args.match_payload_offset:
        offsets = {tuple(record.get('payload_offset_m') or native.numeric('transport_hand_capture').data[:3]) for record in records}
        if len(offsets)!=1:
            raise ValueError('payload-offset matching requires one common captured offset')
        training.body('robot/carried_payload').pos[:] = next(iter(offsets))
        mujoco.mj_setConst(training,mujoco.MjData(training))
    report = dict(schema_version=1,mujoco_version=mujoco.__version__,
        scope='Mapped root/joints and model-specific payload attachment; one-step CPU plant comparison with optional training-model Warp comparison; not mission replay',
        source_sha256=hashlib.sha256(args.states.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),native_joint_friction=args.native_joint_friction,
        match_native_solver=args.match_native_solver,
        match_payload_offset=args.match_payload_offset,
        warp_backend=args.warp,
        training_source_sha256=hashlib.sha256((ROOT/'scripts/g1_payload_training.py').read_bytes()).hexdigest(),
        compiled_model_sha256=dict(native=model_digest(native),training=model_digest(training)),
        static=static_report(native,training),attempts=[])
    for record in records:
        try:
            a,b = run_state(native,record),run_state(training,record,'robot/',True)
            errors = {key:float(np.max(np.abs(a[key]-b[key]))) for key in ('joint_qpos','joint_qvel','joint_force','root_qvel')}
            row = dict(checkpoint=record['checkpoint'],max_abs_deltas=errors,
                worst_velocity_joint=G1_29DOF_JOINTS[int(np.argmax(np.abs(a['joint_qvel']-b['joint_qvel'])))],
                native_solver_iterations=a['solver_iterations'],training_solver_iterations=b['solver_iterations'],
                native_contacts=a['contacts'],training_contacts=b['contacts'])
            if args.warp:
                gpu = run_state(training,record,'robot/',True,warp_backend=True)
                row['training_cpu_warp_max_abs_deltas'] = {key:float(np.max(np.abs(b[key]-gpu[key])))
                    for key in ('joint_qpos','joint_qvel','joint_force','root_qvel')}
                row['native_cpu_training_warp_max_abs_deltas'] = {key:float(np.max(np.abs(a[key]-gpu[key])))
                    for key in ('joint_qpos','joint_qvel','joint_force','root_qvel')}
        except (ValueError,KeyError,TypeError,RuntimeError) as error:
            row = dict(checkpoint=record.get('checkpoint'),error=str(error))
        report['attempts'].append(row)
        print(json.dumps(row),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 1 if any('error' in row for row in report['attempts']) else 0


if __name__=='__main__':
    raise SystemExit(main())
