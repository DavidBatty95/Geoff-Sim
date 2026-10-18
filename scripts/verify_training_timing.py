#!/usr/bin/env python3
"""Exercise native sensor timing, grouped PD gains and per-world load offsets on GPU."""
import argparse
import hashlib
import json
from pathlib import Path

import torch
import warp as wp

from g1_payload_training import ROOT,build_env,ManagerBasedRlEnv,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD
from payload_native_timing import NativeSensorPd,packet_gravity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    wp.config.kernel_cache_dir = str(ROOT/'vendor/mjlab/logs/.warp_payload_cache')
    torch.set_num_threads(2)
    cfg = build_env(16,78,dynamics='native_contact',integrator='euler',native_joint_friction=True,
                    native_timing=True,payload_offset_range=(.12,.19),target_soft_limit_factor=.9,
                    action_feedback='published',max_delay_steps=3)
    report = dict(scope='GPU training timing/reset contract; short zero-action diagnostic, not policy qualification',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  sources={name:hashlib.sha256((ROOT/'scripts'/name).read_bytes()).hexdigest()
                           for name in ('g1_payload_training.py','payload_native_timing.py')},
                  passed=False,steps=[])
    env = None
    try:
        env = ManagerBasedRlEnv(cfg,device='cuda:0')
        env.reset()
        action = env.action_manager.get_term('joint_pos')
        robot = env.scene['robot']
        group = next(item for item in robot.actuators if isinstance(item,NativeSensorPd))
        assert group.packet is action.native_packet
        for _ in range(3):
            observations = env.step(torch.zeros(16,29,device='cuda:0'))[0]
            packet = action.native_packet
            q,dq = robot.data.joint_pos,robot.data.joint_vel
            # Euler integrates joint positions with post-step velocity. The
            # retained joint sensor positions are exactly the pre-step values.
            epoch_error = (q-packet.values[:,:29]-.002*dq).abs().max().item()
            encoder_delta = (q-packet.values[:,:29]).abs().max().item()
            sensor_position_error = (packet.values[:,:29][:,group.target_ids]-group.last_integrated_pos).abs().max().item()
            sensor_velocity_error = (packet.values[:,29:58][:,group.target_ids]-group.last_integrated_vel).abs().max().item()
            actor = observations['actor']
            joint_error = (actor[:,161:190]-(packet.values[:,:29][:,action.joint_ids]-action._offset)).abs().max().item()
            gyro_error = (actor[:,12:15]-.2*packet.values[:,58:61]).abs().max().item()
            gravity_error = (actor[:,27:30]-packet_gravity(packet.values[:,61:65])).abs().max().item()
            report['steps'].append(dict(epoch_identity_error_rad=epoch_error,encoder_delta_rad=encoder_delta,
                sensor_position_epoch_error_rad=sensor_position_error,sensor_velocity_epoch_error_rps=sensor_velocity_error,
                actor_joint_error_rad=joint_error,actor_gyro_error=gyro_error,actor_gravity_error=gravity_error))
            assert epoch_error<1e-6 and encoder_delta>0.
            assert max(sensor_position_error,sensor_velocity_error)<1e-6
            assert max(joint_error,gyro_error,gravity_error)<1e-6
        for name,base_kp,base_kd in zip(robot.joint_names,G1_UNITREE_RL_LAB_BODY_KP,G1_UNITREE_RL_LAB_BODY_KD):
            column = group.target_names.index(name)
            held_arm = action.held if ('shoulder' in name or 'elbow' in name or 'wrist' in name) else torch.zeros_like(action.held)
            torch.testing.assert_close(group.stiffness[:,column],torch.where(held_arm,160.,base_kp))
            torch.testing.assert_close(group.damping[:,column],torch.where(held_arm,4.,base_kd))
            for field,index in (('default_stiffness',0),('default_damping',1),('default_force_limit',2)):
                values = getattr(group,field)[:,column]
                torch.testing.assert_close(values,torch.full_like(values,group.cfg.gains[name][index]))
        torch.testing.assert_close(env.sim.model.actuator_gainprm[:,:,0],torch.ones_like(env.sim.model.actuator_gainprm[:,:,0]))
        assert torch.count_nonzero(env.sim.model.actuator_biasprm)==0
        body = robot.indexing.body_ids[robot.find_bodies('carried_payload')[0][0]]
        offsets = env.sim.model.body_pos[:,body,0]
        torch.testing.assert_close(offsets,action.payload_offsets[:,0])
        report['sampled_offset_range_m'] = [float(offsets.min()),float(offsets.max())]
        assert float(offsets.min())>=.12-1e-7 and float(offsets.max())<=.19+1e-7
        before = action.native_packet.values.clone()
        env.reset(env_ids=torch.tensor([0],device='cuda:0'))
        assert torch.equal(before[1:],action.native_packet.values[1:])
        torch.testing.assert_close(action.native_packet.values[0],action.native_packet.fresh()[0])
        report.update(passed=True,partial_reset_preserved_survivors=True)
    except Exception as error:
        report['error'] = repr(error)
    finally:
        if env is not None:
            env.close()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
