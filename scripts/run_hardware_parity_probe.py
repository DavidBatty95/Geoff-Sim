#!/usr/bin/env python3
"""Launch the articulated-hand profile and passively audit its ROS interfaces."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--hand-model',choices=('dex3','brainco'),default='dex3')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error('output must be inside the repository')
    if output.exists() or output.with_suffix('.simulation.log').exists():
        parser.error('output already exists; retain each attempt separately')
    output.parent.mkdir(parents=True,exist_ok=True)
    name = 'geoff-hardware-probe-'+str(os.getpid())
    settings = dict(mujoco_scene='scene_29dof_dex3.xml' if args.hand_model=='dex3' else 'scene_29dof_brainco_hardware.xml',mujoco_initial_keyframe='unitree_rl_lab_stand',
        hand_model=args.hand_model,
        robot_description_file='/workspace/ros2_ws/src/unitree_g1_description/urdf/'+('g1_29dof_dex3.urdf' if args.hand_model=='dex3' else 'g1_29dof_mode_15_brainco_hand.urdf'),
        run_rviz='false',run_nav2='false',run_slam='false',
        policy_profile='unitree_rl_lab_g1_29dof',
        policy_path='/workspace/vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx',
        onboard_only='true',publish_lidar_tf='false',publish_camera='true',camera_width='192',camera_height='108',
        camera_rate_hz='2.0',use_mujoco_lidar_pose='false',use_cmd_vel_lidar_fallback='false',
        startup_current_pose_hold_time='0.0',startup_blend_time='0.0',startup_hold_time='0.0',
        startup_max_recovery_time='0.0',policy_action_ramp_time='0.0',mujoco_reset_on_controller_start='false')
    environment = dict(os.environ,COLCON_INSTALL_BASE='/workspace/ros2_ws/.verification/install',UNITREE_G1_CONTAINER_NAME=name)
    process = None
    try:
        with output.with_suffix('.simulation.log').open('w') as log:
            process = subprocess.Popen([str(ROOT/'scripts/bringup_full_sim.sh'),*[f'{key}:={value}' for key,value in settings.items()]],
                                       cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(8.)
            result = subprocess.run(['docker','compose','run','--rm','--no-deps','unitree-g1-sim','bash','-lc',
                'source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/.verification/install/setup.bash && '
                 'exec python3 /workspace/scripts/audit_sim_interfaces.py "$@"','bash','--require-hands',
                 '--hand-model',args.hand_model,
                '--duration',str(args.duration),'--output','/workspace/'+str(output.relative_to(ROOT))],cwd=ROOT,capture_output=True,text=True)
            output.with_suffix('.audit.log').write_text(result.stdout+result.stderr)
            print(result.stdout+result.stderr)
            if not output.exists():
                output.write_text(json.dumps(dict(passed=False,error='audit process failed',return_code=result.returncode),indent=2)+'\n')
            return result.returncode
    finally:
        if process is not None:
            try:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=15.)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
            subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


if __name__=='__main__':
    raise SystemExit(main())
