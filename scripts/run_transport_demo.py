#!/usr/bin/env python3
"""Launch the physical RGB-D transport demonstration and collect its evidence."""
import argparse
from datetime import datetime,timezone
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time
import copy
import math
import random
import secrets
import re
import yaml
import sys

from avoidance_worlds import ROOT,SCENES
from demo_suite import case_catalog
from transport_worlds import TRANSPORT_WORLDS,INITIAL_KEYFRAME,generate
from transport_evidence import capture_source
from analyze_response_trace import write_report

sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.policy_response import NATIVE_CONTACT_POLICY_SHA256,NATIVE_CONTACT_PROFILE


def select_transport_task(pairs,task='random',seed=None):
    """Choose IDs/colors only; never read simulator layout or object positions."""
    if task=='all':
        return dict(pairs=copy.deepcopy(pairs),task_seed=None,description='Deliver all matching box/goal pairs',mode='all')
    boxes = {pair['color']:pair['object_id'] for pair in pairs}
    goals = {pair.get('destination_color',pair['color']):pair['destination_id'] for pair in pairs}
    if task=='random':
        seed = secrets.randbits(32) if seed is None else seed
        rng = random.Random(seed)
        box_color,goal_color = rng.choice(sorted(boxes)),rng.choice(sorted(goals))
    else:
        colors = task.split(':')
        if len(colors)!=2 or colors[0] not in boxes or colors[1] not in goals:
            raise ValueError('task must be random, all, or an available box-color:goal-color pair')
        box_color,goal_color = colors
        seed = None
    return dict(pairs=[dict(object_id=boxes[box_color],destination_id=goals[goal_color],
                           color=box_color,destination_color=goal_color)],
                task_seed=seed,mode=task,description=f'Put the {box_color} box on the {goal_color} goal')


def transport_navigation_profile(base,extra_clearance,*,early_overtaking=False):
    """Transport-only human buffers and a shared bound on diagonal escapes."""
    if not math.isfinite(extra_clearance) or not 0.<=extra_clearance<=.5:
        raise ValueError('extra people clearance must be between 0 and 0.5 m')
    profile = copy.deepcopy(base)
    planner = profile.setdefault('human_aware_motion_planner',{}).setdefault('ros__parameters',{})
    supervisor = profile.setdefault('human_collision_avoidance',{}).setdefault('ros__parameters',{})
    planner['body_prediction_margin'] = planner.get('body_prediction_margin',.25)+extra_clearance
    supervisor['prediction_collision_radius'] = supervisor.get('prediction_collision_radius',1.)+extra_clearance
    planner['comfort_radius'] = max(planner.get('comfort_radius',1.8),2.)
    planner['prediction_activation_clearance'] = max(planner.get('prediction_activation_clearance',3.2),3.5)
    planner['path_reference_speed'] = min(planner.get('path_reference_speed',.6),.45)
    planner['overtaking_terminal_horizon'] = 6. if early_overtaking else 0.
    # Independent .45 m/s reverse/lateral components previously produced a
    # .64 m/s loaded diagonal escape and a fall. Bound the resultant consistently
    # in rollout, final supervision and execution, retaining single-axis escapes.
    limits = [profile.setdefault(name,{}).setdefault('ros__parameters',{}) for name in
              ('human_aware_motion_planner','human_collision_avoidance','g1_policy_cmd_vel_controller')]
    speed = min(.45,*(parameters.get('max_planar_speed',.65) for parameters in limits))
    for parameters in limits:
        parameters['max_planar_speed'] = speed
    return profile


def pedestrian_settings(people,seed,start_delay):
    if not 0<=people<=8 or not math.isfinite(start_delay) or not 0.<=start_delay<=120.:
        raise ValueError('people must be 0..8 and pedestrian start delay must be 0..120 seconds')
    return dict(mujoco_pedestrians_enable='true',mujoco_pedestrians_count=str(people),
                mujoco_pedestrian_seed=str(seed),mujoco_pedestrian_scenario='social_natural_plaza',
                mujoco_pedestrian_start_delay=str(float(start_delay)),mujoco_pedestrian_start_gate='false')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',help='Unique artifact-directory name supplied by an experiment runner')
    parser.add_argument('--world',choices=TRANSPORT_WORLDS,default='transport_dual')
    parser.add_argument('--people',type=int,choices=range(9),default=0)
    parser.add_argument('--pedestrian-start-delay',type=float,default=10.,
                        help='Seconds before natural walking starts (0..120); independent of robot goals')
    parser.add_argument('--tracker-wall-evidence',action=argparse.BooleanOptionalAction,default=True,
                        help='Use structural sheet evidence to reject wall-shaped person hypotheses; toggle for A/B tests')
    parser.add_argument('--tracker-body-wall-sheets',action=argparse.BooleanOptionalAction,default=True,
                        help='Use strict body-height sheet geometry where upper wall returns are outside the LiDAR field of view')
    parser.add_argument('--early-overtaking-avoidance',action=argparse.BooleanOptionalAction,default=False,
                        help='Experimental: penalize forward routes a faster following walker will catch beyond the short rollout')
    parser.add_argument('--predictive-refuge',action=argparse.BooleanOptionalAction,default=False,
                        help='Experimental bounded multi-stage clearance search, including during inspection stops')
    parser.add_argument('--policy-device-clock',action=argparse.BooleanOptionalAction,default=True,
                        help='Physical-tick policy cadence for the native simulator; retains wall-clock watchdogs')
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--layout-seed',type=int,default=0,help='Simulator-only mirrored search/building layout')
    parser.add_argument('--task',help='random, all, or box-color:goal-color (e.g. red:blue); exploration defaults to random')
    parser.add_argument('--task-seed',type=int,help='Replay random objective selection; a fresh seed is recorded by default')
    parser.add_argument('--explore-unknown',action='store_true',help='Search cube first using observed-map viewpoints; find destination after pickup')
    parser.add_argument('--slam-loop-closing',action=argparse.BooleanOptionalAction,default=None,
                        help='Online loop closure; defaults off for repetitive unknown-building exploration')
    parser.add_argument('--search-max-viewpoints',type=int,default=20)
    parser.add_argument('--people-clearance-extra',type=float,default=.1,
                        help='Additional planner/supervisor human clearance in metres for runs with people (0..0.5)')
    parser.add_argument('--duration',type=float,default=420.)
    parser.add_argument('--headless',action='store_true',help='Disable RViz; the native MuJoCo viewer may still open')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--fast-start',action='store_true')
    parser.add_argument('--application-timing',action='store_true',help='Sample native command application and log LowCmd publication brackets')
    parser.add_argument('--arm-executor-threads',type=int,choices=(1,2),default=2,
                        help='Compare compositor executor scheduling with asynchronous arm actions')
    parser.add_argument('--arm-repeat-period',type=float,default=0.,
                        help='Coalesce identical composed targets for this many seconds (0 disables, max .05)')
    parser.add_argument('--camera-width',type=int,default=192,
                        help='Native raycast image width; 192 keeps dual-scene acquisition within the confirmation interval')
    parser.add_argument('--camera-height',type=int,default=108)
    parser.add_argument('--carry-posture',choices=('ik','payload_v1'),default='ik',
                        help='Explicit cycle-01 trained posture or historical Cartesian IK carry')
    parser.add_argument('--policy',type=Path,help='Repository-local 480/29 ONNX checkpoint; uses measured arm observations')
    args = parser.parse_args()
    if args.run_id is not None and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}',args.run_id):
        parser.error('--run-id must be a simple alphanumeric, underscore or hyphen name')
    if not math.isfinite(args.arm_repeat_period) or not 0.<=args.arm_repeat_period<=.05:
        parser.error('--arm-repeat-period must be finite and between 0 and .05 seconds')
    if args.policy is None and args.explore_unknown:
        candidates = (
            ('native_contact_cycle_05_20261012',NATIVE_CONTACT_POLICY_SHA256),
            ('native_delay_cycle_03_20261006','25bab45eda339e314c7b15156ba9e3695cfdcfffa157b990a720e59d3d3e65cc'),
        )
        for name,expected in candidates:
            native_policy = ROOT/'vendor/mjlab/logs/payload_training'/name/'policy.onnx'
            if native_policy.is_file() and hashlib.sha256(native_policy.read_bytes()).hexdigest()==expected:
                args.policy = native_policy
                break
    try:
        crowd_settings = pedestrian_settings(args.people,args.seed,args.pedestrian_start_delay)
    except ValueError as exc:
        parser.error(str(exc))
    if args.search_max_viewpoints<=0:
        parser.error('--search-max-viewpoints must be positive')
    if not math.isfinite(args.people_clearance_extra) or not 0.<=args.people_clearance_extra<=.5:
        parser.error('--people-clearance-extra must be between 0 and 0.5 m')
    if args.camera_width<=0 or args.camera_height<=0:
        parser.error('camera dimensions must be positive')
    if args.policy is not None:
        args.policy = args.policy.resolve()
        if not args.policy.is_file() or not args.policy.is_relative_to(ROOT):
            parser.error('--policy must be an existing file inside the repository')
    generate(args.layout_seed)
    available_pairs = json.loads((SCENES/f'transport_pairs_{args.world}.json').read_text())['pairs']
    try:
        objective = select_transport_task(available_pairs,args.task or ('random' if args.explore_unknown else 'all'),args.task_seed)
    except ValueError as exc:
        parser.error(str(exc))
    print(f'Objective: {objective["description"]} (task seed: {objective["task_seed"]})',flush=True)
    run = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo'/(args.run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    run.mkdir(parents=True)
    (run/'objective.json').write_text(json.dumps(objective,indent=2)+'\n')
    install = '/workspace/ros2_ws/.verification/install'
    if not args.fast_start:
        subprocess.run(['docker','compose','run','--rm','-e',f'COLCON_INSTALL_BASE={install}',
            '-e','COLCON_BUILD_BASE=/workspace/ros2_ws/.verification/build','-e','COLCON_LOG_BASE=/workspace/ros2_ws/.verification/log',
            'unitree-g1-sim','/workspace/scripts/build_ws.sh','--packages-select','g1_msgs','unitree_g1_mujoco_bridge',
            'unitree_g1_sim_bringup','unitree_g1_description','dynamic_obstacle_benchmarks'],cwd=ROOT,check=True)
        subprocess.run(['docker','compose','run','--rm','unitree-g1-sim','/workspace/scripts/build_unitree_mujoco.sh'],cwd=ROOT,check=True)
        generate(args.layout_seed)  # Build regenerates fixtures at the default layout.
    command = case_catalog(run,args.seed,True,True,3,onboard_people=max(1,args.people))['onboard-waypoints'].command
    settings = dict(token.split(':=',1) for index,token in enumerate(command) if index and command[index-1]=='--sim-arg')
    container_run = '/workspace/'+str(run.relative_to(ROOT))
    base_file = ROOT/Path(settings['social_navigation_params_file']).relative_to('/workspace')
    profile = yaml.safe_load(base_file.read_text())
    if args.people:
        profile = transport_navigation_profile(profile,args.people_clearance_extra,early_overtaking=args.early_overtaking_avoidance)
        if args.predictive_refuge:
            profile['human_aware_motion_planner']['ros__parameters'].update(
                kinodynamic_search_enabled=True,kinodynamic_budget_ms=12.)
    profile.setdefault('dynamic_obstacle_tracker',{}).setdefault('ros__parameters',{})['reject_wall_supported_clusters'] = args.tracker_wall_evidence
    profile['dynamic_obstacle_tracker']['ros__parameters']['reject_body_wall_sheets'] = args.tracker_body_wall_sheets
    if args.policy is not None and hashlib.sha256(args.policy.read_bytes()).hexdigest()==NATIVE_CONTACT_POLICY_SHA256:
        for node in ('human_aware_motion_planner','human_collision_avoidance','g1_policy_cmd_vel_controller'):
            profile.setdefault(node,{}).setdefault('ros__parameters',{})['response_profile'] = NATIVE_CONTACT_PROFILE
        profile['human_aware_motion_planner']['ros__parameters']['max_clockwise_angular_near_human'] = .6
    profile_bytes = yaml.safe_dump(profile,sort_keys=False).encode()
    (run/'transport_navigation.yaml').write_bytes(profile_bytes)
    navigation_profile_hash = hashlib.sha256(profile_bytes).hexdigest()
    settings['social_navigation_params_file'] = container_run+'/transport_navigation.yaml'
    settings.update(mujoco_scene=f'scene_29dof_transport_{args.world}.xml',mujoco_initial_keyframe=INITIAL_KEYFRAME,
        mujoco_pedestrians_enable='true',mujoco_pedestrians_count=str(args.people),mujoco_pedestrian_seed=str(args.seed),
        run_nav2='true',run_waypoint_explorer='false',run_waypoint_patrol='false',run_frontier_explorer='false',
        run_human_aware_motion_planner='true',run_human_collision_avoidance='true',run_dynamic_obstacle_tracker='true',
        run_arm_compositor='true',run_sim_manipulation_io='true',arm_compositor_goal_tolerance='0.08',
        bridge_arm_kp_override='160.0',bridge_arm_kd_override='4.0',
        bridge_arm_gains_require_authority='true',
        external_arm_observation_mask='true',
        run_rviz='false' if args.headless or args.no_rviz else 'true',publish_camera='true',
        camera_width=str(args.camera_width),camera_height=str(args.camera_height),camera_rate_hz='2.0',
        camera_mount_pitch_deg=str(TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.)),
        startup_current_pose_hold_time='0.0',startup_blend_time='0.0',startup_hold_time='1.0',
        startup_max_recovery_time='0.0',policy_action_ramp_time='0.0',mujoco_reset_on_controller_start='false',
        run_human_aware_run_logger='true',human_aware_run_log_path=container_run+'/trace.jsonl')
    settings.update(crowd_settings)
    settings['policy_use_device_tick'] = str(args.policy_device_clock).lower()
    settings['mujoco_application_timing'] = str(args.application_timing).lower()
    settings['arm_compositor_executor_threads'] = str(args.arm_executor_threads)
    settings['arm_compositor_repeat_publish_period'] = str(args.arm_repeat_period)
    settings['slam_loop_closing'] = str(args.slam_loop_closing if args.slam_loop_closing is not None else not args.explore_unknown).lower()
    if args.policy is not None:
        settings.update(policy_path='/workspace/'+str(args.policy.relative_to(ROOT)),external_arm_observation_mask='false')
    print(f'Locomotion policy: {settings["policy_path"]}; posture: {args.carry_posture}',flush=True)
    name = 'geoff-transport-'+str(os.getpid())
    env = dict(os.environ,COLCON_INSTALL_BASE=install,UNITREE_G1_CONTAINER_NAME=name)
    sources = [Path(__file__),ROOT/'scripts/transport_worlds.py',
               *[ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge'/name
                 for name in ('transport_demo.py','transport_perception.py','arm_kinematics.py','human_aware_motion_planner.py')]]
    if args.policy is not None:
        sources.append(args.policy)
    hashes = {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (run/'manifest.json').write_text(json.dumps(dict(world=args.world,people=args.people,seed=args.seed,
        duration_s=args.duration,build_reused=args.fast_start,carry_posture=args.carry_posture,
        people_clearance_extra_m=args.people_clearance_extra if args.people else 0.,
        navigation_profile_sha256=navigation_profile_hash,
        pedestrian_scenario=crowd_settings['mujoco_pedestrian_scenario'],pedestrian_start_delay_s=args.pedestrian_start_delay,
        tracker_wall_evidence=args.tracker_wall_evidence,
        tracker_body_wall_sheets=args.tracker_body_wall_sheets,
        early_overtaking_avoidance=args.early_overtaking_avoidance if args.people else False,
        predictive_refuge=args.predictive_refuge if args.people else False,
        layout_seed=args.layout_seed if args.world=='transport_search' or TRANSPORT_WORLDS[args.world].get('building') else None,
        objective=objective,
        mission_kind='explore_pick_find_place' if args.explore_unknown else 'transport',
        search_max_viewpoints=args.search_max_viewpoints if args.explore_unknown else None,
        camera_pitch_deg=TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.),
        settings=settings,source_sha256=hashes),indent=2)+'\n')
    capture_source(ROOT,run,args.policy)
    process = None
    try:
        with (run/'simulation.log').open('w') as log:
            process = subprocess.Popen([str(ROOT/'scripts/bringup_full_sim.sh'),*[f'{k}:={v}' for k,v in settings.items()]],
                                       cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(4.)
            pairs = container_run+'/objective.json'
            shell = ('source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/.verification/install/setup.bash && '
                     f'python3 /workspace/scripts/transport_evidence.py {container_run}/runtime_provenance.json && '
                     'exec python3 -m unitree_g1_mujoco_bridge.transport_demo "$@"')
            result = subprocess.run(['docker','compose','run','--rm','--no-deps','--name',name+'-mission','unitree-g1-sim','bash','-lc',shell,'bash',
                '--pairs',pairs,'--output',container_run+'/result.json','--duration',str(args.duration),'--people',str(args.people),
                '--carry-posture',args.carry_posture,
                '--camera-pitch-deg',str(TRANSPORT_WORLDS[args.world].get('camera_pitch_deg',0.)),
                '--search-max-viewpoints',str(args.search_max_viewpoints),
                '--response-profile',profile.get('g1_policy_cmd_vel_controller',{}).get('ros__parameters',{}).get('response_profile','legacy'),
                *(['--explore-unknown'] if args.explore_unknown else [])],cwd=ROOT)
            print(f'Transport artifacts: {run}',flush=True)
            return result.returncode
    finally:
        if process is not None:
            try:
                os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
            subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            subprocess.run(['docker','rm','-f',name+'-mission'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            if (run/'trace.jsonl').is_file():
                write_report(run/'trace.jsonl',run/'response_diagnostics.json')
                if args.application_timing:
                    subprocess.run([sys.executable,str(ROOT/'scripts/analyze_command_application.py'),str(run),
                                    '--output',str(run/'command_application_diagnostics.json')],check=False)


if __name__=='__main__':
    raise SystemExit(main())
