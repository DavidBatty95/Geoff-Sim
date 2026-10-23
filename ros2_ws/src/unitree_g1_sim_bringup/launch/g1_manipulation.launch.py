"""Stationary simulation gate for MoveIt pick-and-place development."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


MANIPULATION_HOLD_POSITIONS = [
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    -0.35, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0,
    -0.35, -0.25, 0.0, 1.0, 0.0, 0.0, 0.0,
]


def _include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    simulation = _include(
        "unitree_g1_sim_bringup",
        "g1_sim.launch.py",
        {
            "mujoco_scene": "scene_29dof_manipulation_dex3.xml",
            "run_arm_compositor": "true",
            "run_sim_manipulation_io": "true",
            "sim_manipulation_frame_id": "odom",
            "run_hold_controller": "true",
            "hold_body_positions": str(MANIPULATION_HOLD_POSITIONS),
            "run_policy_controller": "false",
            "bridge_arm_kp_override": "100.0",
            "bridge_arm_kd_override": "10.0",
            "arm_compositor_goal_tolerance": "0.15",
            "run_sensors": "false",
            "run_slam": "false",
            "run_nav2": "false",
            "run_rviz": "false",
            "run_dynamic_obstacle_tracker": "false",
            "run_human_aware_motion_planner": "false",
            "run_human_collision_avoidance": "false",
            "run_human_aware_run_logger": "false",
            "run_locomotion_diagnostics": "false",
            "run_slam_reset_monitor": "false",
            "mujoco_pedestrians_enable": "false",
            "mujoco_initial_keyframe": "manipulation_stand",
        },
    )
    move_group = _include("g1_moveit_config", "move_group.launch.py")
    manipulation = _include("g1_manipulation", "manipulation.launch.py")

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting simulation-only manipulation gate. Acquire arm authority with "
                    "`ros2 service call /g1_arm_command_compositor/set_arm_authority "
                    "std_srvs/srv/SetBool '{data: true}'` before executing MoveIt trajectories."
                )
            ),
            simulation,
            TimerAction(period=6.0, actions=[move_group]),
            TimerAction(period=10.0, actions=[manipulation]),
        ]
    )
