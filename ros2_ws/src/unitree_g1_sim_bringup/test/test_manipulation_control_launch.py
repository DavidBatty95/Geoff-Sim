import ast
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution
from launch_ros.actions import Node


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "g1_sim.launch.py"
MANIPULATION_LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "g1_manipulation.launch.py"
)


def _description():
    spec = importlib.util.spec_from_file_location("g1_sim_manipulation_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _manipulation_description():
    spec = importlib.util.spec_from_file_location(
        "g1_stationary_manipulation_launch", MANIPULATION_LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _nodes(description):
    return {
        action._Node__node_name: action
        for timer in description.entities
        if isinstance(timer, TimerAction)
        for action in timer.actions
        if isinstance(action, Node)
    }


def _parameters(node):
    return {
        "".join(part.text for part in key): value
        for block in node._Node__parameters if isinstance(block, dict)
        for key, value in block.items()
    }


def test_arm_compositor_is_opt_in_and_is_the_only_moveit_command_route():
    description = _description()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments["run_arm_compositor"].default_value[0].text == "false"
    assert arguments["run_sim_manipulation_io"].default_value[0].text == "false"

    nodes = _nodes(description)
    compositor = nodes["g1_arm_command_compositor"]
    assert isinstance(compositor.condition, IfCondition)
    assert compositor.node_executable == "g1_arm_command_compositor"
    object_io = nodes["g1_sim_manipulation_io"]
    assert isinstance(object_io.condition, IfCondition)
    assert object_io.node_executable == "g1_sim_manipulation_io"

    bridge_parameters = _parameters(nodes["g1_mujoco_bridge"])
    assert bridge_parameters["body_command_topic"][0].text.startswith(
        "/g1/body_position_commands"
    )

    policy_parameters = _parameters(nodes["g1_policy_cmd_vel_controller"])
    selected_topic = policy_parameters["body_command_topic"][0]
    assert isinstance(selected_topic, IfElseSubstitution)
    assert selected_topic.if_value[0].text == "/g1/locomotion_position_commands"
    assert selected_topic.else_value[0].text == "/g1/body_position_commands"


def test_stationary_gate_uses_safe_manipulation_routing_and_tuning():
    description = _manipulation_description()
    simulation = next(
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    )
    arguments = dict(simulation.launch_arguments)

    assert arguments["mujoco_scene"] == "scene_29dof_manipulation_dex3.xml"
    assert arguments["mujoco_initial_keyframe"] == "manipulation_stand"
    assert arguments["run_arm_compositor"] == "true"
    assert arguments["run_sim_manipulation_io"] == "true"
    assert arguments["sim_manipulation_frame_id"] == "odom"
    assert arguments["run_hold_controller"] == "true"
    assert arguments["run_policy_controller"] == "false"
    assert arguments["bridge_arm_kp_override"] == "100.0"
    assert arguments["bridge_arm_kd_override"] == "10.0"
    assert arguments["arm_compositor_goal_tolerance"] == "0.15"
    assert len(ast.literal_eval(arguments["hold_body_positions"])) == 29

    disabled_nodes = (
        "run_sensors",
        "run_slam",
        "run_nav2",
        "run_dynamic_obstacle_tracker",
        "run_human_aware_motion_planner",
        "run_human_collision_avoidance",
        "run_locomotion_diagnostics",
    )
    assert all(arguments[name] == "false" for name in disabled_nodes)
