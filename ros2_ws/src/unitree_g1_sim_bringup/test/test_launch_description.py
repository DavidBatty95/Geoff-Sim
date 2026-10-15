import importlib.util
import unittest
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "g1_sim.launch.py"
CONFIG_DIR = Path(__file__).parents[1] / "config"
NAV2_CONFIG = CONFIG_DIR / "nav2_livox.yaml"
RVIZ_CONFIG = Path(__file__).parents[1] / "rviz" / "g1_sim.rviz"
PROFILE_NAMES = ["perception_only", "balanced", "conservative", "hardware_safe", "oracle"]
SOCIAL_NODE_NAMES = {
    "dynamic_obstacle_tracker",
    "human_aware_motion_planner",
    "human_collision_avoidance",
}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("g1_sim_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class LaunchDescriptionTests(unittest.TestCase):
    def test_goal_checker_rejects_failed_transform_without_relaxing_tolerances(self):
        checker = yaml.safe_load(NAV2_CONFIG.read_text())["controller_server"]["ros__parameters"]["general_goal_checker"]
        self.assertEqual(checker["plugin"], "unitree_social_costmap::ValidatedGoalChecker")
        self.assertEqual(checker["xy_goal_tolerance"], .45)
        self.assertEqual(checker["yaw_goal_tolerance"], .75)
        controller = yaml.safe_load(NAV2_CONFIG.read_text())["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(controller["plugin"], "unitree_social_costmap::ValidatedDWBLocalPlanner")

    def test_onboard_view_shows_measured_sensors_map_and_predictions(self):
        path = Path(__file__).parents[1] / "rviz" / "g1_onboard_waypoints.rviz"
        manager = yaml.safe_load(path.read_text())["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        visible = {d["Topic"]["Value"]: d for d in manager["Displays"] if d.get("Enabled") and "Topic" in d}
        for topic in ("/onboard/lidar", "/scan_raw", "/map", "/dynamic_obstacles",
                      "/camera/camera/color/image_raw", "/camera/camera/depth/image_rect_raw"):
            self.assertIn(topic, visible)
        self.assertEqual(visible["/onboard/lidar"]["Topic"]["Reliability Policy"], "Best Effort")
        self.assertNotIn("/simulation/static_map", visible)
        self.assertNotIn("/livox/lidar_world", visible)

    def test_social_waypoint_view_uses_navigation_map_in_world_frame(self) -> None:
        path = Path(__file__).parents[1] / "rviz" / "g1_social_waypoints.rviz"
        manager = yaml.safe_load(path.read_text())["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "world")
        maps = [display for display in manager["Displays"] if display["Class"] == "rviz_default_plugins/Map"]
        self.assertEqual(maps[0]["Topic"]["Value"], "/simulation/static_map")
        self.assertEqual(manager["Views"]["Current"]["Target Frame"], "base_footprint")

    def test_velocity_smoother_array_rewrites_preserve_other_axes(self) -> None:
        from launch import LaunchContext
        rewrite = _load_launch_module().IndexedParameterRewrites(
            source_file=str(NAV2_CONFIG), convert_types=True,
            param_rewrites={"velocity_smoother.ros__parameters.max_velocity.1": "0.35"},
        )
        output = Path(rewrite.perform(LaunchContext()))
        try:
            values = yaml.safe_load(output.read_text())["velocity_smoother"]["ros__parameters"]["max_velocity"]
            self.assertEqual(values, [0.45, 0.35, 0.45])
        finally:
            output.unlink()

    def test_dynamic_route_parameters_are_inserted_when_absent_on_humble(self) -> None:
        from launch import LaunchContext
        rewrites = {
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": str(CONFIG_DIR / "nav2_dynamic_replanning.xml"),
            "planner_server.ros__parameters.GridBased.max_planning_time": "0.15",
            "global_costmap.global_costmap.ros__parameters.people_layer.prediction_horizon": "1.2",
        }
        rewrite = _load_launch_module().IndexedParameterRewrites(
            source_file=str(NAV2_CONFIG), convert_types=True,
            param_rewrites=rewrites, add_missing_keys=tuple(rewrites))
        output = Path(rewrite.perform(LaunchContext()))
        try:
            data = yaml.safe_load(output.read_text())
            self.assertEqual(data["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"], rewrites[next(iter(rewrites))])
            self.assertEqual(data["planner_server"]["ros__parameters"]["GridBased"]["max_planning_time"], .15)
            self.assertEqual(data["global_costmap"]["global_costmap"]["ros__parameters"]["people_layer"]["prediction_horizon"], 1.2)
        finally:
            output.unlink()

    def test_rviz_exposes_social_mission_and_hides_disabled_camera_panels(self) -> None:
        config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
        displays = config["Visualization Manager"]["Displays"]
        by_name = {display["Name"]: display for display in displays}

        self.assertEqual(
            by_name["Benchmark Mission Goal"]["Topic"]["Value"],
            "/benchmark/current_goal",
        )
        self.assertEqual(
            by_name["Social Demo Status (Oracle)"]["Topic"]["Value"],
            "/benchmark/social_demo_oracle",
        )
        self.assertFalse(by_name["Dynamic Obstacles"]["Namespaces"]["dynamic_obstacle_labels"])
        self.assertFalse(by_name["Dynamic Obstacles"]["Namespaces"]["tracked_object_paths"])
        self.assertFalse(
            by_name["Human-Aware Candidate Trajectories"]["Namespaces"]["human_aware_state"]
        )
        self.assertFalse(by_name["Projected Livox /scan"]["Enabled"])
        self.assertTrue(by_name["SLAM Occupancy Map"]["Enabled"])
        self.assertFalse(by_name["Tracked Object Footprints"]["Enabled"])
        self.assertFalse(by_name["Predicted Object Risk"]["Enabled"])
        self.assertFalse(by_name["Human Social Risk Grid"]["Enabled"])
        self.assertTrue(by_name["Nav2 Global Path"]["Enabled"])
        self.assertFalse(by_name["Nav2 Local Plan"]["Enabled"])
        self.assertFalse(by_name["MuJoCo Livox mj_multiRay"]["Enabled"])
        self.assertFalse(by_name["D435i Color"]["Enabled"])
        self.assertFalse(by_name["D435i Camera View"]["Enabled"])
        self.assertFalse(by_name["D435i Depth"]["Enabled"])
        self.assertEqual(
            config["Visualization Manager"]["Views"]["Current"]["Target Frame"],
            "base_footprint",
        )
        self.assertTrue(config["Window Geometry"]["Hide Left Dock"])
        self.assertTrue(config["Window Geometry"]["Hide Right Dock"])

    def test_key_launch_arguments_are_declared(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        arguments = {
            action.name: action
            for action in launch_description.entities
            if isinstance(action, DeclareLaunchArgument)
        }

        for name in [
            "run_sim",
            "mujoco_scene",
            "mujoco_pedestrian_scenario",
            "mujoco_pedestrian_seed",
            "run_slam",
            "run_nav2",
            "run_human_collision_avoidance",
            "human_collision_input_cmd_vel_topic",
            "social_navigation_params_file",
            "social_navigation_method_params_file",
            "run_locomotion_diagnostics",
            "run_point_goal",
            "run_base_approach",
            "base_approach_params_file",
            "policy_path",
            "policy_profile",
        ]:
            self.assertIn(name, arguments)

        self.assertEqual(arguments["mujoco_scene"].default_value[0].text, "scene_29dof_extreme_lab.xml")
        self.assertEqual(arguments["run_human_collision_avoidance"].default_value[0].text, "true")
        self.assertEqual(
            arguments["human_collision_input_cmd_vel_topic"].default_value[0].text,
            "/cmd_vel_human_aware",
        )
        self.assertEqual(arguments["run_lidar_person_detector"].default_value[0].text, "false")
        self.assertEqual(arguments["tracker_publish_candidate_predictions"].default_value[0].text, "false")
        self.assertEqual(arguments["run_locomotion_diagnostics"].default_value[0].text, "true")
        self.assertEqual(arguments["run_waypoint_explorer"].default_value[0].text, "false")
        self.assertEqual(arguments["run_base_approach"].default_value[0].text, "false")
        self.assertEqual(
            arguments["policy_path"].default_value[0].text,
            _load_launch_module().DEFAULT_POLICY_PATH,
        )
        expected_defaults = {
            "cmd_vel_timeout": "0.4",
            "lowstate_timeout": "0.1",
            "odom_timeout": "0.2",
            "cmd_vel_deadband": "0.05",
            "axis_exclusive_cmd_vel": "false",
            "axis_exclusive_yaw_threshold": "0.05",
            "allow_reverse_cmd_vel": "true",
            "startup_current_pose_hold_time": "0.0",
            "startup_blend_time": "0.0",
            "startup_hold_time": "0.0",
            "startup_max_recovery_time": "0.0",
            "policy_action_ramp_time": "0.0",
            "policy_action_rate_limit": "8.0",
            "mujoco_initial_keyframe": "policy_stand",
            "mujoco_pedestrian_scenario": "waypoints",
            "mujoco_pedestrian_seed": "42",
            "max_lin_vel_y": "0.12",
            "max_ang_vel_z": "0.35",
            "policy_profile": "mjlab",
            "livox_scan_points": "600",
        }
        for name, expected in expected_defaults.items():
            self.assertEqual(arguments[name].default_value[0].text, expected)
        self.assertEqual(
            arguments["social_navigation_params_file"].default_value[0].text,
            str(CONFIG_DIR / "social_navigation_perception_only_v1.yaml"),
        )
        self.assertEqual(
            arguments["social_navigation_method_params_file"].default_value[0].text,
            str(CONFIG_DIR / "social_navigation_method_default_v1.yaml"),
        )

    def test_social_navigation_method_profiles_isolate_baseline_behavior(self) -> None:
        reactive = yaml.safe_load(
            (CONFIG_DIR / "social_navigation_method_reactive_geometric_v1.yaml").read_text()
        )["human_collision_avoidance"]["ros__parameters"]
        temporal = yaml.safe_load(
            (CONFIG_DIR / "social_navigation_method_deterministic_temporal_v1.yaml").read_text()
        )["human_collision_avoidance"]["ros__parameters"]

        self.assertGreater(reactive["prediction_min_confidence"], 1.0)
        self.assertEqual(reactive["predicted_repulsion_range"], 0.0)
        self.assertFalse(reactive["enable_open_space_bias"])
        self.assertEqual(temporal["prediction_uncertainty_scale"], 0.0)
        self.assertGreater(temporal["prediction_collision_radius"], 0.0)
        self.assertFalse(temporal["enable_open_space_bias"])

    def test_social_navigation_profiles_select_sources_and_preserve_radius_ordering(self) -> None:
        for profile_name in PROFILE_NAMES:
            profile_path = CONFIG_DIR / f"social_navigation_{profile_name}_v1.yaml"
            with self.subTest(profile=profile_name), profile_path.open(encoding="utf-8") as stream:
                profile = yaml.safe_load(stream)

            self.assertEqual(set(profile), SOCIAL_NODE_NAMES)
            tracker = profile["dynamic_obstacle_tracker"]["ros__parameters"]
            planner = profile["human_aware_motion_planner"]["ros__parameters"]
            safety = profile["human_collision_avoidance"]["ros__parameters"]

            oracle = profile_name == "oracle"
            self.assertEqual(tracker["use_mujoco_people_detections"], oracle)
            self.assertEqual(tracker["mujoco_people_authoritative"], oracle)
            self.assertEqual(safety["use_mujoco_people"], oracle)
            self.assertFalse(planner["use_legacy_person_inputs"])
            self.assertEqual(planner["tracked_persons_topic"], "/tracked_persons")
            self.assertEqual(planner["predicted_trajectories_topic"], "/predicted_person_trajectories")
            self.assertLess(safety["critical_radius"], safety["danger_radius"])
            self.assertLess(safety["emergency_stop_radius"], safety["critical_radius"])

    def test_social_navigation_profile_is_last_parameter_source_for_each_node(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        nodes = {
            action._Node__node_name: action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node) and action._Node__node_name in SOCIAL_NODE_NAMES
        }

        self.assertEqual(set(nodes), SOCIAL_NODE_NAMES)
        for name, node in nodes.items():
            with self.subTest(node=name):
                final_source = node._Node__parameters[-1]
                self.assertIsInstance(final_source, ParameterFile)
                self.assertIsInstance(final_source.param_file[-1], LaunchConfiguration)
                expected = (
                    "social_navigation_method_params_file"
                    if name == "human_collision_avoidance"
                    else "social_navigation_params_file"
                )
                self.assertEqual(final_source.param_file[-1].variable_name[0].text, expected)

    def test_sim_process_is_conditioned_and_delayed_nodes_exist(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        sim_processes = [action for action in launch_description.entities if isinstance(action, ExecuteProcess)]
        timers = [action for action in launch_description.entities if isinstance(action, TimerAction)]

        self.assertEqual(len(sim_processes), 1)
        self.assertIsInstance(sim_processes[0].condition, IfCondition)
        self.assertGreaterEqual(len(timers), 3)
        periods = sorted(float(timer.period) for timer in timers)
        self.assertIn(2.0, periods)
        self.assertIn(5.0, periods)
        self.assertIn(8.0, periods)

    def test_policy_node_uses_hard_safety_limits(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        policy_node = next(
            action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node)
            and action._Node__node_name == "g1_policy_cmd_vel_controller"
        )
        # Explicit launch limits remain last, above the shared profile file.
        self.assertIsInstance(policy_node._Node__parameters[-1], dict)
        parameters = {
            "".join(part.text for part in key): value
            for key, value in policy_node._Node__parameters[-1].items()
        }

        self.assertEqual(parameters["action_clip"], 1.0)
        self.assertLessEqual(parameters["policy_inference_deadline_sec"], 0.05)
        self.assertEqual(parameters["startup_upright_tilt_threshold"], 0.35)
        self.assertEqual(parameters["cmd_vel_deadband_hysteresis"], 0.01)

    def test_prediction_sweeps_are_not_flattened_into_costmaps(self) -> None:
        config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
        local_layer = config["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
        global_layer = config["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]

        self.assertEqual(local_layer["observation_sources"], "scan tracked_obstacles")
        self.assertEqual(global_layer["observation_sources"], "scan tracked_obstacles")
        self.assertNotIn("predicted_obstacles", local_layer)
        self.assertNotIn("predicted_obstacles", global_layer)

    def test_base_approach_is_opt_in_and_uses_the_grove_config(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        approach_node = next(
            action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node) and action._Node__node_name == "g1_base_approach"
        )

        self.assertIsInstance(approach_node.condition, IfCondition)
        self.assertEqual(approach_node.node_package, "g1_locomotion")
        self.assertEqual(approach_node.node_executable, "g1_base_approach")
        parameter_file = approach_node._Node__parameters[0]
        self.assertIsInstance(parameter_file, ParameterFile)
        self.assertIsInstance(parameter_file.param_file[-1], LaunchConfiguration)
        self.assertEqual(
            parameter_file.param_file[-1].variable_name[0].text,
            "base_approach_params_file",
        )

    def test_nav2_velocity_limits_match_policy_safety_envelope(self) -> None:
        config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
        controller = config["controller_server"]["ros__parameters"]
        planner = controller["FollowPath"]
        smoother = config["velocity_smoother"]["ros__parameters"]

        self.assertEqual(controller["progress_checker"]["required_movement_radius"], 0.15)
        self.assertEqual(controller["controller_frequency"], 15.0)
        self.assertEqual(
            config["local_costmap"]["local_costmap"]["ros__parameters"]["update_frequency"],
            10.0,
        )
        self.assertEqual(controller["progress_checker"]["movement_time_allowance"], 20.0)
        self.assertEqual(planner["max_vel_x"], 0.45)
        self.assertEqual(planner["min_vel_x"], 0.0)
        self.assertLessEqual(planner["max_vel_theta"], 0.45)
        self.assertEqual(planner["acc_lim_x"], 0.35)
        self.assertEqual(planner["acc_lim_theta"], 0.35)
        self.assertGreaterEqual(controller["min_x_velocity_threshold"], 0.05)
        self.assertEqual(smoother["max_velocity"], [0.45, 0.0, 0.45])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.45])
        self.assertEqual(smoother["max_accel"], [0.35, 0.0, 0.35])
        self.assertEqual(smoother["deadband_velocity"], [0.05, 0.0, 0.05])
        self.assertLessEqual(smoother["velocity_timeout"], 0.4)

    def test_global_planner_respects_nonholonomic_motion(self) -> None:
        config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
        planner = config["planner_server"]["ros__parameters"]["GridBased"]

        self.assertEqual(planner["plugin"], "nav2_smac_planner/SmacPlannerHybrid")
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertGreater(planner["minimum_turning_radius"], 0.0)

    def test_waypoint_explorer_prefers_short_forward_sector_goals(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        waypoint_node = next(
            action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node) and action._Node__node_name == "g1_waypoint_explorer"
        )
        parameters = {
            "".join(part.text for part in key): value
            for key, value in waypoint_node._Node__parameters[0].items()
        }

        self.assertEqual(parameters["goal_distance_min"], 1.5)
        self.assertEqual(parameters["goal_distance_max"], 2.4)
        self.assertLessEqual(parameters["max_goal_heading_error"], 0.8)
        self.assertEqual(parameters["min_goal_distance"], 1.2)
        self.assertTrue(parameters["forward_sector_goals_only"])


if __name__ == "__main__":
    unittest.main()
