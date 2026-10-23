from __future__ import annotations

import importlib.util
import io
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest


def load_suite() -> ModuleType:
    path = Path(__file__).with_name("demo_suite.py")
    spec = importlib.util.spec_from_file_location("demo_suite", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_presets_are_scored_and_deduplicated(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=7, headless=True)
    selected = suite.select_cases(
        catalog, "locomotion", ["fixed-forward", "mock-avoidance"]
    )

    assert [case.name for case in selected] == ["mock-avoidance", "fixed-forward"]
    assert all(case.scored and not case.interactive for case in selected)


def test_social_waypoints_pin_independent_wandering_and_known_map(tmp_path: Path) -> None:
    suite = load_suite()
    case = suite.case_catalog(tmp_path, seed=43, headless=True, fast_start=True, social_goals=8)["social-waypoints"]
    assert case.command[case.command.index("--goals") + 1] == "8"
    assert "mujoco_pedestrian_scenario:=social_wandering_plaza" in case.command
    assert "nav2_static_scene_map:=true" in case.command
    assert "nav2_world_costmap:=true" in case.command
    assert "nav2_planner_plugin:=nav2_smac_planner/SmacPlanner2D" in case.command
    assert "--no-rviz" in case.command
    assert case.needs_official_policy


def test_seed_is_random_by_default_and_explicit_for_replay(monkeypatch) -> None:
    suite = load_suite()
    monkeypatch.setattr(suite.secrets, "randbits", lambda bits: 123456)

    assert suite.resolve_seed(None) == 123456
    assert suite.resolve_seed(42) == 42


def test_onboard_visual_defaults_to_long_unscored_session_with_both_viewers(tmp_path):
    suite = load_suite()
    case = suite.case_catalog(tmp_path, seed=43, headless=False)["onboard-visual"]
    assert case.interactive and not case.scored
    assert "--no-rviz" not in case.command
    assert "show_mujoco_goal:=true" in case.command
    assert "publish_camera:=true" in case.command
    assert "--visual-session" in case.command
    assert float(case.command[case.command.index("--duration-seconds") + 1]) == 1800.
    assert int(case.command[case.command.index("--goals") + 1]) > 1000
    assert case.command[case.command.index("--people") + 1] == "8"
    assert "mujoco_pedestrian_scenario:=social_natural_plaza" in case.command
    assert case.required_scene == "scene_29dof_natural_crowd_dex3.xml"
    short = suite.case_catalog(tmp_path, 43, True, visual_duration_s=60., visual_goals=3)["onboard-visual"]
    assert "--no-rviz" in short.command
    assert short.command[short.command.index("--goals") + 1] == "3"


def test_interactive_case_cannot_be_mixed_with_batch_case(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=False)

    with pytest.raises(ValueError, match="interactive cases must be run alone"):
        suite.select_cases(catalog, None, ["waypoint-visual", "mock-avoidance"])


def test_official_policy_cases_pin_profile_and_disable_lateral(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=True)

    fixed = catalog["fixed-forward"]
    waypoint = catalog["waypoint-live"]
    waypoint_visual = catalog["waypoint-visual"]
    social = catalog["social-live"]
    endurance = catalog["social-endurance"]
    visual = catalog["social-visual"]
    assert "--validate" in fixed.command
    assert "unitree_rl_lab_g1_29dof" in fixed.command
    assert "policy_profile:=unitree_rl_lab_g1_29dof" in social.command
    assert "max_lin_vel_y:=0.0" in social.command
    assert "max_lin_vel_x:=0.45" in waypoint.command
    assert "max_ang_vel_z:=0.35" in waypoint.command
    assert "turning_arc_min_forward_speed:=0.06" in waypoint.command
    assert waypoint.command[waypoint.command.index("--goals") + 1] == "4"
    assert any(arg.endswith("demo_suite_waypoint_benchmark.yaml") for arg in waypoint.command)
    assert waypoint_visual.command[waypoint_visual.command.index("--goals") + 1] == "4"
    assert any(arg.endswith("demo_suite_waypoint_benchmark.yaml") for arg in waypoint_visual.command)
    assert "max_lin_vel_x:=0.45" in social.command
    assert social.command.index("max_ang_vel_z:=0.35") < social.command.index("max_ang_vel_z:=0.20")
    assert social.command.index("turning_arc_min_forward_speed:=0.06") < social.command.index(
        "turning_arc_min_forward_speed:=0.20"
    )
    assert "--startup-seconds" not in social.command
    assert any(arg.endswith("demo_suite_social_benchmark.yaml") for arg in social.command)
    assert "mujoco_pedestrian_start_delay:=30.0" in social.command
    assert "tracker_use_sensor_input:=false" in social.command
    assert "tracker_local_tracking_radius:=0.0" in social.command
    assert "livox_scan_points:=1200" in social.command
    assert any(arg.endswith("social_navigation_oracle_v1.yaml") for arg in social.command)
    assert "--require-avoidance" in endurance.command
    assert endurance.command[endurance.command.index("--people") + 1] == "4"
    assert endurance.command[endurance.command.index("--goals") + 1] == "4"
    assert endurance.command[endurance.command.index("--scene") + 1] == "scene_29dof_pedestrians.xml"
    assert endurance.command[endurance.command.index("--environment-name") + 1] == "pedestrian_plaza"
    assert "mujoco_pedestrian_scenario:=demo_endurance_plaza" in endurance.command
    assert "mujoco_pedestrian_start_delay:=0.0" in endurance.command
    assert endurance.command[endurance.command.index("--seed") + 1] == "42"
    assert "max_lin_vel_y:=0.0" in endurance.command
    assert any(arg.endswith("demo_suite_social_endurance_benchmark.yaml") for arg in endurance.command)
    assert "--no-rviz" not in visual.command
    assert "--require-avoidance" in visual.command
    assert "mujoco_pedestrian_scenario:=demo_endurance_plaza" in visual.command
    assert "tracker_use_sensor_input:=false" in visual.command
    assert any(arg.endswith("social_navigation_oracle_v1.yaml") for arg in visual.command)


def test_dry_run_catalog_uses_headless_live_benchmarks(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=True)

    assert "--no-rviz" in catalog["waypoint-live"].command
    assert "--no-rviz" in catalog["social-live"].command
    assert "--no-rviz" in catalog["social-endurance"].command


def test_fast_start_reuses_build_and_shortens_wait(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=False, fast_start=True)

    live_cases = (
        "fixed-forward",
        "waypoint-live",
        "waypoint-visual",
        "social-live",
        "social-endurance",
        "social-visual",
    )
    assert all("--skip-build" in catalog[name].command for name in live_cases)
    command = catalog["social-visual"].command
    assert "--startup-seconds" not in command
    assert "mujoco_pedestrian_start_delay:=0.0" in command


def test_social_showcase_uses_bundled_scene_without_extreme_lab_requirement(tmp_path: Path) -> None:
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, seed=42, headless=False)

    assert catalog["social-visual"].required_scene == "scene_29dof_pedestrians.xml"
    assert catalog["social-endurance"].required_scene == "scene_29dof_pedestrians.xml"
    assert catalog["social-live"].required_scene == "scene_29dof_extreme_lab.xml"


def test_validation_and_delivery_have_distinct_scopes(tmp_path):
    suite = load_suite()
    catalog = suite.case_catalog(tmp_path, 42, True)
    assert [case.scope for case in suite.select_cases(catalog, "validation", [])] == ["regression", "component"]
    assert catalog["dex3-delivery"].scope == "mobile_delivery"
    assert catalog["dex3-delivery"].delivery_result.endswith("/result.json")
    assert "mujoco==3.3.6" in catalog["dex3-contact"].command


@pytest.mark.parametrize("change", [
    {"mission_complete": False}, {"success": False}, {"data_valid": False},
    {"fallen": True}, {"contact_samples": 1}, {"dropped_objects": ["box_red"]},
    {"deliveries": []}, {"deliveries": [None]},
])
def test_delivery_gate_rejects_incomplete_or_unsafe_results(tmp_path, change):
    suite = load_suite()
    data = dict(success=True, mission_complete=True, data_valid=True, fallen=False,
                contact_samples=0, dropped_objects=[],
                deliveries=[dict(object_id="box_red", destination_id="target_red", verified=True)])
    path = tmp_path / "result.json"
    path.write_text(json.dumps(data))
    assert suite.delivery_evidence(path)["passed"]
    path.write_text(json.dumps({**data, **change}))
    assert not suite.delivery_evidence(path)["passed"]


def test_delivery_gate_rejects_missing_and_malformed_evidence(tmp_path):
    suite = load_suite()
    path = tmp_path / "result.json"
    assert not suite.delivery_evidence(path)["passed"]
    path.write_text("not json")
    assert not suite.delivery_evidence(path)["passed"]


def test_timeout_handles_output_without_a_newline():
    suite = load_suite()
    log = io.StringIO()
    code, timed_out = suite.stream_process(
        (sys.executable, "-c", "import time; print('partial', end='', flush=True); time.sleep(30)"),
        log, 1)
    assert code == 124 and timed_out
    assert "partial" in log.getvalue()
