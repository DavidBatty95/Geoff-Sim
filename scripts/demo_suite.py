#!/usr/bin/env python3
"""Run reproducible G1 simulation demos and scored benchmarks."""

from __future__ import annotations

import argparse
import codecs
import hashlib
import json
import math
import os
import secrets
import selectors
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
OFFICIAL_POLICY = (
    REPO_ROOT
    / "vendor/unitree_rl_lab/policies/g1_29dof_velocity_v0/policy.onnx"
)
DEFAULT_RESULTS = (
    REPO_ROOT / "ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite"
)
CONTACT_POLICY = REPO_ROOT / "vendor/mjlab/logs/payload_training/native_contact_cycle_05_20261012/policy.onnx"
CONTACT_POLICY_SHA256 = "4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c"


@dataclass(frozen=True)
class DemoCase:
    name: str
    description: str
    command: tuple[str, ...]
    timeout_s: int
    scored: bool = True
    interactive: bool = False
    needs_official_policy: bool = False
    required_scene: str | None = None
    scope: str = "demo"
    delivery_result: str | None = None


def case_catalog(
    run_dir: Path, seed: int, headless: bool, fast_start: bool = False, social_goals: int = 4,
    visual_duration_s: float = 1800., visual_goals: int | None = None,
    onboard_people: int = 8,
) -> dict[str, DemoCase]:
    rviz = ("--no-rviz",) if headless else ()
    official_launch = (
        "--sim-arg",
        "policy_action_rate_limit:=0.0",
        "--sim-arg",
        f"policy_path:={OFFICIAL_POLICY}",
        "--sim-arg",
        "policy_profile:=unitree_rl_lab_g1_29dof",
        "--sim-arg",
        "policy_obs_dim:=480",
        "--sim-arg",
        "policy_action_dim:=29",
        "--sim-arg",
        "mujoco_initial_keyframe:=unitree_rl_lab_stand",
        "--sim-arg",
        "max_lin_vel_y:=0.0",
    )
    waypoint_motion = (
        "--sim-arg",
        "max_lin_vel_x:=0.45",
        "--sim-arg",
        "max_ang_vel_z:=0.35",
        "--sim-arg",
        "cmd_accel_x:=0.35",
        "--sim-arg",
        "cmd_accel_yaw:=0.35",
        "--sim-arg",
        "turning_arc_min_forward_speed:=0.06",
        "--sim-arg",
        "turning_arc_min_yaw_speed:=0.12",
    )
    endurance_common = (
        str(SCRIPTS / "run_extreme_lab_person_avoidance_benchmark.sh"),
        *(("--skip-build",) if fast_start else ()),
        "--scene",
        "scene_29dof_pedestrians.xml",
        "--environment-name",
        "pedestrian_plaza",
        "--mode",
        "smoke",
        "--seed",
        str(seed),
        "--people",
        "4",
        "--goals",
        "4",
        "--minimum-score",
        "70",
        "--require-avoidance",
        "--config",
        "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_social_endurance_benchmark.yaml",
    )
    endurance_simulation = (
        *official_launch,
        *waypoint_motion,
        "--sim-arg",
        "mujoco_pedestrian_scenario:=demo_endurance_plaza",
        "--sim-arg",
        "mujoco_pedestrian_start_delay:=0.0",
        "--sim-arg",
        "max_ang_vel_z:=0.20",
        "--sim-arg",
        "turning_arc_min_forward_speed:=0.20",
        "--sim-arg",
        "tracker_use_sensor_input:=false",
        "--sim-arg",
        "tracker_local_tracking_radius:=0.0",
        "--sim-arg",
        "livox_scan_points:=1200",
        "--sim-arg",
        "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_oracle_v1.yaml",
    )
    cases = {
        "mock-avoidance": DemoCase(
            name="mock-avoidance",
            description="Deterministic person-avoidance benchmark mechanics",
            command=(
                str(SCRIPTS / "run_person_avoidance_benchmark.sh"),
                "--mode",
                "smoke",
                "--seed",
                str(seed),
                "--people",
                "4",
                "--goals",
                "3",
                "--minimum-score",
                "95",
                "--output",
                str(run_dir / "mock-avoidance/result.json"),
            ),
            timeout_s=300,
        ),
        "fixed-forward": DemoCase(
            name="fixed-forward",
            description="Official-policy forward response and stop characterization",
            command=(
                str(SCRIPTS / "run_g1_fixed_command_test.sh"),
                *(("--skip-build",) if fast_start else ()),
                "--policy",
                str(OFFICIAL_POLICY),
                "--policy-profile",
                "unitree_rl_lab_g1_29dof",
                "--initial-keyframe",
                "unitree_rl_lab_stand",
                "--scene",
                "scene_29dof_empty.xml",
                "--linear-x",
                "0.35",
                "--duration",
                "8",
                "--stop-duration",
                "5",
                "--max-linear-x",
                "0.45",
                "--cmd-accel-x",
                "0.35",
                "--no-rviz",
                "--validate",
                "--result-dir",
                str(run_dir / "fixed-forward"),
            ),
            timeout_s=480,
            needs_official_policy=True,
        ),
        "waypoint-live": DemoCase(
            name="waypoint-live",
            description="Scored map-wide reachable random waypoint navigation",
            command=(
                str(SCRIPTS / "run_extreme_lab_waypoint_benchmark.sh"),
                *(("--skip-build",) if fast_start else ()),
                "--seed",
                str(seed),
                "--goals",
                "4",
                "--minimum-score",
                "80",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_waypoint_benchmark.yaml",
                "--result-dir",
                str(run_dir / "waypoint-live"),
                *rviz,
                *official_launch,
                *waypoint_motion,
            ),
            timeout_s=900,
            needs_official_policy=True,
            required_scene="scene_29dof_extreme_lab.xml",
        ),
        "social-live": DemoCase(
            name="social-live",
            description="Scored conservative live person avoidance",
            command=(
                str(SCRIPTS / "run_extreme_lab_person_avoidance_benchmark.sh"),
                *(("--skip-build",) if fast_start else ()),
                "--mode",
                "smoke",
                "--seed",
                str(seed),
                "--people",
                "2",
                "--goals",
                "1",
                "--minimum-score",
                "70",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_social_benchmark.yaml",
                "--result-dir",
                str(run_dir / "social-live"),
                *rviz,
                *official_launch,
                *waypoint_motion,
                "--sim-arg",
                "max_ang_vel_z:=0.20",
                "--sim-arg",
                "turning_arc_min_forward_speed:=0.20",
                "--sim-arg",
                "mujoco_pedestrian_start_delay:=30.0",
                "--sim-arg",
                "tracker_use_sensor_input:=false",
                "--sim-arg",
                "tracker_local_tracking_radius:=0.0",
                "--sim-arg",
                "livox_scan_points:=1200",
                "--sim-arg",
                "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_oracle_v1.yaml",
            ),
            timeout_s=900,
            needs_official_policy=True,
            required_scene="scene_29dof_extreme_lab.xml",
        ),
        "social-endurance": DemoCase(
            name="social-endurance",
            description="Seeded four-stage pedestrian yield-and-resume challenge",
            command=(
                *endurance_common,
                "--result-dir",
                str(run_dir / "social-endurance"),
                *rviz,
                *endurance_simulation,
            ),
            timeout_s=1200,
            needs_official_policy=True,
            required_scene="scene_29dof_pedestrians.xml",
        ),
        "waypoint-visual": DemoCase(
            name="waypoint-visual",
            description="Visual map-wide reachable random waypoint showcase",
            command=(
                str(SCRIPTS / "run_extreme_lab_waypoint_benchmark.sh"),
                *(("--skip-build",) if fast_start else ()),
                "--seed",
                str(seed),
                "--goals",
                "4",
                "--minimum-score",
                "80",
                "--config",
                "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_waypoint_benchmark.yaml",
                "--result-dir",
                str(run_dir / "waypoint-visual"),
                *official_launch,
                *waypoint_motion,
            ),
            timeout_s=0,
            scored=False,
            interactive=True,
            needs_official_policy=True,
            required_scene="scene_29dof_extreme_lab.xml",
        ),
        "social-waypoints": DemoCase(
            name="social-waypoints",
            description="Reachable Nav2 waypoints with independent people (oracle tracking, known static map)",
            command=(
                str(SCRIPTS / "run_extreme_lab_person_avoidance_benchmark.sh"),
                *(("--skip-build",) if fast_start else ()),
                "--scene", "scene_29dof_pedestrians.xml",
                "--environment-name", "pedestrian_plaza",
                "--mode", "smoke", "--seed", str(seed), "--people", "4", "--goals", str(social_goals),
                "--minimum-score", "70",
                "--config", "/workspace/ros2_ws/src/dynamic_obstacle_benchmarks/config/demo_suite_social_waypoints_benchmark.yaml",
                "--result-dir", str(run_dir / "social-waypoints"),
                *rviz, *official_launch, *waypoint_motion,
                "--sim-arg", "mujoco_pedestrian_scenario:=social_wandering_plaza",
                "--sim-arg", "mujoco_pedestrian_start_delay:=15.0",
                "--sim-arg", "nav2_observation_sources:=scan",
                "--sim-arg", "nav2_max_vel_y:=0.35",
                "--sim-arg", "nav2_min_vel_y:=-0.35",
                "--sim-arg", "nav2_acc_lim_y:=0.60",
                "--sim-arg", "nav2_decel_lim_y:=-0.60",
                "--sim-arg", "nav2_vy_samples:=7",
                "--sim-arg", "nav2_planner_plugin:=nav2_smac_planner/SmacPlanner2D",
                "--sim-arg", "nav2_world_costmap:=true",
                "--sim-arg", "nav2_static_scene_map:=true",
                "--sim-arg", "nav2_people_layer:=true",
                "--sim-arg", "rviz_config:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/rviz/g1_social_waypoints.rviz",
                "--sim-arg", "max_lin_vel_x:=0.65",
                "--sim-arg", "max_lin_vel_y:=0.45",
                "--sim-arg", "cmd_accel_x:=1.20",
                "--sim-arg", "cmd_accel_y:=1.20",
                "--sim-arg", "tracker_use_sensor_input:=false",
                "--sim-arg", "tracker_local_tracking_radius:=0.0",
                "--sim-arg", "livox_scan_points:=1200",
                "--sim-arg", "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_waypoints_oracle_v1.yaml",
            ),
            timeout_s=900,
            needs_official_policy=True,
            required_scene="scene_29dof_pedestrians.xml",
        ),
        "social-visual": DemoCase(
            name="social-visual",
            description="Visual seeded four-stage pedestrian yield-and-resume challenge",
            command=(
                *endurance_common,
                "--result-dir",
                str(run_dir / "social-visual"),
                *endurance_simulation,
            ),
            timeout_s=0,
            scored=False,
            interactive=True,
            needs_official_policy=True,
            required_scene="scene_29dof_pedestrians.xml",
        ),
    }


    oracle = cases["social-waypoints"]
    command = tuple(arg.replace("demo_suite_social_waypoints_benchmark.yaml", "demo_suite_natural_crowd_benchmark.yaml")
                    .replace(str(run_dir / "social-waypoints"), str(run_dir / "onboard-waypoints")) for arg in oracle.command)
    overrides = (
        # Isolated yaw trials support a bounded pivot rather than forcing a
        # forward arc that can orbit nearby targets.
        "turning_arc_min_forward_speed:=0.0", "turning_arc_min_yaw_speed:=0.0",
        "max_ang_vel_z:=0.90", "cmd_accel_yaw:=0.90",
        "cmd_accel_x:=2.40", "cmd_accel_y:=2.40",
        "mujoco_pedestrian_scenario:=social_natural_plaza", "mujoco_pedestrian_start_delay:=10.0",
        "mujoco_pedestrian_start_gate:=false",
        "onboard_only:=true", "nav2_dynamic_replanning:=true", "nav2_world_costmap:=false", "nav2_static_scene_map:=false",
        "tracker_use_sensor_input:=true", "tracker_local_tracking_radius:=10.0",
        "livox_scan_points:=2400", "lidar_rate_hz:=15.0",
        "publish_lidar_tf:=false", "use_mujoco_lidar_pose:=false",
        "use_cmd_vel_lidar_fallback:=false", "run_slam_reset_monitor:=false",
        "run_lidar_person_detector:=false", "lidar_output_frame:=mujoco_livox_frame",
        "show_mujoco_goal:=true",
        *(("publish_camera:=true", "camera_width:=160", "camera_height:=120", "camera_rate_hz:=1.0") if not headless else ()),
        "rviz_config:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/rviz/g1_onboard_waypoints.rviz",
        "social_navigation_params_file:=/workspace/ros2_ws/src/unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml",
    )
    override_names = {value.split(":=", 1)[0] for value in overrides}
    filtered = []
    index = 0
    while index < len(command):
        if command[index] == "--sim-arg" and command[index + 1].split(":=", 1)[0] in override_names:
            index += 2
            continue
        filtered.append(command[index])
        index += 1
    filtered[filtered.index("--people") + 1] = str(onboard_people)
    filtered[filtered.index("--scene") + 1] = "scene_29dof_natural_crowd_dex3.xml"
    cases["onboard-waypoints"] = replace(oracle, name="onboard-waypoints",
        description="Online SLAM, LiDAR tracking and event-driven routes; no scene map or truth localization",
        required_scene="scene_29dof_natural_crowd_dex3.xml",
        command=tuple(filtered) + tuple(arg for value in overrides for arg in ("--sim-arg", value)))
    visual_command = list(cases["onboard-waypoints"].command)
    threshold_index = visual_command.index("--minimum-score")
    del visual_command[threshold_index:threshold_index + 2]
    visual_command[visual_command.index("--goals") + 1] = str(visual_goals or 1_000_000)
    visual_command[visual_command.index("--result-dir") + 1] = str(run_dir / "onboard-visual")
    visual_command += ["--visual-session", "--duration-seconds", str(visual_duration_s)]
    cases["onboard-visual"] = replace(cases["onboard-waypoints"], name="onboard-visual",
        description="Long onboard visual session: MuJoCo goal overlay, RViz sensors, SLAM and predictions",
        command=tuple(visual_command), scored=False, interactive=True,
        timeout_s=math.ceil(visual_duration_s) + 300)
    docker = shutil.which("docker") or "/usr/bin/docker"
    compose = (docker, "compose", "run", "--rm", "--no-deps", "-T")
    cases["workspace-ci"] = DemoCase(
        "workspace-ci", "Build and test ROS, native interfaces and orchestration",
        (*compose, "-e", "ROS_DOMAIN_ID=87", "unitree-g1-sim",
         "/workspace/scripts/test_workspace_inside.sh", "ci"), 1800, scope="regression")
    cases["dex3-contact"] = DemoCase(
        "dex3-contact", "Pinned MuJoCo contact, endurance and rotational hand tests",
        (*compose, "-e", "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1", "unitree-g1-sim",
         "/workspace/scripts/run_mjlab.sh", "--with", "mujoco==3.3.6", "--with", "pytest==8.3.5",
         "python", "-m", "pytest", "-q", "-p", "no:cacheprovider",
         "/workspace/scripts/test_dex3_contact.py", "/workspace/scripts/test_native_probe_payload.py"),
        900, scope="component")
    delivery_id = f"dex3_{run_dir.name}"
    delivery_result = REPO_ROOT / "ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo" / delivery_id / "result.json"
    cases["dex3-delivery"] = DemoCase(
        "dex3-delivery", "Experimental fixed red-to-red contact-only mobile delivery",
        (sys.executable, str(SCRIPTS / "run_transport_demo.py"), "--run-id", delivery_id,
         "--world", "transport_dual", "--task", "red:red", "--people", "0",
         "--seed", str(seed), "--duration", "420", "--no-rviz",
         "--carry-posture", "payload_v1", "--manipulation-heading-hold",
         "--grasp-height-offset", "0.015", "--policy", str(CONTACT_POLICY),
         *(("--fast-start",) if fast_start else ())),
        1200, scope="mobile_delivery", delivery_result=str(delivery_result))
    return cases


PRESETS = {
    "validation": ("workspace-ci", "dex3-contact"),
    "manipulation": ("dex3-contact", "dex3-delivery"),
    "quick": ("mock-avoidance",),
    "locomotion": ("mock-avoidance", "fixed-forward"),
    "navigation": ("mock-avoidance", "fixed-forward", "waypoint-live"),
    "social": ("mock-avoidance", "social-live"),
    "endurance": ("mock-avoidance", "fixed-forward", "social-endurance"),
    "full": (
        "mock-avoidance",
        "fixed-forward",
        "waypoint-live",
        "social-live",
        "social-endurance",
    ),
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(*args: str) -> str | None:
    result = subprocess.run(
        ("git", *args),
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def select_cases(
    catalog: dict[str, DemoCase], preset: str | None, requested: list[str]
) -> list[DemoCase]:
    names = list(PRESETS[preset]) if preset else []
    names.extend(requested)
    if not names:
        names.extend(PRESETS["quick"])
    selected: list[DemoCase] = []
    seen: set[str] = set()
    for name in names:
        if name not in catalog:
            raise ValueError(f"unknown demo case: {name}")
        if name not in seen:
            selected.append(catalog[name])
            seen.add(name)
    if len(selected) > 1 and any(case.interactive for case in selected):
        raise ValueError("interactive cases must be run alone")
    return selected


def write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def stream_process(
    command: tuple[str, ...], log: TextIO, timeout_s: int
) -> tuple[int, bool]:
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = time.monotonic() + timeout_s if timeout_s > 0 else None
    timed_out = False
    decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
    def emit(data: bytes, final: bool = False) -> None:
        text = decoder.decode(data, final=final)
        print(text, end="", flush=True)
        log.write(text)
        log.flush()
    try:
        while True:
            for key, _ in selector.select(timeout=0.2):
                chunk = os.read(key.fileobj.fileno(), 65536)
                if chunk:
                    emit(chunk)
                else:
                    selector.unregister(key.fileobj)
            if process.poll() is not None:
                emit(process.stdout.read(), final=True)
                break
            if deadline is not None and time.monotonic() >= deadline:
                timed_out = True
                os.killpg(process.pid, 15)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, 9)
                    process.wait()
                break
    except KeyboardInterrupt:
        try:
            os.killpg(process.pid, 15)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, 9)
            except ProcessLookupError:
                pass
            process.wait()
        raise
    finally:
        selector.close()
        process.stdout.close()
    return (124 if timed_out else process.returncode), timed_out


def preflight(cases: list[DemoCase], dry_run: bool) -> dict[str, object]:
    checks: dict[str, object] = {}
    compose = subprocess.run(
        ("docker", "compose", "config", "--quiet"),
        cwd=REPO_ROOT,
        check=False,
    )
    if compose.returncode != 0:
        raise RuntimeError("docker compose configuration is invalid")
    checks["docker_compose"] = "ok"
    for case in cases:
        executable = Path(case.command[0])
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f"demo entry point is not executable: {executable}")
    checks["entry_points"] = "ok"
    if any(case.delivery_result for case in cases):
        policy_hash = file_sha256(CONTACT_POLICY) if CONTACT_POLICY.is_file() else None
        if not dry_run and policy_hash != CONTACT_POLICY_SHA256:
            raise RuntimeError("Dex3 delivery requires the checksum-pinned native cycle05 policy")
        checks["contact_policy_sha256"] = policy_hash
    if any(case.needs_official_policy for case in cases):
        if not dry_run:
            subprocess.run(
                (str(SCRIPTS / "download_unitree_rl_lab_policy.sh"),),
                cwd=REPO_ROOT,
                check=True,
            )
        checks["official_policy"] = (
            file_sha256(OFFICIAL_POLICY) if OFFICIAL_POLICY.is_file() else "dry-run"
        )
    scene_root = REPO_ROOT / "ros2_ws/src/unitree_mujoco/unitree_robots/g1"
    for scene_name in sorted(
        {case.required_scene for case in cases if case.required_scene is not None}
    ):
        scene_path = scene_root / scene_name
        if not scene_path.is_file():
            raise RuntimeError(f"required MuJoCo scene is unavailable: {scene_path}")
        checks[f"scene:{scene_name}"] = str(scene_path)
    return checks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=tuple(PRESETS))
    parser.add_argument("--case", action="append", default=[], dest="cases")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--goals", type=int, default=None, help="waypoint goal budget (scored cases: 4; onboard-visual: continuous)")
    parser.add_argument("--duration", type=float, default=1800., help="onboard-visual duration in seconds (default: 30 minutes)")
    parser.add_argument("--people", type=int, choices=range(1, 9), default=8, help="pedestrians in onboard cases (default: 8)")
    parser.add_argument(
        "--fast-start",
        action="store_true",
        help="reuse existing builds to reduce launch overhead",
    )
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument(
        "--seed",
        type=int,
        help="replay a run with a fixed seed; omitted seeds are generated per run",
    )
    parser.add_argument("--result-root", type=Path, default=DEFAULT_RESULTS)
    return parser.parse_args()


def resolve_seed(requested_seed: int | None) -> int:
    return requested_seed if requested_seed is not None else secrets.randbits(31)


def delivery_evidence(path: Path) -> dict[str, object]:
    """Fail closed: a held-object evaluator flag is never delivery success."""
    try:
        data = json.loads(path.read_text())
        valid = (
            isinstance(data, dict)
            and all(data.get(key) is True for key in ("success", "mission_complete", "data_valid"))
            and data.get("fallen") is False
            and data.get("contact_samples") == 0
            and data.get("dropped_objects") == []
            and isinstance(data.get("deliveries"), list)
            and len(data["deliveries"]) == 1
            and data["deliveries"][0].get("object_id") == "box_red"
            and data["deliveries"][0].get("destination_id") == "target_red"
            and data["deliveries"][0].get("verified") is True
        )
        return {"path": str(path), "sha256": file_sha256(path), "passed": bool(valid)}
    except (OSError, ValueError, TypeError, AttributeError) as error:
        return {"path": str(path), "passed": False, "error": str(error)}


def main() -> int:
    args = parse_args()
    seed = resolve_seed(args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir = args.result_root.expanduser().resolve() / f"suite_{stamp}"
    if not run_dir.is_relative_to(REPO_ROOT):
        print(
            f"Demo suite result root must be inside {REPO_ROOT}: {args.result_root}",
            file=sys.stderr,
        )
        return 2
    if args.goals is not None and args.goals < 1:
        print("--goals must be positive", file=sys.stderr)
        return 2
    if not math.isfinite(args.duration) or args.duration <= 0:
        print("--duration must be finite and positive", file=sys.stderr)
        return 2
    catalog = case_catalog(run_dir, seed, args.headless, args.fast_start, args.goals or 4,
                           args.duration, args.goals, args.people)
    if args.list:
        print("Presets:")
        for name, cases in PRESETS.items():
            print(f"  {name:12} {' '.join(cases)}")
        print("Cases:")
        for case in catalog.values():
            kind = "interactive" if case.interactive else "scored"
            print(f"  {case.name:16} [{kind}] {case.description}")
        return 0
    try:
        cases = select_cases(catalog, args.suite, args.cases)
        checks = preflight(cases, args.dry_run)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Demo suite preflight failed: {error}", file=sys.stderr)
        return 2

    manifest = {
        "schema_version": 1,
        "run_id": run_dir.name,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "repository": str(REPO_ROOT),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_dirty": bool(git_output("status", "--porcelain")),
        "seed": seed,
        "headless": args.headless,
        "fast_start": args.fast_start,
        "dry_run": args.dry_run,
        "preflight": checks,
        "cases": [
            {
                **asdict(case),
                "command": list(case.command),
                "command_shell": shlex.join(case.command),
            }
            for case in cases
        ],
    }
    run_dir.mkdir(parents=True, exist_ok=False)
    write_json(run_dir / "manifest.json", manifest)
    results: list[dict[str, object]] = []
    overall_status = "dry-run" if args.dry_run else ("completed" if all(not case.scored for case in cases) else "passed")
    try:
        for case in cases:
            case_dir = run_dir / case.name
            case_dir.mkdir(parents=True, exist_ok=True)
            print(f"\n[{case.name}] {case.description}")
            print(f"Command: {shlex.join(case.command)}")
            started = time.monotonic()
            if args.dry_run:
                return_code, timed_out = 0, False
            else:
                with (case_dir / "suite.log").open("w", encoding="utf-8") as log:
                    try:
                        return_code, timed_out = stream_process(case.command, log, case.timeout_s)
                    except OSError as error:
                        log.write(f"Launch failed: {error}\n")
                        return_code, timed_out = 127, False
            evidence = delivery_evidence(Path(case.delivery_result)) if case.delivery_result and not args.dry_run else None
            passed = return_code == 0 and (evidence is None or evidence["passed"])
            result = {
                "name": case.name,
                "scope": case.scope,
                "evidence": evidence,
                "return_code": return_code,
                "timed_out": timed_out,
                "duration_s": round(time.monotonic() - started, 3),
                "status": (
                    "planned"
                    if args.dry_run
                    else (("passed" if case.scored else "completed") if passed else "failed")
                ),
            }
            results.append(result)
            write_json(case_dir / "suite_status.json", result)
            if not passed:
                overall_status = "failed"
                if not args.continue_on_failure:
                    break
    except KeyboardInterrupt:
        overall_status = "interrupted"
    finally:
        status = {
            "run_id": run_dir.name,
            "status": overall_status,
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "results": results,
        }
        write_json(run_dir / "status.json", status)
        print(f"\nSuite status: {overall_status}")
        print(f"Artifacts: {run_dir}")
    return 0 if overall_status in ("passed", "completed", "dry-run") else 1


if __name__ == "__main__":
    raise SystemExit(main())
