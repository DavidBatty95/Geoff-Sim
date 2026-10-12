#!/usr/bin/env python3
"""Export/apply the reviewed local source changes on top of pinned submodules.

Application is idempotent and refuses mixed/conflicting working trees. It never
resets, cleans, stages, commits, or changes repository configuration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PATCHES = ROOT / "patches"
SOURCES = {
    "unitree_mujoco": ("ros2_ws/src/unitree_mujoco", [
        ".gitignore", "simulate/CMakeLists.txt", "simulate/src/main.cc",
        "unitree_robots/g1/scene_29dof_extreme_lab.xml",
        "simulate/src/goal_overlay.cc", "simulate/src/goal_overlay.h",
        "simulate/src/natural_pedestrian_motion.h", "simulate/src/pedestrian_path_planner.h",
        "simulate/src/pedestrian_scenarios.h", "simulate/test/goal_overlay_test.cc",
        "simulate/test/natural_pedestrians_test.cc", "simulate/test/pedestrian_scenarios_test.cc",
        "simulate/src/pedestrian_start_gate.h", "simulate/test/pedestrian_start_gate_test.cc",
        "simulate/src/mujoco_sensor_worker.h", "simulate/src/raycast_model.h", "simulate/test/raycast_model_test.cc",
        "simulate/test/transport_worlds_test.cc",
        "simulate/src/brainco_hand.h",
        "simulate/src/native_imu.h", "simulate/test/native_imu_test.cc",
        "simulate/src/unitree_sdk2_bridge.h",
        "unitree_robots/g1/g1_29dof.xml",
        "simulate/src/simulated_manipulation.cc", "simulate/test/simulated_manipulation_test.cc",
        "tools/generate_natural_pedestrians.py", "tools/test_natural_meshes.py",
        "unitree_robots/g1/scene_29dof_natural_crowd.xml",
    ]),
    "mjlab": ("vendor/mjlab", [
        "docs/source/changelog.rst", "src/mjlab/tasks/velocity/config/g1/__init__.py",
        "src/mjlab/tasks/velocity/config/g1/env_cfgs.py", "src/mjlab/tasks/velocity/config/g1/rl_cfg.py",
        "tests/test_g1_stand_creep_stop_task.py",
    ]),
}


def git(directory, *arguments, check=True):
    return subprocess.run(["git", *arguments], cwd=directory, capture_output=True, check=check)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def export():
    PATCHES.mkdir(exist_ok=True)
    manifest = {"schema_version": 1, "sources": []}
    for name, (relative, paths) in SOURCES.items():
        source = ROOT / relative
        base = git(source, "rev-parse", "HEAD").stdout.decode().strip()
        pin = git(ROOT, "ls-files", "--stage", "--", relative).stdout.decode().split()[1]
        if base != pin:
            raise RuntimeError(f"{relative}: HEAD differs from the parent repository's staged pinned revision")
        patch = git(source, "diff", "--binary", "--full-index", "HEAD", "--", *paths).stdout
        for path in paths:
            if not git(source, "ls-files", "--", path).stdout and (source/path).is_file():
                result = git(source, "diff", "--no-index", "--binary", "--", "/dev/null", path, check=False)
                if result.returncode != 1:
                    raise RuntimeError(f"Could not export new source {relative}/{path}")
                patch += result.stdout
        filename = name + ".patch"
        (PATCHES/filename).write_bytes(patch)
        manifest["sources"].append({"path": relative, "base_commit": base, "patch": filename,
            "sha256": digest(patch), "files": {path: digest((source/path).read_bytes()) for path in paths}})
        print(f"Exported {filename}: {len(patch)} bytes")
    (PATCHES/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")


def check_or_apply(apply=False):
    manifest = json.loads((PATCHES/"manifest.json").read_text())
    for spec in manifest["sources"]:
        source, patch = ROOT/spec["path"], PATCHES/spec["patch"]
        if digest(patch.read_bytes()) != spec["sha256"]:
            raise RuntimeError(f"Patch checksum mismatch: {patch}")
        if git(source, "rev-parse", "HEAD").stdout.decode().strip() != spec["base_commit"]:
            raise RuntimeError(f"{source}: unexpected base revision; do not apply blindly")
        matches = all((source/path).is_file() and digest((source/path).read_bytes()) == expected
                      for path, expected in spec["files"].items())
        if matches:
            print(f"{spec['path']}: reviewed source snapshot present")
            continue
        possible = git(source, "apply", "--check", str(patch), check=False)
        if not apply:
            raise RuntimeError(f"{spec['path']}: source snapshot differs; run apply on a clean checkout or export reviewed changes")
        if possible.returncode != 0:
            raise RuntimeError(f"{spec['path']}: refusing to overwrite conflicting/local changes:\n{possible.stderr.decode()}")
        git(source, "apply", str(patch))
        print(f"{spec['path']}: applied reviewed source patch")
    if apply:
        check_or_apply(False)


def verify_pristine():
    """Exercise each patch against exact base-file contents, without Git commits."""
    manifest = json.loads((PATCHES/"manifest.json").read_text())
    for spec in manifest["sources"]:
        source = ROOT/spec["path"]
        with tempfile.TemporaryDirectory(prefix="geoff-patch-") as temporary:
            target = Path(temporary)
            for path in spec["files"]:
                original = git(source, "show", f"{spec['base_commit']}:{path}", check=False)
                if original.returncode == 0:
                    output = target/path
                    output.parent.mkdir(parents=True, exist_ok=True)
                    output.write_bytes(original.stdout)
                    mode = git(source, "ls-tree", spec["base_commit"], "--", path).stdout.decode().split()[0]
                    output.chmod(0o755 if mode == "100755" else 0o644)
            patch = PATCHES/spec["patch"]
            if digest(patch.read_bytes()) != spec["sha256"]:
                raise RuntimeError(f"Patch checksum mismatch: {patch}")
            # Committed submodule changes need no overlay patch. Still verify
            # every reviewed file against the exact pinned commit below.
            if patch.stat().st_size:
                git(target, "apply", "--check", str(patch))
                git(target, "apply", str(patch))
            for path, expected in spec["files"].items():
                if digest((target/path).read_bytes()) != expected:
                    raise RuntimeError(f"Pristine application did not reproduce {path}")
        print(f"{spec['path']}: pristine patch application reproduced all reviewed files")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("export", "apply", "check", "verify-pristine"))
    action = parser.parse_args().action
    try:
        if action == "export":
            export()
        elif action == "verify-pristine":
            verify_pristine()
        else:
            check_or_apply(action == "apply")
    except (RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
