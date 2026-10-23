"""Static consistency checks for the Geoff-Sim arms-only MoveIt gate."""

import os
import xml.etree.ElementTree as ET

import yaml


CONFIG_DIR = os.environ["G1_MOVEIT_CONFIG_DIR"]
URDF_PATH = os.environ["G1_MOVEIT_URDF"]
ARM_JOINTS = {
    side: [
        f"{side}_{name}_joint"
        for name in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ]
    for side in ("left", "right")
}
ALL_ARM_JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"]
STATE_NAMES = {
    "home",
    "zero",
    "tucked",
    "ready",
    "reach_front",
    "carry",
    "policy_stand",
}


def _yaml(name):
    with open(os.path.join(CONFIG_DIR, name), encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _roots():
    return ET.parse(URDF_PATH).getroot(), ET.parse(
        os.path.join(CONFIG_DIR, "g1_dex3.srdf")
    ).getroot()


def _chain_joints(urdf, base, tip):
    by_child = {
        joint.find("child").get("link"): joint for joint in urdf.findall("joint")
    }
    joints = []
    link = tip
    while link != base:
        assert link in by_child, f"{tip} does not descend from {base}"
        joint = by_child[link]
        if joint.get("type") != "fixed":
            joints.append(joint.get("name"))
        link = joint.find("parent").get("link")
    return list(reversed(joints))


def test_urdf_srdf_and_arm_groups():
    urdf, srdf = _roots()
    assert urdf.get("name") == srdf.get("name")
    groups = {group.get("name"): group for group in srdf.findall("group")}
    assert set(groups) == {"left_arm", "right_arm", "both_arms"}

    for side in ("left", "right"):
        chain = groups[f"{side}_arm"].find("chain")
        assert chain.get("base_link") == "torso_link"
        assert chain.get("tip_link") == f"{side}_wrist_yaw_link"
        assert _chain_joints(urdf, "torso_link", f"{side}_wrist_yaw_link") == ARM_JOINTS[side]

    members = [entry.get("name") for entry in groups["both_arms"].findall("group")]
    assert members == ["left_arm", "right_arm"]
    assert not any("hand" in name.lower() for name in groups)


def test_limits_match_the_local_urdf_and_cover_only_arms():
    urdf, _ = _roots()
    urdf_joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    configured = _yaml("joint_limits.yaml")["joint_limits"]
    assert list(configured) == ALL_ARM_JOINTS

    for name, limits in configured.items():
        source = urdf_joints[name].find("limit")
        assert limits["has_position_limits"] is True
        assert limits["min_position"] == float(source.get("lower"))
        assert limits["max_position"] == float(source.get("upper"))
        assert limits["has_velocity_limits"] is True
        assert 0.0 < limits["max_velocity"] <= float(source.get("velocity"))
        assert limits["has_acceleration_limits"] is True
        assert limits["max_acceleration"] > 0.0


def test_named_states_are_complete_consistent_and_in_bounds():
    _, srdf = _roots()
    limits = _yaml("joint_limits.yaml")["joint_limits"]
    states = {}
    for state in srdf.findall("group_state"):
        values = {joint.get("name"): float(joint.get("value")) for joint in state}
        states[(state.get("group"), state.get("name"))] = values

    for group in ("left_arm", "right_arm", "both_arms"):
        assert {name for owner, name in states if owner == group} == STATE_NAMES

    for name in STATE_NAMES:
        left = states[("left_arm", name)]
        right = states[("right_arm", name)]
        both = states[("both_arms", name)]
        assert list(left) == ARM_JOINTS["left"]
        assert list(right) == ARM_JOINTS["right"]
        assert list(both) == ALL_ARM_JOINTS
        assert both == {**left, **right}
        for joint, value in both.items():
            assert limits[joint]["min_position"] <= value <= limits[joint]["max_position"]


def test_kinematics_and_ompl_cover_the_planning_groups():
    kinematics = _yaml("kinematics.yaml")
    assert set(kinematics) == {"left_arm", "right_arm"}
    assert all(
        entry["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
        for entry in kinematics.values()
    )
    ompl = _yaml("ompl_planning.yaml")
    assert ompl["planning_plugin"] == "ompl_interface/OMPLPlanner"
    assert all(group in ompl for group in ("left_arm", "right_arm", "both_arms"))


def test_controller_action_and_joint_order():
    config = _yaml("moveit_controllers.yaml")
    assert config["moveit_manage_controllers"] is False
    manager = config["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["arm_trajectory_controller"]
    controller = manager["arm_trajectory_controller"]
    assert controller["type"] == "FollowJointTrajectory"
    assert controller["action_ns"] == "follow_joint_trajectory"
    assert controller["joints"] == ALL_ARM_JOINTS


def test_collision_disables_are_audited_pairs_and_never_cross_arm():
    urdf, srdf = _roots()
    collision_links = {
        link.get("name") for link in urdf.findall("link") if link.find("collision") is not None
    }
    adjacent = {
        frozenset((joint.find("parent").get("link"), joint.find("child").get("link")))
        for joint in urdf.findall("joint")
        if joint.find("parent").get("link") in collision_links
        and joint.find("child").get("link") in collision_links
    }
    disables = srdf.findall("disable_collisions")
    disabled_pairs = {
        frozenset((disable.get("link1"), disable.get("link2"))) for disable in disables
    }
    assert disabled_pairs == adjacent
    for disable in disables:
        first, second = disable.get("link1"), disable.get("link2")
        pair = frozenset((first, second))
        expected_reason = "Adjacent" if pair in adjacent else "Default"
        assert disable.get("reason") == expected_reason
        assert not (
            (first.startswith("left_") and second.startswith("right_"))
            or (first.startswith("right_") and second.startswith("left_"))
        )
