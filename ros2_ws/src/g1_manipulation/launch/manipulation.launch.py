"""Start only the manipulation action server; launch move_group separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("unitree_g1_description")
    moveit_share = get_package_share_directory("g1_moveit_config")
    manipulation_share = get_package_share_directory("g1_manipulation")

    def config(name):
        return os.path.join(moveit_share, "config", name)

    moveit_config = (
        MoveItConfigsBuilder(
            "g1_29dof_dex3", package_name="g1_moveit_config"
        )
        .robot_description(
            file_path=os.path.join(
                description_share,
                "urdf",
                "g1_29dof_dex3.urdf",
            )
        )
        .robot_description_semantic(file_path=config("g1_dex3.srdf"))
        .robot_description_kinematics(file_path=config("kinematics.yaml"))
        .joint_limits(file_path=config("joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="g1_manipulation",
                executable="g1_manipulation_server",
                name="g1_manipulation_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    os.path.join(manipulation_share, "config", "manipulation.yaml"),
                ],
            )
        ]
    )
