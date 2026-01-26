import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    communication_cfg_dir = LaunchConfiguration("communication_cfg_dir")
    communication_dir = get_package_share_directory("communication")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_communication_cfg_dir = DeclareLaunchArgument(
        "communication_cfg_dir",
        default_value=PathJoinSubstitution([communication_dir, "config", "communication.yaml"]),
        description="Path to the communication config file",
    )

    start_communication_node = Node(
        package="communication",
        executable="communication_node",
        namespace=namespace,
        parameters=[communication_cfg_dir],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_communication_cfg_dir)
    ld.add_action(start_communication_node)

    return ld
