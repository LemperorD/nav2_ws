from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description="Robot namespace"
    )

    default_params = os.path.join(
        get_package_share_directory("decision_simple"),
        "params",
        "decision_simple.yaml"
    )
    declare_params = DeclareLaunchArgument(
        "params_file", default_value=default_params, description="decision_simple params"
    )

    node = Node(
        package="decision_simple",
        executable="decision_simple_node",
        name="decision_simple",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params,
        GroupAction([
            PushRosNamespace(namespace),
            node,
        ]),
    ])
