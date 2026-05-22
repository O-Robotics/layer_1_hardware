from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_system_info"),
                "config",
                "amr_sweeper_system_info.yaml",
            ]),
        ),
        Node(
            package="amr_sweeper_system_info",
            executable="amr_sweeper_system_info_node",
            namespace=namespace,
            name="amr_sweeper_system_info_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
