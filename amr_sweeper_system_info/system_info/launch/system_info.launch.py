from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        Node(
            package="amr_sweeper_system_info",
            executable="amr_sweeper_system_info_node",
            namespace=namespace,
            name="amr_sweeper_system_info_node",
            output="screen",
        ),
    ])
