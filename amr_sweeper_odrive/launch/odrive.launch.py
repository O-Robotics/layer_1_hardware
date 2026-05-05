from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    interface = LaunchConfiguration("interface")
    node_id = LaunchConfiguration("node_id")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("node_id", default_value="0"),
        Node(
            package="amr_sweeper_odrive",
            executable="odrive_node",
            namespace=namespace,
            name="odrive_node",
            output="screen",
            parameters=[{
                "interface": interface,
                "node_id": node_id,
            }],
        ),
    ])
