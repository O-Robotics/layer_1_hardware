from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    can_interface = LaunchConfiguration("can_interface")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        Node(
            package="amr_sweeper_battery",
            executable="amr_sweeper_battery_node",
            namespace=namespace,
            name="amr_sweeper_battery_node",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
            }],
        ),
    ])
