from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    can_interface = LaunchConfiguration("can_interface")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper/battery"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_battery"),
                "config",
                "amr_sweeper_battery.yaml",
            ]),
        ),
        Node(
            package="amr_sweeper_battery",
            executable="battery_node",
            namespace=namespace,
            name="battery_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "can_interface": can_interface,
                },
            ],
        ),
    ])
