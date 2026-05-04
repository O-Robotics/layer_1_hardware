from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    can_interface = LaunchConfiguration("can_interface")
    left_motor_can_id = LaunchConfiguration("left_motor_can_id")
    right_motor_can_id = LaunchConfiguration("right_motor_can_id")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("left_motor_can_id", default_value="0x141"),
        DeclareLaunchArgument("right_motor_can_id", default_value="0x142"),
        Node(
            package="amr_sweeper_steadydrive",
            executable="steadydrive_can_node",
            namespace=[namespace, "/steadydrive_left"],
            name="steadydrive_can_node_left",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
                "motor_can_id": left_motor_can_id,
            }],
        ),
        Node(
            package="amr_sweeper_steadydrive",
            executable="steadydrive_can_node",
            namespace=[namespace, "/steadydrive_right"],
            name="steadydrive_can_node_right",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
                "motor_can_id": right_motor_can_id,
            }],
        ),
    ])
