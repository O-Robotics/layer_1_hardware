from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_microros = LaunchConfiguration("use_microros")
    can_interface = LaunchConfiguration("can_interface")
    request_id_min = LaunchConfiguration("request_id_min")
    request_id_max = LaunchConfiguration("request_id_max")
    reply_id_offset = LaunchConfiguration("reply_id_offset")
    same_id_reply = LaunchConfiguration("same_id_reply")
    verbosity = LaunchConfiguration("verbosity")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_microros", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("request_id_min", default_value="0x500"),
        DeclareLaunchArgument("request_id_max", default_value="0x57F"),
        DeclareLaunchArgument("reply_id_offset", default_value="0x80"),
        DeclareLaunchArgument("same_id_reply", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="4"),
        Node(
            package="amr_sweeper_microros",
            executable="amr_sweeper_microros_classic_can_agent",
            namespace=namespace,
            name="micro_ros_agent",
            output="screen",
            arguments=[
                "--can-interface", can_interface,
                "--request-id-min", request_id_min,
                "--request-id-max", request_id_max,
                "--reply-id-offset", reply_id_offset,
                "--same-id-reply", same_id_reply,
                "--verbosity", verbosity,
            ],
            condition=IfCondition(use_microros),
        ),
    ])
