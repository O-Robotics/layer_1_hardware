from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="amr_sweeper_system_info_msgs is an interface package and does not launch runtime nodes."),
    ])
