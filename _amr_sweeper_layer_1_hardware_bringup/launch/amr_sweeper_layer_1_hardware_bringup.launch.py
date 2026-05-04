"""Launch the layer_1_hardware packages that live in this repository."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    log_level = LaunchConfiguration("log_level")
    use_battery_node = LaunchConfiguration("use_battery_node")
    use_system_info_node = LaunchConfiguration("use_system_info_node")
    use_usb_cameras = LaunchConfiguration("use_usb_cameras")

    usb_cameras_launch = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_usb_cameras"),
        "launch",
        "amr_sweeper_usb_cameras.launch.py",
    ])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("robot_namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("use_battery_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_system_info_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_usb_cameras", default_value="true"))

    ld.add_action(Node(
        package="amr_sweeper_battery",
        executable="amr_sweeper_battery_node",
        namespace=robot_namespace,
        name="amr_sweeper_battery_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_battery_node),
    ))

    ld.add_action(Node(
        package="amr_sweeper_system_info",
        executable="amr_sweeper_system_info_node",
        namespace=robot_namespace,
        name="amr_sweeper_system_info_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_system_info_node),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([usb_cameras_launch]),
        launch_arguments={
            "namespace": robot_namespace,
            "log_level": log_level,
        }.items(),
        condition=IfCondition(use_usb_cameras),
    ))

    return ld
