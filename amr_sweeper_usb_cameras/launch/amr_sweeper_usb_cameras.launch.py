from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera_node(camera_key: str, namespace, log_level):
    pkg_share = FindPackageShare("amr_sweeper_usb_cameras")
    params_file = PathJoinSubstitution([pkg_share, "config", f"{camera_key}_params.yaml"])
    enabled = LaunchConfiguration(f"{camera_key}_enabled")
    camera_namespace = PathJoinSubstitution([namespace, camera_key])
    return Node(
        package="amr_sweeper_usb_cameras",
        executable="amr_sweeper_usb_cameras_node",
        namespace=camera_namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
        condition=IfCondition(enabled),
    )


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))

    for camera_key in [
        "front_left_camera",
        "front_right_camera",
        "rear_left_camera",
        "rear_right_camera",
        "tools_camera",
    ]:
        ld.add_action(DeclareLaunchArgument(f"{camera_key}_enabled", default_value="true"))
        ld.add_action(_camera_node(camera_key, namespace, log_level))

    return ld
