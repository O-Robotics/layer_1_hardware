from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera_node(camera_key: str, namespace, log_level):
    pkg_share = FindPackageShare("amr_sweeper_usb_cameras")
    params_file = PathJoinSubstitution([pkg_share, "config", f"{camera_key}_params.yaml"])
    enabled = LaunchConfiguration(f"{camera_key}_enabled")
    return Node(
        package="amr_sweeper_usb_cameras",
        executable="usb_cameras_node",
        namespace=namespace,
        name=f"{camera_key}_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
        condition=IfCondition(enabled),
    )


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    use_simulation = LaunchConfiguration("use_simulation").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    default_enabled = {
        "front_left_camera": "false",
        "front_right_camera": "false",
        "rear_left_camera": "true",
        "rear_right_camera": "true",
        "tools_camera": "true",
    }

    if use_simulation:
        return []

    actions = []
    for camera_key in [
        "front_left_camera",
        "front_right_camera",
        "rear_left_camera",
        "rear_right_camera",
        "tools_camera",
    ]:
        actions.append(_camera_node(camera_key, namespace, log_level))

    return actions


def generate_launch_description():
    default_enabled = {
        "front_left_camera": "false",
        "front_right_camera": "false",
        "rear_left_camera": "true",
        "rear_right_camera": "true",
        "tools_camera": "true",
    }

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper/usb_cameras"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("use_simulation", default_value="false"))

    for camera_key in [
        "front_left_camera",
        "front_right_camera",
        "rear_left_camera",
        "rear_right_camera",
        "tools_camera",
    ]:
        ld.add_action(
            DeclareLaunchArgument(
                f"{camera_key}_enabled",
                default_value=default_enabled[camera_key],
            )
        )

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
