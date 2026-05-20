from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _normalize_namespace(namespace: str) -> str:
    cleaned = namespace.strip().strip("/")
    return f"/{cleaned}" if cleaned else "/"


def _parent_namespace(namespace: str) -> str:
    parts = [part for part in _normalize_namespace(namespace).split("/") if part]
    if len(parts) <= 1:
        return "/"
    return "/" + "/".join(parts[:-1])


def _leaf_name(namespace: str) -> str:
    parts = [part for part in _normalize_namespace(namespace).split("/") if part]
    return parts[-1] if parts else "depth_camera"


def _load_ros_parameter_file(path: str) -> dict:
    data = yaml.safe_load(Path(path).read_text()) or {}
    if "/**" in data:
        return data["/**"].get("ros__parameters", {})
    if len(data) == 1:
        only_node = next(iter(data.values()))
        if isinstance(only_node, dict) and "ros__parameters" in only_node:
            return only_node["ros__parameters"]
    return data


def _launch_setup(context, *args, **kwargs):
    namespace_value = _normalize_namespace(LaunchConfiguration("namespace").perform(context))
    realsense_node_name_value = _leaf_name(namespace_value)

    realsense_namespace_value = _parent_namespace(namespace_value)
    if realsense_namespace_value == "/" and namespace_value != "/":
        realsense_namespace_value = "/"

    realsense_params = _load_ros_parameter_file(
        LaunchConfiguration("realsense_params_file").perform(context)
    )
    depthimage_to_laserscan_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "depthimage_to_laserscan.yaml",
    ])

    default_depth_image_topic = f"{namespace_value}/depth/image_rect_raw"
    default_depth_camera_info_topic = f"{namespace_value}/depth/camera_info"

    depth_image_topic_value = LaunchConfiguration("depth_image_topic").perform(context)
    if not depth_image_topic_value:
        depth_image_topic_value = default_depth_image_topic

    depth_camera_info_topic_value = LaunchConfiguration("depth_camera_info_topic").perform(context)
    if not depth_camera_info_topic_value:
        depth_camera_info_topic_value = default_depth_camera_info_topic

    launch_summary = LogInfo(
        msg=(
            "amr_sweeper_depth_camera: "
            f"realsense namespace={realsense_namespace_value}, "
            f"realsense node={realsense_node_name_value}, "
            f"depth topic={depth_image_topic_value}, "
            f"camera_info topic={depth_camera_info_topic_value}"
        )
    )

    realsense_ros_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=realsense_node_name_value,
        namespace=realsense_namespace_value,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            realsense_params,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "camera_name": realsense_node_name_value,
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_realsense_ros")),
    )

    depthimage_to_laserscan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        namespace=namespace_value,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            depthimage_to_laserscan_params_file,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "output_frame": ParameterValue(LaunchConfiguration("output_frame"), value_type=str),
                "range_min": ParameterValue(LaunchConfiguration("range_min"), value_type=float),
                "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                "scan_height": ParameterValue(LaunchConfiguration("scan_height"), value_type=int),
                "scan_time": ParameterValue(LaunchConfiguration("scan_time"), value_type=float),
            },
        ],
        remappings=[
            ("depth", depth_image_topic_value),
            ("depth_camera_info", depth_camera_info_topic_value),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("use_depthimage_to_laserscan")),
    )

    return [launch_summary, realsense_ros_node, depthimage_to_laserscan_node]


def generate_launch_description():
    default_realsense_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "realsense-ros.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper/depth_camera"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_realsense_ros", default_value="true"),
        DeclareLaunchArgument("realsense_params_file", default_value=default_realsense_params_file),
        DeclareLaunchArgument("use_depthimage_to_laserscan", default_value="true"),
        DeclareLaunchArgument("depth_image_topic", default_value=""),
        DeclareLaunchArgument("depth_camera_info_topic", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("output_frame", default_value="depth_camera_depth_optical_frame"),
        DeclareLaunchArgument("range_min", default_value="0.25"),
        DeclareLaunchArgument("range_max", default_value="8.0"),
        DeclareLaunchArgument("scan_height", default_value="20"),
        DeclareLaunchArgument("scan_time", default_value="0.0333"),
        OpaqueFunction(function=_launch_setup),
    ])
