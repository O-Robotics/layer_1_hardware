import math
import os
from pathlib import Path
import re
import subprocess

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


def _load_ros_parameter_file(path: str) -> dict:
    data = yaml.safe_load(Path(path).read_text()) or {}
    if "/**" in data:
        return data["/**"].get("ros__parameters", {})
    if len(data) == 1:
        only_node = next(iter(data.values()))
        if isinstance(only_node, dict) and "ros__parameters" in only_node:
            return only_node["ros__parameters"]
    return data


def _list_topics_for_domain(
    domain_id: int,
    *,
    use_daemon: bool = False,
    spin_time_seconds: float = 3.0,
    attempts: int = 3,
) -> list[str]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    command = ["ros2", "topic", "list"]
    if not use_daemon:
        command.extend(["--no-daemon", "--spin-time", str(spin_time_seconds)])

    last_error = None
    for _ in range(max(1, attempts)):
        try:
            result = subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=max(10.0, spin_time_seconds + 2.0),
                env=env,
            )
            return [line.strip() for line in result.stdout.splitlines() if line.strip()]
        except subprocess.CalledProcessError as exc:
            last_error = exc

    if last_error is not None:
        raise last_error
    return []


def _discover_camera_id(topics: list[str], source_root_namespace: str, source_camera_model: str) -> str:
    escaped_root = re.escape(source_root_namespace.rstrip("/"))
    escaped_model = re.escape(source_camera_model)
    pattern = re.compile(
        rf"^{escaped_root}/(?P<camera_id>{escaped_model}_[^/_]+)"
        rf"(?:/tf_static|_Color(?:/.*)?|_CompressedColor(?:/.*)?|_Depth(?:/.*)?|"
        rf"_Infrared_1(?:/.*)?|_Infrared_2(?:/.*)?|_Motion(?:/.*)?|_ObjectDetection(?:/.*)?)$"
    )

    matches = {match.group("camera_id") for topic in topics if (match := pattern.match(topic))}
    if not matches:
        raise RuntimeError(
            f"Could not find a {source_camera_model} camera under '{source_root_namespace}' "
            "on the source ROS domain."
        )
    if len(matches) > 1:
        raise RuntimeError(
            "Found multiple candidate depth cameras on the source ROS domain: "
            + ", ".join(sorted(matches))
            + ". Set source_camera_id explicitly."
        )
    return next(iter(matches))


def _discover_optional_pointcloud_topic(topics: list[str], source_camera_root: str) -> str:
    candidates = [
        topic for topic in topics
        if topic.startswith(source_camera_root) and (
            "points" in topic.lower() or "pointcloud" in topic.lower() or "pcl" in topic.lower()
        )
    ]
    if not candidates:
        return ""

    priority_suffixes = (
        "/depth/color/points",
        "_Depth_Color_Points",
        "_DepthColorPoints",
        "_Points",
        "_PCL",
    )
    for suffix in priority_suffixes:
        for topic in candidates:
            if topic.endswith(suffix):
                return topic

    if len(candidates) == 1:
        return candidates[0]
    return ""


def _launch_setup(context, *args, **kwargs):
    namespace_value = _normalize_namespace(LaunchConfiguration("namespace").perform(context))
    depth_camera_params = _load_ros_parameter_file(LaunchConfiguration("params_file").perform(context))
    laserscan_params = _load_ros_parameter_file(
        LaunchConfiguration("laserscan_params_file").perform(context)
    )

    output_frame_value = LaunchConfiguration("output_frame").perform(context)
    if output_frame_value:
        laserscan_params["output_frame"] = output_frame_value

    range_min_value = LaunchConfiguration("range_min").perform(context)
    if range_min_value:
        laserscan_params["range_min"] = float(range_min_value)

    range_max_value = LaunchConfiguration("range_max").perform(context)
    if range_max_value:
        laserscan_params["range_max"] = float(range_max_value)

    scan_height_value = LaunchConfiguration("scan_height").perform(context)
    if scan_height_value:
        laserscan_params["scan_height"] = int(scan_height_value)

    scan_tilt_angle_deg_value = LaunchConfiguration("scan_tilt_angle_deg").perform(context)
    if scan_tilt_angle_deg_value:
        laserscan_params["scan_tilt_angle_deg"] = float(scan_tilt_angle_deg_value)

    scan_time_value = LaunchConfiguration("scan_time").perform(context)
    if scan_time_value:
        laserscan_params["scan_time"] = float(scan_time_value)

    depth_camera_frame_value = LaunchConfiguration("depth_camera_frame").perform(context)
    laserscan_frame_value = str(laserscan_params.get("output_frame", depth_camera_frame_value))
    scan_tilt_angle_rad = math.radians(float(laserscan_params.get("scan_tilt_angle_deg", 0.0)))

    default_depth_image_topic = f"{namespace_value}/depth/image_rect_raw"
    default_depth_camera_info_topic = f"{namespace_value}/depth/camera_info"
    depth_image_topic_value = LaunchConfiguration("depth_image_topic").perform(context)
    if not depth_image_topic_value:
        depth_image_topic_value = default_depth_image_topic

    depth_camera_info_topic_value = LaunchConfiguration("depth_camera_info_topic").perform(context)
    if not depth_camera_info_topic_value:
        depth_camera_info_topic_value = default_depth_camera_info_topic

    use_domain_bridge_value = (
        LaunchConfiguration("use_domain_bridge").perform(context).strip().lower() in ("1", "true", "yes", "on")
    )
    use_laserscan_value = (
        LaunchConfiguration("use_laserscan").perform(context).strip().lower() in ("1", "true", "yes", "on")
    )
    camera_domain_id_value = int(LaunchConfiguration("camera_domain_id").perform(context))
    workspace_domain_id_value = int(LaunchConfiguration("workspace_domain_id").perform(context))
    source_camera_id_value = LaunchConfiguration("source_camera_id").perform(context).strip()
    source_pointcloud_topic_value = ""
    if use_domain_bridge_value:
        source_root_namespace_value = _normalize_namespace(
            LaunchConfiguration("source_root_namespace").perform(context)
        )
        source_camera_model_value = LaunchConfiguration("source_camera_model").perform(context)
        source_topics = _list_topics_for_domain(
            camera_domain_id_value,
            use_daemon=False,
            spin_time_seconds=3.0,
            attempts=3,
        )
        if not source_camera_id_value:
            source_camera_id_value = _discover_camera_id(
                source_topics,
                source_root_namespace_value,
                source_camera_model_value,
            )
        source_camera_root_value = f"{source_root_namespace_value}/{source_camera_id_value}"
        source_pointcloud_topic_value = _discover_optional_pointcloud_topic(
            source_topics,
            source_camera_root_value,
        )

    depth_camera_params.update(
        {
            "camera_domain_id": camera_domain_id_value,
            "workspace_domain_id": workspace_domain_id_value,
            "source_root_namespace": _normalize_namespace(
                LaunchConfiguration("source_root_namespace").perform(context)
            ),
            "source_camera_id": source_camera_id_value,
            "source_pointcloud_topic": source_pointcloud_topic_value,
            "target_namespace_root": namespace_value,
        }
    )

    launch_summary = LogInfo(
        msg=(
            "amr_sweeper_depth_camera: "
            f"custom_bridge={'enabled' if use_domain_bridge_value else 'disabled'}, "
            f"bridge_path={camera_domain_id_value}->{workspace_domain_id_value}, "
            f"source camera={source_camera_id_value or 'auto-discover'}, "
            f"pointcloud={'detected' if source_pointcloud_topic_value else 'not detected'}, "
            f"depth topic={depth_image_topic_value}, "
            f"camera_info topic={depth_camera_info_topic_value}"
        )
    )

    laserscan_node = Node(
        package="amr_sweeper_depth_camera",
        executable="laserscan_node",
        name="laserscan",
        namespace=namespace_value,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            laserscan_params,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        remappings=[
            ("depth", depth_image_topic_value),
            ("depth_camera_info", depth_camera_info_topic_value),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("use_laserscan")),
    )

    depth_camera_node = Node(
        package="amr_sweeper_depth_camera",
        executable="depth_camera_node",
        name="depth_camera",
        namespace=namespace_value,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            depth_camera_params,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration("use_domain_bridge")),
    )

    laserscan_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laserscan_tf",
        namespace=namespace_value,
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", str(scan_tilt_angle_rad),
            "--yaw", "0",
            "--frame-id", depth_camera_frame_value,
            "--child-frame-id", laserscan_frame_value,
        ],
        condition=IfCondition(LaunchConfiguration("use_laserscan")),
    )

    actions = [launch_summary]
    if use_domain_bridge_value:
        actions.append(depth_camera_node)
    if use_laserscan_value:
        actions.extend([laserscan_tf_node, laserscan_node])
    return actions


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "amr_sweeper_depth_camera.yaml",
    ])
    default_laserscan_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "laserscan.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper/depth_camera"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_domain_bridge", default_value="true"),
        DeclareLaunchArgument("camera_domain_id", default_value="5"),
        DeclareLaunchArgument("workspace_domain_id", default_value="0"),
        DeclareLaunchArgument("source_root_namespace", default_value="/realsense"),
        DeclareLaunchArgument("source_camera_model", default_value="D555"),
        DeclareLaunchArgument("source_camera_id", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("use_laserscan", default_value="true"),
        DeclareLaunchArgument(
            "laserscan_params_file",
            default_value=default_laserscan_params_file,
        ),
        DeclareLaunchArgument("depth_image_topic", default_value=""),
        DeclareLaunchArgument("depth_camera_info_topic", default_value=""),
        DeclareLaunchArgument("depth_camera_frame", default_value="depth_camera_link"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("output_frame", default_value=""),
        DeclareLaunchArgument("range_min", default_value=""),
        DeclareLaunchArgument("range_max", default_value=""),
        DeclareLaunchArgument("scan_height", default_value=""),
        DeclareLaunchArgument("scan_tilt_angle_deg", default_value=""),
        DeclareLaunchArgument("scan_time", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
