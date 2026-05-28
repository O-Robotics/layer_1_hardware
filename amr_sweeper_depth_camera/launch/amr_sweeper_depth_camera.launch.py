import math
import os
from pathlib import Path
import re
import subprocess
import tempfile

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
import launch.logging
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


def _list_topics_for_domain(domain_id: int) -> list[str]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    result = subprocess.run(
        ["ros2", "topic", "list"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10.0,
        env=env,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


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


def _resolve_domain_bridge_config(
    template_path: str,
    namespace_value: str,
    source_domain_id: int,
    target_domain_id: int,
    source_camera_id: str,
) -> tuple[str, list[str]]:
    template = Path(template_path).read_text()
    resolved_text = (
        template
        .replace("__FROM_DOMAIN__", str(source_domain_id))
        .replace("__TO_DOMAIN__", str(target_domain_id))
        .replace("__CAMERA_ID__", source_camera_id)
        .replace("__TARGET_NAMESPACE__", namespace_value.rstrip("/"))
    )
    resolved = yaml.safe_load(resolved_text) or {}

    skipped_topics: list[str] = []
    filtered_topics = {}
    for topic_name, topic_config in (resolved.get("topics") or {}).items():
        type_name = str(topic_config.get("type", ""))
        package_name = type_name.split("/", 1)[0]
        try:
            get_package_prefix(package_name)
        except PackageNotFoundError:
            skipped_topics.append(f"{topic_name} ({type_name})")
            continue
        filtered_topics[topic_name] = topic_config

    resolved["topics"] = filtered_topics

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix="amr_sweeper_depth_camera_domain_bridge_",
        delete=False,
    ) as handle:
        yaml.safe_dump(resolved, handle, sort_keys=False)
        return handle.name, skipped_topics


def _write_empty_domain_bridge_config(source_domain_id: int, target_domain_id: int) -> str:
    empty_config = {
        "name": "amr_sweeper_depth_camera_bridge",
        "from_domain": source_domain_id,
        "to_domain": target_domain_id,
        "topics": {},
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix="amr_sweeper_depth_camera_domain_bridge_empty_",
        delete=False,
    ) as handle:
        yaml.safe_dump(empty_config, handle, sort_keys=False)
        return handle.name


def _log_launch_warning(message: str):
    def _emit_warning(context, *args, **kwargs):
        del context, args, kwargs
        launch.logging.get_logger("launch.user").warning(message)
        return []

    return OpaqueFunction(function=_emit_warning)


def _launch_setup(context, *args, **kwargs):
    namespace_value = _normalize_namespace(LaunchConfiguration("namespace").perform(context))
    watchdog_params = _load_ros_parameter_file(
        LaunchConfiguration("watchdog_params_file").perform(context)
    )
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
    use_watchdog_value = (
        LaunchConfiguration("use_watchdog").perform(context).strip().lower() in ("1", "true", "yes", "on")
    )
    use_laserscan_value = (
        LaunchConfiguration("use_laserscan").perform(context).strip().lower() in ("1", "true", "yes", "on")
    )
    source_domain_id_value = int(LaunchConfiguration("source_domain_id").perform(context))
    target_domain_id_value = int(LaunchConfiguration("target_domain_id").perform(context))
    source_camera_id_value = LaunchConfiguration("source_camera_id").perform(context).strip()
    resolved_bridge_config = ""
    skipped_bridge_topics: list[str] = []
    unavailable_reason = ""
    if use_domain_bridge_value:
        source_root_namespace_value = _normalize_namespace(
            LaunchConfiguration("source_root_namespace").perform(context)
        )
        source_camera_model_value = LaunchConfiguration("source_camera_model").perform(context)
        try:
            if not source_camera_id_value:
                source_topics = _list_topics_for_domain(source_domain_id_value)
                source_camera_id_value = _discover_camera_id(
                    source_topics,
                    source_root_namespace_value,
                    source_camera_model_value,
                )

            resolved_bridge_config, skipped_bridge_topics = _resolve_domain_bridge_config(
                LaunchConfiguration("domain_bridge_config_file").perform(context),
                namespace_value,
                source_domain_id_value,
                target_domain_id_value,
                source_camera_id_value,
            )
        except RuntimeError as exc:
            unavailable_reason = str(exc)
            resolved_bridge_config = _write_empty_domain_bridge_config(
                source_domain_id_value,
                target_domain_id_value,
            )
            use_watchdog_value = False
            use_laserscan_value = False

    launch_summary = LogInfo(
        msg=(
            "amr_sweeper_depth_camera: "
            f"bridge={'enabled' if use_domain_bridge_value else 'disabled'}, "
            f"bridge_path={source_domain_id_value}->{target_domain_id_value}, "
            f"source camera={source_camera_id_value or 'auto-discover'}, "
            f"depth topic={depth_image_topic_value}, "
            f"camera_info topic={depth_camera_info_topic_value}, "
            f"skipped optional bridge topics={len(skipped_bridge_topics)}"
        )
    )
    unavailable_summary = _log_launch_warning(
        "amr_sweeper_depth_camera: "
        f"{unavailable_reason} Starting in degraded mode with an empty domain bridge config "
        "and without laserscan or watchdog nodes."
    )
    skipped_topics_summary = LogInfo(
        msg=(
            "amr_sweeper_depth_camera: skipped bridge topics due to unavailable interfaces: "
            + ", ".join(skipped_bridge_topics)
        )
    )

    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        namespace=namespace_value,
        output="screen",
        arguments=[resolved_bridge_config],
        condition=IfCondition(LaunchConfiguration("use_domain_bridge")),
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

    watchdog_node = Node(
        package="amr_sweeper_depth_camera",
        executable="depth_camera_watchdog_node",
        name="depth_camera_watchdog",
        namespace=namespace_value,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            watchdog_params,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        remappings=[
            ("depth", depth_image_topic_value),
            ("depth_camera_info", depth_camera_info_topic_value),
        ],
        condition=IfCondition(LaunchConfiguration("use_watchdog")),
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
    if unavailable_reason:
        actions.append(unavailable_summary)
    if skipped_bridge_topics:
        actions.append(skipped_topics_summary)
    if use_domain_bridge_value:
        actions.append(domain_bridge_node)
    if use_laserscan_value:
        actions.extend([laserscan_tf_node, laserscan_node])
    if use_watchdog_value:
        actions.append(watchdog_node)
    return actions


def generate_launch_description():
    default_watchdog_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "depth_camera_watchdog.yaml",
    ])
    default_laserscan_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "laserscan.yaml",
    ])
    default_domain_bridge_config_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "domain_bridge.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper/depth_camera"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_domain_bridge", default_value="true"),
        DeclareLaunchArgument("source_domain_id", default_value="5"),
        DeclareLaunchArgument("target_domain_id", default_value="0"),
        DeclareLaunchArgument("source_root_namespace", default_value="/realsense"),
        DeclareLaunchArgument("source_camera_model", default_value="D555"),
        DeclareLaunchArgument("source_camera_id", default_value=""),
        DeclareLaunchArgument(
            "domain_bridge_config_file",
            default_value=default_domain_bridge_config_file,
        ),
        DeclareLaunchArgument("use_watchdog", default_value="true"),
        DeclareLaunchArgument("watchdog_params_file", default_value=default_watchdog_params_file),
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
