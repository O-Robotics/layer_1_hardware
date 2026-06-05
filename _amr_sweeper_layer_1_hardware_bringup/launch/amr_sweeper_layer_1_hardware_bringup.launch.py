"""Launch the AMR Sweeper layer-1 hardware stack in staged, readiness-gated steps."""

from dataclasses import dataclass
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


@dataclass
class StageSpec:
    label: str
    launch_actions: list
    readiness_rules: list
    timeout_sec: float


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def _normalize_namespace(value: str) -> str:
    cleaned = value.strip().strip("/")
    return f"/{cleaned}" if cleaned else "/"


def _normalize_fqn(name: str) -> str:
    cleaned = name.strip()
    if not cleaned:
        return cleaned
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    return cleaned.rstrip("/") or "/"


def _qualify_to_ns(namespace_value: str, target: str) -> str:
    if target.startswith("/"):
        return _normalize_fqn(target)
    ns = _normalize_namespace(namespace_value)
    if ns == "/":
        return _normalize_fqn(target)
    return _normalize_fqn(f"{ns}/{target}")


def _as_bool(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _blue(text: str) -> str:
    return f"\033[94m{text}\033[0m"


def _parse_lifecycle_level(raw: str) -> int:
    normalized = raw.strip().upper()
    mapping = {
        "UNCONFIGURED": 1,
        "UNCONFIGURE": 1,
        "UNCONFIG": 1,
        "INACTIVE": 2,
        "CONFIGURED": 2,
        "ACTIVE": 3,
        "FINALIZED": 4,
        "FINAL": 4,
    }
    return mapping.get(normalized, 3)


def _load_readiness_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    stages = data.get("stages", {})
    return stages if isinstance(stages, dict) else {}


def _filter_rules(context, rules: list[dict]) -> list[dict]:
    filtered = []
    for rule in rules:
        if not rule or not isinstance(rule, dict):
            continue
        if not rule.get("required", True):
            continue
        when_true = rule.get("when_arg_true")
        if when_true and not _as_bool(context, when_true):
            continue
        when_false = rule.get("when_arg_false")
        if when_false and _as_bool(context, when_false):
            continue
        filtered.append(rule)
    return filtered


def _wait_for_stage(context, label: str, readiness_rules: list[dict], timeout_sec: float):
    logger = launch_logging.get_logger("launch.user")
    namespace_value = LaunchConfiguration("namespace").perform(context)
    if not readiness_rules:
        logger.info(f"{_blue('Layer 1 ready:')} {label} (no blocking readiness rules)")
        return

    import rclpy
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from rclpy.node import Node as RclpyNode
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message, get_service

    # Do not parse the launch process argv here; those tokens include launch
    # arguments like `namespace:=...`, which rclpy would otherwise treat as
    # deprecated remap rules and warn about.
    rclpy.init(args=[])
    node = RclpyNode("layer_1_bringup_waiter")
    deadline = time.monotonic() + max(timeout_sec, 0.0)

    topic_watchers = {}
    service_clients = {}
    list_controllers_client = None
    list_hardware_client = None

    def node_fqns() -> set[str]:
        results = set()
        for name, namespace in node.get_node_names_and_namespaces():
            ns = namespace.rstrip("/")
            if not ns:
                ns = "/"
            if ns == "/":
                results.add(_normalize_fqn(name))
            else:
                results.add(_normalize_fqn(f"{ns}/{name}"))
        return results

    def topic_types() -> dict[str, list[str]]:
        return {
            _normalize_fqn(name): types
            for name, types in node.get_topic_names_and_types()
        }

    def service_types() -> dict[str, list[str]]:
        return {
            _normalize_fqn(name): types
            for name, types in node.get_service_names_and_types()
        }

    def ensure_topic_watcher(topic_name: str, types: list[str]):
        if topic_name in topic_watchers or not types:
            return
        msg_type = get_message(types[0])
        state = {"received": False}

        def _callback(_msg):
            state["received"] = True

        subscription = node.create_subscription(
            msg_type, topic_name, _callback, qos_profile_sensor_data)
        topic_watchers[topic_name] = {
            "subscription": subscription,
            "state": state,
        }

    def ensure_service_client(service_name: str, types: list[str]):
        if service_name in service_clients or not types:
            return
        srv_type = get_service(types[0])
        service_clients[service_name] = node.create_client(srv_type, service_name)

    try:
        while time.monotonic() <= deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            seen_nodes = node_fqns()
            seen_topics = topic_types()
            seen_services = service_types()
            missing = []

            for rule in readiness_rules:
                rule_type = rule.get("type", "").strip().lower()
                target = rule.get("target", "").strip()
                if not rule_type or not target:
                    continue

                if rule_type == "node":
                    fq_target = _qualify_to_ns(namespace_value, target)
                    if fq_target not in seen_nodes:
                        missing.append(f"node {fq_target}")
                    continue

                if rule_type == "topic":
                    fq_target = _qualify_to_ns(namespace_value, target)
                    types = seen_topics.get(fq_target, [])
                    if not types:
                        missing.append(f"topic {fq_target}")
                        continue
                    ensure_topic_watcher(fq_target, types)
                    if not topic_watchers[fq_target]["state"]["received"]:
                        missing.append(f"topic data {fq_target}")
                    continue

                if rule_type == "service":
                    fq_target = _qualify_to_ns(namespace_value, target)
                    types = seen_services.get(fq_target, [])
                    if not types:
                        missing.append(f"service {fq_target}")
                        continue
                    ensure_service_client(fq_target, types)
                    if not service_clients[fq_target].wait_for_service(timeout_sec=0.0):
                        missing.append(f"service ready {fq_target}")
                    continue

                if rule_type == "controller":
                    if list_controllers_client is None:
                        srv_name = _qualify_to_ns(namespace_value, "controller_manager/list_controllers")
                        list_controllers_client = node.create_client(ListControllers, srv_name)
                    if not list_controllers_client.wait_for_service(timeout_sec=0.0):
                        missing.append("controller_manager/list_controllers")
                        continue
                    future = list_controllers_client.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                    if not future.done() or future.result() is None:
                        missing.append(f"controller {target}")
                        continue
                    controller = next(
                        (item for item in future.result().controller if item.name == target),
                        None,
                    )
                    if controller is None or controller.state != "active":
                        missing.append(f"controller {target}")
                    continue

                if rule_type == "hardware":
                    if list_hardware_client is None:
                        srv_name = _qualify_to_ns(
                            namespace_value, "controller_manager/list_hardware_components")
                        list_hardware_client = node.create_client(
                            ListHardwareComponents, srv_name)
                    if not list_hardware_client.wait_for_service(timeout_sec=0.0):
                        missing.append("controller_manager/list_hardware_components")
                        continue
                    future = list_hardware_client.call_async(ListHardwareComponents.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                    if not future.done() or future.result() is None:
                        missing.append(f"hardware {target}")
                        continue
                    component = next(
                        (item for item in future.result().component if item.name == target),
                        None,
                    )
                    expected_state = _parse_lifecycle_level(rule.get("state", "active"))
                    if component is None or component.state.id != expected_state:
                        missing.append(f"hardware {target}")
                    continue

            if not missing:
                logger.info(
                    f"{_blue('Layer 1 ready:')} {label} "
                    f"(checks={len(readiness_rules)})"
                )
                return

        raise RuntimeError(
            f"Layer 1 stage '{label}' timed out after {timeout_sec:.1f}s; "
            f"missing: {', '.join(missing)}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _wait_action(label: str, readiness_rules: list[dict], timeout_sec: float):
    def _wait_fn(context, *args, **kwargs):
        _wait_for_stage(context, label, readiness_rules, timeout_sec)
        return []
    return OpaqueFunction(function=_wait_fn)


def _build_stages(context):
    namespace = LaunchConfiguration("namespace")
    namespace_value = LaunchConfiguration("namespace").perform(context)
    gnss_namespace = PathJoinSubstitution(["/", namespace, "gnss"])
    battery_namespace = PathJoinSubstitution(["/", namespace, "battery"])
    system_info_namespace = PathJoinSubstitution(["/", namespace, "system_info"])
    usb_cameras_namespace = PathJoinSubstitution(["/", namespace, "usb_cameras"])
    depth_camera_namespace = PathJoinSubstitution(["/", namespace, "depth_camera"])
    imu_namespace = PathJoinSubstitution(["/", namespace, "imu"])

    log_level = LaunchConfiguration("log_level")
    ublox_log_level = LaunchConfiguration("ublox_log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    battery_can_interface = LaunchConfiguration("battery_can_interface")
    battery_params_file = LaunchConfiguration("battery_params_file")
    system_info_params_file = LaunchConfiguration("system_info_params_file")
    depth_camera_params_file = LaunchConfiguration("depth_camera_params_file")
    depth_camera_use_laserscan_cfg = LaunchConfiguration("depth_camera_use_laserscan")
    depth_camera_laserscan_params_file = LaunchConfiguration("depth_camera_laserscan_params_file")
    depth_camera_image_topic = LaunchConfiguration("depth_camera_image_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    depth_camera_frame = LaunchConfiguration("depth_camera_frame")
    depth_camera_scan_topic = LaunchConfiguration("depth_camera_scan_topic")
    depth_camera_output_frame = LaunchConfiguration("depth_camera_output_frame")
    depth_camera_range_min = LaunchConfiguration("depth_camera_range_min")
    depth_camera_range_max = LaunchConfiguration("depth_camera_range_max")
    depth_camera_scan_height = LaunchConfiguration("depth_camera_scan_height")
    depth_camera_scan_tilt_angle_deg = LaunchConfiguration("depth_camera_scan_tilt_angle_deg")
    depth_camera_scan_time = LaunchConfiguration("depth_camera_scan_time")
    depth_camera_camera_domain_id = LaunchConfiguration("depth_camera_camera_domain_id")
    imu_device_path = LaunchConfiguration("imu_device_path")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    imu_params_file = LaunchConfiguration("imu_params_file")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")
    use_joint_broadcaster = LaunchConfiguration("use_joint_broadcaster")

    readiness_config = _load_readiness_config(
        LaunchConfiguration("readiness_config_file").perform(context))

    robot_xacro_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_description"),
        "urdf",
        "robot",
        "robot.urdf.xacro",
    ])
    robot_description = ParameterValue(Command([
        "xacro ",
        robot_xacro_file,
        " robot_namespace:=", namespace,
        " use_ros2_control:=", LaunchConfiguration("use_amr_sweeper_ros2_control"),
        " enable_usb_cameras:=", LaunchConfiguration("use_amr_sweeper_usb_cameras"),
        " enable_gnss:=", LaunchConfiguration("use_amr_sweeper_gnss"),
        " enable_imu:=", LaunchConfiguration("use_amr_sweeper_imu"),
        " enable_depth_camera:=", LaunchConfiguration("use_amr_sweeper_depth_camera"),
    ]), value_type=str)

    def stage_rules(stage_name: str):
        stage_cfg = readiness_config.get(stage_name, {})
        rules = stage_cfg.get("ready", [])
        timeout_sec = float(stage_cfg.get("timeout_sec", 30.0))
        return _filter_rules(context, rules), timeout_sec

    stages = []

    if _as_bool(context, "use_amr_sweeper_description"):
        rules, timeout_sec = stage_rules("robot_description")
        stages.append(StageSpec(
            label="robot_description",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: robot_description")),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=namespace,
                    output="screen",
                    parameters=[{
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }],
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_system_info"):
        rules, timeout_sec = stage_rules("system_info")
        stages.append(StageSpec(
            label="system_info",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: system_info")),
                Node(
                    package="amr_sweeper_system_info",
                    executable="system_info_node",
                    namespace=system_info_namespace,
                    name="system_info_node",
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    parameters=[system_info_params_file],
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_battery"):
        rules, timeout_sec = stage_rules("battery")
        stages.append(StageSpec(
            label="battery",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: battery")),
                Node(
                    package="amr_sweeper_battery",
                    executable="battery_node",
                    namespace=battery_namespace,
                    name="battery_node",
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    parameters=[
                        battery_params_file,
                        {"can_interface": battery_can_interface},
                    ],
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_gnss"):
        rules, timeout_sec = stage_rules("gnss")
        stages.append(StageSpec(
            label="gnss",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: gnss")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_launch_file("amr_sweeper_gnss", "amr_sweeper_gnss.launch.py")),
                    launch_arguments={
                        "use_ublox_dgnss_node": LaunchConfiguration("use_amr_sweeper_gnss"),
                        "use_ublox_nav_sat_fix_hp": LaunchConfiguration("use_amr_sweeper_gnss"),
                        "use_ntrip_client": LaunchConfiguration("use_ntrip_client"),
                        "gnss_namespace": gnss_namespace,
                        "gnss_frame_id": gnss_frame_id,
                        "ntrip_params_file": ntrip_params_file,
                        "ublox_log_level": ublox_log_level,
                        "log_level": log_level,
                    }.items(),
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_imu"):
        rules, timeout_sec = stage_rules("imu")
        stages.append(StageSpec(
            label="imu",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: imu")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_launch_file("amr_sweeper_imu", "amr_sweeper_imu.launch.py")),
                    launch_arguments={
                        "namespace": imu_namespace,
                        "use_sim_time": use_sim_time,
                        "params_file": imu_params_file,
                        "device_path": imu_device_path,
                        "port": imu_port,
                        "baud": imu_baud,
                        "use_imu_node": LaunchConfiguration("use_amr_sweeper_imu"),
                    }.items(),
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_usb_cameras"):
        rules, timeout_sec = stage_rules("usb_cameras")
        stages.append(StageSpec(
            label="usb_cameras",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: usb_cameras")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_launch_file("amr_sweeper_usb_cameras", "amr_sweeper_usb_cameras.launch.py")),
                    launch_arguments={
                        "namespace": usb_cameras_namespace,
                        "log_level": log_level,
                        "front_left_camera_enabled": LaunchConfiguration("front_left_camera_enabled"),
                        "front_right_camera_enabled": LaunchConfiguration("front_right_camera_enabled"),
                        "rear_left_camera_enabled": LaunchConfiguration("rear_left_camera_enabled"),
                        "rear_right_camera_enabled": LaunchConfiguration("rear_right_camera_enabled"),
                        "tools_camera_enabled": LaunchConfiguration("tools_camera_enabled"),
                    }.items(),
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_depth_camera"):
        rules, timeout_sec = stage_rules("depth_camera")
        stages.append(StageSpec(
            label="depth_camera",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: depth_camera")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_launch_file("amr_sweeper_depth_camera", "amr_sweeper_depth_camera.launch.py")),
                    launch_arguments={
                        "namespace": depth_camera_namespace,
                        "log_level": log_level,
                        "use_sim_time": use_sim_time,
                        "camera_domain_id": depth_camera_camera_domain_id,
                        "params_file": depth_camera_params_file,
                        "use_laserscan": depth_camera_use_laserscan_cfg,
                        "laserscan_params_file": depth_camera_laserscan_params_file,
                        "depth_image_topic": depth_camera_image_topic,
                        "depth_camera_info_topic": depth_camera_info_topic,
                        "depth_camera_frame": depth_camera_frame,
                        "scan_topic": depth_camera_scan_topic,
                        "output_frame": depth_camera_output_frame,
                        "range_min": depth_camera_range_min,
                        "range_max": depth_camera_range_max,
                        "scan_height": depth_camera_scan_height,
                        "scan_tilt_angle_deg": depth_camera_scan_tilt_angle_deg,
                        "scan_time": depth_camera_scan_time,
                    }.items(),
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    if _as_bool(context, "use_amr_sweeper_ros2_control"):
        rules, timeout_sec = stage_rules("ros2_control")
        stages.append(StageSpec(
            label="ros2_control",
            launch_actions=[
                LogInfo(msg=_blue("Layer 1 stage: ros2_control")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _launch_file("amr_sweeper_ros2_control", "amr_sweeper_ros2_control.launch.py")),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": use_sim_time,
                        "use_ros2_control": LaunchConfiguration("use_amr_sweeper_ros2_control"),
                        "use_joint_broadcaster": use_joint_broadcaster,
                    }.items(),
                ),
            ],
            readiness_rules=rules,
            timeout_sec=timeout_sec,
        ))

    return stages


def _launch_setup(context, *args, **kwargs):
    stages = _build_stages(context)
    if not stages:
        return [LogInfo(msg="Layer 1 bringup: no enabled stages to launch.")]

    actions = []
    for stage in stages:
        actions.extend(stage.launch_actions)
        actions.append(_wait_action(stage.label, stage.readiness_rules, stage.timeout_sec))
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("ublox_log_level", default_value="WARN"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(DeclareLaunchArgument(
        "readiness_config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_layer_1_hardware_bringup"),
            "config",
            "amr_sweeper_layer_1_hardware_bringup.yaml",
        ]),
    ))

    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_description", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_ros2_control", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_joint_broadcaster", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_battery", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_system_info", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_usb_cameras", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_depth_camera", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_imu", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_gnss", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ntrip_client", default_value="true"))
    ld.add_action(DeclareLaunchArgument("battery_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("battery_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_battery"),
        "config",
        "amr_sweeper_battery.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("system_info_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_system_info"),
        "config",
        "amr_sweeper_system_info.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("depth_camera_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "amr_sweeper_depth_camera.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("depth_camera_use_laserscan", default_value="true"))
    ld.add_action(DeclareLaunchArgument("depth_camera_laserscan_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "laserscan.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("depth_camera_image_topic", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_info_topic", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_frame", default_value="depth_camera_link"))
    ld.add_action(DeclareLaunchArgument("depth_camera_scan_topic", default_value="scan"))
    ld.add_action(DeclareLaunchArgument("depth_camera_output_frame", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_range_min", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_range_max", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_scan_height", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_scan_tilt_angle_deg", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_scan_time", default_value=""))
    ld.add_action(DeclareLaunchArgument("depth_camera_camera_domain_id", default_value="5"))
    ld.add_action(DeclareLaunchArgument("imu_device_path", default_value=""))
    ld.add_action(DeclareLaunchArgument("imu_port", default_value=""))
    ld.add_action(DeclareLaunchArgument("imu_baud", default_value=""))
    ld.add_action(DeclareLaunchArgument("imu_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_imu"),
        "config",
        "amr_sweeper_imu.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("gnss_frame_id", default_value="gnss_link"))
    ld.add_action(DeclareLaunchArgument("ntrip_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_gnss"),
        "config",
        "amr_sweeper_gnss_ntrip_client.yaml",
    ])))

    default_enabled = {
        "front_left_camera_enabled": "false",
        "front_right_camera_enabled": "false",
        "rear_left_camera_enabled": "true",
        "rear_right_camera_enabled": "true",
        "tools_camera_enabled": "true",
    }
    for arg_name, default_value in default_enabled.items():
        ld.add_action(DeclareLaunchArgument(arg_name, default_value=default_value))

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
