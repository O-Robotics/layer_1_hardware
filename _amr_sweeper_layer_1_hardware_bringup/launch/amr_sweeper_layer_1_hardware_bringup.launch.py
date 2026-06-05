"""Launch the AMR Sweeper layer-1 hardware stack in readiness-gated stages."""

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


@dataclass
class StageSpec:
    label: str
    launch_actions: list
    wait_nodes: list
    wait_services: list
    timeout_sec: float = 30.0


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def _normalize_namespace(value: str) -> str:
    cleaned = value.strip().strip("/")
    return f"/{cleaned}" if cleaned else "/"


def _node_fqn(namespace: str, name: str) -> str:
    ns = _normalize_namespace(namespace)
    if ns == "/":
        return f"/{name}"
    return f"{ns}/{name}"


def _as_bool(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _wait_action(label: str, wait_nodes: list[str], wait_services: list[str], timeout_sec: float):
    helper = (
        Path(get_package_share_directory("amr_sweeper_layer_1_hardware_bringup")) /
        "scripts" /
        "wait_for_graph_entities.py"
    )
    cmd = [
        "python3",
        str(helper),
        "--label",
        label,
        "--timeout-sec",
        str(timeout_sec),
    ]
    for node in wait_nodes:
        cmd.extend(["--node", node])
    for service in wait_services:
        cmd.extend(["--service", service])
    return ExecuteProcess(cmd=cmd, output="screen")


def _gate_then_launch(next_stage_bundle):
    def _handler(event, _context):
        if event.returncode == 0:
            return next_stage_bundle
        return [
            LogInfo(
                msg=(
                    "Layer 1 bringup: stopping staged startup because the previous readiness gate "
                    f"failed with exit code {event.returncode}"
                )
            ),
            Shutdown(reason="Layer 1 readiness gate failed"),
        ]
    return _handler


def _build_stages(context):
    namespace = LaunchConfiguration("namespace")
    namespace_value = LaunchConfiguration("namespace").perform(context)
    namespace_fqn = _normalize_namespace(namespace_value)
    gnss_namespace = PathJoinSubstitution(["/", namespace, "gnss"])
    gnss_namespace_value = f"{namespace_fqn}/gnss"
    usb_cameras_namespace = PathJoinSubstitution(["/", namespace, "usb_cameras"])
    usb_cameras_namespace_value = f"{namespace_fqn}/usb_cameras"
    depth_camera_namespace = PathJoinSubstitution(["/", namespace, "depth_camera"])
    depth_camera_namespace_value = f"{namespace_fqn}/depth_camera"
    imu_namespace = PathJoinSubstitution(["/", namespace, "imu"])
    imu_namespace_value = f"{namespace_fqn}/imu"

    log_level = LaunchConfiguration("log_level")
    ublox_log_level = LaunchConfiguration("ublox_log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_amr_sweeper_description = _as_bool(context, "use_amr_sweeper_description")
    use_amr_sweeper_ros2_control = _as_bool(context, "use_amr_sweeper_ros2_control")
    use_joint_broadcaster = LaunchConfiguration("use_joint_broadcaster")
    use_amr_sweeper_battery = _as_bool(context, "use_amr_sweeper_battery")
    use_amr_sweeper_system_info = _as_bool(context, "use_amr_sweeper_system_info")
    use_amr_sweeper_usb_cameras = _as_bool(context, "use_amr_sweeper_usb_cameras")
    use_amr_sweeper_depth_camera = _as_bool(context, "use_amr_sweeper_depth_camera")
    use_amr_sweeper_imu = _as_bool(context, "use_amr_sweeper_imu")
    use_amr_sweeper_gnss = _as_bool(context, "use_amr_sweeper_gnss")
    use_ntrip_client = _as_bool(context, "use_ntrip_client")
    depth_camera_use_laserscan = _as_bool(context, "depth_camera_use_laserscan")

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

    stages = []

    if use_amr_sweeper_description:
        stages.append(StageSpec(
            label="robot_description",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: robot_description"),
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
            wait_nodes=[_node_fqn(namespace_value, "robot_state_publisher")],
            wait_services=[],
            timeout_sec=20.0,
        ))

    if use_amr_sweeper_system_info:
        stages.append(StageSpec(
            label="system_info",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: system_info"),
                Node(
                    package="amr_sweeper_system_info",
                    executable="system_info_node",
                    namespace=namespace,
                    name="amr_sweeper_system_info_node",
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    parameters=[system_info_params_file],
                ),
            ],
            wait_nodes=[_node_fqn(namespace_value, "amr_sweeper_system_info_node")],
            wait_services=[],
        ))

    if use_amr_sweeper_battery:
        stages.append(StageSpec(
            label="battery",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: battery"),
                Node(
                    package="amr_sweeper_battery",
                    executable="battery_node",
                    namespace=namespace,
                    name="amr_sweeper_battery_node",
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    parameters=[
                        battery_params_file,
                        {"can_interface": battery_can_interface},
                    ],
                ),
            ],
            wait_nodes=[_node_fqn(namespace_value, "amr_sweeper_battery_node")],
            wait_services=[],
        ))

    if use_amr_sweeper_gnss:
        gnss_wait_nodes = [
            _node_fqn(gnss_namespace_value, "amr_sweeper_gnss_ublox_node"),
        ]
        if use_ntrip_client:
            gnss_wait_nodes.append(_node_fqn(gnss_namespace_value, "ntrip_client"))

        stages.append(StageSpec(
            label="gnss",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: gnss"),
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
            wait_nodes=gnss_wait_nodes,
            wait_services=[],
            timeout_sec=40.0,
        ))

    if use_amr_sweeper_imu:
        stages.append(StageSpec(
            label="imu",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: imu"),
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
            wait_nodes=[_node_fqn(imu_namespace_value, "imu_node")],
            wait_services=[],
            timeout_sec=40.0,
        ))

    if use_amr_sweeper_usb_cameras:
        usb_wait_nodes = []
        for camera_key in [
            "front_left_camera",
            "front_right_camera",
            "rear_left_camera",
            "rear_right_camera",
            "tools_camera",
        ]:
            if _as_bool(context, f"{camera_key}_enabled"):
                usb_wait_nodes.append(_node_fqn(usb_cameras_namespace_value, f"{camera_key}_node"))

        stages.append(StageSpec(
            label="usb_cameras",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: usb_cameras"),
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
            wait_nodes=usb_wait_nodes,
            wait_services=[],
            timeout_sec=40.0,
        ))

    if use_amr_sweeper_depth_camera:
        depth_wait_nodes = [_node_fqn(depth_camera_namespace_value, "depth_camera")]
        if depth_camera_use_laserscan:
            depth_wait_nodes.append(_node_fqn(depth_camera_namespace_value, "laserscan"))
            depth_wait_nodes.append(_node_fqn(depth_camera_namespace_value, "laserscan_tf"))

        stages.append(StageSpec(
            label="depth_camera",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: depth_camera"),
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
            wait_nodes=depth_wait_nodes,
            wait_services=[],
            timeout_sec=45.0,
        ))

    if use_amr_sweeper_ros2_control:
        stages.append(StageSpec(
            label="ros2_control",
            launch_actions=[
                LogInfo(msg="Layer 1 stage: ros2_control"),
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
            wait_nodes=[_node_fqn(namespace_value, "controller_manager")],
            wait_services=[f"{namespace_fqn}/controller_manager/list_controllers"],
            timeout_sec=60.0,
        ))

    return stages


def _launch_setup(context, *args, **kwargs):
    stages = _build_stages(context)
    if not stages:
        return [LogInfo(msg="Layer 1 bringup: no enabled stages to launch.")]

    actions = []
    previous_gate = None

    for index, stage in enumerate(stages):
        gate = _wait_action(stage.label, stage.wait_nodes, stage.wait_services, stage.timeout_sec)
        stage_bundle = [*stage.launch_actions, gate]
        if index == 0:
            actions.extend(stage_bundle)
        else:
            actions.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=previous_gate,
                    on_exit=_gate_then_launch(stage_bundle),
                )
            ))
        previous_gate = gate

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("ublox_log_level", default_value="WARN"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

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
