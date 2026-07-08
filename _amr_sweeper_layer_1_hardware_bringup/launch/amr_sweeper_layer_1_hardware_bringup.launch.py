"""Launch the layer-1 hardware stack from package launch files."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _child_namespace(root_namespace: str, child_name: str) -> str:
    root = root_namespace.strip().strip("/")
    return f"{root}/{child_name}" if root else child_name


def _scoped_include(package_name: str, launch_file_name: str, launch_arguments: dict[str, str]):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(package_name, launch_file_name)),
                launch_arguments=launch_arguments.items(),
            ),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_simulation = LaunchConfiguration("use_simulation").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    realsense_log_level = LaunchConfiguration("realsense_log_level").perform(context)
    ublox_log_level = LaunchConfiguration("ublox_log_level").perform(context)
    ros2_control_startup_delay_sec = float(
        LaunchConfiguration("ros2_control_startup_delay_sec").perform(context)
    )

    use_amr_sweeper_description = _as_bool(
        LaunchConfiguration("use_amr_sweeper_description").perform(context))
    use_amr_sweeper_ros2_control = _as_bool(
        LaunchConfiguration("use_amr_sweeper_ros2_control").perform(context))
    use_amr_sweeper_battery = _as_bool(
        LaunchConfiguration("use_amr_sweeper_battery").perform(context))
    use_amr_sweeper_system_info = _as_bool(
        LaunchConfiguration("use_amr_sweeper_system_info").perform(context))
    use_amr_sweeper_usb_cameras = _as_bool(
        LaunchConfiguration("use_amr_sweeper_usb_cameras").perform(context))
    use_amr_sweeper_depth_camera = _as_bool(
        LaunchConfiguration("use_amr_sweeper_depth_camera").perform(context))
    use_amr_sweeper_imu = _as_bool(
        LaunchConfiguration("use_amr_sweeper_imu").perform(context))
    use_amr_sweeper_gnss = _as_bool(
        LaunchConfiguration("use_amr_sweeper_gnss").perform(context))
    use_ntrip_client = _as_bool(LaunchConfiguration("use_ntrip_client").perform(context))

    actions = []

    if use_amr_sweeper_description:
        actions.append(_scoped_include(
            "amr_sweeper_description",
            "amr_sweeper_description.launch.py",
            {
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "use_simulation": use_simulation,
                "use_ros2_control": "true" if use_amr_sweeper_ros2_control else "false",
                "enable_usb_cameras": "true" if use_amr_sweeper_usb_cameras else "false",
                "enable_gnss": "true" if use_amr_sweeper_gnss else "false",
                "enable_imu": "true" if use_amr_sweeper_imu else "false",
                "enable_depth_camera": "true" if use_amr_sweeper_depth_camera else "false",
            },
        ))

    if use_amr_sweeper_ros2_control:
        ros2_control_include = _scoped_include(
            "amr_sweeper_ros2_control",
            "amr_sweeper_ros2_control.launch.py",
            {
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "use_simulation": use_simulation,
                "use_ros2_control": "true",
            },
        )
        if ros2_control_startup_delay_sec > 0.0:
            actions.append(
                TimerAction(
                    period=ros2_control_startup_delay_sec,
                    actions=[ros2_control_include],
                )
            )
        else:
            actions.append(ros2_control_include)

    if use_amr_sweeper_system_info:
        actions.append(_scoped_include(
            "amr_sweeper_system_info",
            "amr_sweeper_system_info.launch.py",
            {
                "namespace": _child_namespace(namespace, "system_info"),
                "use_simulation": use_simulation,
                "params_file": LaunchConfiguration("system_info_params_file").perform(context),
            },
        ))

    if use_amr_sweeper_battery:
        actions.append(_scoped_include(
            "amr_sweeper_battery",
            "amr_sweeper_battery.launch.py",
            {
                "namespace": _child_namespace(namespace, "battery"),
                "can_interface": LaunchConfiguration("battery_can_interface").perform(context),
                "use_simulation": use_simulation,
                "params_file": LaunchConfiguration("battery_params_file").perform(context),
            },
        ))

    if use_amr_sweeper_gnss:
        actions.append(_scoped_include(
            "amr_sweeper_gnss",
            "amr_sweeper_gnss.launch.py",
            {
                "gnss_namespace": _child_namespace(namespace, "gnss"),
                "use_simulation": use_simulation,
                "use_sim_time": use_sim_time,
                "use_ntrip_client": "true" if use_ntrip_client else "false",
                "use_nmea_to_caster": "true" if use_ntrip_client else "false",
                "gnss_frame_id": LaunchConfiguration("gnss_frame_id").perform(context),
                "ntrip_params_file": LaunchConfiguration("ntrip_params_file").perform(context),
                "pose_topic": LaunchConfiguration("simulation_pose_topic").perform(context),
                "log_level": log_level,
                "ublox_log_level": ublox_log_level,
            },
        ))

    if use_amr_sweeper_imu:
        actions.append(_scoped_include(
            "amr_sweeper_imu",
            "amr_sweeper_imu.launch.py",
            {
                "namespace": _child_namespace(namespace, "imu"),
                "use_sim_time": use_sim_time,
                "use_simulation": use_simulation,
                "device_path": LaunchConfiguration("imu_device_path").perform(context),
                "port": LaunchConfiguration("imu_port").perform(context),
                "baud": LaunchConfiguration("imu_baud").perform(context),
                "params_file": LaunchConfiguration("imu_params_file").perform(context),
            },
        ))

    if use_amr_sweeper_usb_cameras:
        actions.append(_scoped_include(
            "amr_sweeper_usb_cameras",
            "amr_sweeper_usb_cameras.launch.py",
            {
                "namespace": _child_namespace(namespace, "usb_cameras"),
                "log_level": log_level,
                "use_simulation": use_simulation,
                "front_left_camera_enabled": LaunchConfiguration("front_left_camera_enabled").perform(context),
                "front_right_camera_enabled": LaunchConfiguration("front_right_camera_enabled").perform(context),
                "rear_left_camera_enabled": LaunchConfiguration("rear_left_camera_enabled").perform(context),
                "rear_right_camera_enabled": LaunchConfiguration("rear_right_camera_enabled").perform(context),
                "tools_camera_enabled": LaunchConfiguration("tools_camera_enabled").perform(context),
            },
        ))

    if use_amr_sweeper_depth_camera:
        actions.append(_scoped_include(
            "amr_sweeper_depth_camera",
            "amr_sweeper_depth_camera.launch.py",
            {
                "namespace": _child_namespace(namespace, "depth_camera"),
                "log_level": log_level,
                "realsense_log_level": realsense_log_level,
                "use_sim_time": use_sim_time,
                "use_simulation": use_simulation,
                "camera_domain_id": LaunchConfiguration("depth_camera_camera_domain_id").perform(context),
                "params_file": LaunchConfiguration("depth_camera_params_file").perform(context),
                "use_laserscan": LaunchConfiguration("depth_camera_use_laserscan").perform(context),
                "laserscan_params_file": LaunchConfiguration("depth_camera_laserscan_params_file").perform(context),
                "depth_image_topic": LaunchConfiguration("depth_camera_image_topic").perform(context),
                "depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic").perform(context),
                "depth_camera_frame": LaunchConfiguration("depth_camera_frame").perform(context),
                "scan_topic": LaunchConfiguration("depth_camera_scan_topic").perform(context),
                "output_frame": LaunchConfiguration("depth_camera_output_frame").perform(context),
                "range_min": LaunchConfiguration("depth_camera_range_min").perform(context),
                "range_max": LaunchConfiguration("depth_camera_range_max").perform(context),
                "scan_height": LaunchConfiguration("depth_camera_scan_height").perform(context),
                "scan_tilt_angle_deg": LaunchConfiguration("depth_camera_scan_tilt_angle_deg").perform(context),
                "scan_time": LaunchConfiguration("depth_camera_scan_time").perform(context),
            },
        ))

    return actions


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("realsense_log_level", default_value="error"),
        DeclareLaunchArgument("ublox_log_level", default_value="WARN"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_simulation", default_value="false"),
        DeclareLaunchArgument("use_amr_sweeper_description", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_ros2_control", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_battery", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_system_info", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_usb_cameras", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_depth_camera", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_imu", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_gnss", default_value="true"),
        DeclareLaunchArgument("use_ntrip_client", default_value="true"),
        DeclareLaunchArgument("ros2_control_startup_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("battery_can_interface", default_value="can0"),
        DeclareLaunchArgument("battery_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_battery"),
            "config",
            "amr_sweeper_battery.yaml",
        ])),
        DeclareLaunchArgument("system_info_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_system_info"),
            "config",
            "amr_sweeper_system_info.yaml",
        ])),
        DeclareLaunchArgument("depth_camera_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_depth_camera"),
            "config",
            "amr_sweeper_depth_camera.yaml",
        ])),
        DeclareLaunchArgument("depth_camera_use_laserscan", default_value="true"),
        DeclareLaunchArgument("depth_camera_laserscan_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_depth_camera"),
            "config",
            "laserscan.yaml",
        ])),
        DeclareLaunchArgument("depth_camera_image_topic", default_value=""),
        DeclareLaunchArgument("depth_camera_info_topic", default_value=""),
        DeclareLaunchArgument("depth_camera_frame", default_value="depth_camera_depth_frame"),
        DeclareLaunchArgument("depth_camera_scan_topic", default_value="scan"),
        DeclareLaunchArgument("depth_camera_output_frame", default_value=""),
        DeclareLaunchArgument("depth_camera_range_min", default_value=""),
        DeclareLaunchArgument("depth_camera_range_max", default_value=""),
        DeclareLaunchArgument("depth_camera_scan_height", default_value=""),
        DeclareLaunchArgument("depth_camera_scan_tilt_angle_deg", default_value=""),
        DeclareLaunchArgument("depth_camera_scan_time", default_value=""),
        DeclareLaunchArgument("depth_camera_camera_domain_id", default_value="5"),
        DeclareLaunchArgument("imu_device_path", default_value=""),
        DeclareLaunchArgument("imu_port", default_value=""),
        DeclareLaunchArgument("imu_baud", default_value=""),
        DeclareLaunchArgument("imu_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_imu"),
            "config",
            "amr_sweeper_imu.yaml",
        ])),
        DeclareLaunchArgument("gnss_frame_id", default_value="gnss_link"),
        DeclareLaunchArgument("ntrip_params_file", default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_gnss"),
            "config",
            "amr_sweeper_gnss_ntrip_client.yaml",
        ])),
        DeclareLaunchArgument(
            "simulation_pose_topic",
            default_value="/amr_sweeper/simulation/pose/info",
        ),
        DeclareLaunchArgument("front_left_camera_enabled", default_value="false"),
        DeclareLaunchArgument("front_right_camera_enabled", default_value="false"),
        DeclareLaunchArgument("rear_left_camera_enabled", default_value="true"),
        DeclareLaunchArgument("rear_right_camera_enabled", default_value="true"),
        DeclareLaunchArgument("tools_camera_enabled", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
