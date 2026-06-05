"""Launch the layer-1 bringup orchestrator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}]: {message}"
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    ublox_log_level = LaunchConfiguration("ublox_log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    readiness_config_file = LaunchConfiguration("readiness_config_file")

    use_amr_sweeper_description = LaunchConfiguration("use_amr_sweeper_description")
    use_amr_sweeper_ros2_control = LaunchConfiguration("use_amr_sweeper_ros2_control")
    use_amr_sweeper_battery = LaunchConfiguration("use_amr_sweeper_battery")
    use_amr_sweeper_system_info = LaunchConfiguration("use_amr_sweeper_system_info")
    use_amr_sweeper_usb_cameras = LaunchConfiguration("use_amr_sweeper_usb_cameras")
    use_amr_sweeper_depth_camera = LaunchConfiguration("use_amr_sweeper_depth_camera")
    use_amr_sweeper_imu = LaunchConfiguration("use_amr_sweeper_imu")
    use_amr_sweeper_gnss = LaunchConfiguration("use_amr_sweeper_gnss")
    use_ntrip_client = LaunchConfiguration("use_ntrip_client")

    battery_can_interface = LaunchConfiguration("battery_can_interface")
    battery_params_file = LaunchConfiguration("battery_params_file")
    system_info_params_file = LaunchConfiguration("system_info_params_file")
    depth_camera_params_file = LaunchConfiguration("depth_camera_params_file")
    depth_camera_use_laserscan = LaunchConfiguration("depth_camera_use_laserscan")
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
    front_left_camera_enabled = LaunchConfiguration("front_left_camera_enabled")
    front_right_camera_enabled = LaunchConfiguration("front_right_camera_enabled")
    rear_left_camera_enabled = LaunchConfiguration("rear_left_camera_enabled")
    rear_right_camera_enabled = LaunchConfiguration("rear_right_camera_enabled")
    tools_camera_enabled = LaunchConfiguration("tools_camera_enabled")

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        console_output_format,
    ))
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
    ld.add_action(DeclareLaunchArgument("front_left_camera_enabled", default_value="false"))
    ld.add_action(DeclareLaunchArgument("front_right_camera_enabled", default_value="false"))
    ld.add_action(DeclareLaunchArgument("rear_left_camera_enabled", default_value="true"))
    ld.add_action(DeclareLaunchArgument("rear_right_camera_enabled", default_value="true"))
    ld.add_action(DeclareLaunchArgument("tools_camera_enabled", default_value="true"))

    ld.add_action(Node(
        package="amr_sweeper_layer_1_hardware_bringup",
        executable="layer_1_hardware_bringup_node",
        namespace=namespace,
        name="layer_1_hardware_bringup_node",
        output="screen",
        parameters=[{
            "namespace": namespace,
            "log_level": log_level,
            "ublox_log_level": ublox_log_level,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "readiness_config_file": readiness_config_file,
            "use_amr_sweeper_description": ParameterValue(use_amr_sweeper_description, value_type=bool),
            "use_amr_sweeper_ros2_control": ParameterValue(use_amr_sweeper_ros2_control, value_type=bool),
            "use_amr_sweeper_battery": ParameterValue(use_amr_sweeper_battery, value_type=bool),
            "use_amr_sweeper_system_info": ParameterValue(use_amr_sweeper_system_info, value_type=bool),
            "use_amr_sweeper_usb_cameras": ParameterValue(use_amr_sweeper_usb_cameras, value_type=bool),
            "use_amr_sweeper_depth_camera": ParameterValue(use_amr_sweeper_depth_camera, value_type=bool),
            "use_amr_sweeper_imu": ParameterValue(use_amr_sweeper_imu, value_type=bool),
            "use_amr_sweeper_gnss": ParameterValue(use_amr_sweeper_gnss, value_type=bool),
            "use_ntrip_client": ParameterValue(use_ntrip_client, value_type=bool),
            "battery_can_interface": battery_can_interface,
            "battery_params_file": battery_params_file,
            "system_info_params_file": system_info_params_file,
            "depth_camera_params_file": depth_camera_params_file,
            "depth_camera_use_laserscan": ParameterValue(depth_camera_use_laserscan, value_type=bool),
            "depth_camera_laserscan_params_file": depth_camera_laserscan_params_file,
            "depth_camera_image_topic": depth_camera_image_topic,
            "depth_camera_info_topic": depth_camera_info_topic,
            "depth_camera_frame": depth_camera_frame,
            "depth_camera_scan_topic": depth_camera_scan_topic,
            "depth_camera_output_frame": depth_camera_output_frame,
            "depth_camera_range_min": depth_camera_range_min,
            "depth_camera_range_max": depth_camera_range_max,
            "depth_camera_scan_height": depth_camera_scan_height,
            "depth_camera_scan_tilt_angle_deg": depth_camera_scan_tilt_angle_deg,
            "depth_camera_scan_time": depth_camera_scan_time,
            "depth_camera_camera_domain_id": depth_camera_camera_domain_id,
            "imu_device_path": imu_device_path,
            "imu_port": imu_port,
            "imu_baud": imu_baud,
            "imu_params_file": imu_params_file,
            "gnss_frame_id": gnss_frame_id,
            "ntrip_params_file": ntrip_params_file,
            "front_left_camera_enabled": ParameterValue(front_left_camera_enabled, value_type=bool),
            "front_right_camera_enabled": ParameterValue(front_right_camera_enabled, value_type=bool),
            "rear_left_camera_enabled": ParameterValue(rear_left_camera_enabled, value_type=bool),
            "rear_right_camera_enabled": ParameterValue(rear_right_camera_enabled, value_type=bool),
            "tools_camera_enabled": ParameterValue(tools_camera_enabled, value_type=bool),
        }],
    ))
    return ld
