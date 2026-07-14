"""Launch the full AMR Sweeper GNSS stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare("amr_sweeper_gnss"),
        "launch",
        launch_file_name,
    ])


def generate_launch_description():
    use_ublox_dgnss_node = LaunchConfiguration("use_ublox_dgnss_node")
    use_ublox_nav_sat_fix_hp = LaunchConfiguration("use_ublox_nav_sat_fix_hp")
    use_ntrip_client = LaunchConfiguration("use_ntrip_client")
    use_simulation = LaunchConfiguration("use_simulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nmea_to_caster = LaunchConfiguration("use_nmea_to_caster")
    fix_topic = LaunchConfiguration("fix_topic")
    gnss_namespace = LaunchConfiguration("gnss_namespace")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")
    ublox_params_file = LaunchConfiguration("ublox_params_file")
    device_family = LaunchConfiguration("device_family")
    device_serial_string = LaunchConfiguration("device_serial_string")
    log_level = LaunchConfiguration("log_level")
    ublox_log_level = LaunchConfiguration("ublox_log_level")

    return LaunchDescription([
        DeclareLaunchArgument("use_ublox_dgnss_node", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_ublox_nav_sat_fix_hp", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_ntrip_client", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_nmea_to_caster", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_simulation", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("fix_topic", default_value=TextSubstitution(text="navsat")),
        DeclareLaunchArgument("gnss_namespace", default_value=TextSubstitution(text="amr_sweeper/gnss")),
        DeclareLaunchArgument("gnss_frame_id", default_value=TextSubstitution(text="gnss_link")),
        DeclareLaunchArgument("device_family", default_value=TextSubstitution(text="F9P")),
        DeclareLaunchArgument("device_serial_string", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="INFO")),
        DeclareLaunchArgument("ublox_log_level", default_value=TextSubstitution(text="INFO")),
        DeclareLaunchArgument(
            "ntrip_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "amr_sweeper_gnss_ntrip_client.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "ublox_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "amr_sweeper_gnss.yaml",
            ]),
        ),
        DeclareLaunchArgument("robot_pose_topic", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("navsat_topic", default_value=TextSubstitution(text="navsat")),
        DeclareLaunchArgument("gpsfix_topic", default_value=TextSubstitution(text="fix")),
        DeclareLaunchArgument("odometry_topic", default_value=TextSubstitution(text="odometry")),
        DeclareLaunchArgument("rtcm_topic", default_value=TextSubstitution(text="ntrip_client/rtcm")),
        DeclareLaunchArgument("origin_lat", default_value=TextSubstitution(text="56.164520029")),
        DeclareLaunchArgument("origin_lon", default_value=TextSubstitution(text="10.1453534275")),
        DeclareLaunchArgument("origin_alt", default_value=TextSubstitution(text="100.0")),
        DeclareLaunchArgument("publish_rate_hz", default_value=TextSubstitution(text="5.0")),
        DeclareLaunchArgument("noise_correlation_tau_s", default_value=TextSubstitution(text="12.0")),
        DeclareLaunchArgument("autonomous_noise_h_m", default_value=TextSubstitution(text="1.25")),
        DeclareLaunchArgument("autonomous_noise_v_m", default_value=TextSubstitution(text="2.5")),
        DeclareLaunchArgument("dgps_noise_h_m", default_value=TextSubstitution(text="0.7")),
        DeclareLaunchArgument("dgps_noise_v_m", default_value=TextSubstitution(text="1.3")),
        DeclareLaunchArgument("rtk_float_noise_h_m", default_value=TextSubstitution(text="0.3")),
        DeclareLaunchArgument("rtk_float_noise_v_m", default_value=TextSubstitution(text="0.55")),
        DeclareLaunchArgument("rtk_fixed_noise_h_m", default_value=TextSubstitution(text="0.12")),
        DeclareLaunchArgument("rtk_fixed_noise_v_m", default_value=TextSubstitution(text="0.25")),
        DeclareLaunchArgument("autonomous_satellites", default_value=TextSubstitution(text="10")),
        DeclareLaunchArgument("corrected_satellites", default_value=TextSubstitution(text="14")),
        DeclareLaunchArgument("correction_timeout_s", default_value=TextSubstitution(text="3.0")),
        DeclareLaunchArgument("dgps_warmup_s", default_value=TextSubstitution(text="2.0")),
        DeclareLaunchArgument("rtk_float_warmup_s", default_value=TextSubstitution(text="8.0")),
        DeclareLaunchArgument("simulated_ntrip_publish_rate_hz", default_value=TextSubstitution(text="1.0")),
        DeclareLaunchArgument("simulated_ntrip_startup_delay_s", default_value=TextSubstitution(text="1.5")),
        DeclareLaunchArgument("min_horizontal_stddev_m", default_value=TextSubstitution(text="0.5")),
        DeclareLaunchArgument("min_vertical_stddev_m", default_value=TextSubstitution(text="1.0")),
        DeclareLaunchArgument("horizontal_covariance_scale", default_value=TextSubstitution(text="4.0")),
        DeclareLaunchArgument("vertical_covariance_scale", default_value=TextSubstitution(text="4.0")),
        DeclareLaunchArgument("use_hacc_vacc_covariance_floor", default_value=TextSubstitution(text="true")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ublox_dgnss.launch.py")),
            launch_arguments={
                "use_ublox_dgnss_node": use_ublox_dgnss_node,
                "use_ublox_nav_sat_fix_hp": use_ublox_nav_sat_fix_hp,
                "gnss_namespace": gnss_namespace,
                "gnss_frame_id": gnss_frame_id,
                "use_simulation": use_simulation,
                "use_sim_time": use_sim_time,
                "params_file": ublox_params_file,
                "device_family": device_family,
                "device_serial_string": device_serial_string,
                "log_level": ublox_log_level,
                "robot_pose_topic": LaunchConfiguration("robot_pose_topic"),
                "navsat_topic": LaunchConfiguration("navsat_topic"),
                "gpsfix_topic": LaunchConfiguration("gpsfix_topic"),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "rtcm_topic": LaunchConfiguration("rtcm_topic"),
                "origin_lat": LaunchConfiguration("origin_lat"),
                "origin_lon": LaunchConfiguration("origin_lon"),
                "origin_alt": LaunchConfiguration("origin_alt"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "noise_correlation_tau_s": LaunchConfiguration("noise_correlation_tau_s"),
                "autonomous_noise_h_m": LaunchConfiguration("autonomous_noise_h_m"),
                "autonomous_noise_v_m": LaunchConfiguration("autonomous_noise_v_m"),
                "dgps_noise_h_m": LaunchConfiguration("dgps_noise_h_m"),
                "dgps_noise_v_m": LaunchConfiguration("dgps_noise_v_m"),
                "rtk_float_noise_h_m": LaunchConfiguration("rtk_float_noise_h_m"),
                "rtk_float_noise_v_m": LaunchConfiguration("rtk_float_noise_v_m"),
                "rtk_fixed_noise_h_m": LaunchConfiguration("rtk_fixed_noise_h_m"),
                "rtk_fixed_noise_v_m": LaunchConfiguration("rtk_fixed_noise_v_m"),
                "autonomous_satellites": LaunchConfiguration("autonomous_satellites"),
                "corrected_satellites": LaunchConfiguration("corrected_satellites"),
                "correction_timeout_s": LaunchConfiguration("correction_timeout_s"),
                "dgps_warmup_s": LaunchConfiguration("dgps_warmup_s"),
                "rtk_float_warmup_s": LaunchConfiguration("rtk_float_warmup_s"),
                "min_horizontal_stddev_m": LaunchConfiguration("min_horizontal_stddev_m"),
                "min_vertical_stddev_m": LaunchConfiguration("min_vertical_stddev_m"),
                "horizontal_covariance_scale": LaunchConfiguration("horizontal_covariance_scale"),
                "vertical_covariance_scale": LaunchConfiguration("vertical_covariance_scale"),
                "use_hacc_vacc_covariance_floor": LaunchConfiguration("use_hacc_vacc_covariance_floor"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ntrip_client.launch.py")),
            launch_arguments={
                "use_ntrip_client_node": use_ntrip_client,
                "use_nmea_to_caster": use_nmea_to_caster,
                "fix_topic": fix_topic,
                "gnss_namespace": gnss_namespace,
                "use_simulation": use_simulation,
                "use_sim_time": use_sim_time,
                "params_file": ntrip_params_file,
                "log_level": log_level,
                "simulated_publish_rate_hz": LaunchConfiguration("simulated_ntrip_publish_rate_hz"),
                "simulated_startup_delay_s": LaunchConfiguration("simulated_ntrip_startup_delay_s"),
                "rtcm_topic": LaunchConfiguration("rtcm_topic"),
                "gnss_frame_id": gnss_frame_id,
            }.items(),
        ),
    ])
