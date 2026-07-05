"""Launch the AMR Sweeper GNSS data source for hardware or simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_ublox_dgnss_node = LaunchConfiguration("use_ublox_dgnss_node")
    use_simulation = LaunchConfiguration("use_simulation")
    gnss_namespace = LaunchConfiguration("gnss_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    hardware_condition = IfCondition(PythonExpression([
        "'", use_ublox_dgnss_node, "' == 'true' and '", use_simulation, "' != 'true'"
    ]))
    simulation_condition = IfCondition(PythonExpression([
        "'", use_ublox_dgnss_node, "' == 'true' and '", use_simulation, "' == 'true'"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ublox_dgnss_node",
            default_value=TextSubstitution(text="true"),
            description="Launch the GNSS source node",
        ),
        DeclareLaunchArgument(
            "use_simulation",
            default_value=TextSubstitution(text="false"),
            description="Use the simulated GNSS source instead of the hardware u-blox node",
        ),
        DeclareLaunchArgument(
            "use_ublox_nav_sat_fix_hp",
            default_value=TextSubstitution(text="true"),
            description="Deprecated compatibility argument; the local node publishes navsat directly",
        ),
        DeclareLaunchArgument("gnss_namespace", default_value=TextSubstitution(text="amr_sweeper/gnss")),
        DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("gnss_frame_id", default_value=TextSubstitution(text="gnss_link")),
        DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="WARN")),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "amr_sweeper_gnss.yaml",
            ]),
        ),
        DeclareLaunchArgument("world_name", default_value=TextSubstitution(text="amr_sweeper_test")),
        DeclareLaunchArgument("pose_topic", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("navsat_topic", default_value=TextSubstitution(text="navsat")),
        DeclareLaunchArgument("gpsfix_topic", default_value=TextSubstitution(text="fix")),
        DeclareLaunchArgument("odometry_topic", default_value=TextSubstitution(text="odometry")),
        DeclareLaunchArgument("status_marker_topic", default_value=TextSubstitution(text="status_marker")),
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
        DeclareLaunchArgument("min_fix_type", default_value=TextSubstitution(text="1")),
        DeclareLaunchArgument("min_horizontal_stddev_m", default_value=TextSubstitution(text="0.5")),
        DeclareLaunchArgument("min_vertical_stddev_m", default_value=TextSubstitution(text="1.0")),
        DeclareLaunchArgument("horizontal_covariance_scale", default_value=TextSubstitution(text="4.0")),
        DeclareLaunchArgument("vertical_covariance_scale", default_value=TextSubstitution(text="4.0")),
        DeclareLaunchArgument("use_hacc_vacc_covariance_floor", default_value=TextSubstitution(text="true")),
        Node(
            package="amr_sweeper_gnss",
            executable="gnss_node",
            namespace=gnss_namespace,
            name="gnss_node",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                params_file,
                {
                    "frame_id": gnss_frame_id,
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=hardware_condition,
        ),
        Node(
            package="amr_sweeper_gnss",
            executable="simulated_gnss_node",
            namespace=gnss_namespace,
            name="gnss_node",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "world_name": ParameterValue(LaunchConfiguration("world_name"), value_type=str),
                "pose_topic": ParameterValue(LaunchConfiguration("pose_topic"), value_type=str),
                "navsat_topic": ParameterValue(LaunchConfiguration("navsat_topic"), value_type=str),
                "gpsfix_topic": ParameterValue(LaunchConfiguration("gpsfix_topic"), value_type=str),
                "odometry_topic": ParameterValue(LaunchConfiguration("odometry_topic"), value_type=str),
                "status_marker_topic": ParameterValue(LaunchConfiguration("status_marker_topic"), value_type=str),
                "rtcm_topic": ParameterValue(LaunchConfiguration("rtcm_topic"), value_type=str),
                "frame_id": ParameterValue(gnss_frame_id, value_type=str),
                "origin_lat": ParameterValue(LaunchConfiguration("origin_lat"), value_type=float),
                "origin_lon": ParameterValue(LaunchConfiguration("origin_lon"), value_type=float),
                "origin_alt": ParameterValue(LaunchConfiguration("origin_alt"), value_type=float),
                "publish_rate_hz": ParameterValue(LaunchConfiguration("publish_rate_hz"), value_type=float),
                "noise_correlation_tau_s": ParameterValue(LaunchConfiguration("noise_correlation_tau_s"), value_type=float),
                "autonomous_noise_h_m": ParameterValue(LaunchConfiguration("autonomous_noise_h_m"), value_type=float),
                "autonomous_noise_v_m": ParameterValue(LaunchConfiguration("autonomous_noise_v_m"), value_type=float),
                "dgps_noise_h_m": ParameterValue(LaunchConfiguration("dgps_noise_h_m"), value_type=float),
                "dgps_noise_v_m": ParameterValue(LaunchConfiguration("dgps_noise_v_m"), value_type=float),
                "rtk_float_noise_h_m": ParameterValue(LaunchConfiguration("rtk_float_noise_h_m"), value_type=float),
                "rtk_float_noise_v_m": ParameterValue(LaunchConfiguration("rtk_float_noise_v_m"), value_type=float),
                "rtk_fixed_noise_h_m": ParameterValue(LaunchConfiguration("rtk_fixed_noise_h_m"), value_type=float),
                "rtk_fixed_noise_v_m": ParameterValue(LaunchConfiguration("rtk_fixed_noise_v_m"), value_type=float),
                "autonomous_satellites": ParameterValue(LaunchConfiguration("autonomous_satellites"), value_type=int),
                "corrected_satellites": ParameterValue(LaunchConfiguration("corrected_satellites"), value_type=int),
                "correction_timeout_s": ParameterValue(LaunchConfiguration("correction_timeout_s"), value_type=float),
                "dgps_warmup_s": ParameterValue(LaunchConfiguration("dgps_warmup_s"), value_type=float),
                "rtk_float_warmup_s": ParameterValue(LaunchConfiguration("rtk_float_warmup_s"), value_type=float),
                "min_fix_type": ParameterValue(LaunchConfiguration("min_fix_type"), value_type=int),
                "min_horizontal_stddev_m": ParameterValue(LaunchConfiguration("min_horizontal_stddev_m"), value_type=float),
                "min_vertical_stddev_m": ParameterValue(LaunchConfiguration("min_vertical_stddev_m"), value_type=float),
                "horizontal_covariance_scale": ParameterValue(LaunchConfiguration("horizontal_covariance_scale"), value_type=float),
                "vertical_covariance_scale": ParameterValue(LaunchConfiguration("vertical_covariance_scale"), value_type=float),
                "use_hacc_vacc_covariance_floor": ParameterValue(LaunchConfiguration("use_hacc_vacc_covariance_floor"), value_type=bool),
            }],
            condition=simulation_condition,
        ),
    ])
