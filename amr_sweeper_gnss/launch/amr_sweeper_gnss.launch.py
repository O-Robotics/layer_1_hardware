"""Launch the full AMR Sweeper GNSS stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    namespace = LaunchConfiguration("namespace")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")
    device_family = LaunchConfiguration("device_family")
    device_serial_string = LaunchConfiguration("device_serial_string")
    log_level = LaunchConfiguration("log_level")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ublox_dgnss_node",
            default_value=TextSubstitution(text="true"),
        ),
        DeclareLaunchArgument(
            "use_ublox_nav_sat_fix_hp",
            default_value=TextSubstitution(text="true"),
        ),
        DeclareLaunchArgument(
            "use_ntrip_client",
            default_value=TextSubstitution(text="true"),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=TextSubstitution(text="amr_sweeper/gnss"),
        ),
        DeclareLaunchArgument(
            "gnss_frame_id",
            default_value=TextSubstitution(text="gnss_link"),
        ),
        DeclareLaunchArgument(
            "device_family",
            default_value=TextSubstitution(text="F9P"),
        ),
        DeclareLaunchArgument(
            "device_serial_string",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text="INFO"),
        ),
        DeclareLaunchArgument(
            "ntrip_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "ntrip_client.yaml",
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ublox_rover_hpposllh_navsatfix.launch.py")),
            launch_arguments={
                "use_ublox_dgnss_node": use_ublox_dgnss_node,
                "use_ublox_nav_sat_fix_hp": use_ublox_nav_sat_fix_hp,
                "namespace": namespace,
                "gnss_frame_id": gnss_frame_id,
                "device_family": device_family,
                "device_serial_string": device_serial_string,
                "log_level": log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ntrip_client.launch.py")),
            launch_arguments={
                "use_ntrip_client_node": use_ntrip_client,
                "namespace": namespace,
                "params_file": ntrip_params_file,
                "log_level": log_level,
            }.items(),
            condition=IfCondition(use_ntrip_client),
        ),
    ])
