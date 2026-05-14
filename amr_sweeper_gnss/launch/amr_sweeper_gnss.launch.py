"""Launch the full AMR Sweeper GNSS stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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
    ntrip_use_https = LaunchConfiguration("ntrip_use_https")
    ntrip_host = LaunchConfiguration("ntrip_host")
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_password = LaunchConfiguration("ntrip_password")

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
            default_value=TextSubstitution(text="amr_sweeper"),
        ),
        DeclareLaunchArgument(
            "gnss_frame_id",
            default_value=TextSubstitution(text="gnss_link"),
        ),
        DeclareLaunchArgument(
            "ntrip_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "ntrip_client.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "ntrip_use_https",
            default_value=TextSubstitution(text="false"),
        ),
        DeclareLaunchArgument(
            "ntrip_host",
            default_value=TextSubstitution(text="rtk2go.com"),
        ),
        DeclareLaunchArgument(
            "ntrip_port",
            default_value=TextSubstitution(text="2101"),
        ),
        DeclareLaunchArgument(
            "ntrip_mountpoint",
            default_value=TextSubstitution(text="REPLACE_WITH_RTK2GO_MOUNTPOINT"),
        ),
        DeclareLaunchArgument(
            "ntrip_username",
            default_value=EnvironmentVariable(
                name="NTRIP_USERNAME",
                default_value="REPLACE_WITH_VALID_EMAIL",
            ),
        ),
        DeclareLaunchArgument(
            "ntrip_password",
            default_value=EnvironmentVariable(name="NTRIP_PASSWORD", default_value="none"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ublox_rover_hpposllh_navsatfix.launch.py")),
            launch_arguments={
                "use_ublox_dgnss_node": use_ublox_dgnss_node,
                "use_ublox_nav_sat_fix_hp": use_ublox_nav_sat_fix_hp,
                "namespace": namespace,
                "gnss_frame_id": gnss_frame_id,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("ntrip_client.launch.py")),
            launch_arguments={
                "use_ntrip_client_node": use_ntrip_client,
                "namespace": namespace,
                "params_file": ntrip_params_file,
                "use_https": ntrip_use_https,
                "host": ntrip_host,
                "port": ntrip_port,
                "mountpoint": ntrip_mountpoint,
                "username": ntrip_username,
                "password": ntrip_password,
            }.items(),
            condition=IfCondition(use_ntrip_client),
        ),
    ])
