"""Launch the local AMR Sweeper u-blox receiver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_ublox_dgnss_node = LaunchConfiguration("use_ublox_dgnss_node")
    use_simulation = LaunchConfiguration("use_simulation")
    gnss_namespace = LaunchConfiguration("gnss_namespace")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ublox_dgnss_node",
            default_value=TextSubstitution(text="true"),
            description="Launch the local AMR Sweeper u-blox node",
        ),
        DeclareLaunchArgument(
            "use_simulation",
            default_value=TextSubstitution(text="false"),
            description="Disable the local hardware GNSS node when running in simulation",
        ),
        DeclareLaunchArgument(
            "use_ublox_nav_sat_fix_hp",
            default_value=TextSubstitution(text="true"),
            description="Deprecated compatibility argument; the local node publishes navsat directly",
        ),
        DeclareLaunchArgument(
            "gnss_namespace",
            default_value=TextSubstitution(text="amr_sweeper/gnss"),
        ),
        DeclareLaunchArgument(
            "gnss_frame_id",
            default_value=TextSubstitution(text="gnss_link"),
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text="WARN"),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_gnss"),
                "config",
                "amr_sweeper_gnss.yaml",
            ]),
        ),
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
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", use_ublox_dgnss_node, "' == 'true' and '", use_simulation, "' != 'true'"
            ])),
        ),
    ])
