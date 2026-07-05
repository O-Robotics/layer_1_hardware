"""Launch the AMR Sweeper NTRIP client wrapper for hardware or simulation."""

import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_nmea_to_caster_arg = DeclareLaunchArgument(
        "use_nmea_to_caster", default_value=TextSubstitution(text="true")
    )
    fix_topic_arg = DeclareLaunchArgument(
        "fix_topic", default_value=TextSubstitution(text="navsat")
    )
    use_ntrip_client_node_arg = DeclareLaunchArgument(
        "use_ntrip_client_node", default_value=TextSubstitution(text="true")
    )
    use_simulation_arg = DeclareLaunchArgument(
        "use_simulation", default_value=TextSubstitution(text="false")
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=TextSubstitution(text="false")
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("amr_sweeper_gnss"),
            "config",
            "amr_sweeper_gnss_ntrip_client.yaml",
        ]),
    )
    gnss_namespace_arg = DeclareLaunchArgument(
        "gnss_namespace", default_value=TextSubstitution(text="amr_sweeper/gnss")
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    simulated_publish_rate_arg = DeclareLaunchArgument(
        "simulated_publish_rate_hz", default_value=TextSubstitution(text="1.0")
    )
    simulated_startup_delay_arg = DeclareLaunchArgument(
        "simulated_startup_delay_s", default_value=TextSubstitution(text="1.5")
    )
    rtcm_topic_arg = DeclareLaunchArgument(
        "rtcm_topic", default_value=TextSubstitution(text="ntrip_client/rtcm")
    )
    gnss_frame_id_arg = DeclareLaunchArgument(
        "gnss_frame_id", default_value=TextSubstitution(text="gnss_link")
    )

    ntrip_debug = SetEnvironmentVariable(
        name="NTRIP_CLIENT_DEBUG",
        value="true",
        condition=IfCondition(LaunchConfiguration("debug")),
    )

    hardware_ntrip_node = Node(
        package="amr_sweeper_gnss",
        executable="ntrip_client",
        name="ntrip_client_node",
        namespace=LaunchConfiguration("gnss_namespace"),
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "send_nmea": ParameterValue(LaunchConfiguration("use_nmea_to_caster"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
        remappings=[
            ("fix", LaunchConfiguration("fix_topic")),
            ("nmea", "disabled/nmea"),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("use_ntrip_client_node"), "' == 'true' and '",
                LaunchConfiguration("use_simulation"), "' != 'true'"
            ])
        ),
    )

    simulated_ntrip_node = Node(
        package="amr_sweeper_gnss",
        executable="simulated_ntrip_client",
        name="ntrip_client_node",
        namespace=LaunchConfiguration("gnss_namespace"),
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "rtcm_topic": ParameterValue(LaunchConfiguration("rtcm_topic"), value_type=str),
            "rtcm_frame_id": ParameterValue(LaunchConfiguration("gnss_frame_id"), value_type=str),
            "publish_rate_hz": ParameterValue(LaunchConfiguration("simulated_publish_rate_hz"), value_type=float),
            "startup_delay_s": ParameterValue(LaunchConfiguration("simulated_startup_delay_s"), value_type=float),
            "send_nmea": ParameterValue(LaunchConfiguration("use_nmea_to_caster"), value_type=bool),
        }],
        remappings=[
            ("fix", LaunchConfiguration("fix_topic")),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("use_ntrip_client_node"), "' == 'true' and '",
                LaunchConfiguration("use_simulation"), "' == 'true'"
            ])
        ),
    )

    return launch.LaunchDescription([
        use_nmea_to_caster_arg,
        fix_topic_arg,
        use_ntrip_client_node_arg,
        use_simulation_arg,
        use_sim_time_arg,
        params_file_arg,
        gnss_namespace_arg,
        log_level_arg,
        simulated_publish_rate_arg,
        simulated_startup_delay_arg,
        rtcm_topic_arg,
        gnss_frame_id_arg,
        DeclareLaunchArgument("debug", default_value=TextSubstitution(text="false")),
        ntrip_debug,
        hardware_ntrip_node,
        simulated_ntrip_node,
    ])
