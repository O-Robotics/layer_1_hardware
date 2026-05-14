"""Launch the AMR Sweeper NTRIP client wrapper."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the upstream ntrip client node."""

    use_ntrip_client_node_arg = DeclareLaunchArgument(
        'use_ntrip_client_node', default_value=TextSubstitution(text='true')
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('amr_sweeper_gnss'),
            'config',
            'ntrip_client.yaml',
        ]),
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='amr_sweeper')
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value=TextSubstitution(text='INFO')
    )
    params = [LaunchConfiguration('params_file')]

    container = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ntrip_client_node',
                plugin='ublox_dgnss::NTRIPClientNode',
                name='ntrip_client',
                namespace=LaunchConfiguration('namespace'),
                parameters=params,
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_ntrip_client_node')),
    )

    return launch.LaunchDescription([
        use_ntrip_client_node_arg,
        params_file_arg,
        namespace_arg,
        log_level_arg,
        container,
    ])
