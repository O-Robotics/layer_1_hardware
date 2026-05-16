"""Launch the AMR Sweeper NTRIP client wrapper."""

import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the Jazzy ntrip_client node."""

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
    gnss_namespace_arg = DeclareLaunchArgument(
        'gnss_namespace', default_value=TextSubstitution(text='amr_sweeper/gnss')
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value=TextSubstitution(text='INFO')
    )
    ntrip_debug = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG',
        value='true',
        condition=IfCondition(LaunchConfiguration('debug')),
    )

    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        namespace=LaunchConfiguration('gnss_namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('fix', 'navsat'),
        ],
        condition=IfCondition(LaunchConfiguration('use_ntrip_client_node')),
    )

    return launch.LaunchDescription([
        use_ntrip_client_node_arg,
        params_file_arg,
        gnss_namespace_arg,
        log_level_arg,
        DeclareLaunchArgument(
            'debug', default_value=TextSubstitution(text='false')
        ),
        ntrip_debug,
        ntrip_node,
    ])
