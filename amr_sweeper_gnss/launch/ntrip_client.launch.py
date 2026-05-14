"""Launch amr_sweeper_gnss NTRIP client component."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for ntrip client component."""

    use_ntrip_client_node_arg = DeclareLaunchArgument(
        'use_ntrip_client_node', default_value=TextSubstitution(text='true')
    )
    use_https_arg = DeclareLaunchArgument(
        'use_https', default_value=TextSubstitution(text='false')
    )
    host_arg = DeclareLaunchArgument(
        'host', default_value=TextSubstitution(text='rtk2go.com')
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value=TextSubstitution(text='2101')
    )
    mountpoint_arg = DeclareLaunchArgument(
        'mountpoint', default_value=TextSubstitution(text='REPLACE_WITH_RTK2GO_MOUNTPOINT')
    )
    username_arg = DeclareLaunchArgument(
        'username',
        default_value=EnvironmentVariable(
            name='NTRIP_USERNAME',
            default_value='REPLACE_WITH_VALID_EMAIL',
        )
    )
    password_arg = DeclareLaunchArgument(
        'password', default_value=EnvironmentVariable(name='NTRIP_PASSWORD', default_value='none')
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

    params = [
        LaunchConfiguration('params_file'),
        {
            'use_https': LaunchConfiguration('use_https'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'mountpoint': LaunchConfiguration('mountpoint'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
        },
    ]

    container1 = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='amr_sweeper_gnss',
                plugin='amr_sweeper_ublox_dgnss::NTRIPClientNode',
                name='ntrip_client',
                parameters=params,
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_ntrip_client_node')),
    )

    return launch.LaunchDescription([
        use_ntrip_client_node_arg,
        use_https_arg,
        host_arg,
        port_arg,
        mountpoint_arg,
        username_arg,
        password_arg,
        params_file_arg,
        namespace_arg,
        container1,
    ])
