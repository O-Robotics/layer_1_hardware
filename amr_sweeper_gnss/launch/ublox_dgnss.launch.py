"""Launch the AMR Sweeper rover GNSS wrapper."""

from pathlib import Path

import launch
import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _load_node_parameters(params_file_path: str, node_name: str) -> dict:
    """Load one node's parameter block from a shared YAML file."""
    config = yaml.safe_load(Path(params_file_path).read_text(encoding='utf-8')) or {}
    node_block = config.get(node_name, {})
    ros_parameters = node_block.get('ros__parameters', {})
    if not isinstance(ros_parameters, dict):
        raise ValueError(
            f"Expected '{node_name}.ros__parameters' to be a mapping in {params_file_path}"
        )
    return ros_parameters


def _create_containers(context):
    """Create GNSS component containers with explicit per-node parameters."""
    use_ublox_dgnss_node = LaunchConfiguration('use_ublox_dgnss_node')
    use_ublox_nav_sat_fix_hp = LaunchConfiguration('use_ublox_nav_sat_fix_hp')
    gnss_namespace = LaunchConfiguration('gnss_namespace')
    gnss_frame_id = LaunchConfiguration('gnss_frame_id')
    device_family = LaunchConfiguration('device_family')
    device_serial_string = LaunchConfiguration('device_serial_string')
    log_level = LaunchConfiguration('log_level')
    params_file_path = LaunchConfiguration('params_file').perform(context)

    ublox_dgnss_params = _load_node_parameters(params_file_path, 'ublox_dgnss')
    ublox_nav_sat_fix_hp_params = _load_node_parameters(params_file_path, 'ublox_nav_sat_fix_hp')

    ublox_dgnss_container = ComposableNodeContainer(
        name='ublox_dgnss_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='log',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace=gnss_namespace,
                parameters=[
                    ublox_dgnss_params,
                    {'DEVICE_FAMILY': device_family},
                    {'DEVICE_SERIAL_STRING': device_serial_string},
                    {'FRAME_ID': gnss_frame_id},
                ],
                remappings=[
                    ('/ntrip_client/rtcm', 'rtcm'),
                ],
            )
        ],
        condition=IfCondition(use_ublox_dgnss_node),
    )

    ublox_nav_sat_fix_hp_container = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='log',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                namespace=gnss_namespace,
                parameters=[ublox_nav_sat_fix_hp_params],
                remappings=[('fix', 'navsat')],
            )
        ],
        condition=IfCondition(use_ublox_nav_sat_fix_hp),
    )

    return [ublox_dgnss_container, ublox_nav_sat_fix_hp_container]


def generate_launch_description():
    """Generate launch description for the upstream ublox GNSS components."""
    declare_use_ublox_dgnss_node = DeclareLaunchArgument(
        'use_ublox_dgnss_node',
        default_value=TextSubstitution(text='true'),
        description='Launch UbloxDGNSS component container',
    )

    declare_use_ublox_nav_sat_fix_hp = DeclareLaunchArgument(
        'use_ublox_nav_sat_fix_hp',
        default_value=TextSubstitution(text='true'),
        description='Launch ublox_nav_sat_fix_hp component container',
    )
    declare_gnss_namespace = DeclareLaunchArgument(
        'gnss_namespace',
        default_value=TextSubstitution(text='amr_sweeper/gnss'),
        description='Namespace for GNSS containers',
    )
    declare_frame_id = DeclareLaunchArgument(
        'gnss_frame_id',
        default_value=TextSubstitution(text='gnss_link'),
        description='Frame ID to publish in GNSS message headers',
    )
    declare_device_family = DeclareLaunchArgument(
        'device_family',
        default_value=TextSubstitution(text='F9P'),
        description='u-blox device family',
    )
    declare_device_serial_string = DeclareLaunchArgument(
        'device_serial_string',
        default_value=TextSubstitution(text=''),
        description='Optional serial string of the device to use',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=TextSubstitution(text='WARN'),
        description='Log level for ublox GNSS component containers',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('amr_sweeper_gnss'),
            'config',
            'ublox_dgnss.yaml',
        ]),
        description='Parameter file for ublox_dgnss and ublox_nav_sat_fix_hp nodes',
    )

    return launch.LaunchDescription([
        declare_use_ublox_dgnss_node,
        declare_use_ublox_nav_sat_fix_hp,
        declare_gnss_namespace,
        declare_frame_id,
        declare_device_family,
        declare_device_serial_string,
        declare_log_level,
        declare_params_file,
        OpaqueFunction(function=_create_containers),
    ])
