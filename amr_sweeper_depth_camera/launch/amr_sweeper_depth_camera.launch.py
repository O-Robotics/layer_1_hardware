# Copyright (c) 2026 O-Robotics

import math
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _normalize_namespace(namespace: str) -> str:
    cleaned = namespace.strip().strip('/')
    return f'/{cleaned}' if cleaned else '/'


def _split_namespace(namespace: str) -> tuple[str, str]:
    normalized = _normalize_namespace(namespace)
    parts = [part for part in normalized.split('/') if part]
    if not parts:
        raise RuntimeError("Depth camera namespace must not resolve to '/'.")
    if len(parts) == 1:
        return '/', parts[0]
    return '/' + '/'.join(parts[:-1]), parts[-1]


def _load_ros_parameter_file(path: str) -> dict:
    data = yaml.safe_load(Path(path).read_text()) or {}
    if '/**' in data:
        return data['/**'].get('ros__parameters', {})
    if len(data) == 1:
        only_node = next(iter(data.values()))
        if isinstance(only_node, dict) and 'ros__parameters' in only_node:
            return only_node['ros__parameters']
    return data


def _verify_realsense_package_available() -> None:
    try:
        get_package_prefix('realsense2_camera')
    except PackageNotFoundError as exc:
        depth_camera_share = Path(get_package_share_directory('amr_sweeper_depth_camera'))
        workspace_install = depth_camera_share.parents[1]
        workspace_root = workspace_install.parent
        raise RuntimeError(
            'Required package "realsense2_camera" is not visible in the current ROS environment. '
            f'This launch file is running from workspace "{workspace_root}", so first source '
            f'"{workspace_install / "setup.bash"}". If the package still is not available, build '
            'it in the same workspace with: '
            '"colcon build --packages-select realsense2_camera_msgs '
            'realsense2_camera amr_sweeper_depth_camera". '
            'If you have multiple workspace copies, make sure you are launching from the same one '
            'you built.'
        ) from exc


def _launch_setup(context, *args, **kwargs):
    _verify_realsense_package_available()

    namespace_value = _normalize_namespace(LaunchConfiguration('namespace').perform(context))
    camera_parent_namespace_value, camera_name_value = _split_namespace(namespace_value)
    depth_camera_params = _load_ros_parameter_file(
        LaunchConfiguration('params_file').perform(context)
    )
    laserscan_params = _load_ros_parameter_file(
        LaunchConfiguration('laserscan_params_file').perform(context)
    )

    output_frame_value = LaunchConfiguration('output_frame').perform(context)
    if output_frame_value:
        laserscan_params['output_frame'] = output_frame_value

    range_min_value = LaunchConfiguration('range_min').perform(context)
    if range_min_value:
        laserscan_params['range_min'] = float(range_min_value)

    range_max_value = LaunchConfiguration('range_max').perform(context)
    if range_max_value:
        laserscan_params['range_max'] = float(range_max_value)

    scan_height_value = LaunchConfiguration('scan_height').perform(context)
    if scan_height_value:
        laserscan_params['scan_height'] = int(scan_height_value)

    scan_tilt_angle_deg_value = LaunchConfiguration('scan_tilt_angle_deg').perform(context)
    if scan_tilt_angle_deg_value:
        laserscan_params['scan_tilt_angle_deg'] = float(scan_tilt_angle_deg_value)

    scan_time_value = LaunchConfiguration('scan_time').perform(context)
    if scan_time_value:
        laserscan_params['scan_time'] = float(scan_time_value)

    depth_camera_frame_value = LaunchConfiguration('depth_camera_frame').perform(context)
    laserscan_frame_value = str(laserscan_params.get('output_frame', depth_camera_frame_value))
    scan_tilt_angle_rad = math.radians(float(laserscan_params.get('scan_tilt_angle_deg', 0.0)))

    default_depth_image_topic = f'{namespace_value}/depth/image_rect_raw'
    default_depth_camera_info_topic = f'{namespace_value}/depth/camera_info'
    depth_image_topic_value = LaunchConfiguration('depth_image_topic').perform(context)
    if not depth_image_topic_value:
        depth_image_topic_value = default_depth_image_topic

    depth_camera_info_topic_value = LaunchConfiguration('depth_camera_info_topic').perform(context)
    if not depth_camera_info_topic_value:
        depth_camera_info_topic_value = default_depth_camera_info_topic

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=camera_parent_namespace_value,
        name=camera_name_value,
        output='log',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('realsense_log_level')],
        parameters=[
            depth_camera_params,
            {
                'camera_name': camera_name_value,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            },
        ],
    )

    laserscan_node = Node(
        package='amr_sweeper_depth_camera',
        executable='laserscan_node',
        name='laserscan',
        namespace=namespace_value,
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            laserscan_params,
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            },
        ],
        remappings=[
            ('depth', depth_image_topic_value),
            ('depth_camera_info', depth_camera_info_topic_value),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('use_laserscan')),
    )

    laserscan_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laserscan_tf',
        namespace=namespace_value,
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', str(scan_tilt_angle_rad),
            '--yaw', '0',
            '--frame-id', depth_camera_frame_value,
            '--child-frame-id', laserscan_frame_value,
        ],
        condition=IfCondition(LaunchConfiguration('use_laserscan')),
    )

    return [
        realsense_node,
        laserscan_tf_node,
        laserscan_node,
    ]


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('amr_sweeper_depth_camera'),
        'config',
        'amr_sweeper_depth_camera.yaml',
    ])
    default_laserscan_params_file = PathJoinSubstitution([
        FindPackageShare('amr_sweeper_depth_camera'),
        'config',
        'laserscan.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper/depth_camera'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('realsense_log_level', default_value='error'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_domain_id', default_value='5'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('use_laserscan', default_value='true'),
        DeclareLaunchArgument(
            'laserscan_params_file',
            default_value=default_laserscan_params_file,
        ),
        DeclareLaunchArgument('depth_image_topic', default_value=''),
        DeclareLaunchArgument('depth_camera_info_topic', default_value=''),
        DeclareLaunchArgument('depth_camera_frame', default_value='depth_camera_link'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('output_frame', default_value=''),
        DeclareLaunchArgument('range_min', default_value=''),
        DeclareLaunchArgument('range_max', default_value=''),
        DeclareLaunchArgument('scan_height', default_value=''),
        DeclareLaunchArgument('scan_tilt_angle_deg', default_value=''),
        DeclareLaunchArgument('scan_time', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
