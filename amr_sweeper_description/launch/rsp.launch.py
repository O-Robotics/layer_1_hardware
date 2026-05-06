import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    enable_usb_cameras = LaunchConfiguration('enable_usb_cameras')
    enable_gnss = LaunchConfiguration('enable_gnss')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')

    pkg_path = get_package_share_directory('amr_sweeper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot', 'robot.urdf.xacro')

    robot_description = ParameterValue(Command([
        'xacro ',
        xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' enable_usb_cameras:=', enable_usb_cameras,
        ' enable_gnss:=', enable_gnss,
        ' enable_imu:=', enable_imu,
        ' enable_depth_camera:=', enable_depth_camera,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('enable_usb_cameras', default_value='true'),
        DeclareLaunchArgument('enable_gnss', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_depth_camera', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
