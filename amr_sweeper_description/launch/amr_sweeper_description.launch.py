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
    default_roll_deg = LaunchConfiguration('default_roll_deg')
    default_pitch_deg = LaunchConfiguration('default_pitch_deg')

    pkg_path = get_package_share_directory('amr_sweeper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot', 'robot.urdf.xacro')
    default_ros2_control_file = os.path.join(pkg_path, 'urdf', 'control', 'ros2_control.yaml')

    robot_description = ParameterValue(Command([
        'xacro ',
        xacro_file,
        ' robot_namespace:=', namespace,
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
        # The description package owns the default controller config path used by layer 1 bringup.
        DeclareLaunchArgument('ros2_control_config_file', default_value=default_ros2_control_file),
        DeclareLaunchArgument('enable_usb_cameras', default_value='true'),
        DeclareLaunchArgument('enable_gnss', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_depth_camera', default_value='true'),
        DeclareLaunchArgument('default_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('default_pitch_deg', default_value='4.5'),
        Node(
            package='amr_sweeper_description',
            executable='default_joint_state_publisher.py',
            namespace=namespace,
            output='screen',
            parameters=[{
                'initial_roll_deg': default_roll_deg,
                'initial_pitch_deg': default_pitch_deg,
                'base_roll_joint_name': 'base_roll_joint',
                'base_pitch_joint_name': 'base_pitch_joint',
                'joint_states_topic': 'attitude_controller/joint_states',
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            remappings=[
                ('robot_description', 'description/robot_description'),
                ('/robot_description', 'description/robot_description'),
                ('joint_states', 'attitude_controller/joint_states'),
            ],
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
