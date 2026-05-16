from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        name='namespace',
        default_value='amr_sweeper',
        description='Namespace for IMU node')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use ROS time if true')
    declare_port = DeclareLaunchArgument(
        name='port',
        default_value='/dev/imu_usb',
        description='Serial port for the JY901 IMU')
    declare_baud = DeclareLaunchArgument(
        name='baud',
        default_value='9600',
        description='Baud rate for the JY901 IMU')
    declare_frame_id = DeclareLaunchArgument(
        name='imu_frame_id',
        default_value='imu_link',
        description='Frame ID for published IMU messages')
    declare_publish_hz = DeclareLaunchArgument(
        name='publish_hz',
        default_value='10.0',
        description='Maximum IMU publish rate in Hz')
    declare_params_file = DeclareLaunchArgument(
        name='params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('amr_sweeper_imu'),
            'config',
            'amr_sweeper_imu.yaml',
        ]),
        description='Parameter file for the IMU node',
    )
    declare_use_imu_node = DeclareLaunchArgument(
        name='use_imu_node',
        default_value='true',
        description='Launch imu_node')

    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    publish_hz = LaunchConfiguration('publish_hz')
    use_imu_node = LaunchConfiguration('use_imu_node')

    imu_node = Node(
        package='amr_sweeper_imu',
        executable='imu_node',
        name='imu_node',
        namespace=ns,
        parameters=[
            params_file,
            {'port': port},
            {'baud': baud},
            {'imu_frame_id': imu_frame_id},
            {'publish_hz': publish_hz},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(use_imu_node),
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_port,
            declare_baud,
            declare_frame_id,
            declare_publish_hz,
            declare_params_file,
            declare_use_imu_node,
            imu_node,
        ]
    )
