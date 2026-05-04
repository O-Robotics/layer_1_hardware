from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        name='namespace',
        default_value='hardware_layer',
        description='Namespace for IMU node')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    declare_port = DeclareLaunchArgument(
        name='port',
        default_value='/dev/imu_usb',
        description='Serial port for WIT IMU')
    declare_baud = DeclareLaunchArgument(
        name='baud',
        default_value='115200',
        description='Baud rate for WIT IMU')
    declare_use_imu_node = DeclareLaunchArgument(
        name='use_imu_node',
        default_value='true',
        description='Launch imu_node')

    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    use_imu_node = LaunchConfiguration('use_imu_node')

    rviz_and_imu_node = Node(
        package='amr_sweeper_imu',
        executable='imu_node',
        name='imu_node',
        namespace=ns,
        parameters=[{'port': port},
                    {'baud': baud},
                    {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_imu_node),
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_port,
            declare_baud,
            declare_use_imu_node,
            rviz_and_imu_node,
        ]
    )
