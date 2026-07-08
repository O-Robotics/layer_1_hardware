from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _simulation_input_topic(namespace: str, sensor_name: str, topic_suffix: str) -> str:
    parts = [part for part in namespace.strip().strip('/').split('/') if part]
    if parts and parts[-1] == sensor_name:
        parts = parts[:-1]
    root_namespace = '/'.join(parts)
    prefix = f"{root_namespace}/simulation" if root_namespace else "simulation"
    return f"/{prefix}/{topic_suffix.strip().lstrip('/')}"


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulation = LaunchConfiguration('use_simulation').perform(context).strip().lower() in {'1', 'true', 'yes', 'on'}
    params_file = LaunchConfiguration('params_file')
    device_path = LaunchConfiguration('device_path').perform(context)
    port = LaunchConfiguration('port').perform(context)
    baud = LaunchConfiguration('baud').perform(context)
    imu_frame_id = LaunchConfiguration('imu_frame_id').perform(context)
    publish_hz = LaunchConfiguration('publish_hz').perform(context)
    use_imu_node = LaunchConfiguration('use_imu_node')

    if use_simulation:
        simulation_input_topic = _simulation_input_topic(
            ns.perform(context),
            'imu',
            'imu/data_raw',
        )
        return [
            Node(
                package='amr_sweeper_imu',
                executable='gz_imu_adapter',
                name='imu_node',
                namespace=ns,
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': simulation_input_topic,
                        'override_stamp_with_ros_time': False,
                    }
                ],
                output='screen',
                condition=IfCondition(use_imu_node),
            )
        ]

    parameters = [
        params_file,
        {'use_sim_time': use_sim_time},
    ]

    if device_path:
        parameters.append({'device_path': device_path})
    if port:
        parameters.append({'port': port})
    if baud:
        parameters.append({'baud': int(baud)})
    if imu_frame_id:
        parameters.append({'imu_frame_id': imu_frame_id})
    if publish_hz:
        parameters.append({'publish_hz': float(publish_hz)})

    imu_node = Node(
        package='amr_sweeper_imu',
        executable='imu_node',
        name='imu_node',
        namespace=ns,
        parameters=parameters,
        output='screen',
        condition=IfCondition(use_imu_node),
    )

    return [imu_node]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        name='namespace',
        default_value='amr_sweeper/imu',
        description='Namespace for IMU node')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use ROS time if true')
    declare_use_simulation = DeclareLaunchArgument(
        name='use_simulation',
        default_value='false',
        description='Disable the hardware IMU node when running in simulation')
    declare_device_path = DeclareLaunchArgument(
        name='device_path',
        default_value='',
        description='Optional serial device path override for the JY901 IMU')
    declare_port = DeclareLaunchArgument(
        name='port',
        default_value='',
        description='Optional deprecated compatibility alias override for device_path')
    declare_baud = DeclareLaunchArgument(
        name='baud',
        default_value='',
        description='Optional baud override for the JY901 IMU')
    declare_frame_id = DeclareLaunchArgument(
        name='imu_frame_id',
        default_value='',
        description='Optional frame ID override for published IMU messages')
    declare_publish_hz = DeclareLaunchArgument(
        name='publish_hz',
        default_value='',
        description='Optional IMU publish-rate override in Hz')
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

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_use_simulation,
            declare_device_path,
            declare_port,
            declare_baud,
            declare_frame_id,
            declare_publish_hz,
            declare_params_file,
            declare_use_imu_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
