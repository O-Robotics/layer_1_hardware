"""Launch all AMR Sweeper layer 1 hardware packages from one bringup entrypoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    usb_cameras_namespace = PathJoinSubstitution([robot_namespace, "usb_cameras"])
    depth_camera_namespace = PathJoinSubstitution([robot_namespace, "depth_camera"])
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_robot_description = LaunchConfiguration("use_robot_description")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_battery_node = LaunchConfiguration("use_battery_node")
    use_system_info_node = LaunchConfiguration("use_system_info_node")
    use_usb_cameras = LaunchConfiguration("use_usb_cameras")
    use_depth_camera = LaunchConfiguration("use_depth_camera")
    use_microros = LaunchConfiguration("use_microros")
    use_imu_node = LaunchConfiguration("use_imu_node")
    use_gnss_rover = LaunchConfiguration("use_gnss_rover")
    use_ntrip_client = LaunchConfiguration("use_ntrip_client")
    use_steadydrive_can_nodes = LaunchConfiguration("use_steadydrive_can_nodes")
    use_odrive_node = LaunchConfiguration("use_odrive_node")

    battery_can_interface = LaunchConfiguration("battery_can_interface")
    microros_can_interface = LaunchConfiguration("microros_can_interface")
    microros_request_id_min = LaunchConfiguration("microros_request_id_min")
    microros_request_id_max = LaunchConfiguration("microros_request_id_max")
    microros_reply_id_offset = LaunchConfiguration("microros_reply_id_offset")
    microros_same_id_reply = LaunchConfiguration("microros_same_id_reply")
    microros_verbosity = LaunchConfiguration("microros_verbosity")
    steadydrive_can_interface = LaunchConfiguration("steadydrive_can_interface")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    odrive_interface = LaunchConfiguration("odrive_interface")
    odrive_node_id = LaunchConfiguration("odrive_node_id")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("robot_namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(DeclareLaunchArgument("use_robot_description", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ros2_control", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_battery_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_system_info_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_usb_cameras", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_depth_camera", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_microros", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_imu_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_gnss_rover", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ntrip_client", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_steadydrive_can_nodes", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_odrive_node", default_value="false"))

    ld.add_action(DeclareLaunchArgument("battery_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("microros_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("microros_request_id_min", default_value="0x500"))
    ld.add_action(DeclareLaunchArgument("microros_request_id_max", default_value="0x57F"))
    ld.add_action(DeclareLaunchArgument("microros_reply_id_offset", default_value="0x80"))
    ld.add_action(DeclareLaunchArgument("microros_same_id_reply", default_value="false"))
    ld.add_action(DeclareLaunchArgument("microros_verbosity", default_value="4"))
    ld.add_action(DeclareLaunchArgument("steadydrive_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb"))
    ld.add_action(DeclareLaunchArgument("imu_baud", default_value="9600"))
    ld.add_action(DeclareLaunchArgument("odrive_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("odrive_node_id", default_value="0"))

    # Start micro-ROS first so downstream CAN-connected hardware nodes can depend on it.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_microros", "microros_agent.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "use_microros": use_microros,
            "can_interface": microros_can_interface,
            "request_id_min": microros_request_id_min,
            "request_id_max": microros_request_id_max,
            "reply_id_offset": microros_reply_id_offset,
            "same_id_reply": microros_same_id_reply,
            "verbosity": microros_verbosity,
        }.items(),
        condition=IfCondition(use_microros),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_description", "rsp.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            "enable_usb_cameras": use_usb_cameras,
            "enable_gnss": use_gnss_rover,
            "enable_imu": use_imu_node,
            "enable_depth_camera": use_depth_camera,
        }.items(),
        condition=IfCondition(use_robot_description),
    ))

    ld.add_action(Node(
        package="amr_sweeper_battery",
        executable="amr_sweeper_battery_node",
        namespace=robot_namespace,
        name="amr_sweeper_battery_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "can_interface": battery_can_interface,
        }],
        condition=IfCondition(use_battery_node),
    ))

    ld.add_action(Node(
        package="amr_sweeper_system_info",
        executable="amr_sweeper_system_info_node",
        namespace=robot_namespace,
        name="amr_sweeper_system_info_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_system_info_node),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_usb_cameras", "amr_sweeper_usb_cameras.launch.py")),
        launch_arguments={
            "namespace": usb_cameras_namespace,
            "log_level": log_level,
        }.items(),
        condition=IfCondition(use_usb_cameras),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_depth_camera", "amr_sweeper_depth_camera.launch.py")),
        launch_arguments={
            "namespace": depth_camera_namespace,
            "log_level": log_level,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_depth_camera),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_imu", "imu.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "port": imu_port,
            "baud": imu_baud,
            "use_imu_node": use_imu_node,
        }.items(),
        condition=IfCondition(use_imu_node),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_layer_1_hardware_bringup", "amr_sweeper_ros2_control.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            "enable_usb_cameras": use_usb_cameras,
            "enable_gnss": use_gnss_rover,
            "enable_imu": use_imu_node,
        }.items(),
        condition=IfCondition(use_ros2_control),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_steadydrive", "steadydrive.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "can_interface": steadydrive_can_interface,
        }.items(),
        condition=IfCondition(use_steadydrive_can_nodes),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_gnss", "ublox_rover_hpposllh_navsatfix.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
        }.items(),
        condition=IfCondition(use_gnss_rover),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_gnss", "ntrip_client.launch.py")),
        launch_arguments={
            "namespace": robot_namespace,
            "use_ntrip_client_node": use_ntrip_client,
        }.items(),
        condition=IfCondition(use_ntrip_client),
    ))

    ld.add_action(Node(
        package="amr_sweeper_odrive",
        executable="odrive_node",
        namespace=robot_namespace,
        name="odrive_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "interface": odrive_interface,
            "node_id": odrive_node_id,
        }],
        condition=IfCondition(use_odrive_node),
    ))

    return ld
