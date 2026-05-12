from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_camera = LaunchConfiguration("use_depth_camera")
    use_depth_scan = LaunchConfiguration("use_depthimage_to_laserscan")
    camera_name = LaunchConfiguration("camera_name")
    log_level = LaunchConfiguration("log_level")
    serial_no = LaunchConfiguration("serial_no")
    usb_port_id = LaunchConfiguration("usb_port_id")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")
    align_depth = LaunchConfiguration("align_depth")
    initial_reset = LaunchConfiguration("initial_reset")
    depth_scan_namespace = PathJoinSubstitution([namespace, camera_name])

    amr_depth_share = FindPackageShare("amr_sweeper_depth_camera")
    realsense_share = FindPackageShare("realsense2_camera")
    realsense_params = PathJoinSubstitution([amr_depth_share, "config", "realsense.yaml"])
    depth_scan_params = PathJoinSubstitution([amr_depth_share, "config", "depthimage_to_laserscan.yaml"])
    rs_launch = PathJoinSubstitution([realsense_share, "launch", "rs_launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_depth_camera", default_value="true"),
        DeclareLaunchArgument("use_depthimage_to_laserscan", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="depth_camera"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("usb_port_id", default_value=""),
        DeclareLaunchArgument("enable_pointcloud", default_value="false"),
        DeclareLaunchArgument("align_depth", default_value="false"),
        DeclareLaunchArgument("initial_reset", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "camera_namespace": namespace,
                "camera_name": camera_name,
                "config_file": realsense_params,
                "publish_tf": "false",
                "tf_publish_rate": "0.0",
                "serial_no": serial_no,
                "usb_port_id": usb_port_id,
                "pointcloud.enable": enable_pointcloud,
                "align_depth.enable": align_depth,
                "initial_reset": initial_reset,
                "log_level": log_level,
            }.items(),
            condition=IfCondition(use_camera),
        ),
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            namespace=depth_scan_namespace,
            name="depthimage_to_laserscan",
            output="screen",
            parameters=[depth_scan_params],
            remappings=[
                ("depth", "depth/image_rect_raw"),
                ("depth_camera_info", "depth/camera_info"),
                ("scan", "scan"),
            ],
            condition=IfCondition(use_depth_scan),
        ),
    ])
