from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_realsense_ros = LaunchConfiguration("use_realsense_ros")
    realsense_namespace = LaunchConfiguration("realsense_namespace")
    realsense_node_name = LaunchConfiguration("realsense_node_name")
    realsense_params_file = LaunchConfiguration("realsense_params_file")
    use_depthimage_to_laserscan = LaunchConfiguration("use_depthimage_to_laserscan")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    output_frame = LaunchConfiguration("output_frame")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    scan_height = LaunchConfiguration("scan_height")
    scan_time = LaunchConfiguration("scan_time")

    params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "depthimage_to_laserscan.yaml",
    ])
    default_realsense_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_depth_camera"),
        "config",
        "realsense-ros.yaml",
    ])
    default_depth_image_topic = PathJoinSubstitution([
        realsense_namespace,
        realsense_node_name,
        "depth",
        "image_rect_raw",
    ])
    default_depth_camera_info_topic = PathJoinSubstitution([
        realsense_namespace,
        realsense_node_name,
        "depth",
        "camera_info",
    ])

    realsense_ros_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=realsense_node_name,
        namespace=realsense_namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            realsense_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "camera_name": ParameterValue(realsense_node_name, value_type=str),
                "camera_namespace": ParameterValue(realsense_namespace, value_type=str),
            },
        ],
        condition=IfCondition(use_realsense_ros),
    )

    depthimage_to_laserscan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "output_frame": ParameterValue(output_frame, value_type=str),
                "range_min": ParameterValue(range_min, value_type=float),
                "range_max": ParameterValue(range_max, value_type=float),
                "scan_height": ParameterValue(scan_height, value_type=int),
                "scan_time": ParameterValue(scan_time, value_type=float),
            },
        ],
        remappings=[
            ("depth", depth_image_topic),
            ("depth_camera_info", depth_camera_info_topic),
            ("scan", scan_topic),
        ],
        condition=IfCondition(use_depthimage_to_laserscan),
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper/depth_camera"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_realsense_ros", default_value="true"),
        DeclareLaunchArgument("realsense_namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("realsense_node_name", default_value="depth_camera"),
        DeclareLaunchArgument("realsense_params_file", default_value=default_realsense_params_file),
        DeclareLaunchArgument("use_depthimage_to_laserscan", default_value="true"),
        DeclareLaunchArgument("depth_image_topic", default_value=default_depth_image_topic),
        DeclareLaunchArgument("depth_camera_info_topic", default_value=default_depth_camera_info_topic),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("output_frame", default_value="depth_camera_depth_optical_frame"),
        DeclareLaunchArgument("range_min", default_value="0.25"),
        DeclareLaunchArgument("range_max", default_value="8.0"),
        DeclareLaunchArgument("scan_height", default_value="20"),
        DeclareLaunchArgument("scan_time", default_value="0.0333"),
        realsense_ros_node,
        depthimage_to_laserscan_node,
    ])
