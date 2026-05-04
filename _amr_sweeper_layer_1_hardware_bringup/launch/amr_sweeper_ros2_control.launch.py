import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    enable_top_cameras = LaunchConfiguration("enable_top_cameras")
    enable_gnss = LaunchConfiguration("enable_gnss")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_depth_camera = LaunchConfiguration("enable_depth_camera")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")

    description_pkg_path = get_package_share_directory("amr_sweeper_description")
    xacro_file = os.path.join(description_pkg_path, "urdf", "robot", "robot.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro",
                " ",
                xacro_file,
                " ",
                "use_ros2_control:=", use_ros2_control,
                " ",
                "sim_mode:=", use_sim_time,
                " ",
                "enable_top_cameras:=", enable_top_cameras,
                " ",
                "enable_gnss:=", enable_gnss,
                " ",
                "enable_imu:=", enable_imu,
                " ",
                "enable_depth_camera:=", enable_depth_camera,
            ]),
            value_type=str,
        )
    }

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(use_sim_time),
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            ros2_control_config_file,
        ],
        namespace=namespace,
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--param-file", ros2_control_config_file],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    steadydrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_steadydrive"],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("enable_top_cameras", default_value="true"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_depth_camera", default_value="true"),
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value=os.path.join(
                get_package_share_directory("amr_sweeper_layer_1_hardware_bringup"),
                "config",
                "ros2_control.yaml",
            ),
        ),
        delayed_controller_manager,
        joint_broad_spawner,
        diff_drive_spawner,
        steadydrive_spawner,
    ])
