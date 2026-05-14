import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(use_sim_time),
        parameters=[
            {"use_sim_time": use_sim_time},
            ros2_control_config_file,
        ],
        # Keep the controller manager on the robot_description topic path that
        # works with Humble's deprecated subscriber and Jazzy's required one.
        remappings=[
            ("~/robot_description", "robot_description"),
            ("/robot_description", "robot_description"),
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
        arguments=[
            "joint_broad",
            "--controller-manager",
            ["/", namespace, "/controller_manager"],
            "--controller-manager-timeout",
            "60",
        ],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-manager",
            ["/", namespace, "/controller_manager"],
            "--controller-manager-timeout",
            "60",
            "--param-file",
            ros2_control_config_file,
        ],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    steadydrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "controller_steadydrive",
            "--controller-manager",
            ["/", namespace, "/controller_manager"],
            "--controller-manager-timeout",
            "60",
            "--param-file",
            ros2_control_config_file,
        ],
        namespace=namespace,
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_broad_spawner],
                ),
            ],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[diff_drive_spawner],
                ),
            ],
        )
    )

    delayed_steadydrive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=6.0,
                    actions=[steadydrive_spawner],
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("enable_usb_cameras", default_value="true"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value=os.path.join(
                get_package_share_directory("amr_sweeper_description"),
                "urdf",
                "control",
                "ros2_control.yaml",
            ),
        ),
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_steadydrive_spawner,
    ])
