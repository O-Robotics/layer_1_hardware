"""Launch the AMR Sweeper ros2_control runtime and the joint-state broadcaster."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_joint_broadcaster = LaunchConfiguration("use_joint_broadcaster")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ros2_control_config_file,
        ],
        remappings=[
            ("/robot_description", ["/", namespace, "/robot_description"]),
        ],
        condition=IfCondition(use_ros2_control),
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
        condition=IfCondition(use_joint_broadcaster),
        output="screen",
        additional_env={"RCUTILS_COLORIZED_OUTPUT": "0"},
    )

    gated_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                joint_broad_spawner,
            ],
        ),
        condition=IfCondition(PythonExpression([
            "'",
            use_ros2_control,
            "' == 'true' and '",
            use_joint_broadcaster,
            "' == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("use_joint_broadcaster", default_value="true"),
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_description"),
                "urdf",
                "control",
                "ros2_control.yaml",
            ]),
        ),
        controller_manager,
        gated_joint_broad_spawner,
    ])
