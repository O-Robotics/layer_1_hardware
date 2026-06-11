"""Launch the AMR Sweeper ros2_control runtime and the joint-state broadcaster."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = _as_bool(LaunchConfiguration("use_ros2_control").perform(context))
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ros2_control_config_file,
        ],
        remappings=[
            ("/robot_description", ["/", namespace, "/robot_description"]),
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        namespace=namespace,
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
    )

    if not use_ros2_control:
        return []

    return [
        controller_manager,
        gated_joint_broad_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_sweeper_description"),
                "urdf",
                "control",
                "ros2_control.yaml",
            ]),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
