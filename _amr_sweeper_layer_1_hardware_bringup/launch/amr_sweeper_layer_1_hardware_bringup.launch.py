"""Launch all AMR Sweeper layer 1 hardware packages from one bringup entrypoint."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
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
    namespace = LaunchConfiguration("namespace")
    gnss_namespace = PathJoinSubstitution(["/", namespace, "gnss"])
    usb_cameras_namespace = PathJoinSubstitution(["/", namespace, "usb_cameras"])
    depth_camera_namespace = PathJoinSubstitution(["/", namespace, "depth_camera"])
    imu_namespace = PathJoinSubstitution(["/", namespace, "imu"])
    log_level = LaunchConfiguration("log_level")
    ublox_log_level = LaunchConfiguration("ublox_log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_amr_sweeper_description = LaunchConfiguration("use_amr_sweeper_description")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_amr_sweeper_battery = LaunchConfiguration("use_amr_sweeper_battery")
    use_amr_sweeper_system_info = LaunchConfiguration("use_amr_sweeper_system_info")
    use_amr_sweeper_usb_cameras = LaunchConfiguration("use_amr_sweeper_usb_cameras")
    use_amr_sweeper_depth_camera = LaunchConfiguration("use_amr_sweeper_depth_camera")
    use_amr_sweeper_imu = LaunchConfiguration("use_amr_sweeper_imu")
    use_amr_sweeper_gnss = LaunchConfiguration("use_amr_sweeper_gnss")
    use_ntrip_client = LaunchConfiguration("use_ntrip_client")
    battery_can_interface = LaunchConfiguration("battery_can_interface")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    imu_params_file = LaunchConfiguration("imu_params_file")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("ublox_log_level", default_value="WARN"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_description", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ros2_control", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_battery", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_system_info", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_usb_cameras", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_depth_camera", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_imu", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_amr_sweeper_gnss", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ntrip_client", default_value="true"))
    ld.add_action(DeclareLaunchArgument("battery_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb"))
    ld.add_action(DeclareLaunchArgument("imu_baud", default_value="9600"))
    ld.add_action(DeclareLaunchArgument("imu_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_imu"),
        "config",
        "amr_sweeper_imu.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("gnss_frame_id", default_value="gnss_link"))
    ld.add_action(DeclareLaunchArgument("ntrip_params_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_gnss"),
        "config",
        "ntrip_client.yaml",
    ])))
    ld.add_action(DeclareLaunchArgument("ros2_control_config_file", default_value=PathJoinSubstitution([
        FindPackageShare("amr_sweeper_description"),
        "urdf",
        "control",
        "ros2_control.yaml",
    ])))
    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("amr_sweeper_description", "amr_sweeper_description.launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "use_ros2_control": use_ros2_control,
                    "ros2_control_config_file": ros2_control_config_file,
                    "enable_usb_cameras": use_amr_sweeper_usb_cameras,
                    "enable_gnss": use_amr_sweeper_gnss,
                    "enable_imu": use_amr_sweeper_imu,
                    "enable_depth_camera": use_amr_sweeper_depth_camera,
                }.items(),
                condition=IfCondition(use_amr_sweeper_description),
            ),
        ],
    ))

    ld.add_action(Node(
        package="amr_sweeper_battery",
        executable="amr_sweeper_battery_node",
        namespace=namespace,
        name="amr_sweeper_battery_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "can_interface": battery_can_interface,
        }],
        condition=IfCondition(use_amr_sweeper_battery),
    ))

    ld.add_action(Node(
        package="amr_sweeper_system_info",
        executable="amr_sweeper_system_info_node",
        namespace=namespace,
        name="amr_sweeper_system_info_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_amr_sweeper_system_info),
    ))

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_usb_cameras", "amr_sweeper_usb_cameras.launch.py")),
                launch_arguments={
                    "namespace": usb_cameras_namespace,
                    "log_level": log_level,
                }.items(),
                condition=IfCondition(use_amr_sweeper_usb_cameras),
            ),
        ],
    ))

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_depth_camera", "amr_sweeper_depth_camera.launch.py")),
                launch_arguments={
                    "namespace": depth_camera_namespace,
                    "log_level": log_level,
                    "use_sim_time": use_sim_time,
                }.items(),
                condition=IfCondition(use_amr_sweeper_depth_camera),
            ),
        ],
    ))

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_imu", "amr_sweeper_imu.launch.py")),
                launch_arguments={
                    "namespace": imu_namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": imu_params_file,
                    "port": imu_port,
                    "baud": imu_baud,
                    "use_imu_node": use_amr_sweeper_imu,
                }.items(),
                condition=IfCondition(use_amr_sweeper_imu),
            ),
        ],
    ))

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
            ("~/robot_description", "robot_description"),
            ("/robot_description", "robot_description"),
        ],
        condition=IfCondition(use_ros2_control),
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager],
        condition=UnlessCondition(use_sim_time),
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
            "--controller-ros-args",
            "--remap /tf:=diff_cont_disabled_tf",
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
        ),
        condition=IfCondition(use_ros2_control),
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
        ),
        condition=IfCondition(use_ros2_control),
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
        ),
        condition=IfCondition(use_ros2_control),
    )

    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_steadydrive_spawner)

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_gnss", "amr_sweeper_gnss.launch.py")),
                launch_arguments={
                    "use_ublox_dgnss_node": use_amr_sweeper_gnss,
                    "use_ublox_nav_sat_fix_hp": use_amr_sweeper_gnss,
                    "use_ntrip_client": use_ntrip_client,
                    "gnss_namespace": gnss_namespace,
                    "gnss_frame_id": gnss_frame_id,
                    "ntrip_params_file": ntrip_params_file,
                    "ublox_log_level": ublox_log_level,
                    "log_level": log_level,
                }.items(),
                condition=IfCondition(use_amr_sweeper_gnss),
            ),
        ],
    ))

    return ld
