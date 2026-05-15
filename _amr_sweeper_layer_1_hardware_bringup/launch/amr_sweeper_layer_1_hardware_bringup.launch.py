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
    gnss_namespace = PathJoinSubstitution([namespace, "gnss"])
    usb_cameras_namespace = PathJoinSubstitution([namespace, "usb_cameras"])
    depth_camera_namespace = PathJoinSubstitution([namespace, "depth_camera"])
    imu_namespace = PathJoinSubstitution([namespace, "imu"])
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_robot_description = LaunchConfiguration("use_robot_description")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_battery_node = LaunchConfiguration("use_battery_node")
    use_system_info_node = LaunchConfiguration("use_system_info_node")
    use_usb_cameras = LaunchConfiguration("use_usb_cameras")
    use_depth_camera = LaunchConfiguration("use_depth_camera")
    use_imu_node = LaunchConfiguration("use_imu_node")
    use_gnss_rover = LaunchConfiguration("use_gnss_rover")
    use_ntrip_client = LaunchConfiguration("use_ntrip_client")
    use_steadydrive_can_nodes = LaunchConfiguration("use_steadydrive_can_nodes")
    use_odrive_node = LaunchConfiguration("use_odrive_node")

    battery_can_interface = LaunchConfiguration("battery_can_interface")
    steadydrive_can_interface = LaunchConfiguration("steadydrive_can_interface")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    odrive_interface = LaunchConfiguration("odrive_interface")
    odrive_node_id = LaunchConfiguration("odrive_node_id")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="amr_sweeper"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(DeclareLaunchArgument("use_robot_description", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ros2_control", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_battery_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_system_info_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_usb_cameras", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_depth_camera", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_imu_node", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_gnss_rover", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ntrip_client", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_steadydrive_can_nodes", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_odrive_node", default_value="false"))

    ld.add_action(DeclareLaunchArgument("battery_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("steadydrive_can_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb"))
    ld.add_action(DeclareLaunchArgument("imu_baud", default_value="9600"))
    ld.add_action(DeclareLaunchArgument("odrive_interface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("odrive_node_id", default_value="0"))
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
                    "enable_usb_cameras": use_usb_cameras,
                    "enable_gnss": use_gnss_rover,
                    "enable_imu": use_imu_node,
                    "enable_depth_camera": use_depth_camera,
                }.items(),
                condition=IfCondition(use_robot_description),
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
        condition=IfCondition(use_battery_node),
    ))

    ld.add_action(Node(
        package="amr_sweeper_system_info",
        executable="amr_sweeper_system_info_node",
        namespace=namespace,
        name="amr_sweeper_system_info_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_system_info_node),
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
                condition=IfCondition(use_usb_cameras),
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
                condition=IfCondition(use_depth_camera),
            ),
        ],
    ))

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_imu", "imu.launch.py")),
                launch_arguments={
                    "namespace": imu_namespace,
                    "use_sim_time": use_sim_time,
                    "port": imu_port,
                    "baud": imu_baud,
                    "use_imu_node": use_imu_node,
                }.items(),
                condition=IfCondition(use_imu_node),
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
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_steadydrive", "steadydrive.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "can_interface": steadydrive_can_interface,
                }.items(),
                condition=IfCondition(use_steadydrive_can_nodes),
            ),
        ],
    ))

    ld.add_action(GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file("amr_sweeper_gnss", "amr_sweeper_gnss.launch.py")),
                launch_arguments={
                    "use_ublox_dgnss_node": use_gnss_rover,
                    "use_ublox_nav_sat_fix_hp": use_gnss_rover,
                    "use_ntrip_client": use_ntrip_client,
                    "gnss_namespace": gnss_namespace,
                    "gnss_frame_id": gnss_frame_id,
                    "ntrip_params_file": ntrip_params_file,
                }.items(),
                condition=IfCondition(use_gnss_rover),
            ),
        ],
    ))

    ld.add_action(Node(
        package="amr_sweeper_odrive",
        executable="odrive_node",
        namespace=namespace,
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
