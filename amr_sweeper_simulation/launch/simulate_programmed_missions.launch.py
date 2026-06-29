from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])



def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_profile = LaunchConfiguration("use_profile")
    schedule_ics_path = LaunchConfiguration("schedule_ics_path")
    simulation_speed = LaunchConfiguration("simulation_speed")
    simulation_world_name = LaunchConfiguration("simulation_world_name")
    missions_from_db_directory = LaunchConfiguration("missions_from_db_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    robot_id = LaunchConfiguration("robot_id")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("amr_sweeper_bringup", "amr_sweeper_bringup.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_simulation": "true",
            "use_sim_time": "false",
            "use_profile": use_profile,
            "missions_from_db_directory": missions_from_db_directory,
            "missions_log_directory": missions_log_directory,
            "schedule_ics_path": schedule_ics_path,
            "robot_id": robot_id,
            "trigger_running_on_work_window": "true",
            "auto_build_on_start": "true",
            "watch_for_updates": "true",
            "launch_scheduler": "false",
            "launch_vda5050_parser": "false",
            "record_rosbag": "false",
        }.items(),
    )

    set_sim_speed = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "amr_sweeper_simulation",
                    "set_gazebo_simulation_speed.py",
                    "--speed",
                    simulation_speed,
                    "--world",
                    simulation_world_name,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_profile", default_value="050"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("simulation_speed", default_value="10.0"),
        DeclareLaunchArgument("simulation_world_name", default_value="amr_sweeper_test"),
        DeclareLaunchArgument("missions_from_db_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/simulations"),
        DeclareLaunchArgument("robot_id", default_value="AMR-Sweeper_00006"),
        bringup,
        set_sim_speed,
    ])
