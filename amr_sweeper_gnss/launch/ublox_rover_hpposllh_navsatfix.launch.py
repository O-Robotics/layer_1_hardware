"""Launch amr_sweeper_gnss components for Ublox + NavSat fix conversion."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for amr_sweeper_ublox_dgnss components."""
    declare_use_ublox_dgnss_node = DeclareLaunchArgument(
        'use_ublox_dgnss_node',
        default_value=TextSubstitution(text='true'),
        description='Launch UbloxDGNSS component container',
    )

    declare_use_ublox_nav_sat_fix_hp = DeclareLaunchArgument(
        'use_ublox_nav_sat_fix_hp',
        default_value=TextSubstitution(text='true'),
        description='Launch ublox_nav_sat_fix_hp component container',
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='amr_sweeper'),
        description='Namespace for GNSS containers',
    )

    params = [
        {'FRAME_ID': 'GNSS_link'},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_RATE_MEAS': 200},
        {'CFG_RATE_NAV': 1},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 0},
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0},
    ]

    container1 = ComposableNodeContainer(
        name='ublox_dgnss_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='amr_sweeper_gnss',
                plugin='amr_sweeper_ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace=LaunchConfiguration('namespace'),
                parameters=params,
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_ublox_dgnss_node')),
    )

    container2 = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='amr_sweeper_gnss',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                namespace=LaunchConfiguration('namespace'),
                remappings=[('fix', 'navsat')],
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_ublox_nav_sat_fix_hp')),
    )

    return launch.LaunchDescription([
        declare_use_ublox_dgnss_node,
        declare_use_ublox_nav_sat_fix_hp,
        declare_namespace,
        container1,
        container2,
    ])
