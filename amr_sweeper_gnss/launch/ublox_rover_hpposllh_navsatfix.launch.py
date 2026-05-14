"""Launch amr_sweeper_gnss components for a standalone ZED-F9P rover."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


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
    declare_frame_id = DeclareLaunchArgument(
        'gnss_frame_id',
        default_value=TextSubstitution(text='gnss_link'),
        description='Frame ID to publish in GNSS message headers',
    )

    params = [
        {'FRAME_ID': LaunchConfiguration('gnss_frame_id')},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_USBINPROT_RTCM3X': True},
        {'CFG_USBOUTPROT_RTCM3X': False},
        {'CFG_RATE_MEAS': 200},
        {'CFG_RATE_NAV': 1},
        {'CFG_NAVSPG_FIXMODE': 2},
        {'CFG_NAVSPG_INIFIX3D': True},
        {'CFG_NAVSPG_DYNMODEL': 4},
        {'CFG_SIGNAL_GPS_ENA': True},
        {'CFG_SIGNAL_GPS_L1CA_ENA': True},
        {'CFG_SIGNAL_GPS_L2C_ENA': True},
        {'CFG_SIGNAL_GAL_ENA': True},
        {'CFG_SIGNAL_GAL_E1_ENA': True},
        {'CFG_SIGNAL_GAL_E5B_ENA': True},
        {'CFG_SIGNAL_QZSS_ENA': True},
        {'CFG_SIGNAL_QZSS_L1CA_ENA': True},
        {'CFG_SIGNAL_QZSS_L1S_ENA': True},
        {'CFG_SIGNAL_QZSS_L2C_ENA': True},
        {'CFG_SIGNAL_SBAS_ENA': True},
        {'CFG_SIGNAL_SBAS_L1CA_ENA': True},
        {'CFG_SIGNAL_GLO_ENA': False},
        {'CFG_SIGNAL_GLO_L1_ENA': False},
        {'CFG_SIGNAL_GLO_L2_ENA': False},
        {'CFG_SIGNAL_BDS_ENA': False},
        {'CFG_SIGNAL_BDS_B1_ENA': False},
        {'CFG_SIGNAL_BDS_B2_ENA': False},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 0},
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0},
    ]

    ublox_dgnss_node = Node(
        package='amr_sweeper_gnss',
        executable='amr_sweeper_ublox_dgnss_node',
        name='ublox_dgnss',
        namespace=LaunchConfiguration('namespace'),
        parameters=params,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ublox_dgnss_node')),
    )

    ublox_nav_sat_fix_hp_node = Node(
        package='amr_sweeper_gnss',
        executable='ublox_nav_sat_fix_hp',
        name='ublox_nav_sat_fix_hp',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'min_fix_type': 3,
            'min_horizontal_stddev_m': 1.5,
            'min_vertical_stddev_m': 3.0,
            'horizontal_covariance_scale': 4.0,
            'vertical_covariance_scale': 4.0,
            'use_hacc_vacc_covariance_floor': True,
        }],
        remappings=[('fix', 'navsat')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ublox_nav_sat_fix_hp')),
    )

    return launch.LaunchDescription([
        declare_use_ublox_dgnss_node,
        declare_use_ublox_nav_sat_fix_hp,
        declare_namespace,
        declare_frame_id,
        ublox_dgnss_node,
        ublox_nav_sat_fix_hp_node,
    ])
