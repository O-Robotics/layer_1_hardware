#!/usr/bin/env python3

import math
import random
from typing import Optional, Tuple

import rclpy
from gps_msgs.msg import GPSFix, GPSStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rtcm_msgs.msg import Message as RtcmMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

A = 6378137.0
E2 = 0.00669437999014

GPS_STATE = "gps"
DGPS_STATE = "dgps"
RTK_FLOAT_STATE = "rtk_float"
RTK_FIXED_STATE = "rtk_fixed"


def enu_to_lla(x: float, y: float, z: float, origin_lat: float, origin_lon: float, origin_alt: float) -> Tuple[float, float, float]:
    lat0 = math.radians(origin_lat)
    lon0 = math.radians(origin_lon)
    alt0 = origin_alt
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    n0 = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
    x0 = (n0 + alt0) * cos_lat * cos_lon
    y0 = (n0 + alt0) * cos_lat * sin_lon
    z0 = (n0 * (1 - E2) + alt0) * sin_lat
    dx = -sin_lon * x - sin_lat * cos_lon * y + cos_lat * cos_lon * z
    dy = cos_lon * x - sin_lat * sin_lon * y + cos_lat * sin_lon * z
    dz = cos_lat * y + sin_lat * z
    xp = x0 + dx
    yp = y0 + dy
    zp = z0 + dz
    p = math.sqrt(xp * xp + yp * yp)
    lat = math.atan2(zp, p * (1.0 - E2))
    for _ in range(5):
        sin_iter = math.sin(lat)
        n = A / math.sqrt(1.0 - E2 * sin_iter * sin_iter)
        lat = math.atan2(zp + E2 * n * sin_iter, p)
    sin_final = math.sin(lat)
    n = A / math.sqrt(1.0 - E2 * sin_final * sin_final)
    alt = p / math.cos(lat) - n
    lon = math.atan2(yp, xp)
    return math.degrees(lat), math.degrees(lon), alt


def is_base_link(frame_id: str) -> bool:
    tail = frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
    return tail == "base_link"


def stamp_to_nanoseconds(stamp) -> int:
    return Time.from_msg(stamp).nanoseconds


class SimulatedGnssNode(Node):
    def __init__(self) -> None:
        super().__init__("simulated_gnss_node")

        self.declare_parameter("world_name", "amr_sweeper_test")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("navsat_topic", "navsat")
        self.declare_parameter("gpsfix_topic", "fix")
        self.declare_parameter("odometry_topic", "odometry")
        self.declare_parameter("status_marker_topic", "status_marker")
        self.declare_parameter("rtcm_topic", "ntrip_client/rtcm")
        self.declare_parameter("frame_id", "gnss_link")
        self.declare_parameter("origin_lat", 56.164520029)
        self.declare_parameter("origin_lon", 10.1453534275)
        self.declare_parameter("origin_alt", 100.0)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("stationary_speed_threshold_mps", 0.05)
        self.declare_parameter("noise_correlation_tau_s", 12.0)
        self.declare_parameter("autonomous_noise_h_m", 1.25)
        self.declare_parameter("autonomous_noise_v_m", 2.5)
        self.declare_parameter("dgps_noise_h_m", 0.7)
        self.declare_parameter("dgps_noise_v_m", 1.3)
        self.declare_parameter("rtk_float_noise_h_m", 0.3)
        self.declare_parameter("rtk_float_noise_v_m", 0.55)
        self.declare_parameter("rtk_fixed_noise_h_m", 0.12)
        self.declare_parameter("rtk_fixed_noise_v_m", 0.25)
        self.declare_parameter("autonomous_satellites", 10)
        self.declare_parameter("corrected_satellites", 14)
        self.declare_parameter("correction_timeout_s", 3.0)
        self.declare_parameter("dgps_warmup_s", 2.0)
        self.declare_parameter("rtk_float_warmup_s", 8.0)
        self.declare_parameter("min_fix_type", 1)
        self.declare_parameter("min_horizontal_stddev_m", 0.5)
        self.declare_parameter("min_vertical_stddev_m", 1.0)
        self.declare_parameter("horizontal_covariance_scale", 4.0)
        self.declare_parameter("vertical_covariance_scale", 4.0)
        self.declare_parameter("use_hacc_vacc_covariance_floor", True)
        self.declare_parameter("spike_at_s", -1.0)
        self.declare_parameter("spike_duration_s", 8.0)
        self.declare_parameter("spike_dx_m", 60.0)
        self.declare_parameter("spike_dy_m", 0.0)
        self.declare_parameter("outage_at_s", -1.0)
        self.declare_parameter("outage_duration_s", 25.0)
        self.declare_parameter("spike2_at_s", -1.0)
        self.declare_parameter("spike2_duration_s", 6.0)
        self.declare_parameter("spike2_dx_m", -60.0)
        self.declare_parameter("spike2_dy_m", 0.0)

        self.world_name = self.get_parameter("world_name").get_parameter_value().string_value
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.pose_topic = pose_topic or f"/world/{self.world_name}/pose/info"
        self.navsat_topic = self.get_parameter("navsat_topic").get_parameter_value().string_value
        self.gpsfix_topic = self.get_parameter("gpsfix_topic").get_parameter_value().string_value
        self.odometry_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.status_marker_topic = self.get_parameter("status_marker_topic").get_parameter_value().string_value
        self.rtcm_topic = self.get_parameter("rtcm_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.origin_lat = self.get_parameter("origin_lat").get_parameter_value().double_value
        self.origin_lon = self.get_parameter("origin_lon").get_parameter_value().double_value
        self.origin_alt = self.get_parameter("origin_alt").get_parameter_value().double_value
        self.spawn_x = self.get_parameter("spawn_x").get_parameter_value().double_value
        self.spawn_y = self.get_parameter("spawn_y").get_parameter_value().double_value
        self.publish_rate_hz = max(0.5, self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.publish_period_ns = int(1e9 / self.publish_rate_hz)
        self.stationary_speed_threshold_mps = max(
            0.0,
            self.get_parameter("stationary_speed_threshold_mps").get_parameter_value().double_value,
        )
        self.noise_correlation_tau_s = max(1e-3, self.get_parameter("noise_correlation_tau_s").get_parameter_value().double_value)
        self.autonomous_satellites = max(4, self.get_parameter("autonomous_satellites").get_parameter_value().integer_value)
        self.corrected_satellites = max(self.autonomous_satellites, self.get_parameter("corrected_satellites").get_parameter_value().integer_value)
        self.correction_timeout_s = max(0.1, self.get_parameter("correction_timeout_s").get_parameter_value().double_value)
        self.dgps_warmup_s = max(0.0, self.get_parameter("dgps_warmup_s").get_parameter_value().double_value)
        self.rtk_float_warmup_s = max(0.0, self.get_parameter("rtk_float_warmup_s").get_parameter_value().double_value)
        self.min_fix_type = self.get_parameter("min_fix_type").get_parameter_value().integer_value
        self.min_horizontal_stddev_m = max(0.0, self.get_parameter("min_horizontal_stddev_m").get_parameter_value().double_value)
        self.min_vertical_stddev_m = max(0.0, self.get_parameter("min_vertical_stddev_m").get_parameter_value().double_value)
        self.horizontal_covariance_scale = max(1.0, self.get_parameter("horizontal_covariance_scale").get_parameter_value().double_value)
        self.vertical_covariance_scale = max(1.0, self.get_parameter("vertical_covariance_scale").get_parameter_value().double_value)
        self.use_hacc_vacc_covariance_floor = self.get_parameter("use_hacc_vacc_covariance_floor").get_parameter_value().bool_value

        self.state_sigmas = {
            GPS_STATE: (
                self.get_parameter("autonomous_noise_h_m").get_parameter_value().double_value,
                self.get_parameter("autonomous_noise_v_m").get_parameter_value().double_value,
            ),
            DGPS_STATE: (
                self.get_parameter("dgps_noise_h_m").get_parameter_value().double_value,
                self.get_parameter("dgps_noise_v_m").get_parameter_value().double_value,
            ),
            RTK_FLOAT_STATE: (
                self.get_parameter("rtk_float_noise_h_m").get_parameter_value().double_value,
                self.get_parameter("rtk_float_noise_v_m").get_parameter_value().double_value,
            ),
            RTK_FIXED_STATE: (
                self.get_parameter("rtk_fixed_noise_h_m").get_parameter_value().double_value,
                self.get_parameter("rtk_fixed_noise_v_m").get_parameter_value().double_value,
            ),
        }

        self.navsat_publisher = self.create_publisher(NavSatFix, self.navsat_topic, 10)
        self.gpsfix_publisher = self.create_publisher(GPSFix, self.gpsfix_topic, 10)
        self.odometry_publisher = self.create_publisher(Odometry, self.odometry_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, self.status_marker_topic, 10)
        self.pose_subscription = self.create_subscription(
            TFMessage,
            self.pose_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.rtcm_subscription = self.create_subscription(RtcmMessage, self.rtcm_topic, self.handle_rtcm, 10)

        self.body_frame_id: Optional[str] = None
        self.last_body_xy: Tuple[float, float] = (self.spawn_x, self.spawn_y)
        self.start_ns: Optional[int] = None
        self.last_publish_ns: Optional[int] = None
        self.last_rtcm_ns: Optional[int] = None
        self.first_rtcm_ns: Optional[int] = None
        self.last_status_label = ""
        self.last_fix_state: Optional[str] = None
        self.noise_x = 0.0
        self.noise_y = 0.0
        self.noise_z = 0.0
        self.previous_position: Optional[Tuple[float, float, float]] = None
        self.previous_true_position: Optional[Tuple[float, float, float]] = None
        self.previous_publish_ns: Optional[int] = None
        self.last_track_deg = 0.0

        self.get_logger().info(
            "Simulated GNSS node ready "
            f"(pose_topic={self.pose_topic}, navsat={self.navsat_topic}, gpsfix={self.gpsfix_topic}, rtcm_topic={self.rtcm_topic})"
        )

    def handle_rtcm(self, _: RtcmMessage) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.last_rtcm_ns = now_ns
        if self.first_rtcm_ns is None:
            self.first_rtcm_ns = now_ns
            self.get_logger().info("Simulated GNSS received first RTCM correction packet")

    def elapsed_seconds(self) -> float:
        if self.start_ns is None:
            return 0.0
        return (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9

    def in_window(self, at_param: str, dur_param: str) -> bool:
        at = self.get_parameter(at_param).get_parameter_value().double_value
        if at < 0.0:
            return False
        duration = self.get_parameter(dur_param).get_parameter_value().double_value
        elapsed = self.elapsed_seconds()
        return at <= elapsed < (at + duration)

    def outage_active(self) -> bool:
        return self.in_window("outage_at_s", "outage_duration_s")

    def spike1_active(self) -> bool:
        return self.in_window("spike_at_s", "spike_duration_s")

    def spike2_active(self) -> bool:
        return self.in_window("spike2_at_s", "spike2_duration_s")

    def find_body_transform(self, message: TFMessage):
        # Preferred path: if child_frame_id is ever populated with a real name
        # (e.g. a future bridge fix), lock onto the named base_link for good.
        if self.body_frame_id:
            for transform in message.transforms:
                if transform.child_frame_id == self.body_frame_id:
                    translation = transform.transform.translation
                    if 0.05 < translation.z < 0.8:
                        self.last_body_xy = (translation.x, translation.y)
                        return transform
        for transform in message.transforms:
            if is_base_link(transform.child_frame_id):
                translation = transform.transform.translation
                if 0.05 < translation.z < 0.8:
                    self.body_frame_id = transform.child_frame_id
                    self.last_body_xy = (translation.x, translation.y)
                    return transform

        # The Gazebo Pose_V -> TFMessage bridge used here never carries entity
        # names, so every transform has an empty frame_id/child_frame_id and
        # neither lookup above can ever succeed -- this world/pose/info topic
        # reports every model in the scene, not just the robot. Identify the
        # robot by nearest-neighbor continuity instead: among everything at
        # plausible chassis height, take whichever entry is closest to where
        # we last saw it (seeded from the known spawn pose). The robot moves
        # continuously and slowly relative to this topic's publish rate, so
        # it's always the closest match to its own previous position, while
        # unrelated static props sit elsewhere and never win.
        last_x, last_y = self.last_body_xy
        best_transform = None
        best_distance_sq = float("inf")
        for transform in message.transforms:
            translation = transform.transform.translation
            if not (0.05 < translation.z < 0.8):
                continue
            dx = translation.x - last_x
            dy = translation.y - last_y
            distance_sq = dx * dx + dy * dy
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_transform = transform
        if best_transform is not None:
            translation = best_transform.transform.translation
            self.last_body_xy = (translation.x, translation.y)
        return best_transform

    def current_fix_state(self, now_ns: int) -> str:
        if self.last_rtcm_ns is None or (now_ns - self.last_rtcm_ns) * 1e-9 > self.correction_timeout_s:
            return GPS_STATE
        if self.first_rtcm_ns is None:
            return DGPS_STATE
        corrected_age_s = (now_ns - self.first_rtcm_ns) * 1e-9
        if corrected_age_s < self.dgps_warmup_s:
            return DGPS_STATE
        if corrected_age_s < (self.dgps_warmup_s + self.rtk_float_warmup_s):
            return RTK_FLOAT_STATE
        return RTK_FIXED_STATE

    def sample_correlated_noise(self, dt_s: float, sigma_h: float, sigma_v: float) -> None:
        alpha = math.exp(-dt_s / self.noise_correlation_tau_s)
        beta = math.sqrt(max(0.0, 1.0 - alpha * alpha))
        self.noise_x = alpha * self.noise_x + beta * random.gauss(0.0, sigma_h)
        self.noise_y = alpha * self.noise_y + beta * random.gauss(0.0, sigma_h)
        self.noise_z = alpha * self.noise_z + beta * random.gauss(0.0, sigma_v)

    def publish_status_marker(self, fix_state: str, stamp=None) -> None:
        if fix_state == RTK_FIXED_STATE:
            label = "  GNSS RTK FIXED  "
            color = (0.1, 0.9, 0.1)
        elif fix_state == RTK_FLOAT_STATE:
            label = "  GNSS RTK FLOAT  "
            color = (0.2, 0.7, 1.0)
        elif fix_state == DGPS_STATE:
            label = "  GNSS DGPS  "
            color = (0.95, 0.75, 0.1)
        else:
            label = "  GNSS GPS  "
            color = (0.95, 0.45, 0.0)
        if label == self.last_status_label:
            return
        self.last_status_label = label
        marker = Marker()
        marker.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "gnss_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -18.0
        marker.pose.position.y = 28.0
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 2.5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = label
        self.marker_publisher.publish(marker)

    def publish_fix_state_log(self, fix_state: str) -> None:
        if fix_state == self.last_fix_state:
            return
        self.last_fix_state = fix_state
        self.get_logger().info(f"Simulated GNSS state changed to {fix_state}")

    def compute_spike_offsets(self) -> Tuple[float, float]:
        if self.spike1_active():
            return (
                self.get_parameter("spike_dx_m").get_parameter_value().double_value,
                self.get_parameter("spike_dy_m").get_parameter_value().double_value,
            )
        if self.spike2_active():
            return (
                self.get_parameter("spike2_dx_m").get_parameter_value().double_value,
                self.get_parameter("spike2_dy_m").get_parameter_value().double_value,
            )
        return (0.0, 0.0)

    def covariance_for_sigmas(self, sigma_h: float, sigma_v: float):
        horizontal_floor_stddev = max(
            self.min_horizontal_stddev_m,
            sigma_h if self.use_hacc_vacc_covariance_floor else 0.0,
        )
        vertical_floor_stddev = max(
            self.min_vertical_stddev_m,
            sigma_v if self.use_hacc_vacc_covariance_floor else 0.0,
        )
        horizontal_variance = max(
            sigma_h * sigma_h * self.horizontal_covariance_scale,
            horizontal_floor_stddev * horizontal_floor_stddev,
        )
        vertical_variance = max(
            sigma_v * sigma_v * self.vertical_covariance_scale,
            vertical_floor_stddev * vertical_floor_stddev,
        )
        return [
            horizontal_variance, 0.0, 0.0,
            0.0, horizontal_variance, 0.0,
            0.0, 0.0, vertical_variance,
        ]

    def publish_messages(
        self,
        stamp,
        x: float,
        y: float,
        z: float,
        true_x: float,
        true_y: float,
        true_z: float,
        sigma_h: float,
        sigma_v: float,
        fix_state: str,
        now_ns: int,
    ) -> None:
        latitude, longitude, altitude = enu_to_lla(x, y, z, self.origin_lat, self.origin_lon, self.origin_alt)
        covariance = self.covariance_for_sigmas(sigma_h, sigma_v)

        navsat = NavSatFix()
        navsat.header.stamp = stamp
        navsat.header.frame_id = self.frame_id
        navsat.latitude = latitude
        navsat.longitude = longitude
        navsat.altitude = altitude
        navsat.position_covariance = covariance
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.status.status = NavSatStatus.STATUS_GBAS_FIX if fix_state == RTK_FIXED_STATE else NavSatStatus.STATUS_FIX
        self.navsat_publisher.publish(navsat)

        dt_s = 0.0
        if self.previous_publish_ns is not None:
            dt_s = max(1e-6, (now_ns - self.previous_publish_ns) * 1e-9)
        speed = 0.0
        climb = 0.0
        if self.previous_true_position is not None and dt_s > 0.0:
            dx = true_x - self.previous_true_position[0]
            dy = true_y - self.previous_true_position[1]
            dz = true_z - self.previous_true_position[2]
            vx = dx / dt_s
            vy = dy / dt_s
            vz = dz / dt_s
            speed = math.hypot(vx, vy)
            climb = vz
            if speed > self.stationary_speed_threshold_mps:
                self.last_track_deg = (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0

        gpsfix = GPSFix()
        gpsfix.header.stamp = stamp
        gpsfix.header.frame_id = self.frame_id
        gpsfix.status.header.stamp = stamp
        gpsfix.status.header.frame_id = self.frame_id
        if fix_state == RTK_FIXED_STATE:
            gpsfix.status.status = GPSStatus.STATUS_RTK_FIX
            satellites = self.corrected_satellites
        elif fix_state == RTK_FLOAT_STATE:
            gpsfix.status.status = GPSStatus.STATUS_RTK_FLOAT
            satellites = max(self.corrected_satellites - 1, self.autonomous_satellites)
        elif fix_state == DGPS_STATE:
            gpsfix.status.status = GPSStatus.STATUS_DGPS_FIX
            satellites = max(self.corrected_satellites - 2, self.autonomous_satellites)
        else:
            gpsfix.status.status = GPSStatus.STATUS_FIX
            satellites = self.autonomous_satellites
        gpsfix.status.position_source = GPSStatus.SOURCE_GPS
        gpsfix.status.motion_source = GPSStatus.SOURCE_DOPPLER
        gpsfix.status.orientation_source = GPSStatus.SOURCE_POINTS
        gpsfix.status.satellites_used = satellites
        gpsfix.latitude = latitude
        gpsfix.longitude = longitude
        gpsfix.altitude = altitude
        gpsfix.speed = speed
        gpsfix.climb = climb
        gpsfix.track = self.last_track_deg
        gpsfix.pdop = math.sqrt(max(1e-9, sigma_h * sigma_h + sigma_v * sigma_v))
        gpsfix.hdop = math.sqrt(max(covariance[0], 1e-9))
        gpsfix.vdop = math.sqrt(max(covariance[8], 1e-9))
        gpsfix.err_horz = sigma_h
        gpsfix.err_vert = sigma_v
        gpsfix.err = max(gpsfix.err_horz, gpsfix.err_vert)
        gpsfix.position_covariance = covariance
        gpsfix.position_covariance_type = GPSFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gpsfix_publisher.publish(gpsfix)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.w = 1.0
        odometry.pose.covariance[0] = covariance[0]
        odometry.pose.covariance[7] = covariance[4]
        odometry.pose.covariance[14] = 1e6
        odometry.pose.covariance[21] = 1e6
        odometry.pose.covariance[28] = 1e6
        odometry.pose.covariance[35] = 1e6
        self.odometry_publisher.publish(odometry)

        self.previous_position = (x, y, z)
        self.previous_true_position = (true_x, true_y, true_z)
        self.previous_publish_ns = now_ns

    def pose_callback(self, message: TFMessage) -> None:
        transform = self.find_body_transform(message)
        if transform is None:
            return

        translation = transform.transform.translation
        stamp_ns = stamp_to_nanoseconds(transform.header.stamp)
        if stamp_ns > 0:
            stamp = transform.header.stamp
        else:
            current_time = self.get_clock().now()
            stamp_ns = current_time.nanoseconds
            stamp = current_time.to_msg()

        if self.start_ns is None:
            self.start_ns = stamp_ns
        if self.last_publish_ns is not None and (stamp_ns - self.last_publish_ns) < self.publish_period_ns:
            return
        if self.outage_active():
            self.publish_status_marker(GPS_STATE, stamp)
            return

        fix_state = self.current_fix_state(stamp_ns)
        self.publish_status_marker(fix_state, stamp)
        self.publish_fix_state_log(fix_state)
        sigma_h, sigma_v = self.state_sigmas[fix_state]
        dt_s = 0.0 if self.last_publish_ns is None else max(0.0, (stamp_ns - self.last_publish_ns) * 1e-9)
        self.sample_correlated_noise(dt_s, sigma_h, sigma_v)
        self.last_publish_ns = stamp_ns
        spike_dx_m, spike_dy_m = self.compute_spike_offsets()
        true_x = translation.x + spike_dx_m
        true_y = translation.y + spike_dy_m
        true_z = translation.z
        x = true_x + self.noise_x
        y = true_y + self.noise_y
        z = true_z + self.noise_z
        self.publish_messages(
            stamp,
            x,
            y,
            z,
            true_x,
            true_y,
            true_z,
            sigma_h,
            sigma_v,
            fix_state,
            stamp_ns,
        )


def main() -> None:
    rclpy.init()
    node = SimulatedGnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
