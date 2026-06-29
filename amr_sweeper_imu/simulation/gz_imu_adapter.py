#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from compass_msgs.msg import Azimuth
from sensor_msgs.msg import Imu


def _normalize_quaternion(x: float, y: float, z: float, w: float) -> Optional[tuple[float, float, float, float]]:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return None
    return x / norm, y / norm, z / norm, w / norm


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class GzImuAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gz_imu_adapter")

        self._input_topic = self.declare_parameter("input_topic", "data_raw_gz").value
        self._raw_output_topic = self.declare_parameter("raw_output_topic", "data_raw").value
        self._acc_gyro_output_topic = self.declare_parameter("acc_gyro_output_topic", "data_acc_gyro").value
        self._heading_output_topic = self.declare_parameter("heading_output_topic", "data_heading").value
        self._azimuth_output_topic = self.declare_parameter("azimuth_output_topic", "azimuth").value
        self._frame_id = self.declare_parameter("frame_id", "imu_link").value
        self._orientation_covariance = list(
            self.declare_parameter(
                "orientation_covariance",
                [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05],
            ).value
        )
        self._heading_variance = float(self.declare_parameter("heading_variance", 0.05).value)
        self._warned_invalid_orientation = False
        publisher_qos = QoSProfile(depth=10)
        publisher_qos.reliability = QoSReliabilityPolicy.RELIABLE
        publisher_qos.durability = QoSDurabilityPolicy.VOLATILE

        self._raw_publisher = self.create_publisher(Imu, self._raw_output_topic, publisher_qos)
        self._acc_gyro_publisher = self.create_publisher(Imu, self._acc_gyro_output_topic, publisher_qos)
        self._heading_publisher = self.create_publisher(Imu, self._heading_output_topic, publisher_qos)
        self._azimuth_publisher = self.create_publisher(Azimuth, self._azimuth_output_topic, publisher_qos)
        self._subscription = self.create_subscription(
            Imu,
            self._input_topic,
            self._handle_imu,
            qos_profile_sensor_data,
        )

    def _handle_imu(self, msg: Imu) -> None:
        raw_msg = Imu()
        raw_msg = msg
        raw_msg.header.frame_id = self._frame_id

        normalized = _normalize_quaternion(
            raw_msg.orientation.x,
            raw_msg.orientation.y,
            raw_msg.orientation.z,
            raw_msg.orientation.w,
        )
        if normalized is None:
            normalized = (0.0, 0.0, 0.0, 1.0)
            if not self._warned_invalid_orientation:
                self.get_logger().warn(
                    "Gazebo IMU provided an invalid orientation quaternion; falling back to identity orientation."
                )
                self._warned_invalid_orientation = True

        raw_msg.orientation.x = normalized[0]
        raw_msg.orientation.y = normalized[1]
        raw_msg.orientation.z = normalized[2]
        raw_msg.orientation.w = normalized[3]

        if len(raw_msg.orientation_covariance) != 9 or raw_msg.orientation_covariance[0] < 0.0:
            raw_msg.orientation_covariance = self._orientation_covariance
        elif all(abs(value) < 1.0e-12 for value in raw_msg.orientation_covariance):
            raw_msg.orientation_covariance = self._orientation_covariance

        self._raw_publisher.publish(raw_msg)

        acc_gyro_msg = Imu()
        acc_gyro_msg = raw_msg
        acc_gyro_msg.orientation.x = 0.0
        acc_gyro_msg.orientation.y = 0.0
        acc_gyro_msg.orientation.z = 0.0
        acc_gyro_msg.orientation.w = 1.0
        acc_gyro_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._acc_gyro_publisher.publish(acc_gyro_msg)

        yaw = _yaw_from_quaternion(
            raw_msg.orientation.x,
            raw_msg.orientation.y,
            raw_msg.orientation.z,
            raw_msg.orientation.w,
        )
        yaw_qx, yaw_qy, yaw_qz, yaw_qw = _yaw_to_quaternion(yaw)

        heading_msg = Imu()
        heading_msg = raw_msg
        heading_msg.orientation.x = yaw_qx
        heading_msg.orientation.y = yaw_qy
        heading_msg.orientation.z = yaw_qz
        heading_msg.orientation.w = yaw_qw
        heading_msg.orientation_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, self._heading_variance,
        ]
        self._heading_publisher.publish(heading_msg)

        azimuth_msg = Azimuth()
        azimuth_msg.header = raw_msg.header
        azimuth_msg.azimuth = yaw
        azimuth_msg.variance = self._heading_variance
        azimuth_msg.unit = Azimuth.UNIT_RAD
        azimuth_msg.orientation = Azimuth.ORIENTATION_ENU
        azimuth_msg.reference = Azimuth.REFERENCE_GEOGRAPHIC
        self._azimuth_publisher.publish(azimuth_msg)


def main() -> None:
    rclpy.init()
    node = GzImuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
