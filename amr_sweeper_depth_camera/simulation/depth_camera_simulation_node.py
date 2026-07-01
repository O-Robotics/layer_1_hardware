#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


class DepthCameraSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_camera_simulation_node")

        self._input_scan_topic = self.declare_parameter("input_scan_topic", "scan_gz").value
        self._output_scan_topic = self.declare_parameter("output_scan_topic", "scan").value
        self._camera_info_topic = self.declare_parameter("camera_info_topic", "depth/camera_info").value
        self._pointcloud_topic = self.declare_parameter("pointcloud_topic", "depth/color/points").value
        self._camera_frame_id = self.declare_parameter("camera_frame_id", "depth_camera_depth_frame").value
        self._output_frame = self.declare_parameter("output_frame", "laserscan_link").value
        self._range_min = float(self.declare_parameter("range_min", 0.20).value)
        self._range_max = float(self.declare_parameter("range_max", 5.0).value)
        self._scan_time = float(self.declare_parameter("scan_time", 0.05).value)
        self._camera_width = int(self.declare_parameter("camera_width", 640).value)
        self._camera_height = int(self.declare_parameter("camera_height", 480).value)
        self._camera_fov_deg = float(self.declare_parameter("camera_fov_deg", 87.0).value)
        publisher_qos = QoSProfile(depth=10)
        publisher_qos.reliability = QoSReliabilityPolicy.RELIABLE
        publisher_qos.durability = QoSDurabilityPolicy.VOLATILE

        self._scan_publisher = self.create_publisher(LaserScan, self._output_scan_topic, publisher_qos)
        self._camera_info_publisher = self.create_publisher(CameraInfo, self._camera_info_topic, publisher_qos)
        self._pointcloud_publisher = self.create_publisher(PointCloud2, self._pointcloud_topic, publisher_qos)
        self._subscription = self.create_subscription(
            LaserScan,
            self._input_scan_topic,
            self._handle_scan,
            qos_profile_sensor_data,
        )

    def _handle_scan(self, msg: LaserScan) -> None:
        scan_msg = msg
        scan_msg.header.frame_id = self._output_frame
        scan_msg.range_min = self._range_min
        scan_msg.range_max = self._range_max
        if self._scan_time > 0.0:
            scan_msg.scan_time = self._scan_time
        self._scan_publisher.publish(scan_msg)

        camera_info = self._build_camera_info(scan_msg)
        self._camera_info_publisher.publish(camera_info)

        pointcloud = self._build_pointcloud(scan_msg)
        self._pointcloud_publisher.publish(pointcloud)

    def _build_camera_info(self, scan_msg: LaserScan) -> CameraInfo:
        info = CameraInfo()
        info.header = scan_msg.header
        info.header.frame_id = self._camera_frame_id
        info.width = self._camera_width
        info.height = self._camera_height
        focal_length = (self._camera_width * 0.5) / math.tan(math.radians(self._camera_fov_deg) * 0.5)
        info.k = [
            focal_length, 0.0, self._camera_width * 0.5,
            0.0, focal_length, self._camera_height * 0.5,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            focal_length, 0.0, self._camera_width * 0.5, 0.0,
            0.0, focal_length, self._camera_height * 0.5, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info

    def _build_pointcloud(self, scan_msg: LaserScan) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = scan_msg.header
        cloud.height = 1

        points: list[tuple[float, float, float]] = []
        angle = scan_msg.angle_min
        for reading in scan_msg.ranges:
            if math.isfinite(reading) and self._range_min <= reading <= self._range_max:
                points.append((
                    reading * math.cos(angle),
                    reading * math.sin(angle),
                    0.0,
                ))
            angle += scan_msg.angle_increment

        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
        return cloud


def main() -> None:
    rclpy.init()
    node = DepthCameraSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
