#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


class DepthCameraSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_camera_simulation_node")

        self._input_depth_topic = self.declare_parameter(
            "input_depth_topic", "input/depth/image_rect_raw"
        ).value
        self._input_camera_info_topic = self.declare_parameter(
            "input_camera_info_topic", "input/depth/camera_info"
        ).value
        self._depth_topic = self.declare_parameter(
            "depth_topic", "depth/image_rect_raw"
        ).value
        self._camera_info_topic = self.declare_parameter(
            "camera_info_topic", "depth/camera_info"
        ).value
        self._pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "depth/color/points"
        ).value

        self._depth_publisher = self.create_publisher(Image, self._depth_topic, qos_profile_sensor_data)
        self._camera_info_publisher = self.create_publisher(CameraInfo, self._camera_info_topic, qos_profile_sensor_data)
        self._pointcloud_publisher = self.create_publisher(PointCloud2, self._pointcloud_topic, qos_profile_sensor_data)

        self._latest_camera_info: CameraInfo | None = None
        self._depth_subscription = self.create_subscription(
            Image,
            self._input_depth_topic,
            self._handle_depth_image,
            qos_profile_sensor_data,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._input_camera_info_topic,
            self._handle_camera_info,
            qos_profile_sensor_data,
        )

    def _handle_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _handle_depth_image(self, msg: Image) -> None:
        self._depth_publisher.publish(msg)

        if self._latest_camera_info is None:
            return

        camera_info = CameraInfo()
        camera_info.header = msg.header
        camera_info.height = self._latest_camera_info.height
        camera_info.width = self._latest_camera_info.width
        camera_info.distortion_model = self._latest_camera_info.distortion_model
        camera_info.d = list(self._latest_camera_info.d)
        camera_info.k = list(self._latest_camera_info.k)
        camera_info.r = list(self._latest_camera_info.r)
        camera_info.p = list(self._latest_camera_info.p)
        camera_info.binning_x = self._latest_camera_info.binning_x
        camera_info.binning_y = self._latest_camera_info.binning_y
        camera_info.roi = self._latest_camera_info.roi
        self._camera_info_publisher.publish(camera_info)
        self._pointcloud_publisher.publish(self._build_pointcloud(msg, camera_info))

    def _build_pointcloud(self, depth_image: Image, camera_info: CameraInfo) -> PointCloud2:
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        width = depth_image.width
        height = depth_image.height
        if fx == 0.0 or fy == 0.0 or width == 0 or height == 0:
            return self._empty_pointcloud(depth_image.header)

        points: list[tuple[float, float, float]] = []
        if depth_image.encoding == '16UC1':
            step = depth_image.step
            for v in range(height):
                row_offset = v * step
                for u in range(width):
                    offset = row_offset + (u * 2)
                    depth_mm = struct.unpack_from('<H', depth_image.data, offset)[0]
                    if depth_mm == 0:
                        continue
                    z = depth_mm * 0.001
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append((x, y, z))
        elif depth_image.encoding == '32FC1':
            step = depth_image.step
            for v in range(height):
                row_offset = v * step
                for u in range(width):
                    offset = row_offset + (u * 4)
                    z = struct.unpack_from('<f', depth_image.data, offset)[0]
                    if not math.isfinite(z) or z <= 0.0:
                        continue
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append((x, y, z))
        else:
            self.get_logger().error(
                'Unsupported simulated depth encoding %s; expected 16UC1 or 32FC1.',
                depth_image.encoding,
            )
            return self._empty_pointcloud(depth_image.header)

        cloud = PointCloud2()
        cloud.header = depth_image.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        return cloud

    def _empty_pointcloud(self, header) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 0
        cloud.is_dense = False
        cloud.data = b''
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


if __name__ == '__main__':
    main()
