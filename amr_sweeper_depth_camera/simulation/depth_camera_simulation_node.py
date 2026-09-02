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

        self._streams = {}
        self._add_stream(
            "depth",
            self.declare_parameter("input_depth_topic", "input/depth/image_rect_raw").value,
            self.declare_parameter("input_camera_info_topic", "input/depth/camera_info").value,
            self.declare_parameter("depth_topic", "depth/image_rect_raw").value,
            self.declare_parameter("camera_info_topic", "depth/camera_info").value,
            self.declare_parameter("depth_frame_id", "depth_camera_depth_optical_frame").value,
        )
        self._add_stream(
            "infra1",
            self.declare_parameter("input_infra1_topic", "input/infra1/image_rect_raw").value,
            self.declare_parameter("input_infra1_camera_info_topic", "input/infra1/camera_info").value,
            self.declare_parameter("infra1_topic", "infra1/image_rect_raw").value,
            self.declare_parameter("infra1_camera_info_topic", "infra1/camera_info").value,
            self.declare_parameter("infra1_frame_id", "depth_camera_infra1_optical_frame").value,
        )
        self._add_stream(
            "infra2",
            self.declare_parameter("input_infra2_topic", "input/infra2/image_rect_raw").value,
            self.declare_parameter("input_infra2_camera_info_topic", "input/infra2/camera_info").value,
            self.declare_parameter("infra2_topic", "infra2/image_rect_raw").value,
            self.declare_parameter("infra2_camera_info_topic", "infra2/camera_info").value,
            self.declare_parameter("infra2_frame_id", "depth_camera_infra2_optical_frame").value,
        )
        self._pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "depth/color/points"
        ).value
        self._color_frame_id = self.declare_parameter(
            "color_frame_id", "depth_camera_color_optical_frame"
        ).value

        self._pointcloud_publisher = self.create_publisher(PointCloud2, self._pointcloud_topic, qos_profile_sensor_data)
        self._color_image_publisher = self.create_publisher(
            Image,
            self.declare_parameter("color_topic", "color/image_raw").value,
            qos_profile_sensor_data,
        )
        self._color_camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.declare_parameter("color_camera_info_topic", "color/camera_info").value,
            qos_profile_sensor_data,
        )

    def _add_stream(
        self,
        name: str,
        input_image_topic: str,
        input_camera_info_topic: str,
        image_topic: str,
        camera_info_topic: str,
        frame_id: str,
    ) -> None:
        stream = {
            "frame_id": frame_id,
            "latest_camera_info": None,
            "image_publisher": self.create_publisher(Image, image_topic, qos_profile_sensor_data),
            "camera_info_publisher": self.create_publisher(CameraInfo, camera_info_topic, qos_profile_sensor_data),
        }
        stream["image_subscription"] = self.create_subscription(
            Image,
            input_image_topic,
            lambda msg, stream_name=name: self._handle_image(stream_name, msg),
            qos_profile_sensor_data,
        )
        stream["camera_info_subscription"] = self.create_subscription(
            CameraInfo,
            input_camera_info_topic,
            lambda msg, stream_name=name: self._handle_camera_info(stream_name, msg),
            qos_profile_sensor_data,
        )
        self._streams[name] = stream

    def _handle_camera_info(self, stream_name: str, msg: CameraInfo) -> None:
        self._streams[stream_name]["latest_camera_info"] = msg

    def _handle_image(self, stream_name: str, msg: Image) -> None:
        stream = self._streams[stream_name]
        frame_id = stream["frame_id"]
        if frame_id:
            msg.header.frame_id = frame_id
        stream["image_publisher"].publish(msg)

        latest_camera_info = stream["latest_camera_info"]
        if latest_camera_info is None:
            return

        camera_info = CameraInfo()
        camera_info.header = msg.header
        camera_info.height = latest_camera_info.height
        camera_info.width = latest_camera_info.width
        camera_info.distortion_model = latest_camera_info.distortion_model
        camera_info.d = list(latest_camera_info.d)
        camera_info.k = list(latest_camera_info.k)
        camera_info.r = list(latest_camera_info.r)
        camera_info.p = list(latest_camera_info.p)
        camera_info.binning_x = latest_camera_info.binning_x
        camera_info.binning_y = latest_camera_info.binning_y
        camera_info.roi = latest_camera_info.roi
        stream["camera_info_publisher"].publish(camera_info)
        if stream_name == "depth":
            self._publish_color_from_depth(msg, camera_info)
            self._pointcloud_publisher.publish(self._build_pointcloud(msg, camera_info))

    def _publish_color_from_depth(self, depth_image: Image, camera_info: CameraInfo) -> None:
        color = Image()
        color.header = depth_image.header
        if self._color_frame_id:
            color.header.frame_id = self._color_frame_id
        color.height = depth_image.height
        color.width = depth_image.width
        color.encoding = "rgb8"
        color.is_bigendian = False
        color.step = color.width * 3
        color.data = self._build_color_data(depth_image)

        color_info = CameraInfo()
        color_info.header = color.header
        color_info.height = camera_info.height
        color_info.width = camera_info.width
        color_info.distortion_model = camera_info.distortion_model
        color_info.d = list(camera_info.d)
        color_info.k = list(camera_info.k)
        color_info.r = list(camera_info.r)
        color_info.p = list(camera_info.p)
        color_info.binning_x = camera_info.binning_x
        color_info.binning_y = camera_info.binning_y
        color_info.roi = camera_info.roi

        self._color_image_publisher.publish(color)
        self._color_camera_info_publisher.publish(color_info)

    def _build_color_data(self, depth_image: Image) -> bytes:
        width = depth_image.width
        height = depth_image.height
        rgb = bytearray(width * height * 3)
        if width == 0 or height == 0:
            return bytes(rgb)

        for v in range(height):
            for u in range(width):
                depth_norm = 0
                if depth_image.encoding == "16UC1":
                    offset = (v * depth_image.step) + (u * 2)
                    depth_mm = struct.unpack_from("<H", depth_image.data, offset)[0]
                    depth_norm = min(255, depth_mm // 24)
                elif depth_image.encoding == "32FC1":
                    offset = (v * depth_image.step) + (u * 4)
                    depth_m = struct.unpack_from("<f", depth_image.data, offset)[0]
                    if math.isfinite(depth_m):
                        depth_norm = min(255, int(depth_m * 42.0))

                out = ((v * width) + u) * 3
                rgb[out] = (u * 255) // max(1, width - 1)
                rgb[out + 1] = (v * 255) // max(1, height - 1)
                rgb[out + 2] = depth_norm

        return bytes(rgb)

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
