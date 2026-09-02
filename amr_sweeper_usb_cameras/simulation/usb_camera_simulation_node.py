#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


CAMERA_NAMES = [
    "front_left_camera",
    "front_right_camera",
    "rear_left_camera",
    "rear_right_camera",
    "tools_camera",
]


class UsbCameraSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_simulation_node")
        self._width = int(self.declare_parameter("width", 320).value)
        self._height = int(self.declare_parameter("height", 240).value)
        self._publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self._sync_topic = str(self.declare_parameter("sync_topic", "").value)
        camera_names = self.declare_parameter("camera_names", CAMERA_NAMES).value
        self._camera_names = [str(name) for name in camera_names if str(name)]
        self._camera_publishers = {}
        self._start_time = self.get_clock().now()

        for index, camera_name in enumerate(self._camera_names):
            self._camera_publishers[camera_name] = {
                "index": index,
                "frame_id": f"{camera_name}_optical_frame",
                "image": self.create_publisher(
                    Image,
                    f"{camera_name}/image_raw",
                    qos_profile_sensor_data,
                ),
                "info": self.create_publisher(
                    CameraInfo,
                    f"{camera_name}/{camera_name}_info",
                    qos_profile_sensor_data,
                ),
            }

        self._timer = None
        self._sync_subscription = None
        if self._sync_topic:
            self._sync_subscription = self.create_subscription(
                Image,
                self._sync_topic,
                self._handle_sync_image,
                qos_profile_sensor_data,
            )
        else:
            period = 1.0 / self._publish_rate_hz if self._publish_rate_hz > 0.0 else 0.1
            self._timer = self.create_timer(period, self._publish_all)
        self.get_logger().info(
            f"Publishing simulated USB RGB cameras: {', '.join(self._camera_names)}"
        )

    def _publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_with_stamp(stamp)

    def _handle_sync_image(self, message: Image) -> None:
        self._publish_with_stamp(message.header.stamp)

    def _publish_with_stamp(self, stamp) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        for camera_name, camera in self._camera_publishers.items():
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = camera["frame_id"]
            image.height = self._height
            image.width = self._width
            image.encoding = "rgb8"
            image.is_bigendian = False
            image.step = self._width * 3
            image.data = self._build_image_data(camera["index"], elapsed)

            info = self._build_camera_info(image.header, camera_name)
            camera["image"].publish(image)
            camera["info"].publish(info)

    def _build_image_data(self, camera_index: int, elapsed: float) -> bytes:
        data = bytearray(self._width * self._height * 3)
        phase = int((math.sin(elapsed + camera_index) + 1.0) * 48.0)
        base_r = (43 + (camera_index * 37)) % 256
        base_g = (118 + (camera_index * 53)) % 256
        base_b = (191 + (camera_index * 29)) % 256

        for v in range(self._height):
            for u in range(self._width):
                idx = ((v * self._width) + u) * 3
                stripe = 64 if ((u // 32) + (v // 24) + camera_index) % 2 == 0 else 0
                data[idx] = (base_r + ((u * 80) // max(1, self._width - 1)) + phase) % 256
                data[idx + 1] = (base_g + ((v * 70) // max(1, self._height - 1)) + stripe) % 256
                data[idx + 2] = (base_b + stripe + phase) % 256

        return bytes(data)

    def _build_camera_info(self, header, camera_name: str) -> CameraInfo:
        info = CameraInfo()
        info.header = header
        info.height = self._height
        info.width = self._width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(self._width)
        fy = float(self._width)
        cx = (self._width - 1) * 0.5
        cy = (self._height - 1) * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def main() -> None:
    rclpy.init()
    node = UsbCameraSimulationNode()
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
