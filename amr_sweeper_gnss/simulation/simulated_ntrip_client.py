#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix


class SimulatedNtripClient(Node):
    def __init__(self) -> None:
        super().__init__("simulated_ntrip_client")

        self.declare_parameter("rtcm_topic", "ntrip_client/rtcm")
        self.declare_parameter("rtcm_frame_id", "gnss_link")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("startup_delay_s", 1.5)
        self.declare_parameter("send_nmea", True)

        self.rtcm_topic = self.get_parameter("rtcm_topic").get_parameter_value().string_value
        self.rtcm_frame_id = self.get_parameter("rtcm_frame_id").get_parameter_value().string_value
        self.publish_rate_hz = max(
            0.1,
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value,
        )
        self.startup_delay_s = max(
            0.0,
            self.get_parameter("startup_delay_s").get_parameter_value().double_value,
        )
        self.send_nmea = self.get_parameter("send_nmea").get_parameter_value().bool_value

        self.publisher = self.create_publisher(Message, self.rtcm_topic, 10)
        self.latest_fix: Optional[NavSatFix] = None
        self.first_fix_logged = False
        self.started_ns = self.get_clock().now().nanoseconds
        self.ready_logged = False

        self.fix_subscription = self.create_subscription(
            NavSatFix,
            "fix",
            self.handle_fix,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_rtcm)

        self.get_logger().info(
            "Simulated NTRIP client ready "
            f"(topic={self.rtcm_topic}, send_nmea={str(self.send_nmea).lower()}, "
            f"startup_delay_s={self.startup_delay_s:.1f})"
        )

    def handle_fix(self, message: NavSatFix) -> None:
        self.latest_fix = message
        if self.send_nmea and not self.first_fix_logged:
            self.first_fix_logged = True
            self.get_logger().info(
                "Simulated NTRIP client received first NavSatFix and can start RTCM output"
            )

    def publish_rtcm(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.started_ns) * 1e-9 < self.startup_delay_s:
            return

        if self.send_nmea and self.latest_fix is None:
            self.get_logger().debug(
                "Simulated NTRIP client waiting for initial NavSatFix because send_nmea is enabled"
            )
            return

        if not self.ready_logged:
            self.ready_logged = True
            self.get_logger().info("Simulated NTRIP client publishing RTCM corrections")

        msg = Message()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.rtcm_frame_id
        msg.message = [0xD3, 0x00, 0x01, 0x00]
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimulatedNtripClient()
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
