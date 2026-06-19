#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState


class DefaultJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("default_joint_state_publisher")

        self._topic_name = self.declare_parameter(
            "joint_states_topic", "attitude_controller/joint_states"
        ).value
        self._roll_joint_name = self.declare_parameter(
            "base_roll_joint_name", "base_roll_joint"
        ).value
        self._pitch_joint_name = self.declare_parameter(
            "base_pitch_joint_name", "base_pitch_joint"
        ).value
        self._initial_roll_deg = float(self.declare_parameter("initial_roll_deg", 0.0).value)
        self._initial_pitch_deg = float(self.declare_parameter("initial_pitch_deg", 4.5).value)
        self._publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 5.0).value)
        self._publisher_frame_id = self.declare_parameter(
            "publisher_frame_id", "default_joint_state_seed"
        ).value
        self._stop_on_external_message = bool(
            self.declare_parameter("stop_on_external_message", True).value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(JointState, self._topic_name, qos)
        self._subscription = self.create_subscription(
            JointState, self._topic_name, self._handle_joint_state, qos
        )

        self._external_override_seen = False
        self._message = JointState()
        self._message.header.frame_id = self._publisher_frame_id
        self._message.name = [self._roll_joint_name, self._pitch_joint_name]
        self._message.position = [
            math.radians(self._initial_roll_deg),
            math.radians(self._initial_pitch_deg),
        ]

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 5.0

        self._publish_once()
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_once)

    def _handle_joint_state(self, msg: JointState) -> None:
        if not self._stop_on_external_message:
            return
        if msg.header.frame_id == self._publisher_frame_id:
            return

        joint_names = set(msg.name)
        required = {self._roll_joint_name, self._pitch_joint_name}
        if not required.issubset(joint_names):
            return

        if not self._external_override_seen:
            self._external_override_seen = True
            self.get_logger().info(
                "Received external joint-state update on %s; stopping default joint-state seeding.",
                self._topic_name,
            )
            self._timer.cancel()

    def _publish_once(self) -> None:
        if self._external_override_seen:
            return
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main() -> None:
    rclpy.init()
    node = DefaultJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
