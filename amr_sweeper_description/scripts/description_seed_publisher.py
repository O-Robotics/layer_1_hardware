#!/usr/bin/env python3

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class DescriptionSeedPublisher(Node):
    def __init__(self) -> None:
        super().__init__("description_seed_publisher")

        self.publish_rate_hz = max(0.1, float(self.declare_parameter("publish_rate_hz", 2.0).value))
        self.graph_check_period_sec = max(
            0.1, float(self.declare_parameter("graph_check_period_sec", 0.5).value)
        )

        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_footprint_frame = str(
            self.declare_parameter("base_footprint_frame", "base_footprint").value
        )
        self.base_link_frame = str(self.declare_parameter("base_link_frame", "base_link").value)

        self.base_link_seed_roll_deg = float(
            self.declare_parameter("base_link_seed_roll_deg", 0.0).value
        )
        self.base_link_seed_pitch_deg = float(
            self.declare_parameter("base_link_seed_pitch_deg", 4.5).value
        )
        self.base_link_seed_yaw_deg = float(
            self.declare_parameter("base_link_seed_yaw_deg", 0.0).value
        )
        self.base_link_seed_z_m = float(self.declare_parameter("base_link_seed_z_m", 0.13).value)

        self.joint_names = [
            str(name)
            for name in self.declare_parameter(
                "joint_names",
                [
                    "LeftWheel_joint",
                    "RightWheel_joint",
                    "LeftBrush_joint",
                    "RightBrush_joint",
                ],
            ).value
        ]
        self.joint_positions = [
            float(position)
            for position in self.declare_parameter(
                "joint_positions", [0.0] * len(self.joint_names)
            ).value
        ]
        if len(self.joint_positions) != len(self.joint_names):
            self.get_logger().warn(
                "joint_positions length did not match joint_names; resetting all seed positions to zero."
            )
            self.joint_positions = [0.0] * len(self.joint_names)

        self.attitude_owner_names = self._qualified_names(
            self.declare_parameter("attitude_owner_node_names", ["attitude_controller_node"]).value
        )
        self.localization_owner_names = self._qualified_names(
            self.declare_parameter("localization_owner_node_names", ["odometry_projection"]).value
        )
        self.mapping_owner_names = self._qualified_names(
            self.declare_parameter("mapping_owner_node_names", ["map_pose_node"]).value
        )

        self.seed_map_to_odom = True
        self.seed_odom_to_base_footprint = True
        self.seed_base_footprint_to_base_link = True
        self.seed_joint_states = bool(self.joint_names)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)

        self.publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_seed_data)
        self.graph_timer = self.create_timer(self.graph_check_period_sec, self._update_seed_ownership)

        self.get_logger().info(
            "Publishing description seed TF/joint states until runtime owners appear."
        )

    def _qualified_names(self, relative_names: Iterable[str]) -> set[str]:
        namespace = self.get_namespace().strip("/")
        qualified_names: set[str] = set()
        for name in relative_names:
            clean_name = str(name).strip().strip("/")
            if not clean_name:
                continue
            if namespace:
                qualified_names.add(f"/{namespace}/{clean_name}")
            else:
                qualified_names.add(f"/{clean_name}")
        return qualified_names

    def _graph_nodes(self) -> set[str]:
        return {
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def _update_seed_ownership(self) -> None:
        active_nodes = self._graph_nodes()

        if self.seed_base_footprint_to_base_link and self.attitude_owner_names.intersection(active_nodes):
            self.seed_base_footprint_to_base_link = False
            self.get_logger().info(
                f"Stopping seeded {self.base_footprint_frame} -> {self.base_link_frame} because the attitude controller is now running."
            )

        if self.seed_odom_to_base_footprint and self.localization_owner_names.intersection(active_nodes):
            self.seed_odom_to_base_footprint = False
            self.get_logger().info(
                f"Stopping seeded {self.odom_frame} -> {self.base_footprint_frame} because the localization projector is now running."
            )

        if self.seed_map_to_odom and self.mapping_owner_names.intersection(active_nodes):
            self.seed_map_to_odom = False
            self.get_logger().info(
                f"Stopping seeded {self.map_frame} -> {self.odom_frame} because map_pose_node is now running."
            )

        if self.seed_joint_states and self.count_publishers("joint_states") > 1:
            self.seed_joint_states = False
            self.get_logger().info(
                "Stopping seeded joint_states because another joint-state publisher is now running."
            )

        if not any(
            [
                self.seed_map_to_odom,
                self.seed_odom_to_base_footprint,
                self.seed_base_footprint_to_base_link,
                self.seed_joint_states,
            ]
        ):
            self.get_logger().info("All description seed publishers have been superseded; shutting down.")
            self.publish_timer.cancel()
            self.graph_timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

    def _publish_seed_data(self) -> None:
        now = self.get_clock().now().to_msg()

        if self.seed_map_to_odom:
            self.tf_broadcaster.sendTransform(self._identity_transform(now, self.map_frame, self.odom_frame))

        if self.seed_odom_to_base_footprint:
            self.tf_broadcaster.sendTransform(
                self._identity_transform(now, self.odom_frame, self.base_footprint_frame)
            )

        if self.seed_base_footprint_to_base_link:
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.base_footprint_frame
            transform.child_frame_id = self.base_link_frame
            transform.transform.translation.z = self.base_link_seed_z_m
            qx, qy, qz, qw = _quaternion_from_rpy(
                math.radians(self.base_link_seed_roll_deg),
                math.radians(self.base_link_seed_pitch_deg),
                math.radians(self.base_link_seed_yaw_deg),
            )
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        if self.seed_joint_states:
            joint_state = JointState()
            joint_state.header.stamp = now
            joint_state.name = self.joint_names
            joint_state.position = self.joint_positions
            self.joint_state_publisher.publish(joint_state)

    @staticmethod
    def _identity_transform(stamp, parent_frame: str, child_frame: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.rotation.w = 1.0
        return transform


def main() -> None:
    rclpy.init()
    node = DescriptionSeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
