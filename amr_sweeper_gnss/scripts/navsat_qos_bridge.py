#!/usr/bin/env python3

"""Bridge NavSatFix from best-effort GNSS QoS to reliable QoS for NTRIP."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix


class NavSatQosBridge(Node):
    """Republish NavSatFix with reliable QoS for consumers that need it."""

    def __init__(self) -> None:
        super().__init__('navsat_qos_bridge')

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._publisher = self.create_publisher(NavSatFix, 'ntrip_fix', reliable_qos)
        self._subscription = self.create_subscription(
            NavSatFix,
            'navsat',
            self._handle_navsat,
            best_effort_qos,
        )

    def _handle_navsat(self, msg: NavSatFix) -> None:
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = NavSatQosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
