#!/usr/bin/env python3
"""Republish /octomap_binary at a capped rate for the WiFi link.

octomap_server emits a new tree on every cloud insert.  Measured on this
workspace's synthetic sweep: 72 kB per message at 8 Hz = 4.6 Mbit/s.  That is
the same order as the camera stream that was measured driving the congested
2.4 GHz campus link to 56.7% packet loss and 3.5 s RTT, so streaming it raw to
RViz on the laptop takes the link down with it.

At 1 Hz the same tree costs 0.58 Mbit/s, which the link carries comfortably and
which is still far faster than a human can watch a map fill in.

This exists instead of `topic_tools throttle` because ros-humble-topic-tools is
not installed on either machine and this needs no new apt package.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from octomap_msgs.msg import Octomap


class OctomapThrottle(Node):

    def __init__(self):
        super().__init__('octomap_throttle')
        self.declare_parameter('in_topic', '/octomap_binary')
        self.declare_parameter('out_topic', '/octomap_binary_throttled')
        self.declare_parameter('rate_hz', 1.0)

        self.min_period = 1.0 / max(0.05, float(
            self.get_parameter('rate_hz').value))
        self._last = None
        self._latest = None
        self._dropped = 0
        self._sent = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            Octomap, self.get_parameter('out_topic').value, qos)
        self.create_subscription(
            Octomap, self.get_parameter('in_topic').value, self.on_map, qos)
        self.create_timer(self.min_period, self.emit)
        self.create_timer(30.0, self.report)

        self.get_logger().info(
            "throttling %s -> %s at %.2f Hz"
            % (self.get_parameter('in_topic').value,
               self.get_parameter('out_topic').value,
               1.0 / self.min_period))

    def on_map(self, msg):
        # Keep only the newest: the octree is a full snapshot, not a delta, so
        # every dropped message is fully superseded by the next one.
        if self._latest is not None:
            self._dropped += 1
        self._latest = msg

    def emit(self):
        if self._latest is None:
            return
        self.pub.publish(self._latest)
        self._sent += 1
        self._latest = None

    def report(self):
        if self._sent:
            self.get_logger().info(
                "%d published, %d superseded in the last 30 s"
                % (self._sent, self._dropped))
        self._sent = 0
        self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = OctomapThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
