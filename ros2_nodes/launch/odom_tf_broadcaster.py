#!/usr/bin/env python3

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

DEFAULT_ODOM_TOPIC = "/ekf_odom"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_footprint"


def frame_or_default(value: str, default: str) -> str:
    return value if value else default


class OdomTfBroadcaster(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("odom_tf_broadcaster")
        self._broadcaster = TransformBroadcaster(self)
        self._last_frames: tuple[str, str] | None = None
        self._subscriber = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.get_logger().info(f"broadcasting TF from {odom_topic}")

    def _odom_callback(self, msg: Odometry) -> None:
        parent_frame = frame_or_default(msg.header.frame_id, DEFAULT_ODOM_FRAME)
        child_frame = frame_or_default(msg.child_frame_id, DEFAULT_BASE_FRAME)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)

        current_frames = (parent_frame, child_frame)
        if current_frames != self._last_frames:
            self._last_frames = current_frames
            self.get_logger().info(
                f"publishing dynamic TF {parent_frame} -> {child_frame}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    args = parser.parse_args()

    rclpy.init()
    node = OdomTfBroadcaster(args.odom_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
