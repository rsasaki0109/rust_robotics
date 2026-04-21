#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

DEFAULT_ODOM_TOPIC = "/synthetic_odom"
DEFAULT_SCAN_TOPIC = "/scan"
DEFAULT_DIAGNOSTICS_TOPIC = "/synthetic_slam_diagnostics"

ANGLE_MIN = -1.2
ANGLE_MAX = 1.2
RANGE_COUNT = 1401
RANGE_MIN = 0.05
RANGE_MAX = 12.0

WORLD_POINTS = [
    (2.0, -0.82),
    (2.15, -0.36),
    (2.35, 0.12),
    (2.55, 0.68),
    (2.85, -1.02),
    (3.05, -0.52),
    (3.25, 0.36),
    (3.55, 1.02),
    (3.85, -1.18),
    (4.1, -0.64),
    (4.32, -0.08),
    (4.55, 0.52),
    (4.8, 1.18),
    (5.05, -1.35),
    (5.3, -0.78),
    (5.55, -0.18),
    (5.85, 0.46),
    (6.1, 1.08),
    (6.4, -1.42),
    (6.65, -0.92),
    (6.95, -0.28),
    (7.2, 0.32),
    (7.48, 0.96),
    (7.8, -1.2),
    (8.05, -0.58),
    (8.35, 0.08),
    (8.65, 0.72),
]


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def parse_fields(line: str) -> dict[str, str]:
    fields = {}
    for token in line.split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value
    return fields


class SyntheticSlamIcpFeed(Node):
    def __init__(
        self,
        *,
        odom_topic: str,
        scan_topic: str,
        diagnostics_topic: str,
        step_distance: float,
    ) -> None:
        super().__init__("synthetic_slam_icp_feed")
        self._step_distance = step_distance
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self._diagnostics_sub = self.create_subscription(
            String,
            diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        self.matched_diagnostics: str | None = None
        self.last_diagnostics: str | None = None

    def _diagnostics_callback(self, msg: String) -> None:
        self.last_diagnostics = msg.data
        fields = parse_fields(msg.data)
        if (
            fields.get("status") == "icp_ok"
            and fields.get("gate_reason") == "accepted"
            and fields.get("blend_applied") == "true"
        ):
            self.matched_diagnostics = msg.data

    def publish_frame(self, step_index: int) -> None:
        pose_x = step_index * self._step_distance
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = pose_x
        odom.pose.pose.position.y = 0.0
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = quaternion_from_yaw(0.0)
        odom.twist.twist.linear.x = self._step_distance
        self._odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "base_scan"
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX
        scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / (RANGE_COUNT - 1)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        ranges = [math.inf] * RANGE_COUNT
        for world_x, world_y in WORLD_POINTS:
            rel_x = world_x - pose_x
            rel_y = world_y
            point_range = math.hypot(rel_x, rel_y)
            if not (RANGE_MIN < point_range < RANGE_MAX) or rel_x <= 0.1:
                continue
            angle = math.atan2(rel_y, rel_x)
            if not (ANGLE_MIN <= angle <= ANGLE_MAX):
                continue
            idx = round((angle - ANGLE_MIN) / scan.angle_increment)
            if 0 <= idx < RANGE_COUNT and point_range < ranges[idx]:
                ranges[idx] = point_range
        scan.ranges = ranges
        self._scan_pub.publish(scan)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Publish deterministic synthetic odom/scan data until SLAM ICP accepts a clean match."
    )
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--scan-topic", default=DEFAULT_SCAN_TOPIC)
    parser.add_argument("--diagnostics-topic", default=DEFAULT_DIAGNOSTICS_TOPIC)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--step-distance", type=float, default=0.08)
    parser.add_argument("--max-steps", type=int, default=80)
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticSlamIcpFeed(
        odom_topic=args.odom_topic,
        scan_topic=args.scan_topic,
        diagnostics_topic=args.diagnostics_topic,
        step_distance=args.step_distance,
    )

    deadline = time.monotonic() + args.timeout
    step_index = 0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_frame(step_index % args.max_steps)
            rclpy.spin_once(node, timeout_sec=0.12)
            if node.matched_diagnostics:
                print("Synthetic ICP acceptance matched:")
                print(node.matched_diagnostics)
                return 0
            step_index += 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        "Timed out waiting for status=icp_ok gate_reason=accepted diagnostics.",
        file=sys.stderr,
    )
    if node.last_diagnostics:
        print("Last diagnostics:", file=sys.stderr)
        print(node.last_diagnostics, file=sys.stderr)
    else:
        print("No diagnostics received.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
