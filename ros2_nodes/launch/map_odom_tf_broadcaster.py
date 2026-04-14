#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

DEFAULT_MAP_ODOM_TOPIC = "/slam_odom"
DEFAULT_RAW_ODOM_TOPIC = "/ekf_odom"
DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_footprint"


def frame_or_default(value: str, default: str) -> str:
    return value if value else default


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def inverse_pose_2d(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    inv_x = -(cos_yaw * x + sin_yaw * y)
    inv_y = sin_yaw * x - cos_yaw * y
    return inv_x, inv_y, -yaw


def compose_pose_2d(
    left_x: float,
    left_y: float,
    left_yaw: float,
    right_x: float,
    right_y: float,
    right_yaw: float,
) -> tuple[float, float, float]:
    cos_yaw = math.cos(left_yaw)
    sin_yaw = math.sin(left_yaw)
    x = left_x + cos_yaw * right_x - sin_yaw * right_y
    y = left_y + sin_yaw * right_x + cos_yaw * right_y
    yaw = left_yaw + right_yaw
    return x, y, yaw


class MapOdomTfBroadcaster(Node):
    def __init__(self, map_odom_topic: str, raw_odom_topic: str) -> None:
        super().__init__("map_odom_tf_broadcaster")
        self._broadcaster = TransformBroadcaster(self)
        self._latest_map_odom: Odometry | None = None
        self._latest_raw_odom: Odometry | None = None
        self._last_frames: tuple[str, str] | None = None

        self._map_subscriber = self.create_subscription(
            Odometry,
            map_odom_topic,
            self._map_odom_callback,
            10,
        )
        self._raw_subscriber = self.create_subscription(
            Odometry,
            raw_odom_topic,
            self._raw_odom_callback,
            10,
        )
        self.get_logger().info(
            f"broadcasting map->odom TF from {map_odom_topic} against {raw_odom_topic}"
        )

    def _map_odom_callback(self, msg: Odometry) -> None:
        self._latest_map_odom = msg
        self._publish_if_ready()

    def _raw_odom_callback(self, msg: Odometry) -> None:
        self._latest_raw_odom = msg
        self._publish_if_ready()

    def _publish_if_ready(self) -> None:
        if self._latest_map_odom is None or self._latest_raw_odom is None:
            return

        map_parent = frame_or_default(
            self._latest_map_odom.header.frame_id,
            DEFAULT_MAP_FRAME,
        )
        raw_parent = frame_or_default(
            self._latest_raw_odom.header.frame_id,
            DEFAULT_ODOM_FRAME,
        )
        map_child = frame_or_default(
            self._latest_map_odom.child_frame_id,
            DEFAULT_BASE_FRAME,
        )
        raw_child = frame_or_default(
            self._latest_raw_odom.child_frame_id,
            DEFAULT_BASE_FRAME,
        )
        if map_child != raw_child:
            self.get_logger().warn(
                f"skipping TF publish because child frames differ: {map_child} vs {raw_child}"
            )
            return

        map_pose = self._latest_map_odom.pose.pose
        raw_pose = self._latest_raw_odom.pose.pose
        map_yaw = yaw_from_quaternion(
            map_pose.orientation.x,
            map_pose.orientation.y,
            map_pose.orientation.z,
            map_pose.orientation.w,
        )
        raw_yaw = yaw_from_quaternion(
            raw_pose.orientation.x,
            raw_pose.orientation.y,
            raw_pose.orientation.z,
            raw_pose.orientation.w,
        )

        inv_raw_x, inv_raw_y, inv_raw_yaw = inverse_pose_2d(
            raw_pose.position.x,
            raw_pose.position.y,
            raw_yaw,
        )
        map_to_odom_x, map_to_odom_y, map_to_odom_yaw = compose_pose_2d(
            map_pose.position.x,
            map_pose.position.y,
            map_yaw,
            inv_raw_x,
            inv_raw_y,
            inv_raw_yaw,
        )

        transform = TransformStamped()
        transform.header.stamp = self._latest_map_odom.header.stamp
        transform.header.frame_id = map_parent
        transform.child_frame_id = raw_parent
        transform.transform.translation.x = map_to_odom_x
        transform.transform.translation.y = map_to_odom_y
        transform.transform.translation.z = 0.0
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion_from_yaw(map_to_odom_yaw)
        self._broadcaster.sendTransform(transform)

        current_frames = (map_parent, raw_parent)
        if current_frames != self._last_frames:
            self._last_frames = current_frames
            self.get_logger().info(
                f"publishing dynamic TF {map_parent} -> {raw_parent}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-odom-topic", default=DEFAULT_MAP_ODOM_TOPIC)
    parser.add_argument("--raw-odom-topic", default=DEFAULT_RAW_ODOM_TOPIC)
    args = parser.parse_args()

    rclpy.init()
    node = MapOdomTfBroadcaster(args.map_odom_topic, args.raw_odom_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
