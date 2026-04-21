#!/usr/bin/env python3

import argparse
import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

DEFAULT_INPUT_TOPIC = "/ekf_odom"
DEFAULT_OUTPUT_TOPIC = "/slam_input_odom"
ODOM_QOS = 10


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    return angle


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
    yaw = normalize_angle(left_yaw + right_yaw)
    return x, y, yaw


class BiasedOdomPublisher(Node):
    def __init__(
        self,
        *,
        input_topic: str,
        output_topic: str,
        xy_scale: float,
        yaw_scale: float,
        yaw_bias_rad: float,
        yaw_bias_per_meter: float,
        linear_scale: float,
        angular_scale: float,
    ) -> None:
        super().__init__("biased_odom_publisher")
        self._xy_scale = xy_scale
        self._yaw_scale = yaw_scale
        self._yaw_bias_rad = yaw_bias_rad
        self._yaw_bias_per_meter = yaw_bias_per_meter
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._anchor: tuple[float, float, float] | None = None
        self._publisher = self.create_publisher(Odometry, output_topic, ODOM_QOS)
        self._subscriber = self.create_subscription(
            Odometry,
            input_topic,
            self._odom_callback,
            ODOM_QOS,
        )
        self.get_logger().info(
            "biasing odom "
            f"{input_topic} -> {output_topic}: "
            f"xy_scale={xy_scale:.6f} yaw_scale={yaw_scale:.6f} "
            f"yaw_bias_rad={yaw_bias_rad:.6f} "
            f"yaw_bias_per_meter={yaw_bias_per_meter:.6f} "
            f"linear_scale={linear_scale:.6f} angular_scale={angular_scale:.6f}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        raw_x = pose.position.x
        raw_y = pose.position.y
        raw_yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        if self._anchor is None:
            self._anchor = (raw_x, raw_y, raw_yaw)
            self.get_logger().info(
                f"anchored odom bias at x={raw_x:.3f} y={raw_y:.3f} yaw={raw_yaw:.3f}"
            )

        anchor_x, anchor_y, anchor_yaw = self._anchor
        inv_anchor = inverse_pose_2d(anchor_x, anchor_y, anchor_yaw)
        rel_x, rel_y, rel_yaw = compose_pose_2d(
            inv_anchor[0],
            inv_anchor[1],
            inv_anchor[2],
            raw_x,
            raw_y,
            raw_yaw,
        )
        rel_distance = math.hypot(rel_x, rel_y)
        biased_rel_yaw = normalize_angle(
            rel_yaw * self._yaw_scale
            + self._yaw_bias_rad
            + self._yaw_bias_per_meter * rel_distance
        )
        biased_x, biased_y, biased_yaw = compose_pose_2d(
            anchor_x,
            anchor_y,
            anchor_yaw,
            rel_x * self._xy_scale,
            rel_y * self._xy_scale,
            biased_rel_yaw,
        )

        out = copy.deepcopy(msg)
        out.pose.pose.position.x = biased_x
        out.pose.pose.position.y = biased_y
        (
            out.pose.pose.orientation.x,
            out.pose.pose.orientation.y,
            out.pose.pose.orientation.z,
            out.pose.pose.orientation.w,
        ) = quaternion_from_yaw(biased_yaw)
        out.twist.twist.linear.x *= self._linear_scale
        out.twist.twist.linear.y *= self._linear_scale
        out.twist.twist.angular.z *= self._angular_scale
        self._publisher.publish(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC)
    parser.add_argument("--output-topic", default=DEFAULT_OUTPUT_TOPIC)
    parser.add_argument("--xy-scale", type=float, default=1.0)
    parser.add_argument("--yaw-scale", type=float, default=1.0)
    parser.add_argument("--yaw-bias-rad", type=float, default=0.0)
    parser.add_argument("--yaw-bias-per-meter", type=float, default=0.0)
    parser.add_argument("--linear-scale", type=float, default=None)
    parser.add_argument("--angular-scale", type=float, default=None)
    args = parser.parse_args()

    linear_scale = args.xy_scale if args.linear_scale is None else args.linear_scale
    angular_scale = args.yaw_scale if args.angular_scale is None else args.angular_scale

    rclpy.init()
    node = BiasedOdomPublisher(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        xy_scale=args.xy_scale,
        yaw_scale=args.yaw_scale,
        yaw_bias_rad=args.yaw_bias_rad,
        yaw_bias_per_meter=args.yaw_bias_per_meter,
        linear_scale=linear_scale,
        angular_scale=angular_scale,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
