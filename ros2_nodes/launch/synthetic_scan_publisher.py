#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import subprocess
import threading
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

DEFAULT_GZ_POSE_TOPIC = "/world/default/dynamic_pose/info"
DEFAULT_ENTITY_NAME = "burger"
DEFAULT_SCAN_TOPIC = "/scan"
DEFAULT_CLOCK_TOPIC = "/clock"
DEFAULT_FRAME_ID = "base_scan"
DEFAULT_UPDATE_RATE_HZ = 5.0
DEFAULT_SAMPLES = 360
DEFAULT_RANGE_MIN = 0.12
DEFAULT_RANGE_MAX = 3.5
LASER_X_OFFSET_M = -0.032

# Approximation of turtlebot3_world/model.sdf obstacles near the smoke routes.
CYLINDERS: tuple[tuple[float, float, float], ...] = tuple(
    (x, y, 0.15)
    for x in (-1.1, 0.0, 1.1)
    for y in (-1.1, 0.0, 1.1)
)

# Coarse outer boundary to keep max-range scans from looking completely empty.
SEGMENTS: tuple[tuple[float, float, float, float], ...] = (
    (-3.5, -3.5, 3.5, -3.5),
    (3.5, -3.5, 3.5, 3.5),
    (3.5, 3.5, -3.5, 3.5),
    (-3.5, 3.5, -3.5, -3.5),
)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_gz_json(message: dict[str, object], entity_name: str) -> Pose2D | None:
    pose_entries = message.get("pose")
    if not isinstance(pose_entries, list):
        return None

    for pose in pose_entries:
        if not isinstance(pose, dict) or pose.get("name") != entity_name:
            continue
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            continue
        return Pose2D(
            x=float(position.get("x", 0.0)),
            y=float(position.get("y", 0.0)),
            yaw=yaw_from_quaternion(
                float(orientation.get("x", 0.0)),
                float(orientation.get("y", 0.0)),
                float(orientation.get("z", 0.0)),
                float(orientation.get("w", 1.0)),
            ),
        )
    return None


def ray_circle_distance(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    cx: float,
    cy: float,
    radius: float,
) -> float | None:
    fx = ox - cx
    fy = oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - radius * radius
    discriminant = b * b - c
    if discriminant < 0.0:
        return None

    root = math.sqrt(discriminant)
    for distance in (-b - root, -b + root):
        if distance >= 0.0:
            return distance
    return None


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def ray_segment_distance(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float | None:
    sx = x2 - x1
    sy = y2 - y1
    denom = cross(dx, dy, sx, sy)
    if abs(denom) < 1e-9:
        return None

    qpx = x1 - ox
    qpy = y1 - oy
    distance = cross(qpx, qpy, sx, sy) / denom
    segment_fraction = cross(qpx, qpy, dx, dy) / denom
    if distance >= 0.0 and 0.0 <= segment_fraction <= 1.0:
        return distance
    return None


class SyntheticScanPublisher(Node):
    def __init__(
        self,
        gz_pose_topic: str,
        entity_name: str,
        scan_topic: str,
        clock_topic: str,
        frame_id: str,
        update_rate_hz: float,
        samples: int,
        range_min: float,
        range_max: float,
    ) -> None:
        super().__init__("synthetic_scan_publisher")
        self._entity_name = entity_name
        self._frame_id = frame_id
        self._update_rate_hz = update_rate_hz
        self._samples = samples
        self._range_min = range_min
        self._range_max = range_max
        self._latest_pose: Pose2D | None = None
        self._latest_clock: tuple[int, int] | None = None
        self._pose_lock = threading.Lock()
        self._stream_should_stop = threading.Event()
        self._gz_process: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None
        self._logged_first_pose = False

        self._publisher = self.create_publisher(
            LaserScan,
            scan_topic,
            10,
        )
        self._clock_subscription = self.create_subscription(
            Clock,
            clock_topic,
            self._clock_callback,
            10,
        )
        self.create_timer(1.0 / update_rate_hz, self._publish_scan)
        self._start_pose_stream(gz_pose_topic)
        self.get_logger().info(
            (
                "publishing synthetic %s from %s entity=%s clock=%s "
                "samples=%d range=[%.2f,%.2f]"
            )
            % (
                scan_topic,
                gz_pose_topic,
                entity_name,
                clock_topic,
                samples,
                range_min,
                range_max,
            )
        )

    def destroy_node(self) -> bool:
        self._stop_pose_stream()
        return super().destroy_node()

    def _start_pose_stream(self, gz_pose_topic: str) -> None:
        cmd = ["gz", "topic", "-e", "-t", gz_pose_topic, "--json-output"]
        self._gz_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._reader_thread = threading.Thread(
            target=self._pose_reader,
            name="synthetic-scan-pose-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _stop_pose_stream(self) -> None:
        self._stream_should_stop.set()
        if self._gz_process is not None and self._gz_process.poll() is None:
            self._gz_process.terminate()
            try:
                self._gz_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._gz_process.kill()
                self._gz_process.wait(timeout=2.0)
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)

    def _pose_reader(self) -> None:
        if self._gz_process is None or self._gz_process.stdout is None:
            return

        for line in self._gz_process.stdout:
            if self._stream_should_stop.is_set():
                return
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                self.get_logger().warn(f"ignoring non-JSON gz pose payload: {line[:120]}")
                continue
            pose = pose_from_gz_json(payload, self._entity_name)
            if pose is None:
                continue
            with self._pose_lock:
                self._latest_pose = pose
            if not self._logged_first_pose:
                self._logged_first_pose = True
                self.get_logger().info(
                    "received first Gazebo pose for %s at x=%.3f y=%.3f yaw=%.3f"
                    % (self._entity_name, pose.x, pose.y, pose.yaw)
                )

    def _clock_callback(self, msg: Clock) -> None:
        self._latest_clock = (msg.clock.sec, msg.clock.nanosec)

    def _raycast(self, ox: float, oy: float, angle: float) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self._range_max - 0.01

        for cx, cy, radius in CYLINDERS:
            distance = ray_circle_distance(ox, oy, dx, dy, cx, cy, radius)
            if distance is not None and self._range_min <= distance < best:
                best = distance

        for x1, y1, x2, y2 in SEGMENTS:
            distance = ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2)
            if distance is not None and self._range_min <= distance < best:
                best = distance

        return max(self._range_min, min(best, self._range_max - 0.01))

    def _publish_scan(self) -> None:
        with self._pose_lock:
            pose = self._latest_pose
        if pose is None:
            return

        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = (angle_max - angle_min) / max(1, self._samples - 1)
        scan_time = 1.0 / self._update_rate_hz

        sensor_x = pose.x + LASER_X_OFFSET_M * math.cos(pose.yaw)
        sensor_y = pose.y + LASER_X_OFFSET_M * math.sin(pose.yaw)
        ranges = [
            float(self._raycast(sensor_x, sensor_y, pose.yaw + angle_min + i * angle_increment))
            for i in range(self._samples)
        ]

        msg = LaserScan()
        if self._latest_clock is None:
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            msg.header.stamp.sec = self._latest_clock[0]
            msg.header.stamp.nanosec = self._latest_clock[1]
        msg.header.frame_id = self._frame_id
        msg.angle_min = float(angle_min)
        msg.angle_max = float(angle_max)
        msg.angle_increment = float(angle_increment)
        msg.time_increment = float(scan_time / max(1, self._samples))
        msg.scan_time = float(scan_time)
        msg.range_min = float(self._range_min)
        msg.range_max = float(self._range_max)
        msg.ranges = ranges
        msg.intensities = []
        self._publisher.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gz-pose-topic", default=DEFAULT_GZ_POSE_TOPIC)
    parser.add_argument("--entity-name", default=DEFAULT_ENTITY_NAME)
    parser.add_argument("--scan-topic", default=DEFAULT_SCAN_TOPIC)
    parser.add_argument("--clock-topic", default=DEFAULT_CLOCK_TOPIC)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--update-rate-hz", type=float, default=DEFAULT_UPDATE_RATE_HZ)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--range-min", type=float, default=DEFAULT_RANGE_MIN)
    parser.add_argument("--range-max", type=float, default=DEFAULT_RANGE_MAX)
    args = parser.parse_args()

    if args.update_rate_hz <= 0.0:
        raise ValueError("--update-rate-hz must be positive")
    if args.samples < 4:
        raise ValueError("--samples must be at least 4")
    if args.range_min <= 0.0 or args.range_max <= args.range_min:
        raise ValueError("--range-max must be greater than --range-min")

    rclpy.init()
    node = SyntheticScanPublisher(
        gz_pose_topic=args.gz_pose_topic,
        entity_name=args.entity_name,
        scan_topic=args.scan_topic,
        clock_topic=args.clock_topic,
        frame_id=args.frame_id,
        update_rate_hz=args.update_rate_hz,
        samples=args.samples,
        range_min=args.range_min,
        range_max=args.range_max,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
