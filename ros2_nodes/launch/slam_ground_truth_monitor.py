#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import subprocess
import threading
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String

DEFAULT_RAW_ODOM_TOPIC = "/ekf_odom"
DEFAULT_SLAM_ODOM_TOPIC = "/slam_odom"
DEFAULT_GZ_POSE_TOPIC = "/world/default/dynamic_pose/info"
DEFAULT_GROUND_TRUTH_ENTITY = "burger"
DEFAULT_STATUS_TOPIC = "/slam_ground_truth_status"
DEFAULT_SYNC_TOLERANCE_SEC = 0.5
PUBLISH_PERIOD_SEC = 0.5
LOG_INTERVAL_SEC = 5.0
ODOM_QOS = QoSPresetProfiles.SENSOR_DATA.value


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


def stamp_to_sec(msg: Odometry) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def stamp_dict_to_sec(stamp: dict[str, object]) -> float:
    sec = float(stamp.get("sec", 0.0))
    nsec = float(stamp.get("nsec", 0.0))
    return sec + nsec * 1e-9


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


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_sec: float


def pose_from_odom(msg: Odometry) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        x=pose.position.x,
        y=pose.position.y,
        yaw=yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        frame_id=msg.header.frame_id or "",
        stamp_sec=stamp_to_sec(msg),
    )


def pose_from_gz_json(
    message: dict[str, object],
    *,
    entity_name: str,
    frame_id: str,
) -> Pose2D | None:
    pose_entries = message.get("pose")
    if not isinstance(pose_entries, list):
        return None

    stamp = message.get("header", {}).get("stamp", {})
    if not isinstance(stamp, dict):
        stamp = {}

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
            frame_id=frame_id,
            stamp_sec=stamp_dict_to_sec(stamp),
        )

    return None


def relative_pose(origin: Pose2D, pose: Pose2D) -> Pose2D:
    inv_x, inv_y, inv_yaw = inverse_pose_2d(origin.x, origin.y, origin.yaw)
    x, y, yaw = compose_pose_2d(inv_x, inv_y, inv_yaw, pose.x, pose.y, pose.yaw)
    return Pose2D(
        x=x,
        y=y,
        yaw=yaw,
        frame_id=pose.frame_id,
        stamp_sec=pose.stamp_sec,
    )


class SlamGroundTruthMonitor(Node):
    def __init__(
        self,
        raw_odom_topic: str,
        slam_odom_topic: str,
        gz_pose_topic: str,
        ground_truth_entity_name: str,
        status_topic: str,
        sync_tolerance_sec: float,
    ) -> None:
        super().__init__("slam_ground_truth_monitor")
        self._sync_tolerance_sec = sync_tolerance_sec
        self._ground_truth_entity_name = ground_truth_entity_name
        self._ground_truth_frame = f"{ground_truth_entity_name}_ground_truth"
        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._latest_raw: Odometry | None = None
        self._latest_slam: Odometry | None = None
        self._latest_ground_truth: Pose2D | None = None
        self._ground_truth_anchor: Pose2D | None = None
        self._last_processed_key: tuple[float, float, float] | None = None
        self._last_status: str | None = None
        self._sample_count = 0
        self._raw_xy_sq_sum = 0.0
        self._slam_xy_sq_sum = 0.0
        self._raw_yaw_sq_sum = 0.0
        self._slam_yaw_sq_sum = 0.0
        self._raw_xy_max = 0.0
        self._slam_xy_max = 0.0
        self._last_log_time = 0.0
        self._seen_sources: set[str] = set()
        self._stream_should_stop = threading.Event()
        self._gz_process: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None

        self.create_subscription(
            Odometry,
            raw_odom_topic,
            self._raw_callback,
            ODOM_QOS,
        )
        self.create_subscription(
            Odometry,
            slam_odom_topic,
            self._slam_callback,
            ODOM_QOS,
        )
        self.create_timer(PUBLISH_PERIOD_SEC, self._publish_status)
        self.get_logger().info(
            "monitoring raw=%s slam=%s gz_pose=%s entity=%s status=%s"
            % (
                raw_odom_topic,
                slam_odom_topic,
                gz_pose_topic,
                ground_truth_entity_name,
                status_topic,
            )
        )
        self._start_ground_truth_stream(gz_pose_topic)

    def destroy_node(self) -> bool:
        self._stop_ground_truth_stream()
        return super().destroy_node()

    def _raw_callback(self, msg: Odometry) -> None:
        self._latest_raw = msg
        self._log_first_ros_message("raw", msg)

    def _slam_callback(self, msg: Odometry) -> None:
        self._latest_slam = msg
        self._log_first_ros_message("slam", msg)

    def _log_first_ros_message(self, source: str, msg: Odometry) -> None:
        if source in self._seen_sources:
            return
        self._seen_sources.add(source)
        stamp_sec = stamp_to_sec(msg)
        self.get_logger().info(
            f"received first {source} odom in frame "
            f"{msg.header.frame_id or 'unknown'} at t={stamp_sec:.3f}"
        )

    def _log_first_ground_truth_message(self, pose: Pose2D) -> None:
        if "ground_truth" in self._seen_sources:
            return
        self._seen_sources.add("ground_truth")
        self.get_logger().info(
            f"received first ground-truth pose for {self._ground_truth_entity_name} "
            f"in frame {pose.frame_id} at t={pose.stamp_sec:.3f}"
        )

    def _start_ground_truth_stream(self, gz_pose_topic: str) -> None:
        cmd = [
            "gz",
            "topic",
            "-e",
            "-t",
            gz_pose_topic,
            "--json-output",
        ]
        self._gz_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._reader_thread = threading.Thread(
            target=self._ground_truth_reader,
            name="slam-ground-truth-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _stop_ground_truth_stream(self) -> None:
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

    def _ground_truth_reader(self) -> None:
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
                self.get_logger().warn(f"ignoring non-JSON ground-truth payload: {line[:120]}")
                continue
            pose = pose_from_gz_json(
                payload,
                entity_name=self._ground_truth_entity_name,
                frame_id=self._ground_truth_frame,
            )
            if pose is None:
                continue
            if self._ground_truth_anchor is None:
                self._ground_truth_anchor = pose
            relative = relative_pose(self._ground_truth_anchor, pose)
            self._latest_ground_truth = relative
            self._log_first_ground_truth_message(relative)

    def _publish_text(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_publisher.publish(msg)

    def _publish_status(self) -> None:
        if (
            self._latest_raw is None
            or self._latest_slam is None
            or self._latest_ground_truth is None
        ):
            status = " ".join(
                [
                    "status=waiting_for_data",
                    f"have_raw={self._latest_raw is not None}",
                    f"have_slam={self._latest_slam is not None}",
                    f"have_gt={self._latest_ground_truth is not None}",
                ]
            )
            self._last_status = status
            self._publish_text(status)
            return

        raw = pose_from_odom(self._latest_raw)
        slam = pose_from_odom(self._latest_slam)
        gt = self._latest_ground_truth

        raw_dt = abs(raw.stamp_sec - gt.stamp_sec)
        slam_dt = abs(slam.stamp_sec - gt.stamp_sec)
        if raw_dt > self._sync_tolerance_sec or slam_dt > self._sync_tolerance_sec:
            status = (
                "status=waiting_for_sync "
                f"raw_dt={raw_dt:.3f} slam_dt={slam_dt:.3f} "
                f"raw_frame={raw.frame_id or 'unknown'} "
                f"slam_frame={slam.frame_id or 'unknown'} "
                f"gt_frame={gt.frame_id or 'unknown'}"
            )
            self._last_status = status
            self._publish_text(status)
            return

        key = (raw.stamp_sec, slam.stamp_sec, gt.stamp_sec)
        if key == self._last_processed_key:
            if self._last_status is not None:
                self._publish_text(self._last_status)
            return

        raw_xy_error = math.hypot(raw.x - gt.x, raw.y - gt.y)
        slam_xy_error = math.hypot(slam.x - gt.x, slam.y - gt.y)
        raw_yaw_error = abs(normalize_angle(raw.yaw - gt.yaw))
        slam_yaw_error = abs(normalize_angle(slam.yaw - gt.yaw))

        self._sample_count += 1
        self._raw_xy_sq_sum += raw_xy_error * raw_xy_error
        self._slam_xy_sq_sum += slam_xy_error * slam_xy_error
        self._raw_yaw_sq_sum += raw_yaw_error * raw_yaw_error
        self._slam_yaw_sq_sum += slam_yaw_error * slam_yaw_error
        self._raw_xy_max = max(self._raw_xy_max, raw_xy_error)
        self._slam_xy_max = max(self._slam_xy_max, slam_xy_error)
        self._last_processed_key = key

        raw_xy_rmse = math.sqrt(self._raw_xy_sq_sum / self._sample_count)
        slam_xy_rmse = math.sqrt(self._slam_xy_sq_sum / self._sample_count)
        raw_yaw_rmse = math.sqrt(self._raw_yaw_sq_sum / self._sample_count)
        slam_yaw_rmse = math.sqrt(self._slam_yaw_sq_sum / self._sample_count)
        improvement_xy = raw_xy_error - slam_xy_error
        improvement_yaw = raw_yaw_error - slam_yaw_error

        status = " ".join(
            [
                "status=ok",
                f"samples={self._sample_count}",
                f"raw_frame={raw.frame_id or 'unknown'}",
                f"slam_frame={slam.frame_id or 'unknown'}",
                f"gt_frame={gt.frame_id or 'unknown'}",
                f"raw_xy_error={raw_xy_error:.3f}",
                f"slam_xy_error={slam_xy_error:.3f}",
                f"raw_yaw_error={raw_yaw_error:.3f}",
                f"slam_yaw_error={slam_yaw_error:.3f}",
                f"raw_xy_rmse={raw_xy_rmse:.3f}",
                f"slam_xy_rmse={slam_xy_rmse:.3f}",
                f"raw_yaw_rmse={raw_yaw_rmse:.3f}",
                f"slam_yaw_rmse={slam_yaw_rmse:.3f}",
                f"raw_xy_max={self._raw_xy_max:.3f}",
                f"slam_xy_max={self._slam_xy_max:.3f}",
                f"improvement_xy={improvement_xy:.3f}",
                f"improvement_yaw={improvement_yaw:.3f}",
                f"raw_dt={raw_dt:.3f}",
                f"slam_dt={slam_dt:.3f}",
                f"slam_better_xy={slam_xy_error <= raw_xy_error}",
                f"slam_better_yaw={slam_yaw_error <= raw_yaw_error}",
            ]
        )
        self._last_status = status
        self._publish_text(status)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_log_time >= LOG_INTERVAL_SEC:
            self._last_log_time = now_sec
            self.get_logger().info(status)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-odom-topic", default=DEFAULT_RAW_ODOM_TOPIC)
    parser.add_argument("--slam-odom-topic", default=DEFAULT_SLAM_ODOM_TOPIC)
    parser.add_argument("--gz-pose-topic", default=DEFAULT_GZ_POSE_TOPIC)
    parser.add_argument("--ground-truth-entity-name", default=DEFAULT_GROUND_TRUTH_ENTITY)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--sync-tolerance-sec", type=float, default=DEFAULT_SYNC_TOLERANCE_SEC)
    args = parser.parse_args()

    rclpy.init()
    node = SlamGroundTruthMonitor(
        raw_odom_topic=args.raw_odom_topic,
        slam_odom_topic=args.slam_odom_topic,
        gz_pose_topic=args.gz_pose_topic,
        ground_truth_entity_name=args.ground_truth_entity_name,
        status_topic=args.status_topic,
        sync_tolerance_sec=args.sync_tolerance_sec,
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
