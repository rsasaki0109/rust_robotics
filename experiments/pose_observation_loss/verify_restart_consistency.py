"""Independently reconstruct stationary correction windows from logged observations."""

import math
from collections import deque


def average(poses):
    return (
        sum(p[0] for p in poses) / len(poses),
        sum(p[1] for p in poses) / len(poses),
        math.atan2(
            sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses)
        ),
    )


def check_restart_track(track, enabled):
    window = deque(maxlen=25)
    pre = None
    epoch = None
    evidence = 0.0
    for before, row in zip(track, track[1:]):
        observed = tuple(
            float(before[axis])
            + float(row[f"noise_{axis}"])
            + float(row[f"bias_{axis}"])
            for axis in ["x", "y", "yaw"]
        )
        active = before["guard_active"] == "true"
        t = float(row["time_s"])
        stationary = abs(float(before["v"])) <= 0.02
        precision = row["precision_activation"] == "true"
        if enabled:
            if not active or not stationary or precision:
                window.clear()
                pre, epoch = None, None
            elif row["correction"] == "true":
                received = row["correction_received"] == "true"
                pre = average(window) if received and len(window) == 25 else None
                epoch = t if received else None
                window.clear()
            if active and stationary and not precision:
                window.append(observed)
        age = t - epoch if epoch is not None else -1.0
        position, yaw = -1.0, -1.0
        if pre is not None and window:
            post = average(window)
            position = math.hypot(post[0] - pre[0], post[1] - pre[1])
            yaw = abs(
                math.atan2(math.sin(post[2] - pre[2]), math.cos(post[2] - pre[2]))
            )
        ready = (
            pre is not None
            and len(window) >= 10
            and age <= 1.5
            and position <= 0.2
            and yaw <= math.radians(5)
        )
        assert int(row["pre_sample_count"]) == (25 if pre is not None else 0)
        assert int(row["stationary_sample_count"]) == len(window)
        for name, value in [
            ("consistency_age", age),
            ("position_delta", position),
            ("yaw_delta_rad", yaw),
        ]:
            assert abs(float(row[name]) - value) < 4e-9, (name, t, value, row[name])
        assert (row["consistency_ready"] == "true") == ready
        pose = average(window) if ready else observed
        for axis, value in zip(["x", "y", "yaw"], pose):
            delta = float(row[f"preview_pose_{axis}"]) - value
            if axis == "yaw":
                delta = math.atan2(math.sin(delta), math.cos(delta))
            assert abs(delta) < 3e-9
        if enabled:
            margin = float(row["preview_margin"])
            release_ready = (
                active
                and ready
                and stationary
                and float(row["preview_clearance_lower_bound"]) > margin + 0.05
            )
            evidence = evidence + 0.02 if release_ready else 0.0
            released = evidence + 1e-9 >= 0.5
            assert (row["guard_released"] == "true") == released
            if released:
                assert row["correction_received"] == "false" and 0 <= age <= 1.5
                evidence = 0.0
            assert abs(float(row["guard_release_evidence"]) - evidence) < 1e-8
