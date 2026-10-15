"""Reconstruct motion compensation with independent odometry-frame anchors."""

from collections import deque
import math


def check_motion_filter_track(track, enabled):
    if not enabled:
        assert all(int(row.get("motion_filter_sample_count", 0)) == 0 for row in track)
        return
    assert int(track[0]["motion_filter_sample_count"]) == 0
    anchors = deque(maxlen=25)
    ox = oy = heading = 0.0
    previous = None
    for before, row in zip(track, track[1:]):
        if previous is not None:
            velocity, steering = previous
            distance = velocity * 0.02
            ox += distance * math.cos(heading)
            oy += distance * math.sin(heading)
            heading += distance / 0.4 * math.tan(steering)
        if row["correction"] == "true" or row["precision_activation"] == "true":
            anchors.clear()
        x, y, yaw = (
            float(before[axis])
            + float(row[f"noise_{axis}"])
            + float(row[f"bias_{axis}"])
            for axis in ["x", "y", "yaw"]
        )
        angle = yaw - heading
        sine, cosine = math.sin(angle), math.cos(angle)
        anchors.append(
            (x - cosine * ox + sine * oy, y - sine * ox - cosine * oy, sine, cosine)
        )
        n = len(anchors)
        ax, ay, sn, cs = (sum(p[i] for p in anchors) / n for i in range(4))
        expected = (
            ax + cs * ox - sn * oy,
            ay + sn * ox + cs * oy,
            math.atan2(
                sn * math.cos(heading) + cs * math.sin(heading),
                cs * math.cos(heading) - sn * math.sin(heading),
            ),
        )
        assert int(row["motion_filter_sample_count"]) == n
        for axis, value in zip(["x", "y", "yaw"], expected):
            error = float(row[f"preview_pose_{axis}"]) - value
            if axis == "yaw":
                error = math.atan2(math.sin(error), math.cos(error))
            assert abs(error) < 1e-8, (axis, row["time_s"], error)
        previous = float(before["v"]), float(row["steering"])
