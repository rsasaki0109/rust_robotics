"""Reconstruct motion compensation with independent odometry-frame anchors."""

from collections import deque
import math


def check_epoch_pair(reference, candidate):
    """Before the first available fusion position, applied control is identical."""
    bookkeeping = {
        "stationary_epoch_fusion",
        "epoch_fusion_available",
        "epoch_fusion_group_count",
    }
    for before, after in zip(reference, candidate):
        if after["epoch_fusion_available"] == "true":
            return
        assert all(before[k] == after[k] for k in before if k not in bookkeeping)
    assert len(reference) == len(candidate)


def check_motion_filter_track(track, enabled, epoch_enabled=False):
    assert not epoch_enabled or enabled
    if epoch_enabled or "stationary_epoch_fusion" in track[0]:
        assert track[0]["stationary_epoch_fusion"] == "false"
        assert track[0]["epoch_fusion_available"] == "false"
        assert int(track[0]["epoch_fusion_group_count"]) == 0
        for row in track[1:]:
            assert (row["stationary_epoch_fusion"] == "true") == epoch_enabled
            if not epoch_enabled:
                assert row["epoch_fusion_available"] == "false"
                assert int(row["epoch_fusion_group_count"]) == 0
    if not enabled:
        assert all(int(row.get("motion_filter_sample_count", 0)) == 0 for row in track)
        return
    # Import after the geometry modules have initialized: the frozen prototype
    # also imports their offline evaluators. Its estimator API accepts sensors
    # and applied commands only; truth below just reconstructs those sensors.
    if epoch_enabled:
        from analyze_stationary_epoch_fusion import StationaryEpochFusion

        fusion = StationaryEpochFusion()
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
        for axis, value in zip(["x", "y"], expected[:2]):
            field = f"motion_filter_pose_{axis}"
            if epoch_enabled or field in row:
                assert abs(float(row[field]) - value) < 1e-8
        if epoch_enabled:
            fusion.observe(
                float(row["time_s"]),
                (x, y, yaw),
                float(before["v"]),
                float(row["steering"]),
                row["correction_received"] == "true",
                row["precision_activation"] == "true",
                float(row["observation_age"]),
            )
            position = fusion.position()
            assert (row["epoch_fusion_available"] == "true") == (position is not None)
            assert int(row["epoch_fusion_group_count"]) == len(fusion.groups)
            if position is not None:
                expected = (*position, expected[2])
        for axis, value in zip(["x", "y", "yaw"], expected):
            error = float(row[f"preview_pose_{axis}"]) - value
            if axis == "yaw":
                error = math.atan2(math.sin(error), math.cos(error))
            assert abs(error) < 1e-8, (axis, row["time_s"], error)
        previous = float(before["v"]), float(row["steering"])
