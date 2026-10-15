"""Reconstruct the joint admission timer and atomic loss/guard releases."""

import math


def release_margin(row):
    return (
        float(row["guard_release_margin"])
        if "guard_release_margin" in row
        else float(row["preview_margin"]) + 0.05
    )


def check_guard_track(track):
    """Reconstruct every guard transition, including the shared loss release."""
    active = False
    evidence = 0.0
    for before, row in zip(track, track[1:]):
        pending = before["safety_paused"] == "true" or (
            row["correction"] == "true" and row["correction_received"] == "false"
        )
        lower, margin = float(row["preview_clearance_lower_bound"]), float(
            row["preview_margin"]
        )
        triggered = released = False
        if not active and lower <= margin:
            active, triggered, evidence = True, True, 0.0
        elif active and not pending:
            ready = abs(float(before["v"])) <= 0.02 and lower > release_margin(row)
            evidence = evidence + 0.02 if ready else 0.0
            if evidence + 1e-9 >= 0.5:
                active, released, evidence = False, True, 0.0
        if pending:
            evidence = 0.0
            if row["resumed"] == "true":
                released, active = active, False
        assert (row["guard_active"] == "true") == active
        assert (row["guard_triggered"] == "true") == triggered
        assert (row["guard_released"] == "true") == released
        assert abs(float(row["guard_release_evidence"]) - evidence) < 1e-8


def check_joint_track(track):
    evidence = 0.0
    guarded = False
    for before, row in zip(track, track[1:]):
        missing = row["correction"] == "true" and row["correction_received"] == "false"
        pending = before["safety_paused"] == "true" or missing
        margin = (
            0.6
            + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
            + 0.02 * float(row["observation_age"])
        )
        x, y, yaw = (float(row[f"recovery_{k}"]) for k in ["x", "y", "yaw"])
        bounds = all(
            margin <= p <= 50 - margin
            for p in [x, y, x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)]
        )
        if pending:
            # Reconstruct the post-release goal command, including the five-second
            # recovery cap. Do not infer readiness from the output latch states.
            goal_command = (
                0.0
                if row["stop_latched"] == "true"
                else min(
                    0.4,
                    max(0.1, 0.5 * float(row["sensed_goal_distance"])),
                    0.2 if row["restart_limited"] == "true" else 1.0,
                )
            )
            assert abs(goal_command - float(row["uncapped_target_speed"])) < 2e-9
            ready = (
                float(row["observation_age"]) <= 1.5
                and abs(float(before["v"])) <= 0.02
                and int(row["recovery_sample_count"]) >= 10
                and float(row["recovery_clearance"]) > margin
                and bounds
                and float(row["nominal_target_speed"]) > 0
                and float(row["preview_clearance_lower_bound"]) > release_margin(row)
            )
            evidence = evidence + 0.02 if ready else 0.0
            release = evidence + 1e-9 >= 1
            assert (row["resumed"] == "true") == release
            assert (row["safety_paused"] == "true") == (not release)
            assert (row["guard_released"] == "true") == (release and guarded)
            if release:
                assert row["guard_active"] == "false"
                assert row["target_speed"] == row["nominal_target_speed"]
                assert row["steering"] == row["nominal_steering"]
                evidence = 0.0
            assert float(row["guard_release_evidence"]) == 0
        else:
            assert row["resumed"] == "false" and row["safety_paused"] == "false"
        assert abs(float(row["recovery_evidence"]) - evidence) < 1e-8
        guarded = row["guard_active"] == "true"
