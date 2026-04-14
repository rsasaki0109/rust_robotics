# RustRobotics Handoff Plan

Last updated: 2026-04-15  
Audience: Copilot / next agent taking over this repository  
Status: Phase 4 is no longer "to do". The Gazebo navigation demo and the first real ROS2 navigation stack slices are already merged. This file is now a handoff document, not a design sketch.

## 1. Current Repository Snapshot

- Git repo: `rust_robotics/`
- Current branch: `main`
- Remote state at last handoff update: `main` includes the ICP blend-gating merge (ICP blend gating + smoke capture hardening + docs; builds on `9547124`).
- Worktree state: clean after that merge unless you have new local edits.

This repo lives inside a larger directory that is not itself a git repo:

```text
rust_robo_ws/
├── rust_robotics/          # THIS git repo
├── safe_drive/             # sibling repo, used by ros2_nodes/*
├── safe_drive_msg/         # sibling repo
├── safe_drive_tutorial/    # sibling repo
├── ekf_localizer_ros2/     # sibling repo, not wired into current demo
└── notes/
```

## 2. What Is Already Done

The original "Phase 4 Gazebo navigation demo" plan is complete and significantly exceeded. The stack now includes:

- Gazebo TurtleBot3 mission demo
- A* global path planning from live `/map`
- DWA local planner outputting `/cmd_vel`
- Rust SLAM node producing occupancy grid maps
- EKF-backed odometry path for navigation consumers
- multi-waypoint mission execution
- recovery state machine for stuck missions
- RViz observability topics and mission markers
- ROS2 smoke testing, including headless CI coverage
- optional corrected SLAM frame pipeline
- SLAM diagnostics and Gazebo ground-truth comparison

The most important merged milestones on `main` are:

- `5762e43` `[codex] Add Gazebo navigation demo integration (#28)`
  - established the end-to-end Gazebo demo
- `c049766` `[codex] Add EKF-backed mission odometry stack (#29)`
  - added EKF odom path, waypoint mission execution, and follow-up fixes
- `578c7c6` `Add mission recovery state machine`
  - introduced stuck detection and `Running / Recovering / Failed / Completed`
- `7980b14` `Add recovery transition regression tests`
  - locked recovery transitions with unit tests
- `c7c70ab` `Add headless mission recovery regression`
  - added a headless example for recovery behavior
- `46145cc` `Add RViz mission observability slice`
  - added `/mission_status`, `/mission_markers`, RViz config, and observability wiring
- `09354de` `Add ROS2 navigation smoke test`
  - added local launch smoke automation
- `d028846` `Add headless ROS2 smoke workflow`
  - brought ROS2 smoke into CI
- `dce2517` `Add dynamic nav TF broadcaster`
  - added dynamic odom-to-base TF publishing
- `6fa8d6a` `Make ROS2 demo frame semantics honest`
  - stopped lying about `map` when data was really odom-framed
- `b5bacd8` `Add corrected SLAM frame pipeline`
  - added opt-in corrected SLAM frame mode with `/slam_pose`, `/slam_odom`, `map -> odom`
- `9547124` `Add SLAM diagnostics and ground-truth monitor`
  - added `/slam_diagnostics` and Gazebo ground-truth comparison via `gz topic`
- ICP blend gating merge: `Add ICP blend gating for corrected SLAM and harden navigation smoke test`
  - ICP quality gate / attenuation for corrected mode; `blend_alpha` and `gate_reason` in diagnostics
  - smoke test uses stream capture instead of brittle `ros2 topic echo --once`
  - README and `docs/ros2_integration.md` updated for the above

## 3. Current Navigation Stack Architecture

### 3.1 Normal mission mode

The default stack is now honest about frames. Unless corrected SLAM mode is explicitly enabled:

- global frame is `odom`
- `/map` is published in `odom`
- `/planned_path` is published in `odom`
- `/goal_pose` and `/mission_markers` are published in `odom`
- a dynamic TF is broadcast from odom frame to robot base frame
- `map -> odom` is not required in this default mode

Primary nodes and roles:

- `ros2_nodes/slam_node`
  - subscribes to raw scan + odom
  - publishes occupancy grid map
  - optionally publishes corrected SLAM pose/odom
- `ros2_nodes/path_planner_node`
  - consumes `/map`, odom, and `/goal_pose`
  - snaps invalid start/goal cells to nearby free cells
  - publishes `/planned_path`
- `ros2_nodes/dwa_planner_node`
  - consumes scan, odom, planned path
  - publishes `/cmd_vel`
  - publishes stop command after path clear
- `ros2_nodes/ekf_localizer_node`
  - filters raw `/odom`
  - publishes `/ekf_odom` and `/ekf_pose`
- `ros2_nodes/waypoint_navigator_node`
  - drives the mission state machine
  - emits `/goal_pose`
  - publishes `/mission_status`
  - publishes `/mission_markers`
  - handles stuck detection and recovery

Key launch wrappers:

- `ros2_nodes/launch/run_gazebo_demo.sh`
- `ros2_nodes/launch/run_gazebo_mission_demo.sh`
- `ros2_nodes/launch/run_navigation_smoke_test.sh`

### 3.2 Corrected SLAM frame mode

Corrected mode is opt-in:

```bash
ENABLE_SLAM_CORRECTED_FRAME=true ./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

In corrected mode:

- navigation odom topic switches to `/slam_odom`
- global frame switches to `map`
- `slam_node` publishes `/slam_pose` and `/slam_odom`
- `map_odom_tf_broadcaster.py` estimates dynamic `map -> odom`
- `slam_ground_truth_monitor.py` compares `/ekf_odom` and `/slam_odom` against Gazebo ground truth

This is working as a pipeline, but not yet proven to be consistently more accurate than raw odom. That is the most important remaining technical problem.

## 4. Observability and Regression Infrastructure

The repo now has several layers of observability:

- `/mission_status`
  - current mission state, active waypoint, recovery phase
- `/mission_markers`
  - route markers, active goal, status overlay
- `/slam_diagnostics`
  - per-scan SLAM / ICP status
- `/slam_ground_truth_status`
  - raw-vs-corrected odom error relative to Gazebo ground truth
- RViz config:
  - `ros2_nodes/launch/navigation_demo.rviz`
- Local smoke:
  - `ros2_nodes/launch/run_navigation_smoke_test.sh`
- CI smoke:
  - `.github/workflows/ros2-smoke.yml`

There is also a non-ROS headless regression path in the umbrella crate:

- `crates/rust_robotics/examples/headless_mission_recovery.rs`

That example verifies recovery behavior without requiring the full ROS2 launch stack.

## 5. Important Environment and Build Notes

### 5.1 `ros2_nodes/*` are standalone Cargo packages

They are not members of the root Rust workspace. Build them individually:

```bash
source /opt/ros/jazzy/setup.bash
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/ekf_localizer_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/waypoint_navigator_node/Cargo.toml
```

They depend on sibling checkout `../safe_drive`, so a missing or broken `safe_drive` repo will break ROS2 node builds.

### 5.2 Typical demo commands

Regular mission demo:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=burger
./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

Corrected frame demo:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=burger
export ENABLE_GAZEBO_GUI=false
export ENABLE_RVIZ=false
export ENABLE_SLAM_CORRECTED_FRAME=true
./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

Smoke test:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=89
export TURTLEBOT3_MODEL=burger
export ENABLE_RVIZ=false
./ros2_nodes/launch/run_navigation_smoke_test.sh
```

Corrected-frame smoke:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=98
export TURTLEBOT3_MODEL=burger
export ENABLE_RVIZ=false
export ENABLE_GAZEBO_GUI=false
export ENABLE_SLAM_CORRECTED_FRAME=true
./ros2_nodes/launch/run_navigation_smoke_test.sh
```

## 6. Recently Merged: ICP Quality Gating and Smoke Stability

The following landed in the ICP blend-gating merge (was previously local WIP on top of `9547124`).

### 6.1 File: `ros2_nodes/slam_node/src/main.rs`

This file includes an ICP quality-gate / attenuation path for corrected SLAM mode.

Intent:

- stop blindly mixing ICP into corrected pose whenever ICP returns a result
- reject obviously bad corrections
- attenuate borderline corrections
- expose the gating decision in diagnostics

Core additions:

- new gating thresholds:
  - `ICP_FULL_WEIGHT_ERROR`
  - `ICP_REJECT_ERROR`
  - `ICP_FULL_WEIGHT_ITERATIONS`
  - `ICP_REJECT_ITERATIONS`
  - `ICP_FULL_WEIGHT_TRANSLATION_CORRECTION`
  - `ICP_FULL_WEIGHT_YAW_CORRECTION`
  - `ICP_FULL_WEIGHT_TRANSLATION_MOTION`
  - `ICP_FULL_WEIGHT_YAW_MOTION`
- new `IcpBlendDecision { alpha, reason }`
- new `compute_icp_blend_decision(...)`
- `blend_motion_delta(...)` now takes explicit `alpha`
- diagnostics now include:
  - `blend_alpha`
  - `gate_reason`

Behavior after merge:

- if ICP is not converged, invalid, or clearly out-of-family, corrected mode falls back to raw odom
- if ICP quality is borderline, the blend factor is reduced below the base `ICP_BLEND_ALPHA`
- if ICP is good, full blend is still allowed

The status strings now distinguish:

- `icp_ok`
- `icp_attenuated`
- `icp_rejected`
- `odom_only`
- `bootstrap`

Unit tests included in the merge:

- reject high error
- attenuate low motion
- accept clean match
- diagnostics formatting includes `blend_alpha` and `gate_reason`

### 6.2 File: `ros2_nodes/launch/run_navigation_smoke_test.sh`

This file includes an anti-flake rewrite of topic capture (same merge as section 6).

Problem that existed before:

- `ros2 topic echo --once` was racy for transient status topics
- long `std_msgs/String` payloads could get truncated with `...`
- the smoke test could fail even when the mission and observability stack actually worked

Fix applied in the merge:

- added `capture_topic_stream_until()`
- this runs a background subscriber with:
  - `ros2 topic echo --full-length`
- it waits until specific regex patterns appear in the capture file
- it replaced brittle `--once` capture for:
  - `/mission_status`
  - `/slam_diagnostics`
  - `/mission_markers`
  - `/slam_ground_truth_status`

This matters because the corrected-frame validation depends on being able to reliably read:

- `blend_alpha=...`
- `gate_reason=...`
- `status=completed`
- `slam_xy_error=...`

### 6.3 Files: `README.md`, `docs/ros2_integration.md`

Docs were updated in the same merge to match the quality-gated corrected frame.

The key doc deltas are:

- corrected mode now explicitly says:
  - high-error, low-motion, or outlier ICP falls back to odom
  - medium-quality ICP is attenuated
- `/slam_diagnostics` description now includes:
  - `blend_alpha`
  - `gate_reason`

## 7. Validation Already Performed Before Merge

The changes above were exercised locally before merge.

Executed successfully:

```bash
cargo test --manifest-path ros2_nodes/slam_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
bash -n ros2_nodes/launch/run_navigation_smoke_test.sh
```

Corrected-frame smoke also passed locally after the capture rewrite:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87
export TURTLEBOT3_MODEL=burger
export ENABLE_RVIZ=false
export ENABLE_GAZEBO_GUI=false
export ENABLE_SLAM_CORRECTED_FRAME=true
./ros2_nodes/launch/run_navigation_smoke_test.sh
```

and again on a separate domain:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=86
export TURTLEBOT3_MODEL=burger
export ENABLE_RVIZ=false
export ENABLE_GAZEBO_GUI=false
export ENABLE_SLAM_CORRECTED_FRAME=true
./ros2_nodes/launch/run_navigation_smoke_test.sh
```

Observed outcome:

- mission completed
- observability topics were captured
- TF checks passed
- `blend_alpha` and `gate_reason` appeared in `/slam_diagnostics`
- `slam_ground_truth_status` was captured without the old truncation/race failure

## 8. Known Technical Gaps

This is the part Copilot should treat as the real forward plan.

### 8.1 The corrected SLAM frame is not yet clearly better than raw odom

The pipeline works, but the accuracy story is not done.

In successful smoke runs, the corrected estimate has sometimes been only comparable to, or slightly worse than, raw odom in XY error. Example outcomes have looked like:

- `raw_xy_error=0.024 slam_xy_error=0.035`
- `raw_xy_error=0.028 slam_xy_error=0.029`

That means:

- the plumbing is working
- diagnostics are available
- ground-truth comparison is available
- but the correction policy still needs tuning before it can be called trustworthy

### 8.2 Success criteria are not yet quantitative enough

The smoke test currently verifies:

- mission completes
- observability topics exist
- TF exists
- corrected mode publishes diagnostics and ground-truth status

It does not yet enforce a quantitative condition like:

- corrected SLAM must beat raw odom in XY RMSE
- corrected SLAM must not exceed some drift threshold
- a minimum fraction of scans must be accepted instead of rejected

### 8.3 Diagnostics are still string-based

`/slam_diagnostics` and `/slam_ground_truth_status` are still `std_msgs/String`.

That is acceptable for fast iteration, but eventually:

- either a structured custom message
- or `diagnostic_msgs`

would be a better interface for downstream tooling.

## 9. Recommended Next Steps

### Immediate next step

Tune corrected-frame quality using the new diagnostics and ground truth.

Recommended order:

1. Gather several corrected-frame smoke runs and record:
   - `raw_xy_error`
   - `slam_xy_error`
   - `blend_alpha`
   - `gate_reason`
2. Check whether gating is too conservative:
   - too many `high_error` or `low_motion` rejections may mean corrected mode is effectively disabled
3. Check whether attenuation thresholds are too loose:
   - if bad corrections still slip through, tighten correction/error limits
4. Only after the behavior is more stable, promote the smoke test to enforce a quantitative improvement condition

### After the quality story is good enough

Then the next clean follow-ups are:

1. move diagnostics away from ad hoc strings
2. consider making quality thresholds configurable via env
3. add a longer corrected-frame regression scenario
4. decide whether corrected mode should remain experimental or become the default

## 10. Practical Advice For The Next Agent

- Do not restart from the old "Phase 4 TODO" mindset. That document is obsolete.
- Assume the stack already works end-to-end and focus on the corrected-frame accuracy problem.
- Treat `run_navigation_smoke_test.sh` as part of the product, not just a throwaway script. Its stability directly affects how credible the corrected-mode work is.
- When evaluating corrected SLAM, prefer repeated headless smoke runs over one-off Gazebo GUI impressions.
- Keep frame semantics honest. This repo already spent time unwinding misleading `map` labeling; do not reintroduce that.
- Before changing anything in corrected mode, read:
  - `ros2_nodes/slam_node/src/main.rs`
  - `ros2_nodes/launch/map_odom_tf_broadcaster.py`
  - `ros2_nodes/launch/slam_ground_truth_monitor.py`
  - `ros2_nodes/launch/run_navigation_smoke_test.sh`
  - `docs/ros2_integration.md`

## 11. Short Version

If you only remember five things, remember these:

1. The Gazebo navigation demo is done. This is no longer a "build the demo" repo state.
2. `main` through `9547124` already had mission stack, recovery, observability, corrected-frame pipeline, and ground-truth monitoring; the ICP blend-gating merge adds ICP gating and smoke stability.
3. The central remaining technical problem is not wiring, but whether corrected SLAM is measurably better than raw odom.
4. The next good move is: tune corrected mode against ground-truth metrics, then consider quantitative smoke thresholds.
5. Treat `run_navigation_smoke_test.sh` as part of the product; its stability defines how credible corrected-mode tuning is.
