# RustRobotics Handoff Plan

Last updated: 2026-04-16  
Audience: Copilot, Claude Code, and the next agent taking over this repository  
Status: Phase 4 is no longer "to do". The Gazebo navigation demo and the first real ROS2 navigation stack slices are already merged. This file is now a handoff document, not a design sketch. **2026-04-16:** CI regression on `main` was fixed and pushed; see Section 13.

## 1. Current Repository Snapshot

- Git repo: `rust_robotics/`
- Current branch: `main`
- Remote state at last handoff update: `main` includes the ICP blend-gating merge (ICP blend gating + smoke capture hardening + docs; builds on `9547124`), plus the **2026-04-16 CI hotfix** commit `0caef92` (`fix(ci): stabilize BIT* prune test; use grep in ROS smoke script`).
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
- it waits until specific substring patterns appear in the capture file (matched with `grep -Fq` as of 2026-04-16; see Section 13)
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

**Update (2026-04-16):** CI runners did not have `rg` (ripgrep) installed; `wait_for_log_pattern` and related checks were switched from `rg -q` to `grep -Fq`. Do not rely on `rg` being on `PATH` in GitHub Actions unless the workflow installs it.

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

1. Run `ros2_nodes/launch/run_slam_icp_acceptance_test.sh` as a positive control before tuning. It verifies that a clean synthetic scan-to-scan match reaches `status=icp_ok`, `gate_reason=accepted`, and `blend_applied=true`.
2. Run `ros2_nodes/launch/run_navigation_revaluation_matrix.sh` and compare the generated CSV/JSONL rows:
   - `raw_xy_error`
   - `slam_xy_error`
   - `improvement_xy`
   - `slam_better_xy`
   - `blend_alpha`
   - `gate_reason`
   - `gate_reason_counts`
3. Run `SLAM_REVALUATION_PROFILE_SET=tuning ros2_nodes/launch/run_navigation_revaluation_matrix.sh` when you need a small conservative ICP sweep:
   - `default`
   - `low_alpha` (`SLAM_ICP_BLEND_ALPHA=0.10`)
   - `strict_error` (`SLAM_ICP_REJECT_ERROR=0.011`)
   - `strict_low_alpha` (both overrides)
4. Run `SLAM_REVALUATION_ODOM_PROFILE_SET=biased ros2_nodes/launch/run_navigation_revaluation_matrix.sh` when you need to test whether corrected SLAM can recover from intentionally degraded SLAM input odom while keeping `/ekf_odom` as the raw ground-truth baseline:
   - `raw_realistic`
   - `odom_xy_scale_1pct`
   - `odom_yaw_drift_1deg_per_m`
5. Check whether gating is too conservative:
   - too many `high_error` or `low_motion` rejections may mean corrected mode is effectively disabled
6. Check whether attenuation thresholds are too loose:
   - if bad corrections still slip through, tighten correction/error limits
7. Only after the behavior is more stable, promote the smoke test to enforce a quantitative improvement condition

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

## 12. Recent stack updates (2026)

- **ICP error scale:** `rust_robotics_slam::icp_matching` now exposes `final_error_mean` (and `point_count`) alongside the legacy sum `final_error`. `slam_node` gates and `/slam_diagnostics` use **mean** NN distance \[m/point\], so `SLAM_ICP_FULL_WEIGHT_ERROR` / `SLAM_ICP_REJECT_ERROR` are interpretable and no longer depend on laser point count the same way sum error did.
- **Multi-mission revaluation:** `ros2_nodes/launch/run_navigation_revaluation_matrix.sh` runs several `WAYPOINT_NAV_WAYPOINTS` profiles sequentially with corrected-frame smoke and writes ignored CSV/JSONL reports under `reports/slam_revaluation/` by default. Rows include mission completion, ground-truth XY/yaw error metrics, ICP mean error, blend alpha, and gate/status counts. `SLAM_REVALUATION_PROFILE_SET=tuning` adds a small conservative ICP sweep (`default`, `low_alpha`, `strict_error`, `strict_low_alpha`) for comparing tuning changes without editing the script (see `docs/ros2_integration.md`).
- **ICP reject default tightened:** A 12-run Gazebo tuning matrix (`SLAM_REVALUATION_PROFILE_SET=tuning`, 4 profiles x 3 scenarios) showed the older mean-error reject threshold `0.014` let small attenuated ICP corrections degrade XY by roughly 4-10 mm in the bundled missions. The shipped `DEFAULT_ICP_REJECT_ERROR` is now `0.011`, matching the `strict_error` profile that kept all tested scenarios mission-complete and raw-equivalent in XY by rejecting weak matches.
- **SLAM-input odom bias profiles:** `ros2_nodes/launch/biased_odom_publisher.py` can degrade only `slam_node`'s odom input while leaving `/ekf_odom` as the raw ground-truth baseline. `SLAM_REVALUATION_ODOM_PROFILE_SET=biased` adds `raw_realistic`, `odom_xy_scale_1pct`, and `odom_yaw_drift_1deg_per_m` to the revaluation matrix.
- **Biased odom matrix result:** A 9-run Gazebo matrix with `SLAM_REVALUATION_ODOM_PROFILE_SET=biased` completed all bundled runs. `odom_xy_scale_1pct` degraded corrected SLAM by about 1-2 mm because strict ICP rejected the weak matches and passed biased odom through; `odom_yaw_drift_1deg_per_m` stayed raw-equivalent overall and improved `long_two_legs` by about 2 mm. The harness is useful, but raw-better corrected SLAM still needs a scenario where ICP can be safely accepted.
- **Synthetic ICP acceptance smoke:** `ros2_nodes/launch/run_slam_icp_acceptance_test.sh` starts `slam_node` without Gazebo, feeds deterministic synthetic `/scan` plus `/synthetic_odom`, and requires `/synthetic_slam_diagnostics` to report `status=icp_ok`, `gate_reason=accepted`, and `blend_applied=true`. This is the positive-control check that strict gating can still accept a clean match.
- **Revaluation summaries:** `scripts/summarize_slam_revaluation.py` summarizes one or more generated CSV reports by odom profile, ICP profile, and scenario, including completion counts, `slam_better_xy`, `improvement_xy` average/min/max, scenario winners, and aggregate ICP gate/status counts. It also supports earlier CSV reports without profile columns by treating them as `odom_profile=raw_realistic` and `profile=default`.

## 13. CI regression fix (2026-04-16) — what broke and what changed

This section documents a real `main` breakage and the fix, so the next agent does not rediscover it from scratch.

### 13.1 Symptom on GitHub Actions

Two workflows failed on the same push (example run IDs from 2026-04-15):

- **`.github/workflows/ci.yml` → job `build-test` → step "Run headless tests (no default features)"**  
  - Failure: `batch_informed_rrt_star::tests::test_pruning_reduces_tree_size` panicked.  
  - Message shape: `Pruning should keep tree size reasonable: 189 vs 28` (numbers vary; the assertion compared two **independent** stochastic planner runs).

- **`.github/workflows/ros2-smoke.yml` → job `navigation-smoke` → step "Run headless navigation smoke test"**  
  - Failure: `./ros2_nodes/launch/run_navigation_smoke_test.sh: line 89: rg: command not found` (repeated in a loop).  
  - Root cause: the smoke script used **ripgrep** (`rg`) for `wait_for_log_pattern` and other checks, but **`rg` is not installed by default** on `ubuntu-latest` GitHub runners.

Local developer machines often have `rg` installed; CI did not, so the smoke test passed locally for some people and failed in CI.

### 13.2 Fix (commit `0caef92` on `main`)

**Rust / BIT\* test**

- File: `crates/rust_robotics_planning/src/batch_informed_rrt_star.rs`
- Removed the flaky test that compared tree sizes between two planners using `rand::thread_rng()` (non-deterministic across runners).
- Added a **deterministic** unit test `test_prune_nodes_removes_dominated_leaf` that builds a minimal two-node tree, sets `c_best` so the leaf is prunable under `prune_nodes`, and asserts the leaf is removed.
- Added `#[cfg(test)] impl BatchInformedRRTStar { fn prune_nodes_for_test(...) }` as a thin wrapper around private `prune_nodes` for that test.

**ROS2 smoke script**

- File: `ros2_nodes/launch/run_navigation_smoke_test.sh`
- Replaced `rg -q` with **`grep -Fq`** (fixed-string match) everywhere this script greps log or capture files.
- TF echo checks now use the literal substring `Rotation: in Quaternion (xyzw)` (no regex escapes; `-F` is fixed string).

After push, **CI**, **ROS2 Smoke**, and **Pages** workflows for that commit completed **success** on GitHub.

### 13.3 Implications for future edits

- **Do not reintroduce `rg` as a hard dependency** in `run_navigation_smoke_test.sh` unless the workflow installs ripgrep (e.g. `sudo apt-get install -y ripgrep`). Prefer POSIX `grep` for CI portability.
- **Avoid tests that compare two separate `thread_rng()` planning runs** unless you inject a seeded RNG into the planner (not currently exposed).
- The smoke script still uses **stream capture** and **pattern waits** as in Section 6.2; only the **grep implementation** changed from ripgrep to grep.

### 13.4 Optional follow-ups (not done in `0caef92`)

- **Dependabot:** GitHub may report a low-severity dependency alert on the default branch; review Security / Dependabot separately from this handoff.
- **Node 20 deprecation warnings** on Actions: informational only for now; consider bumping action versions or `FORCE_JAVASCRIPT_ACTIONS_TO_NODE24` when you next touch workflows.

## 14. Handoff for Claude Code / 次エージェント向け（長め）

Read this block first if you are continuing in Claude or another IDE agent.

### 14.1 Repository and workspace

- Treat **`rust_robotics/`** as the only git root for this project in the typical layout. The parent folder **`rust_robo_ws/`** may not be a git repository; clones are often `.../rust_robo_ws/rust_robotics`.
- Remote: `git@github.com:rsasaki0109/rust_robotics.git` (also HTTPS equivalent). Default branch: **`main`**.
- ROS2 nodes under `ros2_nodes/*` are **not** workspace members; build with `--manifest-path` per node (see Section 5.1).
- `safe_drive` is expected as **`../safe_drive`** relative to the repo root when building ROS2 nodes (CI clones it in the sibling workflow step).

### 14.2 What is “done” vs what is “next”

- **Done:** End-to-end Gazebo mission demo, EKF path, DWA, SLAM map publishing, waypoint mission + recovery, RViz topics, headless examples, ROS2 smoke in CI, corrected-frame pipeline, ICP blend gating, mean-ICP diagnostics, revaluation script (Sections 2, 6, 12).
- **Next (product / research):** Corrected SLAM should become **measurably** better than raw odom on agreed metrics (Section 8–9). Smoke tests verify plumbing, not yet strict accuracy thresholds.

### 14.3 CI map (quick reference)

| Workflow file | Purpose |
|----------------|---------|
| `.github/workflows/ci.yml` | Rust: build, test, `--no-default-features` tests, headless examples, clippy, rustdoc `-D warnings`, fmt, cargo-deny, tarpaulin → Codecov, optional semver on PR |
| `.github/workflows/ros2-smoke.yml` | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3 sim, build ROS2 nodes, run `run_navigation_smoke_test.sh` (and corrected-frame variant) |
| `.github/workflows/pages.yml` | `cargo doc` + copy `docs/` and `api/` to GitHub Pages |

If you change `run_navigation_smoke_test.sh`, assume **both** local ROS users and **headless CI** must succeed without extra packages unless you add them to the workflow.

### 14.4 Files to open before touching corrected SLAM or smoke

1. `ros2_nodes/slam_node/src/main.rs` — ICP blend, gating, diagnostics strings  
2. `ros2_nodes/launch/run_navigation_smoke_test.sh` — mission + topic + TF verification (**grep**, not `rg`)  
3. `ros2_nodes/launch/map_odom_tf_broadcaster.py`, `slam_ground_truth_monitor.py` — corrected frame + GT  
4. `docs/ros2_integration.md` — documented behavior  
5. `CLAUDE.md` — short crate layout and `cargo` commands for the **library** workspace

### 14.5 Commands the next agent will likely run

Library workspace (from repo root):

```bash
cargo build --workspace --lib --tests --examples
cargo test --workspace --lib --tests
cargo test -p rust_robotics_planning --no-default-features --lib --tests   # includes BIT* prune test
cargo clippy --workspace --all-targets -- -W clippy::all -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
cargo fmt --all -- --check
```

ROS2 (with Jazzy sourced):

```bash
bash -n ros2_nodes/launch/run_navigation_smoke_test.sh
# full smoke: see Section 5.2
```

### 14.6 Communication note

Previous session (Cursor) fixed **`main` CI** via `0caef92`, verified **green** CI + ROS2 Smoke + Pages on GitHub, and updated **`plan.md`** for handoff. If `main` has moved, run `git log -1` and compare with Section 13.

## 15. One-line summary for Claude

**`main` is CI-green after `0caef92`; BIT\* pruning is tested deterministically; ROS smoke uses `grep -Fq` not `rg`; next priority is corrected-frame accuracy tuning (Sections 8–9), not demo plumbing.**
