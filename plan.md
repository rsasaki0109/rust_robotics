# RustRobotics Development Plan

## Project Overview

RustRobotics is a Rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics). The workspace is split across planning, localization, control, mapping, SLAM, visualization, and shared core crates.

Repository: `rsasaki0109/rust_robotics`  
Current branch snapshot: `feature/more-algorithms-wave4`  
Plan status: updated to reflect the local branch state as of `2026-04-11`

---

## What This File Is For

This file is no longer a Wave 3 task sheet.

At this point, `plan.md` is the branch-level status document for:

1. completed algorithm waves that are already in the source tree,
2. active experiment tracks that are driving current design decisions,
3. the stable interfaces that have survived those experiments,
4. the next practical priorities.

Detailed per-preset measurements live in `docs/experiments*.md`, `docs/decisions*.md`, and `docs/interfaces.md`. This file summarizes those results so the current branch can be understood from one place.

---

## Current Branch Snapshot

### Completed and stabilized work

- Wave 1 + Wave 2 implementations are complete.
- Wave 3 implementations are complete.
- The branch now also contains additional planning modules beyond the original Wave 3 scope.
- The branch now contains drone trajectory comparison infrastructure that promotes only a small stable helper surface while keeping concrete generator variants experimental.
- The branch now contains a feature-gated, example-scoped `dora-rs` planning demo that has been validated locally with `dora-cli 0.3.13` and now exchanges a structured JSON path report between the planner node and sink.

### Actively experimental work

- planning runtime aggregation strategies for MovingAI crossover buckets
- `A* / Fringe Search / IDA*` threshold-budget comparison on local and larger windows
- drone trajectory generator trade-offs across stop-go, pass-through, acceleration, jerk-enriched, and coupled-continuity waypoint boundary presets

### Current branch reality

- The branch has advanced significantly beyond the previous `main`-based Wave 3 plan.
- The current plan must therefore describe both stable algorithm additions and experimental comparison work.
- The branch contains many uncommitted changes, so this file should be read as a local branch status snapshot rather than a released changelog.

---

## Completed Implementation Waves

### Wave 1 + Wave 2

The earlier implementation waves added `14` algorithms across control, mapping, localization, and planning.

| # | Algorithm | Crate | Status |
|---|-----------|-------|--------|
| 1 | PID Controller | control | complete |
| 2 | DBSCAN Clustering | mapping | complete |
| 3 | Complementary Filter | localization | complete |
| 4 | Bidirectional RRT | planning | complete |
| 5 | Tangent Bug | planning | complete |
| 6 | Line Extraction (Split-and-Merge) | mapping | complete |
| 7 | Iterated EKF (IEKF) | localization | complete |
| 8 | RRG | planning | complete |
| 9 | Occupancy Grid Map (Log-Odds) | mapping | complete |
| 10 | Information Filter | localization | complete |
| 11 | Sliding Mode Control | control | complete |
| 12 | Feedback Linearization | control | complete |
| 13 | LPA* | planning | complete |
| 14 | ARA* | planning | complete |

These waves established the basic implementation pattern used throughout the workspace:

- public config struct with validation
- algorithm struct with `new` / `try_new`
- inline `#[cfg(test)]` tests
- `nalgebra` for math
- `rust_robotics_core` types and traits for stable contracts

### Wave 3

Wave 3 closed the medium-hard implementation gap that the previous version of this file was centered on.

| # | Algorithm | Crate | File | Status |
|---|-----------|-------|------|--------|
| 15 | Square Root UKF | localization | `crates/rust_robotics_localization/src/square_root_ukf.rs` | complete |
| 16 | Backstepping Control | control | `crates/rust_robotics_control/src/backstepping_control.rs` | complete |
| 17 | FMT* | planning | `crates/rust_robotics_planning/src/fmt_star.rs` | complete |
| 18 | Pose Graph Optimization | slam | `crates/rust_robotics_slam/src/pose_graph_optimization.rs` | complete |
| 19 | Correlative Scan Matching | slam | `crates/rust_robotics_slam/src/correlative_scan_matching.rs` | complete |
| 20 | iLQR | control | `crates/rust_robotics_control/src/ilqr.rs` | complete |

Wave 3 is no longer a pending milestone. It should be treated as finished baseline work for the current branch.

### Branch-local planning additions after Wave 3

Beyond the original Wave 3 target, the current branch also contains additional planning implementations that broaden the repo beyond the old plan:

| Module | File | Purpose | Current role |
|---|---|---|---|
| Bipedal Planner | `crates/rust_robotics_planning/src/bipedal_planner.rs` | footstep / LIPM-style trajectory planning | branch-local algorithm addition |
| RRT Sobol | `crates/rust_robotics_planning/src/rrt_sobol.rs` | low-discrepancy sampling variant of RRT | branch-local algorithm addition |
| RRT Path Smoothing | `crates/rust_robotics_planning/src/rrt_path_smoothing.rs` | shortcut-based smoothing wrapper over RRT | branch-local algorithm addition |
| A* Variants | `crates/rust_robotics_planning/src/a_star_variants.rs` | beam / iterative / dynamic / theta-like / jump-corner family | branch-local algorithm addition |
| Fringe Search | `crates/rust_robotics_planning/src/fringe_search.rs` | thresholded best-first grid search | branch-local algorithm addition |
| IDA* | `crates/rust_robotics_planning/src/ida_star.rs` | memory-light iterative deepening A* | branch-local algorithm addition |

These additions are real source modules in the tree. Some also participate in the active experiment tracks described below.

---

## Active Experimental Tracks

## 1. Planning Runtime Aggregation

### Problem slice

The repo is comparing multiple runtime aggregation strategies over MovingAI crossover buckets before treating one workflow as the stable evaluation path.

### Current decision

- `full-bucket` remains the convergence reference.
- `variance-triggered` is the best current exploratory proxy.
- `first-scenario` remains useful only as a cheap smoke test.

### Why it matters

This track affects how planner comparisons are interpreted. It is not adding a new planner; it is deciding how much evidence is enough before a planner “winner” label is trusted.

### Current stable boundary

The stable shared surface is intentionally small:

- `RuntimeAggregationVariant`
- `RuntimeSamplingPlan`
- `RuntimeExperimentCase`
- `RuntimeObservation`
- `run_variant_suite`

Concrete aggregation variants stay under package-local `experiments/` code until they prove reusable across packages.

### Current status

- This track is active and reusable across packages.
- It already feeds planning evidence and is designed so localization / control / mapping can reuse the same evaluation contract.

Primary references:

- `docs/experiments.md`
- `docs/decisions.md`
- `docs/interfaces.md`

---

## 2. Planning Grid Threshold Comparison

### Problem slice

Compare `AStarPlanner`, `FringeSearchPlanner`, and `IDAStarPlanner` under the same `PathPlanner` contract and then push `IDA*` until its practical exactness boundary is understood, not assumed.

### Stable contract

```rust
PathPlanner::plan(Point2D, Point2D) -> Result<Path2D, RoboticsError>
```

This is important: no new planning core abstraction has been promoted from this work. The experiment is intentionally happening behind the existing trait boundary.

### Current result

- `AStarPlanner` remains the reference winner on median-timed runtime.
- `FringeSearchPlanner` remains useful as a lower-peak-live alternative.
- `IDAStarPlanner` remains experimental, but its exactness boundary is now much better characterized than before.

### Current evidence ladder

The branch now has explicit exact/non-exact thresholds for several local MovingAI slices:

- bucket `5`: one-iteration cheap floor reaches exact at `22`, while `21` still fails
- bucket `11`: cheap floor reaches exact at `43`, while `42` still fails
- bucket `12`: cheap floor reaches exact at `132`, while `131` still fails
- bucket `10`: threshold-round recovery returns at `iter32`, while `iter24` still fails
- bucket `15`: this is the hard slice; representative `iter256-exp3000000` still fails, full-slice `iter512-exp1750775` stays mixed (`window08` exact, `window16/24` expansion-stop), and full-slice `iter512-exp1750776` reaches exact
- bucket `15` fixed-budget iteration floor: once budget is fixed at `iter512-exp1750776`, the iteration floor is `275`, while `274` still fails with a mean next-threshold gap of `0.058875`

### Why this track matters

This experiment is no longer just “which planner is faster.” It now explains where the bounded `IDA*` approximation breaks, how it recovers, and what exactness costs on harder local slices. That gives the branch a real basis for deciding whether `IDA*` should stay experimental or be pushed further.

### Current status

- This track is active.
- The interface is stable.
- The evidence is deep enough to guide future bounded-search work.
- The bucket-`15` diagnostics follow-up is now in place: failure-preserving reports show the hard slice is dominated by contour re-expansion, not broad frontier growth.
- The next value is no longer adding basic effort metrics; it is deciding whether to push the planning track further with memory-oriented diagnostics or switch attention back to controller + generator pairings and the optional `dora` payload follow-up.

Primary references:

- `docs/experiments_planning_grid_threshold.md`
- `docs/decisions_planning_grid_threshold.md`
- `docs/interfaces.md`

---

## 3. Drone Trajectory Generator Comparison

### Problem slice

Compare multiple drone trajectory generators under the same sampled `DesiredState` + PD-tracker contract, then promote only the helper surface that survives repeated re-runs.

### Stable contract

- sampled `DesiredState` sequences
- consumed by the existing quadrotor PD tracker

The stable helper surface promoted so far is deliberately small:

- `generate_waypoint_trajectory_with_durations`
- `sample_trajectory_segments`
- `simulate_desired_states`

Concrete generators remain experimental:

- `quintic-uniform`
- `quintic-distance-scaled`
- `minimum-snap-distance-scaled`

### Presets already compared

The current branch has five concrete control presets for this problem:

1. `control-drone-trajectory-variants`
2. `control-drone-trajectory-pass-through`
3. `control-drone-trajectory-pass-through-accel`
4. `control-drone-trajectory-pass-through-accel-jerk`
5. `control-drone-trajectory-coupled-continuity`

### Current decision

Across the current branch, the practical default still remains:

- `quintic-distance-scaled`

But the branch no longer has a clean global loser:

- `minimum-snap-distance-scaled` still loses clearly on the tested non-coupled presets,
- while under coupled-continuity plus the improved bounded-feedback controller it now has a tiny local quality edge that is still too expensive and too narrow to promote.

### What changed across presets

- stop-go preset: baseline comparison for mixed-length loops
- pass-through preset: adds heuristic waypoint velocity
- pass-through-accel preset: adds finite-difference waypoint acceleration
- pass-through-accel-jerk preset: adds finite-difference waypoint jerk
- coupled-continuity preset: solves closed-loop waypoint velocities globally, preserves `C2` waypoint acceleration continuity, and derives waypoint jerk from adjacent coupled segments

The boundary-only reruns did not flip the ranking cleanly enough to promote minimum-snap.

The newer controller + generator pairing rerun did sharpen that story:

- on `stop-go`, the improved controller changes the best-RMSE label from `quintic-uniform` to `quintic-distance-scaled`,
- on `pass-through`, `pass-through-accel`, and `pass-through-accel-jerk`, `quintic-distance-scaled` remains the best practical pairing,
- on `coupled-continuity`, `minimum-snap-distance-scaled` becomes marginally best on RMSE and max error, but only by a tiny margin and at roughly `4x` the generation cost.

### Important nuance

The jerk-enriched preset did not fundamentally change the quintic branch. The added jerk term only changes the minimum-snap branch because the quintic generator consumes waypoint position, velocity, and acceleration but not jerk. This means the jerk-only result is evidence about minimum-snap sensitivity, not about a new common stable surface.

The coupled-continuity preset is different: it smooths both branches because both consume the globally solved velocity/acceleration boundary profile. On the current branch that sharply lowers generated jerk and snap, but it also pushes the bottleneck into the existing PD tracker and raises tracking RMSE on the harder loop families.

### Current status

- This track is active.
- The stable helper boundary is good enough for current callers.
- The experimental generator comparison is still valuable.
- The explicit pairing question is now answered well enough for this branch snapshot.
- The next step should be gain-sensitivity work or broader coupled-family coverage, not widening the public helper API without evidence.

Primary references:

- `docs/experiments_control_drone_trajectory.md`
- `docs/experiments_control_drone_trajectory_pass_through.md`
- `docs/experiments_control_drone_trajectory_pass_through_accel.md`
- `docs/experiments_control_drone_trajectory_pass_through_accel_jerk.md`
- `docs/experiments_control_drone_trajectory_coupled_continuity.md`
- `docs/experiments_control_drone_trajectory_controller_generator_pairings.md`
- `docs/decisions_control_drone_trajectory.md`
- `docs/decisions_control_drone_trajectory_pass_through.md`
- `docs/decisions_control_drone_trajectory_pass_through_accel.md`
- `docs/decisions_control_drone_trajectory_pass_through_accel_jerk.md`
- `docs/decisions_control_drone_trajectory_coupled_continuity.md`
- `docs/decisions_control_drone_trajectory_controller_generator_pairings.md`
- `docs/interfaces.md`

---

## Stable Engineering Conventions

These conventions remain valid and should continue to shape new work:

- keep public interfaces small and concrete
- prefer experiment-first comparison over pre-emptive abstraction
- keep inline `#[cfg(test)]` coverage for algorithm modules
- use `nalgebra` for matrix and vector math
- use `rust_robotics_core` types and traits as the stable contract surface
- avoid `unsafe`
- keep public docs present and clippy-clean

In practice, this means:

- new algorithms can be added freely as package-local modules,
- but new core traits should only appear if multiple variants truly need them,
- and experiment results should update the docs in the same change that introduces or reruns them.

---

## Current Priorities

### Priority 1: drone boundary quality — GAIN SENSITIVITY COMPLETE

The branch now includes:

- **`LateralGainSet`** struct for parameterized lateral feedback gains
- **`evaluate_lateral_gain_case()`** for running arbitrary gain sets
- **Gain sensitivity sweep** (5x5 grid: `lateral_position_gain` x `lateral_velocity_gain`)

Results on expanded coupled-continuity families (5 families):

- Default gains (pg=0.15, vg=0.45) are within 10% of the best sweep point for both generators
- Best quintic-distance-scaled: pg=0.25, vg=0.65 (RMSE 1.469)
- Best minimum-snap-distance-scaled: pg=0.25, vg=0.55 (RMSE 1.457)
- minimum-snap achieves marginally better RMSE at the best gain point, but the gap is tiny
- Higher gains reduce RMSE but increase torque demand monotonically

The ranking is stable to mild gain retuning: quintic-distance-scaled remains the practical default.

### Priority 2: drone controller variants — EXPANDED FAMILY COVERAGE COMPLETE

Coupled-continuity families expanded from 3 to 5:

- original: `oval-cruise-coupled`, `figure-eight-climb-coupled`, `banked-diamond-coupled`
- new: `spiral-climb-coupled`, `reverse-s-coupled`

Full 5-family x 3-generator x 3-controller pairings confirm:

- bounded lateral-feedback controller wins decisively (mean RMSE 1.63 vs baseline 13.26)
- the controller advantage generalizes to all 5 coupled families
- minimum-snap remains marginally better on RMSE only under coupled-continuity with tuned gains, but at ~4x generation cost

### Priority 3: planning bounded-search diagnostics — COMPLETE

The planning threshold work now includes per-contour diagnostics:

- `ContourStats` and `contour_history` added to `IDAStarPlanReport`
- Per-contour breakdown of expanded/reexpanded/generated/prune counts and max depth
- The `iter274/275` boundary on bucket `15` is now fully explained:
  - final contour raises depth from 59 to 60 via a threshold step of 0.059
  - 99.94% of all expansions are re-expansions across 274+ contours
  - total ~1.75M expansions to touch only ~1,000 unique states
- No further memory-oriented diagnostics are needed to explain this boundary

### Priority 4: optional `dora-rs` follow-up — SEMANTIC NODE ADDED

The dataflow now has three nodes:

1. **planner node** — runs A* and emits `path-report` (JSON)
2. **metrics node** (NEW) — receives `path-report`, computes per-segment quality metrics (efficiency, turning angles, segment length stats), and emits `path-metrics` (JSON)
3. **sink node** — receives `path-report` and prints a human-readable summary

The metrics node (`dora_path_metrics_node.rs`) is the "second node that consumes the path semantically" described in the previous plan. It calculates:

- path efficiency (direct_distance / path_length)
- segment length statistics (mean, min, max)
- turning angle statistics (mean, max)

The next useful increment would be a bridge into another runtime boundary such as ROS2, or a node that drives a simulated robot along the path.

### Priority 5: keep the plan in sync with the branch

This file previously fell behind the actual branch state. Going forward, `plan.md` should be updated whenever:

- a new algorithm wave is materially added,
- a new experiment preset is promoted into repeated use,
- or a stable boundary changes.

---

## Verification Commands

### Workspace-level

```bash
cargo check --workspace
```

### `dora-rs` demo

```bash
cargo build -p rust_robotics --example dora_path_planning_node --example dora_path_planning_sink --example dora_path_metrics_node --features "planning,dora"
dora run crates/rust_robotics/examples/dora_path_planning_dataflow.yml
```

### Planning comparison work

```bash
cargo test -p rust_robotics_planning --test grid_threshold_planner_comparison -- --nocapture
cargo clippy -p rust_robotics_planning --all-targets -- -D warnings
```

### Drone trajectory comparison work

```bash
cargo test -p rust_robotics_control --test drone_trajectory_variant_comparison -- --nocapture
cargo clippy -p rust_robotics_control --all-targets -- -D warnings
```

### Wave 3 historical verification set

```bash
cargo test -p rust_robotics_control -- backstepping ilqr --nocapture
cargo test -p rust_robotics_planning -- fmt_star --nocapture
cargo test -p rust_robotics_slam -- pose_graph correlative_scan --nocapture
```

---

## Key Reference Files

### Planning algorithms

- `crates/rust_robotics_planning/src/fmt_star.rs`
- `crates/rust_robotics_planning/src/bipedal_planner.rs`
- `crates/rust_robotics_planning/src/rrt_sobol.rs`
- `crates/rust_robotics_planning/src/rrt_path_smoothing.rs`
- `crates/rust_robotics_planning/src/a_star_variants.rs`
- `crates/rust_robotics_planning/src/fringe_search.rs`
- `crates/rust_robotics_planning/src/ida_star.rs`

### Control algorithms and drone helpers

- `crates/rust_robotics_control/src/backstepping_control.rs`
- `crates/rust_robotics_control/src/ilqr.rs`
- `crates/rust_robotics_control/src/drone_3d_trajectory.rs`
- `crates/rust_robotics_control/src/minimum_snap_trajectory.rs`
- `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`
- `crates/rust_robotics_control/tests/drone_trajectory_variant_comparison.rs`

### SLAM / localization anchors

- `crates/rust_robotics_slam/src/pose_graph_optimization.rs`
- `crates/rust_robotics_slam/src/correlative_scan_matching.rs`
- `crates/rust_robotics_localization/src/square_root_ukf.rs`

### `dora-rs` example anchors

- `crates/rust_robotics/Cargo.toml`
- `crates/rust_robotics/examples/dora_path_planning_node.rs`
- `crates/rust_robotics/examples/dora_path_metrics_node.rs`
- `crates/rust_robotics/examples/dora_path_planning_sink.rs`
- `crates/rust_robotics/examples/dora_path_planning_dataflow.yml`

### Current decision documents

- `docs/experiments.md`
- `docs/decisions.md`
- `docs/interfaces.md`
- `docs/experiments_planning_grid_threshold.md`
- `docs/decisions_planning_grid_threshold.md`
- `docs/experiments_control_drone_trajectory.md`
- `docs/experiments_control_drone_trajectory_pass_through.md`
- `docs/experiments_control_drone_trajectory_pass_through_accel.md`
- `docs/experiments_control_drone_trajectory_pass_through_accel_jerk.md`
- `docs/experiments_control_drone_trajectory_coupled_continuity.md`
- `docs/decisions_control_drone_trajectory_coupled_continuity.md`

---

## Summary

The old question was whether Wave 3 could be implemented cleanly. That question is answered.

The current branch is now in a different phase:

- algorithm breadth has expanded beyond the old Wave 3 scope,
- planning evaluation has become more experiment-driven,
- drone trajectory work has a stable helper boundary but still-open generator/controller trade-offs,
- and the next useful work is not more abstraction, but better evidence on the current experimental fronts.
