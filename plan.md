# RustRobotics Development Plan

## Project Overview

RustRobotics is a Rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics). The project is a Cargo workspace with multiple crates covering planning, localization, control, mapping, SLAM, and visualization.

Repository: `rsasaki0109/rust_robotics`
Branch: `main`
Current state: 90+ algorithms implemented, 860+ tests passing, clippy/fmt clean.

---

## Current State (as of 2026-04-01)

### Crate Structure

```
crates/
├── rust_robotics_core/          — Core types, traits, errors (Point2D, Pose2D, Path2D, etc.)
├── rust_robotics_planning/      — 50+ path planning algorithms
├── rust_robotics_localization/  — 6 localization filters (EKF, UKF, CKF, EnKF, PF, Histogram)
├── rust_robotics_control/       — 15+ control/tracking algorithms + MPC with QP solver
├── rust_robotics_mapping/       — 10 mapping algorithms
├── rust_robotics_slam/          — 5 SLAM algorithms (EKF-SLAM, FastSLAM1/2, GraphSLAM, ICP)
├── rust_robotics_viz/           — Visualization (gnuplot wrapper)
└── rust_robotics/               — Umbrella crate (feature-gated re-exports)
```

### Test Status

- `cargo test --workspace -- --test-threads=1`: **860 passed, 0 failed, 31 ignored**
- `cargo clippy --workspace --all-targets`: 0 errors
- `cargo fmt --all -- --check`: 0 diffs
- CI: GitHub Actions (build, test, clippy, rustdoc, fmt, cargo-deny, coverage)

### PythonRobotics Porting Status: ~95%

#### Planning (50+ modules)
- **Grid-based**: A*, BFS, DFS, GreedyBestFirst, BidirectionalA*, BidirectionalBFS, Dijkstra, D* Lite, D*, FlowField, JPS, Theta*, HybridA*, SIPP, 3D GridA*
- **Sampling-based**: RRT, RRT*, InformedRRT*, BatchInformedRRT*, BIT*, RRT-Dubins, RRT*-Dubins, RRT*-ReedsShepp, ClosedLoopRRT*, LQR-RRT*, PRM, VoronoiRoadMap, VisibilityRoadMap
- **Curve-based**: BezierPath, BSpline, CatmullRom, Eta3, CubicSpline, QuinticPolynomials, ClothoidPath, DubinsPath, ReedsSheppPath
- **Optimization**: DWA, PotentialField, LQR Planner, FrenetOptimalTrajectory, StateLattice, MPTG, ElasticBands, DMP, PSO, TimeBasedPlanning
- **Coverage**: GridBasedSweepCPP, WavefrontCPP, SpiralSpanningTreeCPP
- **Reactive**: BugPlanning

#### Control (15+ modules)
- **Path tracking**: PurePursuit, Stanley, RearWheelFeedback, LQR Steer, LQR Speed+Steer, MPC
- **Other**: CGMRES NMPC, LQR Control, MoveToPose, BehaviorTree, StateMachine
- **Arm**: TwoJointArm, NJointArm IK, ArmObstacleNavigation
- **Aerial**: Drone3D Trajectory, RocketLanding

#### Localization (6 modules)
EKF, UKF, CKF (Cubature), EnKF (Ensemble), ParticleFilter, HistogramFilter

#### Mapping (10 modules)
GaussianGridMap, RayCastingGridMap, NDT, LidarToGridMap, DistanceMap, CircleFitting, RectangleFitting, KMeansClustering, NormalVectorEstimation, PointCloudSampling

#### SLAM (5 modules)
EKF-SLAM, FastSLAM1, FastSLAM2, GraphBasedSLAM, ICPMatching

### Remaining Unported (~5%)
- ArmNavigation: rrt_star_seven_joint_arm_control, n_joint_arm_3d
- Mapping: grid_map_lib (utility, Python-specific)

---

## Benchmark Results (2026-04-01)

### Planning Benchmark: 9 Planners x 3 Scenarios (50x50 grid)

Benchmark file: `crates/rust_robotics_planning/benches/unified_planning_benchmark.rs`
Run command: `cargo bench --bench unified_planning_benchmark -p rust_robotics_planning`
Tables below use the Criterion median from the 2026-04-01 rerun after the maze layout fix.

#### Open Grid (50x50, boundary only)

| Planner | Time (µs) | Path Length | Waypoints | Speed Rank |
|---|---|---|---|---|
| DFS | 20.25 | 65.05 | 47 | 1 |
| GreedyBestFirst | 23.82 | 65.05 | 47 | 2 |
| A* | 27.04 | 65.05 | 47 | 3 |
| JPS | 70.55 | 65.05 | 47 | 4 |
| BidirectionalA* | 134.82 | 65.05 | 47 | 5 |
| Theta* | 143.86 | 65.05 | 2 | 6 |
| FlowField | 241.43 | 65.05 | 47 | 7 |
| BidirectionalBFS | 415.25 | 65.64 | 48 | 8 |
| BFS | 747.97 | 65.05 | 47 | 9 |

#### Maze (50x50, walls with gaps)

| Planner | Time (µs) | Path Length | Waypoints | Notes |
|---|---|---|---|---|
| GreedyBestFirst | 46.58 | 81.11 | 73 | Fastest but 51% longer than A* |
| JPS | 88.03 | 66.14 | 63 | 2.8x faster than A*, 23% longer path |
| FlowField | 195.95 | 53.60 | 43 | Same path quality as A* |
| DFS | 237.75 | **758.32** | 546 | **Path quality disaster: 14.1x A*** |
| A* | 242.61 | 53.60 | 43 | Optimal grid path |
| BidirectionalBFS | 391.83 | 53.60 | 43 | Same path as BFS/A*, slower |
| BFS | 449.26 | 53.60 | 43 | |
| Theta* | 485.11 | **52.62** | 7 | **Best path quality (any-angle)** |
| BidirectionalA* | 587.23 | 54.18 | 44 | Slower and slightly worse than A* |

#### Dense Grid (50x50, ~20% obstacles)

| Planner | Time (µs) | Path Length | Waypoints | Notes |
|---|---|---|---|---|
| GreedyBestFirst | 24.82 | 69.98 | 54 | Fastest but suboptimal path |
| DFS | 39.82 | **153.91** | 111 | **Path quality disaster: 2.3x optimal** |
| JPS | 160.56 | 71.74 | 57 | Faster than A*, slightly worse path |
| FlowField | 255.43 | 67.98 | 52 | Essentially tied with A* |
| A* | 255.54 | 67.98 | 52 | Optimal grid path |
| Theta* | 330.99 | **66.19** | 11 | **Best path quality (any-angle)** |
| BidirectionalBFS | 401.52 | 67.98 | 52 | |
| BidirectionalA* | 601.04 | 67.98 | 52 | |
| BFS | 647.15 | 67.98 | 52 | |

### Key Findings from Benchmark

1. **DFS is fast but brittle in clutter** — On open grids DFS is the fastest planner (20.25µs) and happens to trace the same diagonal as A*. In clutter, path quality collapses: 2.3x optimal on dense grids (153.91 vs 67.98) and 14.1x optimal on the maze (758.32 vs 53.60).

2. **JPS is scenario-dependent even on 50x50 grids** — It is slower than A* on the open grid (70.55 vs 27.04µs), but faster on maze (88.03 vs 242.61µs) and dense maps (160.56 vs 255.54µs). The speedup comes with a path-quality penalty in clutter (maze 66.14 vs 53.60, dense 71.74 vs 67.98).

3. **Theta* consistently produces the shortest paths** — It yields the best path quality in maze (52.62, 7 waypoints) and dense maps (66.19, 11 waypoints), and collapses the open-grid path to 2 waypoints. The compute cost ranges from 1.3x to 5.3x A* depending on scenario.

4. **BidirectionalA* still underperforms unidirectional A*** — It is slower than A* in all three scenarios, and on the maze it also returns a slightly worse path (54.18 vs 53.60).

5. **FlowField is a strong balanced baseline** — It matches A* path quality on maze and dense maps while being faster on maze (195.95 vs 242.61µs) and effectively tied on dense (255.43 vs 255.54µs).

6. **GreedyBestFirst dominates latency but quality depends on map** — It is fastest on maze and dense maps (46.58µs and 24.82µs), but its path can be either mildly worse (dense: +3%) or substantially worse (maze: +51%).

7. **BFS remains a poor latency baseline** — It is the slowest planner on open and dense grids, and still near the bottom on maze despite returning the same grid-optimal path as A*.

8. **The maze scenario is now useful instead of broken** — The upstream JPS regression maze is solvable for all 9 planners and exposes meaningful trade-offs across latency, path quality, and waypoint count.

### Localization Benchmark: 6 Filters

Test file: `crates/rust_robotics_localization/tests/unified_filter_comparison.rs`
Run command: `cargo test -p rust_robotics_localization --test unified_filter_comparison -- --nocapture`

| Filter | RMSE (m) | Time (ms) | Final Error (m) | Notes |
|---|---|---|---|---|
| EKF | 0.337 | 9.3 | 0.382 | Fastest, good accuracy |
| UKF | 0.252 | 31.4 | 0.388 | Best RMSE, 3x EKF time |
| CKF | 0.254 | 20.2 | 0.388 | Similar to UKF, faster |
| EnKF | 0.740 | 121.1 | 0.214 | Worst RMSE, best final | 
| PF | 0.212 | 253.7 | 0.298 | Best RMSE but slowest Kalman |
| Histogram | 0.242 | 558.0 | 0.173 | Best final, very slow |

### Key Findings from Localization Benchmark

1. **CKF is a better UKF** — Same accuracy (0.254 vs 0.252) but 35% faster (20ms vs 31ms). CKF has no tuning parameters (alpha/beta/kappa), making it more practical.

2. **EKF is 3x faster than anything else** — 9.3ms vs next best 20.2ms (CKF). For real-time applications, EKF is hard to beat.

3. **EnKF has worst RMSE but best final error** — The ensemble approach is noisy during tracking but converges well at the end. High computation cost (121ms) makes it impractical for real-time.

4. **PF and Histogram are too slow** — 254ms and 558ms respectively. Only viable for offline processing or very low update rates.

5. **UKF vs CKF trade-off** — UKF is slightly more accurate, CKF is faster and simpler. CKF should be the default recommendation over UKF.

---

## New Results (2026-04-04)

### 1. JPS Grid Size Crossover Analysis (COMPLETED)

Benchmark file: `crates/rust_robotics_planning/benches/jps_crossover_benchmark.rs`
Run command: `cargo bench --bench jps_crossover_benchmark -p rust_robotics_planning`

#### Open Grid (boundary only): JPS never wins

| Size | A* (µs) | JPS (µs) | JPS/A* |
|---|---|---|---|
| 50 | 14.0 | 54.3 | 3.9x slower |
| 100 | 29.8 | 183.7 | 6.2x slower |
| 200 | 61.4 | 656.6 | 10.7x slower |
| 500 | 162.9 | 3,799 | 23.3x slower |
| 1000 | 333.3 | 14,707 | 44.1x slower |

**Conclusion**: On open grids, JPS is always slower than A* and the gap widens with grid size. A* traces the diagonal in O(n), JPS jump scan overhead dominates.

#### Dense Grid (~20% random obstacles): JPS always loses (~2x)

| Size | A* (µs) | JPS (µs) | JPS/A* |
|---|---|---|---|
| 50 | 322 | 526 | 1.6x slower |
| 100 | 1,587 | 2,930 | 1.8x slower |
| 200 | 7,377 | 15,319 | 2.1x slower |
| 500 | 46,454 | 98,998 | 2.1x slower |
| 1000 | 229,400 | 440,560 | 1.9x slower |

**Conclusion**: On dense random obstacles, JPS is consistently ~2x slower than A*. Random obstacle placement breaks jump symmetry, making forced-neighbor overhead dominant.

#### Maze Grid (walls with gaps): JPS wins, the only favorable scenario

| Size | A* (µs) | JPS (µs) | JPS/A* |
|---|---|---|---|
| 50 | 321 | 254 | 1.26x faster |
| 100 | 1,857 | 1,711 | 1.09x faster |
| 200 | 9,255 | 8,858 | 1.04x faster |
| 500 | 60,656 | 50,042 | 1.21x faster |
| 1000 | 284,500 | 223,360 | 1.27x faster |

**Conclusion**: Maze is the only scenario where JPS beats A*. The advantage is modest (1.04-1.27x) and never reaches the 10-40x speedups claimed in literature. The no-corner-cutting grid semantics limit jump distances.

#### Overall JPS Crossover Verdict

JPS wins **only on structured mazes** with max 1.27x speedup. It loses on open grids (up to 45x slower) and dense grids (~2x slower). There is **no grid-size crossover** on open or dense maps — JPS gets worse, not better, as grids grow.

### 2. Lazy Theta* Implementation (COMPLETED)

File: `crates/rust_robotics_planning/src/lazy_theta_star.rs`
Tests: 6 passed (path finding, A* comparison, Theta* comparison, obstacle points, config validation, legacy interface)

#### Benchmark Results (50x50 unified benchmark)

| Planner | Open (µs) | Maze (µs) | Dense (µs) |
|---|---|---|---|
| Theta* | 160.4 | 389.3 | 428.8 |
| LazyTheta* | 37.8 | 156.6 | 100.6 |
| **Speedup** | **4.2x** | **2.5x** | **4.3x** |

#### Path Quality (any-angle, shorter than grid-optimal)

| Planner | Open | Maze | Dense |
|---|---|---|---|
| Theta* | 65.05 (2wp) | 53.88 (8wp) | 71.59 (15wp) |
| LazyTheta* | 65.05 (43wp) | 50.00 (2wp) | 65.46 (6wp) |
| A* (ref) | 65.05 (47wp) | 56.53 (48wp) | 75.60 (65wp) |

**Key findings**:
1. **LazyTheta* is 2.5-4.3x faster than Theta*** by deferring line-of-sight checks
2. **LazyTheta* produces shorter or equal paths** — maze: 50.00 vs 53.88, dense: 65.46 vs 71.59
3. **LazyTheta* is the fastest any-angle planner** and competitive with grid-optimal planners on time

### 3. CKF vs UKF Broad Comparison (COMPLETED)

Test file: `crates/rust_robotics_localization/tests/ckf_vs_ukf_broad_comparison.rs`
Run command: `cargo test -p rust_robotics_localization --test ckf_vs_ukf_broad_comparison -- --nocapture`

#### 10 Scenario Results

| Scenario | CKF RMSE | UKF RMSE | RMSE Δ | CKF Time | UKF Time | Time Δ |
|---|---|---|---|---|---|---|
| Low noise | 0.1595 | 0.1593 | +0.1% | 9.3ms | 13.4ms | -30% |
| High process noise | 0.2486 | 0.2458 | +1.1% | 9.2ms | 13.4ms | -31% |
| High obs noise | 0.4937 | 0.4909 | +0.6% | 9.4ms | 13.3ms | -30% |
| Tight circle | 0.2679 | 0.2662 | +0.7% | 9.6ms | 13.3ms | -28% |
| Fast motion | 0.2784 | 0.2789 | -0.2% | 9.2ms | 13.3ms | -31% |
| Long horizon (500) | 0.2685 | 0.2682 | +0.1% | 44.8ms | 64.4ms | -30% |
| Asymmetric noise | 0.3923 | 0.3961 | -1.0% | 9.2ms | 12.7ms | -27% |
| Near-linear | 0.2612 | 0.2608 | +0.2% | 8.6ms | 12.9ms | -33% |
| High proc + low obs | 0.1057 | 0.1056 | +0.2% | 9.3ms | 12.4ms | -25% |
| Low proc + high obs | 0.5914 | 0.6022 | -1.8% | 8.9ms | 12.7ms | -29% |

**Summary**:
- CKF total time: 127.6ms, UKF total time: 181.6ms — **CKF is 30% faster**
- RMSE difference: **±2% across all 10 scenarios**
- CKF tuning parameters: **0**, UKF: **3** (alpha=0.001, beta=2.0, kappa=0.0)
- CKF wins RMSE on 3/10, UKF on 7/10, but differences are negligible (<2%)

**Recommendation**: CKF should be the default over UKF for typical robotics scenarios.

### 4. DFS + Path Smoothing Pipeline (COMPLETED)

File: `crates/rust_robotics_planning/src/path_smoothing.rs`
Tests: 5 passed (waypoint reduction, endpoint preservation, obstacle respect, DFS+smooth vs A*, empty/single)

#### Benchmark Results (50x50 unified benchmark)

| Planner | Open | Maze | Dense |
|---|---|---|---|
| DFS (raw) | 10.6µs / 65.05 len | 125.6µs / 686.20 len | 441.7µs / 254.68 len |
| DFS+Smooth | 24.0µs / 65.05 len | 238.7µs / 71.15 len | 509.2µs / 74.96 len |
| A* | 14.0µs / 65.05 len | 228.0µs / 56.53 len | 329.1µs / 75.60 len |

**Key findings**:
1. **DFS+Smooth rescues DFS path quality**: maze 686→71 (90% reduction), dense 255→75 (71% reduction)
2. **Open grid**: DFS+Smooth matches A* quality (65.05) and is competitive on time (24.0 vs 14.0µs)
3. **Maze**: DFS+Smooth path (71.15) is still 26% longer than A* (56.53) — smoothing cannot recover from DFS exploring dead ends
4. **Dense**: DFS+Smooth (74.96) nearly matches A* (75.60) — smoothing works well when detours are small
5. **Verdict**: DFS+Smooth is useful for open/sparse environments but **not a replacement for A* in clutter**

---

## Improvement Opportunities Identified

Based on the benchmark results, these are concrete areas where algorithms can be improved. Each is a potential research contribution.

### HIGH PRIORITY (clear improvement path)

#### 1. JPS Grid Size Crossover Analysis
**Status**: COMPLETED (see above)
**Result**: JPS wins only on mazes (1.04-1.27x), loses on open (3.9-45x) and dense (~2x). No crossover found.
**Potential contribution**: Quantified JPS break-even analysis. Prior literature (Harabor 2011) only shows JPS winning on large maps.

#### 2. BidirectionalA* Overhead Reduction
**Problem**: BidirectionalA* is slower than A* on all tested scenarios.
**Hypothesis**: Meeting-point detection (checking if popped node is in opposite closed set) has O(n) overhead per expansion.
**Action**: Profile to find bottleneck. Optimize meeting detection with shared grid index. Compare lazy vs eager meeting check. Expected improvement: 2-5x on 50x50, should beat unidirectional A* on large grids.
**Potential contribution**: Practical bidirectional A* that actually outperforms unidirectional on grids.

#### 3. DFS Path Quality with Post-Processing
**Status**: COMPLETED (see above)
**Result**: DFS+Smooth works well on open/sparse (matches A*) but not in clutter (26% longer on maze). Not a universal replacement for A*.

#### 4. Adaptive Localization Filter Selection
**Problem**: No single filter is best in all conditions.
**Finding**: EKF is fastest, CKF is most balanced, PF handles non-Gaussian best.
**Action**: Implement adaptive filter that switches between EKF (low noise), CKF (moderate), PF (high noise/multimodal) based on innovation statistics or NEES.
**Potential contribution**: Adaptive filter selection for real-time robotics.

#### 5. CKF as Default Over UKF
**Status**: COMPLETED (see above)
**Result**: CKF is 30% faster, ±2% accuracy across 10 scenarios, 0 tuning params. CKF should be the default.

### MEDIUM PRIORITY (needs investigation)

#### 6. FlowField for Multi-Agent Planning
**Problem**: FlowField computes distance to ALL cells, which is wasteful for single-agent but perfect for multi-agent (compute once, extract path for each agent).
**Action**: Add multi-agent benchmark. Compare FlowField (1 computation + N path extractions) vs N independent A* runs.
**Potential contribution**: Quantified break-even for FlowField vs per-agent planning.

#### 7. MPC Solver Improvement
**Problem**: Projected-gradient solver has 4-step phase lead vs Python CLARABEL QP.
**Finding**: QP solver (clarabel) improves state parity 65.9% but worsens step count.
**Action**: Hybrid solver — use projected-gradient for forward driving, QP for reverse maneuvers.
**Potential contribution**: Adaptive solver selection based on maneuver phase.

#### 8. Theta* Acceleration
**Status**: COMPLETED (see above)
**Result**: Lazy Theta* is 2.5-4.3x faster than Theta* with equal or better path quality. Exceeded the expected 2-3x speedup.

#### 9. RRT* Convergence Speed
**Problem**: RRT* variants (6 implementations) have different convergence profiles not yet compared.
**Action**: Run all 6 RRT* variants on identical scenarios, plot cost vs iteration count. Identify which converges fastest and under what conditions.
**Potential contribution**: Unified RRT* variant comparison.

### LOW PRIORITY (nice to have)

#### 10. Maze Benchmark Refresh
**Status**: completed on 2026-04-01
**Action**: keep downstream notes/plots synchronized if benchmark figures are exported again.

#### 11. D* Dynamic Re-Planning
**Problem**: D* `plan_with_new_obstacles` has a bug (returns None when path should exist after re-plan).
**Action**: Investigate h.is_finite() check interaction with re-plan. Fix or document limitation.
**File**: `crates/rust_robotics_planning/src/d_star.rs`, test `test_dstar_with_new_obstacles` is `#[ignore]`d.

#### 12. Coverage Planner Comparison
**Problem**: 3 coverage planners (GridBasedSweep, Wavefront, SpiralSpanningTree) not benchmarked.
**Action**: Compare coverage percentage, path length, computation time on identical environments.

---

## MPC Parity Analysis (completed 2026-03-31)

### Summary

Comprehensive analysis of the MPC switch-back parity between Rust (projected-gradient solver) and Python (CVXPY CLARABEL QP solver).

### Key Findings

1. **Pre-reversal (step 0-70)**: strict parity with Python reference
2. **Reverse (step 71+)**: bounded parity, solver gap exists
3. **Phase lead**: expanded solver starts terminal maneuver 4 steps earlier
4. **Root cause**: solver type difference (projected-gradient vs QP), NOT linearization or state update
5. **QP solver (clarabel)**: 65.9% closed-loop parity improvement (y: 100x, v: 500x better)
6. **Production decision**: keep projected-gradient + candidate expansion (step count matches Python)

### Adjoint Analysis

y affects steering through: `y_error -> lambda[1] -> A^T[3,1]*lambda[1] -> lambda[3] -> B^T[1,3]*lambda[3] -> steer_gradient`

Where `A^T[3,1] = DT * v_bar * cos(yaw_bar)`. Near yaw = pi/2, cos(yaw) = 0.02 (small but nonzero). v_bar threshold for pull ratio drop: v = -0.695.

### Python-Rust Differences

| Item | Python | Rust | Impact |
|---|---|---|---|
| Solver | CVXPY CLARABEL (QP) | Projected-gradient + line search | Main gap source |
| linearized_rollout yaw | No normalize | normalize_angle | Step 75 only, no closed-loop effect |
| State constraints | v bounds in QP | v clamp in update only | No practical effect |
| get_linear_model_matrix | Identical | Identical | None |
| predict_motion | Identical | Identical | None |
| Cost weights Q/Qf/R/Rd | Identical | Identical | None |

### V-Injection Pull Ratio Threshold

Sweep from e325_v (-0.804) to e326_v (-0.604):

| Injected v | Pull Ratio |
|---|---|
| -0.804 (e325) | 112.8 |
| -0.786 | 15.1 |
| -0.768 | 8.5 |
| -0.750 | 6.1 |
| -0.731 | 5.1 |
| -0.713 | 4.5 |
| **-0.695** | **4.0 (threshold)** |
| -0.677 | 3.7 |
| -0.659 | 3.4 |
| -0.604 (e326) | 2.6 |

### MPC Test Inventory (29 ignored, long-running)

All in `crates/rust_robotics_control/src/mpc.rs`:
- Prefix trace parity (71 steps), reverse transition/detailed trace/step 75/78 parity
- Solver gap with Python warm start, Python-style linearized cost comparison
- Candidate expansion (various configurations), recovery window, terminal condition analysis
- State injection (component, subset, per-step), context swap (step 325-326)
- Outer loop iteration isolation, adjoint decomposition, yaw normalize effect
- No-angle-wrap closed-loop parity, QP solver comparison (inner + closed-loop), V threshold sweep

### Reference Files (outside git, on local disk)
- `/media/sasaki/aiueo/ai_coding_ws/rust_robo_ws/notes/rust_robotics_mpc_step325_326_analysis_2026-03-31.md`
- `/media/sasaki/aiueo/ai_coding_ws/rust_robo_ws/notes/rust_robotics_mpc_solver_gap_2026-03-29.md`
- `/media/sasaki/aiueo/ai_coding_ws/rust_robo_ws/notes/rust_robotics_pythonrobotics_parity_audit_2026-03-28.md`
- `/media/sasaki/aiueo/ai_coding_ws/rust_robo_ws/notes/rust_robotics_mpc_handoff_plan_2026-03-31.md`

---

## JPS Correctness Audit (completed 2026-03-31)

- Audited against Harabor & Grastien (2011) paper
- Result: **PASS** -- all 8 direction cases correct, no bugs
- Practical concern: `jump()` uses recursion (stack overflow risk on large grids, should be iterative loop)

---

## Benchmark Infrastructure

### Planning Benchmark
- File: `crates/rust_robotics_planning/benches/unified_planning_benchmark.rs`
- Command: `cargo bench --bench unified_planning_benchmark -p rust_robotics_planning`
- Uses criterion, tests 9 planners on 3 scenarios
- Outputs path summary table to stderr
- Regression guard: `crates/rust_robotics_planning/tests/unified_planning_benchmark_maze.rs` ensures `maze_50x50` remains solvable for all 9 planners

### Localization Comparison
- File: `crates/rust_robotics_localization/tests/unified_filter_comparison.rs`
- Command: `cargo test -p rust_robotics_localization --test unified_filter_comparison -- --nocapture`
- Simulates circular motion, runs all 6 filters, prints RMSE/time table

### Rust vs Python Speed Comparison
- Rust: `crates/rust_robotics/examples/speed_comparison.rs`
- Python: `scripts/speed_comparison.py`
- Comparison: `scripts/compare_speed.py`
- Command:
  ```bash
  cargo run -p rust_robotics --example speed_comparison --features full --release > /tmp/rust_bench.csv
  python3 scripts/speed_comparison.py > /tmp/python_bench.csv
  python3 scripts/compare_speed.py /tmp/rust_bench.csv /tmp/python_bench.csv
  ```
- Tests: A* (100x100), RRT, EKF (1000 steps), CubicSpline (1000 runs)
- Result (2026-04-01):

| Algorithm | Rust (ms) | Python (ms) | Speedup | Runs |
|---|---:|---:|---:|---:|
| A* | 4.003 | 924.464 | 230.9x | 100 |
| RRT | 0.123 | 5.676 | 46.2x | 100 |
| EKF | 0.190 | 103.134 | 542.6x | 1000 |
| CubicSpline | 0.922 | 6.921 | 7.5x | 1000 |

- Raw outputs:
  `/tmp/rust_bench_2026-04-01.csv`, `/tmp/python_bench_2026-04-01.csv`,
  `/tmp/rust_bench_2026-04-01.stderr`, `/tmp/python_bench_2026-04-01.stderr`
- Figure outputs:
  `img/paper/speed_comparison.csv`, `img/paper/rust_vs_python.pdf`
- Environment note: Python run emitted a matplotlib `Axes3D` import warning, but completed and produced CSV output.

### Paper Figure Scripts
- Directory: `scripts/paper_figures/`
- `plot_planning_comparison.py` -- reads CSV, produces bar charts + Pareto scatter
- `plot_localization_comparison.py` -- reads CSV, produces RMSE/time bar charts
- `plot_speed_comparison.py` -- reads Rust vs Python CSV, produces dual bar chart
- `plot_framework_overview.py` -- generates crate architecture diagram (no data needed)
- All use IEEE/ICRA matplotlib style (8pt serif, 3.5in width, PDF output)
- Output: `img/paper/*.pdf`

---

## Paper Plan: Algorithm Improvement via Unified Benchmarking

### Thesis (revised)

The value is NOT "we ported PythonRobotics to Rust." The value is: by having 90+ algorithms in a single framework, we can **identify concrete improvement opportunities** that are invisible when algorithms are compared across different codebases. The benchmark results above already show 8+ actionable findings.

### Research Direction

Use the unified framework as a **laboratory for algorithm improvement**:
1. Run benchmarks to find weaknesses
2. Propose and implement improvements
3. Validate improvements on the same benchmark
4. Repeat

### Concrete Paper Ideas (from benchmark results)

**Paper A: "When Does JPS Actually Help? Grid-Size Crossover Analysis"**
- JPS is 3x slower than A* on 50x50. Find the crossover grid size.
- If crossover is 500x500+, JPS is only useful for large maps.
- Implementation insight: forced neighbor check overhead dominates on small grids.
- Conference: IROS/ICRA short paper or RA-L.

**Paper B: "CKF Should Replace UKF: An Empirical Study"**
- CKF = same accuracy as UKF, 35% faster, zero tuning parameters.
- Run on 10+ scenarios (varying noise, nonlinearity, dimension).
- If CKF consistently matches UKF, this is a practical contribution.
- Conference: IROS/ICRA localization session.

**Paper C: "Fast-Then-Smooth: DFS + Path Smoothing as a Planning Pipeline"**
- DFS is fastest but path quality is 2.3x worse.
- Add shortcutting / line-of-sight smoothing to DFS output.
- If DFS+smooth total time < A* time AND path quality >= 95% optimal, this is valuable.
- Conference: IROS planning session.

**Paper D: "Adaptive MPC Solver Selection for Nonholonomic Vehicles"**
- Use projected-gradient for forward, QP for reverse maneuvers.
- Adjoint coupling analysis determines switch condition.
- Conference: CDC/CCTA control session.

### Target Venues
- **RA-L** (IEEE Robotics and Automation Letters) -- systems/benchmark papers welcome
- **IROS** -- systems and planning tracks
- **ICRA Workshop** -- open-source robotics
- **JOSS** (Journal of Open Source Software) -- software paper (easier, get DOI/citations)
- **SoftwareX** -- software journal

---

## Known Issues

### Test Flakiness
- `cargo test --workspace` with default parallel threads occasionally fails (CPU contention, floating-point sensitivity)
- Workaround: `--test-threads=1`

### D* Dynamic Re-Plan Bug
- `d_star::tests::test_dstar_with_new_obstacles` is `#[ignore]`d
- D* `plan_with_new_obstacles` returns None after adding obstacles when path should exist
- Root cause: `h.is_finite()` check (added to fix blocked-path bug) also rejects valid re-plan results
- Needs: separate finite check for initial plan vs re-plan

### Stochastic Tests
- EnKF, KMeans, Eta3 tests occasionally fail due to random initialization
- Mitigated by: seeded RNG where possible, wider tolerances

### clarabel Dependency
- clarabel (QP solver) is in `[dependencies]` not `[dev-dependencies]`
- Reason: `solve_qp_clarabel()` exists in production code (previous session)
- Could be moved to dev-dependency if production QP solver is removed

---

## Build & Test Commands

```bash
# Full build
cargo build --workspace

# All tests (single-threaded for reliability)
cargo test --workspace -- --test-threads=1

# MPC tests (normal)
cargo test -p rust_robotics_control --lib -- --test-threads=1

# MPC ignored (long-running) tests
cargo test -p rust_robotics_control test_mpc_switch_back -- --ignored --nocapture

# Planning benchmark
cargo bench --bench unified_planning_benchmark -p rust_robotics_planning

# Localization comparison
cargo test -p rust_robotics_localization --test unified_filter_comparison -- --nocapture

# Rust vs Python speed comparison
cargo run -p rust_robotics --example speed_comparison --features full --release > /tmp/rust_bench.csv
python3 scripts/speed_comparison.py > /tmp/python_bench.csv
python3 scripts/compare_speed.py /tmp/rust_bench.csv /tmp/python_bench.csv

# Paper figures (after collecting benchmark data)
python3 scripts/paper_figures/plot_framework_overview.py
python3 scripts/paper_figures/plot_planning_comparison.py
python3 scripts/paper_figures/plot_localization_comparison.py
python3 scripts/paper_figures/plot_speed_comparison.py

# Clippy
cargo clippy --workspace --all-targets -- -D warnings

# Format
cargo fmt --all -- --check

# Coverage
cargo tarpaulin --workspace --out html
```

---

## File Locations

### MPC Implementation
- Main: `crates/rust_robotics_control/src/mpc.rs` (~7800 lines)
- Test data: `crates/rust_robotics_control/src/testdata/mpc_switch_back_*.csv`

### Planning Algorithms
- Grid-based: `crates/rust_robotics_planning/src/{a_star,breadth_first_search,depth_first_search,dijkstra,jps,theta_star,d_star,d_star_lite,flow_field,bidirectional_a_star,bidirectional_bfs,greedy_best_first_search,hybrid_a_star,sipp,grid_a_star_3d}.rs`
- Sampling: `crates/rust_robotics_planning/src/{rrt,rrt_star,rrt_dubins,rrt_star_dubins,rrt_star_reeds_shepp,informed_rrt_star,batch_informed_rrt_star,closed_loop_rrt_star,lqr_rrt_star,bit_star,prm,voronoi_road_map,visibility_road_map}.rs`
- Curves: `crates/rust_robotics_planning/src/{bezier_path,bspline_path,catmull_rom_spline,eta3_spline,cubic_spline_planner,quintic_polynomials,clothoid_path,dubins_path,reeds_shepp_path}.rs`
- Coverage: `crates/rust_robotics_planning/src/{grid_based_sweep_cpp,wavefront_cpp,spiral_spanning_tree_cpp}.rs`
- Other: `crates/rust_robotics_planning/src/{bug_planning,dynamic_movement_primitives,elastic_bands,flow_field,lqr_planner,model_predictive_trajectory_generator,particle_swarm_optimization,time_based_path_planning,dwa,frenet_optimal_trajectory,potential_field,state_lattice/}.rs`

### Benchmarks
- Planning: `crates/rust_robotics_planning/benches/{grid_planners,unified_planning_benchmark}.rs`
- Localization: `crates/rust_robotics_localization/tests/unified_filter_comparison.rs`
- Speed: `crates/rust_robotics/examples/speed_comparison.rs`

### Scripts
- Speed comparison: `scripts/{speed_comparison.py,compare_speed.py}`
- Paper figures: `scripts/paper_figures/{plot_planning_comparison,plot_localization_comparison,plot_speed_comparison,plot_framework_overview}.py`

### CI
- `.github/workflows/ci.yml`

---

## Codex Handoff Notes

### What was done (2026-03-31 ~ 2026-04-01)

**Session 1: MPC Analysis + Algorithm Porting**
1. MPC switch-back parity analysis (adjoint coupling, QP solver, v threshold, normalize_angle)
2. JPS correctness audit (PASS, all 8 directions correct)
3. Ported 40+ algorithms from PythonRobotics (420+ tests)
4. Fixed test failures (arm IK, drone, EnKF, D* blocked path, eta3, bit_star, kmeans)
5. 14 commits pushed to main, README updated

**Session 2: Benchmark Infrastructure + Data Collection**
6. Created unified planning benchmark (9 planners x 3 scenarios)
7. Created unified localization comparison (6 filters)
8. Created Rust vs Python speed comparison scripts
9. Created paper figure generation scripts (IEEE/ICRA style)
10. Fixed D* blocked path bug (h.is_finite() check)
11. Ran planning benchmark -- collected data with 8+ actionable findings
12. Ran localization comparison -- CKF vs UKF insight
13. Ran Rust vs Python speed comparison -- Rust was faster in all 4 sampled workloads (7.5x to 542.6x)
14. Added MovingAI `.map` / `.scen` loader support and benchmark env hooks in `rust_robotics_planning`
15. Ran the first MovingAI benchmark subset (`arena2`, `8room_000`, `random512-10-0`, bucket 80), then fixed the `JPS` failure on `arena2`
16. Added shared no-corner-cutting grid transitions, revalidated `arena2` bucket 80, and confirmed that `A*`/`BFS`/`FlowField` now match the MovingAI optimal length while `Bidirectional*`/`JPS`/`Theta*` still diverge
17. Fixed `BidirectionalA*`, `BidirectionalBFS`, and `JPS` correctness on `arena2` bucket 80, and tightened `Theta*` line-of-sight semantics; the grid-optimal family now matches `323.20` on that scenario, but `Bidirectional*`/`JPS` regress in runtime
18. Fixed `FlowField` weighted-cost semantics, reran the full 3-map MovingAI subset, and confirmed that `A*`/`BidirectionalA*`/`BidirectionalBFS`/`JPS`/`FlowField` all match the scenario optima on `arena2`, `8room_000`, and `random512-10-0`
19. Split the unified planning benchmark summary into `Search Baselines`, `Grid-Optimal Family`, and `Any-Angle Family`, with `Ref`/`Delta` columns so `Theta*` is no longer mixed into the grid-shortest-path story
20. Added JPS diagnostics and measured fallback usage: current `JPS` falls back with `invalid_jump_path` on 5/6 benchmark scenarios and 3/3 current MovingAI scenarios, so its standard-map results are wrapper results rather than pure JPS
21. Broke down `invalid_jump_path`: all 5 current fallback cases are `cost_mismatch`, with zero `invalid_step_sequence`, so the remaining JPS bug is cost-consistency rather than collision-invalid reconstruction
22. Measured `JPS` cost-mismatch metrics on the current MovingAI subset: `arena2`, `8room_000`, and `random512-10-0` all have `search_cost < reconstructed_path_length` with non-zero `detour_segment_count`, so the remaining bug is underpriced jump edges under the shared no-corner-cutting semantics rather than a fallback-only reporting artifact
23. Tightened the no-corner-cutting JPS rules using the `Improving Jump Point Search` guidance (no diagonal forced-neighbour tests, shifted straight stop rule) and re-ran MovingAI diagnostics: the previous `cost_mismatch` cases now fall back via `search_exhausted` instead, with non-zero start successor counts (`4`, `3`, `6`), so the remaining issue is compressed-graph completeness rather than path-length inconsistency
24. Added a `suboptimal_jump_path` guard to the JPS correctness wrapper so pure JPS no longer returns longer-than-grid-optimal paths silently; current MovingAI diagnostics now split into `arena2 = search_exhausted` and `8room_000/random512-10-0 = suboptimal_jump_path`
25. Added raw JPS tracing for the current MovingAI subset: `arena2` now shows late-stage dead-end chaining (`368` expansions, final dead-ends with `successors=0`), while `8room_000` and `random512-10-0` reach the goal through very long pure-JPS jump-point chains (`105` and `108` jump points), confirming that the remaining bugs are structurally different
26. Refined the raw trace diagnostics: `arena2` has an earliest dead-end chain right near the start (`(22,106) -> (23,107) -> (27,107)`), `8room_000` shows high direction churn (`89` changes over `104` segments), and `random512-10-0` shows high short-segment density (`42/107`, ratio `0.393`), which suggests different pruning failures per map family
27. Added direction-aware JPS state keys plus reference-turn diagnostics: state collapse was a contributing factor (`arena2` raw expansions increased to `894`) but not the root cause, and the new A* turn-point comparison shows `arena2` misses the very first east-going reference turn (`(25,106)`), while `8room_000` and `random512-10-0` diverge from the reference at turn indices `1` and `3`
28. Fixed the `arena2` MovingAI completeness regression by allowing pure perpendicular successors in cardinal forced-neighbour cases; `arena2` now solves as pure JPS at the MovingAI optimal length again
29. Generalized the cardinal forced-neighbour fix so pure perpendicular successors no longer depend on diagonal validity, then strengthened the release regression: the current MovingAI subset (`arena2`, `8room_000`, `random512-10-0`, bucket 80) now solves as pure JPS with `used_fallback=false` and exact MovingAI-optimal path lengths on all 3 cases
30. Re-ran the short unified benchmark on the current code and refreshed `JPS Fallback Summary`: the current 6-scenario harness (`open`, `maze`, `dense`, plus the 3 MovingAI cases) now shows `JPS fallback rate (all scenarios) = 0/6` and `JPS fallback rate (MovingAI scenarios) = 0/3`
31. Added a sampled MovingAI bucket sweep regression (`bucket 0/20/40/60/80`) across all 3 vendored maps; the current `JPS` now solves all 15 sampled cases as pure JPS with exact scenario-optimal path lengths and zero fallback
32. Extended MovingAI coverage again: long-tail release checks now pass on `arena2` max bucket plus `8room_000/random512-10-0` `100+` buckets, and a newly vendored `maze512-1-0` family also passes sampled buckets (`0/20/40/60/80/120`) as pure JPS
33. Pushed `maze512-1-0` into long-tail release coverage as well: `bucket 200/400/800/1211` all solve as pure JPS with exact scenario-optimal path lengths, so the current maze family check now reaches the scenario maximum bucket
34. Re-ran the short criterion benchmark on the expanded 4-family bucket-80 subset (`arena2`, `8room_000`, `random512-10-0`, `maze512-1-0`) and confirmed `JPS fallback rate (all scenarios) = 0/7`; runtime is now clearly family-dependent (`JPS` slower than `A*` on `arena2` and `maze512-1-0`, roughly tied or slightly faster on `8room_000/random512-10-0`)
35. Vendored a `street` family sample (`Berlin_0_512`) and added sampled release coverage through its maximum bucket (`0/20/40/60/80/120/160/186`); all 8 cases solve as pure JPS with exact scenario-optimal path lengths, so coverage is now `dao / room / random / maze / street`
36. Added a release runtime snapshot on max-bucket representatives across the 5 vendored families; `JPS` only beats `A*` on `arena2/bucket90`, while `8room_000/bucket213`, `random512-10-0/bucket177`, `maze512-1-0/bucket1211`, and `Berlin_0_512/bucket186` are all tied or slower, so long-tail runtime is also family-dependent
37. Added a sampled crossover runtime sweep across the 5 vendored families using one representative scenario per bucket; the current flips are `arena2: bucket 80-85`, `8room_000: 80-120`, and `random512-10-0: 120-140`, `maze512-1-0` stays `A*`-faster through `bucket1211`, and `Berlin_0_512` is non-monotonic across sampled buckets, so the next runtime step is bucket-internal scenario aggregation rather than just more sparse bucket points
38. Replaced the representative-only view with a 3-scenario bucket aggregate (`slot 0/4/9`) across the current runtime windows; this materially changed the story: `arena2` no longer flips inside `bucket 80-85`, `8room_000` narrows to `80-90`, `random512-10-0` narrows to `130-135`, `maze512-1-0` remains consistently `A*`-faster, and `Berlin_0_512` stays non-monotonic but trends more `JPS`-friendly at high buckets
39. Added full `10`-scenario aggregates on the narrowed runtime windows; this locks down `arena2` to `bucket 89-90`, keeps `8room_000` in a tight `85-90` band, keeps `random512-10-0` in `132-135`, and flips the `Berlin_0_512` story again because its full-bucket medians are `JPS`-faster on every checked high bucket (`120/140/160/186`)

### What to do next (priority order)

1. **Add more vendored MovingAI maps before making a paper claim** -- correctness and bucket coverage are much better, but geometry coverage is still one map per family even after adding `street`
2. **Densify the remaining full-bucket crossover windows** -- the current best windows are now `arena2: 89-90`, `8room_000: 85-90`, and `random512-10-0: 132-135`; these are small enough that dense full-bucket checks will settle them quickly
3. **Map the lower-bucket onset for `Berlin_0_512`** -- full-bucket medians on `120/140/160/186` are all `JPS`-faster, so the remaining question is where that advantage begins
4. **Decide how to report current `JPS`** -- on the current short-benchmark subset, sampled bucket sweep, 3-scenario aggregate sweep, full-bucket narrowed-window sweep, long-tail checks, maze max-bucket checks, and `street` sampled sweep it is a pure baseline again, but the performance claim must now be conditioned on map family, bucket range, and scenario variance
5. **Strengthen any-angle baselines** -- implement `Lazy Theta*` first, then add `Anya` or any-angle `Subgoal Graph` so `Theta*` is not the strongest any-angle comparator by default
6. **Re-run unified planning benchmark on standard maps** -- compare the grid-optimal family and the any-angle family on the same harness, but do not collapse them into one story
7. **Choose a real JPS research branch** -- either static-map `JPS+` or weighted-grid `JPSW`; do not stop at a crossover-only study
8. **Keep DFS+smoothing and BidirectionalA* optimization as engineering tasks** -- useful for the crate, but weaker as a main paper claim
9. **Run broader CKF vs UKF comparison only if the paper focus shifts to localization** -- otherwise keep planning as the primary story
10. **Generate paper figures after standard-map data lands** -- avoid locking in plots from the current synthetic-only benchmark
11. **Write paper after the benchmark scope is upgraded** -- current results are better suited for an internal benchmark note than an ICRA main-track submission

### Important conventions

- No `Co-Authored-By` in commits (user preference)
- No AI generation tags in PRs
- Cool, flat tone for papers (no hype, no aggressive language)
- `#[forbid(unsafe_code)]` in all crates
- Docstring `[m]` needs escaping as `\[m\]` for rustdoc
- CI runs clippy with `-D warnings`
- Tests should use `--test-threads=1` for reliability

---

## Claude Handoff (2026-04-04)

### Read this first

The older handoff above is still useful, but it stops at the point where the work was still mostly "planning/JPS/MovingAI benchmark cleanup + paper-oriented algorithm comparison."  
As of 2026-04-02 ~ 2026-04-04, the active center of gravity changed.

The current project story is no longer:

- "find the one best exploratory proxy"
- "pick the right benchmark shortcut and standardize on it"
- "push a single concrete implementation into core"

The current story is:

- build multiple disposable comparison strategies under one shared contract
- compare them on multiple packages and multiple problem presets
- only promote the minimal comparison contract into stable core
- keep concrete exploratory strategies in `experiments/`
- stop exploration when the evidence is already strong enough to reject "single concrete winner"

If there is any conflict between:

- old notes in this file
- older markdown notes under `../notes/`
- earlier conversational summaries

then trust, in this order:

1. generated package/workspace summary docs under `docs/`
2. machine-checked workspace guard tests
3. the shared contract in `rust_robotics_core`
4. older notes and chat history

### Executive conclusion

#### Confirmed conclusion

For this workspace, the evidence is now strong that **no single concrete exploratory proxy should be promoted into core**.

What should be promoted into core is only the shared experiment contract:

- sampling-plan description
- observation/summary schema
- reference annotation
- cross-package summary flow
- machine-checked stop-condition guard

What should *not* be promoted into core:

- `first-scenario`
- `sampled-bucket`
- `percentile-bucket`
- `variance-triggered`
- any other single "cheap but good enough" exploratory strategy

#### Interpretation

Internally, this is already a strong architectural result.

Externally, this is **not yet a strong-accept paper result**.  
The missing piece is a downstream metric, such as:

- review latency
- bug catch rate
- rework rate
- wrong-choice rate on real tasks
- time-to-decision under different experiment policies

In other words:

- internal architecture/governance claim: strong
- external research contribution claim: not strong enough yet

### What was built after the earlier JPS/MovingAI phase

The repo was reoriented around an experiment-driven package-development workflow.

The stable shared contract now lives in:

- `crates/rust_robotics_core/src/experiments.rs`

That shared contract is reused by at least these sibling experiment systems:

- planning: `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`
- localization: `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`
- control: `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`
- mapping: `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`

The key design move was:

- keep the reusable comparison grammar in `rust_robotics_core`
- keep all concrete exploratory proxies package-local and disposable

This is the main architectural shift Claude should preserve.

### Shared experiment contract

The current stable contract is intentionally small. It supports:

- multiple concrete sampling/comparison variants
- a per-problem descriptor
- a per-window or per-bucket observation
- a full-reference outcome
- cheap-proxy outcomes
- derived comparison metrics
- generated markdown summaries
- workspace-level aggregation

This contract is deliberately more stable than any concrete proxy.

The practical rule is:

- if two packages can share a type without sharing a concrete winner, it belongs in core
- if a type or policy assumes one exploratory strategy is generally best, it does **not** belong in core

### Concrete variants currently in play

The current reusable family of comparison variants includes:

- `first-scenario`
- `sampled-bucket`
- `percentile-bucket`
- `variance-triggered`
- `full-bucket`

`full-bucket` acts as reference.  
The other four are exploratory policies.  
The important result is not that one of these wins. The important result is that **the winner rotates by problem and by package**.

### Package-level results

#### Planning

Planning summary docs:

- `docs/experiments_planning_summary.md`
- `docs/decisions_planning_summary.md`

Planning currently validates multiple presets, including:

- `crossover-windows`
- `long-tail-windows`
- `synthetic-local-grids`

Current planning best-proxy rotation from the latest generated docs:

- `first-scenario x2`
- `variance-triggered x1`

Important implication:

- planning does not support promoting `percentile-bucket` or `sampled-bucket` as a universal choice
- planning also changed across reruns, so old conversational numbers are less trustworthy than the current generated docs

Planning still carries the older JPS/MovingAI benchmark history, but that is now a package-specific evidence source inside the larger experiment-governance story, not the whole project story.

#### Localization

Localization summary docs:

- `docs/experiments_localization_summary.md`
- `docs/decisions_localization_summary.md`

Localization now has a broad preset suite, including:

- noise windows
- long horizon
- dropout/bias
- outlier burst
- process mismatch
- sensor-rate mismatch
- control latency
- process-noise anisotropy
- sensor bias burst
- actuator saturation

Current localization best-proxy rotation from the latest generated docs:

- `first-scenario x4`
- `sampled-bucket x1`
- `percentile-bucket x3`
- `variance-triggered x2`

Localization alone is enough to reject "one fixed concrete winner for all conditions."

#### Control

Control summary docs:

- `docs/experiments_control_summary.md`
- `docs/decisions_control_summary.md`

Current control presets include:

- `control-tracking-windows`
- `control-actuation-mismatch-windows`

Current control best-proxy rotation from the latest generated docs:

- `percentile-bucket x2`

Control is the main place where one concrete policy looks stable *inside that package*.  
That is not enough to promote it into workspace core, because the other packages disagree.

#### Mapping

Mapping summary docs:

- `docs/experiments_mapping_summary.md`
- `docs/decisions_mapping_summary.md`

Current mapping presets include:

- `mapping-point-cloud-sampling-windows`
- `mapping-occlusion-corruption-windows`
- `mapping-density-shift-windows`
- `mapping-anisotropic-noise-windows`
- `mapping-sparse-outlier-burst-windows`
- `mapping-resolution-ladder-windows`

Current mapping best-proxy rotation from the latest generated docs:

- `sampled-bucket x2`
- `percentile-bucket x2`
- `variance-triggered x2`

Mapping is particularly useful because it prevents overfitting the contract to planning/localization only.

### Workspace-level result

The workspace summary is now the most important evidence artifact.

Read these first:

- `docs/experiments_workspace_summary.md`
- `docs/decisions_workspace_summary.md`

Current latest generated workspace numbers:

- `4` packages
- `21` validated presets
- strongest concrete winner: `percentile-bucket (7/21)`
- `cheapest smoke test` mismatch: `13/21`
- best proxy still imperfect: `3/21`

Current workspace best-proxy rotation from the latest generated docs:

- `first-scenario x6`
- `sampled-bucket x3`
- `percentile-bucket x7`
- `variance-triggered x5`

This is the key anti-convergence result.

It means:

- even the most frequent winner only explains `7/21`
- the cheapest shortcut fails often enough that it cannot be governance-default
- even the best-per-preset choice is not perfectly stable against the full reference

So the correct stable policy is:

- standardize the contract
- do not standardize one concrete exploratory proxy

### Machine-checked stop condition

This part matters because it converts the conclusion from narrative to enforcement.

The current workspace guard lives in:

- `crates/rust_robotics_core/tests/workspace_summary_guard.rs`

That test suite now checks both:

- actual current docs/status
- synthetic failure injection

It is not just "does the summary exist?"  
It also checks that the governance conclusion would fail if the workspace ever collapsed to:

- a single winner everywhere
- a universal cheapest smoke test
- missing guard language
- broken cross-package summary registration

The current stop-condition evidence is therefore encoded in two places:

1. generated workspace summary markdown
2. executable guard tests

This is important for Claude because it means the conclusion should not be weakened casually in prose without also changing tests.

### Generated docs and generators

Current generators worth knowing:

- planning/package + interfaces side: `crates/rust_robotics_planning/examples/update_moving_ai_runtime_docs.rs`
- localization/package side: `crates/rust_robotics_localization/examples/update_ukf_ckf_accuracy_docs.rs`
- control/package side: `crates/rust_robotics_control/examples/update_path_tracking_accuracy_docs.rs`
- mapping/package side: `crates/rust_robotics_mapping/examples/update_point_cloud_sampling_docs.rs`
- workspace summary: `crates/rust_robotics_core/examples/update_workspace_experiment_summary.rs`

Current important generated docs:

- `docs/interfaces.md`
- `docs/experiments_planning_summary.md`
- `docs/decisions_planning_summary.md`
- `docs/experiments_localization_summary.md`
- `docs/decisions_localization_summary.md`
- `docs/experiments_control_summary.md`
- `docs/decisions_control_summary.md`
- `docs/experiments_mapping_summary.md`
- `docs/decisions_mapping_summary.md`
- `docs/experiments_workspace_summary.md`
- `docs/decisions_workspace_summary.md`

If Claude updates the process story, these files are the canonical output surface.

### What should not be redone

Claude should **not** spend time re-solving these questions unless there is a real regression:

- whether a single concrete exploratory proxy can be treated as universal
- whether planning alone is enough to define the shared comparison contract
- whether localization/control/mapping reuse was only hypothetical
- whether workspace-level stop conditions need enforcement

Those questions have already been answered strongly enough for internal governance.

### What is still unresolved

The big unresolved piece is external value.

We still do **not** have a strong answer to:

- does this process improve developer throughput?
- does it reduce wrong architectural decisions?
- does it catch more mistakes earlier?
- does it reduce review or rework cost?

That is why the current state is:

- internal strong architectural evidence: yes
- external strong-accept paper evidence: no

### Best next step

If Claude continues this line of work, the next task should **not** be "add more internal presets forever."

The next task should be a downstream metric experiment, something like:

- `workspace_decision_quality`

Suggested design:

- compare several experiment-governance policies on real or replayable tasks
- at minimum compare:
  - cheapest-only
  - fixed `percentile-bucket`
  - fixed `variance-triggered`
  - full-reference-only
  - contract-driven adaptive selection
- measure:
  - decision cost
  - wrong-choice rate
  - review latency
  - bug catch rate
  - rework cost

Suggested output files:

- `docs/experiments_workspace_decision_quality.md`
- `docs/decisions_workspace_decision_quality.md`

If that lands, the story can move from:

- "we should not standardize one proxy"

to:

- "contract-only promotion improves decision quality / cost tradeoff over fixed proxy promotion"

That would be much closer to an externally strong result.

### Recommended verification commands

If Claude needs to refresh the current state, the minimum useful commands are:

```bash
# Planning/package summaries + interfaces sync
cargo run -p rust_robotics_planning --example update_moving_ai_runtime_docs --release

# Localization summaries
cargo run -p rust_robotics_localization --example update_ukf_ckf_accuracy_docs

# Control summaries
cargo run -p rust_robotics_control --example update_path_tracking_accuracy_docs

# Mapping summaries
cargo run -p rust_robotics_mapping --example update_point_cloud_sampling_docs --release

# Workspace summary
cargo run -p rust_robotics_core --example update_workspace_experiment_summary

# Guard tests
cargo test -p rust_robotics_core -- --nocapture
```

### Current practical stopping rule

For internal repo governance, exploration can be considered "done enough" while all of the following remain true:

- cross-package winner rotation persists
- strongest concrete winner is still far from universal
- cheapest smoke test is still materially unsafe as default
- the workspace guard remains green

At that point, continuing to add internal presets yields diminishing returns.

### One-line handoff

Claude should treat this repo as having **already crossed the line from algorithm benchmarking into experiment-governed package design**.  
The stable thing is the shared comparison contract and the summary/guard machinery.  
The unstable thing is every concrete exploratory proxy.  
The next meaningful contribution is downstream decision-quality evidence, not more internal preset count.
