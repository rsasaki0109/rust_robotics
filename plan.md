# RustRobotics Development Plan — Phase 2

## Overview

All algorithm waves (1-3) and post-wave additions are complete. This plan covers the next phase of work:

- **Task A**: Create PR to merge `feature/more-algorithms-wave4` → `main`
- **Task B**: Benchmark new algorithms with performance comparisons
- **Task C**: Add gnuplot visualization examples for new algorithms
- **Task D**: Wave 4 — implement 8 more algorithms
- **Task E**: Expand dora-rs real-time planning integration
- **Task F**: Complete the A*/Fringe/IDA* experiment track

**Branch**: `feature/more-algorithms-wave4`
**All lib.rs changes for existing modules are done. Tests pass. Clippy clean.**

---

## Task A: PR & Merge

### Goal
Create a clean PR from `feature/more-algorithms-wave4` → `main`.

### Steps

1. Run full CI locally first:
```bash
cargo fmt --all -- --check
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
```

2. Clean up any temporary files:
```bash
rm -rf crates/rust_robotics/examples/out/
rm -rf logs/
rm -f scripts/demo_wave1.sh scripts/wave1_implement.sh
```

3. Update README.md — add the new algorithms to the algorithm table. The current README lists the original algorithms but not the 26 new ones. Add them to the appropriate sections:

**Localization section — add:**
- Complementary Filter
- Iterated EKF (IEKF)
- Information Filter
- Square Root UKF (SR-UKF)

**Control section — add:**
- PID Controller
- Sliding Mode Control
- Feedback Linearization
- Backstepping Control
- iLQR (iterative LQR)

**Planning section — add:**
- Bidirectional RRT
- Tangent Bug
- RRG (Rapidly-exploring Random Graph)
- LPA* (Lifelong Planning A*)
- ARA* (Anytime Repairing A*)
- FMT* (Fast Marching Tree)
- Bipedal Planner
- RRT Sobol
- RRT Path Smoothing
- A* Variants (Beam, Iterative, Dynamic)
- Fringe Search
- IDA*

**Mapping section — add:**
- DBSCAN Clustering
- Line Extraction (Split-and-Merge)
- Occupancy Grid Map (Log-Odds)

**SLAM section — add:**
- Pose Graph Optimization
- Correlative Scan Matching

4. Push and create PR:
```bash
git add -A
git commit -m "Add 26 new algorithms (Wave 1-3 + extras), benchmarks, experiments, dora-rs integration"
git push -u origin feature/more-algorithms-wave4
gh pr create --title "Add 26 new robotics algorithms" --body "$(cat <<'EOF'
## Summary
- 26 new algorithms across all domains (localization, control, planning, mapping, SLAM)
- Wave 1 (7): PID, DBSCAN, Complementary Filter, Bidirectional RRT, Tangent Bug, Line Extraction, IEKF
- Wave 2 (7): RRG, Occupancy Grid, Information Filter, Sliding Mode, Feedback Linearization, LPA*, ARA*
- Wave 3 (6): SR-UKF, Backstepping, FMT*, Pose Graph Optimization, Correlative Scan Matching, iLQR
- Post-wave (6): Bipedal Planner, RRT Sobol, RRT Path Smoothing, A* Variants, Fringe Search, IDA*
- All tests passing, clippy clean
- Experiment infrastructure for runtime aggregation and grid threshold comparison
- dora-rs integration demo

## Test plan
- [ ] cargo test --workspace passes
- [ ] cargo clippy --workspace -- -D warnings clean
- [ ] All headless examples run
- [ ] README updated with new algorithm list
EOF
)"
```

---

## Task B: Benchmarks

### Goal
Create a benchmark example that compares performance of related algorithms. Output a markdown table to stdout.

### B1: Grid Planner Benchmark
**File**: `crates/rust_robotics/examples/benchmark_grid_planners.rs`
**Features**: `planning`

Compare on the same 100x100 grid with obstacles:
- A* vs Dijkstra vs JPS vs Theta* vs Lazy Theta* vs LPA* vs ARA* vs Fringe Search vs IDA*

Measure: planning time (µs), path length, nodes expanded.

```rust
// Pseudocode structure
fn main() {
    let (ox, oy) = build_test_obstacles(); // 100x100 grid with wall
    let start = Point2D::new(10.0, 10.0);
    let goal = Point2D::new(90.0, 90.0);

    let planners: Vec<(&str, Box<dyn Fn() -> RoboticsResult<Path2D>>)> = vec![
        ("A*", Box::new(|| { /* create and plan */ })),
        ("Dijkstra", Box::new(|| { /* ... */ })),
        // ...
    ];

    println!("| Planner | Time (µs) | Path Length | Nodes |");
    println!("|---------|-----------|-------------|-------|");
    for (name, plan_fn) in &planners {
        let start_time = std::time::Instant::now();
        let path = plan_fn().unwrap();
        let elapsed = start_time.elapsed().as_micros();
        println!("| {} | {} | {:.1} | - |", name, elapsed, path.total_length());
    }
}
```

Add to `Cargo.toml`:
```toml
[[example]]
name = "benchmark_grid_planners"
required-features = ["planning"]
```

### B2: Localization Filter Benchmark
**File**: `crates/rust_robotics/examples/benchmark_localizers.rs`
**Features**: `localization`

Compare on the same simulated trajectory (circular motion with noise):
- EKF vs UKF vs CKF vs IEKF vs Complementary Filter vs Information Filter vs SR-UKF

Measure: final position error (m), average update time (µs).

### B3: Sampling Planner Benchmark
**File**: `crates/rust_robotics/examples/benchmark_sampling_planners.rs`
**Features**: `planning`

Compare: RRT vs RRT* vs Bidirectional RRT vs RRG vs FMT* vs BIT*

Measure: planning time (ms), path length, success rate (over 50 runs).

### B4: Update README with benchmark results
After running benchmarks, add a "Performance" section to README.md with the tables.

---

## Task C: Visualization Examples

### Goal
Add gnuplot examples for key new algorithms. Each example should generate a PNG in `img/`.

### C1: PID Controller Step Response
**File**: `crates/rust_robotics/examples/pid_controller.rs`
**Features**: `control`, `viz`

Plot the step response: time vs output, showing P-only, PI, PID, PID+anti-windup.
Save to `img/control/pid_step_response.png`

### C2: DBSCAN vs K-means Comparison
**File**: `crates/rust_robotics/examples/dbscan_vs_kmeans.rs`
**Features**: `mapping`, `viz`

Generate crescent-shaped clusters. Show DBSCAN correctly separating them while K-means fails.
Save to `img/mapping/dbscan_vs_kmeans.png`

### C3: Bidirectional RRT Tree Growth
**File**: `crates/rust_robotics/examples/bidirectional_rrt.rs`
**Features**: `planning`, `viz`

Plot both trees (start=blue, goal=red) and the connecting path (green).
Save to `img/path_planning/bidirectional_rrt.png`

### C4: iLQR Trajectory Optimization
**File**: `crates/rust_robotics/examples/ilqr_trajectory.rs`
**Features**: `control`, `viz`

Plot initial guess trajectory vs optimized trajectory around obstacles.
Save to `img/control/ilqr_trajectory.png`

### C5: Filter Comparison
**File**: `crates/rust_robotics/examples/filter_comparison.rs`
**Features**: `localization`, `viz`

Plot ground truth vs EKF vs IEKF vs SR-UKF estimates on the same trajectory.
Save to `img/localization/filter_comparison.png`

### C6: Occupancy Grid Map Building
**File**: `crates/rust_robotics/examples/occupancy_grid_demo.rs`
**Features**: `mapping`, `viz`

Simulate a robot scanning with LIDAR, show the occupancy grid being built.
Save to `img/mapping/occupancy_grid.png`

For each example, add the corresponding `[[example]]` entry to `crates/rust_robotics/Cargo.toml`.

---

## Task D: Wave 4 — 8 New Algorithms

### Goal
Add 8 more algorithms that further distinguish this project from PythonRobotics.

**Add `pub mod` to the appropriate lib.rs files first.**

### D1: RRT-Connect
**File**: `crates/rust_robotics_planning/src/rrt_connect.rs`
**Reference**: `bidirectional_rrt.rs`, `rrt.rs`

RRT-Connect (Kuffner & LaValle 2000) aggressively extends both trees toward each other using CONNECT (not just EXTEND). Faster convergence than Bidirectional RRT.

Algorithm: Each iteration, one tree does EXTEND toward random sample. If success, other tree does CONNECT (repeatedly EXTEND) toward the new node. Swap trees. Terminate when CONNECT reaches the other tree.

Config: `expand_dis(3.0)`, `path_resolution(0.5)`, `max_iter(500)`, `robot_radius(0.8)`
Tests (3): no obstacles, with obstacles, faster than bidirectional RRT on same problem

### D2: CHOMP (Covariant Hamiltonian Optimization for Motion Planning)
**File**: `crates/rust_robotics_planning/src/chomp.rs`

Trajectory optimization that minimizes a functional of smoothness + obstacle cost using covariant gradient descent.

Config: `n_waypoints(50)`, `dt(0.1)`, `max_iterations(100)`, `learning_rate(0.01)`, `obstacle_cost_weight(1.0)`, `smoothness_weight(1.0)`
Input: start Point2D, goal Point2D, obstacles Vec<CircleObstacle>
Output: Path2D (optimized trajectory)

Algorithm:
1. Initialize straight-line trajectory from start to goal
2. For each iteration:
   - Compute smoothness cost gradient (finite differences of acceleration)
   - Compute obstacle cost gradient (distance field to obstacles)
   - Update trajectory: path -= learning_rate * (smoothness_grad + obstacle_weight * obstacle_grad)
   - Fix start and goal points
3. Return optimized path

Tests (3): converges, avoids obstacles, smoother than RRT path

### D3: DDP (Differential Dynamic Programming)
**File**: `crates/rust_robotics_control/src/ddp.rs`
**Reference**: `ilqr.rs`

DDP extends iLQR by including second-order dynamics terms (f_xx, f_ux, f_uu). For the unicycle model these are zero, so implement with a double-integrator or car-like model where they matter.

Use a simple car model: state=[x, y, theta, v], control=[accel, steer]

Config: `horizon(50)`, `dt(0.1)`, `max_iterations(50)`, `tolerance(1e-6)`, state/control cost weights
Tests (4): converges, reaches goal, cost decreases, config defaults

### D4: Gaussian Process Regression (for mapping)
**File**: `crates/rust_robotics_mapping/src/gaussian_process.rs`

GP regression for terrain/surface mapping from sparse measurements.

Config: `length_scale(1.0)`, `signal_variance(1.0)`, `noise_variance(0.01)`
Input: training points (x, y, z), query points (x, y)
Output: predicted z values + uncertainty at query points

Algorithm: Standard GP regression with RBF kernel.
- K = kernel_matrix(train, train)
- K_star = kernel_matrix(query, train)
- mean = K_star * (K + noise*I)^-1 * z_train
- variance = K_star_star - K_star * (K + noise*I)^-1 * K_star^T

Tests (3): predicts known function, uncertainty increases away from data, config defaults

### D5: Rapidly-exploring Random Belief Trees (RRBT)
**File**: `crates/rust_robotics_planning/src/rrbt.rs`

Planning under uncertainty: extends RRT to plan in belief space (mean + covariance).

Simplified version: Each node stores (x, y, covariance_trace). Edge cost includes both distance and uncertainty growth. Use EKF-style prediction for covariance propagation.

Config: `expand_dis(3.0)`, `max_iter(500)`, `process_noise(0.1)`, `measurement_noise(0.5)`, observation landmarks
Tests (3): finds path, path prefers observable regions, config defaults

### D6: Rapidly-exploring Random Tree Star with informed sampling (RRT*-Informed) with path heuristic
**File**: Already exists as `informed_rrt_star.rs` — SKIP, replace with:

### D6: Hybrid A* (if not already complete, verify and enhance)
Check if `hybrid_a_star.rs` exists and has tests. If tests are missing, add comprehensive tests.

### D7: Monte Carlo Localization (MCL / Adaptive Particle Filter)
**File**: `crates/rust_robotics_localization/src/monte_carlo_localization.rs`
**Reference**: `particle_filter.rs`

MCL with adaptive particle count (KLD-sampling). Adjusts number of particles based on the complexity of the posterior distribution.

Config: `min_particles(100)`, `max_particles(5000)`, `kld_epsilon(0.05)`, `kld_z(2.326)` (99% quantile)
Same motion/observation model as particle filter.
Key difference: after resampling, compute KLD-based particle count for next iteration.

Tests (4): converges, adaptive count grows with multimodal distribution, decreases with convergence, config defaults

### D8: Probabilistic Road Map Star (PRM*)
**File**: `crates/rust_robotics_planning/src/prm_star.rs`
**Reference**: `prm.rs`

PRM with asymptotically optimal connection radius: r_n = gamma * (log(n)/n)^(1/d)

Config: `n_samples(500)`, `robot_radius(0.8)`, `gamma(2.5)` (connection radius scale)
Tests (3): finds path, path quality improves with more samples, config defaults

---

## Task E: dora-rs Integration

### Goal
Expand the existing dora-rs demo into a more complete planning + localization pipeline.

### Current state
- `dora_path_planning_node.rs` — planner node (exists)
- `dora_path_planning_sink.rs` — sink node (exists)
- `dora_path_metrics_node.rs` — metrics node (exists)

### E1: Add EKF Localization Node
**File**: `crates/rust_robotics/examples/dora_ekf_node.rs`

A dora-rs node that:
1. Receives control input (v, omega) on input `control`
2. Receives noisy position measurement on input `measurement`
3. Runs EKF predict + update
4. Publishes estimated state on output `state_estimate`

### E2: Add Full Navigation Pipeline Dataflow
**File**: `crates/rust_robotics/examples/dora_navigation.yaml`

A dora-rs dataflow YAML connecting:
- planner node → control node → EKF node → planner (replanning loop)

### E3: Document dora-rs setup
**File**: `docs/dora_integration.md`

Document:
- How to install dora-rs
- How to run the planning demo
- How to run the full navigation pipeline
- Architecture diagram (text-based)

---

## Task F: Complete Experiment Tracks

### Goal
Finalize the A*/Fringe/IDA* comparison and produce a summary.

### F1: Run the grid threshold comparison
Execute the comparison on 5 MovingAI maps (different sizes), collecting:
- Planning time
- Path length (optimality gap vs A*)
- Memory usage (node count)

### F2: Write summary
**File**: `docs/planning_comparison_summary.md`

Create a clear summary with tables:

| Planner | Map Size | Time (ms) | Path Length | Optimal? | Memory |
|---------|----------|-----------|-------------|----------|--------|
| A*      | 100x100  | ...       | ...         | Yes      | ...    |
| Fringe  | 100x100  | ...       | ...         | Yes      | ...    |
| IDA*    | 100x100  | ...       | ...         | Yes      | ...    |

Include analysis: when to use which algorithm.

### F3: Add to README
Add a "Algorithm Comparison" section to README.md with a link to the detailed report.

---

## Verification

After completing each task, run:
```bash
cargo fmt --all
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
```

---

## File Conventions Reminder

- Config struct with `Default` impl and `validate()` method
- Constructors: `new()` (panics), `try_new()` (returns Result)
- Inline `#[cfg(test)] mod tests` with 3-5 tests
- `#![forbid(unsafe_code)]` is at crate level
- Doc comments on public items, escape brackets: `\[m\]`, `\[rad\]`
- Wrap bare URLs: `<https://...>`
- Use `rust_robotics_core` types and traits
- Use `nalgebra` for matrix operations

## Reference Files

| Pattern | Reference file |
|---------|---------------|
| Grid planner | `crates/rust_robotics_planning/src/a_star.rs` |
| Sampling planner | `crates/rust_robotics_planning/src/rrt_star.rs` |
| Localization filter | `crates/rust_robotics_localization/src/ekf.rs` |
| Control (simulate) | `crates/rust_robotics_control/src/move_to_pose.rs` |
| Control (trajectory opt) | `crates/rust_robotics_control/src/ilqr.rs` |
| Mapping | `crates/rust_robotics_mapping/src/kmeans_clustering.rs` |
| SLAM | `crates/rust_robotics_slam/src/icp_matching.rs` |
| Viz example | `crates/rust_robotics/examples/a_star.rs` |
| Benchmark example | `crates/rust_robotics/examples/speed_comparison.rs` |
