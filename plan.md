# RustRobotics Research Reproduction Plan

Updated: 2026-06-04

This plan tracks the current pure-Rust reproduction effort for robotics papers
whose official code is unavailable, delayed, or too heavy to integrate directly.
The goal is to turn each paper idea into deterministic library code, examples,
benchmarks, SVG artifacts, and focused tests that fit this repository.

## Current Direction

The active direction is reproducible robotics algorithms with a visible artifact
and a small testable core. ROS2-only integration work is intentionally skipped
for now unless there is a standalone Rust algorithmic slice.

The current emphasis is:

1. Planning and control ideas that can run without large simulators.
2. Multi-robot planning, formal constraints, and kinodynamic occupancy.
3. MPPI-style control extensions with measurable behavior changes.
4. Geometry-heavy planning such as rigid-body MIP-style collision avoidance.
5. Gallery-ready SVG assets so every reproduction has a visual result.

## Implemented Reproduction Slices

### Conformal SIPP

Source theme: time-aware motion planning with conformal prediction.

Implemented:

- Conformal safe-interval planner over predicted dynamic obstacle trajectories.
- Coverage-aware interval inflation and risk-aware safe interval extraction.
- Headless demo and benchmark.
- Documentation page for the reproduction slice.

Primary files:

- `crates/rust_robotics_planning/src/conformal_sipp.rs`
- `crates/rust_robotics/examples/headless_conformal_sipp.rs`
- `crates/rust_robotics/examples/benchmark_conformal_sipp.rs`
- `docs/conformal_sipp_reproduction.md`

Next useful extension:

- Add calibration-set utilities and benchmark coverage-vs-delay tradeoffs.

### Traversal Risk Graph

Source theme: TRG-style traversal risk path planning in unstructured terrain.

Implemented:

- Terrain risk cells with traversability, stability, and exposure costs.
- Elevation-derived risk map.
- Risk smoothing.
- Clearance exposure and obstacle inflation.
- Risk-weight Pareto sweep with CSV and SVG output.

Primary files:

- `crates/rust_robotics_planning/src/traversal_risk_graph.rs`
- `crates/rust_robotics/examples/headless_traversal_risk_graph.rs`
- `crates/rust_robotics/examples/headless_elevation_risk_graph.rs`
- `crates/rust_robotics/examples/headless_risk_map_smoothing.rs`
- `crates/rust_robotics/examples/headless_clearance_risk_graph.rs`
- `crates/rust_robotics/examples/render_traversal_risk_graph_svg.rs`
- `crates/rust_robotics/examples/benchmark_traversal_risk_sweep.rs`
- `docs/assets/traversal-risk-graph-demo.svg`
- `docs/assets/traversal-risk-weight-sweep.csv`
- `docs/assets/traversal-risk-weight-sweep.svg`

Next useful extension:

- Add larger terrain maps and compare against shortest-path baselines.

### Adaptive Costmap NAMO

Source theme: path planning in partially known environments with movable
obstacles.

Implemented:

- Adaptive movable-obstacle costmap.
- Motion progress observations.
- Replanning behavior around movable/blocked cells.
- Headless demo and reproduction doc.

Primary files:

- `crates/rust_robotics_planning/src/adaptive_costmap_namo.rs`
- `crates/rust_robotics/examples/headless_adaptive_costmap_namo.rs`
- `docs/adaptive_costmap_namo_reproduction.md`

Next useful extension:

- Add multi-obstacle scenarios and compare static-vs-adaptive planning cost.

### TD-CD-MPPI Family

Source theme: MPPI control with constraint discounting, learned value terms, and
adaptive temperature behavior.

Implemented:

- General MPPI rollout controller.
- Constraint discounting.
- Terminal value grid.
- Online value learning and replay value learning.
- Adaptive lambda.
- Track-progress terminal objective.
- Multiple headless examples and SVG renderers.

Primary files:

- `crates/rust_robotics_control/src/mppi.rs`
- `crates/rust_robotics/examples/headless_mppi_double_integrator.rs`
- `crates/rust_robotics/examples/headless_mppi_constraint_discount.rs`
- `crates/rust_robotics/examples/headless_mppi_terminal_value.rs`
- `crates/rust_robotics/examples/headless_mppi_value_learning.rs`
- `crates/rust_robotics/examples/headless_mppi_replay_value_learning.rs`
- `crates/rust_robotics/examples/headless_mppi_adaptive_temperature.rs`
- `crates/rust_robotics/examples/headless_mppi_track_progress.rs`
- `crates/rust_robotics/examples/render_mppi_value_grid_svg.rs`
- `crates/rust_robotics/examples/render_mppi_track_progress_svg.rs`
- `docs/assets/mppi-replay-value-grid.svg`
- `docs/assets/mppi-track-progress.svg`

Next useful extension:

- Add benchmark tables for solve time, constraint violation, and terminal cost.

### Racing MPPI

Source theme: reference-free agile racing through gates.

Implemented:

- Gate-progress reward.
- Oriented gate crossing logic.
- Reference-free MPPI objective.
- Waypoint-reference baseline comparison.
- Headless and SVG examples.

Primary files:

- `crates/rust_robotics_control/src/mppi.rs`
- `crates/rust_robotics/examples/headless_mppi_racing_gate_progress.rs`
- `crates/rust_robotics/examples/render_mppi_racing_gate_progress_svg.rs`
- `docs/assets/mppi-racing-gate-progress.svg`

Next useful extension:

- Extend from 2-D oriented gates to 3-D gate planes and richer dynamics.

### Adap-RPF-lite MPPI

Source theme: adaptive trajectory sampling for robot person following.

Implemented:

- Target-centric following-point sampler.
- Visibility, proximity, distance, travel, and stickiness scoring.
- Moving pedestrians.
- Horizon goal trajectories.
- Prediction-aware MPPI with moving obstacle cost.
- Headless and SVG examples.

- `PersonFollowingMetricsAccumulator2D` rollout counters: target visibility
  ratio, target spacing ratio (plus mean spacing/error), min pedestrian
  clearance, and collision steps.
- Occlusion and moving-pedestrian benchmark sweep comparing fixed back-point vs
  adaptive following, with seeded deterministic scenes and CSV/SVG output.

Primary files:

- `crates/rust_robotics_control/src/person_following_mppi.rs`
- `crates/rust_robotics_control/src/mppi.rs`
- `crates/rust_robotics/examples/headless_adap_rpf_mppi.rs`
- `crates/rust_robotics/examples/render_adap_rpf_mppi_svg.rs`
- `crates/rust_robotics/examples/benchmark_adap_rpf_metrics.rs`
- `docs/assets/adap-rpf-lite-mppi.svg`
- `docs/assets/adap-rpf-metrics-sweep.{csv,svg}`
- `docs/adap_rpf_mppi_reproduction.md`

Next useful extension:

- Sweep follower weights (visibility vs proximity vs stickiness) and report the
  Pareto front of visibility ratio against spacing ratio.

### BranchOut-lite

Source theme: multimodal autonomous driving decisions.

Implemented:

- Lane-level driving scene.
- Keep, yield, lane-change-left, and lane-change-right modes.
- GMM-style mixture probabilities.
- Multimodal metrics including pairwise diversity, Frechet coverage, negative
  log likelihood, speed JSD, and expected route completion.
- Headless and SVG examples.

Primary files:

- `crates/rust_robotics_planning/src/branchout_multimodal.rs`
- `crates/rust_robotics/examples/headless_branchout_multimodal_driving.rs`
- `crates/rust_robotics/examples/render_branchout_multimodal_driving_svg.rs`
- `docs/assets/branchout-multimodal-driving.svg`
- `docs/branchout_multimodal_reproduction.md`

Next useful extension:

- Add closed-loop receding-horizon route completion and no-collision counters.

### STL-CBS

Source theme: cBOT / STL-KCBS style formal multi-robot planning.

Implemented:

- Grid CBS with vertex and edge-swap constraints.
- Time-expanded low-level A* with waits.
- Independent shortest-path conflict baseline.
- STL robustness primitives:
  eventually reach, always avoid, and pairwise separation.
- Headless and SVG examples.

Primary files:

- `crates/rust_robotics_planning/src/stl_cbs.rs`
- `crates/rust_robotics/examples/headless_stl_cbs_multi_robot.rs`
- `crates/rust_robotics/examples/render_stl_cbs_multi_robot_svg.rs`
- `docs/assets/stl-cbs-multi-robot.svg`
- `docs/stl_cbs_reproduction.md`

Next useful extension:

- Add richer STL task specifications and cost-map learning hooks.

### Kinodynamic STL-CBS

Source theme: kinodynamic multi-robot planning with formal occupancy checks.

Implemented:

- Heading-aware state `(x, y, heading, t)`.
- Time-consuming forward, turn, wait, and optional reverse primitives.
- Terminal heading constraints.
- CBS repair over oriented paths.
- Continuous-time pairwise occupancy checks between integer ticks.
- Exact closest-approach robustness over linear interpolated segments.
- Detection of between-tick conflicts even when integer-time cells do not
  collide.
- Headless and SVG examples.

Primary files:

- `crates/rust_robotics_planning/src/kinodynamic_stl_cbs.rs`
- `crates/rust_robotics/examples/headless_kinodynamic_stl_cbs.rs`
- `crates/rust_robotics/examples/render_kinodynamic_stl_cbs_svg.rs`
- `docs/assets/kinodynamic-stl-cbs.svg`
- `docs/kinodynamic_stl_cbs.md`

- Continuous-time conflicts are now promoted into high-level CBS branching
  constraints: when integer-time CBS is clean but a between-tick near-miss
  remains, the planner derives a vertex/edge constraint at the offending tick
  for each involved agent and replans. `KinodynamicStlCbsPlan2D` reports
  `continuous_conflicts_resolved`.

Current validation:

- `cargo test -p rust_robotics_planning kinodynamic_stl_cbs`
- `cbs_resolves_continuous_only_perpendicular_conflict` covers the
  integer-clean-but-continuously-unsafe case.
- Headless example reports integer and continuous separation robustness plus
  `continuous_resolved`.

Next useful extension:

- Add randomized multi-agent crossings and assert continuous robustness stays
  non-negative across a sweep.

### Hierarchical MAPF Replanning

Source theme: hierarchical large-scale multi-robot path / trajectory replanning.

Implemented:

- Independent shortest-path MAPF baseline.
- Coarse rectangular region hierarchy.
- Time-indexed region conflicts.
- Connected affected-agent grouping.
- Group-only CBS replanning with full-plan fallback.
- Four-agent visual demo.
- 50, 100, and 200 agent scale benchmark.
- CSV and SVG benchmark artifacts.

Primary files:

- `crates/rust_robotics_planning/src/hierarchical_mapf.rs`
- `crates/rust_robotics/examples/headless_hierarchical_mapf_replanning.rs`
- `crates/rust_robotics/examples/render_hierarchical_mapf_replanning_svg.rs`
- `crates/rust_robotics/examples/benchmark_hierarchical_mapf_scale.rs`
- `crates/rust_robotics/examples/benchmark_hierarchical_mapf_sweeps.rs`
- `docs/assets/hierarchical-mapf-replanning.svg`
- `docs/assets/hierarchical-mapf-scale.csv`
- `docs/assets/hierarchical-mapf-scale.svg`
- `docs/assets/hierarchical-mapf-region-sweep.csv`
- `docs/assets/hierarchical-mapf-region-sweep.svg`
- `docs/assets/hierarchical-mapf-density-sweep.csv`
- `docs/assets/hierarchical-mapf-density-sweep.svg`
- `docs/hierarchical_mapf_replanning.md`

Current benchmark snapshot:

- 50 agents: 25 repair groups, max group size 2, final conflicts 0.
- 100 agents: 50 repair groups, max group size 2, final conflicts 0.
- 200 agents: 100 repair groups, max group size 2, final conflicts 0.
- Full-plan fallback remains false in the scale scenario.
- Region sweep (6 swaps): region 4/8/12/24 -> max group size 2/4/6/12 and
  runtime climbs steeply as decomposition is lost; fallback stays false.
- Density sweep (4-18 agents): repair groups stay size 2 against a bounded
  4-agent flat-CBS baseline; deterministic seeded scenes.

Next useful extension:

- Sweep `region_width` and `region_height` independently (anisotropic regions)
  and add a fallback-regime row that stays solvable for the fallback-rate chart.

### Rigid-Body MIP-Style Planning

Source theme: rigid-body path planning using mixed-integer linear programming.

Implemented:

- Rectangular rigid body over discretized SE(2).
- Convex polygon obstacles represented by half-spaces.
- AABB helper constructor.
- A* over `(x, y, theta)` lattice states.
- Per-pose binary-style separation certificates.
- Segment certificates over start/mid/end swept-footprint samples.
- Transition rejection when endpoints are feasible but midpoint occupancy
  collides.
- Narrow convex-polygon slot demo and SVG.

- `RigidBodyPlanningBackend` trait with a backend-agnostic
  `RigidBodyPlanOutcome2D`, so an exact MILP backend can later sit behind the
  same call site. The lattice planner is the deterministic fallback backend.
- `RigidBodyRrtBackend2D`: a sampling RRT over SE(2) reusing the lattice
  planner's pose/segment separation-certificate geometry, with seeded
  SplitMix64 sampling for reproducibility.
- Backend comparison benchmark (lattice A* vs RRT) over seeded scenes, emitting
  CSV/SVG.

Primary files:

- `crates/rust_robotics_planning/src/rigid_body_mip.rs`
- `crates/rust_robotics/examples/headless_rigid_body_mip_planning.rs`
- `crates/rust_robotics/examples/render_rigid_body_mip_planning_svg.rs`
- `crates/rust_robotics/examples/benchmark_rigid_body_backends.rs`
- `docs/assets/rigid-body-mip-planning.svg`
- `docs/assets/rigid-body-backend-benchmark.{csv,svg}`
- `docs/rigid_body_mip_planning.md`

Current validation:

- `cargo test -p rust_robotics_planning rigid_body_mip`
- `cargo test -p rust_robotics --example benchmark_rigid_body_backends --no-default-features --features planning`
- Headless example reports pose certificates, segment certificates, and minimum
  segment margin.
- Benchmark reports per-backend success rate, mean path length, heading change,
  sample effort, and min separation margin.

Next useful extension:

- Implement the exact MILP backend behind `RigidBodyPlanningBackend` (e.g. via a
  branch-and-bound disjunctive-constraint solver) and benchmark optimality vs
  the lattice/RRT backends.

## Gallery Assets

The docs gallery is extended with research reproduction assets:

- `docs/assets/traversal-risk-graph-demo.svg`
- `docs/assets/traversal-risk-weight-sweep.svg`
- `docs/assets/mppi-replay-value-grid.svg`
- `docs/assets/mppi-track-progress.svg`
- `docs/assets/mppi-racing-gate-progress.svg`
- `docs/assets/adap-rpf-lite-mppi.svg`
- `docs/assets/adap-rpf-metrics-sweep.svg`
- `docs/assets/branchout-multimodal-driving.svg`
- `docs/assets/stl-cbs-multi-robot.svg`
- `docs/assets/kinodynamic-stl-cbs.svg`
- `docs/assets/hierarchical-mapf-replanning.svg`
- `docs/assets/hierarchical-mapf-scale.svg`
- `docs/assets/rigid-body-mip-planning.svg`
- `docs/assets/rigid-body-backend-benchmark.svg`

The gallery index is `docs/app.js`.

## Verification Commands

Focused tests that should remain green:

```bash
cargo test -p rust_robotics_planning conformal_sipp
cargo test -p rust_robotics_planning traversal_risk_graph
cargo test -p rust_robotics_planning adaptive_costmap_namo
cargo test -p rust_robotics_planning branchout
cargo test -p rust_robotics_planning stl_cbs
cargo test -p rust_robotics_planning hierarchical_mapf
cargo test -p rust_robotics_planning kinodynamic_stl_cbs
cargo test -p rust_robotics_planning rigid_body_mip
cargo test -p rust_robotics_control mppi
cargo test -p rust_robotics_control person_following
```

Representative examples:

```bash
cargo run -p rust_robotics --example benchmark_hierarchical_mapf_scale --no-default-features --features planning
cargo run -p rust_robotics --example render_kinodynamic_stl_cbs_svg --no-default-features --features planning
cargo run -p rust_robotics --example render_rigid_body_mip_planning_svg --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_rigid_body_backends --no-default-features --features planning
cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control
cargo run -p rust_robotics --example benchmark_adap_rpf_metrics --no-default-features --features control
cargo run -p rust_robotics --example render_branchout_multimodal_driving_svg --no-default-features --features planning
```

## Next Concrete Queue

1. ~~Continuous-conflict CBS branching.~~ **Done (2026-06-04).**
   - Continuous-time kinodynamic conflicts are converted into CBS vertex/edge
     constraints in `kinodynamic_stl_cbs.rs`.
   - `cbs_resolves_continuous_only_perpendicular_conflict` covers the
     integer-clean-but-continuously-unsafe case.

2. ~~Hierarchical MAPF benchmark sweeps.~~ **Done (2026-06-04).**
   - `benchmark_hierarchical_mapf_sweeps` sweeps region size and agent density,
     with a bounded flat-CBS subset baseline and seeded deterministic scenes.
   - Emits CSV/SVG charts for repair-group size, fallback, and runtime.
   - Remaining: anisotropic `region_width != region_height` sweep and a
     solvable fallback-regime data point for an explicit fallback-rate chart.

3. ~~Rigid-body MIP backend abstraction.~~ **Done (2026-06-05).**
   - `RigidBodyPlanningBackend` trait hosts pluggable backends behind a
     backend-agnostic `RigidBodyPlanOutcome2D`; an exact MILP backend can be
     added later without touching call sites.
   - The lattice A* planner is the deterministic fallback backend.
   - `RigidBodyRrtBackend2D` (seeded RRT over SE(2)) shares the lattice
     planner's separation-certificate geometry; `benchmark_rigid_body_backends`
     compares the two on identical scenes with CSV/SVG output.

4. ~~Adap-RPF metrics.~~ **Done (2026-06-05).**
   - `PersonFollowingMetricsAccumulator2D` reports target visibility ratio and
     target spacing ratio (plus mean spacing/error, min clearance, collisions).
   - `benchmark_adap_rpf_metrics` sweeps occlusion and moving-pedestrian
     scenarios (fixed back-point vs adaptive following), seeded and
     deterministic, emitting CSV/SVG.

5. BranchOut closed-loop metrics.
   - Add receding-horizon rollout.
   - Report route completion, no-collision rate, comfort, and time-to-collision
     counters.

6. Racing MPPI 3-D gates.
   - Add 3-D gate planes.
   - Add lap-progress metrics and richer drone dynamics.

## Push Checklist

Before pushing a batch:

1. Run `cargo fmt`.
2. Run the focused tests for modules touched in the batch.
3. Regenerate SVG/CSV artifacts for examples touched in the batch.
4. Update `docs/app.js`, README commands, and reproduction docs.
5. Update `docs/research_reproduction_candidates.md` and this `PLAN.md`.
6. Commit with a message that describes the reproduction slice rather than a
   generic cleanup.

