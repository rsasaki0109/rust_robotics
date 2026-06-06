# RustRobotics Research Reproduction Plan

Updated: 2026-06-05

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
- 3-D rectangular gate planes with plane-crossing aperture passage logic.
- Point-mass drone dynamics with drag, gravity, and actuation limits.
- Open courses and closed laps with lap-progress metrics.
- CSV/SVG benchmark across planar, undulating, climbing, and high-drag courses.
- Full quadrotor attitude model (`racing_mppi_quadrotor`): collective thrust +
  body-rate inputs with quaternion attitude kinematics; the gate-progress
  objective drives orientation, so the drone tilts to aim its thrust at the next
  gate. Reports tilt and body-rate metrics; closes a full lap on attitude alone.
- CSV/SVG quadrotor benchmark across slalom, climbing, closed-lap, and heavy
  high-gravity courses.
- Motor-level rotor-mixing model (`racing_mppi_motor`): four rotor thrusts mix
  into collective thrust and body torques, body rates are states with rate
  damping, and rotors saturate. Reports a rotor saturation fraction; a
  thrust-limited drone saturates ~57% vs ~44% and falls short on the slalom.
- CSV/SVG motor benchmark contrasting an agile vs a thrust-limited drone.
- Motor-lag and battery-sag powertrain (`racing_mppi_powertrain`): a composition
  layer over the rotor model adding first-order spin-up lag and a thrust ceiling
  that droops with load and state of charge. `PowertrainParams::ideal` reduces
  exactly to the motor model. A powertrain-unaware controller plans against the
  ideal model; a pack drained to 25% saturates its lowered ceiling on nearly
  every step and stalls after one gate where the fresh drone finishes.
- CSV/SVG powertrain benchmark: one slalom, four powertrains, with a
  battery-state-of-charge trace panel.
- Powertrain-aware controller (`PowertrainMppiController`): rolls candidates out
  through the lag/battery model so it plans within deliverable authority and
  conserves charge for later gates. On an ideal powertrain it plans the same
  command as the motor controller (unit-tested). `benchmark_racing_powertrain_aware`
  pits aware vs unaware: on a 25%-drained pack the unaware controller stalls at
  1/4 gates draining to ~2%, while the aware one threads 4/4 and finishes with
  ~18% charge in reserve.
- Charge-budget term (`ChargeBudget`): penalizes below-reserve current draw so
  the aware controller eases off when the pack runs low. `benchmark_racing_powertrain_budget`
  sweeps the weight over a draining multi-lap square and traces one Pareto
  frontier (heavier budget -> slower, fewer laps, more reserve). Honest finding:
  on a hover-dominated quad with no regen, pacing buys reserve, not extra laps.
- Battery-recovery model (`PowertrainParams::with_recovery`): a relaxation
  overpotential that builds under load and decays on ease-off, so the terminal
  voltage (thrust ceiling) recovers during a rest while SoC keeps falling.
  `terminal_voltage_scale` folds it in; with recovery off it equals `voltage_scale`.
  `benchmark_racing_powertrain_recovery` (dynamics-only) drives a scripted
  hard/rest profile and shows the terminal voltage climbing back across a rest.
- Budget x recovery capstone (`benchmark_racing_powertrain_endurance`): a 2x2 of
  {greedy, budgeted} x {recovery off, on} on a draining undulating lap. Honest
  finding: recovery reverses the budget's sign — without recovery pacing gives up
  a full lap, with recovery it pulls even on laps while flying faster with more
  reserve (a Pareto win). A strict more-laps flip needs low hover overhead or
  true idle rests.

Primary files:

- `crates/rust_robotics_control/src/mppi.rs`
- `crates/rust_robotics_control/src/racing_mppi_3d.rs`
- `crates/rust_robotics_control/src/racing_mppi_quadrotor.rs`
- `crates/rust_robotics_control/src/racing_mppi_motor.rs`
- `crates/rust_robotics_control/src/racing_mppi_powertrain.rs`
- `crates/rust_robotics/examples/headless_mppi_racing_gate_progress.rs`
- `crates/rust_robotics/examples/render_mppi_racing_gate_progress_svg.rs`
- `crates/rust_robotics/examples/benchmark_racing_mppi_3d.rs`
- `crates/rust_robotics/examples/benchmark_racing_quadrotor.rs`
- `crates/rust_robotics/examples/benchmark_racing_motor.rs`
- `crates/rust_robotics/examples/benchmark_racing_powertrain.rs`
- `crates/rust_robotics/examples/benchmark_racing_powertrain_aware.rs`
- `crates/rust_robotics/examples/benchmark_racing_powertrain_budget.rs`
- `crates/rust_robotics/examples/benchmark_racing_powertrain_recovery.rs`
- `crates/rust_robotics/examples/benchmark_racing_powertrain_endurance.rs`
- `docs/assets/mppi-racing-gate-progress.svg`
- `docs/assets/racing-mppi-3d.svg`
- `docs/assets/racing-mppi-3d.csv`
- `docs/assets/racing-quadrotor.svg`
- `docs/assets/racing-quadrotor.csv`
- `docs/assets/racing-motor.svg`
- `docs/assets/racing-motor.csv`
- `docs/assets/racing-powertrain.svg`
- `docs/assets/racing-powertrain.csv`
- `docs/assets/racing-powertrain-aware.svg`
- `docs/assets/racing-powertrain-aware.csv`
- `docs/assets/racing-powertrain-budget.svg`
- `docs/assets/racing-powertrain-budget.csv`
- `docs/assets/racing-powertrain-recovery.svg`
- `docs/assets/racing-powertrain-recovery.csv`
- `docs/assets/racing-powertrain-endurance.svg`
- `docs/assets/racing-powertrain-endurance.csv`

Next useful extension:

- Chase the strict more-laps flip on a low-hover-overhead platform (a fixed-wing
  or a much lighter quad) or a course with true idle rests, where rest-and-recover
  pacing can win outright. Separately, add a per-rotor torque/current map so yaw
  authority sags apart from thrust.

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

- Receding-horizon closed-loop rollout (`simulate_closed_loop`) that selects a
  mode each step and tracks its lane with a bounded rate, with closed-loop
  counters: route completion, no-collision rate, comfort, and time-to-collision.

Primary files:

- `crates/rust_robotics_planning/src/branchout_multimodal.rs`
- `crates/rust_robotics/examples/headless_branchout_multimodal_driving.rs`
- `crates/rust_robotics/examples/render_branchout_multimodal_driving_svg.rs`
- `crates/rust_robotics/examples/benchmark_branchout_closed_loop.rs`
- `docs/assets/branchout-multimodal-driving.svg`
- `docs/assets/branchout-closed-loop.{csv,svg}`
- `docs/branchout_multimodal_reproduction.md`

Next useful extension:

- Add multi-vehicle interactive traffic (reactive obstacles) and a comfort vs
  completion Pareto sweep over the planner weights.

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
- `docs/assets/hierarchical-mapf-anisotropic-sweep.csv`
- `docs/assets/hierarchical-mapf-anisotropic-sweep.svg`
- `docs/assets/hierarchical-mapf-fallback-sweep.csv`
- `docs/assets/hierarchical-mapf-fallback-sweep.svg`
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
- Anisotropic sweep `(region_width, region_height)`: on a horizontal swap row,
  max repair-group size tracks width (2/4/8 for width 4/8/16) and is invariant
  to height — `(8, 4)` and `(8, 16)` both give group 4.
- Fallback-rate sweep: an adjacent edge-swap straddles a region boundary with
  probability ~1/region, so the fallback rate falls 1.00 -> 0.56 -> 0.34 ->
  0.22 -> 0.19 across regions 1/2/3/4/6, and every scene still resolves.

Next useful extension:

- Add dynamic task insertions (re-plan with new start/goal pairs arriving over
  time) and trajectory-duration costs to the region routes.

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
- `RigidBodyExactBackend2D`: an exact branch-and-bound backend that minimizes
  true Euclidean path length over a 16-connected motion grid with the body
  oriented along travel; feasibility uses the same disjunctive separating-
  half-space certificates (best-first search with an admissible straight-line
  lower bound = length-optimal on the motion graph).
- Backend comparison benchmark (lattice A* vs RRT vs exact branch-and-bound)
  over seeded scenes, emitting CSV/SVG. The exact backend is shortest: on the
  open-detour scene it cuts the lattice's 10.0 path to 8.06.

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

- Add continuous swept-volume certificates (exact area-swept separation between
  lattice poses) and a heading-constrained variant of the exact backend.

### SafeDec-lite STL-Shielded Navigation

Source theme: constrained decoding for safe robot navigation policies.

Implemented:

- Deterministic greedy goal-seeking base policy over a discrete action set.
- STL shield reusing `stl_cbs` primitives: a hard `always-avoid` geofence (with
  an optional safety margin) and a soft `eventually-reach` goal region.
- Deterministic constrained beam search (`SafeDecoder::decode`) returning both
  the greedy and shielded paths, with robustness values and an intervention
  count.
- SVG comparing the greedy hazard-cutting path against the shielded detour.

Primary files:

- `crates/rust_robotics_planning/src/safe_decode_nav.rs`
- `crates/rust_robotics/examples/render_safe_decode_nav_svg.rs`
- `docs/assets/safe-decode-nav.svg`
- `docs/safe_decode_nav_reproduction.md`

Next useful extension:

- Replace the greedy policy with a sampled/learned action distribution and add
  `eventually`/`until` task chains (visit A then B while avoiding C).

### CBF Safety Filter (PolyMerge-lite)

Source theme: polytope-covering safety and control-barrier-function filtering.

Implemented:

- `CbfConvexObstacle2D` convex-polygon obstacles with a true point-to-polygon
  signed-distance barrier and outward gradient (vertex regions handled honestly).
- `CbfSafetyFilter` CBF quadratic-program filter solved by an exact 2-D
  active-set enumeration, with a configurable clearance margin.
- `simulate_cbf_navigation` comparing a raw go-to-goal controller against the
  filtered one (clearance, collisions, interventions).
- Four-scenario CSV/SVG benchmark (grazing box, slalom, ridge, wide box).

Primary files:

- `crates/rust_robotics_control/src/cbf_safety_filter.rs`
- `crates/rust_robotics/examples/benchmark_cbf_safety_filter.rs`
- `docs/assets/cbf-safety-filter.svg`
- `docs/assets/cbf-safety-filter.csv`
- `docs/cbf_safety_filter_reproduction.md`

Next useful extension:

- Filter a global path's reference velocity (removing the reactive head-on
  deadlock) and cover non-convex regions with a multi-polytope decomposition.

### Long Range Navigator-lite Frontier Navigation

Source theme: long-range navigation past the local map.

Implemented:

- Occlusion-aware sensing accumulating a persistent known map.
- Frontier extraction and 8-connected clustering with openness counts.
- Affordance scoring (goal progress, known-free travel cost, line of sight,
  openness) to select the next frontier.
- Dijkstra local-planner handoff over the known-free map, with a final plan to
  the goal once it is revealed.

Primary files:

- `crates/rust_robotics_planning/src/frontier_navigator.rs`
- `crates/rust_robotics/examples/render_frontier_navigator_svg.rs`
- `docs/assets/frontier-navigator.svg`
- `docs/frontier_navigator_reproduction.md`

Next useful extension:

- Replace the hand-tuned affordance weights with a learned frontier value and
  add multi-goal / patrol frontier scheduling.

## Gallery Assets

The docs gallery is extended with research reproduction assets:

- `docs/assets/traversal-risk-graph-demo.svg`
- `docs/assets/traversal-risk-weight-sweep.svg`
- `docs/assets/mppi-replay-value-grid.svg`
- `docs/assets/mppi-track-progress.svg`
- `docs/assets/mppi-racing-gate-progress.svg`
- `docs/assets/racing-mppi-3d.svg`
- `docs/assets/racing-quadrotor.svg`
- `docs/assets/racing-motor.svg`
- `docs/assets/adap-rpf-lite-mppi.svg`
- `docs/assets/adap-rpf-metrics-sweep.svg`
- `docs/assets/branchout-multimodal-driving.svg`
- `docs/assets/branchout-closed-loop.svg`
- `docs/assets/stl-cbs-multi-robot.svg`
- `docs/assets/kinodynamic-stl-cbs.svg`
- `docs/assets/safe-decode-nav.svg`
- `docs/assets/hierarchical-mapf-replanning.svg`
- `docs/assets/hierarchical-mapf-scale.svg`
- `docs/assets/hierarchical-mapf-anisotropic-sweep.svg`
- `docs/assets/hierarchical-mapf-fallback-sweep.svg`
- `docs/assets/rigid-body-mip-planning.svg`
- `docs/assets/rigid-body-backend-benchmark.svg`
- `docs/assets/cbf-safety-filter.svg`
- `docs/assets/frontier-navigator.svg`

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
cargo run -p rust_robotics --example benchmark_branchout_closed_loop --no-default-features --features planning
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

5. ~~BranchOut closed-loop metrics.~~ **Done (2026-06-05).**
   - `BranchOutPlanner2D::simulate_closed_loop` runs a receding-horizon rollout
     that re-plans each step and tracks the selected mode's lane.
   - `BranchOutClosedLoopMetrics2D` reports route completion, no-collision rate,
     comfort, and time-to-collision (min + risky-step count).
   - `benchmark_branchout_closed_loop` sweeps overtake/yield/lead/oncoming
     scenes with CSV/SVG output.

6. ~~Racing MPPI 3-D gates.~~ **Done (2026-06-05).**
   - `RacingGatePlane3D` is a rectangular 3-D gate aperture (center, race-
     direction normal, orthonormalized up/right axes) with plane-crossing
     aperture-containment passage logic.
   - `RacingDroneDynamics3D` is a point-mass drone with linear drag, optional
     gravity, speed cap, and acceleration-magnitude cap — richer than the 2-D
     double integrator.
   - `RacingGateLap3D` supports open courses and closed laps (active gate wraps
     modulo gate count); `simulate_lap_race` runs a seeded deterministic MPPI
     and reports `RacingLapReport3D` lap-progress metrics (laps completed, lap
     fraction, first-lap time, mean/peak speed, aperture margin).
   - `benchmark_racing_mppi_3d` sweeps planar/undulating closed squares, an
     ascending helix, and a high-drag slalom with CSV/SVG output.
   - **Follow-up (2026-06-05):** `racing_mppi_quadrotor` adds a full quadrotor
     attitude model (collective thrust + body rates, quaternion kinematics); the
     gate-progress objective drives orientation. `benchmark_racing_quadrotor`
     flies slalom/climb/closed-lap/heavy courses with tilt and body-rate metrics.

## Next Concrete Queue (v2 — 2026-06-05)

The first queue (#1–#6 above) is fully landed. This second queue leans into
breadth: three brand-new reproduction papers with pure-Rust algorithmic cores
and strong reuse of existing modules, plus three recorded depth extensions kept
as low-risk wins. Each item is one slice: library code + tests + a deterministic
benchmark/headless example + SVG/CSV artifact + docs.

1. ~~SafeDec-lite: STL-shielded navigation decoding.~~ **Done (2026-06-06).**
   `safe_decode_nav.rs` runs a deterministic constrained beam search shielding a
   greedy grid policy with `stl_cbs` always-avoid / eventually-reach robustness;
   `render_safe_decode_nav_svg` contrasts the greedy hazard-cutting path with the
   shielded detour (avoid robustness -1.0 -> +1.0).

2. ~~CBF safety filter (PolyMerge-lite).~~ **Done (2026-06-06).**
   `cbf_safety_filter.rs` filters a go-to-goal command with an exact 2-D
   active-set CBF-QP over convex-polygon obstacles (true point-to-polygon
   barrier); `benchmark_cbf_safety_filter` contrasts colliding raw control vs the
   filtered safe control (clearance -0.15 -> +0.08..0.12) across four scenarios.

3. ~~Long Range Navigator-lite.~~ **Done (2026-06-06).**
   `frontier_navigator.rs` does occlusion-aware sensing, affordance-scored
   frontier selection (goal progress / known-free cost / line of sight /
   openness), and a Dijkstra local-planner handoff;
   `render_frontier_navigator_svg` routes a robot around two walls it cannot see
   past to the goal.

4. ~~Hierarchical MAPF anisotropic region sweep (extension).~~ **Done
   (2026-06-06).** `benchmark_hierarchical_mapf_sweeps` adds an anisotropic
   `(region_width, region_height)` sweep (group size tracks width, invariant to
   height) and a fallback-rate sweep (adjacent edge-swaps straddle a region
   boundary with probability ~1/region; rate 1.00 -> 0.19 over regions 1..6,
   every scene still resolved).

5. ~~Rigid-body exact MILP backend (extension).~~ **Done (2026-06-06).**
   `RigidBodyExactBackend2D` is a length-optimal branch-and-bound over a
   16-connected motion grid (body oriented along travel) using the disjunctive
   separating-half-space certificates; `benchmark_rigid_body_backends` now
   compares lattice vs RRT vs exact and the exact path is shortest (open-detour
   10.0 -> 8.06).

6. ~~Racing motor-level model (extension).~~ **Done (2026-06-07).**
   `racing_mppi_motor` adds a four-rotor rotor-mixing quadrotor (body rates as
   states, rotors saturate); `benchmark_racing_motor` shows a thrust-limited
   drone saturating ~57% vs ~44% and falling short on the slalom — the
   thrust/torque trade-off the body-rate model cannot express.

7. ~~Motor-lag and battery-sag powertrain (extension).~~ **Done (2026-06-07).**
   `racing_mppi_powertrain` composes a first-order motor-lag and battery-sag
   layer over the rotor model (`PowertrainParams::ideal` reduces exactly to the
   motor model). `benchmark_racing_powertrain` flies one slalom under four
   powertrains with a powertrain-unaware controller: a pack drained to 25%
   saturates its lowered thrust ceiling ~98% of steps and stalls after one gate
   where the fresh drone threads all four.

8. ~~Powertrain-aware controller (extension).~~ **Done (2026-06-07).**
   `PowertrainMppiController` rolls candidates out through the lag/battery model,
   so it plans within deliverable authority (and matches the motor controller on
   an ideal powertrain, unit-tested). `benchmark_racing_powertrain_aware` shows
   that on a 25%-drained pack the unaware controller stalls at 1/4 gates while the
   aware one threads 4/4 and finishes with ~18% charge in reserve.

9. ~~Charge-budget term (extension).~~ **Done (2026-06-07).**
   `ChargeBudget` penalizes below-reserve current draw in the aware rollout;
   `benchmark_racing_powertrain_budget` sweeps its weight over a draining
   multi-lap square and traces a Pareto frontier (heavier budget -> slower,
   fewer laps, more reserve). Honest finding documented: on a hover-dominated
   quad with no regen, pacing buys reserve, not extra laps.

10. ~~Battery-recovery model (extension).~~ **Done (2026-06-07).**
    `PowertrainParams::with_recovery` adds a relaxation overpotential that builds
    under load and decays on ease-off, so `terminal_voltage_scale` (the thrust
    ceiling) recovers during a rest while SoC keeps falling. Recovery off equals
    the prior behavior. `benchmark_racing_powertrain_recovery` (dynamics-only)
    shows the terminal voltage climbing back across a hover phase (~0.585 ->
    0.614) as charge only ever drops.

11. ~~Budget x recovery capstone (extension).~~ **Done (2026-06-07).**
    `benchmark_racing_powertrain_endurance` runs the 2x2 of {greedy, budgeted} x
    {recovery off, on} on a draining undulating lap. Honest finding: recovery
    reverses the budget's sign — without recovery pacing gives up a full lap
    (1.75 vs 2.75); with recovery it pulls even on laps (1.75) while flying faster
    (2.0 vs 1.75 m/s) with more reserve. A strict more-laps flip needs low hover
    overhead or true idle rests, recorded as the next extension.

12. ~~Quasi-static planar pushing (new domain: manipulation).~~ **Done
    (2026-06-07).** `pusher_slider.rs` reproduces the classic pusher-slider in the
    spirit of "Push Anything": ellipsoidal limit surface, single-point contact
    with stick/slide contact modes, and a seeded MPPI pusher that modulates
    contact offset, slip, and push speed (so it brakes on the goal). First
    manipulation target in the repo. See `docs/pusher_slider_reproduction.md`.

13. ~~Multi-face pushing (extension).~~ **Done (2026-06-07).** Generalized the
    contact model to any of the four faces (per-face contact frame; the
    limit-surface solve is shared, the friction cone is evaluated along each
    face's normal/tangent). `PusherSliderMppiController` is now face-aware: it
    runs MPPI per face and executes the lowest-cost face. This resolves the
    single-face limitation — `benchmark_pusher_slider` now includes a `spin`
    scenario (pure rotation, no net translation) reachable only by switching
    faces.

14. ~~Multi-object pushing (extension).~~ **Done (2026-06-07).** `simulate_multi_push`
    pushes several sliders to goal slots one at a time, treating the other objects
    as soft keep-out discs (`obstacle_weight`/`obstacle_radius` in the controller
    cost) so the active slider routes around blocks in its path.
    `benchmark_pusher_slider_multi` has object 0 detour around object 1 sitting in
    its straight-line path; all three settle within ~1 cm. Reproduces the
    multi-object setting of "Push Anything". Next: simultaneous two-pusher
    multi-contact pushing (a contact-implicit complementarity problem).

## Push Checklist

Before pushing a batch:

1. Run `cargo fmt`.
2. Run the focused tests for modules touched in the batch.
3. Regenerate SVG/CSV artifacts for examples touched in the batch.
4. Update `docs/app.js`, README commands, and reproduction docs.
5. Update `docs/research_reproduction_candidates.md` and this `PLAN.md`.
6. Commit with a message that describes the reproduction slice rather than a
   generic cleanup.

