# Vanilla MPPI Baseline

This is a deterministic, seedable baseline for reproducing MPPI-family control
papers such as TD-CD-MPPI.

Implemented slice:

1. 2-D double-integrator dynamics.
2. Seeded Gaussian control perturbations.
3. Exponential path-integral weighting.
4. Receding-horizon nominal control shift.
5. Goal, velocity, control, and terminal costs.
6. Circular obstacle penalties with temporal constraint discounting.
7. Grid-sampled terminal value estimates with bilinear interpolation.
8. Online terminal-value grid updates from discounted rollout cost-to-go.
9. Replay-buffer terminal-value updates from recent best rollouts.
10. Static SVG export for the learned replay value grid and executed rollout.
11. Sampling diagnostics and adaptive MPPI temperature selection by effective sample size.
12. Waypoint-track progress terminal values for race-track style MPPI.
13. Static SVG export comparing single-goal and track-progress MPPI on a slalom course.
14. Reference-free racing gates with gate-crossing logic and a gate-progress MPPI objective.
15. Static SVG export comparing waypoint-reference MPPI against gate-progress MPPI.
16. Moving circular obstacles and horizon-aligned goal trajectories for
    prediction-aware MPPI.
17. 3-D rectangular gate planes, a point-mass drone with drag and actuation
    limits, open/closed laps, and lap-progress metrics.
18. CSV/SVG benchmark sweeping planar, undulating, climbing, and high-drag 3-D
    racing courses.

Run:

```bash
cargo run -p rust_robotics --example headless_mppi_double_integrator --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_constraint_discount --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_terminal_value --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_value_learning --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_replay_value_learning --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_value_grid_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_adaptive_temperature --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_track_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_track_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_racing_gate_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_racing_gate_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_adap_rpf_mppi --no-default-features --features control
cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control
```

The constraint-discounted example compares vanilla MPPI against rollouts that
penalize circular obstacle violations with a discount factor over the horizon.

The terminal-value example compares a short-horizon controller against the same
controller with a sampled goal-distance value estimate at the rollout endpoint.

The value-learning example refines that grid online: it walks backward through
the best rollout's model costs, computes discounted cost-to-go targets, and
updates the nearest value-grid cells with an exponential moving average.

The replay-value example keeps a finite buffer of recent best rollouts and
reuses them for additional value-grid backups after each planning step.

The SVG export writes `docs/assets/mppi-replay-value-grid.svg`, combining the
learned terminal-value heatmap, obstacle safety margin, goal, and final executed
trajectory.

The adaptive-temperature example reports effective sample size and entropy for
fixed versus adaptive MPPI sampling weights.

The track-progress example converts a slalom waypoint course into terminal
values that reward progress along the course and penalize lateral deviation.

The track-progress SVG export writes `docs/assets/mppi-track-progress.svg`,
showing the slalom track, obstacles, terminal-value heatmap, and both MPPI
trajectories.

The racing-gate example follows the reference-free objective from "Rethinking
Reference Trajectories in Agile Drone Racing": each rollout minimizes the
change in squared distance to the active gate, switches gates only after a
geometric crossing through the gate width, and uses no goal-tracking cost in
the gate-progress controller.

The racing-gate SVG export writes
`docs/assets/mppi-racing-gate-progress.svg`, showing oriented gates, obstacle
margins, and waypoint-reference versus gate-progress MPPI trajectories.

The Adap-RPF-lite example uses MPPI's moving-obstacle and goal-trajectory
extensions to track predicted person-following goals while proactively avoiding
predicted pedestrian motion. See `docs/adap_rpf_mppi_reproduction.md`.

## 3-D Gate Racing with Lap Progress

The racing objective extends from 2-D oriented gates to full 3-D in
`racing_mppi_3d`:

- `RacingGatePlane3D` is a rectangular gate aperture in 3-D, built from a center,
  a race-direction normal, and an in-plane up hint that is orthonormalized into
  `right`/`up` axes. A rollout passes the gate when a segment crosses the plane
  from behind to in front and the crossing point lies inside the rectangle, so
  gates can be tilted and stacked at any height.
- `RacingDroneDynamics3D` is a point-mass drone with linear aerodynamic drag, an
  optional gravity term, a speed cap, and an acceleration-magnitude cap — richer
  than the pure double integrator, so a draggier, lower-top-speed drone is
  honestly slower around the same course.
- `RacingGateLap3D` arranges gates into an open course or a closed lap; closed
  laps wrap the active gate modulo the gate count so the drone can fly repeated
  laps.
- `simulate_lap_race` drives a seeded, deterministic MPPI controller around the
  lap and returns `RacingLapReport3D`: laps completed, the fraction of the
  current lap, first-lap time, mean and peak speed, executed path length, mean
  control effort, and the minimum aperture margin at any gate crossing (1 at the
  gate center, 0 at the aperture edge, slightly negative when the drone clips a
  frame within the crossing tolerance).

The benchmark sweeps a planar closed square (two laps), an undulating closed
square where the drone climbs and descends each lap, an open five-gate ascending
helix, and a high-drag agile slalom, writing `docs/assets/racing-mppi-3d.csv`
and `.svg`:

```bash
cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control
```

On the bundled courses the reference-free controller completes two laps of both
square courses, climbs the full helix, and threads the slalom, with the draggier
slalom drone reaching a lower mean speed — the lap-progress, speed, and aperture
trade-offs the 3-D counters expose.

Next useful extension is a full quadrotor attitude model (thrust and body-rate
inputs) so the gate-progress objective drives orientation as well as position.
