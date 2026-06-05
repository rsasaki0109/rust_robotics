# Adap-RPF-lite Person-Following MPPI

This is a deterministic, dependency-light reproduction slice of
Adap-RPF: Adaptive Trajectory Sampling for Robot Person Following in Dynamic
Crowded Environments.

Implemented slice:

1. Linear constant-velocity prediction for target and surrounding pedestrians.
2. Target-centric semi-annular following-point sampling.
3. Low-discrepancy candidate coverage using a Halton sequence.
4. Multi-objective candidate scoring:
   visibility/occlusion, proximity, desired distance, travel effort, and
   stickiness.
5. Horizon-aligned person-following goal trajectories.
6. Prediction-aware MPPI support through moving circular obstacles.
7. Static SVG export comparing fixed back-point following against adaptive
   following-point sampling.

Run:

```bash
cargo run -p rust_robotics --example headless_adap_rpf_mppi --no-default-features --features control
cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control
```

The headless demo creates a target moving forward while another pedestrian
occupies the fixed trailing point. The fixed baseline tracks a single point
behind the target. The adaptive version samples a social semi-annulus around
the predicted target, selects a lower-occlusion following point, converts it
into a predicted goal trajectory, and tracks that trajectory with MPPI while
avoiding predicted pedestrian motion.

The SVG export writes `docs/assets/adap-rpf-lite-mppi.svg`.

## Metric Sweep: Visibility and Spacing

The library now exposes Adap-RPF reporting counters through
`PersonFollowingMetricsAccumulator2D`, fed one robot/target/pedestrian snapshot
per closed-loop step:

- **target visibility ratio** — fraction of steps the robot-to-target line of
  sight stays clear of every pedestrian disk
  (`PersonFollowingRolloutMetrics2D::target_visibility_ratio`);
- **target spacing ratio** — fraction of steps the robot holds the desired
  spacing band (`target_spacing_ratio`), plus mean spacing and mean spacing
  error;
- minimum pedestrian clearance and collision-step count.

The sweep benchmark runs the closed loop under four scenarios of increasing
difficulty — `open`, `trailing-occluder`, `crossing-peds`, and
`dense-occlusion` — for both the fixed back-point baseline and the adaptive
follower, with a fixed seed so results are deterministic:

```bash
cargo run -p rust_robotics --example benchmark_adap_rpf_metrics --no-default-features --features control
```

It writes `docs/assets/adap-rpf-metrics-sweep.csv` and
`docs/assets/adap-rpf-metrics-sweep.svg`. On the bundled scenes the fixed
baseline loses the target whenever a pedestrian camps on the trailing point
(0% visibility under occlusion), while the adaptive follower sidesteps to keep
the target visible (72% under a trailing occluder, 90% through crossing
pedestrians, 52% in the dense scene) and stays collision-free throughout —
the visibility/spacing trade-off the Adap-RPF metrics are designed to expose.
