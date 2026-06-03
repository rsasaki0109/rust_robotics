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
