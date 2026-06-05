# BranchOut-lite Multimodal Driving

This is a deterministic 2-D reproduction slice of BranchOut: Capturing
Realistic Multimodality in Autonomous Driving Decisions.

Implemented slice:

1. Lane-level driving scene with a stopped obstacle.
2. Compact mode head with keep-lane, yield, lane-change-left, and
   lane-change-right trajectories.
3. GMM-style mixture weights from trajectory costs.
4. Collision, progress, lane, route-deviation, and comfort costs.
5. Multimodal evaluation metrics:
   pairwise final displacement, pairwise discrete Frechet distance,
   minimum ground-truth Frechet distance, negative log likelihood, speed JSD,
   and expected route completion.
6. Static SVG export comparing the multimodal planner against a single-mode
   keep-lane baseline.

Run:

```bash
cargo run -p rust_robotics --example headless_branchout_multimodal_driving --no-default-features --features planning
cargo run -p rust_robotics --example render_branchout_multimodal_driving_svg --no-default-features --features planning
```

The demo uses a simple overtake scene where yielding and lane changes are all
plausible human-like decisions. The multimodal planner assigns probability mass
to yield, pass-left, and pass-right, while the unimodal baseline collapses to a
single keep-lane trajectory and scores poorly under multimodal ground-truth
coverage.

The SVG export writes `docs/assets/branchout-multimodal-driving.svg`.

## Closed-Loop Receding-Horizon Metrics

Beyond the one-shot multimodal evaluation, the planner now drives a
receding-horizon closed loop through `BranchOutPlanner2D::simulate_closed_loop`.
At every control step it re-plans from the current ego pose, commits the
highest-probability mode with a bounded-rate lane-tracking step (the per-mode
rollout's lateral curve is back-loaded, so the closed loop tracks the selected
mode's target lane rather than replaying only its first sample), advances the
optionally moving traffic, and accumulates `BranchOutClosedLoopMetrics2D`:

- **route completion** and a `reached_goal` flag;
- **no-collision rate** and collision-step count;
- **comfort** (mean lateral jerk plus longitudinal acceleration);
- **time-to-collision** — the minimum TTC over the rollout and a count of risky
  steps below a threshold, computed from the disk-vs-disk closing geometry.

The benchmark sweeps four scenarios and writes
`docs/assets/branchout-closed-loop.csv` and `.svg`:

```bash
cargo run -p rust_robotics --example benchmark_branchout_closed_loop --no-default-features --features planning
```

On the bundled scenes the ego overtakes when there is room (100% completion,
collision-free), yields safely behind a fully blocked lane (incomplete but
collision-free, with risky-TTC steps logged while it brakes), follows a slow
lead car at a safe distance, and changes lanes to clear an oncoming obstacle —
the route-completion / safety / comfort / TTC trade-offs the closed-loop
counters are meant to expose.
