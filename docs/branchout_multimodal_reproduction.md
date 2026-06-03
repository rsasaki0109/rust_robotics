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
