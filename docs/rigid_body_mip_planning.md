# Rigid-Body MIP-Style Planning

This is a deterministic pure-Rust reproduction slice for rigid-body path
planning with mixed-integer obstacle avoidance.

Implemented slice:

1. Rectangular rigid body in discretized SE(2).
2. Convex polygon obstacles represented as half-spaces, with AABB as a helper
   constructor.
3. A* search over `(x, y, theta)` lattice states.
4. Per-pose binary-style separation certificates: for every obstacle, one
   active half-space must separate the whole robot rectangle.
5. Per-segment certificates over start/mid/end swept-footprint samples, so a
   transition can be rejected even when both endpoint poses are feasible.
6. Narrow-slot demo where the rectangle must rotate before passing the gap.
7. Static SVG export showing obstacles, robot footprints, path, and certificate
   metrics.

Run:

```bash
cargo run -p rust_robotics --example headless_rigid_body_mip_planning --no-default-features --features planning
cargo run -p rust_robotics --example render_rigid_body_mip_planning_svg --no-default-features --features planning
```

The demo starts the rectangle vertically, rotates it horizontally, then sends it
through a slot between two slanted convex obstacles. Each accepted pose and
segment stores one half-space certificate per obstacle, matching the binary
disjunction structure used in MILP collision-avoidance formulations.

The SVG export writes `docs/assets/rigid-body-mip-planning.svg`.

## Backend Abstraction and RRT Comparison

The lattice planner is now exposed through a `RigidBodyPlanningBackend` trait so
a different solver — for example an exact MILP backend — can be dropped in
behind the same call site later. The trait returns a backend-agnostic
`RigidBodyPlanOutcome2D` carrying comparable metrics: the realized SE(2) path,
its Euclidean position length, accumulated absolute heading change, the search
effort (lattice expansions or RRT samples), and the tightest separation margin.

Two backends implement the trait today:

1. `RigidBodyMipPlanner2D` — the exhaustive lattice A* search, used as the
   deterministic fallback backend (`name() == "lattice-astar"`).
2. `RigidBodyRrtBackend2D` — a sampling RRT over SE(2) (`name() == "rrt-se2"`).
   It reuses the lattice planner's geometric feasibility machinery (pose and
   swept-segment separation certificates), so both backends share an identical
   notion of collision and differ only in search strategy. Sampling is driven by
   a seeded SplitMix64 PRNG, so runs are fully reproducible (no `thread_rng`).

The benchmark example runs both backends on identical scenes — the lattice once
(it is deterministic) and the RRT over a fixed sweep of seeds — and reports a
success rate plus the mean path length, heading change, and sample effort:

```bash
cargo run -p rust_robotics --example benchmark_rigid_body_backends --no-default-features --features planning
```

It writes `docs/assets/rigid-body-backend-benchmark.csv` and
`docs/assets/rigid-body-backend-benchmark.svg`. On the bundled scenes the
deterministic lattice search produces axis-aligned paths with wide clearance and
zero heading churn, while the unsmoothed RRT reaches the goal with comparable
path length but tighter margins and noticeable heading change — the expected
trade-off between an exhaustive optimal-in-its-metric search and cheap sampling.
