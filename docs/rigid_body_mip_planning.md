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

The lattice planner is exposed through a `RigidBodyPlanningBackend` trait so
alternative solvers can be dropped in behind the same call site. The trait
returns a backend-agnostic `RigidBodyPlanOutcome2D` carrying comparable metrics:
the realized SE(2) path, its Euclidean position length, accumulated absolute
heading change, the search effort (lattice expansions, RRT samples, or
branch-and-bound expansions), and the tightest separation margin.

Three backends implement the trait today:

1. `RigidBodyMipPlanner2D` — the exhaustive lattice A* search, used as the
   deterministic fallback backend (`name() == "lattice-astar"`).
2. `RigidBodyRrtBackend2D` — a sampling RRT over SE(2) (`name() == "rrt-se2"`).
   It reuses the lattice planner's geometric feasibility machinery (pose and
   swept-segment separation certificates), so all backends share an identical
   notion of collision and differ only in search strategy. Sampling is driven by
   a seeded SplitMix64 PRNG, so runs are fully reproducible (no `thread_rng`).
3. `RigidBodyExactBackend2D` — an exact branch-and-bound backend
   (`name() == "exact-bnb"`) that minimizes true Euclidean path length over a
   16-connected motion grid, with the body oriented along its direction of
   travel. Feasibility uses the same disjunctive separating-half-space
   certificates (for each obstacle, at least one half-space must separate the
   swept footprint — the binary/disjunctive choice). Best-first search with an
   admissible straight-line lower bound makes it length-optimal on the motion
   graph, so its path is never longer than the lattice's.

The benchmark example runs all three backends on identical scenes — the lattice
and exact backends once (deterministic), the RRT over a fixed sweep of seeds —
and reports a success rate plus the mean path length, heading change, and search
effort:

```bash
cargo run -p rust_robotics --example benchmark_rigid_body_backends --no-default-features --features planning
```

It writes `docs/assets/rigid-body-backend-benchmark.csv` and
`docs/assets/rigid-body-backend-benchmark.svg`. On the bundled scenes the
deterministic lattice search produces axis-aligned paths with wide clearance and
zero heading churn, the unsmoothed RRT reaches the goal with tighter margins and
noticeable heading change, and the exact branch-and-bound produces the shortest
path of the three (on the open-detour scene it cuts the lattice's length of 10.0
to 8.06) — the expected ordering between an exhaustive length-optimal search,
the coarse fallback, and cheap sampling.
