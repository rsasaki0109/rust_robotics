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
