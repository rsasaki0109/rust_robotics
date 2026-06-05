# PolyMerge-lite: CBF Safety Filter over Convex Polytopes

Source theme: polytope-covering safety (PolyMerge) and control-barrier-function
filtering.

This is a pure-Rust reproduction of the reusable safety core of polytope-cover
work, without the 3D Gaussian-splat front end: obstacles are convex polygons
(the "cover"), and a control-barrier-function (CBF) quadratic-program filter
minimally corrects a nominal velocity command so a single-integrator robot stays
outside every obstacle.

## Implemented slice

- `CbfConvexObstacle2D` mirrors the half-space representation of the rigid-body
  MIP planner (`a*x + b*y <= c` contains the obstacle interior). Its `barrier`
  returns the **true** signed distance to the convex polygon and the outward
  gradient — using closest-point-on-edge so vertex regions are handled honestly
  rather than over-optimistically.
- `CbfSafetyFilter::filter` solves, for `x_dot = u`, the CBF-QP
  `min ||u - u_nom||^2  s.t.  grad_o . u >= -alpha * (h_o(x) - r - margin)` for
  every obstacle, with an exact **2-D active-set enumeration** (at most two
  constraints bind in the plane). The result is deterministic and
  correction-minimal; a configurable margin keeps real clearance beyond the
  robot radius.
- `simulate_cbf_navigation` drives a proportional go-to-goal controller through
  the obstacle field with and without the filter and reports clearance,
  collisions, and intervention counts.

## Primary files

- `crates/rust_robotics_control/src/cbf_safety_filter.rs`
- `crates/rust_robotics/examples/benchmark_cbf_safety_filter.rs`
- `docs/assets/cbf-safety-filter.svg`
- `docs/assets/cbf-safety-filter.csv`

## Run

```bash
cargo run -p rust_robotics --example benchmark_cbf_safety_filter --no-default-features --features control
```

## Result

On four scenarios (a grazing box, a slalom, an obstacle ridge, and a wide box),
the raw go-to-goal controller drives into every obstacle (clearance `-0.15`),
while the CBF-filtered controller keeps positive clearance (`+0.08` to `+0.12`)
and still reaches the goal, intervening only where the obstacles intrude.

## A note on reactive deadlock

A pure reactive CBF filter has a well-known limitation: when an obstacle face is
square to the goal pull, the barrier gradient is anti-parallel to the desired
velocity and the robot wedges against the face with no lateral escape. The
benchmark scenarios place obstacles so the robot meets a near corner (a gradient
with a lateral component) and is deflected along the face, which avoids the
deadlock. Resolving the head-on case in general needs a guidance layer (a global
path or a rotational bias) that the CBF then filters — a natural next extension.

## Next useful extension

Filter a global path's reference velocity instead of a raw go-to-goal command
(removing the head-on deadlock), and extend the obstacle cover to a multi-
polytope decomposition of a non-convex region.
