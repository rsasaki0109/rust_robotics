# Kinodynamic STL-CBS

This is a deterministic extension of the STL-CBS reproduction slice toward
kinodynamic multi-robot planning.

Implemented slice:

1. Heading-aware search state `(x, y, heading, t)`.
2. Time-consuming motion primitives:
   forward moves, in-place turns, waits, and optional reverse moves.
3. Conservative occupancy while a forward primitive consumes multiple ticks.
4. CBS vertex and edge constraints applied to the time-expanded oriented
   low-level search.
5. Optional terminal heading constraints.
6. Continuous-time pairwise occupancy checks between integer ticks, with exact
   closest-approach robustness over each linear segment.
7. Static SVG export showing independent oriented plans versus CBS-repaired
   paths.

Run:

```bash
cargo run -p rust_robotics --example headless_kinodynamic_stl_cbs --no-default-features --features planning
cargo run -p rust_robotics --example render_kinodynamic_stl_cbs_svg --no-default-features --features planning
```

The demo uses two robots facing each other across a grid corridor. Independent
oriented paths collide. Kinodynamic STL-CBS repairs the conflict by assigning a
detour with explicit heading changes and two-tick forward moves. The reported
summary includes both integer-time separation robustness and continuous-time
separation robustness; boundary contact is robustness zero, while between-tick
penetration is reported as a continuous conflict.

The SVG export writes `docs/assets/kinodynamic-stl-cbs.svg`.
