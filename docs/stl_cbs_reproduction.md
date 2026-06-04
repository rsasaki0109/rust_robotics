# STL-CBS Multi-Robot Planning

This is a deterministic foundation for reproducing the formal multi-robot
planning slice of STL-KCBS / cBOT-style work.

Implemented slice:

1. Grid multi-agent conflict-based search.
2. Low-level time-expanded A* with wait, vertex constraints, and edge-swap
   constraints.
3. High-level CBS node expansion that replans only the constrained agent.
4. STL-style robustness primitives:
   eventually reach a goal region, always avoid a rectangular region, and
   pairwise separation over a time interval.
5. Independent shortest-path baseline for conflict inspection.
6. Static SVG export showing independent conflicting paths versus CBS-resolved
   paths.

Run:

```bash
cargo run -p rust_robotics --example headless_stl_cbs_multi_robot --no-default-features --features planning
cargo run -p rust_robotics --example render_stl_cbs_multi_robot_svg --no-default-features --features planning
```

The demo uses three robots crossing a small grid. Independent shortest paths
collide at the center. CBS resolves the vertex/edge conflicts by adding
time-indexed constraints, and the final plan reports positive STL goal and
pairwise separation robustness.

The SVG export writes `docs/assets/stl-cbs-multi-robot.svg`.
