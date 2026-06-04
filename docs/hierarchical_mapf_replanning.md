# Hierarchical MAPF Replanning

This is a deterministic pure-Rust reproduction slice for hierarchical
large-scale multi-robot path / trajectory replanning.

Implemented slice:

1. Independent shortest-path baseline using the STL-CBS low-level planner.
2. Coarse rectangular region hierarchy over the grid.
3. Region-occupancy conflict detection over time.
4. Connected affected-agent grouping from region conflicts.
5. Group-only CBS replanning with full-plan fallback if local repair leaves
   cell conflicts.
6. Static SVG export showing independent paths, region triggers, and final
   conflict-free paths.
7. Scale benchmark over 50, 100, and 200 agents with isolated local repair
   groups, CSV output, and SVG timing chart.

Run:

```bash
cargo run -p rust_robotics --example headless_hierarchical_mapf_replanning --no-default-features --features planning
cargo run -p rust_robotics --example render_hierarchical_mapf_replanning_svg --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_hierarchical_mapf_scale --no-default-features --features planning
```

The demo uses four grid robots. The independent shortest paths create a central
cell conflict and four coarse region triggers. The hierarchical layer replans
only agents `[0, 1]` with CBS, leaves the other agents independent, and finishes
with zero cell conflicts without using the full-plan fallback.

The SVG export writes `docs/assets/hierarchical-mapf-replanning.svg`.

The scale benchmark writes `docs/assets/hierarchical-mapf-scale.csv` and
`docs/assets/hierarchical-mapf-scale.svg`. The benchmark uses 25, 50, and 100
independent corridor-swap pairs. Each pair produces one local CBS group of size
2, so the 200-agent case finishes with 100 repair groups, zero final conflicts,
and no full-plan fallback.
