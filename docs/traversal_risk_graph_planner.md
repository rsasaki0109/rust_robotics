# Traversal-Risk Graph Planner

This pure-Rust planner is a first slice toward risk-aware unstructured-terrain
planning, inspired by traversal-risk graph planners such as TRG-planner.

It is not a full paper reproduction. The implemented slice is intentionally
small:

1. Represent a terrain grid with blocked cells plus traversability, stability,
   and exposure risk channels.
2. Search the grid with A* using cost:

```text
distance_weight * travel_distance
+ risk_weight * average_cell_risk * travel_distance
```

3. Return a path with decomposed distance, risk, and total costs.

The module also includes terrain-preprocessing and evaluation helpers:

- `terrain_risk_from_elevation_map` converts elevation grids into slope and
  roughness risk channels.
- `inflate_blocked_cells_by_radius` applies a circular robot footprint to
  blocked cells before planning.
- `clearance_map` and `add_clearance_exposure_risk` convert distance from
  blocked cells into low-clearance exposure risk.
- `smooth_terrain_risk` applies Gaussian-style local smoothing to
  traversability, stability, and exposure risk channels while preserving
  blocked-cell topology.
- `sweep_traversal_risk_weights` evaluates a fixed query across multiple
  terrain-risk weights and records the unweighted terrain-risk cost for Pareto
  analysis.

Run:

```bash
cargo run -p rust_robotics --example headless_traversal_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example headless_elevation_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example headless_risk_map_smoothing --no-default-features --features planning
cargo run -p rust_robotics --example headless_clearance_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_traversal_risk_sweep --no-default-features --features planning
```

The example compares a distance-only route against a risk-aware route. With
`risk_weight=0`, the planner takes the short center corridor. With higher risk
weight, it detours around the hazardous band even though the path is longer.

Render the docs SVG asset:

```bash
cargo run -p rust_robotics --example render_traversal_risk_graph_svg --no-default-features --features planning
```

This writes `docs/assets/traversal-risk-graph-demo.svg`, a static heatmap with
the distance-only and risk-aware routes overlaid.

The sweep example writes:

- `docs/assets/traversal-risk-weight-sweep.csv`
- `docs/assets/traversal-risk-weight-sweep.svg`

Natural next slices:

- add diagonal-motion and footprint-aware comparisons.
