# Long Range Navigator-lite: Frontier Navigation

Source theme: long-range navigation past the local map (Long Range Navigator).

This reproduces the decision core of long-range navigation without learned
affordances: a robot with a limited, occlusion-aware sensor cannot see a distant
goal, so it repeatedly picks the frontier (a known-free cell bordering unknown
space) that best advances toward the goal and hands off to a local planner to
drive there, revealing more of the world until a path to the goal appears.

## Implemented slice

- Occlusion-aware sensing: cells are revealed only along a clear line of sight
  within the sensor range, so obstacles cast unknown shadows whose edges become
  frontiers, and the robot accumulates a persistent known map.
- Frontier extraction and clustering: known-free cells bordering unknown space
  are grouped into 8-connected clusters with a centroid representative and an
  openness (bordered-unknown) count.
- Affordance scoring: each frontier is scored by goal progress, the known-free
  Dijkstra travel cost to reach it, whether it is in direct line of sight, and
  its openness (information gain).
- Local-planner handoff: a Dijkstra distance field over the known-free map; the
  robot follows its gradient toward the chosen frontier for a bounded step
  budget before re-sensing. When the goal becomes known and reachable, a final
  Dijkstra plan drives straight to it.

## Primary files

- `crates/rust_robotics_planning/src/frontier_navigator.rs`
- `crates/rust_robotics/examples/render_frontier_navigator_svg.rs`
- `docs/assets/frontier-navigator.svg`

## Run

```bash
cargo run -p rust_robotics --example render_frontier_navigator_svg --no-default-features --features planning
```

## Result

On a world with two offset walls the robot cannot see past, it selects frontiers
that route it around each occlusion in an S, revealing the far side until the
goal becomes reachable and the local planner finishes the trip — reaching the
goal deterministically with the full known map drawn for inspection.

## Next useful extension

Replace the hand-tuned affordance weights with a learned frontier value
(predicting reachable progress from local map features), and add multi-goal /
patrol frontier scheduling.
