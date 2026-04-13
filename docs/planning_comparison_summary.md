# Planning Algorithm Comparison Summary

This report captures the Task F experiment snapshot for grid planners using the benchmark example in `rust_robotics`.

## Benchmark Command

```bash
cargo run -p rust_robotics --example benchmark_grid_planners --features planning 2>/dev/null
```

## Captured Benchmark Output

```text
| Planner | Time (us) | Path Length |
|---------|-----------:|------------:|
| A* | 161342 | 128.368 |
| Dijkstra | 1090664 | 130.953 |
| JPS | 138790 | 128.368 |
| Theta* | 135274 | 123.065 |
| LPA* | 186686 | 128.368 |
| ARA* | 155118 | 128.368 |
| Fringe Search | 1028065 | 128.368 |
| IDA* | 78187702 | 128.368 |
```

## Interpretation and Usage Guidance

- **A\***: Use as the default general-purpose planner on grids when you need optimal paths with predictable behavior and straightforward tuning.
- **JPS**: Use on uniform-cost grids (4/8-connected occupancy grids) where A* is spending effort on symmetric expansions; JPS preserves optimality while typically running much faster.
- **Fringe Search**: Use when memory pressure is important but you still require the same optimality target as A*; expect similar solution quality with different expansion ordering and potential runtime tradeoffs.
- **IDA\***: Use when memory is extremely limited and A*/Fringe node storage is not feasible; it is often much slower due to repeated depth-bounded iterations.
- **LPA\***: Use in dynamic environments where edge costs or obstacles change incrementally; it reuses previous search effort and avoids full replanning from scratch.
- **ARA\***: Use in real-time systems that need an anytime answer; it returns a fast suboptimal path first and improves toward optimality as more planning time is available.
- **Theta\* / Lazy Theta\***: Use when any-angle paths are needed instead of grid-constrained stair-step paths; this usually gives shorter, smoother trajectories for downstream controllers.

## Recommendation Table

| Scenario | Recommended | Why |
|----------|-------------|-----|
| Static map, single-shot planning, correctness first | A* | Reliable baseline with guaranteed optimal path on admissible heuristic settings. |
| Large uniform-cost occupancy grid with strict latency budget | JPS | Strong speedup vs A* on symmetric grid expansions while preserving optimality. |
| RAM-limited embedded runtime, but optimal path still required | Fringe Search | Lower practical memory pressure than classic A* open/closed management in many cases. |
| Very tight memory ceiling (cannot hold large frontier sets) | IDA* | Uses depth-first iterative deepening with minimal memory, trading heavily for runtime. |
| Repeated replanning after local map/cost updates | LPA* | Incremental updates reuse prior search tree and reduce repeated work. |
| Hard real-time loop requiring immediate feasible path then refinement | ARA* | Anytime behavior provides quick initial path and progressively better solutions. |
| Navigation where path smoothness and heading changes matter | Theta* / Lazy Theta* | Any-angle planning reduces zig-zag artifacts and often shortens route length. |

