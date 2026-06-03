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
8. Sweep benchmarks over region size and agent density, with a bounded flat-CBS
   baseline, CSV output, and SVG charts for repair-group size, fallback, and
   runtime.

Run:

```bash
cargo run -p rust_robotics --example headless_hierarchical_mapf_replanning --no-default-features --features planning
cargo run -p rust_robotics --example render_hierarchical_mapf_replanning_svg --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_hierarchical_mapf_scale --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_hierarchical_mapf_sweeps --no-default-features --features planning
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

## Sweep benchmarks

`benchmark_hierarchical_mapf_sweeps` runs two sweeps over seeded splitmix64
scenes (the scenarios and all structural metrics are deterministic across runs;
only the measured `*_ms` columns vary):

- **Region-size sweep** — `docs/assets/hierarchical-mapf-region-sweep.{csv,svg}`.
  A fixed scene of six spatially separated head-on swaps is replanned at region
  edge lengths 4, 8, 12, and 24. Coarser regions merge neighboring swaps into a
  single CBS group, so the maximum repair-group size grows 2 → 4 → 6 → 12 and
  runtime climbs steeply (the merged swaps are independent, yet flat CBS still
  branches combinatorially over them — the motivation for region decomposition).

- **Density sweep** — `docs/assets/hierarchical-mapf-density-sweep.{csv,svg}`.
  Two through nine swap clusters (4–18 agents) are replanned at a fixed local
  region size. Hierarchical decomposition keeps every repair group at size two,
  so planning time stays low while the agent count grows. A *bounded* flat-CBS
  baseline is run only on a four-agent subset (flat CBS does not scale to the
  full scene) and confirmed conflict-free as a correctness reference.

Both sweeps keep `fallback_full_replan` false and finish with zero cell
conflicts. Region sizing must be at least as coarse as a local conflict's
spatial extent; a region finer than a swap can split the two swapping agents
into different regions, miss the trigger, and force the full-plan fallback.
