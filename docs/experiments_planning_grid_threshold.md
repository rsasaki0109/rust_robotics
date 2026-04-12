# Experiments

## Problem

Preset: `grid-threshold-planners`

Compare three grid planners that share the same `PathPlanner` contract but use different frontier mechanics:

- `AStarPlanner`
- `FringeSearchPlanner`
- `IDAStarPlanner`

The narrow slice is plan-only runtime on deterministic local fixtures while holding path quality constant.

## Shared Input

- contract: `PathPlanner::plan(Point2D, Point2D) -> Result<Path2D, RoboticsError>`
- obstacle inflation: `resolution = 1.0`, `robot_radius = 0.5`
- datasets:
  - `simple-barrier`: bounded `10x10` room with a short internal wall
  - `movingai-sample`: first scenario from `src/testdata/moving_ai/sample.map.scen`
  - `open-grid`: bounded `20x20` empty room
- timing method: median wall-clock microseconds across `5` timing windows per planner/case
  - each window runs `plan_with_stats()` for at least `100 ms` and at least `8` calls after `8` warmup calls
- quality metric: path length ratio versus `A*` on the same case
- output size metric: waypoint count
- effort metrics:
  - expanded/generated/pruned counts
  - control rounds
  - peak live search state
    - `A*`: max priority-queue frontier length
    - `Fringe Search`: max combined `current_fringe + next_fringe`
    - `IDA*`: max recursion-path depth

## Variant Summary

| Variant | Style | Mean runtime us | Mean waypoints | Mean length / A* | Mean expanded | Mean generated | Mean pruned | Mean rounds | Mean peak live | Code LOC | Branches | Knobs |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `a-star` | priority-queue-best-first | 161.71 | 10.67 | 1.0000 | 19.33 | 94.67 | 6.67 | 1.00 | 70.67 | 370 | 18 | 1 |
| `fringe-search` | dual-fringe-threshold | 230.13 | 10.67 | 1.0000 | 20.00 | 56.00 | 139.00 | 7.67 | 34.67 | 340 | 19 | 1 |
| `ida-star` | recursive-contour-threshold | 313.10 | 10.67 | 1.0000 | 18.67 | 48.67 | 49.33 | 2.00 | 10.67 | 474 | 27 | 3 |

## Case Comparison

| Planner / Case | Runtime us | Path length | Expanded | Generated | Pruned | Rounds | Peak live | Waypoints |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `a-star / simple-barrier` | 320.65 | 10.2426 | 36 | 156 | 20 | 1 | 102 | 10 |
| `a-star / movingai-sample` | 24.52 | 4.0000 | 5 | 15 | 0 | 1 | 12 | 5 |
| `a-star / open-grid` | 139.95 | 22.6274 | 17 | 113 | 0 | 1 | 98 | 17 |
| `fringe-search / simple-barrier` | 447.73 | 10.2426 | 38 | 76 | 323 | 19 | 30 | 10 |
| `fringe-search / movingai-sample` | 26.94 | 4.0000 | 5 | 9 | 0 | 1 | 6 | 5 |
| `fringe-search / open-grid` | 215.72 | 22.6274 | 17 | 83 | 94 | 3 | 68 | 17 |
| `ida-star / simple-barrier` | 760.47 | 10.2426 | 34 | 126 | 148 | 4 | 10 | 10 |
| `ida-star / movingai-sample` | 34.18 | 4.0000 | 5 | 4 | 0 | 1 | 5 | 5 |
| `ida-star / open-grid` | 144.66 | 22.6274 | 17 | 16 | 0 | 1 | 17 | 17 |

## Windowed Larger Spot Check

- case: `movingai-arena2-b5-window16`
- window policy: crop a `16`-cell margin around the first `arena2` bucket-`5` scenario before building planner obstacles
- bounded `IDA*` policy: `initial_lookahead_depth = 2`, `max_iterations = 1`, `max_expanded_nodes = 500`

| Planner / Case | Runtime us | Path length | Expanded | Generated | Pruned | Rounds | Peak live | Waypoints |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `a-star / movingai-arena2-b5-window16` | 723.34 | 23.7990 | 79 | 365 | 50 | 1 | 238 | 19 |
| `fringe-search / movingai-arena2-b5-window16` | 60471.13 | 23.7990 | 83 | 204 | 3165 | 70 | 69 | 19 |
| `ida-star-bounded / movingai-arena2-b5-window16` | 192.74 | 23.7990 | 22 | 32 | 16 | 1 | 19 | 19 |

## Practical Bounded IDA* Sweep

- dataset family: `arena2`
- scenario slice: first bucket-`5` scenario
- windows: `margin = 8 / 16 / 24`
- bounded `IDA*` profiles:
  - `look2-iter1-exp500`
  - `look8-iter2-exp2000`
  - `look8-iter16-exp80000`
- exact `A*` and `Fringe Search` still matched on every sweep case
- all `9/9` bounded `IDA*` runs completed exactly after the threshold tolerance fix

| Case | IDA* profile | Runtime us | Outcome |
| --- | --- | ---: | --- |
| `movingai-arena2-b5-window08` | `look2-iter1-exp500` | 211.14 | `exact` |
| `movingai-arena2-b5-window08` | `look8-iter2-exp2000` | 3437.70 | `exact` |
| `movingai-arena2-b5-window08` | `look8-iter16-exp80000` | 3721.07 | `exact` |
| `movingai-arena2-b5-window16` | `look2-iter1-exp500` | 185.32 | `exact` |
| `movingai-arena2-b5-window16` | `look8-iter2-exp2000` | 4207.21 | `exact` |
| `movingai-arena2-b5-window16` | `look8-iter16-exp80000` | 3345.65 | `exact` |
| `movingai-arena2-b5-window24` | `look2-iter1-exp500` | 3234.16 | `exact` |
| `movingai-arena2-b5-window24` | `look8-iter2-exp2000` | 3610.47 | `exact` |
| `movingai-arena2-b5-window24` | `look8-iter16-exp80000` | 4191.72 | `exact` |

## Cheap Budget Floor

- dataset family: `arena2`
- scenario slice: first bucket-`5` scenario
- windows: `margin = 8 / 16 / 24`
- fixed bounded profile shape: `initial_lookahead_depth = 2`, `max_iterations = 1`
- swept budgets: `18 / 20 / 21 / 22 / 24`

| Budget profile | Outcome summary |
| --- | --- |
| `look2-iter1-exp18` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp20` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp21` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp22` | `3/3` exact |
| `look2-iter1-exp24` | `3/3` exact |

## Harder Slice Cheap Budget Floor

- dataset family: `arena2`
- scenario slice: first bucket-`12` scenario
- windows: `margin = 8 / 16 / 24`
- fixed bounded profile shape: `initial_lookahead_depth = 2`, `max_iterations = 1`
- swept budgets: `22 / 128 / 129 / 130 / 131 / 132 / 133`

| Budget profile | Outcome summary |
| --- | --- |
| `look2-iter1-exp22` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp128` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp129` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp130` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp131` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp132` | `3/3` exact |
| `look2-iter1-exp133` | `3/3` exact |

## Additional Harder Slice Reachability

- dataset family: `arena2`
- scenario slices: first bucket-`10`, bucket-`11`, and bucket-`15` scenarios
- windows: `margin = 8 / 16 / 24`
- probe profile: `initial_lookahead_depth = 2`, `max_iterations = 1`, `max_expanded_nodes = 10000`
- reference planner for path shape: exact `A*`

| Slice | Outcome summary | Reference path length | Reference waypoints |
| --- | --- | ---: | ---: |
| `bucket-10` | `3/3` `max_iterations` stop | `42.7990` | `38` |
| `bucket-11` | `3/3` exact | `44.7279` | `42` |
| `bucket-15` | `3/3` `max_iterations` stop | `63.9706` | `60` |

## Mid Slice Cheap Budget Floor

- dataset family: `arena2`
- scenario slice: first bucket-`11` scenario
- windows: `margin = 8 / 16 / 24`
- fixed bounded profile shape: `initial_lookahead_depth = 2`, `max_iterations = 1`
- swept budgets: `42 / 43 / 44`

| Budget profile | Outcome summary |
| --- | --- |
| `look2-iter1-exp42` | `3/3` `max_expanded_nodes` stop |
| `look2-iter1-exp43` | `3/3` exact |
| `look2-iter1-exp44` | `3/3` exact |

## Threshold-Round Floors On Harder Slices

- dataset family: `arena2`
- scenario slices: first bucket-`10` and bucket-`15` scenarios
- windows: `margin = 8 / 16 / 24`
- fixed bounded profile shape: `initial_lookahead_depth = 2`, `max_expanded_nodes = 100000`
- swept `max_iterations`: `1 / 2 / 3 / 4 / 6 / 8 / 12 / 16 / 24 / 32 / 48 / 64 / 96 / 128 / 192 / 256`

| Slice | Min exact iterations | Max `max_iterations` stop | First `max_expanded_nodes` stop | Outcome summary |
| --- | ---: | ---: | ---: | --- |
| `bucket-10` | `32` | `24` | `-` | raising threshold rounds alone restores exact reachability |
| `bucket-15` | `-` | `96` | `128` | no exact run in the current sweep; iteration and expansion budgets now interact |

## Bucket-15 Expansion Bands After Iteration Ramp

- dataset family: `arena2`
- scenario slice: first bucket-`15` scenario
- windows: `margin = 8 / 16 / 24`
- sweep A: `initial_lookahead_depth = 2`, `max_iterations = 128`, `max_expanded_nodes = 100000 / 125000 / 150000 / 175000 / 200000 / 250000 / 300000 / 400000 / 600000 / 800000`
- sweep B: `initial_lookahead_depth = 2`, `max_iterations = 256`, `max_expanded_nodes = 200000 / 250000 / 300000 / 400000 / 600000 / 800000 / 1200000`

| Sweep band | Outcome summary |
| --- | --- |
| `look2-iter128-exp100000..200000` | `3/3` `max_expanded_nodes` stop |
| `look2-iter128-exp250000..800000` | `3/3` `max_iterations` stop |
| `look2-iter256-exp200000..1200000` | `3/3` `max_expanded_nodes` stop |

## Bucket-15 Ultra-High Representative Probe

- dataset family: `arena2`
- scenario slice: bucket-`15`, first scenario, representative `window16`
- fixed profile shape: `initial_lookahead_depth = 2`, `max_iterations = 256`
- swept `max_expanded_nodes`: `1500000 / 2000000 / 2500000 / 3000000`

| Budget profile | Outcome summary |
| --- | --- |
| `look2-iter256-exp1500000..3000000` | representative `window16` stayed `max_iterations` throughout |

## Bucket-15 Iter512 Budget Floor

- dataset family: `arena2`
- scenario slice: first bucket-`15` scenario
- windows: `margin = 8 / 16 / 24`
- fixed profile shape: `initial_lookahead_depth = 2`, `max_iterations = 512`
- swept `max_expanded_nodes`: `1500000 / 2000000`

| Budget profile | Outcome summary |
| --- | --- |
| `look2-iter512-exp1500000` | `3/3` `max_expanded_nodes` stop |
| `look2-iter512-exp2000000` | `3/3` exact |

## Bucket-15 Iter512 Budget Refinement

- dataset family: `arena2`
- representative refinement: `window16`, `initial_lookahead_depth = 2`, `max_iterations = 512`
- representative swept `max_expanded_nodes`: `1750775 / 1750776`
- full-slice refinement: first bucket-`15` scenario, `margin = 8 / 16 / 24`
- full-slice swept `max_expanded_nodes`: `1750775 / 1750776`

| Budget profile | Outcome summary |
| --- | --- |
| representative `look2-iter512-exp1750775` | `window16` `max_expanded_nodes` stop |
| representative `look2-iter512-exp1750776` | `window16` exact |
| full-slice `look2-iter512-exp1750775` | mixed: `window08` exact, `window16/24` `max_expanded_nodes` stop |
| full-slice `look2-iter512-exp1750776` | `3/3` exact |

| Budget profile | Exact runs | Mean expanded | Mean unique | Mean re-expanded | Mean rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| full-slice `look2-iter512-exp1750775` | `1/3` | `1750156.7` | `1022.0` | `1749134.7` | `275.0` |
| full-slice `look2-iter512-exp1750776` | `3/3` | `1750157.3` | `1022.7` | `1749134.7` | `275.0` |

- At `look2-iter512-exp1750775`, `window08` already reaches exact after `1000` unique expansions, while `window16/24` stop at `1033` unique expansions and become exact at `1034`.
- That means the current full-slice `1750775 -> 1750776` boundary is not a broad runtime shift. It is a one-more-unique-state edge on the harder windows after `275` threshold rounds have already been spent.

## Bucket-15 Fixed-Budget Iteration Boundary

- dataset family: `arena2`
- fixed budget: `initial_lookahead_depth = 2`, `max_expanded_nodes = 1750776`
- full-slice iteration sweep: `273 / 274 / 275`

| Iteration profile | Outcome summary |
| --- | --- |
| full-slice `look2-iter273-exp1750776` | `3/3` `max_iterations` stop |
| full-slice `look2-iter274-exp1750776` | `3/3` `max_iterations` stop |
| full-slice `look2-iter275-exp1750776` | `3/3` exact |

| Iteration profile | Exact runs | Mean expanded | Mean unique | Mean re-expanded | Mean rounds | Mean next-threshold gap |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| full-slice `look2-iter273-exp1750776` | `0/3` | `1719882.7` | `1019.7` | `1718863.0` | `273.0` | `0.041631` |
| full-slice `look2-iter274-exp1750776` | `0/3` | `1735617.3` | `1020.7` | `1734596.7` | `274.0` | `0.058875` |
| full-slice `look2-iter275-exp1750776` | `3/3` | `1750157.3` | `1022.7` | `1749134.7` | `275.0` | `-` |

- The failure-side threshold gap is consistent across all three windows: `iter273` still needs `0.041631` more `f` cost, and `iter274` still needs `0.058875`.
- The `274 -> 275` flip is therefore not just a floating-point tie-break. At the fixed exact budget, the last missing contour still sits a measurable threshold step away.

## Trade-offs

- Quality tied on the current cases. All three variants matched `A*` path length exactly.
- `A*` won on the current median wall-clock timing while keeping control rounds fixed at `1`, but it also carried the widest live frontier.
- `Fringe Search` reduced mean generated nodes and roughly halved peak live search state versus `A*`, but it paid for that with more deferred/pruned work and more threshold rounds.
- `IDA*` still lagged on the local fixtures, but contour-local `best-g` pruning, heuristic neighbor ordering, the tighter octile lower bound, the admissible start-lookahead backup, and threshold tolerance cut its mean runtime from roughly `10 ms` down to roughly `0.31 ms` on the current machine.
- On the bucket-`5` larger-window spot check, exact `A*`, `Fringe Search`, and bounded `IDA*` all matched.
- On the practical sweep, all three bounded profiles reached exact solutions on all three windows after the threshold tolerance fix.
- On the cheap-budget sweep, the current exact floor is `22` expanded nodes and the largest failing budget is `21`.
- On the bucket-`11` mid slice, the cheap exact floor is `43` expanded nodes and the largest failing budget is `42`.
- On the harder bucket-`12` slice, the cheap exact floor rises to `132` expanded nodes and the largest failing budget is `131`, so the bucket-`5` floor does not generalize.
- On buckets `10` and `15`, the same `look2-iter1` profile does not reach an exact solution even with `max_expanded_nodes = 10000`; it stops on `max_iterations` instead.
- On bucket `10`, increasing only `max_iterations` eventually restores exactness at `32` rounds, with `24` still failing.
- On bucket `15`, increasing only `max_iterations` is not enough under `max_expanded_nodes = 100000`; the sweep still fails on `max_iterations` through `96`, then flips to `max_expanded_nodes` at `128`.
- On bucket `15`, increasing `max_expanded_nodes` after that flip is still not enough in the tested bands: `iter128` reaches no exact run through `exp800000`, and `iter256` still reaches no exact run through `exp1200000`.
- On bucket `15`, even the representative `window16` case still fails on `max_iterations` through `iter256-exp3000000`.
- On bucket `15`, raising both resources together eventually restores exactness: under `iter512`, `exp1500000` still fails, the full-slice exact floor now refines to `exp1750776`, and `exp1750775` is still non-exact.
- On bucket `15`, once that exact budget is fixed at `exp1750776`, the corresponding full-slice iteration floor is `275`, with `274` still stopping on `max_iterations`.
- The new failure-preserving report path shows why bucket `15` is hard: even near the exact floor, the runs spend about `1.75M` total expansions to touch only about `1.0k` unique states, so the dominant cost is repeated contour re-expansion rather than frontier breadth.
- The `iter512-exp1750775` split is narrow but real: `window16/24` need one more unique expansion than the failing run budget allows.
- The fixed-budget `iter274` failure is also not a bookkeeping artifact: it still leaves a mean next-threshold gap of `0.058875` before the `iter275` exact contour.

## Core / Experiments

- Stable boundary: `rust_robotics_core::PathPlanner`
- Compared concrete variants:
  - `crates/rust_robotics_planning/src/a_star.rs`
  - `crates/rust_robotics_planning/src/fringe_search.rs`
  - `crates/rust_robotics_planning/src/ida_star.rs`
- Package-local diagnostics used by this preset: `IDAStarPlanner::plan_with_report(...) -> IDAStarPlanReport`, including `unique_expanded_nodes`, `reexpanded_nodes`, `last_searched_threshold`, `next_threshold`, and `contour_history: Vec<ContourStats>`

## Per-Contour Diagnostics (Bucket 15, iter274 vs iter275)

Per-contour effort breakdown on `movingai-arena2-b15-window08` with fixed budget `exp1750776`:

### iter274 (max\_iterations)

| Round | Threshold | Expanded | New unique | Re-expanded | Generated | Thr prune | Cycle pr | Trn pr | Depth |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 43.7696 | 6 | 6 | 0 | 27 | 22 | 5 | 7 | 4 |
| 2 | 44.3553 | 10 | 3 | 7 | 40 | 31 | 10 | 15 | 5 |
| 3 | 44.5980 | 13 | 2 | 11 | 47 | 35 | 13 | 23 | 5 |
| ... | | | | | | | | | |
| 273 | 63.8701 | 15644 | 2 | 15642 | 19465 | 3822 | 16660 | 83030 | 59 |
| 274 | 63.9117 | 15672 | 1 | 15671 | 19486 | 3815 | 16688 | 83177 | 59 |

- Total: 1,734,404 expanded, 998 unique, re-expansion ratio 99.94%
- Peak re-expansion contour: round 274

### iter275 (exact)

| Round | Threshold | Expanded | New unique | Re-expanded | Generated | Thr prune | Cycle pr | Trn pr | Depth |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 43.7696 | 6 | 6 | 0 | 27 | 22 | 5 | 7 | 4 |
| ... | | | | | | | | | |
| 274 | 63.9117 | 15672 | 1 | 15671 | 19486 | 3815 | 16688 | 83177 | 59 |
| 275 | 63.9706 | 14516 | 2 | 14514 | 18242 | 3727 | 15410 | 76620 | 60 |

- Total: 1,748,920 expanded, 1,000 unique, re-expansion ratio 99.94%
- Depth reached 60 on the final contour (59 in all prior contours)

### Interpretation

The iter274/275 boundary is explained by a single threshold step (63.9117 to 63.9706) that raises the achievable recursion depth from 59 to 60, allowing the search to reach the goal. The threshold gap between successive contours narrows to a mean of ~0.074 in the late contours, so each additional iteration only slightly widens the reachable `f`-cost band while re-expanding nearly all previously visited states. This confirms that on bucket 15 the dominant cost is contour re-expansion (99.94% of all expansions), not frontier growth.
- This preset does not promote a new planning-specific core abstraction.
