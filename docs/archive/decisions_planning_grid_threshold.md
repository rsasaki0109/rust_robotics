# Decisions

## Context

- Preset: `grid-threshold-planners`
- Scope: compare `A*`, `Fringe Search`, and `IDA*` under the same `PathPlanner` request/result shape on small local fixtures.

## Adopt

- Keep `AStarPlanner` as the convergence reference for this preset. It matched the best quality, had the best current median runtime at `161.71 us`, and avoided threshold-style outer control loops.
- Keep `FringeSearchPlanner` as a useful alternate frontier design. It preserves quality and cuts mean peak live search state to `34.67` versus `70.67` for `A*`, even though it was slower and needed `7.67` threshold rounds on average.
- Keep the contour-local `best-g` pruning, heuristic neighbor ordering, octile lower bound, admissible start-lookahead backup, and threshold tolerance in `IDA*`. Together they materially reduced re-expansion cost without breaking local optimal-length parity.
- Keep the bucket-`5` windowed sweep as the larger-map smoke test for now. It is still cheap enough for a unit-test-time check while being materially harder than the previous trivial bucket-`0` slice.
- Keep `look2-iter1-exp500` as the current larger-window exact floor for bounded `IDA*`. After the threshold tolerance fix, it solved all three current bucket-`5` windows exactly.
- Keep `22` expanded nodes as the current cheap exact floor on the bucket-`5` slice. `21` still stops on `max_expanded_nodes`, while `22` and `24` solve `3/3` windows exactly.
- Keep bucket-`11` as the current mid-slice finite-floor reference. On that slice, `42` still stops while `43` and `44` solve `3/3` windows exactly.
- Keep the bucket-`12` cheap-budget boundary as the current harder-slice counterexample. On that slice, `131` still stops while `132` and `133` solve `3/3` windows exactly.
- Keep buckets `10` and `15` as one-iteration reachability counterexamples. On those slices, `look2-iter1-exp10000` still stops on `max_iterations`.
- Keep bucket-`10` as the current threshold-round floor reference. Under `max_expanded_nodes = 100000`, `24` still stops on `max_iterations` while `32` and above solve `3/3` windows exactly.
- Keep bucket-`15` as the current threshold-round/expansion interaction counterexample. Under `max_expanded_nodes = 100000`, `96` still stops on `max_iterations`, and `128` is the first sweep point that flips to `max_expanded_nodes`.
- Keep the bucket-`15` high-iteration expansion band as the current severe counterexample. Even after raising the budget mix to `look2-iter256-exp1200000`, the tested windows still stop on `max_expanded_nodes`.
- Keep the bucket-`15` ultra-high representative band as the current next counterexample. On representative `window16`, even `look2-iter256-exp3000000` still stops on `max_iterations`.
- Keep the bucket-`15` `iter512` budget floor as the current high-cost recovery point. On the full `margin = 8 / 16 / 24` slice, `look2-iter512-exp1750775` is still non-exact while `look2-iter512-exp1750776` solves `3/3` windows exactly.
- Keep the bucket-`15` fixed-budget iteration floor as the current threshold-round recovery point. Once the budget is fixed at `look2-iter512-exp1750776`, `iter274` still stops on `max_iterations` while `iter275` solves `3/3` windows exactly.
- Keep the failure-preserving `IDAStarPlanReport` diagnostics in the package-local planning surface. They show the bucket-`15` edge is re-expansion-dominated: near the exact floor the full slice spends about `1.75M` total expansions to touch only about `1.0k` unique states.
- Keep the per-contour diagnostics (`ContourStats` in `contour_history`) in `IDAStarPlanReport`. They explain the iter274/275 boundary as a depth 59→60 transition with 99.94% re-expansion ratio, confirming that contour re-expansion, not frontier growth, dominates the budget on hard slices.
- Keep the stable boundary at `PathPlanner`. The current evidence does not justify a new planner-family abstraction.

## Reject

- Reject promoting `IDAStarPlanner` as the default grid reference. It matched quality, but its mean runtime `313.10 us` still trailed `A* 161.71 us`.
- Reject reading too much into the single trivial win where `IDA*` was fastest (`movingai-sample`). That case is too small to dominate the choice.
- Reject assuming the current bucket-`5` result generalizes automatically to harder windows. The bucket-`12` `margin = 8 / 16 / 24` slice raises the cheap exact floor from `22` to `132`.
- Reject assuming any single cheap-budget floor is global. The current evidence already splits between bucket-`5` (`22`) and bucket-`12` (`132`).
- Reject assuming the one-iteration policy degrades smoothly with map difficulty. The current evidence spans bucket-`11` exact at `43`, bucket-`12` exact at `132`, and buckets `10`/`15` failing on `max_iterations`.
- Reject assuming a modest iteration increase is always enough to repair one-iteration failures. Bucket `10` needs `32`, while bucket `15` still does not reach exactness by `96`.
- Reject assuming threshold rounds are the only missing resource on harder slices. Bucket `15` reaches `max_expanded_nodes` before exactness once `max_iterations` becomes large enough.
- Reject assuming a larger expansion ceiling repairs bucket `15` once threshold rounds are raised. `look2-iter128` still has no exact run through `exp800000`, and `look2-iter256` still has no exact run through `exp1200000`.
- Reject assuming `iter256` is already enough once the expansion ceiling is made very large. Representative `window16` still fails on `max_iterations` through `exp3000000`.
- Reject treating the `1750775 / 1750776` or `274 / 275` boundaries as noise. Per-contour diagnostics now show: the budget edge is a one-more-unique-state boundary on the harder windows; the `iter274/275` boundary is a depth 59→60 transition where the final threshold step (63.91→63.97) first allows reaching the goal; and both boundaries exhibit a 99.94% contour re-expansion ratio.

## Re-run Triggers

- Re-run this preset after adding planner statistics such as exact memory bytes, contour-cache pressure, or stack/recursion footprint.
- Re-run if `Fringe Search` or `IDA*` gains pruning, caching, or iterative implementations that materially change runtime.
- Re-run on larger MovingAI windows before making any package-level default change.
- Re-run the bucket-`5` larger-window sweep after changing the crop margin, the threshold update rule, or bounded `IDA*` cutoffs.
- Re-run this preset if threshold tolerances or floating-point comparison rules change, because the current cheaper exact result now depends on that boundary handling.
- Re-run the cheap-budget floor sweep after changing the budget check order or any pruning rule in `IDA*`.

## Next

- Tighten the bucket-`15` iteration boundary below `275` only if there is a reason to change the fixed budget. At the current exact budget, the floor is already pinned at `iter275`.
- The per-contour diagnostics (`ContourStats` / `contour_history`) now explain the iter274/275 boundary: the final contour raises achievable depth from 59 to 60 via a 0.059 threshold step, with 99.94% of all expansions being re-expansions. No further memory-oriented diagnostics are needed to explain this boundary.
- If stack/memory profiling is still desired, the next step would be measuring peak RSS or recursion stack bytes per contour, but the current contour-level evidence is sufficient for the practical question.
- Keep `IDA*` experimental until larger-map evidence improves.
