# Decisions

## Context

- Problem preset: `crossover-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` scenarios per bucket and therefore defines the current reference labels.
- Keep `variance-triggered` as the best current exploratory proxy. It reduces evaluation cost from `18582.57 ms` to `16356.51 ms`, while keeping hard agreement with `full-bucket` at `0.933` and near-tie-aware agreement at `0.933` on the current problem.
- Keep `first-scenario` as the cheapest smoke-test strategy. It runs in `1896.14 ms`, but its hard agreement with `full-bucket` is only `0.533` and its near-tie-aware agreement is `0.533`, so it should not drive design decisions.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`4` families) and emit the same `RuntimeObservation` records.
- `RuntimeSamplingPlan` has now been reused across multiple validated problem presets without adding new core methods.
- Treat buckets inside the near-tie band (`|ratio - 1.0| < 0.020`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive planner wins.

## Reject

- Reject single-path convergence. The current comparison set shows that cheaper variants distort the winner label on some buckets even when they are useful for search.
- Reject the idea that one aggregation strategy should become the only path. The current repository should keep `full-bucket` for convergence and cheaper variants for search.

## Next

- Keep the shared comparison contract from `rust_robotics_core::experiments` fixed while it is reused by planning, localization, and control.
- If the gap between `variance-triggered` and `full-bucket` remains large, add another disposable variant instead of widening the core again.
- Keep only the comparison contract in core. Leave the concrete variants under `experiments/` until at least one more independent package uses the same interface.

## Related Planning Preset

- `grid-threshold-planners` currently keeps `AStarPlanner` as the reference winner on median-timed runtime while `FringeSearchPlanner` stays a useful lower-peak-live alternate design and `IDAStarPlanner` remains experimental.
- The current larger-window sweep improved `IDA*` again. On `arena2` bucket-`5` windows (`margin = 8 / 16 / 24`), all `9/9` bounded `IDA*` runs solved exactly after the threshold tolerance fix.
- The current cheap-budget floor on that slice is `22` expanded nodes; `21` still fails.
- On the `arena2` bucket-`11` slice, the cheap-budget floor is `43`; `42` still fails.
- That cheap floor does not generalize to the first `arena2` bucket-`12` slice. There, `131` still fails and `132` is the current minimum exact budget.
- On `arena2` buckets `10` and `15`, the same one-iteration policy still stops on `max_iterations`, so some slices need more threshold rounds instead of just a larger expansion budget.
- On `arena2` bucket `10`, a threshold-round sweep restores exactness at `32` with `24` still failing.
- On `arena2` bucket `15`, the same sweep still fails on `max_iterations` through `96` and then hits `max_expanded_nodes` at `128`.
- On `arena2` bucket `15`, raising the expansion ceiling still does not recover exactness in the current tested bands: `look2-iter128` fails through `exp800000`, and `look2-iter256` fails through `exp1200000`.
- On `arena2` bucket `15`, representative `window16` still fails through `iter256-exp3000000`, and on the full slice `look2-iter512-exp1750775` stays mixed while `look2-iter512-exp1750776` reaches exact.
- On the same bucket-`15` slice, the new failure-preserving `IDA*` reports show near-floor runs are overwhelmingly re-expansion-dominated, and once the budget is fixed at `look2-iter512-exp1750776`, the iteration floor is `275`, with `274` still failing on `max_iterations` and leaving a mean next-threshold gap of `0.058875`.
- This preset did not justify a new core abstraction; the stable boundary stays at `PathPlanner`.
- Detailed decision log: [`docs/decisions_planning_grid_threshold.md`](decisions_planning_grid_threshold.md)

## Related Control Preset

- `control-drone-trajectory-variants` currently keeps the stable aerial helper surface at sampled desired states plus duration-aware quintic generation.
- On the latest local run, `quintic-distance-scaled` is the practical default because it reduces mean peak speed and acceleration against `quintic-uniform` without materially changing generation cost.
- `minimum-snap-distance-scaled` stays experimental on the current stop-go waypoint preset because it is slower and more aggressive than the quintic variants.
- Detailed decision log: [`docs/decisions_control_drone_trajectory.md`](decisions_control_drone_trajectory.md)
- `control-drone-trajectory-pass-through` did not change that decision. Under the current heuristic pass-through velocity policy, `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on cost and quality.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_pass_through.md`](decisions_control_drone_trajectory_pass_through.md)
- `control-drone-trajectory-pass-through-accel` still did not flip the ranking. Adding finite-difference waypoint acceleration improved the boundary model, but `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on the current evidence.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_pass_through_accel.md`](decisions_control_drone_trajectory_pass_through_accel.md)
- `control-drone-trajectory-pass-through-accel-jerk` also did not flip the ranking. Adding finite-difference waypoint jerk changed only the minimum-snap branch and still was not enough to justify promoting it.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_pass_through_accel_jerk.md`](decisions_control_drone_trajectory_pass_through_accel_jerk.md)
- `control-drone-trajectory-coupled-continuity` shows that much smoother boundary synthesis alone still does not solve the comparison. It sharply reduces jerk and snap, but the current PD tracker follows the resulting trajectories worse on the harder families.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_coupled_continuity.md`](decisions_control_drone_trajectory_coupled_continuity.md)
- `control-drone-trajectory-coupled-continuity-controllers` shows controller-side leverage is real. Pure attitude-rate damping is mixed, but a bounded lateral-feedback variant on top of that damping now looks like the strongest current controller candidate on the coupled preset while still not justifying a wider stable API or default promotion yet.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_coupled_continuity_controllers.md`](decisions_control_drone_trajectory_coupled_continuity_controllers.md)
- `control-drone-trajectory-noncoupled-controllers` shows that the same bounded lateral-feedback controller also generalizes across the tested non-coupled presets, so it is now the leading experimental controller candidate on this branch even though it still should not be promoted to a stable default yet.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_noncoupled_controllers.md`](decisions_control_drone_trajectory_noncoupled_controllers.md)
- `control-drone-trajectory-controller-generator-pairings` shows that better tracking does move some generator winner labels, but not enough to overturn the practical branch-wide choice: `quintic-distance-scaled` still owns the non-coupled presets, while coupled-continuity only gives `minimum-snap-distance-scaled` a tiny coupled-only edge at much higher generation cost.
- Detailed decision log: [`docs/decisions_control_drone_trajectory_controller_generator_pairings.md`](decisions_control_drone_trajectory_controller_generator_pairings.md)
