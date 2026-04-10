# Decisions

## Context

- Problem preset: `control-drone-trajectory-coupled-continuity`

## Adopt

- Keep `quintic-distance-scaled` as the current practical default. The coupled preset adds useful evidence, but it does not overturn the runtime and simplicity advantages of the existing default.
- Keep the stable aerial helper surface unchanged. The coupled boundary synthesis is still experiment-local and does not yet justify a wider public interface.
- Treat controller/feedforward comparison as the next highest-value follow-up, because the current PD tracker becomes the obvious bottleneck once the reference trajectory is this smooth.

## Reject

- Reject promoting `minimum-snap-distance-scaled` on coupled-continuity evidence alone. Its mean RMSE is no longer clearly dominated, but it remains much slower and still does not win enough of the metric surface to justify adoption.
- Reject widening trajectory helpers to expose coupled boundary synthesis before it proves reusable outside this experiment path.

## Next

- Compare controller variants or trajectory feedforward tracking against the coupled-continuity preset instead of adding another local finite-difference boundary heuristic.
- If trajectory-side work continues, prefer a more principled duration/boundary optimizer over ad hoc scaling of the coupled solution.
