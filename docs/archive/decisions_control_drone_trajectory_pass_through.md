# Decisions

## Context

- Problem preset: `control-drone-trajectory-pass-through`

## Adopt

- Keep `quintic-distance-scaled` as the practical default even under the current pass-through waypoint policy.
- Keep the stable surface unchanged. This preset reuses the sampled desired-state helpers from the stop-go drone comparison and does not justify a larger aerial interface.

## Reject

- Reject promoting `minimum-snap-distance-scaled` based on the current pass-through evidence. It remained slower and worse on mean RMSE, jerk RMS, snap RMS, and peak speed.
- Reject interpreting pass-through velocities alone as sufficient evidence for minimum-snap adoption. The boundary policy still uses heuristic tangents and zero waypoint acceleration/jerk.

## Next

- Re-run with a stronger boundary model: non-zero waypoint acceleration, coupled multi-segment continuity, or a controller with more trajectory feedforward.
- If `minimum-snap-distance-scaled` starts winning under that richer preset, revisit whether any new stable helper should move out of `experiments`.
