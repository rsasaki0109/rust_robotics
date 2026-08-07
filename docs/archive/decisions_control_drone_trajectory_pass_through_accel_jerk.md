# Decisions

## Context

- Problem preset: `control-drone-trajectory-pass-through-accel-jerk`

## Adopt

- Keep `quintic-distance-scaled` as the current practical default after the jerk-boundary re-run.
- Keep the stable aerial helper surface unchanged. The preset reuses the same sampled desired-state helpers and does not justify a wider public interface.

## Reject

- Reject promoting `minimum-snap-distance-scaled` on the current jerk-boundary evidence. It remained slower and worse on the key quality metrics.
- Reject treating finite-difference waypoint jerk alone as sufficient evidence for widening the stable surface. The current jerk synthesis is still heuristic and the controller is still the same PD tracker.

## Next

- Re-run with coupled multi-segment continuity or a more principled boundary optimizer instead of only local finite differences.
- If the ranking still does not flip, shift effort to drone controller variants and feedforward tracking instead of widening trajectory helpers again.
