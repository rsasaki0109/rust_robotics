# Decisions

## Context

- Problem preset: `control-drone-trajectory-pass-through-accel`

## Adopt

- Keep `quintic-distance-scaled` as the current practical default after the richer acceleration-boundary re-run.
- Keep the stable aerial surface unchanged. The preset reuses the same sampled desired-state helpers and does not justify a wider public interface.

## Reject

- Reject promoting `minimum-snap-distance-scaled` on the current acceleration-boundary evidence. It remained slower and worse on the key quality metrics.
- Reject reading the richer boundary model as a sufficient explanation by itself. The controller is still the same PD tracker and waypoint jerk is still zeroed.

## Next

- Re-run with either non-zero waypoint jerk or a coupled multi-segment optimizer, not only finite-difference acceleration.
- If the generator ranking is still unchanged after that, shift effort to drone controller variants instead of widening trajectory helpers again.
