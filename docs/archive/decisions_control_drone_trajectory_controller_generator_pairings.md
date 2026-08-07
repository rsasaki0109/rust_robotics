# Decisions

## Context

- Problem preset: `control-drone-trajectory-controller-generator-pairings`

## Adopt

- Keep `quintic-distance-scaled` as the branch-wide practical generator default. Under the bounded lateral-feedback controller it still leads all four tested non-coupled presets on mean RMSE and mean max error while remaining much cheaper than `minimum-snap-distance-scaled`.
- Treat the bounded lateral-feedback controller as real generator-ranking evidence, not just a controller-only win. It changes some local winner labels, so future generator decisions should continue to be made on paired controller + generator evidence rather than on trajectory generation alone.
- Treat the coupled-continuity `minimum-snap-distance-scaled` edge as a narrow interaction-specific result, not as a global promotion signal. The quality lead is real but very small, and it still comes with materially higher generation cost and somewhat higher torque.

## Reject

- Reject claiming that the improved controller makes generator choice irrelevant. The explicit pairing rerun shows the winner label can still move when the controller changes.
- Reject promoting `minimum-snap-distance-scaled` to the practical default from the coupled-continuity pairing alone. Its lead there is too small and too expensive to outweigh the broader non-coupled evidence.
- Reject widening the stable control or trajectory API from this result. The evidence sharpens the experiment decision, but it still does not justify a new public abstraction.

## Next

- Gain sensitivity sweep is now complete: default gains are within 10% of the best sweep point, and the paired ranking is stable to mild retuning. No further gain sweep is needed on the current families.
- Coupled-continuity family set is now expanded to 5 families. The bounded lateral-feedback controller wins across all of them.
- If this track is revisited, the next useful increment would be:
  - testing on different waypoint geometries (e.g., aggressive 3D maneuvers, tight corridors)
  - or sweeping the saturation bounds (max_lateral_correction, max_attitude_command) rather than the proportional gains
