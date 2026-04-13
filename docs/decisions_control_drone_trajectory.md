# Decisions

## Context

- Problem preset: `control-drone-trajectory-variants`

## Adopt

- Keep the drone trajectory surface small: `generate_waypoint_trajectory_with_durations`, `sample_trajectory_segments`, and `simulate_desired_states` are the current stable helpers for aerial trajectory experiments.
- Treat `quintic-distance-scaled` as the current practical default for drone waypoint loops that need lower peak speed and peak acceleration without materially increasing generation cost.
- Keep `minimum_snap_trajectory` available, but treat it as experimental evidence rather than a new default controller path.

## Reject

- Reject promoting `minimum-snap-distance-scaled` as the default on the current stop-go waypoint preset. Its mean RMSE, jerk RMS, snap RMS, and generation runtime all came out worse than the quintic distance-scaled baseline on the latest local run.
- Reject widening the stable surface to a generator trait right now. The current preset does not justify a new abstraction beyond sampled desired states.

## Next

- Re-run the comparison with pass-through waypoint velocities or coupled multi-segment constraints. The current preset stops at every waypoint, which is a weak fit for minimum-snap trajectories.
- If a future preset shows a clear win for minimum-snap under non-stop-go boundaries, revisit whether any extra stable helper is justified.
