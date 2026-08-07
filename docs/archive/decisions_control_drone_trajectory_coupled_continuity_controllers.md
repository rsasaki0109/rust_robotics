# Decisions

## Context

- Problem preset: `control-drone-trajectory-coupled-continuity-controllers`

## Adopt

- Keep the controller comparison local to the drone trajectory experiment harness. It is useful evidence, but it still does not justify a wider stable control API.
- Treat bounded lateral position/velocity feedback plus attitude-rate damping as the strongest current controller candidate on the coupled preset because it recovers the figure-eight regressions, sharply lowers mean tracking error, and still keeps control effort far below the baseline tracker.

## Reject

- Reject declaring the bounded-feedback variant the new default tracker. Its gains are only validated on this coupled-continuity experiment path, so the result is still too narrow for a stable controller promotion.
- Reject reading this result as proof that the generator question is settled. The outcome is still interaction-dependent across trajectory family and controller choice.

## Next

- Re-run the improved controller on non-coupled presets to check whether the gain survives outside the coupled-continuity boundary policy.
- If this line continues, evaluate controller + generator pairings explicitly rather than assuming one controller will dominate every trajectory family.
