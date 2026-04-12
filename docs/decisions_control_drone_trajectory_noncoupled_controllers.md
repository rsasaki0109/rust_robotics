# Decisions

## Context

- Problem preset: `control-drone-trajectory-noncoupled-controllers`

## Adopt

- Treat bounded lateral position/velocity feedback plus attitude-rate damping as the current leading experimental controller variant across the tested non-coupled drone trajectory presets.
- Keep the controller comparison local to the drone trajectory experiment harness. The result is strong evidence, but it still does not justify widening the stable control API.

## Reject

- Reject promoting the bounded-feedback controller to a repository-wide default tracker yet. The gain is still only validated inside this package-local experiment harness with one tuned parameter set.
- Reject reading the result as proof that generator choice no longer matters. The controller got better, but the branch still has not answered the full controller + generator pairing question explicitly.

## Next

- Run an explicit controller + generator pairing summary across both coupled and non-coupled presets so the branch can tell whether the improved tracker changes the practical generator ranking.
- If this controller path remains attractive, probe mild gain sensitivity before treating the current constants as more than experiment-local tuning.
