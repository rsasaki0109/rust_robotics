# Frozen live stationary-epoch preview experiment

Freeze this protocol before generating live candidate trajectories. Compare
`aligned` with opt-in `epoch`, using known seeds 268,269,270 and new seeds
271,272,273, all three loss modes, all nine map/planner cases, and local noise
correlation times 0 and 1 seconds. Use the existing fine-padding planning
fixtures. This is four 162-case runs, 648 cases total (324 per variant).
Keep known/new and white/correlated cohorts separate in the report.

Port the sensor-only StationaryEpochFusion prototype frozen at main 355e235
without changing its rules. Retain at most three received correction epochs,
each with at most 25 stationary observations at age <=0.5 seconds. Keep a
15-second inclusive horizon; require ten samples in each older epoch and one
in the current epoch. Clear history above measured speed magnitude 0.02 or
on precision activation. Missed corrections do not create epochs. Transport
retained samples using previous measured speed and previously applied steering.
Use equal weights for the three epoch means when current age <=1.5 seconds.

Replace only stopping-preview x,y when the fusion is available. Preserve the
existing motion-filter yaw and use the original filtered position otherwise.
The preview supplies the existing entry, release and joint recovery decisions,
so subsequent motion and observations may differ. Preserve raw recovery means,
nominal steering, speed-probe inputs, margins, timers, noise and goal logic.
Do not execute the offline selected steering arcs or add uncertainty bounds.
The new CLI option defaults to false and requires aligned terminal release.

Verify every physical interval, sensor/noise stream, filter and fusion state,
guard/recovery transition, timer, contact, boundary and hold result. Reconstruct
the Rust fusion from the unchanged Python prototype on every candidate row,
including unavailable rows. Audit stopping and recovery geometry with existing
independent full scans, including every guard trigger/release and loss resume.
For known aligned trajectories, require every existing CSV field and summary
to match the preceding holdout raw runs exactly with the option disabled.
Check each pair is identical before the first available fusion position, apart
from the new estimator bookkeeping fields. Keep all outcomes in the analysis.

Report paired held gains/losses, contacts, out-of-bounds, waits (union of guard
and loss pauses), completion times, short reentries, fusion use and fallback.
Evaluate original/fused position error at the same pre-command pose on every
available candidate row; these repeated rows are dependent observations.
Save per-case metrics, paired comparisons, cohort summaries, verification
counts, raw provenance and source hashes. Repeat aggregation byte for byte.
Keep raw trajectories outside Git and document exact reproduction commands.
Do not tune thresholds to these results or enable the option by default.

Logging clarification after the first generated batch: the old nine-decimal
speed field rounded values just above 0.02 onto the inclusive boundary, making
epoch-history replay ambiguous. Append measured_speed in round-trip precision
and regenerate the same matrix. Preserve all original fields, estimator rules
and controller behavior. Retain the initial raw batch and require identical
values for all its existing fields and summaries in the regenerated batch.
