# Received/missed correction transitions in the existing recorded trajectories

Start from main 29d4877. Preserve the entire 648-case accounting and all 368
timeouts. Replay those same complete trajectories with the pinned observation
window/fusion helper. Retain both recorded arms and every cohort, mode and
fallback. Unlike earlier selected checkpoints, include every correction or
precision-activation opportunity with logged time in [150,180] seconds, without
selecting on movement, active route, receipt, error improvement or release.

Treat scheduled correction, received correction and precision activation as
distinct flags. Classify opportunities as received, missed, precision_only,
received_and_precision or missed_and_precision. Include missed corrections
even though they do not reset the anchor. Keep non-joint and inactive routes.
Each scheduled timeout track must retain all seven opportunities 150..180.

Use exact step offsets -1,0,9,25,75 at dt=.02: pre, immediate, +.18, +.5,
and +1.5 seconds. Both actual recovery and preview inputs are reconstructed
at every available phase. Decision inputs in logged row t use physical truth
from the preceding row; record both logged/decision time and that truth time.
Do not compare the inputs to the already integrated state in row t.

Keep the pre-event sample even for the final event at 180 s. Export unavailable
future phases explicitly as right-censored rows, never as zero error or failure.
Pair each post phase with that opportunity's pre phase, using matched available
pairs for both RMS denominators. Record estimator transitions, counts, covariance
model, 4 position-error components, xy/yaw error, estimated displacement and
true displacement. Verify delta-estimate minus delta-truth equals delta-error.
Classify 2D error change as better/worse/within 2e-8 m or unavailable. Do not
call an estimated-position jump an error reduction without the truth comparison.

Inspect the actual active release route and requirements using the pinned gate
helper. Record inactive/independent/joint routes at each phase, pending/joint
base usability and all active requirements, positive target command, guard
release and joint resume. Conditions are descriptive; input kinds share these
same actual actions and must not be added together as extra releases.

For each opportunity, record immediate actions separately and observe the
following 75 decision intervals, event_time < t <= event_time+1.5. Track active
ready intervals and maximum consecutive runs for each route, actual joint
resumes and guard releases, first-action delay, positive-target intervals,
physical travel and new intervening scheduled/precision opportunities. A final
or otherwise truncated window remains censored with its actual interval count.
Do not replace censored windows with zero-action complete windows or discard
events with no movement. Overlapping windows, if any, are descriptive repeats.

Check replayed window counts/fusion availability on every selected interval.
At all original 2,120 checkpoints, reproduce the previous position norms for
both inputs; at every new sampled time, match logged actual/preview/motion xy
and yaw within 1e-8. Verify component closure and saved source/raw hashes.
Raw physics/contact/timer validation is inherited; this is observation replay
and recorded-action observation, not a new rollout or new physics validation.

Export full accounting, reconstruction counts, opportunity records, all phase/
input snapshots, paired changes, matched-pair error/component summaries and
opportunity/action summaries. Preserve received/missed/precision categories,
arms/cohorts, input, phase and estimator transition summaries, including empty
or censored-only groups with blank RMS rather than fabricated zero values.
Repeat byte for byte. If plotted include both arms/inputs, all post phases and
every observed opportunity category, with improvement/worsening/tie/censoring
partitioning the full event denominator.

Report corrected centres and actual errors together. Event association in the
outcome-selected, repeated timeout records is not a causal estimate of the
value of receiving a correction. There is no simulated missing-data replacement,
new sensor information, margin/control change or new held-goal outcome. Retain
the previous live result 140->140 without revising it from these local windows.
