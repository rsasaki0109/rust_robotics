# Recorded correction gates and uninterrupted release evidence

Start from main a567fec. Preserve all 648 accounted cases, all 368 timeouts and
all 2,583 opportunities selected by correction-centre-transitions. Keep every
received, missed, precision-only and combined category, both recorded arms,
all cohorts/modes and the original event IDs. Do not select only the 33 resumes,
improved errors, stationary inputs or active joint routes.

Use the same raw files and source hashes. For every event export the pre-event
decision, immediate decision and every available following decision through
event+1.5 s, at exact offsets -1,0,1..75 with dt=.02. Preserve events at 180 s
with pre/immediate rows and no fabricated future intervals. Metadata, decisions
and control timers are recorded inputs; do not replay new trajectories or infer
the gate from the already integrated physical state.

Reconstruct the active route and requirements with the pinned release_inputs /
gate_context helpers. Joint requirements are stationary, fresh, samples,
recovery_geometry, positive_command, stop and terminal. Independent guard gates
use only stationary, stop and terminal; inactive routes have no requirements.
Represent simultaneous failures with an exclusive bitmask in that fixed order.
Never label a missing/inactive requirement as false or assign an arbitrary sole
blocker. The joint base is the conjunction stationary/fresh/samples.

Audit active-route timers from the preceding row: joint evidence accumulates
dt while all seven requirements hold, otherwise resets, with resume at 1.0 s;
independent evidence uses its three requirements and release at .5 s. Compare
both logged timer values and action flags, including the zeroed value on the
release interval and the reset on a newly missed correction. Preserve evidence
carried into the observation window. Treat guard release accompanying a joint
resume as the same recorded interval, not an independent guard-timer release.

Match every available prior phase snapshot for both inputs to the new gate
records. Recompute the previous ActionWindow counts, first delays, ready/max
runs, movement and intervening events exactly. Audit timers once per distinct
sampled decision; export per-event observations even where windows overlap.
Inherited full observation reconstruction and physics validation are not new
checks in this study.

For each event summarize only offsets 1..75 as the post window. Export the
maximal consecutive spans with the same route/failure mask and action flags;
separate a lost requirement, a route change, a release, a horizon and a recorded
end. Ready spans that touch a window boundary are not assumed to have started
or failed there. Record every requirement's true intervals, joint base/full
intervals, first full readiness, uninterrupted ready runs and their endings.
Also record whether each individual joint requirement was seen true, even
if they were never simultaneously true.

Partition complete windows by the event's initial route. Independent/inactive
starts retain their own categories regardless of later transitions. For joint
starts use, in order: actual resume, no joint intervals, no usable base, usable
base but never all ready, some full readiness with maximum run <50 intervals,
or >=50 intervals without a resume. Right-censored windows remain a separate
category. Empty categories stay explicit. These are observations within this
fixed horizon, not permanent failure diagnoses or policy counterfactuals.

Export full accounting, event summaries, all sampled intervals, maximal spans,
partition summaries, exclusive failure-pattern summaries and per-requirement
summaries. Compare failures both over all post intervals and conditional on the
joint base. Marginal failures overlap and must not be summed as a partition.
Keep event/case/interval denominators distinct. Repeat all CSVs and provenance
byte for byte; any plot must retain all opportunities and censoring, both arms
and every observed event category.

No requirement, margin, timer, input position or controller is changed. These
repeated windows are selected on eventual timeout and include intervening
observations. Do not estimate causal correction benefits, restored holds or
independent trial error rates. Keep the recorded held-goal result 140->140.
