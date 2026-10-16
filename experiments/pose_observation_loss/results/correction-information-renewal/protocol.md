# Information retained across recorded correction opportunities

Start from main 45ed3ee. Pin correction-centre-transitions accounting, all
2,583 events, all 25,830 input snapshots and provenance. Reuse its exact
phases: pre=-1, immediate=0, after_0p18=9, after_0p5=25, after_1p5=75 steps
relative to each opportunity, DT=.02 s. Keep both inputs (actual recovery and
preview), both arms, all cohorts and every event kind. Keep the original
648-case accounting and 368 timeout histories. This is not a new controller
or a redefinition of the earlier 550/509 loss-selected population.

Replay the pinned raw histories with the existing estimator-window helper,
checking all sample counts and fusion availability at every interval. At all
available source snapshot times, match the recorded poses, preceding physical
truth, estimator, sample counts, component vectors and noise/anchor marginal
variances. Use prior snapshot IDs, event IDs, phases and case identities;
multiple references to one physical input are not independent observations.
Keep unavailable source snapshots as explicit rows with blank model fields.

For every available snapshot export each actual estimator group's weight,
anchor ID, anchor sigma, observation sigma and exact sample-step list. Do not
assume contiguous groups, disjoint windows, equal sample counts or independent
noise across anchors. Reconstruct normalized sample coefficients and aggregate
anchor weights from these groups. Initial anchor 0 has deterministic zero
variance. Check consistent sigma/vector values for each shared anchor ID.

Join pre to each later phase for the same event and input, producing 20,664
requested pairs. Record available versus censored pairs without zero-filling
missing outcomes. Use actual common observation steps and AR covariance for
cross-time noise covariance. Cross-anchor covariance is the sum over shared
anchor IDs of products of normalized weights and anchor sigmas. Check marginal
and joint PSD/nonnegative constraints within 1e-12 m^2. Compute variance of the
error-vector change, VA+VB-2*Cov(A,B), and the independent-input comparison
VA+VB. These are per-axis noise-plus-anchor partial models excluding drift
and nonlinear transport, not calibrated confidence bounds or release criteria.

Distinguish an event's actual anchor replacement from a missing scheduled
correction that clears estimator windows but leaves the anchor unchanged.
Record the anchor ID immediately at the event and whether it differs from
pre. Partition post anchor weight into (1) IDs shared with pre, (2) the event's
new anchor if not already in pre, and (3) all other IDs. Also record the event
anchor's total weight separately, since it can overlap pre on a miss. Older
anchors newly entering a preview are not newly received event anchors.
Count intervening actual anchor replacements in (event,post], retaining every
pair instead of dropping contaminated windows. No inference of continuity
from loss-only rows or merging across source histories.

Retain observed pre/post error vectors and norms, their vector change, actual
pose jump and truth displacement; truth subtraction defines error change.
Keep the closed .10 m truth-position disk and all four pre/post accuracy
categories plus unavailable, without heading. Export observed improvement,
worsening and numerical ties using the prior 1e-8 m tolerance. Neither lower
error nor renewed information establishes a release or a physical outcome.

Summaries keep all cohort/arm/event-kind/post-phase/input/post-estimator
groups including empty and unavailable groups. Pool each model's variances
over matched event/input/phase pairs and take a square root; event phases and
inputs overlap and are not independent attempts. Report denominators before
model values. Keep source event action fields unchanged rather than using
estimate accuracy as an action proxy. Any figure keeps both arms, both inputs,
all four post phases and the complete requested populations, with censored
counts explicit and all event-kind numeric series available.

Export unchanged accounting/events, snapshot information, exact group weights,
all requested transitions, complete summaries, per-history reconstruction
checks and provenance. Repeat all seven CSVs and provenance byte for byte.
Figure data, SVG, PNG and provenance also repeat. Raw paths can relocate via
--raw-root, while fingerprints stay fixed. Original observation/physics
validation is inherited; recorded control and held goals remain 140->140.
