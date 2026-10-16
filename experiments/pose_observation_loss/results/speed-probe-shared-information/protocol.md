# Shared observation/anchor information and pairwise position agreement

Start from main bd510d2. Pin speed-probe-error-components' accounting, events,
all 19,830 components, all 26,990 groups and provenance, plus the preceding
19,800 loss-membership rows. Preserve 648 case records, 368 timeouts, 2,583
opportunities, 6,610 audited states, both arms, every cohort/event kind and
the unchanged 550/509 loss populations. Tracked inputs suffice; raw replay,
new trajectories and new geometry are not required.

For every recorded state compare raw/recovery, raw/preview and recovery/preview.
Keep source pose/state/chain IDs and recorded estimators. Truth is the same
preceding physical row for every input. Match all group-weighted component
vectors and all source marginal noise/anchor variances. No outcome filter
selects the state pairs; each pairing has the same 6,610-state denominator.

Reconstruct observation weights from each group's contiguous sample bounds,
sample count and weight. The existing component rows have uniform sample sigma
within each input; assert this. Require one anchor ID per group, disjoint
sample ranges within each input and normalized nonnegative weights. Sum
weights by anchor ID and export the realized anchor vectors and current-anchor
weight. Validate raw/recovery each use the current anchor only. A preview can
use that anchor alone or three distinct epoch anchors.

Per axis, calculate AR noise covariance Cn(A,B) by the actual sample steps,
weights and cohort rho: sum_i,j wa_i*wb_j*sigma_a*sigma_b*rho^|step_i-step_j|.
Use a sorted-step recurrence and test against the full double sum. Split
same-observation covariance from temporal AR covariance. Anchor covariance
Ca(A,B) equals the product of current-anchor weights times that anchor's
variance, known from the raw marginal. Each pair contains raw or recovery,
so no other anchor is shared; assert this intersection rather than guessing
historical anchor variances. Use recorded three-anchor marginal variances.
Reproduce all marginals within 1e-12 m^2 and check Cauchy/PSD constraints for
the full three-input noise-plus-anchor covariance matrix at every state.

Compare the correct partial-model difference variance
Vdiff=VA+VB-2*(Cn+Ca) with the independence assumption VA+VB. Retain separate
noise/anchor contributions, model correlation, shared observations and shared
anchor weights. Clamp only negative roundoff within 1e-12 m^2; reject a more
negative variance. This partial covariance model excludes drift and nonlinear
motion/heading transport and is not a calibrated posterior or error guarantee.

Also compute the observed position difference and the four component-vector
differences. Truth cancels in that difference; verify vector closure. Retain
wrapped heading separation as metadata, without using it in agreement tests.
Freeze position agreement at <=.02, <=.05 and <=.10 m. Truth-position accuracy
uses the prior .10 m closed disk, separately for each input, without heading.
At every threshold partition records exclusively into agree_both_inside,
agree_a_inside_b_outside, agree_a_outside_b_inside, agree_both_outside or
disagree. These descriptive checks are not release conditions or calibrated
failure rates. Keep a separate <=1e-8 m reconstruction-tolerance agreement flag.

Export all inputs, per-anchor weights, all 19,830 pair records and one original
loss link per 550 transitions with three pair IDs and the 509-subset flag.
Keep all source loss members; do not count pair/input views as independent
actions. Summaries use all_states, raw_zero_loss and command_only_zero_loss,
all cohorts/arms/event kinds/pairings/thresholds, and anchor modes all,
shared_single and preview_three. Preserve empty groups and distinct state,
case and opportunity denominators. Event-kind unions may overlap.

Repeat accounting, events, inputs, anchor-weights, pairs, loss-links, summary
and provenance byte for byte. Any figure must show the complete fixed
550-loss population for both arms and all three pairings, including all five
agreement bins at the frozen .05 m threshold, and a separately labeled
comparison of partial-model difference uncertainty with independence. Keep
all numeric source rows, empty event kinds and reproducible plot provenance.

This studies shared information, rather than independent new observation
batches from the older fixed-centre information-floor study. No estimator,
heading, steering, speed margin, timer, first-chord certificate, physical
trajectory or success count changes. Observation/physics validation is
inherited and recorded held goals remain 140->140.
