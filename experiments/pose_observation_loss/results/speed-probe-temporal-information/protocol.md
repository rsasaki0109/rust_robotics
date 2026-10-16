# Temporal support and shared information within agreement spans

Start from main 85d6e88. Pin the shared-information accounting, events, all
19,830 pairs, all input marginals/anchor weights, 550 loss links and provenance;
the original 423 audited chains/6,610 members and provenance; and the recorded
position components/estimator groups. Preserve 648 case records, 368 timeouts,
2,583 opportunities, all three pairings and the fixed 550/509 loss populations.
Tracked artifacts suffice; no raw replay, geometry or controller change.

Use only the explicit existing chain edges, never gaps between selected rows
or between original loss records. For each chain and pairing partition every
decision into maximal constant-category spans at four frozen position agreement
rules: <=1e-8 m reconstruction tolerance, <=.02, <=.05, <=.10 m. Reuse the
prior five categories (agree_both_inside, agree_a_inside_b_outside,
agree_a_outside_b_inside, agree_both_outside, disagree). Each input's accuracy
uses the same closed .10 m truth-position disk, without heading. These are
observational diagnostics, not release conditions. Every rule/pairing retains
the full 6,610-state denominator. Keep singleton spans and all comparison bins.

Within each agreement span, split information blocks whenever either input's
anchor-ID/weight signature changes. Signatures come from recorded anchor
weights, not realized vector equality or agreement labels. At a stable
signature assert each anchor marginal and cross-input anchor covariance stays
constant within 1e-12 m^2. Retain source span IDs, chain indices, first/last
pair/pose/state IDs, time support and source event unions. Boundary labels
distinguish audit_start/audit_end, category_change and signature_change.
Audit boundaries truncate support; singleton elapsed support is zero.
Elapsed support is (last_time-first_time)=(states-1)*.02, without a future step.

For each block, reconstruct all per-input noise coefficients from group weights,
sample counts, uniform sigma and actual sample steps, matching prior noise
marginals. Average those coefficients over block decisions; AR covariance of
the resulting sparse map gives the partial-model noise variance of that
temporal mean. This retains both repeated observations and cross-time AR
correlation. Record observation uses versus unique observation steps.

Stable anchor signatures mean the block's mean has the same anchor random
variable as each constituent pose; its variance does not divide by the number
of decisions. This uses recorded marginal variances, without inventing old
anchor-specific variances. Retain the constant cross-input anchor covariance.
Calculate both inputs' temporal-mean partial variances, their covariance and
the variance of their mean difference. Check PSD/nonnegative constraints.
Compare with sum_t(V_t)/n^2 for independent decisions. For the mean difference,
also distinguish preserving same-time A/B covariance from assuming independence
across both inputs and time. All partial models exclude drift and nonlinear
transport and are not calibrated posterior bounds.

Compute observed mean error vectors and mean pair differences from the pinned
components, using the same decisions; truth cancels in A-B. These are offline
diagnostics using known truth, not new online estimates. Do not change the
source point labels using a mean error or select block lengths from outcomes.
Do not treat repeated agreement samples, blocks or case variants as independent
physical attempts or a full error-episode census.

Join every one of the 550 loss transitions to all 3 pairings/4 rules, retaining
the 509 flag and span/block membership. Never stitch loss-only timestamps.
Report the earlier 168 numerically identical recovery/preview loss rows and
their 102 both-outside rows by unique spans/blocks, recorded cases and source
opportunities, without dropping other rows. Summaries keep every cohort,
arm, event kind, pairing, rule and category, including empty groups. Whole-span
event-kind unions may overlap. Pool model variances with one weight per block,
separately from original state/loss-row counts; no confidence intervals.

Export unchanged accounting/events/chains/members, all agreement spans,
all stable-signature blocks, all 6,600 loss memberships, span summaries,
block summaries and provenance. Repeat all nine CSVs/provenance byte for byte.
Any figure must keep both arms and all three pairings, with the full .05 m
agreement-span partition and a separately labeled temporal covariance view.
Keep singleton/truncated spans, all bins and numerical plot data.

Original observation/physics validation is inherited, source first-chord
decisions and recorded control remain fixed, and held goals remain 140->140.
