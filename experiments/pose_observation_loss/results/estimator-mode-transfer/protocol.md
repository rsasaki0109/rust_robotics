# Estimator composition and fixed-cutoff transfer

Start from main 1970a08. Use the pinned contrast-risk-transfer scores,
thresholds, retention summaries, accounting, events, loss links and provenance,
plus contrast-common-errors simultaneous records/provenance. Never refit a
score or threshold. Both seed cohorts have been examined previously; this
is a retrospective descriptive decomposition, without a new holdout or rollout.

The new decomposition covers all 19,830 simultaneous references and all three
pairings. The 20,664 requested lagged references (18,456 available, 2,208 missing)
remain inherited data, outside this simultaneous estimator stratification.
Preserve 648 accounting records, 368 timeout cases, 2,583 opportunities and
550/509 loss links. Original control and held goals remain 140->140.

Join every simultaneous score to its projection record by source family/ID;
assert full case identity, input IDs, event union and view agreement. Freeze
three structural strata using the source b_estimator and anchor_mode:
recovery_mean/shared_single, motion_filter/shared_single, and
three_epoch/preview_three. Assert their allowed pairings: Raw/Recovery uses
only recovery_mean; the two Preview pairings use only motion_filter or
three_epoch. The first input uses the current anchor; shared_single uses that
anchor and preview_three combines it with two older anchors. These labels
describe source estimators/anchor counts, not randomized treatment groups.
Keep every structural stratum, including empty ones, in each group.

Keep known/new, white/correlated, baseline/curved, all/source-event-kind unions,
and all three simultaneous views. Keep all three existing high-risk scores,
both midpoint/residual norm>.10 m targets, exact 1e-8 m score ticks, and all
q25/q50/q75/all_available rules. Use the previously fitted known-cohort cutoff
for both arms and partitions; include entire tied ticks. Do not select strata,
cutoffs, score direction or outcomes from new stratified results.

Within each original reporting group give available rows weight 1/n_case,
where n_case counts all available references of that full case in the group.
Carry these weights into the strata and through retention. Do not rebalance
cases within a stratum or after retention. Stratum mass is additive; stratum
case counts can overlap. Export raw row/case counts, mass/share, retained
mass, coverage, both retained outside fractions and joint retained-outside
mass fractions. Also report within-stratum AUC for both outcomes, using the
inherited case weights, frozen score direction, exact tick ties and half tie
credit; leave absent-class AUC blank.

For partition P and stratum s let p[P,s] be its fraction of original case
mass, c[P,s] its retained fraction, and j[P,s,T] its joint retained-and-outside
fraction for target T. Overall coverage C[P]=sum_s p[P,s]*c[P,s], joint mass
J[P,T]=sum_s p[P,s]*j[P,s,T], and retained risk R[P,T]=J[P,T]/C[P].

Freeze a composition-first decomposition for coverage and each joint mass:
delta=sum_s (p[new,s]-p[known,s])*v[known,s]
      +sum_s p[new,s]*(v[new,s]-v[known,s]), where v is c or j.
The first term changes the estimator mixture while holding known within-mode
rates; the second changes within-mode rates at the new mixture. Export the
intermediate new-mixture/known-rates quantity and the opposite standardization
sum_s p[known,s]*v[new,s]. Export standardized retained risks as standardized
joint mass divided by standardized coverage, never an average of stratum
risks. This is an algebraic, order-dependent description, not causal attribution.

Require observed support for each used rate. A new-positive stratum absent
from known makes the composition-first decomposition unavailable. A
known-positive stratum absent from new makes the opposite standardization
unavailable. Preserve support flags and missing strata; never impute zero
rates or renormalize onto common support. If either partition is empty, keep
counts and leave comparisons blank. Empty retention has coverage/joint mass
zero and undefined retained risk. All_available should have coverage one and
zero coverage shift wherever both groups exist.

Cross-check every aggregate simultaneous coverage, retained mass/count and
target risk against the pinned prior retention summaries. Use exact rational
case weights/decomposition until final export. Test unequal case lengths,
stratum-crossing cases, ties, support gaps, empty/single-class populations,
ratio standardization and decomposition closure. Independently recalculate
saved outputs and repeat all analysis CSV/provenance and plot outputs byte
for byte. Figures show both noise types, arms, all simultaneous views, all
scores and known/new mixtures; quantitative tables keep all event kinds and
budgets. No confidence intervals, calibrated risk or controller/release changes.
