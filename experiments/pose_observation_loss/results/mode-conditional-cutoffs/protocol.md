# Known-cohort cutoffs conditional on source estimator mode

Start from main 46d902a. Pin estimator-mode-components accounting, events,
loss links, source membership, original thresholds, component summaries and
provenance, plus the preceding risk scores/provenance. Use every one of the
19,830 simultaneous pairs, all three views and the existing Recovery/Filter/
Fusion structural strata. Preserve 648 accounting records, 368 timeout
histories, 2,583 opportunities and 550/509 loss links. The 20,664 requested
lagged references, including 2,208 missing, remain inherited outside this
simultaneous comparison. No new rollout or control change; held goals 140->140.

Known seeds 269/270/271 alone fit thresholds; existing new seeds 274/275/276
evaluate the frozen rules. Both cohorts have already been examined, so this
is retrospective, not untouched validation. No outcome labels enter fitting.
Keep white/correlated separate, all three fixed high-risk scores, original
1e-8 m ticks, both midpoint/residual norm>.10 m targets, all event-kind unions,
both arms and q25/q50/q75/all_available rules. Never reverse a score direction.

The pooled comparator is the unchanged preceding threshold table, fitted by
view/noise/score on both known arms. Fit a conditional cutoff additionally
within each fixed estimator stratum. Both arms share each conditional cutoff.
Within the full known view/noise fitting population first assign each row
weight 1/n_case using full case identity and all its available references.
Carry these parent weights into each stratum. Do not recompute unit case
weights after conditioning: a case crossing modes keeps its original mass
split. Take the first tick whose exact rational cumulative stratum mass is
at least q times its total mass; retain score_tick<=cutoff and keep every tie.

Include all three structural strata for every view, including inapplicable
ones. Empty conditional training gets no_training_data for quantile rules;
all_available still selects all rows. Export parent/stratum training rows,
cases, exact-mass-derived totals, seed sets, parent/stratum source fingerprints,
cutoffs and mass below/at/retained by the cutoff. Requested fitting quantiles
are not achieved coverage guarantees; show tie overshoot explicitly.

At reporting time retain each partition/noise/arm/event-kind/view group,
including pooled and all structural strata. Compute parent weights 1/n_case
before stratification or retention. Compare both methods with these same
weights. Keep pre-selection counts/mass, retained rows/cases/mass, row and
case coverage, both outside counts/mass/fractions and observed RMS. Fractions
divide by retained mass; never rebalance selected cases to unit weight.

Partition each reporting population into both retained, pooled only,
conditional only and neither. Save row counts/mass and both outside counts/
mass/fractions for every overlap cell. Export signed conditional-minus-pooled
coverage and retained risk differences where defined. Overlap counts/mass
must close, and conditional-only minus pooled-only retained/error mass must
reproduce each method's mass changes. These are diagnostic record subsets,
not executed robot actions.

If any represented mode lacks a quantile fit, export unsupported rows/mass
and mode names. Leave complete conditional metrics and overlap comparisons
undefined instead of dropping unsupported rows or treating them as rejected.
Empty reporting populations keep zero counts/mass and undefined fractions.
All_available must agree between methods. In single-mode Raw/Recovery, the
conditional rule must reproduce the pooled cutoff and selection.

Cross-check every pooled target summary against the pinned preceding
component summaries. Test training-only fitting, label independence, shared
arm cutoffs, parent weights across modes, exact quantiles/ties, empty training,
unsupported evaluation modes, preserved reporting weights, ratio metrics and
overlap mass/error identities. Repeat all CSV/provenance byte for byte and
independently recalculate fitted cutoffs and saved reporting fields.

Figures keep q50 with both partitions, noises, arms, all three views/scores,
and both targets. Show each method's case coverage and retained outside
fraction, including missing/empty cells; all budgets/event kinds stay in
numeric tables. Repeat SVG/PNG/series/provenance and inspect final layouts.
Do not select a winning method from these outcomes or infer calibrated error
risk, confidence intervals, causal effect, independence or a release policy.
