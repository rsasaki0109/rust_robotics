# Retrospective transfer of contrast/common-error risk diagnostics

Start from main d956c7b. Pin every contrast-common-errors simultaneous and
lagged record, accounting, events, loss links and provenance. Preserve all
648 case records, 368 timeout histories, 2,583 opportunities, 19,830
simultaneous pairs, 20,664 requested lagged pairs and 550/509 loss links.
Use tracked inputs only, without new controller/geometry/physics execution.

Split only by the existing seed cohorts: known-white/known-correlated with
269/270/271 are fitting records; new-white/new-correlated with 274/275/276
are evaluation records. Assert these exact memberships and disjoint seed
sets. The evaluation observations have already appeared in prior studies:
this is retrospective transfer, not untouched held-out validation. Keep
whole recorded cases together and never split their times or event phases.

Use eleven fixed views: all three simultaneous pairings and all four lagged
post phases for each of actual recovery/preview. Keep white/correlated noise
separate. Fit each view/score/noise threshold using the known cohort pooled
over both arms, without consulting new-cohort scores or outcomes. Keep the
same fitted threshold for both arms. Source losses do not select fit rows.

Freeze three high-risk score directions: contrast norm |D|, model midpoint
sigma sqrt(VM), model common-residual sigma sqrt(VR). For lagged pairs, |D|
subtracts known truth motion and is an oracle score; do not describe it as
an online error-change measurement. The sigmas remain partial, uncalibrated
noise-plus-anchor model quantities. No score fitting, inversion, recombining
features or selecting the best score from evaluation outcomes.

Before comparisons map every finite nonnegative score to the nearest 1e-8 m
integer tick using round(score/1e-8), with Python's ties-to-even rule. Preserve
raw score values too. This fixed reconstruction-scale guard prevents tiny
floating differences from being treated as ranking information. Keep tied
scores together in quantiles, retention and AUC calculations.

For each available fitting row assign weight 1/n_case, where n_case counts
available references of the same full case identity within that view/noise
fit group. Every represented case therefore has total fitting weight one.
Take lower weighted quantiles at .25, .50 and .75: first tick with cumulative
weight at least q times total weight, including all equal ticks. At evaluation
retain score_tick<=threshold_tick. Add an all_available reference. If a fit
group is empty, fitted quantile rules remain unavailable rather than borrowing
evaluation data; all_available still retains every available row. Requested
coverage is a training quantile, not a guaranteed retained fraction on either
cohort. Export fit row/case counts, seed list and source-reference fingerprint.

Two frozen diagnostic outcomes are midpoint_norm>.10 m and residual_norm>.10 m.
Closed .10 m is inside. These are labels for abstract observed error modes,
not relabeling of original point accuracy/agreement, geometry or actions.
Report both outcomes, with no empirical choice of threshold or target. Missing
lagged projections have blank scores/targets and stay in requested denominators.

Evaluation groups retain each cohort partition, noise, arm, event-kind union
and view, including empty groups. Within each reporting group assign available
rows weight 1/n_case again. Do not renormalize weights after retention: coverage
is retained mass/total case mass and retained risk is error mass/retained mass.
Keep raw row counts and retained cases alongside these case-balanced measures.
Compute case-weighted AUC with high scores predicting outside errors, half
credit for exact tick ties, and blanks when either outcome class is absent.
Do not flip AUC below .5. Also preserve unweighted AUC for comparison.

Keep all .25/.50/.75 and all_available retention rows. Report requested,
available, missing, retained, error counts, case mass, coverage and retained
error fraction/RMS for both targets. Rank summaries do not depend on a fitted
cutoff. Inputs, phases, source event unions and case variants overlap; case
balancing does not create independence. No confidence intervals, conformal
claims, calibrated risk guarantees or release decisions.

Export unchanged accounting/events/loss links, all scored source references,
all fitted thresholds, complete retention and ranking summaries, and provenance.
Repeat all seven CSVs/provenance byte for byte. Any figure keeps both arms,
noise types, cohort partitions, all eleven views and all three scores, with
numeric event-kind series and missing counts available. Repeat plot CSV/SVG/
PNG/provenance. Original validation is inherited; held goals remain 140->140.
