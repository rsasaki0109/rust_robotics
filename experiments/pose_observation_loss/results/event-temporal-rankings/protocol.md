# Time proximity within saved source events

Frozen from main 61cf12c7fd407b3cf69192ede32202534e7f1600 before computing
time-conditioned outcomes. Extend the saved source-event study without new
simulation, fitting, thresholds, estimator changes or score reversal.

Retain all 10,995 Recovery/Preview states, 157 represented cases, 972 planned
cases, 324 families and the 3,997-event census. Keep the three existing cohorts,
both noises/arms, all five strata, both targets, all three scores and E/P/U.
Use mixed planners, both source/common_family populations and original 1/n_case
weights before stratum/population selection. Common-family eligibility remains
the previous availability-only rule. Do not normalize by event or time bin.

Use the absolute difference between saved state time_s values, within the same
full case key and the same single saved census event_id. Preserve all strict
event joins and raw partition/source-event metadata. This tag is not complete
Fusion sample ancestry or an independent information stream. Event time, truth
time, age, underlying source_event_id and matching timestamps do not define
new event groups. Do not create comparisons between different saved event IDs.

Convert each finite saved state time to nearest integer microseconds, ties to
even, and require the reconstruction error to be <=1e-7 seconds. Reject inputs
outside that tolerance instead of silently coarsening them. Save each original
time, integer time and signed reconstruction error. Freeze three upper bounds:
20,000, 100,000 and 500,000 microseconds (20/100/500 milliseconds). These represent
one nominal simulation step and two longer diagnostic horizons, not tuned
prediction intervals. Include distinct states at equal timestamps if present.

Partition every same-event positive/negative state pair into exactly four bins:
[0,20] ms, (20,100] ms, (100,500] ms and >500 ms. Also sum the first one, two or
three bins for cumulative <=20/100/500 ms metrics. Comparisons are symmetric in
time: the positive state may precede or follow the negative state. A time gap
is not a recorded transition or a continuity assertion; do not interpolate,
stitch chains, invent missing states or treat these AUCs as future forecasts.

For every bin and cumulative scope compute exact rational concordant, tied and
discordant pair masses, numerator, denominator and AUC. High score=outside,
ties half; norm >.10 m outside with the closed inside disk unchanged. Sum masses
before division, never average event AUCs. Report pair counts, supported event
and case counts, and denominator shares of both all same-event comparisons and
full comparisons. Blank denominator means blank share/AUC; zero support remains
explicit. Assert all four bins reproduce every previous same-event pair mass
and retain/check every prior ranking field and copied audit/reference table.

Save a score-independent support row for every nonempty event/stratum group,
target and E/P/U label (3,300 rows), with class counts/masses, first/last saved
times and each bin's pair count and denominator. The preceding event-support
table continues to represent all empty event strata. Save 2,160 expanded
ranking rows, 8,640 time-bin rows and 10,995 time-audit rows. Preserve the full
prior census, case/family/stratum accounting, loss and cutoff references.

Freeze five figures to residual_sigma and Filter/Fusion: each cohort's residual
observed-E within-event and <=500/100/20 ms AUCs for both populations, the latest
modeled-P equivalent, and all cohorts' residual E/P time-bin composition of
same-event comparison mass. Annotate supported counts and denominator shares;
retain both targets and full-denominator shares in numeric exports. Display
NA where classes or nearby opposite-label states are absent. No outcome-based
choice of cases, events, spans, horizons or panels.

Pin inputs, protocol, helpers, scripts and outputs. Test inclusive endpoints,
floating-point boundary noise, out-of-grid/nonfinite times, zero gaps, exact
ties, unequal case weights, event/case isolation, missing classes, cumulative
mass closure and preservation of previous results. Repeat aggregation/plots
byte for byte, inspect all figures and run the full Python suite. Save an
independent NumPy direct-state-pair verifier importing no analyzers; reconstruct
times from original projections using Decimal rounding as a separate method.
Publish reproducible CSV/JSON/SVG, a report and README, then merge this bounded
study under the standing instruction.

These are retrospective comparisons of already observed, dependent states.
Small pair mass and supported-event counts limit interpretation; none of the
counts establish independent trials, confidence intervals or calibration.
Historical live outcomes remain 140->140 and latest 62->60.
