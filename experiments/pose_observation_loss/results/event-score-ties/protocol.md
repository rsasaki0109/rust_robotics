# Saved-score ties within event and time-conditioned rankings

Frozen from main ae4d253268176260b68957fb6f4003e4fb3f68da before calculating
new tie diagnostics or saved-score ranking outcomes. Retain all 10,995 saved
RP states, 157 represented cases, 972 planned cases, 324 families and the full
3,997-event census. Preserve three cohorts, both noises/arms, all five strata,
both targets, three scores and E/P/U. Keep mixed planners, original source and
common_family populations, and 1/n_case weights before population/stratum
selection. No new simulation, fitting, cutoff, estimator or score reversal.

Within the same full case and same single saved census event_id, retain the
fixed integer-microsecond time audit and disjoint [0,20], (20,100], (100,500]
and >500 ms bins. Summaries retain all event times and cumulative <=20/100/500
ms. Reuse strict event joins and raw partition metadata. Time is symmetric;
no adjacency, independence or future prediction is inferred.

The primary score remains the existing integer tick at 1e-8 m resolution, with
high score=outside and half credit for ties. Separately join the original
saved *_score_m strings by source cohort/full case/source ID, check all metadata
and the saved tick, parse finite binary64 values, and require that rounding the
saved value/1e-8 reproduces the tick. Preserve strings and float.hex encodings
for all three scores in one audit row per saved RP state. This is the precision
of an existing diagnostic export, not a new estimator or a physically exact
score. Do not choose a new tolerance or quantization grid after seeing outcomes.

Partition each positive-negative pair into exactly five score-order classes:
tick_concordant, tick_discordant, tick_tied_native_concordant,
tick_tied_native_equal and tick_tied_native_discordant. Native means the strict
ordering of the parsed saved score, with no extra equality tolerance. Assert
tick order is monotone in native order. Native-equal pairs necessarily have
equal ticks; distinct native values sharing a tick are rounding-created ties.

For each time bin and cumulative scope retain original concordant/tied/
discordant masses, numerator, denominator, AUC, pair count, supported events/
cases and full/event denominator shares. Add native concordant/tied/discordant
masses, numerator/AUC and its change, resolved-tie masses, tick/native tie
shares, resolved-tie share, and tick/native unequal-pair counts plus supported
event/case counts. Expose C/(C+D) separately for tick and native scores, blank
when all comparisons tie. This conditional concordance changes the comparison
population and is not a replacement AUC, a correction, a confidence interval or
a guarantee. AUC=.5 from all ties must remain distinguishable from mixed wins
and losses. Keep all original labels: norm >.10 m outside, closed inside disk.

Compute with exact rational original case weights; sum masses before division,
never average event AUCs or renormalize cases after removing ties. Assert each
time bin reproduces its preceding five pair masses and metadata; cumulative
bins reproduce every preceding time-scope field. Check
native_auc-tick_auc=(resolved_concordant-resolved_discordant)/(2*denominator).
Retain all empty groups and undefined AUCs/concordances. Previous CSVs remain
unchanged, referenced through pinned hashes rather than duplicated in this
bundle; independently recheck all previous audit and ranking fields.

Save 10,995 score-audit rows, 8,640 scope rows and 8,640 time-bin rows. Freeze
five residual_sigma figures to Filter/Fusion: three cohorts' residual observed-E
and latest modeled-P tick/native AUCs, tick-unequal concordance, tie shares and
support at all four cumulative scopes, for both populations; then all cohorts'
<=20 ms E/P composition into tick-unequal, native-equal and resolved rounded
ties. Numeric exports retain both targets and all plotted fields. Show NA for
missing classes or all-tie conditional comparisons. Mark nonzero native changes
even if displayed rounding hides them. No outcome-based case/span/panel choices.

Pin source/protocol/helper/output hashes. Test all five order classes, rounding
joins, metadata isolation, nonfinite/invalid scores, all-tie versus mixed .5,
missing classes, unequal original case weights, event/time boundaries, cumulative
closure and conditional support unions. Repeat aggregation/plotting byte for
byte, inspect all figures and run the full Python suite. The independent verifier
imports no analyzers and reconstructs original states/weights, uses Decimal
ordering of original score strings and NumPy direct pair masks, checking that
Decimal and binary64 orders agree for all consumed native scores. Save report,
README and independent evidence, then merge this completed bounded study under
the standing instruction. Retrospective repeated states/events/planners remain
dependent; live outcomes remain historical 140->140 and latest 62->60.
