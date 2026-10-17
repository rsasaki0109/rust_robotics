# Delete-one case and planner-family sensitivity

Frozen from main ed41fa20d10f93a75384a4077f021fb290fb328e before calculating
delete-one outcomes. Extend the saved-score tie study at all-event and fixed
<=500/100/20 ms scopes, keeping both original tick and strict saved-score order.
Preserve all 10,995 RP states, 157 represented cases, 972 planned cases, 324
families and the full 3,997-event census. Retain three cohorts, both noises/arms,
five strata, both targets, three scores, E/P/U and both populations. No new
simulation, fitting, cutoffs, score reversal or estimator/control change.

Freeze the source/common_family selected states before deletion, using the
previous availability-only eligibility. Each eligible deletion unit has at least
one selected RP state, regardless of labels or pair support. A case is its full
case key; a family removes only the planner field from that key. Deleting a
family removes all selected planner cases in it. Include every such case and
family, including single-class or otherwise pair-free units. Empty selection
groups retain explicit summaries but have no fabricated deletion rows. Planned
zero-record cases/families remain in their pinned preceding audits.

Delete exactly one unit at a time, exhaustively. Common-family eligibility is
not recomputed after deletion; removing one planner case does not additionally
remove the surviving family. Remaining states keep original 1/n_case weights
from the full RP parent history, before population/stratum selection. Do not
renormalize cases, events or families. Time scopes, integer-microsecond boundary
rules, labels and score orders remain fixed. Only same-full-case/same-saved-
event comparisons contribute, so their weighted masses add across cases and
families. A saved event tag is not complete Fusion ancestry or an independent
stream; a planner family is not an independent physical attempt.

Reconstruct original states/scores and exact rational per-case contributions,
verify that they sum to every preceding precision-scope field, and subtract the
removed case/family contributions. Check subtraction against direct remaining-
state recomputation in meaningful unit tests. Save every deletion's removed
records/case mass/cases/events, tick/native concordant/tied/discordant masses,
pair counts and shares of original total/unequal comparison mass. Retain remaining
records/cases/events, supported events/cases, pair mass/count, original and native
AUCs, tick/native unequal-score concordance C/(C+D), and changes from baseline.
Undefined denominators/changes stay blank. Removing support is not a zero AUC
or a sign flip. Pair-free deletions are retained even when metrics do not change.

For every preceding scope cell and deletion kind save baseline metrics/support,
unit counts and pair/unequal-supported unit counts, maximum removed comparison
shares, and each metric's defined/undefined deletion counts, min/max, maximum
absolute change and counts below/equal/above .5. A strict side flip requires
baseline and deletion both defined, with one strictly below and one strictly
above .5. Baseline .5 departures are shown by side counts, not called flips.
Use exact rational comparisons to .5. Do not average deletion AUCs or present
ranges as confidence intervals, jackknife standard errors or evidence of
independent sampling. No selection of a preferred deletion, score or direction.

Save a label-blind selection/unit audit, all delete-one rows and all 17,280
scope/deletion-kind summaries. Retain every prior result/reference CSV unchanged
through pinned hashes rather than duplicating them. All original planned case,
family, event, loss and cutoff accounting remains accessible in those inputs.

Freeze five figures to residual_sigma and Filter/Fusion: each cohort's residual
E baseline AUC and case/family deletion ranges/support loss at all four scopes,
for both populations; latest modeled-P equivalent; then all cohorts' <=20 ms
E/P maximum removed total and unequal-pair shares for both unit kinds. Annotate
NA and unit counts. Numeric exports retain both targets, all four metrics and
complete summary fields. Keep every predeclared panel, including empty ones.

Test hand-calculated deletions, different original case weights, whole-family
removal, fixed eligibility, same-case/event isolation, pair-free/all-tie/missing-
class units, support loss, exact half/strict flips and direct recomputation.
Repeat aggregation and plots byte for byte, inspect every figure and run the
full Python suite. A standalone verifier importing no analyzers reconstructs
raw states and Decimal score order, uses independent NumPy pair masks on each
remaining unit selection, rechecks all baseline and new fields plus hashes,
and checks exported numeric plot values. Save the report/README/evidence and
merge this bounded study under the standing instruction. These are dependent,
retrospective deletion sensitivities; historical held goals remain 140->140
and latest 62->60, with no new live outcomes.
