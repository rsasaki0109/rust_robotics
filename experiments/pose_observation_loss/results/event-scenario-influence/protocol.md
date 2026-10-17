# Same-event scenario deletion and seed/scenario decomposition

Frozen from main 8eeb14645be9db36e7b4f1e29e583d3b960a3925 before computing
new scenario outcomes. Extend the seed sensitivity study with the three planned
scenario identifiers dense_50x50, maze_50x50, open_50x50, in this order. Reject
unknown, duplicate, missing or malformed planned scenarios, and unknown observed
seed/scenario identifiers. Each cohort retains its three existing planned seeds:
known 269/270/271, new 274/275/276, fresh 277/278/279.

Keep all 10,995 RP states, 157 represented cases, 972 planned cases, 324 planned
families, 32,985 source-view references and the 3,997-event census. Retain all
cohorts, both noises/arms, five strata, two targets, three scores, E/P/U and
source/common_family populations with mixed planners. Freeze the original
availability-only eligibility and selected states before any subset operation.
No rollouts, fitting, cutoffs, score reversal or estimator/controller changes.
Historical held goals remain 140->140 and latest 62->60.

Remove one scenario's selected cases across all three seeds at a time, and
separately retain only that scenario. Also retain each of the nine planned
seed/scenario cells alone, including cells with no records or no class pairs.
Every one of the 120 label-blind selection groups has exactly three scenario
units and nine seed/scenario audits. Empty groups retain explicit zero audits,
effects and summaries; do not replace planned identifiers with observed lists.
Cases/families stay joined on their full original keys; original full-RP-parent
1/n_case weights never change. Do not renormalize or recompute eligibility.

Only same-full-case/same-saved-event pairs contribute, at all-event and existing
integer-microsecond <=500/100/20 ms gaps. Preserve both original 1e-8 m ticks
and strict saved-score order; native order describes export precision rather
than physically exact scores. Labels, timestamps and event tags remain fixed.
Scenario names denote these saved environments, not independent map samples;
event tags are not complete Fusion ancestry or independent physical streams.

Reconstruct exact Fraction per-case contributions. Require additive closure of
all nine cells and all three scenarios against every preceding baseline field.
Subtract a scenario for deletion and retain its masses for scenario-only values;
never average AUCs across cells. Save 1,080 label-blind seed/scenario audit rows,
25,920 scenario effect rows, 8,640 summaries and 77,760 seed/scenario scope rows.
Audit records, full case/family IDs, case/event counts and original case mass.
Effect rows retain the preceding deletion schema, with scenario_ fields for
scenario-only N/D/native N, nine pair/support counts and four metrics. Save
removed six tick/native C/T/D masses and total/unequal comparison shares. In
seed/scenario rows save availability, original case mass, all six masses,
N/D/native N, nine pair/support counts and all four metrics.

Summaries keep baseline metrics/support, exactly three planned units,
represented and pair/unequal-supported scenario counts, maximum removed shares,
deletion min/max, defined/undefined counts, maximum absolute changes and exact
rational counts below/equal/above .5 and strict side flips. Add scenario-only
min/max and defined/undefined counts for all four metrics. Strict flips require
defined baseline and deletion strictly on opposite sides of .5. Missing
denominators stay blank; loss of support and .5 departures are not strict flips.
Absent/pair-free scenarios are deletion no-ops and have undefined subset AUCs.

Freeze six figures to residual_sigma and Filter/Fusion: residual E for each
cohort and latest P, showing baseline AUC and scenario deletion/scenario-only
ranges at all four scopes and both populations, with undefined counts out of
three. The fifth shows all three individual scenario-only and deletion AUCs
at <=20 ms for residual E/P, all cohorts/noises/arms/modes/populations, with
retained comparison mass shares. The sixth shows all nine seed/scenario-only
AUCs and their shares of original comparison mass in the same focused cells.
Keep all panels and NA entries. Numeric exports have 768 complete summary rows
(both targets), 288 focused scenario effects and 864 focused seed/scenario rows.
Track six SVGs, three numeric CSVs and provenance; PNGs stay in scratch.

Test exact hand calculations, multiple seeds/cases/families per scenario,
different original case weights, scenario deletion across seeds, all nine
planned cells, absent/pair-free/all-empty selections, missing classes, all ties,
native/tick score order, support loss, exact half/strict flips, original stratum
weights, fixed eligibility, exact time boundaries, case/event isolation,
malformed/duplicate/unknown identifiers and nonmutation. Compare every retained
and remaining subset against direct recomputation. Repeat all collector and
plot artifacts byte for byte, inspect all figures and run the full Python suite.

A standalone verifier importing no analyzers reconstructs raw states and Decimal
score order and checks prior case/family and seed outputs too. Independent NumPy
pair masks with exact Fraction weights directly evaluate each remaining scenario,
scenario alone and each seed/scenario cell. Verify every new field, numeric plot
export and input/helper/script/protocol/output/plot hash. Retain preceding files
unchanged through pinned hashes instead of copying them.

These already observed seed and scenario subsets are retrospective sensitivities,
not held-out validation, confidence intervals, jackknife errors or independent
physical replications. Do not select a scenario or seed for deployment. Save the
report, README and verification evidence, then merge this bounded study under
the standing instruction.
