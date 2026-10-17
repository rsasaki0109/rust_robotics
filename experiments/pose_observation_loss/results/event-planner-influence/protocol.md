# Same-event planner deletion and seed/scenario/planner subsets

Frozen from main 64f3f3197f1c44fdb518af4a9fea64c5173bcb30 before calculating
new planner outcomes. Extend the scenario study with exactly A*, JPS, Theta*,
in this order. Reject missing, duplicate, malformed or unknown planned planner
identifiers and unknown observed planner/seed/scenario identifiers. Preserve
the three planned seeds per cohort and dense_50x50 / maze_50x50 / open_50x50.

Retain all 10,995 RP states, 157 represented cases, 972 planned cases, 324 planned
families, 32,985 source-view references and 3,997 census events through pinned
preceding inputs. Keep all three cohorts, both noises/arms, five strata, two
targets, three scores, E/P/U and source/common_family populations. Original
availability-only eligibility and selected states are fixed before planner
operations. No simulation, fitting, cutoff, score reversal or controller change.
Historical held goals remain 140->140 and latest 62->60.

Delete one planner's selected cases across all seeds/scenarios, or retain only
that planner, once each. Each of the 120 selection groups has exactly three
planner units and 27 planned seed/scenario/planner audits, including empty cells.
Do not substitute observed identifiers or fabricate observations. Full case keys
remain distinct, including their original mode and planner. Source/common-family
eligibility is never recomputed after deleting a planner: the surviving two
planners keep their already selected cases. Every state retains original full-
RP-parent 1/n_case weight, before stratum/population/planner selection.

Comparisons stay within the same full case and single saved event, at all-event
and the existing integer-microsecond <=500/100/20 ms scopes. Keep both original
1e-8 m ticks and strict saved-score order. Native order describes export precision,
not physically exact scores. Labels, event tags and timestamps remain fixed.
No cross-planner pair is introduced. Holding seed/scenario names fixed does not
align trajectories, state times, class availability or physical attempts; these
are descriptive subsets, not a causal comparison of planner interventions.

Use exact Fraction per-case contributions to reproduce every previous precision
baseline field, sum by planner, and subtract a planner for deletion. Never average
planner AUCs. Save 3,240 label-blind leaf audits, 25,920 planner effect rows and
8,640 summaries. Effects retain original removed availability, six tick/native
C/T/D masses, pair and unequal shares/counts, remaining availability/support,
N/D/native N and all four metrics/deltas. Prefix planner_ describes planner-only
N/D/native N, nine pair/support counts and the four metrics; unprefixed fields
describe deletion.

Save all 27 seed/scenario/planner contributions for the predeclared residual_sigma,
Filter/Fusion, E/P subset, both targets, all scopes/cohorts/noises/arms/populations:
20,736 rows, including absent and pair-free cells. Each contains original
availability/case mass, six masses, N/D/native N, nine support counts and four
metrics. Their planner sums must reproduce all 6,912 corresponding preceding
seed/scenario rows exactly. Other score/stratum/label case contributions remain
accessible through pinned preceding case/family outputs; all planner aggregate
effects/summaries still cover every original score, stratum and label.

Every summary has three planned units, represented and pair/unequal-supported
planner counts, baseline values, maximum removed shares, deletion min/max,
defined/undefined counts, maximum absolute change and exact below/equal/above-
half and strict-flip counts. Add planner-only min/max and defined/undefined
counts for all four metrics. Undefined denominators stay blank; removing support
is not a zero or a strict flip. Strict flips require two defined exact rational
values strictly on opposite sides of .5. Absent/pair-free planners are deletion
no-ops with undefined planner-only AUCs. Do not renormalize any weights.

Freeze six residual_sigma figures: residual E ranges for each cohort and latest
P, Filter/Fusion, all four scopes and both populations; all cohorts' individual
planner-only/deletion residual E/P AUCs and retained comparison mass shares at
<=20 ms; then latest cohort's 27 seed/scenario/planner AUCs/shares at <=20 ms
in all four noise/arm panels, both modes/labels/populations. Preserve every panel
and NA cell. Numeric exports retain 768 complete summary rows (both targets),
288 focused planner effects and 2,592 focused leaf rows across all cohorts.
Track six SVGs, three CSVs and provenance; reproducible PNGs stay in scratch.

Test hand-calculated unequal original weights, multi-seed/scenario planner
groups, deletion of one planner with other common-family planners preserved,
all 27 cells, absent/pair-free/all-empty selections, missing classes, all ties,
native/tick precision, support loss, exact half/strict flips, original stratum
weights, time boundaries, case/event isolation, bad identifiers and nonmutation.
Directly recompute every remaining/retained planner and leaf subset in tests.
Repeat collector and plot artifacts byte for byte, inspect all six figures and
run the full Python suite.

A standalone verifier importing no analyzers reconstructs original states and
Decimal score order, rechecks preceding case/family, seed and scenario results,
and directly masks NumPy state pairs for every remaining and retained planner
and focused leaf. Compute exact Fraction counts/weights independently of the
collector's subtraction. Check every new field, preceding leaf sums, all numeric
plot rows and input/helper/script/protocol/output/plot hashes. Pin old files in
place without copying them.

These already observed subsets are retrospective sensitivities, not confidence
intervals, jackknife errors, independent physical replications or held-out
predictive validation. Common-family availability does not imply equal saved
windows. Do not select a winning planner or deploy changes from these subsets.
Save the report, README and evidence, then merge this bounded study under the
standing instruction.
