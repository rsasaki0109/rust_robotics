# Planner families and exact saved diagnostic windows

Start from main e9f7dba. All source outcomes and cohort/seed comparison results
are already observed. Commit this protocol before computing new planner-family
or duplicate-window results. This is a retrospective weighting sensitivity study,
not a new validation cohort, estimator correction or independence estimate.

Keep known 269/270/271, previous additional 274/275/276 and latest additional
277/278/279 separate. Reuse the unchanged adapters for all 32,985 simultaneous
references at 10,995 states, 157 source cases and 275 received opportunities.
Fix Recovery/Preview, all event kinds together and all_available, both targets,
all three original scores, all E/P/U labels, all estimator strata and pooled all.
No new simulation, raw replay, fitting, cutoff changes or controller changes.

A planned family is the full case key with only planner removed: cohort, arm,
loss mode, seed and map. Audit all 972 cases / 324 planned families, including
families without any source state and missing planner support. Family IDs and
case IDs are stable sorted mappings to those full keys. Do not merge families
across cohorts, seeds, maps, arms or loss conditions.

Within each family, define exact equality of the complete saved Recovery/Preview
diagnostic window. At each ordered state include time_s, truth_time_s, estimator/
anchor/stratum strings, all three integer score ticks, saved residual a/b weights,
and both targets' saved observed x/y, partial variance, four weighted 2D error
components and E/P/U outside labels. Also include sorted source-event semantic
tuples (event_kind, event_time_s, truth_time_s, scheduled, received,
precision_activation). Include every state, not only switching or outside rows.
Exclude artifact case/state/pose/event IDs and planner names from the window
signature. These arbitrary IDs remain in the separate source mapping.

Canonicalize finite parsed floats with float.hex(), normalizing signed zero,
keep integers/booleans/strings typed, sort object keys and order states by time.
Hash the canonical JSON with SHA-256 and verify canonical payload equality for
hash matches. No tolerance, rounding of vectors, approximate matching, longest
common prefix or selection of a representative planner. Whole windows with
different lengths/support are distinct even if some individual states overlap.
Equality concerns saved diagnostic inputs only, not physical trajectories or
independent noise streams; unequal hashes do not establish independence.

Compare exactly three prespecified parent weights, calculated before mode/seed
conditioning. Let n_c be the full parent case's Recovery/Preview state count,
k_f the number of represented cases in its family, and m_w the number of cases
with its exact complete window in that family:

* case: 1/n_c, the unchanged reference;
* window: 1/(n_c*m_w), one unit per exact saved window;
* family: 1/(n_c*k_f), one unit per represented planner family.

Keep every original row. Missing cases/families get explicit zero support and
no imputed observations. Check that total parent mass equals represented cases,
window classes or families respectively, and carry weights unchanged through
mode and seed conditioning. Report original row/case/seed counts separately
from analysis unit counts and mass. These units are not effective independent
sample sizes. Family weighting changes the question even for unequal windows.

For all three weightings save complete-group components, outside fractions,
full fixed-direction AUC, within/between-seed AUC and comparison-mass shares,
supported seed counts and all nine ordered seed blocks. Reuse the existing
exact rational pair helper: high score=outside, half credit for ties, norm
>.10 m outside with a closed inside disk, absent-class AUC blank. Preserve all
signed energies, both targets and every score/label, including empty strata.
Match every case-reference component/ranking/seed-block value to the preceding
bundle. No score reversal, threshold refit, seed normalization or winning scheme.

Freeze figures to residual_sigma, all cohorts/noises/arms and Filter/Fusion,
both targets and E/P: show all three weightings' full and within-seed AUCs,
NA cells, source counts, parent unit counts and within-seed comparison shares.
Also show represented cases/windows/families and exact-window multiplicities
for every cohort/noise/arm, retaining zero-support planner families in tables.

Save code, meaningful exact-signature/grouping/weight/replication/NA tests,
CSV/JSON/SVG, report and README reproduction commands. Pin protocol/inputs/helpers,
copy unchanged accounting/support/events/cutoffs and historical 550/509 plus
latest 101/94 loss references. Repeat all aggregation/plot artifacts byte for
byte, inspect figures and independently verify signatures, unit weights,
weighted metrics, seed blocks and plotted values without importing analyzers.
Run the Python suite and merge this bounded study under the standing instruction.
Live outcomes remain historical 140->140 and latest 62->60. Reweighting repeated
selected histories supplies neither new independent evidence, confidence
intervals, causal effects nor real-world calibration claims.
