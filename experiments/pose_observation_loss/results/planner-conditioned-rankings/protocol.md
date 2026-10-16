# Planner-conditioned ranking protocol

Frozen before calculating planner-specific outcomes, starting from main
51b8e7dbe8ff8e02b3ae8d9099fd73cec2c24973. This bounded retrospective study uses
the existing 32,985 reconstructed pair references, 10,995 Recovery/Preview states,
157 represented cases and 972 planned cases. No new simulations, fitting,
threshold changes, score reversal or estimator/controller changes.

Keep known 269/270/271, previous additional 274/275/276 and latest additional
277/278/279 separate. Retain both noises/arms, all five existing strata (all,
Recovery single, Filter single, Fusion three, Recovery fallback single), both
targets, all three integer-tick scores and observed/modeled/unmodeled E/P/U
labels. Reuse the immutable reconstruction/join checks and whole-window family
audit. A family removes only planner from cohort/variant/mode/seed/scenario/
planner. Preserve the accounting partition alias existing_new -> new and its
original label in the copied case/family audits.

Use only the original case reference weight 1/n_case, where n_case counts every
parent Recovery/Preview state before stratum, planner or family selection.
Carry it unchanged through every subset. Do not renormalize after selection or
choose among the preceding case/window/family estimands. The prior study already
reports those weighting sensitivities; this study isolates planner composition.

For each reporting group and stratum, compare exactly two populations:

* source: all available states in that stratum;
* common_family: retain a family only if every planned planner (A*, JPS, Theta*)
  has at least one available state in that same stratum. For all, require all
  three planners somewhere in the full Recovery/Preview window. Determine
  eligibility separately for each stratum and never use outside labels/scores.

Within each population, save mixed-planner and each of the three individual
planner summaries. Matching fixes family membership, not state times, number of
observations, mode mass or physical trajectories. Original mode fractions remain
in the weights. These are descriptions of selected saved windows, not causal
planner performance comparisons. Preserve all 324 planned families in all five
strata, with planner-specific source counts/mass and missing/eligible indicators;
do not impute observations or suppress empty/one-class cells.

For every population/group/stratum/target/score/label, decompose mixed AUC into
the nine ordered positive-planner/negative-planner blocks. Save counts, inherited
case masses, concordant/tied/discordant pair masses, numerator, denominator,
fixed-direction AUC, denominator share and numerator contribution. Within each
planner block also sum the three same-seed numerators and denominators. Assert
that the nine blocks reconstruct full AUC and the existing within-seed result.
Report full, same-seed, same-planner, and jointly same-seed-and-planner AUCs with
comparison-mass shares, using sums before division, never an average of AUCs.
Absent-class or zero-pair AUC stays blank. Scores rank high=outside, ties receive
half credit; outside remains norm > .10 m with a closed inside disk.

Save signed component metrics, original counts/case masses, represented families
and all three seed support rows for every planner selection. Match every existing
case-reference component/ranking field for source + mixed planners to the prior
planner-window bundle. Copy unchanged accounting, case support, events, pooled
and conditional cutoffs, separate historical/latest loss links and prior full
case/family audits. Pin original inputs, helpers, protocol and output hashes.

Freeze figures to residual_sigma and Filter/Fusion: each cohort's observed-E
full/within-seed AUCs for both populations and all four planner selections; latest
cohort modeled-P equivalent; all cohorts' per-stratum common-family support;
latest correlated/curved Fusion residual-P planner-block matrices for both
populations. Numeric figure exports include both targets, E/P, support, NA and
comparison shares even where a focused panel uses only residual or E/P. Plot
all specified cells, never choose panels after examining outcomes.

Add meaningful tests for stratum-specific matching, planned/missing planners,
inherited weights, exact ordered/tied comparisons, seed/planner intersections,
missing-class AUC and baseline fidelity. Repeat aggregation and plotting byte
for byte, inspect figures, independently reconstruct and verify new selections,
metrics, blocks and plotted values without importing analysis helpers, and run
the complete Python suite. Save code, CSV/JSON/SVG, report and README commands;
merge this completed bounded study under the standing instruction.

These already observed cohorts are not fresh validation. Repeated planners and
states do not supply independent seed streams, effective sample sizes, confidence
intervals or deployment calibration. Live outcomes remain historical 140->140
and latest 62->60. Report sensitivities and absent support without picking a
winning planner, population or score direction.
