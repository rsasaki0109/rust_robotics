# Seed comparison support across the three observed cohorts

Start from main f427fc9. The historical component outcomes and the additional-
seed case/seed diagnosis are already observed. In particular the additional
curved-correlated Fusion residual P AUC .792 uses 75% of its comparison mass
across seeds 278/279. This extension quantifies the same issue for all three
observed cohorts; commit this protocol before computing the new decompositions.
This is retrospective comparison support, not untouched validation.

Keep cohorts separate: known 269/270/271, previous additional 274/275/276
(partition new), and latest additional 277/278/279 (partition fresh). Do not
pool training and evaluation cohorts into a single AUC. Reuse the two existing
reconstruction adapters without modification: 19,830 historical references at
6,610 states plus 13,155 latest references at 4,385 states. Preserve full source
joins, saved signed components, target labels and fixed 1e-8 m score ticks.
Source IDs remain scoped to their artifact family; full case keys include cohort.
No new rollout, raw replay, fit, cutoff change or controller change is needed.

Fix Recovery/Preview, all event kinds together and all_available. Keep both
midpoint/residual targets, both noises/arms, all three original scores, E/P/U
outside labels, original estimator strata and pooled all. Include the explicit
recovery_fallback_single stratum as empty where historical tables omit it.
All other source outcomes remain unchanged. E is observed error, P=noise+anchor,
U=drift+transport; norm >.10 m is outside and the closed disk is inside.

Assign exact 1/n_case weights on the full parent cohort/noise/arm/view group
before mode or seed conditioning. Never renormalize seeds or represented modes
to equal mass. Preserve every one of each cohort's three seed values, including
empty, all-inside and all-outside groups. Export row/case/seed/mass support,
E/P/U outside counts/masses and complete-group component metrics.

For each complete group/score/label, decompose the unchanged AUC into all nine
ordered positive-seed/negative-seed blocks. Keep positive/negative counts, cases
and inherited masses, concordant/tied/discordant mass, numerator, denominator,
block AUC, share of full comparison mass and contribution to full AUC. High
score means outside; ties have half credit. Undefined AUCs stay blank, including
blocks with zero comparison mass. Do not reverse or select score directions.

Sum the diagonal block numerators/denominators to obtain AUC from comparisons
within the same seed; sum off-diagonal blocks for comparisons between seeds.
Report both AUCs and their full-denominator mass shares, numbers of represented
seeds and seeds with both classes, and full AUC minus within-seed AUC only when
both exist. Assert exact rational numerator/denominator closure. This within-
seed AUC is comparison-weighted, not the unweighted average of seed AUCs. Its
support excludes one-class seeds explicitly and is a different diagnostic
question, not a bias correction or an improved predictor.

Match all supported complete-group component/ranking values to the two existing
component bundles. The additional cohort's seed block masses must also match
aggregation of the previously saved ordered case-pair blocks via cases.csv;
its diagonal AUCs and class masses must match the saved single-seed rows.
Historical fallback rows have explicit empty support and no invented prior row.

Freeze figures to residual_sigma, both targets, all cohorts/noises/arms and
Filter/Fusion: display unchanged full AUC, within-seed AUC and within-seed
comparison-mass share for E/P, with class-support counts and undefined cells.
Also show all nine Fusion residual seed-pair blocks for E/P in every cohort and
noise/arm condition, with block AUC and full comparison-mass shares. Other
scores, labels and strata remain in complete tables; no winning metric is chosen.

Pin source/protocol/helper bytes; save code, meaningful support/weight/tie/
closure/undefined tests, CSV/JSON/SVG, report and README commands. Copy the full
972-case accounting/support, 3,997 opportunities, both cutoff tables, and
separate unchanged historical 550/509 and latest 101/94 loss-link references.
Repeat aggregation and plotting byte for byte, inspect all figures and use a
separate checker without analyzer imports for reconstructed labels, weighted
metrics, every seed block and plotted value. Run the Python suite, then merge
this bounded study to main under the standing instruction. Three observed seed
values per cohort and repeated selected histories do not supply confidence
intervals, new independent evidence, causal effects or real-world calibration.
Inherited live outcomes remain historical 140->140 and latest 62->60.
