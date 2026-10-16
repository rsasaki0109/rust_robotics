# Fixed cutoffs on the additional seeds under the original audit selection

Start from main 1b531d9. Seeds 277/278/279 and all 324 trajectories were generated
previously; their broad immediate-event mode census and live outcomes have been
observed. Their narrow speed-probe audit states, projections and fixed-cutoff
risks have not yet been computed. This is a prespecified extension of the
original audit selection on these existing additional-seed histories, not a
claim that the whole new dataset remains untouched. Do not simulate new cases,
replace seeds, refit cutoffs, select a favorable score/budget or change controls.

Account for all 972 historical+additional trajectories. Reuse every timeout's
correction/precision opportunities in logged [150,180] seconds. For each event,
include its pre point (offset -1) and immediate through +75 decision intervals,
clipped at 180 s. Select exactly the endpoints of adjacent decisions where at
least one endpoint is on the joint route and active_ready is true, matching
analyze_correction_gate_boundaries.py. Union states within case/time; retain
the set of source events from selected transitions. Do not use error, score,
estimator mode, raw zero loss or future controller outcomes to refine selection.
Record raw-zero loss links only for post-phase joint_lost transitions whose
nominal command reason is speed_cap_zero; mask 16 marks command-only links.

Replay complete timeout histories with the unchanged Replay helper. Check all
logged window/fusion counts and all selected actual/preview/motion poses. Use
preceding-row physical truth. Reconstruct raw, actual-recovery and preview
sample groups, their four error components and shared AR-noise/current-anchor
covariance. Apply the existing project() function for all three simultaneous
pairings. Preserve its 1e-12 variance degeneracy rule and possible negative
weights. Use the original three scores, 1e-8 m ties-to-even ticks and midpoint/
residual error norms >.1 m (the .1 m boundary stays inside).

Before new conclusions, reproduce the historical 6,610 selected states, 19,830
simultaneous pairs, source-event memberships, estimator strata, zero-loss link
counts and every historical score tick/outside label. Compare floating model/
error quantities within existing reconstruction tolerances. Also reproduce the
saved historical pooled/conditional comparison results with the new evaluator.
Historical regressions are mandatory, not optional examples. Retain explicit
empty case/event groups rather than discarding cases without selected states.

Read the pinned 72 pooled and 216 mode-conditional cutoffs trained previously
on 269/270/271, shared across arms; copy their bytes unchanged. Evaluate the
additional seeds only, with no fitting. Preserve all views, scores, q25/q50/q75/
all_available budgets, both target labels, all event-kind unions, both noises,
arms and original estimator strata. Report strata with the same parent case
weights 1/n_case computed within each noise/arm/event-kind/view group before
stratification or retention. Keep raw row coverage and parent-case coverage,
retained outside fractions and RMS, overlap cells and signed conditional-minus-
pooled deltas. Do not renormalize onto retained cases or average subgroup risks.

If recovery raw_fallback occurs in the narrow audit, retain it as an explicit
recovery_fallback_single stratum for Raw/Recovery, with missing conditional
training support. Unknown conditional support makes the whole evaluated group
undefined, except all_available. Report that stratum even if empty. Preview
conditional strata remain defined by the original preview-estimator/anchor
mode rule; retain a_estimator so any new actual-input mode remains visible.

Show complete per-seed/arm/noise mode-support counts, including no-audit cases.
The predetermined figure focus is Recovery/Preview residual_sigma q50, all
event kinds: compare fresh coverage/outside fractions for both targets and both
methods, plus per-seed Filter/Fusion support. Full CSV grids preserve all other
scores/budgets and strata. Do not infer real-world calibration, causality,
independence of repeated states or a justified policy/default change. Distinguish
this fixed-simulator narrow audit from the preceding broad census.

Pin raw/source/helper/protocol bytes. Save collector, meaningful regression
tests, portable results, plots and reproduction instructions. Repeat complete
aggregation and plotting byte for byte; inspect the figures and independently
check the saved metrics. Physics/live outcomes remain inherited from the two
already verified live studies. Run the Python suite, then merge this bounded
extension to main under the user's standing merge instruction.
