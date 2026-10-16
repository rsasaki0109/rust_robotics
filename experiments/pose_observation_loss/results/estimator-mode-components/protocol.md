# Observed error components within source estimator modes

Start from main 088a57e. Pin the preceding estimator-mode membership, stratum
summaries, accounting/events/loss links/thresholds and provenance; the existing
risk scores, retention/ranking summaries and provenance; simultaneous
contrast/common projections; and the original per-input error components.
Use tracked inputs only. Both known/new seed cohorts have already been
observed; there is no new holdout, rollout, estimator fitting or cutoff fitting.

Analyze all 19,830 simultaneous references (6,610 states, 100 cases linked to
171 received-correction opportunities) with both abstract error targets:
midpoint M and common residual R. Keep the full 648-case accounting, 368
timeout histories, 2,583 opportunities and 550/509 loss links. All 20,664
requested lagged references remain inherited outside this stratification.
Recorded control and held goals remain 140->140.

Join source-family/pair ID, full case identity, input IDs, state, time and event
union. Keep the frozen recovery_single/filter_single/fusion_three strata and
their allowed pairings. For each source input error keep its four recorded
2D vectors: noise, anchor, drift and transport. Apply the existing M weights
(.5,.5) or existing model-derived R weights (a_weight,b_weight) to every
component. Do not refit weights, clip negative weights, or change the handling
of degenerate contrasts. Check component sums against both original target
vectors/norms and every saved residual component.

For each target let P=noise+anchor and U=drift+transport in the recorded frame.
The exact observed identity is |E|^2=|P|^2+|U|^2+2 P dot U. Keep all four
component squared norms and all six signed pairwise twice-dot products too.
Negative cross terms describe cancellation and must not be clipped. Export
the model's two-axis reference energy 2*V_M or 2*V_R alongside observed
energies. This is a partial second-moment reference, not a calibrated bound.
Compare weighted observed/partial-model energy ratios only where the model
reference is positive; leave zero-reference ratios undefined.

Retain known/new, white/correlated, both arms, all/source-event-kind unions,
all three simultaneous views, all three fixed high-risk scores, and all
q25/q50/q75/all_available rules. Include pooled and every structural stratum,
including empty ones. Use the previous parent reporting-group weights
1/n_case, carry them into strata, and keep them through retention. Normalize
energy means only by retained mass; do not give retained cases fresh unit
weight. Keep parent/stratum/retained row and case counts, masses and coverage.

For both M and R keep observed norm>.10 m, and also diagnostic P-norm>.10 m
and U-norm>.10 m, with the closed .10 m disk inside. Tabulate all four
observed/P inside-outside combinations as raw counts and inherited mass,
plus U-outside counts/mass. These are arithmetic component-removal diagnostics
in the saved frame, not predictions available online or counterfactual robot
trajectories. Removing a component can increase norm; do not assume monotonic
error reduction. Preserve original observed target labels exactly.

For all_available rows, report within-mode and pooled AUC for the same three
scores against observed/P/U outside labels, with high-score direction fixed,
1e-8 m ticks, half credit for ties, parent case weights and blank AUC when
one class is absent. Do not reverse low AUC or select scores from outcomes.
Cross-check original observed AUC, retention counts/mass and error fractions
against the pinned preceding tables for every comparable row.

Write source-reconstruction audits, full component summaries, full diagnostic
rankings and provenance, and unchanged accounting/events/loss links/thresholds/
membership. Repeat all CSV/provenance byte for byte. Tests cover vector and
energy closure, negative cross terms and R weights, rotations, unequal case
lengths and stratum-crossing cases, tied selection/ranking, component-removal
reversals, empty selections/classes and zero model energy. Independently
recalculate saved outputs using a separate calculation.

Freeze figures to Recovery/Preview, both targets, both noise types and arms,
known/new, Filter/Fusion, all_available versus residual-sigma q50. Show signed
P/U/cross energies and observed/partial-model totals; keep empty retention
visible. Full tables retain all views/scores/budgets/event kinds. Repeat SVG/
PNG/numeric series/provenance and visually check final layouts. No causal
attribution, independence, confidence interval, risk calibration or release
decision follows from these selected histories.
