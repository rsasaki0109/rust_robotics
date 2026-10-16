# Seed-exclusion checks for pooled and mode-conditional cutoffs

Start from main f2672f3. Pin the preceding membership, accounting/events/loss
links, original pooled thresholds, all-known conditional thresholds,
comparison table and provenance, plus the source risk scores/provenance.
Keep all 19,830 simultaneous pairs (6,610 audited states, 100 cases, 171
received opportunities), all 648 accounting records, 368 timeout histories,
2,583 opportunities and 550/509 loss links. The 20,664 requested lagged
references, including 2,208 missing, stay inherited outside this analysis.
No new rollout or controller change; recorded held goals remain 140->140.

Freeze exactly three folds: omit known seed 269, 270 or 271. Fit only the
remaining two known seeds, holding the omitted seed out together across
noise types, arms, maps, planners, availability modes, times and pairings.
Evaluate (a) that omitted known seed and (b) the same existing new seeds
274/275/276, separately. Existing new data must never enter fitting. Both
cohorts were already observed; these are retrospective seed-block checks,
not untouched holdouts or independent replications of the new cohort.

Reuse the preceding pooled/conditional fitting and reporting algorithms.
Keep all three simultaneous views, both noises/arms, all score directions,
q25/q50/q75/all_available rules, fixed 1e-8 m score ticks and both observed
midpoint/residual norm>.10 m outcomes. Pool both training arms for each
threshold. Give each remaining known case total fitting weight one in its
view/noise group, carrying those weights into conditional modes. Do not
rebalance cases after conditioning. Reporting weights are recomputed within
each evaluation group before stratification/retention, then preserved.

The structural census before this protocol shows all known Filter cases in
seed 271. Preserve the resulting missing conditional fits when 271 is omitted.
Do not infer their cutoffs, reject those evaluation rows, use all-known
thresholds to fill them, or renormalize onto supported modes. Export unsupported
rows/mass and mode names; complete conditional/overlap metrics stay blank.
Fusion-only groups may remain supported. All_available remains defined.
Every represented pooled view/noise evaluation group must have training
data; assert this prerequisite rather than changing or dropping a fold.

Export all 648 cases' roles for every fold, including cases without audited
simultaneous pairs. Check full-case and seed disjointness, exact role coverage,
and that every original known case is held out once. Existing new cases recur
as evaluations in all folds; never add them as new independent cases.
Export per-fold pooled and conditional thresholds, fit counts/mass/seeds/
fingerprints, complete method/overlap summaries and missing support. Keep
all event-kind and structural groups, including empty ones.

Reproduce the all-known fit: all conditional thresholds, all simultaneous
pooled thresholds and every original comparison row must match the pinned
preceding tables. Use all-known comparisons only as a reference for existing
new evaluation. Never evaluate an omitted known seed with that reference.
Export a compact existing-new reference table alongside the fold outputs.

For each existing-new reporting group export the all-known reference and
three-fold ranges of each method's coverage and both retained error fractions.
Show defined-fold counts. Emit min/max/span only when all three values are
defined; incomplete or empty groups keep blank ranges. Report counts of
positive/zero/negative conditional-minus-pooled error differences only among
defined comparisons, with undefined counts explicit. These finite ranges
are sensitivity descriptions, not confidence intervals. Do not average held
seed risks or select a preferred fold, score, quantile or method from outcomes.

Tests cover seed/case isolation, evaluation exclusion from fitting, mode
support loss, unchanged reporting populations, missing/empty ranges, original
reference exclusion for held seeds and sign counts. Independently recalculate
fits and reporting fields and repeat every CSV/provenance byte for byte.
Freeze figures to Recovery/Preview residual-sigma q50, both targets/noises/
arms, all three omitted-known evaluations and existing-new fold evaluations,
plus the existing-new all-known reference. Keep counts and unsupported cells
visible; full tables retain every view/score/budget/event kind/stratum. Repeat
SVG/PNG/numeric series/provenance and inspect final layouts. No calibrated
risk, causal effect, independence or controller release claim.
