# Same-event seed deletion and seed-only sensitivity

Frozen from main 2b7e1d8bf6d4cf1a48f39c6ac8de1063cbb34d5e before computing
new seed sensitivity outcomes. Follow the case/family deletion study, preserving
10,995 RP states, 157 represented cases, 972 planned cases, 324 planned families
and the 3,997-event census. Keep all three cohorts, both noises/arms, five
strata, two targets, three scores, E/P/U, mixed planners and source/common_family
populations. No new rollouts, fitting, cutoffs, score reversal or controller
changes. Preserve the historical held goals 140->140 and latest 62->60.

For every label-blind selection group include exactly the three planned seeds:
known 269/270/271, new 274/275/276, fresh 277/278/279. Reject unknown, duplicate
or malformed planned seed identifiers. Include absent and pair-free seeds;
empty groups have three explicit zero-record audits and three effect rows per
scope. Do not substitute observed seed lists or fabricated states. Freeze
source/common_family selection and availability-only eligibility before seed
operations. Keep original full-RP-parent 1/n_case weights, with no normalization
or eligibility recomputation after removing or retaining seeds.

Remove every selected case/family belonging to one seed at a time, and
separately retain only that seed's selected states. Keep same-full-case,
same-saved-event comparisons and the existing integer-microsecond all-event,
<=500/100/20 ms scopes. Retain both 1e-8 m ticks and strict saved-score order,
which describes export precision, not exact physical precision. Preserve all
labels, event tags, scores and boundary rules. Event tags are not complete
Fusion ancestry and neither cases, families nor seeds establish independent
physical attempts.

Reconstruct exact rational per-case contributions, add them by seed, and check
that all three seeds sum to the original baseline at every scope. Subtract a
seed for deletion; retain its contributions for seed-only values. Save all
original baseline precision fields unchanged through pinned input references.
Use direct remaining/retained-state recomputation in boundary tests and the
independent verifier. Save label-blind seed records/cases/families/events and
case mass; every seed's removed six tick/native C/T/D masses, comparison shares,
record/case/event availability, retained and remaining pair counts/mass,
supported cases/events, AUC, native AUC and both unequal-score concordances.
Unprefixed effect fields describe deletion; seed_ fields describe seed-only.
Keep original deletion deltas, with blank changes for undefined operands.

Save 360 audit rows, 25,920 seed effect rows and 8,640 scope summaries. Each
summary has three units, represented and pair/unequal-supported seed counts,
maximum removed total and unequal comparison shares, baseline metrics and
deletion min/max, defined/undefined counts, maximum absolute change and exact
rational below/equal/above-half and strict-side-flip counts. For each metric
also save seed-only min/max and defined/undefined counts. A strict flip needs
two defined values strictly on opposite sides of .5. Support loss, a .5
departure and undefined values are not strict flips. Keep absent/pair-free
seeds as deletion no-ops and seed-only missing values. Do not average AUCs.

Freeze five figures to residual_sigma and Filter/Fusion: residual E for each
cohort, latest modeled P, showing baseline plus deletion and seed-only ranges
at all four scopes and both populations. Each range includes its undefined
count out of three. The fifth figure shows every cohort's individual three
seed-only and three leave-one-seed AUCs for residual E/P at <=20 ms, both
populations, all noise/arm/mode combinations; annotate remaining comparison
mass shares and NA. Do not pick a preferred seed or omit empty panels. Numeric
exports contain 768 complete summary rows (both targets, E/P, both modes,
populations and all scopes) and 288 individual-seed effect rows for the fifth
figure. Track five SVGs and numeric exports; reproducible PNGs stay in scratch.

Test hand calculations, multiple cases/families per seed, unequal original
case weights, no-op absent/pair-free seeds, all-empty groups, all ties, missing
classes, support loss, boundary scopes, native-versus-tick ties, exact half and
strict flips, malformed/duplicate/unknown seeds and direct deletion/seed-only
recomputation with fixed eligibility. Repeat aggregation and all plots byte
for byte, inspect all five figures and run the full Python suite. A standalone
verifier importing no analyzers reconstructs original raw states and Decimal
score order; independent NumPy pair masks compute remaining and retained seed
populations, check every field, prior case/family results, numeric plot exports
and hashes. Pin all old outputs unchanged rather than copying them.

These are retrospective sensitivities of already observed seeds, not held-out
prediction, cross-validation, confidence intervals, jackknife standard errors
or independent replications. Save a report, README and verification evidence,
then merge this bounded study under the standing instruction.
