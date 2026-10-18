# Frozen endings of the original known-pass segments

Freeze from main 9d4ca904a64d1ebee693aa3bb79e8764deb0f7f7 before extracting
new segment-ending masks, reset amounts or deduplicated counts. Reuse exactly
the preceding 25 selected references, 1,275 planned slots, 1,215 included pairs,
247 unique case/steps and 191 known-pass segments. Preserve the 60 slots
at/after next loss and the 82 before-return historical contexts. Keep the fixed
raw-away four, reset 15, initial-unsafe 23 and first-chord two subgroups.
No new rollout, raw extraction, horizon, controller, outcome or cutoff changes.
Original populations remain 972 cases, 324 families, 570 timeouts, 3,997 anchors,
137 returns, 3,751 decisions, 1,096 zero probes and 336 summary cells.

Pin preceding sources/proof, every study artifact and the full transitive input,
collector/verifier helper and original Rust chain. This is postprocessing of
already verified tables. Raw/window/cap/admission execution is inherited, not
freshly repeated. Check every inherited native context query, input and trace
against all included slots: exact IDs/strings/counts/age/speed, known gate bits,
logged counters/flags and readiness. Unknown center results permit either bit;
retain their prior uncertainty. Preserve explicit native row IDs at each end.

Independently rescan all 25 x 51 slots into maximal same-branch known-pass
segments, breaking at fail/unknown/missing/branch changes or a preceding release.
Match every preceding ready-runs.csv field and order. Check that segment L
and actual counters obey carry + L = last-after; for a release at the final
checkpoint use carry + L = threshold and last-after = 0. No earlier release
is allowed inside a segment. Distinguish L checkpoints, (L-1)/50-s span and
L/50-s evidence. Context-limited segments do not establish complete onsets/ends.

Classify each segment end, in order: recorded_release if the final checkpoint
releases; right_context_limit if no next planned slot; missing_context if the
next slot is missing (keep its exact reason); branch_change if its branch
differs; gate_failure if the same-branch next conjunction is known fail;
unknown_boundary if it is unknown. A same-branch known pass without a preceding
release must have been included in the segment and is an error. A missing or
window boundary never establishes an actual reset or a failed unobserved gate.

For an observed gate_failure require consecutive exact source steps, previous
after = next before > 0, next after = 0, action reset and observed ready false.
Save every simultaneous failed/unknown gate in source order, all seven next
source gate states, nominal sign/zero reason and before/after/action fields.
Classify the full known failure as nominal_only, nominal_with_other or
other_only; do not select a single cause or calculate counterfactual recovery.
Save the lost ticks and seconds, with lost = carried-before + L. Preserve
pre/reference/post by the terminator offset, not by segment start. Unknowns
may show an observed reset but do not receive a certified gate cause/lost amount.

Save run-ends.csv with every preceding run column plus ending details, native
last/next row IDs and counter decomposition. Preserve original reference order.
Save reset-edges.csv deduplicating only observed gate_failure ends by exact
(case_id, next source_step), sorted by case ID and step. Require identical
source inputs, gate states, masks, zero reason, actual counters and native output
for duplicate physical edges. Save multiplicity, ordered event/reference/member
native-row IDs, before/after, full masks and lost ticks/seconds. Distinct source
edges remain descriptive records, not independent experimental trials. Do not
deduplicate context boundaries into alleged physical resets.

Save reference-breaks.csv for the 25 references only, linking their unchanged
original available/branch identity, direction/origin, original action and prior
maximum segment lengths/missing count. Add run/failure-kind/post counts, ending
status counts, sum lost ticks and all/post maxima (blank if no certified reset).
These are historical context metrics, not added original decisions. Keep
anchor-breaks.csv for all 3,997 anchors with original metadata, received/returned
flags, exact preselection census/received/RP weights and RP counts. Nonreturns
have blank new counts/flags; other returned anchors have zeros/false. Add summed
reference counts and flags for any post nominal-only/mixed/other reset.

Save break-summary.csv for all 336 original cells, retaining original group
keys, planned/timeout/held/census/returned case counts and original census,
received, returned and RP event/record/mass denominators. Add reference totals
and the three available anchor flags with raw/case masses and fractions over
original returned anchors. Preserve empty denominator blanks, masses 570/570/157
and RP 10,995. Require additive mode/category closures. No new weighting based
on selection, slots, segments, reset multiplicity or physical edge count.

Freeze one two-panel figure: 25 horizontal stacked bars in original reference
order showing all ending categories (three failure kinds plus five other
statuses), and grouped bars for every observed full failed mask comparing
overlapping window endings with distinct case/step resets. Show complete totals,
label the different denominators and keep all masks in lexicographic order.
Export all run ends, reset edges, 252 original-mode cells and 137 returns.
Track SVG/CSV/provenance and inspect PNG. Reproduce all five collector tables
and sources, and all plot files, byte for byte.

The standalone verifier imports no analyzers/plotters. Use the pinned independent
original census/identity reader, but implement segment traversal, end priority,
counter checks, physical-edge grouping and reference/anchor/summary accounting
independently. Verify all exports, the original canonical identities and weights,
and the inherited query/trace links. New counts/IDs/states/strings and Fraction
values are exact; seconds compare at abs 1e-12/rel 1e-10. Native execution from
the preceding audit remains inherited evidence and is reported separately from
current Python tests. No unnecessary raw or native rerun is required.

Test simultaneous masks; nominal/mixed/other causes; positive reset amounts;
carried evidence and release thresholds; all boundary priority rules; unknowns;
branch changes; missing/loss context; exact source-step adjacency; duplicate
physical edge agreement/multiplicity; inconsistent duplicates; segment coverage;
pre/reference/post sides; original unequal weights, nonreturns, empty shares,
mode/category closures; native row/input/gate/counter mismatches. Run the full
Python suite, save report/README/evidence, then merge/push under standing authority.
