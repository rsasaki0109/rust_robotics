# Frozen point-gap decomposition at the same 19 reset edges

Freeze from main f12dee7faea7a8a35d2262ab702be03db9c9434d before deriving
new radial-gap values or point-gap category counts. Retain the same 19 reset
edges, original blocker IDs, 17 initial-unsafe/two first-chord origins and all
185 overlapping-ending multiplicities. Copy all 625 preceding projection
columns verbatim. No new times, objects, decisions, trajectories, windows,
trials, weights, outcome labels or trial denominators.

Pin preceding sources/proof, all 11 preceding artifacts and complete transitive
input/helper/Rust provenance. Original 25 contexts, 60 missing future slots,
972 cases, 324 families, 570 timeouts, 3,997 anchors, 137 returns, 3,751 decisions,
1,096 zero probes and 336 cells remain inherited. No new raw extraction,
geometry rollout or native dataset replay.

Use the same raw gap h_raw=r-m, mean gap h_mean=d-m, current common margin m,
radial component a, gain beyond radial e, radial endpoint r+a and reconstructed
mean distance d from the preceding table. Form radial_gap=h_raw+a and
rebuilt_mean_gap=radial_gap+e. These are terms of a same-step spatial identity,
not temporal motion, a controller change or a simulated path.

Calculate nine triples: radial_gap, rebuilt_mean_gap,
gap_identity_residual=h_mean-rebuilt_mean_gap,
radial_gap_from_endpoint=(r+a)-m,
radial_gap_residual=radial_gap-radial_gap_from_endpoint,
mean_gap_from_distance=d-m,
distance_gap_residual=h_mean-mean_gap_from_distance,
remainder_gap=h_mean-radial_gap,
remainder_gap_residual=remainder_gap-e.
Validate source point states against their gap intervals and preserve the exact
common-margin string. Every available residual center must have magnitude
<=1e-10 and its full interval must contain zero.

Use inherited enclosures intact, with the preceding add/subtract endpoint
rules and P=1e-12 elemental padding. Do not clip a gap or use comparison
tolerance to narrow intervals. Clear requires lower>0, blocked requires
upper<=0, otherwise unknown; exact zero gap is blocked.

For preceding projection_status=complete, calculate all triples and classify
radial_gap and rebuilt_mean_gap. Otherwise retain the entire row, leave all
nine new triples blank, set both new states unknown and use margin_status
unresolved_projection. Reject unrecognized status, nonfinite/reversed bounds
and mismatched source point states/margin strings. Complete rows use
margin_status complete.

Freeze six exhaustive point_gap_category values, with unresolved status first:
unresolved_projection; already_clear when original raw and mean are clear;
blocked_to_clear_radial_clear / blocked_to_clear_radial_blocked /
blocked_to_clear_radial_unknown when original raw is blocked and mean clear,
using the new radial-gap state; other_point_states for all other original
state pairs. These are point-identity categories, not policy recoveries or
evidence that a curved chord/release test would pass. Preserve both original
first-chord failure records and their separate direct unsafe evidence.

Save margins.csv and margin-summary.csv. Keep the same four cases plus
all_cases crossed with three original zero origins plus all_origins (20 cells).
Include physical/multiplicity counts, two status counts, three state counts
for each new point gap and six category counts (16 additive count fields).
Use min/min and max/max interval extrema for radial_gap, rebuilt_mean_gap and
remainder_gap over available boxes; empty extrema stay blank. Only counts
add across partitions; all extrema are recomputed and unresolved rows stay
in counts and exports. No selected weights or new rate.

Freeze one two-panel figure in original case/step order: original raw gap,
new radial gap and original mean gap; then inherited radial component and
gain-beyond-radial contribution. Include all 19 labels, full bound ranges,
zero references and both first-chord markers. Show unresolved status if
present. Export every margin row and all 20 summary cells. Inspect PNG;
track SVG/CSV/provenance. Replay both collector tables/sources and all plot
artifacts bytewise.

The standalone verifier imports no analyzers/plotters. Independently compute
new interval arithmetic, point states, categories, summaries and original
census joins with 60-digit Decimal from binary64-parsed inherited boxes.
Copied strings, IDs, categories and counts compare exactly. New numeric
comparisons use abs 1e-12 / rel 1e-10, never for interval classification.
Count 19 fresh margin-pair evaluations, zero fresh raw reads and zero fresh
native dataset runs; preceding raw/native execution stays inherited.

Test radial-clear, radial-blocked and radial-unknown blocked-to-clear pairs,
already-clear and other original point states, exact/near-zero boundaries,
zero/near-zero common margins, residual zero containment, invalid geometry/
states/margins/statuses, incomplete projections, source-string preservation,
duplicate edges, empty/unresolved summaries, interval extrema and additive
closures. Run all Python tests, save report/README/evidence, then merge/push
under standing authority. No controller or dependency changes.
