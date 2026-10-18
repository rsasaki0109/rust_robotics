# Frozen radial/transverse projection of the same 19 estimate differences

Freeze from main 1362b968aadf61b4173a28664836b975fac42dd1 before deriving
new normalized projections, reconstructed distances or remainder quantities.
Retain exactly the preceding 19 reset edges, original blocker IDs, 17 initial-
unsafe/two first-chord origins and 185 overlapping-ending multiplicities.
Copy all preceding displacement columns verbatim. No new times, objects,
decisions, trajectories, windows, trials, weights or outcome labels.

Pin preceding sources/proof, all 11 preceding study artifacts, complete
transitive input/helper/Rust chain and preceding verifier. The original 25
contexts, 60 missing future slots, 972 cases, 324 families, 570 timeouts,
3,997 anchors, 137 returns, 3,751 decisions, 1,096 zero probes and 336 cells
remain inherited. No fresh raw extraction or native dataset replay.

Use inherited u=raw-obstacle and v=mean-raw component boxes, original raw and
mean obstacle-distance boxes r and d, preceding displacement-squared box q
and preceding distance-gain box g. This is a spatial estimate comparison at
one recorded step, not elapsed robot motion or a replacement probe policy.
For certified r_lo>0, calculate radial a=(u dot v)/r and signed transverse
b=(u_x*v_y-u_y*v_x)/r. Positive/negative transverse uses the recorded x/y
coordinate convention; assign no physical left/right meaning.

Calculate triples for radial_component, transverse_component, transverse_abs,
projection_norm_squared=a^2+b^2, projection_norm_residual=q-(a^2+b^2),
radial_endpoint=r+a, reconstructed_distance=sqrt((r+a)^2+b^2),
reconstruction_residual=d-reconstructed_distance,
distance_gain_beyond_radial=g-a, rational_denominator=d+(r+a),
rational_remainder=b^2/(d+r+a), and remainder_residual=(g-a)-rational_remainder.
The last division is allowed only if its denominator lower bound is positive.
These identities include finite spatial differences, not a first-order
approximation. Do not infer a path between the two positions.

If r_lo<=0, retain the row, use unresolved_raw_distance, leave all 12 new
quantity triples blank and set new signs unknown. If r is certified positive
but (d+r+a)_lo<=0, retain every other quantity, use
unresolved_remainder_denominator and leave only rational_remainder and
remainder_residual blank. Otherwise use complete. Never divide through a
zero-containing interval, discard a row or resolve a denominator using center
values or numeric comparison tolerance. Reversed/nonfinite inputs fail.

Use inherited boxes without narrowing and conditional elemental padding
P=1e-12. Add/subtract and product/square/sqrt rules match the frozen preceding
audit; squared sums and sqrt lower bounds clip at zero. Division takes all
four endpoint quotients, then +/-P, requiring a nonzero denominator interval.
Absolute value uses zero lower when crossing zero, otherwise minimum endpoint
absolute value; upper is maximum endpoint absolute value; pad +/-P and clip
lower at zero. Do not clip the distance_gain_beyond_radial interval to zero.

Require all available identity residual centers to have magnitude <=1e-10
and their full intervals to contain zero. Preserve all old source states and
direct unsafe chord evidence. Classify radial as outward/inward/unknown,
signed transverse and distance_gain_beyond_radial as positive/negative/unknown
only from strict lower>0 / upper<0 / otherwise. No angle, ranking, cutoff,
significance test, empirical transition or counterfactual recovery claim.

Save projections.csv and projection-summary.csv. The summary has the same
four cases plus all_cases crossed with the three original zero origins plus
all_origins (20 cells). Include physical/multiplicity counts, all three status
counts and all three state counts for each of the three new signs. Include
componentwise min/min and max/max interval extrema of radial_component,
transverse_abs and distance_gain_beyond_radial using only available boxes;
empty extrema stay blank. Only counts add across partitions; extrema are
recomputed, and unresolved rows remain in counts and exports.

Freeze one two-panel figure in original case/step order: radial versus signed
transverse components, and inherited distance gain versus gain beyond radial.
Include all 19 labels, full bound ranges, zero references and two first-chord
markers; show unresolved status explicitly if present. Export every projection
row and all 20 summary cells. Inspect PNG and track SVG/CSV/provenance. Replay
both collector tables/sources and every plot artifact bytewise.

The standalone verifier imports no analyzers/plotters. Independently compute
projections, interval division, identities, signs, summaries and joins to the
original census with 60-digit Decimal from binary64-parsed inherited boxes.
Copied strings/IDs/counts/states compare exactly; new numeric comparisons use
abs 1e-12 and rel 1e-10, never for interval signs or denominator certification.
Count 19 fresh projection-pair evaluations and zero fresh raw/native runs.

Test outward/inward/tangential/mixed/overshoot differences, signed transverse
orientation, zero raw distance and zero-containing denominators, exact and
near-zero remainders, intervals crossing zero, endpoint division/absolute
bounds, nonfinite/reversed boxes, corrupted geometry identities, string
preservation, empty/unresolved summaries, interval extrema and additive
closures. Run all Python tests, save report/README/evidence, merge and push
under standing authority. No controller/dependency changes.
