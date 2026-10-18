# Frozen raw-to-mean displacement at the 19 reset edges

Freeze from main 810409fa265762d614261016c85f8cd398365ab6 before deriving
new displacement lengths, distance gains or squared-distance decompositions.
Reuse exactly the preceding 19 distinct resets and their 19 original blocker
witnesses, with all 185 overlapping-ending multiplicities. Retain 17 initial-
unsafe and two first-chord origins; select no new times, objects or outcomes.
The original 25 contexts, 60 missing future slots, 972 cases, 324 families,
570 timeouts, 3,997 anchors, 137 returns, 3,751 decisions, 1,096 zero probes,
336 cells and preselection weights remain inherited and unchanged.

Pin preceding sources/proof, all preceding study artifacts, complete transitive
input/helper/Rust chain and verifier helper. Use the inherited reconstructed
current raw and retained-window mean point centers/rectangles, exact map strings,
current common margin and per-obstacle point-distance/gap witnesses. Join by
exact edge, case, event/source step and global obstacle ID; require one original
witness per selected edge. Preserve every preceding probe field and every
witness field under witness_ in displacements.csv. No new raw extraction,
trajectory, window, full-probe or native dataset replay.

Let u = raw - obstacle and v = mean - raw at the same recorded source step.
Calculate v_x/v_y, u_x/u_y, ||v||, distance gain d_mean-d_raw, gap gain, linear
term 2 u dot v, quadratic term ||v||^2, observed squared-distance change
d_mean^2-d_raw^2, prediction 2 u dot v + ||v||^2, identity residual and
gap-gain minus distance-gain residual. Do not normalize u or divide by a
possibly zero raw distance. This is a spatial comparison of two estimates
at one step, not robot motion during a time interval or a counterfactual policy.

Check center distances against the inherited witnesses using sqrt(dx^2+dy^2).
Check each inherited distance-gap center equals the same current margin.
Center closure residuals must have magnitude <=1e-10; both residual enclosures
must contain zero. Preserve the inherited point states, masks, origin and gaps;
do not replace a curved first-chord test with a point predicate.

Use conditional interval arithmetic with P=1e-12 elemental padding: add/subtract
endpoints with +/-P; product takes all four endpoint products with +/-P;
square uses zero lower when its interval crosses zero, otherwise minimum
endpoint square, upper maximum endpoint square, +/-P and nonnegative lower
clipping. Sum the two squared displacement components, clip lower at zero,
and apply sqrt with +/-P/nonnegative lower clipping. Use the same product
rule for factor two. Source coordinate/distance/gap rectangles remain intact.
Do not narrow any interval using center checks or comparison tolerances.

Classify distance gain as increase/decrease/unknown only if lower>0/upper<0/
otherwise, and the linear term as outward/inward/unknown by the same rule.
Retain a blocked-raw/clear-mean flag and the original raw/mean point-state pair.
No ranking, new cutoff, event significance or inferred continuous transition.

Save displacement-summary.csv for the same four cases plus all_cases crossed
with the three original zero-origin categories plus all_origins (20 cells).
Include physical counts, original multiplicity counts, gain/linear-state counts,
blocked-to-clear counts, and lower/upper enclosures of minimum and maximum gain
and displacement norm (componentwise min/min and max/max over intervals).
Empty extrema stay blank. Only counts add across case/origin partitions;
extrema and sets are recomputed. No selected weights or new trial denominator.

Freeze one two-panel figure of all 19 edges in original case/step order: raw
versus mean inherited current point-gap intervals with zero reference, and
distance-gain versus displacement-norm intervals. Keep complete bound ranges
and all labels, marking the two inherited first-chord rows. Export every
displacement row and all 20 summary cells. Inspect PNG and track SVG/CSV/
provenance. Reproduce both collector tables/sources and all plot files bytewise.

The standalone verifier imports no analyzers/plotters. Use pinned independent
original census readers, but independently join inputs, calculate the spatial
quantities and intervals with 60-digit Decimal, classify and aggregate. Copied
strings/IDs/counts/states compare exactly; new numeric comparisons use abs 1e-12
and rel 1e-10, never to resolve interval signs. Native execution from preceding
geometry audits remains inherited evidence; the full existing Python suite may
run its own native unit fixtures. Count 19 new displacement-pair evaluations,
zero fresh raw reads and zero new native dataset runs.

Test outward, inward, tangential and overshoot displacements, zero raw distance,
zero displacement, sign-unknown enclosures, nonzero/near-zero margins, common-
margin cancellation, square intervals crossing zero, reversed/nonfinite bounds,
wrong witness distances/coordinates/identities, duplicate/missing witnesses,
original string preservation, empty summaries, interval extrema and additive
count closures. Run all Python tests, save report/README/evidence, then merge
and push under standing authority. No controller/dependency/outcome changes.
