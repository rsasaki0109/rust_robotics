# Frozen comparison of raw and recovery geometry

Freeze from main 19afeccaef19bb2e590eeac130d42ffb0c059804 before reading new
recovery-pose geometry outcomes. Audit all 1,096 zero speed-probe references
from the preceding study: 1,035 initial-test failures and 61 directly bounded
first-chord failures, including available and next-loss joint/guard branches.
Retain all 3,751 decision references, 3,997 anchors, 137 returns, 972 cases,
324 families, 570 timeouts and the original 336 summary cells. The preceding
25 nominal-only conjunction failures and 15 nominal-only timer resets are
predeclared subgroups, not a new selection. Keep the two source failure stages.

Pin the preceding sources/proof, complete transitive input/helper/code/raw/map
chain, protocol and freeze, and the older admission-decisions.csv in
freeze-pins.json. Join admission recovery_x/y/yaw, recovery_clearance,
recovery_sample_count and gate_clearance by unique (event_id,time_s), checking
all overlapping fields except the explicitly refined gate_nominal,
nominal_gap_lo/hi, certainty, failed_gates and unknown_gates. Earlier labels,
timers, commands, Fusion metadata and initial/first-chord enclosures are
inherited evidence and remain unchanged. No policy/parameter/rollout, new RP
selection, contact label or causal benefit is introduced.

For each zero reference retain ALL definite original obstacle witnesses:
definite_obstacle_ids for initial failures, otherwise
first_definite_obstacle_ids. Preserve map row IDs and duplicates. Fail if
either failure stage lacks a definite obstacle witness or has a definite wall
witness, since that would contradict this fixed obstacle comparison scope.
Compare these same points with (1) the recovery start and (2) the original
straight recovery preview, length binary64(0.6), using logged recovery yaw.
Also recompute the minimum recovery-preview distance over the complete map.
Do not use a nearest obstacle selected after seeing recovery results.

Verify in pinned source that the recovery and speed margins have the same
expression, the speed probe receives raw sensed pose, the recovery pose uses
the stationary sample mean when nonempty, and the fallback uses sensed pose.
Use the already logged speed_margin as the common margin; do not infer drift
arguments from incomplete provenance. The recovery scalar checks obstacles,
while its boundary predicate is separate and stays inherited here. Reuse the
original point_segment_distance projection/clamp rule and Point2D distance.

Each nine-decimal recovery coordinate/yaw/margin has half-unit H=5e-10. Require
finite magnitudes <=100 and margin in (0,10). Bound the recovery endpoint with
sin/cos global Lipschitz ranges and an elemental allowance P=1e-12 on each
arithmetic/trig operation. Bound segment distances by the rounded-center
segment and the maximum endpoint-rectangle displacement (Hausdorff radius),
plus a segment-comparison allowance S=1e-9. Use the same start radius for
point distances. These are conditional numerical assumptions, not a formal
proof for all platform libm implementations. Classify gap >0 as clear, upper
gap <=0 as blocked, otherwise unknown. Minima over original blockers and over
all map records retain certain/ambiguous witness IDs. Check the independent
all-map distance enclosure intersects the logged recovery_clearance rounding
interval and never contradicts an inherited certain clearance gate.

Report signed recovery-minus-raw start displacement and its norm enclosure
from the preceding raw x/y rectangles and recovery x/y rounding rectangles.
Keep the yaw and preview differences explicit: first-chord failures also
involve a different curved segment/length. A point displacement is descriptive;
do not call it an isolated causal explanation or a proposed controller fix.
Do not reinterpret estimated margins as true collision distances.

Save recovery-geometry-decisions.csv for all 3,751 references, retaining all
preceding fields. Add an audit flag and blank new geometry on positive probes.
For audited rows save recovery inputs, blocker IDs, displacement enclosures,
recovery endpoint/radius, same-blocker start/segment gap enclosures and states,
all-map clearance/gap enclosures and state, and certain/ambiguous recovery
blocker IDs. Save anchor-recovery-geometry.csv for all 3,997 anchors and
recovery-geometry-summary.csv for all 336 cells. Add phase x branch counts of
audited, same-blocker clear/blocked/unknown, all-map clear/blocked/unknown,
prior-unknown clear and nominal-only-reset clear references. Add available
same-blocker clear/blocked/unknown anchor flags and original exact case masses
and shares. Nonreturns have blank new fields. Reconstruct census/received/RP
weights before selection, preserving 10,995 RP records and masses 570/570/157.
Check additive mode/category closures; empty denominators remain blank.

Freeze one 2x3 figure of available same-blocker recovery segment states by
original partition and joint/guard branch, with denominators and empty cells.
Export all 252 original-mode summary cells and all 137 returned anchors.
Track SVG/CSV/provenance, keep PNG scratch, inspect and reproduce all analysis
and plot artifacts byte for byte.

Both readers freshly hash/stream all 108 raw files, validate complete unique
case grids and noisy initial branches, and extract all zero points and their
predecessors. Check recovery inputs against the raw log, raw x/y input fields
and the inherited sensed rectangles. The standalone verifier imports no
analyzers/plotters: it may reuse pinned verify_first_chord.py for its independent
raw reader, map reader, Decimal Taylor trig and projection distance, and
verify_epoch_retention.py for original identities/hash/weights. Independently
rebuild the new enclosures, summaries and every plot field using Decimal with
60-digit precision. Compare new numeric fields at abs 1e-9, rel 1e-10; require
exact states/IDs/flags/counts and inherited strings. Never narrow enclosures
using comparison tolerance.

Compile the unchanged recovery preview/clearance source fragment, original
Point2D distance and point_segment_distance with stdlib Rust. Trace all 1,096
rounded recovery poses against all original obstacles. Check endpoint and
clearance enclosure membership. This cross-check does not reconstruct original
unrounded means or run a closed-loop trajectory. Save small native inputs and
trace with hashes. Run native tests for straight/rotated previews, segment
interior/endpoints, duplicate points and inclusive contact. Test source routing,
joins, stages, duplicate/simultaneous witnesses, rounding ambiguity, full-map
versus selected blockers, displacement rectangles, raw fallback, unaudited
blanks, branch/phase accounting, unequal weights, censoring and closures.
Run the full Python suite, record README/report/evidence, then merge/push this
bounded audit under the standing authorization.
