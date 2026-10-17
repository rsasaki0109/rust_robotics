# Frozen direct first-chord audit

Freeze from main 0836de71eaf2cf03e476cb16a1ed3e0022be7285 before reading
new yaw/noise-yaw/bias-yaw/nominal-steering outcomes. Audit all 61 preceding
first_chord_by_exclusion references: 60 available and one next-loss boundary.
Two are among the preceding 25 nominal-only conjunction failures and both
reset a carried timer. Do not select only those two. Preserve all 3,751
decision references, 3,997 census anchors, 137 first returns, 972 planned
cases, 324 families, 570 timeouts and the same 336 summary cells.

Pin fusion-return-probe-origin/sources.json and independent-verification.json
at their SHA-256 values recorded in freeze-pins.json alongside this protocol.
Verify the complete preceding input/helper/code/raw/map chain and its four
outputs. Earlier initial-test, command/admission/timer/Fusion metadata results
stay inherited and unchanged. No controller, parameter, rollout, horizon, RP
selection, contact/error label or causal benefit is introduced.

Retain the preceding sensed x/y rectangles and verify their selected raw
inputs. Reconstruct sensed yaw from preceding truth yaw plus current noise_yaw
and bias_yaw, in the source's two-addition order. Use the logged nominal_steering
captured before pause/guard overrides, not the final steering. Each nine-place
input has half-unit radius 5e-10. Summed yaw gets a 1e-12 guard, matching the
preceding coordinate policy. Require finite coordinates/yaw with absolute
value <=100, steering absolute value <=0.612 and margins in (0,10).

Evaluate the original first endpoint at source length float(0.05), curvature
tan(steering)/float(0.4), and the straight branch abs(curvature)<1e-9. Use
interval arithmetic with a 1e-12 elemental numerical allowance on sin/cos/tan
and each arithmetic operation. Sin/cos ranges use the global Lipschitz bound,
clipped to [-1,1], with that allowance. Tan is monotone on the permitted angle
interval. Keep source arithmetic ordering, including the curved sin/cos
differences divided by curvature; do not replace it with a stable geometric
arc and claim identical source arithmetic.

If the curvature interval comes within [-1e-6,1e-6], use a deliberately coarse
endpoint rectangle x/y within the start rectangle expanded by 0.06, and mark
coarse_fallback. This covers the 0.05 displacement plus 0.01 arithmetic reserve:
the curved branch denominator is at least 1e-9 and the elemental allowances
give a displacement reserve below 4*1e-12/1e-9. Keep unresolved predicates
unknown rather than choosing a branch or tuning the threshold after outcomes.
Outside that band, interval division stays away from zero. State the numerical
allowances and fallback as assumptions of this bounded source audit; do not
claim a formal proof of all possible platform libm implementations.

For each original obstacle, bound distance to the first chord using a rounded-
center segment and the Hausdorff bound max(start endpoint displacement,
final endpoint displacement) over the endpoint rectangles. Segment distance
is 1-Lipschitz in this displacement bound. Retain the original source projection/
clamp distance and add a conservative 1e-9 segment-comparison allowance, with
the preceding margin interval. Check all four endpoint walls and all original
map records, retaining duplicate IDs and simultaneous boundary/obstacle masks.
The first chord remains within the source's nearby cutoff by its short length;
do not introduce a distant blocker. Preserve certain/possible witness IDs.

Save first-chord-decisions.csv for all 3,751 references. Retain all preceding
columns and add an audited flag; new geometry fields are blank for the other
3,690 references. For the 61 add raw yaw/steering inputs, angle/curvature and
endpoint enclosures, rounded-center endpoints, fallback flag, segment error
radius, endpoint-boundary and obstacle gap intervals, direct first-test state
and every witness/mask. Keep direct results separate from the inherited
first_chord_by_exclusion label. A certainly safe direct result contradicts
the preceding zero probe and must fail validation. Unknown stays explicit.

Save anchor-first-chord.csv for all 3,997 anchors and first-chord-summary.csv
for the same 336 original cells, retaining every preceding column. Add available/
boundary x joint/guard audited/direct-unsafe/direct-unknown and three failure-
mask counts, plus direct-result counts for prior-unknown conjunction references
and nominal-only resets. Nonreturns have blank new fields. Reconstruct original
census/received/RP measures before selection and exact Fraction masses/shares;
preserve 10,995 RP records and masses 570/570/157. Check new additive mode/category
closures. Keep empty denominators blank and shares/case counts nonadditive.

Freeze one figure of available first-chord direct boundary-only, obstacle-only,
both and unknown counts by original partition and joint/guard branch. Keep
explicit denominators and empty branches. Export all 252 original-mode summary
cells and all 137 returned anchors. Track SVG/CSV/provenance; PNG stays scratch.
Inspect it and repeat collector and plot artifacts byte for byte.

Both readers freshly hash/stream all 108 original raw files and validate unique
complete case grids. Extract the 61 selected points and predecessors, check
overlapping prior raw fields and preserve all original identities. The standalone
verifier imports no analyzers or collector/plot code; reuse only the pinned
original standalone census/hash helper. Independently use 60-digit Decimal
Taylor sin/cos and derived tan, interval arithmetic and exact binary64 map
coordinates. Use Decimal projection/clamp distance for the center segment,
independent of collector float calculations.

Compile the unchanged full speed_free_distance, original Point2D distance,
point_segment_distance and minimal VehicleState with stdlib Rust. Extract the
unchanged first-endpoint arithmetic for native tracing; the full original probe
provides the independent zero check at rounded centers. Save native endpoints,
initial-test state, endpoint-wall and nearby obstacle witnesses and probe return.
Check endpoint enclosures and every certain direct condition/witness against
these center results. A center is a cross-check, not the unrounded original
input. No closed-loop trajectory or later-chord replay is claimed as new evidence.

Test angle ordering, nominal versus final steering, near-zero fallback and
straight/curved arithmetic, interval operations, trig range extrema, inclusive
walls/obstacles, segment-interior blockers, endpoint/Hausdorff enclosures,
duplicate and simultaneous witnesses, safe-result contradictions, blank unaudited
fields, branch/phase categories, censoring/nonreturns, original unequal measures,
blank shares and closures. Run the complete Python suite and native tests for
straight/curved endpoints, first-chord-only blockers, initial blockers, inclusive
segment distances and free probes. Save report/README/evidence, then merge/push
this bounded audit under the standing authorization.
