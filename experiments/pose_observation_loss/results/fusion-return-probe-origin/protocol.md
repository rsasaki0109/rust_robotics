# Frozen initial speed-probe geometry audit

Freeze from main 46878148318e4aaec13696426723433238f60cd5 before reading
new selected raw pose/noise/bias geometry or initial-test outcomes. The preceding
source-domain audit has 1,096 cap-zero references among all 3,751 audited
decisions: 1,067 available and 29 next-loss boundaries. All 909 available joint
nominal zeros have a cap-zero reason; 25 have no other failed admission gate,
including 15 timer resets. Do not select only those 25.

Pin fusion-return-command-sign/sources.json at SHA-256
2090042807aaef058d8c43eee5b112ab0ef658b934b1bccad29e81a5af4bc62b and
independent-verification.json at
3d82c7af8398d4a954043d8179ee244083cb5141b7752cce5e4b165e194dd3b8.
Verify its complete input/helper/code/raw chain and five output hashes. Keep
all 972 planned cases, 324 families, 570 timeouts, 3,997 anchors, 137 first
returns, 3,751 decision references and the preceding 336 summary cells. No
controller, rollout, parameter, RP selection, horizon or admission result changes.

The speed probe returns zero either at its initial unsafe test or after its
first 0.05 chord. Audit the initial test using bounded logged inputs; do not
reconstruct curved endpoints. A certainly unsafe start identifies the initial
branch. A certainly safe start plus the previously verified zero free distance
identifies the first chord by exclusion of source return paths. Retain this
as first_chord_by_exclusion, not a direct replay of unrounded chord geometry.
If the initial test is unresolved, keep ambiguous_zero. Positive probes retain
positive_probe and must not have a certainly unsafe initial test.

The current row prints state after state.update. Reconstruct the sensed pose
used by the probe from the preceding row's x/y plus the current noise_x/y and
bias_x/y, with the original two-addition ordering. Retain both row times and
all six scalar inputs. Validate from step-zero effective_position_sigma or
effective_yaw_sigma_rad that the base noise is positive in every timeout
history, establishing the noisy sensed-pose branch without assuming drift or
runner arguments. Reject an unestablished branch. Do not substitute the current
post-update truth position, recovery mean or Fusion preview position.

Each printed nine-place scalar gets a half-unit radius 5e-10. Add a 1e-12
guard to each summed coordinate enclosure and each final clearance comparison.
Keep the speed_margin interval separate. Require finite bounded coordinates
(absolute value <=100), positive margins below 10, and exact binary64 map
coordinates parsed from the original map strings. Pinned code uses hypot for
point distance. Enclose point-to-obstacle distance using minimum/maximum
distance from an axis-aligned x/y rectangle; compare with the margin interval.
Independently use 60-digit Decimal arithmetic and squared distances in the
verifier. Boundary inequalities are inclusive unsafe (distance <= margin).
Do not force a side at rounded equality.

Check all four walls and all obstacle records, preserving duplicate map rows
as distinct original IDs. The nearby filter cannot exclude an obstacle whose
distance is <=margin because its cutoff is margin+2. Establish positive margins
and preserve a conservative comparison guard; do not attribute a distant point
as a blocker. Save minimum boundary/obstacle slack enclosures, all certainly
violating wall/obstacle IDs, all ambiguous IDs and boundary/obstacle/both masks.
Do not choose a priority cause when constraints fail simultaneously.

Use the original obstacles.csv pinned at
59ff820ccbf076cfd90906da055c4f0b95c58207652d7d28610b294f6c51b3e4.
Check all six raw provenance hashes, compiler fields and both arms' map hashes.
Track the exact map bytes as obstacle-map.csv. Both readers freshly hash all
108 raw files and validate unique complete case grids. Extract the selected
points and their immediate predecessors, rejoin all overlapping prior raw
fields, and retain all original identities and phase/branch labels.

Save probe-decisions.csv with all 3,751 references, old command/admission/timer
columns, predecessor truth/current noise/bias strings, coordinate enclosures,
initial-test state, zero origin and every witness mask. Save anchor-probe-origin.csv
for all 3,997 anchors, retaining all preceding columns and blank new fields for
nonreturns. Add available/boundary x joint/guard counts by origin and by the
three initial-failure masks, plus origin counts for the preceding 25 unknown
conjunction references and nominal-only timer resets. Summarize the same fields
in probe-summary.csv with all 336 original cells. Old result fields remain
inherited from the pinned proof; independently reconstruct new columns and
original census/received/RP weights before selection, exact Fraction masses
and new raw/case-mass shares. Preserve 10,995 RP records and masses 570/570/157.
Check mode/category closures; distinct case counts and shares are not additive.

Freeze one figure of available cap-zero origins by original partition and
joint/guard branch, with initial-unsafe, first-chord-by-exclusion and ambiguous
counts and explicit zero denominators. Export all 252 original-mode summary
cells and all 137 returned anchors. Track SVG/CSV/provenance; PNG stays scratch.
Inspect it and repeat collector and plot artifacts byte for byte.

The standalone verifier imports no analyzers or new collector/plot code. Reuse
only the pinned original standalone census/hash helper. Independently parse
selected raw grids, build Decimal coordinate rectangles, compare squared
obstacle distances and retain the same conservative guards. Extract the unchanged
initial source test, actual Point2D/hypot implementation and minimal VehicleState
into a stdlib Rust harness. Test inclusive walls/obstacles, nearby filtering,
strictly safe starts and simultaneous constraints; compare native rounded-center
results only where bounded classification is certain. Center evaluation is a
cross-check, not recovery of unrounded poses. Check every output field, inherited
column, new summary, weight, closure and source/protocol/raw/map/output/plot hash.
Earlier admission geometry, timers, command-domain and Fusion metadata evidence
remain explicitly inherited. No full curved-probe or controller replay is claimed.

Test predecessor/current row ordering, noisy-branch establishment, finite/range
guards, exact map binary64 conversion, inclusive thresholds, rectangle distance
enclosures, certain/ambiguous blockers, duplicate IDs, simultaneous wall/obstacle
failures, source-path exclusion, positive-probe contradictions, all original
branch/phase categories, nominal-only resets, original unequal parent measures,
blank/zero shares, censoring/nonreturns, case/grid isolation and nonmutation.
Run the full Python suite. Save report/README/evidence, then merge/push this
bounded audit under the standing authorization.
