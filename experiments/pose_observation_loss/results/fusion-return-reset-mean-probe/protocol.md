# Fixed mean-position replay of the 19 reset probes

Freeze from main 4fcb8a8365dadf6280adb1a41909d9291a974496 before reading the
previously unextracted nominal steering or evaluating the new full mean-position
probes. This is a retrospective, rounded-input geometry diagnostic using known
reset selections, not a blinded trial or a closed-loop controller comparison.

Retain all 19 distinct reset edges in their original order, the four case
histories, two families, 17 initial-point and two first-chord origins, all full
failed-gate masks and the 185 overlapping-ending multiplicities. Do not select
new times, objects, seeds or weights. Do not count overlapping endings as
independent trials. The prior raw trajectories and original controller outcomes
remain unchanged.

Pin the original surplus table, compact review, original raw/proof manifest,
obstacle map and the three Rust source files in freeze-pins.json. Derive the
selected raw file names mechanically from cohort/variant/mode/seed. Their full
SHA-256 values must match the existing first-chord manifest before extraction.
Do not regenerate missing raw data with a newer simulator.

Extract exactly each selected decision row and its immediate predecessor from
the two original trajectory CSV files. Preserve round-trip logged strings for
prior truth x/y/yaw, current noise and bias x/y/yaw, nominal_steering,
speed_margin, speed_free_distance, speed_cap and uncapped_target_speed.
Join scenario/planner/time to the original case/step and compare every available
overlapping field with the inherited surplus table. Preserve CSV row numbers,
raw paths and hashes. Track this small extraction and its provenance so future
geometry replay needs no multi-gigabyte raw cache. Keep an extraction command
for independently rebuilding it from the original recorded raw roots.

For the raw arm reconstruct sensed x/y/yaw as (previous truth + current noise)
+ current bias, in the original two-addition order. For the mean arm replace
only x/y with the inherited decision_mean_x_center/decision_mean_y_center.
Keep the raw sensed yaw, original logged nominal steering, original margin,
map, nearby-obstacle rule and all other probe parameters identical. Do not use
mean yaw, recompute steering, choose a new arc, shrink a margin or use truth
in the diagnostic's geometry. Truth is only part of reconstructing the logged
original sensed input. These centers are reconstructed from rounded logs;
they are not hidden unrounded simulator states or interval guarantees.

Implement the unchanged speed_free_distance semantics in Python: initial wall
and obstacle tests are inclusive at margin; nearby includes every original map
record at distance <=2+margin from that arm's start; retain duplicates. Use
curvature=tan(steering)/0.4 and the original abs(curvature)<1e-9 branch. Probe
40 chords at step*0.05, using the original sin/cos subtraction formulas,
inclusive endpoint wall tests and closest-point segment/obstacle distance.
Return (step-1)*0.05 at the first unsafe chord or 2.0 for an entirely clear
two-unit horizon. A zero return can result from either an unsafe initial point
or an unsafe first chord; keep these distinct in the new output.

Retain the first failing step, every simultaneous wall/obstacle witness and
initial-point status for each arm. Report raw/mean free distance, the original
origin, mean positive-distance and full-horizon-clear flags. Also compute the
existing algebraic speed cap clamp(sqrt(0.15^2+distance)-0.15,0,1), and the
hypothetical nominal target min(original uncapped target, cap). Positive target
is only a same-step nominal predicate. Do not infer new release gates, timer
values, recovery decisions, travel, contacts or mission success from it.

Require every reconstructed raw probe to return the recorded zero, with initial
versus first-chord failure matching the inherited original origin. Fail the
audit on disagreement rather than adjusting the selection or tolerances.
Keep all mean outcomes, including initial failures, first-chord failures,
later failures and full-horizon passes. Summaries retain all cases and original
origins, both distinct-edge counts and original ending multiplicities; use
counts rather than new success rates. Preserve the original failed-gate masks
to show which other original gates were failing without predicting their new
values.

Cross-check both arms using a standalone verifier that imports no collector.
Extract the unchanged Rust speed_free_distance, point_segment_distance,
Point2D distance/constructor, VehicleState and wheelbase constant from the
pinned repository sources. Compile a standard-library harness with rustc and
query all 38 centers using the full original map. Require exact agreement on
the discrete free-distance values. Independently recompute the speed-cap and
nominal-target formulas with 60-digit Decimal and compare at absolute 1e-12;
this tolerance is not used to change the native probe result or any geometric
predicate. Save compiler/source/harness/query/trace provenance and the actual
native output in the verification packet. These are new native function calls,
not new native dataset trajectories.

Test initial and first-chord failures, later chords, a clear full horizon,
wall/obstacle equality, segment-interior blockers, duplicates and simultaneous
witnesses, straight/curved/mirrored and near-zero steering, positive partial
distance versus full-horizon status, reconstruction order, exact input joins,
duplicate/missing rows, corrupt pins/provenance and output-path protection.
Include Python/native comparisons on synthetic cases that exercise these
branches. Run the complete existing Python/native test suite without skips.

Run extraction, collection and the native verifier on the original selections.
Re-extract and replay into fresh external directories, compare deterministic
CSV/provenance bytes, and check that generated inputs exactly match the tracked
extraction. Track the compact input/outputs, protocol, pins, provenance and a
report with all 19 rows; link existing geometric figures where relevant rather
than adding a picture of the same points. Leave raw logs, binaries and work
directories outside the checkout. Add documented replay commands, require CI
success, then merge and push under the standing authorization.
