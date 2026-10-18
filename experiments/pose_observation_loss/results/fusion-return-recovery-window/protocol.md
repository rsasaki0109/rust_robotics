# Frozen reconstruction of stationary recovery samples

Freeze from main fe8152e9331cd4a93682fbf87379d45af98253d9 before extracting
new window-membership, eviction, reconstructed-mean or decomposition outcomes.
Audit all 1,096 preceding zero-probe references, including all available and
next-loss joint/guard references. The preceding 25 nominal-only conjunction
failures and 15 nominal-only resets are fixed subgroups. Keep all 3,751 decision
references, 3,997 anchors, 137 returns, 972 cases, 324 families, 570 timeouts and
336 summary cells. Earlier geometry/admission/command/timer/Fusion results stay
inherited. No policy change, new rollout, contact label or causal benefit.

Pin preceding sources/proof, complete transitive input/helper/code/raw chain,
protocol and freeze. Both readers freshly hash/stream all 108 raw files and
validate unique complete case grids and the 570 noisy initial branches. Fully
reconstruct the window only in histories containing a selected zero reference;
report their count and number of replayed decision steps separately from the
complete raw-grid accounting. Never claim all 570 windows were newly replayed.

For each selected history start with an empty window, last observation step
zero, and the initial post-step paused flag. At each n>=1 reconstruct sensed
x/y/yaw using preceding logged truth plus current noise then bias, in source
two-addition order. Use measured_speed (roundtrip binary64), not rounded v,
for the source speed threshold. Update last observation step on correction
received or precision activation. Compute observation_age using original
binary64 n*0.02-last_step*0.02; compare the six-place logged age with a half-unit
5e-7 plus 1e-12 guard. Do not replace the source comparison at 1.5 by integer
ticks or the printed age. The paused input to window admission is preceding
post-step safety_paused OR current correction&&!correction_received. Carry
the current logged post-step flag for the next iteration, rather than replaying
the rest of the controller or presuming no release occurred.

Apply the unchanged clear predicate: received, precision activation, missed
scheduled correction, abs(measured_speed)>0.02, or exact age>1.5. Preserve all
simultaneous clear reasons. Append sensed pose if paused, abs(speed)<=0.02 and
age<=1.5, then drop the oldest if count>25. Verify the logged sample count at
every replayed step. Keep the latest clear step/mask, all member step IDs,
pushes and capacity evictions since that clear. Do not assume memberships are
contiguous. For a nonempty window use source-order arithmetic x/y means and
circular yaw mean atan2(sum sin,sum cos). Otherwise use the current sensed pose.
Check source routing, thresholds, order, constants and helper/code hashes.

For every selected reference save a lineage row per retained member, ordered
oldest to newest, including source step/time, prior truth, noise, bias and
rounded sensed centers. Empty windows get one explicit raw_fallback row with
sample_order zero, not a fictional retained sample. Preserve case/event/phase
identities and the original reference order. Track a selected native query
table and trace; the full native replay input stays scratch and hash-pinned.

Each nine-decimal truth/noise/bias input has H=5e-10. Require finite magnitudes
<=100. Each sensed component gets radius 3H+2P with P=1e-12 per elementary
operation. Independently propagate source-order sum/division intervals for
x/y means, with P on each operation. Require reconstructed mean intervals to
intersect logged recovery x/y intervals (H). Native means at rounded inputs
must lie inside the independent enclosures; do not require these centers to
equal the unavailable unrounded source means.

Yaw uses sin/cos global Lipschitz intervals with P. For nonempty windows form
mean sine/cosine enclosures and their vector displacement radius. If the
center resultant norm is positive and radius<=norm/2, use circular angle
radius 2*radius/norm+P; otherwise keep full_circle uncertainty explicitly.
Use canonical center zero for full_circle, whose center is otherwise arbitrary.
The bound follows asin(r/norm)<=2r/norm on [0,1/2]. Compare logged/native yaw
on the circle, including wrap at pi, with the corresponding log half-unit or
P guard. The standalone verifier uses pinned Decimal Taylor sin/cos and a
separate 80-digit, half-angle-reduced atan series/atan2. No platform-wide libm
proof is claimed; all allowances are conditional numerical assumptions.

For x/y decompose rounded-center mean minus current sensed into mean-minus-
current prior truth, noise and bias terms, in original member order. For a
raw fallback each term is zero. Save signed components, their norms and the
source-order closure residual; require absolute residual<=1e-12. These are an
algebraic decomposition of rounded inputs, not a physical causal attribution.
Report predeclared min/max ranges for all selected references and the preceding
25 subgroup. Do not introduce a dominance threshold or select new cases.

Save window-decisions.csv retaining all preceding columns, with an audited
flag and new fields blank for positive probes. Add membership IDs/count,
clear metadata, pushes/evictions, reconstructed x/y means and intervals,
circular yaw center/radius/kind, logged residuals, component differences/norms
and closure residuals. Save window-lineage.csv. Save anchor-window.csv and
window-summary.csv retaining all prior columns and original measures. Add
phase x branch counts for audited/mean/fallback/growing/full/evicted,
prior-unknown mean and nominal-only-reset mean. Add available mean/fallback/
evicted anchor flags with original raw/case-mass shares. Nonreturns have blank
new fields. Original weights precede selection; keep 10,995 RP records and
masses 570/570/157, exact Fraction aggregation and additive mode/category
closures. Empty denominators stay blank; overlapping flags are nonadditive.

Freeze one 2x3 figure of available mean/fallback reference counts by original
partition and joint/guard branch, with denominators and empty branches. Export
all 252 original-mode cells and all 137 returned anchors. Track SVG/CSV/manifest,
inspect scratch PNG, and reproduce collector/plot artifacts byte for byte.

The collector may reuse pinned preceding analyzers for hashes/identities and
original exact weights. The standalone verifier imports no analyzers/plotters:
reuse pinned verify_recovery_geometry.py/verify_first_chord.py and their core
for hashes, original identities and Decimal trig only. Independently replay
membership with a list since the last clear, taking its last 25 admitted
members, and rebuild all new fields, lineage, summaries and plot exports.
Compare new numeric fields at abs 1e-9, rel 1e-10, yaw centers circularly;
require exact counts/IDs/masks/flags and inherited strings. Tolerance never
narrows any enclosure. Compare every lineage raw string exactly.

Compile the unchanged original recovery-window update and mean fragment,
original VehicleState and constructor, and original DT with stdlib Rust.
Feed all steps of selected histories using independently reconstructed inputs
and source exact ages; verify all counts and trace all selected means. Reuse
no collector in this native input path. Test every clear reason and simultaneous
clears, clear-then-append order, inclusive speed/age thresholds, binary64 age
edges, pause-before versus post-step release, capacity/ordering, noncontiguous
memberships, empty fallback, circular wrap/weak resultants, mean rounding,
truth/noise/bias closure, original unequal weights, censoring, empty shares and
closures. Run full Python and meaningful native tests, record evidence/README/
report, then merge/push this bounded audit under the standing authorization.
