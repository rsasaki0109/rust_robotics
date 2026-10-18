# Frozen adjacent fixed-obstacle point-clearance audit

Freeze from main 9941ce31dcfacd75aa948c0041965565ff1d0bd6 before extracting
new predecessor obstacle distances or margins. Audit exactly the preceding
1,096 zero-probe references, preserving all 3,751 decisions, 3,997 anchors,
137 returns, 972 cases, 324 families, 570 timeouts and 336 summary cells.
The original 25 nominal-only conjunction failures and 15 nominal-only resets
are fixed subgroups. Predecessors n-1 remain historical context, including
before a return; they add no decisions or follow-up. No source/controller,
parameter, outcome, contact label or rollout changes.

Use ALL definite obstacle IDs inherited from the current initial-unsafe test,
or from the current direct first-chord test. Keep their original global row
IDs, order and duplicate coordinates. Reject empty/duplicate ID selections or
wall-only ambiguity. Do not select a different nearest obstacle, reduce a
multi-obstacle set, or extend the set to the whole map. The original 61
first-chord references can have clear raw starts: a point test is distinct
from the curved 0.05-m chord, 0.6-m recovery segment, boundary or whole probe.

Both readers freshly hash/stream the preceding 108 raw files, validate complete
case grids and the 570 noisy initial branches, and replay the 63 selected
histories (567,000 steps). Reuse pinned collector deque and independent
admissions-list/window mean implementations, including original speed, age,
pause, clear/append and circular-yaw checks. Reconstruct n-1 and n, checking
both logged means and inherited current and previous rounded centers/counts.
Use reconstructed means, not the separately rounded logged recovery centers.
Empty windows use raw fallback. Save each context's own exact raw speed_margin
string and use it for raw AND mean point gaps. Pin that original recovery and
speed margin expressions agree for this recorded branch; do not infer drift
arguments or treat the margin as fixed across contexts.

For each fixed obstacle and each raw/mean point at each context, compute
Euclidean point distance d and gap g=d-margin. Use the source Point2D.distance
sqrt(dx.powi(2)+dy.powi(2)), not a segment or hypot predicate. Inputs retain
H=5e-10, P=1e-12 and sensed coordinate radius R=3H+2P. Mean radius is R+2P
when nonempty, R for fallback. Obstacle coordinates are the pinned binary64
map constants. Bound each coordinate difference by center difference +/-
point radius +/-P, then bound the Euclidean norm from the rectangle (minimum
absolute coordinate is zero when it contains zero), padding sqrt by P and
clipping the lower bound at zero. Margin interval is logged value +/-H.
Subtract intervals with P. These are conditional arithmetic bounds on rounded
inputs, not recovery of original unrounded values or a formal floating proof.

Also take the minimum distance across the ENTIRE fixed set for each point and
context, independently minimizing lower and upper bounds. Subtract the common
context margin. Record exact rounded-center minimum ID ties and every possible
minimum ID whose lower distance <= minimum upper distance. Preserve the same
metrics per obstacle in witnesses; neither a tie nor a minimum switch drops IDs.
A gap is clear only if lower>0, blocked if upper<=0, else unknown. Record all
nine previous-to-current state transitions, for raw and mean separately.

For each witness and fixed-set minimum, compute adjacent distance, margin and
gap changes with interval subtraction/P. Classify distance toward if upper<0,
away if lower>0, else unresolved (including zero). Margin change uses increase,
decrease, unresolved; gap change uses wider, narrower, unresolved. Verify
Delta gap = Delta distance - Delta margin with rounded-center residual<=P;
check the independent interval prediction intersects the direct gap-change
interval and encloses the rounded native difference. Save signed center and
bound fields; a zero-containing interval is never forced to be no change.

Save point-decisions.csv with all prior columns and a new audit flag; positive
probes have blank new fields. Save each context's raw/mean coordinate bounds,
margin strings/bounds/change/direction, all distance/gap bounds, states and
transitions, distance/gap change directions, closure residuals and minimum IDs.
Save point-witnesses.csv in original reference and inherited obstacle order,
including coordinates and all per-obstacle point metrics. Save anchor-point.csv
and point-summary.csv retaining all prior fields and original exact weights.
Add phase x branch audited counts, nine transitions for each point kind,
three distance directions and three gap directions for each kind, three margin
directions, and original-25 / nominal-only-reset raw-narrower-gap counts.
Add available anchor flags for raw clear-to-blocked, mean clear-to-blocked,
and raw-toward with mean-away, and raw/case-mass shares. Reconstruct original
measures before selection; preserve 10,995 RP records and masses 570/570/157.
Check all additive mode/category closures. Blank nonreturns/empty denominators;
flags can overlap. No counts of witnesses are called new decision references.

Freeze one 2x3 figure of available RAW point-gap transitions, nine states,
by original partition and joint/guard branch, retaining empty branches. Export
all 252 original-mode summary cells and 137 returned anchors. Track SVG/CSV/
provenance, inspect scratch PNG, repeat collector/plot artifacts byte for byte.
Report raw/mean transition and direction counts plus min-lower/max-upper
Delta distance/margin/gap envelopes for all 1,096 references, the preceding
887 pairs with two nonempty windows, and the original 25 subgroup. Use the two
already fixed first-chord examples (C0228/2987/175.30 and C0310/2994/176.16).

The standalone verifier imports no analyzers or plotters. Reuse pinned
independent raw/window/native helpers; independently calculate all new point
geometry/intervals and exports with 60-digit Decimal. Compare new numeric fields
at abs 1e-9, rel 1e-10, exact states/IDs/counts/raw and inherited strings.
Tolerance never narrows intervals. Compile the unchanged original Rust window
fragment and original Point2D struct/new/distance. Replay original window counts
and trace both contexts of every reference; separately trace every rounded
raw/mean point vs every fixed obstacle and validate native distances, gaps,
minima and changes against declared intervals. Save small queries/traces and
hash the full scratch replay inputs. The native geometry checks rounded centers,
not the unrounded original controller and not a closed-loop replay.

Test inclusive gap equality, strict signed changes and unresolved zeros,
rectangle/corner/inside bounds, fallback means, different previous margins,
fixed multi-obstacle minimum switches and duplicate-coordinate IDs, closure
intervals, raw string fidelity, first-chord point/probe distinction, native
source mapping, censoring/nonreturns, unequal measures and additive closures.
Run full Python and native suites, save report/README/evidence, then merge/push
this bounded audit under the standing authorization.
