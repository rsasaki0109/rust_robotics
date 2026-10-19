# Position accuracy around every joint release

Freeze from main956f4284ad16c53efba225bd8862ccf0641c7aee before computing new
error distributions. Known design observations: seed1003 has108 cases/54 pairs,
251 resumes and12801 lag0..50 window points. The previous fixed-input replay
cleared all13 candidate lag2 entries by continuing mean x/y, but104/126 anchors
invalidate at lag3 and every anchor by lag18. A longer handoff is not established.

Retain the same complete population and fixed50-decision follow-up, clipped
only at packet endpoint, with overlapping anchors separate. Include held and
timeout, both arms, every noise/mode/map/planner, no-loss zero-resume cases,
zero targets and invalid carry prefixes. No new seed, simulation, controller,
threshold, timer, duration or default change. Truth is evaluation-only.

Bind the complete137-file raw manifest, both16-file live audits, and BOTH
14-file post-handoff replays by their frozen manifests before analysis. Bind
all inherited inputs and prior sources/results. Reuse prior contact/hold and
full estimator audits by hash, without claiming to repeat those audits here.
Commit implementation and meaningful tests before computing the new errors.

TIME ALIGNMENT: speed-probes.csv true_x/y/yaw/v are written AFTER state.update,
although sensor and nominal-command fields describe the preceding decision.
For step1 use the full-precision initial trajectory checkpoint; at every later
step use the previous sidecar's full-precision post-state. This is the physical
state before the current update. Verify ALL827706 transitions: measured speed
exactly equals predecessor v; sidecar post-state matches trajectory rounding;
post x/y/v and normalized yaw match original vehicle dynamics using current
applied acceleration/steering; time and case joins match. Do not substitute the
current post-state or rounded predecessor trajectory when exact sidecars exist.
This is an alignment/dynamics audit, not a rerun of contact or hold checks.

Evaluate seven predeclared poses at EVERY lag0..50 point with the original
stateless native recovery/stopping/curved-terminal/aligned-release functions:
recorded, carried_diagnostic_xy, gated_carried_xy, base_estimate_xy,
current_recovery_xy, truth_xy_eval, truth_pose_eval. Use the frozen existing
native_guard_release_census compiler/evaluator unchanged, with an explicit map.
At lag0 the two carry variants use the release mean; at following lags bind
their x/y to the previous replay's carried point and hypothetical valid prefix.
Independently verify sequential transport and irreversible prefix invalidation
from observed sensors, previous measured speed and actual applied steering.
No truth may enter a sensor-only pose or its transport/selection.

base_estimate_xy uses the original guard base x/y before mean selectors;
current_recovery_xy uses the CURRENT recovery mean, recording count0/raw fallback
without discarding it. Both retain recorded selected yaw. The first three
variants must reproduce all37901 previous native variant geometries exactly;
at lag0 the two carry poses must also equal the recorded release x/y.
truth_xy_eval replaces only x/y by the aligned physical state, retaining selected
yaw; truth_pose_eval also uses aligned physical yaw. Both preserve the actual
measured speed (verified equal to physical pre-v), nominal target/steering,
ordered margins, precision and age. These are evaluation references, not
controllers, perfect paths, new actions or predicted treatment outcomes.

Export x/y and Euclidean position error, wrapped absolute yaw error, entry
clearance error against truth_xy_eval and truth_pose_eval, and BOTH directions
of entry/release threshold disagreement. Separately count entry disagreements
when the recorded prior guard is inactive and joint_pending is false. Compare
each pose's position error with recorded: closer if lower by>1e-12m, farther if
higher by>1e-12m, otherwise tied. This epsilon only handles numerical ties.
Retain current eligibility and observed pose source, every invalid-prefix row
and zero target. Never describe a threshold disagreement as an actual collision
or an optimal safety decision; margins and nominal path still belong to the
recorded uncertain controller. truth_xy isolates position; truth_pose includes
the effect of selected yaw. Preserve unfavorable errors and missed thresholds.

For all cases and summaries report counts, mean/RMSE/max position error, signed
x/y error, mean absolute yaw error and mean entry clearance error. Also retain
eligible-prefix counts/mean/RMSE and closer/farther/tied counts. Empty cells have
zero counts and EMPTY accuracy values, not zero error. Use pooled window-point
statistics, without treating points or shared-seed cases as independent trials.
Keep108 cases for EACH variant,54 pairs for EACH variant, every251-resume record,
and all noise(all/white/correlated),mode(all/none/random/blackout),arm,
outcome(all/held/timeout),variant summary cells for fixed scopes lag0,lag1,lag2,
within5,within25,within50,triggers_within50. within scopes exclude lag0;
triggers scope selects actual trigger rows within50, not favorable queries.
Report every original lag2-trigger event (including worse/tied error) alongside
the whole lag2 cohort; do not restrict analysis to the known13 events.

Run two complete fresh evaluations on identical retained inputs. Require every
table, native query/trace/source/binary/log and provenance byte identical.
Keep all bulk rows outside Git, retaining complete manifests, compact
case/pair/summary/verification/resume exports and the native source. Reject
Python-O, changed hashes, incomplete/mixed/reordered joins, nonfinite inputs,
existing/internal output, and uncommitted implementation. Preserve failed logs
and document any amendment without replacing original inputs. Run the complete
suite without skips and related CI; merge under standing authorization.
