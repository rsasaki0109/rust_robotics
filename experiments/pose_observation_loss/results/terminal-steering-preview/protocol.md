# Fixed comparison of straight and commanded-steering terminal previews

Use main 5da7775's epoch-divergence diagnostic as the frozen input. Retain all
324 pairs in accounting and all 121 command-changing pairs in this comparison.
Use every published checkpoint without selecting by outcome, error, steering
angle or clearance: 1,116 arm/time checkpoints, each evaluated at the logged
pose, original filtered x,y with logged yaw, true x,y with logged yaw, and full
true pose (4,464 rows). Pin input tables, provenance, map and simulator hashes.
The source trajectories have already been validated; this portable calculation
uses the published checkpoint tables and needs no raw trajectory files.

Reconstruct the identical stopping forecast endpoint with the recorded measured
speed, nominal target and nominal steering: 10 Euler steps at dt=.02 with
a=target-v, then a=clamp(-4v,-1,0), wheelbase=.4, until |v|<=.001 after at least
11 steps, at most 200 steps. Check every reconstructed step count. Reuse the
published exact stopping-prefix clearance, which this experiment does not
change. Verify that a zero-steering .6 terminal arc reproduces every published
straight terminal clearance within 2e-8.

Compare just two .6 terminal paths from that same predicted stop pose:
the existing straight path and a constant-steering arc using that row's existing
nominal steering. Do not search steering angles, use truth to select a curve,
change margins, rerun controls or claim a new trajectory outcome. Evaluate exact
point-obstacle and map-wall minima, including interior wall extrema. Audit both
terminal paths at all four poses against an independently constructed 120-chord
polyline with an analytic sagitta error bound plus 1e-9 numerical tolerance.

Retain entry and release thresholds separately: entry slack is
min(stop,terminal-.1)-entry_margin; release slack is
min(stop,terminal-.05)-release_margin. The physical radius check uses
min(stop,terminal)>.5. Compare logged-pose decisions against full true-pose
decisions using the same controls and margins; record the intermediate true-x,y
pose to isolate heading effects. Record numerical ambiguity within 2e-8.
Report both overly optimistic passes and overly conservative blocks; do not
equate the heuristic margin with the physical vehicle radius or a confidence
bound. Truth is evaluation input only.

Summarize each arm, cohort and both gates for all checkpoints, the first
decisive checkpoint (one per pair), common/own endpoints deduplicated by time,
and final-window fresh stationary checkpoints. These subsets overlap. Highlight
all four hold-changing pairs at the decisive point and candidate own endpoint.
Repeated checkpoints, shared seeds and the command-change selection preclude
treating these counts as independent trials or overall policy success rates.

Changing a terminal-clearance decision alone does not demonstrate readiness:
the existing stationary/freshness/sample, raw recovery straight-line geometry,
positive-command conditions and .5/1-second timers still apply. This checkpoint
study cannot establish continuous readiness, contact avoidance during a rollout
or recovered holds. Save geometry, decisions, subset summaries, accounting and
source fingerprints. Test forecast/geometry boundaries and repeat aggregation
byte for byte. Leave controller defaults and prior published results unchanged.
