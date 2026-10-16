# Speed-probe zeros at the start or first 0.05 m chord

Start from main 1faf949. Preserve the complete 648-case accounting, 368 timeout
cases and 2,583 opportunities of correction-gate-boundaries. Use every one of
its 6,610 audited distinct states and all 6,187 detailed adjacent pairs, including
readiness acquisition/continuation/loss and successful route exits. Other
opportunities remain explicit with zero audited states/pairs; they are outside
this geometric subset, not geometrically blocked or fabricated future samples.

Pin the tracked accounting, events, states, transitions, changes, geometry,
sources, map and helpers. Reuse the prior state/transition/event IDs. This study
requires those tracked artifacts, not a new raw replay or new trajectories.
Keep both arms, all cohorts, missed/precision/combined categories and source
right-censoring. Existing 180 s opportunities have no new future decisions.

For each audited state evaluate raw, recovery, preview and recorded truth poses
at exactly the recorded nominal steering and speed_margin. Each pose uses its
own recorded heading; truth is the preceding physical row already matched in
the source. This changes only the pose in a geometric query. Do not recompute
the controller, heading, steering, target, uncertainty, timers or trajectory.
Truth uses the same recorded margin, not just the physical collision radius.

Compute separate obstacle and wall clearances at the start and along the first
chord of length .05. Use the source straight branch for |curvature|<1e-9 and a
stable sinc midpoint arc endpoint otherwise. The first chord includes its start.
Scan the complete fixed map; retain nearest obstacle witnesses and endpoints.
At every stage clearance <= speed_margin blocks, including wall equality.
Partition poses exclusively into start_obstacle, start_wall, start_both,
first_chord_obstacle, first_chord_wall, first_chord_both or positive_first_chord.
Start failures take precedence over the chord; do not call the same pose two
different causes. An initially blocked position cannot be cleared by changing
only heading/steering while keeping that position and margin.

Match raw positive/zero first-chord classification to the prior independently
audited full speed-probe free distance at all 6,610 states. Positive means its
recorded distance is at least .05; zero means a start or first-chord failure.
Mark any of the four signed gaps within 2e-8 m of a boundary but do not change
the <= rule or tune tolerances based on results. Do not extrapolate a positive
first chord into a positive whole-path safety claim or an alternative full cap.

For each of raw/recovery/preview compare first-chord classification with truth
at the same margin and steering. Keep both_clear, estimate_clear_truth_blocks,
estimate_blocks_truth_clear and both_block. These are geometric comparisons,
not calibrated sensor error rates or physical collision observations.

For every source detailed pair and all three estimate inputs export pre/post
pose IDs, stage transitions, true classifications, original gate label and
source command/failed mask. Preserve all three input views of a shared recorded
action; do not add them as independent events or restored releases.

Summarize all detailed pairs, plus the fixed source subsets raw_zero_loss
(post joint_lost with recorded speed_cap_zero) and command_only_zero_loss
(additionally failed mask 16). Their source counts remain 550 and 509; retain
every member without selection on geometric outcome. Report input/classification
counts and opportunities/cases separately. Preserve empty source categories
and phases. A pose clearing the first chord has not passed other release gates.

Export accounting, all opportunities with geometric-subset counts, all pose
queries, detailed three-input pairs, state summaries, pair summaries and one
raw-zero-loss record per original transition. Repeat seven CSVs and provenance
byte for byte. Any figure must show both recorded arms and the complete fixed
raw-zero-loss denominator, retaining zero event-kind groups in its numeric data.

No new rollout, input policy, speed cap, margin, collision model or timer is
introduced. Repeated outcome-selected timeouts are descriptive. Keep inherited
observation/physics validation and the held-goal result 140->140 unchanged.
