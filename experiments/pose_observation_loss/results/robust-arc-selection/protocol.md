# Fixed offline comparison of uncertainty-aware steering selection

Use all 497 paired checkpoints (81 final, 416 fresh stationary) already pinned
in `aligned-arcs`. These are selected failed runs, not unseen validation data.
Keep the same maps, forward length 0.6, wheelbase 0.4 and one-degree steering
grid from -35 to 35 degrees. Do not execute a new controller or tune to results.

The target at both estimated and true poses remains the previous full recovery
margin `M = 0.6 + 2*sqrt(2)*sigma + 0.02*age`. All sigma values in these inputs
are 0.1. Fix a heading-error interval of +/-4 degrees and sample its nine
integer-degree offsets. This is an assumed error envelope, not a Gaussian
confidence claim, including for motion-filter poses.

- `nominal`: choose the largest filtered-pose clearance at the central heading;
  accept iff that clearance exceeds M + 1e-9. Reproduce the previous best angle.
- `yaw_min`: choose the largest minimum over sampled headings, subtracting the
  continuous-heading discretization allowance, and accept iff this lower bound
  exceeds M + 1e-9.
- `pose_bound`: use the same steering as `yaw_min`, additionally subtract
  `E = 2*sqrt(2)*sigma + 0.02*age`, and accept iff the resulting bound exceeds
  M + 1e-9. This applies a position envelope in addition to retaining the full
  previous truth-side margin; count the resulting abstentions explicitly.

The heading-grid allowance is `2*0.6*sin(radians(0.5)/2)`. If actual position
error <= E and wrapped heading error <=4 degrees, the pose-bound clearance is
a lower bound on the true-pose clearance at the same fixed steering. Measure
envelope coverage separately; do not use truth to select or accept an angle.
For equal scores within 1e-10, prefer the smaller absolute steering then the
smaller signed angle, matching the previous analysis.

Report accepted candidates, true-margin passes and mismatches, abstentions,
available true grid witnesses among abstentions, angle changes and envelope
coverage. Check analytic geometry against chords, verify exact obstacle
pruning, and verify the conditional bound on every covered checkpoint.
Repeat aggregation byte for byte. No safety, timer, progress or hold improvement
is established by this fixed-pose comparison.
