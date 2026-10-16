# Fixed selection comparison using the three-epoch positional spread

Use the tracked 497 estimates and retained epoch means from stationary-epoch-fusion.
Keep availability (389), fallback (108), fused positions, heading, map, margins,
steering grid, arc length and wheelbase unchanged. No new estimator or drive.
These are the same selected failed-run snapshots, not unseen validation samples.

Compare the original baseline (pinned reference), fused_center (reproduce prior
three_epoch), and two predeclared rules at available checkpoints:

- epoch_min: for every steering angle, take the minimum clearance over the
  three retained epoch mean positions at the unchanged filtered heading; choose
  the angle maximizing that score. This is discrete agreement, not a bound for
  positions between or outside those means.
- spread_yaw_bound: R is the largest distance from the fused center to a retained
  epoch mean. At the fused center sample heading offsets -4..4 degrees in integer
  steps, subtract 2*0.6*sin(radians(0.5)/2) from the minimum clearance, then
  subtract R. Choose the angle maximizing this score. If true position is within
  R of the center and wrapped heading error <=4 degrees, this is a conditional
  lower bound on true clearance at the same steering. Do not call the empirical
  R or the heading envelope a probability/confidence guarantee.

At unavailable checkpoints all three candidate rules fall back to the original
baseline exactly, without a new heading or position allowance. Use the previous
tie rule and strict score > M+1e-9 threshold, retaining the same full true-side
margin M=0.6+2sqrt(2)*sigma+0.02*age. No tuning to these results; no truth enters
steering selection, scores or acceptance. Truth is used only for evaluation and
empirical coverage of the assumed envelope.

Report accepted true passes, mismatches, abstentions, changes and paired gains/
losses against fused_center, with all/available/fallback subsets and separate
final/fresh and white/correlated cohorts. Report R and envelope coverage. Check
the conditional lower bound on all covered selected spread-rule arcs. Audit
selected score-origin poses and true poses with full obstacle scans and chords;
reproduce pinned baseline/fused_center decisions. Show that agreement at mean
positions alone does not certify their convex hull.

Pin tracked input/helper/protocol hashes. Save all scores and decisions, repeat
all CSVs and sources.json byte for byte. No live control, timer, hold, physical
collision probability or deployability claim is established by this comparison.
