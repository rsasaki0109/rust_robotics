# Fixed pose-component diagnosis

Reuse all 497 paired checkpoints from aligned-arcs and the nominal/yaw_min
decisions from robust-arc-selection. These are previously selected failed runs,
not new validation samples. Do not change steering, acceptance, margins, maps,
arc length, wheelbase, noise or controller. pose_bound uses yaw_min's angle and
adds no distinct geometry, so do not count it as a third independent rule.

For each fixed selected angle compute the exact clearance at four poses:
FF = filtered position and heading; TF = true position, filtered heading;
FT = filtered position, true heading; TT = true position and heading.
Use full obstacle scans and check every distinct checkpoint/angle/pose with
independent 120-chord geometry. Match FF and TT to prior results. Retain the
same full true-side recovery margin M, including sigma and age.

Among accepted candidates whose TT clearance fails M, partition the results:
only the position substitution fails; only the heading substitution fails;
either substitution alone fails; neither alone fails but both together fail.
Use the existing strict pass threshold > M + 1e-9. These substitutions describe
geometry at a fixed angle, not an executable estimator or alternative policy.

Decompose total signed clearance loss FF-TT symmetrically over both orders:
position = ((FF-TF)+(FT-TT))/2;
heading = ((FF-FT)+(TF-TT))/2.
Check that contributions sum to total loss. Preserve negative contributions,
which mean a component mitigates loss rather than adding to it. Report counts,
means and medians separately for the 81 final and 416 fresh stationary pairs
and each rule. These signed attributions are descriptive and do not establish
causal estimator error, statistical independence or attainable improvement.

Count mismatches where true starting clearance alone is <=M: these cannot
meet the same margin for any arc containing that start, regardless of steering.
Keep this separate from the component partition and do not infer physical
contact from a full-margin failure.

Record input, helper and protocol hashes. Repeat all CSVs and sources.json
byte for byte. No live controller, timer, reachability or hold claim is made.
