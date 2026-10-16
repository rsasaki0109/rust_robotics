# Fixed causal three-epoch position-fusion prototype

Replay the same 81 failed trajectories and 497 aligned-arcs checkpoints. This is
an offline prototype on selected old outcomes, not new validation or a controller
simulation. Freeze the following rule before results; do not tune it afterward.

Inputs to the estimator are sensed pose reconstructed from the log, measured
speed, applied steering, received-correction/precision flags, observation age and
time. It receives no truth, separate noise/bias fields, map or goal. In the log
adapter only, sensed pose equals the pre-command physical pose plus logged noise
and bias, exactly as in the unchanged simulator. Truth is reserved for evaluation.

Keep at most three distinct received-correction epochs, at most 25 samples per
epoch, and no epoch more than 15 seconds old (inclusive). A missed scheduled
correction does not create a new independent epoch. Clear history on measured
speed magnitude >0.02 or precision activation. A received correction starts a new
epoch without discarding stationary history. Retain only local samples whose
age since the latest received correction is <=0.5 seconds, including its first
sample. Propagate retained samples causally with each sample's own sensed heading,
previous measured speed, previous applied steering, DT=0.02 and wheelbase=0.4.

Use a fused position only when stationary, observation age <=1.5 seconds, exactly
three retained epochs exist, the newest matches the latest received correction,
and each older epoch has >=10 samples. The current epoch needs >=1 sample.
Average positions within each epoch, then average those three means equally.
Preserve the existing motion-filter heading. Otherwise use the logged baseline
filtered position and heading unchanged. Record availability and group counts.
No covariance reduction, bias oracle, drift subtraction or confidence claim.

At all checkpoints report baseline and candidate position errors, separating
final/fresh checkpoints, white/correlated cohorts and available/fallback subsets.
For each pose choose maximum-clearance steering over the same -35..35 degree
grid, forward length 0.6 and wheelbase 0.4, using the previous tie rule. Accept
only if clearance > M+1e-9 with the same full margin M=0.6+2sqrt(2)*sigma+0.02*age
at estimated and true poses. Count accepted true passes, mismatches, abstentions,
paired gains/losses and angle changes; preserve sigma and observation age.
Truth never selects an angle or decides acceptance. Audit selected arcs with
full obstacle scans and independent chords. Verify baseline matches saved results.

Pin raw/input/helper/protocol hashes. Save checkpoint estimates and per-epoch
means/counts; repeat all CSVs and provenance byte for byte. No live estimator,
controller, release timer, reachability or hold improvement is established.
