# Frozen new-seed observation replay validation

Fix seeds 268, 269, 270, all three loss modes, all three maps/planners and noise
correlation times 0 and 1 seconds. Generate unchanged aligned-release controller
trajectories with the existing run.py --aligned-release-comparison command.
The runner also generates entryline controls; analyze all 162 aligned cases only.
Record both run provenance documents, hashes and the runner's verification.
Use the same fine-padding fixtures and simulator as the prior aligned study.

Freeze StationaryEpochFusion and the four arc selectors exactly at main 36154a8;
do not tune rules or thresholds to these new data. The prototype runs only in
offline observation replay. The generating controller does not use its output.
This validates on new observations, not alternative-control reachability/holds.

Select one final pre-command checkpoint from every aligned trajectory, including
held, timeout or collision outcomes, stationary or moving. Also select fixed times
150.5,155.5,...,175.5 seconds when present, observation age is 0.5 and measured
pre-command speed magnitude <=0.02. A checkpoint coinciding with the final row
is retained once as final. Do not filter by success, failure or estimator error.
Keep logged sigma, including precision-active 0.03, and observation age. Evaluate
true and estimated pose at the same pre-command epoch; the row timestamp is the
original simulator's observation event convention.

Replay the existing sensor-only fusion API and reconstruct the baseline filter
at every step. Select baseline, fused_center, epoch_min, spread_yaw_bound with
the same grid, tie rules, conditional radius/heading envelope and full margin
M=0.6+2sqrt(2)*sigma+0.02*age. Truth is used only to reconstruct the sensor values
in the adapter (exactly as the simulator), and for evaluation/coverage. It never
enters the estimator API, angle scores or acceptance. All unavailable checkpoints
fall back exactly. Audit selected geometry with full scans and independent chords.

Report case outcomes of the unchanged generating controller, without attributing
them to replay rules. Separate held/timeout/collision outcome groups, cohort,
final/fresh and available/fallback subsets. Report position errors, accepted true
passes, mismatches, abstentions, paired gains/losses and envelope coverage. Include
all selected checkpoints and preserve repeated-case dependence; no confidence
interval or independent-observation success-rate claim.

Save portable estimates, epoch means, all decisions/scores, case outcomes,
coverage, geometry audits and provenance. Repeat aggregation byte for byte. The
raw runs remain outside Git; document exact commands to recreate them. No new
live estimator, controller default, timer or safety guarantee is adopted.
