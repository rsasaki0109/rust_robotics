# Fixed decomposition of recovery and epoch-fused position error

Start from main 993ad62 and the pinned commanded-arc-components accounting and
components. Keep all 648 cases, the same 368 timeouts and 2,120 checkpoints.
Use one position record per checkpoint and input (actual recovery / preview),
deduplicating the identical position attached to straight and commanded paths.
Use commanded-path decisions only as fixed subgroup labels: all, optimistic,
conservative and optimistic extra arc permissions. Preserve the overlapping
all / first-usable-epoch / age-.5 / endpoint subsets and all four cohorts/arms.
Do not reselect by error, anchor contribution, movement or fusion availability.

Reload identical terminal-arc-live raw records using the SHA chain in
joint-admission-geometry/sources.json. Verify provenance, simulator and helper
fingerprints. No new trajectories, controller changes, margin tuning or sigma
reduction. The prior full physics/contact/timer audit is inherited; explicitly
report the observation-window reconstruction added in this study.

Track received and missed corrections, precision activation, previous measured
speed/applied steering, motion-filter windows, raw recovery windows and the
three-epoch fusion groups for every consumed interval. Verify sample counts,
fusion group counts and availability against the raw log. At all fixed
checkpoints reconstruct recovery xy/yaw, motion-filter xy/yaw, fused/fallback
preview xy and the previously published position-error norm within 1e-8.
Recovery count zero uses one instantaneous sensed fallback, not an empty mean.
Preview uses equal epoch weights when fusion is available, including the valid
partially filled latest epoch; otherwise use the motion filter. Do not average
all fusion samples equally or treat endpoint samples as full stationary windows.

Represent each input position error as weighted vectors: observation noise;
the received correction/precision anchor shared by each sample; drift since
that anchor; and transported historical true position minus current truth.
For raw recovery samples transport is zero and the historical-position residual
still counts. Initialize the pre-correction anchor deterministically at zero;
missed corrections do not replace it. Verify vector and signed error-direction
projection sums, keeping negative terms and zero-error cancellation.

Compute the noise model using actual sample steps and normalized weights:
Vnoise=sum(i,j) wi*wj*sigma_i*sigma_j*rho^abs(step_i-step_j), with the logged
cohort's exact rho=exp(-.02/tau) (zero for independent noise). Record the inverse
weighted unit covariance as effective noise sample count; sigma may differ.
For independent reset anchors, Vanchor=sum(anchor IDs) (sum_i wi*anchor_sigma_i)^2.
Record sqrt(Vnoise), sqrt(Vanchor) and sqrt(Vnoise+Vanchor) per axis separately.
These are unconditional model quantities excluding drift/transport uncertainty,
not calibrated confidence intervals or coverage on outcome-selected records.
Do not use sigma/sqrt(total samples) as the input-position uncertainty.

Save full accounting, checkpoint/input vectors and weights/model statistics,
epoch-group mean contributions and weights, subgroup summaries and raw/source
fingerprints. Group aggregates must reproduce every weighted input vector;
the raw hashes retain the individual observation history. Report all input
fallbacks and the predeclared 43 actual-input optimistic extra permissions.
Repeat aggregation byte for byte. If plotted, show fixed age-.5 error/component
summaries by noise cohort and input/estimator with their actual denominators.
Do not infer a new recovery, contact or held-goal result from this decomposition.
