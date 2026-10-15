# Pure Pursuit with missing external pose observations

This experiment compares the existing simulated goal slowdown/stop controller
with a candidate pause-and-recovery policy. It uses the repository's bicycle
model and Pure Pursuit implementation, fixed A*, JPS, and Theta* paths, and
paired noise/loss streams. The policy lives in an example; library behavior is
unchanged.

## Reproduce

From the repository root, with the repository's Rust toolchain and Python 3.9+:

```bash
python3 experiments/pose_observation_loss/run.py
```

No GUI or Python packages are required. Cargo builds the release example using
`--locked --no-default-features`. The default experiment runs 540 cases: three
maps × three planners × ten seeds × three availability conditions × two
controllers. It writes raw trajectories, summaries, an SVG chart, a report,
and provenance to a new ignored directory under `runs/`. Expect several minutes
and about 1 GB of trajectories. Outputs are never overwritten.

For a smaller check or a different output directory:

```bash
python3 experiments/pose_observation_loss/run.py --seeds 201 --output /tmp/pose-loss-check
python3 experiments/pose_observation_loss/verify.py /tmp/pose-loss-check
```

Run Python without `-O`: the verifier uses assertions. The verifier checks motion
and braking, pause/resume events, observation age, paired latent noise, RMS,
clearance summaries, and true goal-hold conditions. It also compares applicable
cases with the [recorded results](results/all-results.csv), and verifies that
both controllers produce identical results without observation loss. CSV
numbers are rounded; invariant checks allow the corresponding rounding error.

## Inputs and model

The checked-in fixtures were exported at commit
`594185e52c9c570090fe5a55f2b69551825fe03d` using the definitions in
[`unified_planning_benchmark.rs`](../../crates/rust_robotics_planning/benches/unified_planning_benchmark.rs).
Planning used grid spacing 1, robot radius 0.5, and heuristic weight 1.
`path-metrics.csv` records path geometry and JPS fallback metadata. These are
fixed path fixtures; this runner does not regenerate paths with future planner
versions. Path segments are subdivided to at most 0.1 without cutting corners.

Simulation timestep is 0.02 s, wheelbase 0.4, circular collision radius 0.5,
top speed 1, maximum steering 35°, and each case has a 180 s budget. Coordinates
and speed use consistent model units. Collision checks cover the swept center
segment against point obstacles, with the radius applied continuously.

The controller receives a local pose estimate with independent Gaussian noise
(σ position 0.1, yaw 2°), plus a fixed-direction position drift of 0.02/s and
yaw drift of ±0.2°/s. An external pose correction is scheduled every 5 s; its
noisy residual replaces the accumulated bias. Speed is measured exactly in
this model. An idealized precision mode activates on a received observation
when the previous filtered estimated goal distance is at most 2; it adds an
initial correction and changes noise/residual σ to position 0.03 and yaw 0.6°.
Once active, this lower white-noise level is retained through subsequent loss.
This represents loss of external pose corrections while local estimates remain
available, rather than failure of all sensors.

Availability conditions:

- `none`: all external corrections are available.
- `random`: independent probability 0.3 of unavailability per 5 s block; the
  same block availability applies to scheduled and initial precision observations.
- `blackout`: unavailable from time 45 s inclusive to 75 s exclusive.

Seeds 201–210 share latent white-noise, correction-residual, drift, and
availability streams between controllers. Missed scheduled corrections still
consume their random samples to preserve pairing after recovery. These seeds
were used in earlier experiments; they are not an unseen evaluation set.

Both arms slow near the estimated goal, filter estimated goal distance, and
latch a stop after distance ≤0.2 for 0.2 s. A latch releases after filtered
distance >0.5 for 1 s; subsequent motion is capped at 0.2. Braking uses
`clamp(-4 * speed, -1, 0)`. A successful hold requires the goal stop latch,
**true** goal distance <0.25, and **true** speed ≤0.02 for ten continuous seconds.
True pose is used for simulation and evaluation, not for policy decisions.

## Candidate pause/recovery policy

A missed scheduled correction latches a pause immediately, commanding speed
zero and steering zero with the same bounded braking response. A fresh
correction resets Pure Pursuit's forward-only path cursor while paused.
To resume, the following must hold continuously for 1 s:

- Latest received observation is at most 1.5 s old.
- Measured speed is at most 0.02.
- A forward 0.6 segment from the **estimated** pose clears known obstacles and
  map boundaries by `0.5 + 0.1 + 2*sqrt(2)*position_sigma + 0.02*observation_age`.

After resuming, speed is capped at 0.4 for 5 s. The policy uses missed-observation
metadata, measured speed, estimated pose, and the known map. It does not use
future availability or true collision clearance.

The forward segment is a simple recovery gate, not a curved stopping-trajectory
check or replanner. The uncertainty margin is heuristic and Gaussian errors
are unbounded. Zero observed contacts in these fixtures is not a safety
guarantee. Conservative waiting can prevent mission completion within 180 s.
See the [recorded report](results/report.md) for that tradeoff and validation.

## Average stationary poses before recovery

An optional comparison keeps the original pause policy as `aware` and adds an
`averaged` arm:

```bash
python3 experiments/pose_observation_loss/run.py --recovery-comparison
python3 experiments/pose_observation_loss/run.py --recovery-comparison \
  --seeds 211 212 213 214 215 216 217 218 219 220
```

The first command repeats the reference seeds 201–210. The second evaluates the
same fixed setting on new seeds. Each command runs 540 cases. The existing
command without `--recovery-comparison` still compares `baseline` and `aware`.

While paused, the new arm averages at most 25 consecutive measured poses
(0.5 s): arithmetic means for x/y and a circular mean for yaw. A correction,
missed scheduled observation, measured speed above 0.02, or observation age
above 1.5 s clears the window. Only stationary samples within the current
correction epoch are admitted. At least ten samples are required before
starting the existing one-second confirmation. The averaged pose is used only
for the recovery clearance/boundary check; driving and goal control still use
the original pose estimates. The distance margin, observation age limit,
braking response, speed cap, and true ten-second hold criterion are unchanged.

This targets a failure in the original recovery check: one noisy clearance
estimate can reset the one-second confirmation, leaving the vehicle waiting
through another observation cycle. Averaging reduces independent white noise;
it does not eliminate the shared correction residual or accumulated bias, and
the uncertainty margin is not reduced. The verifier checks sample provenance,
window resets, warmup, and the circular mean against logged measurements.

See [the averaging comparison](results/recovery-averaging/report.md), which
separates the reference seeds from the new-seed evaluation. All maps remain the
same. Independent noise every 0.02 s is a model assumption; correlated sensor
errors could reduce the benefit of averaging.

## Temporally correlated local pose noise

Use `--noise-correlation-time SECONDS` to keep each sample's marginal σ fixed
while varying temporal persistence. Zero is the original independent-noise
model and remains the default. For example, with the previous evaluation seeds:

```bash
python3 experiments/pose_observation_loss/run.py --recovery-comparison \
  --noise-correlation-time 0.2 --seeds 211 212 213 214 215 216 217 218 219 220
python3 experiments/pose_observation_loss/run.py --recovery-comparison \
  --noise-correlation-time 1 --seeds 211 212 213 214 215 216 217 218 219 220
```

For each normalized x/y/yaw component, `z[1] = epsilon[1]` and subsequent samples
follow `z[t] = rho*z[t-1] + sqrt(1-rho²)*epsilon[t]`, where
`rho = exp(-0.02/correlation_time)` and epsilon is independent standard normal.
This starts with stationary unit variance. The current position/yaw σ scales
z; normalized state persists when the idealized precision mode changes σ.
The same three Gaussian innovations are consumed at each step in every arm.
The zero-time branch uses exactly the legacy independent samples.

Only the local measurement-noise component changes. External correction
residuals, drift, availability, maps, policies, recovery margins, and success
criteria are fixed. Temporal correlation is not passed to the control policy;
the policy continues to use its original instantaneous σ-based margin. This
experiment measures sensitivity of the fixed policy rather than adapting it
to a known noise model.

The verifier checks the AR recurrence, stationary initialization, paired
innovations, and the existing motion/recovery/hold invariants. The optional
standard-library analysis below compares completed runs, including a white
run, and checks a fixed 50 s open-map noise prefix for all seeds. It checks
innovation pairing across correlation times and reports empirical mean,
variance, lag-one correlation, and the theoretical SD of a 25-sample mean.
Broad statistical sanity bounds are not confidence intervals.

```bash
python3 experiments/pose_observation_loss/compare_correlations.py \
  WHITE_RUN CORRELATED_RUN_02 CORRELATED_RUN_1 --output NEW_ANALYSIS_DIRECTORY
```

See the [recorded correlation study](results/correlated-noise/report.md).
The same maps and previously evaluated seeds are reused; this is a sensitivity
study, not an unseen evaluation or a fitted sensor-noise model.
