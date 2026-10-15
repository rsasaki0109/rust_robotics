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

## Always-on stopping-preview comparison

The optional `--stopping-preview-comparison` compares `averaged` recovery with
`guarded`: the same controller plus a stopping-preview brake during ordinary
tracking. It is mutually exclusive with the other comparison switches.

```bash
python3 experiments/pose_observation_loss/run.py --stopping-preview-comparison \
  --noise-correlation-time 1 --seeds 221 222 223 224 225 226 227 228 229 230
python3 experiments/pose_observation_loss/run.py --stopping-preview-comparison \
  --seeds 221 222 223 224 225 226 227 228 229 230
```

Each command runs 540 cases. The preview starts at the measured pose and speed,
holds the proposed steering, applies the proposed speed controller for 0.2 s,
then applies the existing `clamp(-4*v, -1, 0)` brake until speed is at most 0.001
(up to 4 s). The actual simulator responds to braking immediately; the initial
0.2 s of nominal motion is a conservative assumption of the preview rather
than added actuator latency in the simulation.

A brake triggers when the predicted center path comes within
`0.5 + 2*sqrt(2)*position_sigma + 0.02*observation_age` of point obstacles or map
boundaries. The optimized check returns a conservative clearance lower bound:
near obstacles use all swept segments, while distant obstacles use the triangle
inequality with total predicted travel. While the guard is active, target speed
is zero and steering is held at its triggering value. The existing loss pause
still takes priority and requires zero steering. On a received external
correction while either pause is active, the path cursor resets as before.

Release requires measured speed at most 0.02 and predicted clearance exceeding
the trigger margin by another 0.05, continuously for 0.5 s. Prediction on a
release probe uses the nominal command that would apply after release. Goal
hold conditions and the 180 s budget are unchanged. Guard time is logged
separately and can overlap the existing observation-loss pause time.

The preview uses estimated pose and the known map. True position is used for
physics and evaluation only. It is a fixed-steering, deterministic prediction
with a heuristic positional margin, not a reachable-set or probabilistic safety
guarantee. It does not account for all future steering changes or propagate yaw
uncertainty through the curve. Conservative stopping may also prevent completion
on paths planned using only the nominal radius of 0.5.

The runner performs the regular trajectory checks, then audits exact stopping
geometry at up to eight deterministic checkpoints per guarded case: the first
step, first trigger/release, contact endpoint if any, and fixed later intervals.
This independently checks the optimized lower bound and threshold classification
using exact obstacle geometry. All contact rows are independently recomputed.
The audit can also be run directly:

```bash
python3 experiments/pose_observation_loss/verify_stopping_preview.py RUN_DIRECTORY
```

See the [stopping-preview report](results/stopping-preview/report.md) for both
contacts and mission completion. The guard is an experiment option; the default
runner and library controller behavior are unchanged.

To combine the reference and two new-seed runs into case/summary CSVs and the
recorded plot (optional matplotlib dependency for `--plot`):

```bash
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --reference REFERENCE_RUN --white WHITE_RUN --correlated CORRELATED_RUN \
  --output NEW_ANALYSIS_DIRECTORY --plot
```

## Stationary correction consistency before restart

The optional `--restart-consistency-comparison` pairs the previous `guarded`
controller with `consistent`. All comparison switches are mutually exclusive.
The new candidate keeps the driving commands and trigger rule, and adds a
confirmation gate while the stopping guard is already active:

- Retain up to 25 stationary measured poses (speed at most 0.02). At a received
  scheduled correction, retain the pre-correction mean only if all 25 samples
  exist, then start a new window. Position uses arithmetic means; heading uses
  circular means.
- Require 10–25 post-correction samples, correction age at most 1.5 s, position
  difference at most 0.2, and wrapped heading difference at most 5 degrees.
- When these checks pass, use the post-correction mean pose in the stopping
  preview. Release still requires clearance above the existing margin plus 0.05
  continuously for 0.5 s. Speed, radius, margins and the mission budget stay fixed.

Motion, a missed scheduled correction, a terminal precision activation, or an
inactive guard invalidates the comparison. Stationary windows may accumulate
while the correction is stale, so the next received correction can have a full
pre-window; release cannot use a stale comparison. A terminal precision
activation alone does not qualify as the scheduled correction. After release,
driving and previews again use the instantaneous measured pose. The new option
combines confirmation with release-only averaging; its comparison does not
isolate these two effects. The observation-loss pause remains a separate gate.

This compares observations, not their error against truth. A shared bias can
pass, and a useful correction of a bad earlier estimate can fail. In particular,
averaging correlated samples does not make them independent. No covariance or
probabilistic accuracy claim follows from these thresholds.

```bash
# Known counterexample from the previous study, then separate fixed new seeds.
python3 experiments/pose_observation_loss/run.py --restart-consistency-comparison \
  --noise-correlation-time 1 --seeds 227 --output /tmp/restart-reference
python3 experiments/pose_observation_loss/run.py --restart-consistency-comparison \
  --noise-correlation-time 0 --seeds 231 232 233 234 235 236 237 238 239 240 \
  --output /tmp/restart-white
python3 experiments/pose_observation_loss/run.py --restart-consistency-comparison \
  --noise-correlation-time 1 --seeds 231 232 233 234 235 236 237 238 239 240 \
  --output /tmp/restart-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --restart-consistency --reference /tmp/restart-reference \
  --white /tmp/restart-white --correlated /tmp/restart-correlated \
  --output /tmp/restart-analysis --plot
```

The runner reconstructs pre/post windows, freshness, heading wrap, preview pose
and release evidence from logged measurements, in addition to the existing
physics, noise pairing, hold and contact checks. The exact geometry audit checks
the candidate variant (`consistent` in this mode). New telemetry includes
`consistency_ready`, `pre_sample_count`, `stationary_sample_count`,
`consistency_age`, `position_delta`, `yaw_delta_rad`, and `preview_pose_{x,y,yaw}`.
Unavailable ages/deltas use -1; a missing pre-window has sample count 0.
The simulator's new final optional argument is `CONSISTENT_RESTART`, requiring
the earlier `STOPPING_GUARD` and `AVERAGED_RECOVERY` options. Omitted arguments
retain the previous behavior.

See the [restart consistency report](results/restart-consistency/report.md) for
the known counterexample, new-seed comparison and completion cost.

## Planning clearance comparison

Generate and audit paths before comparing the same `guarded` controller with
two planning radii at a fixed grid resolution of 0.125:

```bash
python3 experiments/pose_observation_loss/prepare_clearance_paths.py \
  --output /tmp/planning-clearance-inputs
python3 experiments/pose_observation_loss/run.py \
  --path-clearance-comparison /tmp/planning-clearance-inputs \
  --noise-correlation-time 1 --seeds 227 234 --output /tmp/clearance-reference
python3 experiments/pose_observation_loss/run.py \
  --path-clearance-comparison /tmp/planning-clearance-inputs \
  --seeds 241 242 243 --output /tmp/clearance-white
python3 experiments/pose_observation_loss/run.py \
  --path-clearance-comparison /tmp/planning-clearance-inputs \
  --noise-correlation-time 1 --seeds 241 242 243 --output /tmp/clearance-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --path-clearance --reference /tmp/clearance-reference \
  --white /tmp/clearance-white --correlated /tmp/clearance-correlated \
  --output /tmp/clearance-analysis --plot
```

All output directories must be new. Cargo exports paths using the existing A*,
JPS and Theta* implementations and the original obstacle CSV and endpoints.
No GUI is needed. Preparation records planner source hashes and exports four
sets: resolution 1 with planning radius 0.5 or 0.9, and resolution 0.125 with
radius 0.5 or 0.99. Planning failures remain recorded failures; simulation does
not substitute a nominal path for them.

The coarse sets demonstrate that node inflation does not imply the same
continuous segment clearance. On these integer-coordinate maps, increasing
radius from 0.5 to 0.9 leaves occupancy unchanged. The finer padded set uses
0.99 (above 0.9 plus half the cell diagonal) and is admitted to the comparison
only after every segment clears all original point obstacles and map boundaries
by at least 0.9. This audit concerns these exported paths, not a general theorem
about all future planner outputs. `path-audit.csv` records both planning radius
and independently measured clearance, including coarse paths that miss their
requested radius.

For simulation, `guarded` uses `fine-nominal` (planning radius 0.5), and
`clearance` uses `fine-padding` (0.99). Both use resolution 0.125 and exactly the
same stopping guard, observation-loss recovery, noise and goal controller. The
stationary correction consistency option is off. The vehicle's physical radius
stays 0.5; padding affects planning only. Inputs are copied into each run and
hashed separately by variant. Verification recomputes path geometry and checks
these hashes. Golden results from different path fixtures are not compared.
The trajectory, paired-noise, hold/contact and exact preview audits still run.

The fine-grid reference changes the paths from earlier studies, so its results
cannot be treated as replays of those coarse paths. In particular, improving
over the fine nominal arm alone does not establish improvement over the original
resolution-1 fixtures. Larger path clearance does not bound pose error or prevent
Pure Pursuit from leaving the planned centerline. See the
[planning clearance report](results/planning-clearance/report.md) for outcomes
and remaining contacts.

## Anticipatory speed comparison on padded paths

This comparison holds the fine-grid padded paths fixed and pairs the previous
`clearance` controller with `paced`, which additionally caps speed using an
estimated geometric probe:

```bash
python3 experiments/pose_observation_loss/run.py \
  --anticipatory-speed-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --noise-correlation-time 1 --seeds 234 243 --output /tmp/speed-reference
python3 experiments/pose_observation_loss/run.py \
  --anticipatory-speed-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 244 245 246 --output /tmp/speed-white
python3 experiments/pose_observation_loss/run.py \
  --anticipatory-speed-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --noise-correlation-time 1 --seeds 244 245 246 --output /tmp/speed-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --anticipatory-speed --reference /tmp/speed-reference \
  --white /tmp/speed-white --correlated /tmp/speed-correlated \
  --output /tmp/speed-analysis --plot
```

Both arms use the exact same `fine-padding` fixtures (resolution 0.125, planning
radius 0.99) copied and hashed per run. Existing comparison modes and omitted
simulator arguments retain their previous behavior. The simulator's new final
optional Boolean argument is `ANTICIPATORY_SPEED`, requiring the stopping guard
and averaged recovery, with correction-consistency restart disabled.

At each decision, `paced` takes the nominal steering from the measured pose and
probes a constant-curvature arc of length 2.0, represented by 0.05-length arc
intervals joined by straight chords. Curvature is `tan(steering)/0.4`; zero
curvature gives a straight line. It checks the original point obstacles and
map boundaries against `0.5 + 0.1 + 2*sqrt(2)*position_sigma + 0.02*observation_age`.
If the initial pose is inside that margin, the free distance D is zero. At the
first unsafe chord, D is the arc length to its start; otherwise D is 2.0.

The speed cap is `clamp(sqrt(0.0225 + D) - 0.15, 0, 1)`, solving
`0.3*v + v*v/(2*0.5) <= D` before the global cap of 1. This reserves 0.3 s of
travel and uses design deceleration 0.5. The nominal goal/recovery target is
capped before the existing stopping preview. When measured speed exceeds the
cap by more than 1e-9, the controller uses the existing bounded brake
`clamp(-4*v, -1, 0)`; otherwise it retains the usual acceleration. Other stop
conditions and frozen steering during a guard remain in force. This comparison
combines target-speed capping and extra braking; it does not isolate their effects.

The probe uses estimated pose and the known map, never true pose or future
observations. Fixed steering, chord geometry and the heuristic noise margin
do not bound future closed-loop motion or pose error. The 0.3 s term is a design
reserve; no physical actuator delay is added. Collision radius, braking limit,
goal hold criterion and the 180 s budget are unchanged.

New trajectory fields are `speed_free_distance`, `speed_cap`, `speed_margin`,
`speed_braking` and `uncapped_target_speed`. The generic verifier checks the cap,
target and brake response at every step. The geometry auditor recomputes the
full obstacle scan for the speed probe at the same fixed checkpoints as the
stopping preview. `control-events.csv` includes speed-limited and extra-braking
times outside goal stops, loss pauses and stopping guards; these exclude time
already stopped by another condition. Applicable old padded-path summaries are
also compared for the reference arm.

See the [anticipatory speed report](results/anticipatory-speed/report.md) for
contact outcomes and mission-completion costs.

## Diagnose lost holds and interacting recovery gates

The offline diagnostic below reads the recorded speed-cap study's two new-seed
runs, selects cases held by `clearance` but not by `paced`, and examines
`150 < time <= 180`. It does not change or run a control policy:

```bash
python3 experiments/pose_observation_loss/analyze_recovery_stalls.py \
  --white /tmp/speed-white --correlated /tmp/speed-correlated \
  --output /tmp/recovery-stall-analysis --plot
```

The raw run directories must contain their trajectories and copied planning
inputs. If needed, regenerate `/tmp/speed-white` and `/tmp/speed-correlated`
using the corresponding commands above with seeds 244–246 and the simulator
version at commit `07f0ea4` (before the joint-recovery option). The source hash
is intentionally pinned; a newer simulator must not be substituted. The diagnostic
requires the recorded result rows, simulator hash and paired path hashes to
match; it is a post-hoc analysis of that frozen dataset. Python's standard
library suffices without `--plot`; plotting requires Matplotlib.

Outputs include case-level command durations and actual travel, selected
checkpoints, source trajectory hashes, and an optional stacked duration plot.
Each of the 1,500 intervals per case belongs to one exclusive command state:
both loss pause and guard, loss only, guard only, goal stop, zero speed cap,
speed-limited target, or ordinary target. Stop states take priority over speed
states. A guard includes braking time, so these are command durations rather
than stationary durations. The separate moving time uses the pre-update
measured speed threshold 0.02. Travel sums the true logged center displacement
over each interval. `near_stationary` means travel below 0.1 during this window.
The plot stacks totals; it is not a chronological timeline.

The script distinguishes guard release while a loss pause remains from actual
movement, and loss recovery while a guard remains. Checkpoints include every
release, stationary observations aged 0.5 s, and the final decision.
`guard_ready` and `recovery_ready` are instantaneous predicates before the
0.5 s and 1 s continuous-evidence requirements; they do not imply release.
Recovery line geometry is independently recomputed at every checkpoint, and
the physical response is checked across all 30,000 selected intervals.

Truth is used only for offline attribution: minimum center clearance and the
forward 0.6 line from the true pose are compared with the existing margins,
keeping sigma fixed and using zero observation age. This tests necessary
geometric conditions at the final pose, not a deployable localization method
or a prediction that changing margins would recover a mission. It also does
not establish safety of an alternative steering or restart policy. See the
[recovery-stall diagnosis](results/recovery-stalls/report.md).

## Compare simultaneous loss and stopping-guard recovery

`--joint-recovery-comparison FIXTURES` compares the existing `paced` arm with
opt-in `joint`, using the same pinned fine-padding paths, noise innovations,
corrections and speed-cap policy. For example:

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --joint-recovery-comparison "$fixtures" --seeds 244 245 \
  --noise-correlation-time 1 --output /tmp/joint-reference
python3 experiments/pose_observation_loss/run.py \
  --joint-recovery-comparison "$fixtures" --seeds 245 \
  --noise-correlation-time 0 --output /tmp/joint-known-white
python3 experiments/pose_observation_loss/run.py \
  --joint-recovery-comparison "$fixtures" --seeds 247 248 249 \
  --noise-correlation-time 0 --output /tmp/joint-white
python3 experiments/pose_observation_loss/run.py \
  --joint-recovery-comparison "$fixtures" --seeds 247 248 249 \
  --noise-correlation-time 1 --output /tmp/joint-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --joint-recovery --reference /tmp/joint-reference \
  --known-white /tmp/joint-known-white \
  --white /tmp/joint-white --correlated /tmp/joint-correlated \
  --output /tmp/joint-summary --plot
```

The appended simulator flag `JOINT_RECOVERY` defaults to false and requires
`ANTICIPATORY_SPEED=true`. During a loss pause, `joint` probes the command
that would follow release: the ordinary goal command capped at 0.4, raw sensed
Pure Pursuit steering, then the existing arc speed cap. Actual paused target
and steering remain zero. `nominal_target_speed`, `nominal_steering` and
`uncapped_target_speed` describe this prospective command during such pauses;
their meaning differs from the zero-command probe in `paced`.

The existing averaged forward-0.6 recovery line, map bounds, sample count,
stationary speed and observation age requirements must all pass. In addition,
the prospective target must be positive and its stopping-preview clearance
must exceed the existing guard release margin. Both sets of conditions must
hold for the same continuous second, using `recovery_evidence`. Then the loss
pause and any active guard clear together. While a loss pause remains, the
independent guard-release timer is zero. Guard entry uses the prospective
probe; a guard by itself retains its original 0.5-second release rule. The
five-second recovery speed limit remains. Goal stops retain priority.

This comparison jointly changes the paused probe command and synchronizes
release confirmation. It does not isolate either change, replace the forward
line with a curved recovery path, change uncertainty margins, or use true pose
for decisions. Atomic release does not imply subsequent progress or a safe
future closed-loop trajectory.

The verifier reconstructs the common timer and post-release command at every
step, checks atomic release, and compares all trajectory columns before the
first missed correction. No-loss trajectories must match in full. Applicable
`paced` result rows must match the recorded speed-cap study. The geometry
auditor scans the full obstacle set for both the speed probe and recovery line
at its fixed checkpoints and every joint loss resume, and independently checks
stopping-preview clearance. Original default results are also regression
checked separately with seed 201. See the [joint recovery report](results/joint-recovery/report.md).

## Audit forward recovery arcs at the frozen stalls

The arc diagnostic below selects the eight near-stationary cases from the
recorded recovery-stall analysis and reads their `joint` trajectories in the
two known-case joint-recovery runs. It is an offline geometry study; no simulator
or controller policy is changed:

```bash
python3 experiments/pose_observation_loss/analyze_recovery_arcs.py \
  --known-white /tmp/joint-known-white --reference /tmp/joint-reference \
  --output /tmp/recovery-arcs --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_recovery_arc_geometry.py
```

Use the `--seeds 245 --noise-correlation-time 0` and
`--seeds 244 245 --noise-correlation-time 1` joint-comparison commands above to
regenerate missing raw runs. The diagnostic requires their result rows,
planning inputs and simulator hash to match the recorded study. Its simulator
version is available at commit `e1821b0`; a future changed simulator is not a
substitute for these frozen inputs. Python's standard library suffices without
`--plot`; plotting requires Matplotlib. Raw trajectory hashes, helper/source
hashes and the selection hash are saved in `sources.json`.

The probe follows the vehicle center forward for arc length 0.6 with fixed
steering from -35 to 35 degrees in one-degree increments and wheelbase 0.4.
It computes exact point-to-circular-arc clearance, including endpoints, radial
projections and interior extrema against all four map boundaries. This spatial
arc is a counterfactual kinematic curve, not a new rollout of the simulator's
0.02-second integration, braking, controller or release timers.

Checkpoints are before the final t=180 physical update and each stationary
observation aged exactly 0.5 s at t=150.5, 155.5, ..., 175.5. A missed correction
or motion excludes that fresh checkpoint. Each includes true pose for offline
evaluation and reconstructed raw observed pose. Recovery means are included
only when at least ten same-epoch stationary samples exist; no raw-pose fallback
is labeled as an average. The window is independently reconstructed. The
physical updates of all 12,000 final-window intervals and final summary values
are also checked.

`checkpoints.csv` separates physical radius 0.5, radius plus buffer 0.6, and the
existing recovery margin `0.6 + 2*sqrt(2)*sigma + 0.02*observation_age`.
The age-zero version keeps sigma fixed and removes only observation age. Truth
is never treated as a realizable observation. Sigma limits in the CSV are
algebraic thresholds for the same fixed pose, not evidence that localization
can achieve those values or that lowering the configured sigma is safe.

A curve includes its starting point, so starting clearance already within the
margin excludes every curve containing that point. For the remaining cases,
the maximum over continuous fixed steering is bounded above by the grid maximum
plus `0.6^2/2 * sec(35 deg)^2/0.4 * radians(0.5)` (about 0.005853).
This follows from the position sensitivity to curvature, at most `s^2/2`, and
the maximum curvature derivative over the steering interval. The upper bound
also cannot exceed starting clearance; a numeric allowance of 1e-9 is added.
`all_fixed_steering_blocked` requires this upper bound to be no larger than the
margin. `grid_pass` requires a grid clearance greater than the margin plus
1e-9; all other cases are `unresolved`. Thus a failed grid alone is not used as
a proof about intermediate steering angles.

`steering-grid.csv` contains every tested angle. `geometry-audit.csv` compares
all final-pose grid arcs and straight/best/current-steering arcs at the fresh
checkpoints with independent 120-chord geometry. The analytic/polyline gap must
not exceed the circular sagitta plus 1e-9. The geometry tests exercise analytic
fixtures, wall extrema, clockwise and counterclockwise wrap, near-straight
numerics, deterministic random curves and the continuous steering bound.

These checks do not cover varying steering, reversing, shorter moves, replanning,
observation improvements or successful recovery. See the
[forward-arc feasibility report](results/recovery-arcs/report.md).

## Trace clearance loss before the final stops

This offline diagnostic follows the same eight frozen `joint` cases through all
180 seconds. It reads exactly the raw trajectory bytes hashed by the arc study:

```bash
python3 experiments/pose_observation_loss/analyze_stop_entry.py \
  --known-white /tmp/joint-known-white --reference /tmp/joint-reference \
  --output /tmp/stop-entry --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_stop_entry_geometry.py
```

The two raw runs and their frozen simulator version are described in the arc
study above. Python's standard library suffices without plotting. The script
checks all 72,000 physical intervals, strong-brake decisions, and reconstruction
of the observed pose. It compares the raw inputs and metadata with the frozen
arc study and records trajectory and helper hashes in `sources.json`.

The fixed diagnostic threshold is the minimum recovery margin at sigma=0.1 and
observation age zero: `0.6 + 2*sqrt(2)*0.1`. This is a necessary center-clearance
condition, not the complete recovery-line or guard condition. A down-crossing
uses true center clearance before and after the physical update. The persistent
crossing is the final down-crossing only if clearance remains below this
threshold at the end; earlier temporary crossings are counted separately.

The final stationary interval begins at the first post-update state after the
last speed greater than 0.02 and ends at t=180. Its duration is `180 - start`.
The terminal brake episode is the contiguous run of strong-brake commands that
includes this onset, combining the speed cap, loss pause, guard and goal stop.
It is the final such episode, not the first brake or guard in the trial. The
script compares its start with the persistent center-clearance crossing; these
may be far apart when earlier stop/release episodes already lost clearance.

`checkpoints.csv` selects the first and persistent center crossings, one second
before the persistent crossing, the terminal brake start and the final stationary
onset, merging coincident events. Each independently recomputes the original
0.2-second nominal-command stopping preview, both at observed pose and offline
true pose. The logged preview lower bound and threshold classifications are
checked against full geometry. `guard_margin` is the existing trigger margin;
the diagnostic recovery threshold is 0.1 larger at the same observation age.

A separate immediate-brake counterfactual starts from the pre-update pose and
measured speed, holds that step's actual steering, and applies the existing
`clamp(-4*v, -1, 0)` brake with DT=0.02. Its swept clearance is computed for both
true and observed initial pose. It stops its rollout at speed <=0.001; the omitted
strong-brake tail travels at most `abs(v_end)/4`. The `*_immediate_brake_lower`
fields subtract that tail length and therefore bound clearance if the same
brake continues. These bounds do not cover later acceleration or altered
steering. `matching_actual_brake_steps` reports how many forecast steps match
the logged trajectory before the actual steering or brake mode changes; no
longer match is assumed.

Center clearance uses exact expanding spatial cells; `center-audit.csv` compares
its answers with full obstacle scans at one-second intervals and every selected
event. `cases.csv` records the episode times and clearances. `traces.csv` keeps
0.1-second samples, every selected event and full-resolution samples within five
seconds of the final stationary onset. In the figure, time zero is that post-
update onset; clearance and speed plotted at each logged decision are pre-update.
The fixed axes show clearance 0..2 and speed 0..1.1; larger values are clipped.

The seven geometry tests check indexed versus full-scan distances, empty-map
boundaries, cell-corner search, the analytic linear-brake tail, stationary
initial-point clearance, swept segment interiors and mirrored turning brakes.
No control setting or simulator source is changed by this analysis. See the
[stop-entry diagnosis](results/stop-entry/report.md).

## Compare stopping-guard and recovery distance buffers

This paired study changes the stopping-preview margin of `joint`, the policy
used by the frozen diagnoses. `buffered` adds 0.1 to that guard margin:

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --guard-buffer-comparison "$fixtures" --seeds 244 245 \
  --noise-correlation-time 1 --output /tmp/buffer-reference
python3 experiments/pose_observation_loss/run.py \
  --guard-buffer-comparison "$fixtures" --seeds 245 \
  --noise-correlation-time 0 --output /tmp/buffer-known-white
python3 experiments/pose_observation_loss/run.py \
  --guard-buffer-comparison "$fixtures" --seeds 250 251 252 \
  --noise-correlation-time 0 --output /tmp/buffer-white
python3 experiments/pose_observation_loss/run.py \
  --guard-buffer-comparison "$fixtures" --seeds 250 251 252 \
  --noise-correlation-time 1 --output /tmp/buffer-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --guard-buffer --reference /tmp/buffer-reference \
  --known-white /tmp/buffer-known-white --white /tmp/buffer-white \
  --correlated /tmp/buffer-correlated --output /tmp/buffer-summary --plot
```

The appended simulator flag `RECOVERY_GUARD_BUFFER` defaults to false and
requires `JOINT_RECOVERY=true`. Both arms retain the same paths, speed cap,
recovery line, pose averaging, common recovery timer, braking and goal holds.
In `buffered`, stopping-preview entry uses
`0.6 + 2*sqrt(2)*sigma + drift_speed*observation_age`, matching the existing
speed-probe and recovery buffer. Release uses that margin plus the unchanged
0.05 hysteresis. This raises both entry and release thresholds by 0.1; the
shared recovery gate also uses this higher guard release threshold. The
preview's distant-point pruning uses the new margin too. Simply comparing
old logged lower bounds against a larger threshold is not this experiment.
The recorded `preview_margin` exposes the actual threshold; no true pose
enters a control decision. The library controller is unchanged.

The verifier independently reconstructs the margin at every step, all guard
state transitions and both release timers, then checks physical responses and
paired noise. Applicable `joint` rows match the recorded joint-recovery study.
The geometry auditor checks every `buffered` guard entry, guard release and loss
resume, plus its fixed checkpoints, using exact stopping geometry and full
obstacle scans for the speed probe and recovery line. The original seed-201
default pair is checked separately as a regression.

For this comparison, `control-events.csv` includes `total_wait_time_s`, the
union of loss-pause and stopping-guard command durations, and
`stationary_wait_time_s`, the same union restricted to pre-update speed <=0.02.
The first includes braking; the second is a measured-speed classification.
Neither sums overlapping waits or includes goal stops as a separate wait.
The summary reports their means over each mode/cohort. Durations are observed
until termination, including the 10-second goal hold for successful trials.

These fixed margins remain heuristics for noisy poses. A larger margin can
increase waiting and prevent completion; evaluate contacts and holds together.
See the [guard-buffer comparison](results/guard-buffer/report.md).

## Compare a motion-dependent entry reserve with unchanged release

The previous study raised both guard thresholds. This study compares `joint`
with `entry`, which only changes the entry rule. Let
`base = 0.5 + 2*sqrt(2)*sigma + drift_speed*observation_age`. The new entry
threshold is `base + 0.1*min(abs(measured_speed)/0.4, 1)`; both the independent
guard release and joint loss/guard release still use `base + 0.05`.
A flat entry increment of 0.1 would exceed the unchanged release threshold
at rest. Tapering the increment avoids that inversion: at release-eligible
speed <=0.02 the release threshold exceeds entry by at least 0.045 for the
same observation. This does not guarantee release under changing noisy poses.

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --entry-buffer-comparison "$fixtures" --seeds 250 251 252 \
  --noise-correlation-time 1 --output /tmp/entry-reference
python3 experiments/pose_observation_loss/run.py \
  --entry-buffer-comparison "$fixtures" --seeds 250 251 252 \
  --noise-correlation-time 0 --output /tmp/entry-known-white
python3 experiments/pose_observation_loss/run.py \
  --entry-buffer-comparison "$fixtures" --seeds 253 254 255 \
  --noise-correlation-time 0 --output /tmp/entry-white
python3 experiments/pose_observation_loss/run.py \
  --entry-buffer-comparison "$fixtures" --seeds 253 254 255 \
  --noise-correlation-time 1 --output /tmp/entry-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --entry-buffer --reference /tmp/entry-reference \
  --known-white /tmp/entry-known-white --white /tmp/entry-white \
  --correlated /tmp/entry-correlated --output /tmp/entry-summary --plot
```

The appended simulator flag `MOTION_ENTRY_BUFFER` defaults to false, requires
`JOINT_RECOVERY=true`, and cannot be combined with `RECOVERY_GUARD_BUFFER`.
All other control rules and paired inputs are unchanged. Distant-obstacle
pruning uses the actual entry threshold. The trajectory adds
`guard_release_margin` and `guard_entry_extra`; `preview_margin` remains the
entry threshold. Initial rows have zero sentinels for these decision fields.
Verifiers accept older trajectories without the new fields by using their
original `preview_margin + 0.05` release rule.

The fixed design is evaluated on known seeds 250–252 and previously unused
seeds 253–255, with both noise models: 648 comparison cases. Every step's
thresholds, guard transitions and both release timers are checked; geometry
audits cover all candidate guard entries, releases and loss resumes, plus
fixed checkpoints. Known `joint` summaries match the previous guard-buffer
study, and the original seed-201 default pair is checked separately.
Wait duration uses the union of loss and guard waits as defined above.

Comparing `entry` against the earlier `buffered` arm changes both the
speed dependence of entry and the release threshold. It cannot isolate the
effect of release alone. See the [entry-buffer comparison](results/entry-buffer/report.md)
for results, limitations and the decision to keep this option disabled by default.

## Diagnose the entry-buffer study's lost holds

This offline analysis selects all ten previously unused-seed pairs in the
recorded entry-buffer study where `joint` held the goal and `entry` timed out.
It changes no policy and provides no additional validation seeds. Use the raw
white/correlated runs from the preceding section, or recreate just those two
runs with seeds 253–255. Their simulator SHA-256 must match the recorded study
(the engine in commit `0262ea4`). Later simulator changes require regenerating
these historical tracks with that engine before running the current analysis.

```bash
python3 experiments/pose_observation_loss/analyze_entry_cycles.py \
  --white /tmp/entry-white --correlated /tmp/entry-correlated \
  --output /tmp/entry-cycles --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_entry_cycles.py
```

Within each pair, wait and travel are compared through the successful `joint`
run's termination time, so both arms have the same observation duration.
The candidate's final 30 seconds are analyzed separately. A completed release
bout runs from a guard release decision to its next guard entry decision.
Distance includes the release command interval and excludes the new entry's
braking interval. Initial entries and final releases without a later entry
are omitted from completed bouts. A short re-entry requires a positive release
command, no intervening loss pause, lag <=1 second and travel <0.1.

Exact stopping geometry is recomputed at every selected candidate guard entry,
guard release and loss resume, and at each final decision. A reserve-only entry
means that the exact clearance at that candidate state passes the original
entry threshold but fails the added reserve. It does not recreate `joint`'s
diverged trajectory. Tail gate-failure durations use the recorded predicates
while loss recovery is pending; they overlap and must not be summed. The
continuous-ready duration is restricted to the final 30-second window.

The script validates pinned summaries and fixture hashes, reconstructs all
selected guard/recovery transitions, checks physical integration for both
arms, and records raw-track and helper hashes. Optional plotting requires
Matplotlib; the analysis and seven interval/censoring tests use the standard
library. See the [entry-cycle diagnosis](results/entry-cycles/report.md).

## Decompose the inputs behind short guard re-entries

This second offline diagnosis freezes all 27 short re-entries selected above.
Use the same historical raw runs; their candidate track hashes must match the
recorded entry-cycle diagnosis. New runtime timestamps and commit metadata are
accepted, while simulator, seeds, fixtures and recorded track contents are pinned.

```bash
python3 experiments/pose_observation_loss/analyze_reentry_inputs.py \
  --white /tmp/entry-white --correlated /tmp/entry-correlated \
  --output /tmp/reentry-inputs
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_reentry_inputs.py
```

For each release/entry pair, the script evaluates all 16 combinations of the
release and entry physical pose, observation error (noise plus drift), initial
speed and planned target/steering. It averages each factor's clearance change
over all 24 replacement orders (Shapley allocation), then adds the independent
base-margin and speed-reserve threshold changes. The six contributions sum to
the change in exact clearance minus entry threshold. Initial-speed geometry
and its speed-dependent reserve are separate terms.

The analysis also freezes one input at its release value while retaining the
other entry inputs. These are synthetic endpoint evaluations, not reachable
trajectories or runnable policies. The 432 forecasts include 54 recorded
endpoints, which are checked against the previous exact audit. No controller
change, new validation seeds or success-rate claim follows from this analysis.
The [input diagnosis](results/reentry-inputs/report.md) records the results,
limitations and the next candidate to evaluate.

## Compare motion-compensated stopping-preview observations

This study compares the original `joint` policy with `filtered`. Only the
stopping-preview pose changes: both guard entry and release, including the
joint release gate, use the same motion-compensated local pose average.
The nominal steering/goal command, arc speed probe, stationary recovery line,
guard distance thresholds and release timers retain their previous rules.
The speed-dependent entry reserve is disabled in both arms.

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --motion-filter-comparison "$fixtures" --seeds 253 254 255 \
  --noise-correlation-time 1 --output /tmp/filter-reference
python3 experiments/pose_observation_loss/run.py \
  --motion-filter-comparison "$fixtures" --seeds 253 254 255 \
  --noise-correlation-time 0 --output /tmp/filter-known-white
python3 experiments/pose_observation_loss/run.py \
  --motion-filter-comparison "$fixtures" --seeds 256 257 258 \
  --noise-correlation-time 0 --output /tmp/filter-white
python3 experiments/pose_observation_loss/run.py \
  --motion-filter-comparison "$fixtures" --seeds 256 257 258 \
  --noise-correlation-time 1 --output /tmp/filter-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --motion-filter --reference /tmp/filter-reference \
  --known-white /tmp/filter-known-white --white /tmp/filter-white \
  --correlated /tmp/filter-correlated --output /tmp/filter-summary --plot
```

Before adding the latest local observation, the filter transports each stored
pose through the previous applied command interval using its own estimated yaw,
the previous measured speed, and the actual previous steering command. It uses
the simulator's discrete bicycle motion model. It keeps at most 25 observations
(about 0.5 s), averages aligned positions and uses a circular heading mean.
Every scheduled external correction, including a missed one, and every terminal
precision activation resets the window. Local pose observations continue during
external correction loss, as in the original model. No true position or yaw is
available to this filter. The experiment assumes measured speed and the applied
steering are exact; it does not add wheel-slip or actuator-model error.

The appended `MOTION_FILTER` simulator flag defaults to false, requires joint
recovery, and excludes the two optional guard-buffer flags. The trajectory adds
`motion_filter_sample_count`; `preview_pose_*` exposes the actual filtered pose.
The initial row has count zero. All margins retain their original noise sigma
and drift allowance; averaging does not reduce the claimed uncertainty margin.

The Python verifier independently reconstructs each average using anchors in a
dead-reckoning frame, rather than incrementally transporting every old pose as
the Rust filter does. It checks all samples and preview poses, physical motion,
paired randomness, guard states and release timers. The geometry audit covers
every candidate guard entry, release and loss resume, plus fixed checkpoints.
Known `joint` summaries match the prior entry-buffer study; the seed-201 original
default pair is checked separately. Filter fixtures run with:

```bash
cargo test --locked -p rust_robotics_control --no-default-features \
  --example pose_observation_loss
```

In addition to waits and holds, `control-events.csv` and the summary count short
re-entries using the preceding diagnosis's definition: positive release command,
no intervening loss wait, lag <=1 s and travel <0.1 before the next entry command.
Counts cover each run until termination; successful and timed-out runs have
different observation durations. The fixed comparison uses known seeds 253–255
and new seeds 256–258, both noise models and all loss modes: 648 cases.
See the [motion-filter comparison](results/motion-filter/report.md).

## Diagnose remaining motion-filter timeouts

This offline study selects all four pairs that lost a hold with `filtered`,
plus all 30 new-seed correlated-noise candidate timeouts from the preceding
study. One case overlaps, giving 33 unique candidate tracks and four successful
`joint` counterparts. It adds no policy, validation seeds or simulation runs.

```bash
python3 experiments/pose_observation_loss/analyze_filter_stalls.py \
  --reference /tmp/filter-reference --known-white /tmp/filter-known-white \
  --white /tmp/filter-white --correlated /tmp/filter-correlated \
  --output /tmp/filter-stalls --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_filter_stalls.py
```

Use the same historical runs or recreate them with the engine from commit
`8707566`. The analyzer checks the pinned summaries, simulator, seeds and
fixtures, then records raw-track and helper hashes. It reconstructs filtered
poses, guard/recovery states and physical motion for the selected trajectories.
For each lost pair, waits use a common horizon ending at `joint`'s successful
termination. Candidate tail diagnostics cover command intervals (150, 180].

Timeouts are classified by first true goal entry and tail travel. The maximum
possible hold budget is `180 - first_entry + 0.02`: the original simulator
credits a timestep at the first qualifying endpoint. Thus first entry at
170.02 s can still allow 10 s; 170.04 s cannot. No-entry tracks use the existing
strict tail-travel threshold <0.1 to distinguish near-stationary from moving.

Geometry checkpoints include stationary observation age 0.5 s, the first
stationary zero-speed-probe wait, all guard/loss releases and the final decision
in the tail. At each point, an offline counterfactual substitutes the filtered
pose into the speed probe while keeping its original steering and margin.
The resulting target speed is passed through a new stopping forecast before
checking joint readiness. Existing freshness, sample and recovery-line gates
remain required. These snapshots cannot establish continuous one-second
admission or the behavior of a changed policy. Optional plots require Matplotlib.
See the [remaining-timeout diagnosis](results/filter-stalls/report.md).
