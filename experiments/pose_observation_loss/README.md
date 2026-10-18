# Pure Pursuit with missing external pose observations

This experiment compares the existing simulated goal slowdown/stop controller
with a candidate pause-and-recovery policy. It uses the repository's bicycle
model and Pure Pursuit implementation, fixed A*, JPS, and Theta* paths, and
paired noise/loss streams. The policy lives in an example; library behavior is
unchanged.

For the completed 19-reset evidence audit, start with the
[consolidated review](results/fusion-return-reset-review/report.md) and its
[one-command replay](#review-and-replay-the-reset-evidence).

The [mean-position full-probe diagnostic](results/fusion-return-reset-mean-probe/report.md)
then replays the same 19 reset probes with only x/y replaced by the recorded
stationary mean. Eighteen reach the two-unit probe horizon; one returns 0.20
before its fifth chord fails. All 38 raw/mean evaluations match the unchanged
Rust function. These are rounded-input geometry results; controller recovery
and mission outcomes are not rerun. See the
[replay commands](#replay-the-reset-probes-at-mean-positions).

## Continuous checks

The [experiment workflow](../../.github/workflows/pose-observation-loss.yml)
runs on pushes and pull requests that change this experiment, the control/core
crates, or the workflow itself. It can also be started with `workflow_dispatch`.
It uses Python 3.12 and stable Rust to run all `test_*.py` tests, including the
native fixtures that compile and cross-check the recorded Rust expressions.
Zero discovered tests, skipped tests and test failures all fail the check.
The test suite uses NumPy, the Python standard library and `rustc`. Install its
Python dependency with the same pinned requirements used by CI:

```bash
python3 -m pip install --only-binary=:all: -r experiments/pose_observation_loss/requirements-ci.txt
```

The workflow also checks the six reset studies' frozen artifact/script hashes,
the saved review's runner/protocol/catalogue and export hashes, and the exact
19-row projection from the original surplus table. Logs are uploaded on both
success and failure for 14 days.

The `reset-evidence-replay` job performs the
[complete reset replay](#review-and-replay-the-reset-evidence): six collectors,
six plotters and six independent numerical verifiers, followed by the completed
packet integrity check. It uses Python 3.12 on Ubuntu 24.04 with the pinned
Matplotlib rendering profile in `requirements-replay.txt`. Every table and SVG
must match the frozen bytes. JSON provenance must have identical content,
with source/plot digest references bound to the actual fresh files; object key
order is ignored. Array order, value types, methods, arithmetic/audit counts,
script hashes and all other fields remain frozen. The fresh PNGs are checked
against their completion-manifest hashes.

Each stage's files and provenance are checked before the next stage starts.
A mismatch preserves that stage's logs and stops the replay. The completed
packet is published only after another full check of all stages and frozen
inputs, which also detects earlier files changed by later children.

Each stage also checks that only the `stages/` directory exists at the packet
root. Unexpected entries stop the replay and preserve child outputs before
the overview, index and completion manifest are written.

Both CLI modes bind the frozen inputs to the original recorded review. Before
a replay creates its output directory, it checks the recorded review manifest,
runner, protocol, freeze catalogue, original CSV exports and all 91 input pins.
The replay repeats these checks before writing its completion manifest. Changes
to those references fail validation even when a packet is otherwise consistent.

The completion manifest is published by an atomic rename after its temporary
file closes successfully. Write or publication failures retain stage outputs
and leave no completion manifest; handled failures also remove the temporary file.

If a collector, plotter or verifier fails, the CLI reports its stage, phase,
status and absolute log path. It stops immediately and preserves the partial
outputs and original log bytes. The CLI preserves positive child exit codes
and returns `128 + signal` for a child terminated by a signal.

Download the `reset-evidence-replay` artifact from the workflow run to obtain
the full `fusion-return-reset-review/` folder, its 18 child logs, the runner
log, the integrity receipt and the source/Python/package version record. The
upload runs on success and failure with 14-day retention. A failed replay retains
its partial outputs and diagnostics. Failure annotations show the last child and
runner log lines and the first differences from frozen artifacts.

The first line of `reset-review-environment.txt` is `source_commit=<full SHA>`
from the checkout that produced the artifact. Use a clean checkout of that commit
and run the following command from its repository root. The check validates
driver, helper and dependency-file hashes, so a later `main` can
reject an older valid packet after those files change. This read-only check
needs only the Python standard library; it does not rerun the six stages.

```bash
python3 -S -B experiments/pose_observation_loss/ci_reset_replay.py \
  --check /path/to/extracted-artifact/fusion-return-reset-review
```

The CI command is `ci_reset_replay.py --output /tmp/new-ci-reset-replay`.
It preserves generated files exactly as written by the original collectors,
plotters and independent verifiers, including their actual provenance hashes.
`ci-replay-sources.json` records this comparison policy, the driver/helper and
dependency-file hashes, actual file hashes and frozen reference proof hashes.
The study index references actual fresh proof/source hashes. This handles
directory enumeration differences between filesystems. The historical
`review_reset_evidence.py` command and `check_reset_review.py` retain their
original byte checks and `review-sources.json` format.

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

## Compare a terminal recovery preview

This comparison retains the motion filter in both arms. `recoverable` adds a
forward recovery segment at the predicted stopping pose, while `filtered`
retains the original stopping footprint. The fixed settings are evaluated on
known seeds 256–258 and new seeds 259–261, with independent and 1 s correlated
noise, all loss modes, maps and planners: 648 comparison cases.

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --recovery-preview-comparison "$fixtures" --seeds 256 257 258 \
  --noise-correlation-time 1 --output /tmp/recovery-preview-reference
python3 experiments/pose_observation_loss/run.py \
  --recovery-preview-comparison "$fixtures" --seeds 256 257 258 \
  --noise-correlation-time 0 --output /tmp/recovery-preview-known-white
python3 experiments/pose_observation_loss/run.py \
  --recovery-preview-comparison "$fixtures" --seeds 259 260 261 \
  --noise-correlation-time 0 --output /tmp/recovery-preview-white
python3 experiments/pose_observation_loss/run.py \
  --recovery-preview-comparison "$fixtures" --seeds 259 260 261 \
  --noise-correlation-time 1 --output /tmp/recovery-preview-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --recovery-preview --reference /tmp/recovery-preview-reference \
  --known-white /tmp/recovery-preview-known-white \
  --white /tmp/recovery-preview-white \
  --correlated /tmp/recovery-preview-correlated \
  --output /tmp/recovery-preview-summary --plot
```

Let `m = 0.5 + 2*sqrt(2)*sigma + drift_speed*observation_age`. The existing
forecast applies 0.2 s of nominal command then bounded braking until speed
<=0.001. From its terminal pose and heading, the candidate checks a further
straight segment of length 0.6 against every obstacle and map boundary.
With stopping clearance `Dstop` and terminal segment clearance `Drecovery`,
the combined lower bound is `min(Dstop, Drecovery - 0.1)`.

Entry compares this combined value with `m`; both ordinary and joint release
compare it with `m + 0.05`. Thus entry additionally requires terminal clearance
> `m + 0.1`, matching the original recovery distance allowance, and release
requires > `m + 0.15`. The original 0.5 s/1 s timers apply to the combined rule.
This changes both entry and release. It does not isolate an earlier-braking
effect from extra release waiting, or guarantee the later measured recovery
line is clear. The terminal pose is estimated; the actual guard applies braking
immediately, whereas the forecast contains the existing 0.2 s command phase.

The appended `RECOVERY_PREVIEW` flag defaults to false and requires
`MOTION_FILTER=true`. The trajectory adds `recovery_preview_active`,
`stop_preview_clearance_lower_bound` and `terminal_recovery_clearance`.
For this arm `preview_clearance_lower_bound` is the combined decision metric,
not a physical clearance from one path. The terminal column includes boundaries
and is zero when disabled; initial rows have false/zero sentinels.
The stopping-path pruning retains its original threshold. The terminal segment
uses a full obstacle scan. Actual recovery, speed capping, filtering, paired
randomness and goal holding retain their previous rules.

The verifier checks the combination at every step and independently recalculates
both geometries at all candidate guard entries, releases, loss resumes and fixed
checkpoints. `terminal_only_guard_entries` counts entries that pass the original
stopping-distance test. Baseline `filtered` summaries match the previous study,
and the seed-201 original defaults are checked separately. Analytic geometry
fixtures cover segment interiors, terminal headings, boundaries and symmetry:

```bash
cargo test --locked -p rust_robotics_control --no-default-features \
  --example pose_observation_loss
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_recovery_preview_geometry.py
```

See the [terminal-recovery comparison](results/recovery-preview/report.md) for
results, waits, holds, contacts and limitations.

## Diagnose terminal-preview release gates

After generating the two new-seed runs above, diagnose all 162 `recoverable`
tracks and the `filtered` counterparts for the 34 lost holds:

```bash
python3 experiments/pose_observation_loss/analyze_preview_gates.py \
  --white /tmp/recovery-preview-white \
  --correlated /tmp/recovery-preview-correlated \
  --output /tmp/preview-gates
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_preview_gates.py
```

The output directory must be new. This analysis uses the pinned
`results/recovery-preview/all-results.csv` and provenance, checks planning-input
hashes and records trajectory hashes. It verifies every candidate guard/loss
release and filtered pose, bicycle integration on all selected tracks, and exact
geometry at both ends of every short re-entry. No simulator rerun or Matplotlib
installation is needed for this analysis; generating the inputs still requires
the Rust workflow above.

Release opportunities use the pre-command guard/loss state, include the actual
release interval and exclude an independent guard's triggering interval. Each
contiguous release route forms one episode. Time categories first separate
failures of non-preview requirements, then failures of the stopping footprint,
terminal line, both or neither. These categories partition release opportunities,
not the full waiting-time union. Independent and joint evidence windows are
0.5 s and 1 s. Removing the terminal test only from the recorded readiness
calculation keeps all later inputs fixed; it does not replay a different policy.

CSV outputs contain cases, route episodes, short re-entries, independent geometry
checkpoints, matched-horizon waits for lost pairs and route totals. `sources.json`
records provenance and hashes. See the [release-gate diagnosis](results/preview-gates/report.md)
for results and interpretation limits.

## Compare terminal entry with stop-only release

This ablation keeps `recoverable`'s terminal-line entry rule in both arms.
Candidate `entryline` changes both independent and joint release to the original
stopping-footprint check. The fixed [protocol](results/terminal-entry-only/protocol.md)
uses known seeds 259–261 and new seeds 262–264, independently paired in each
noise/loss condition (648 runs):

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --terminal-entry-only-comparison "$fixtures" --seeds 259 260 261 \
  --noise-correlation-time 1 --output /tmp/terminal-entry-reference
python3 experiments/pose_observation_loss/run.py \
  --terminal-entry-only-comparison "$fixtures" --seeds 259 260 261 \
  --noise-correlation-time 0 --output /tmp/terminal-entry-known-white
python3 experiments/pose_observation_loss/run.py \
  --terminal-entry-only-comparison "$fixtures" --seeds 262 263 264 \
  --noise-correlation-time 0 --output /tmp/terminal-entry-white
python3 experiments/pose_observation_loss/run.py \
  --terminal-entry-only-comparison "$fixtures" --seeds 262 263 264 \
  --noise-correlation-time 1 --output /tmp/terminal-entry-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --terminal-entry-only --reference /tmp/terminal-entry-reference \
  --known-white /tmp/terminal-entry-known-white \
  --white /tmp/terminal-entry-white \
  --correlated /tmp/terminal-entry-correlated \
  --output /tmp/terminal-entry-summary --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_terminal_entry_release.py
```

With `m` as defined above, both arms enter when
`min(Dstop, Dterminal - 0.1) <= m`. The reference requires the same minimum to
exceed `m + 0.05` for release, while the candidate only requires
`Dstop > m + 0.05`. Independent release still needs stationary speed and 0.5 s
of evidence. Joint recovery still needs the original measured recovery line,
freshness, stationary observations and positive planned command for 1 s.
The entry condition may fail again immediately after a stop-only release; no
extra latch, threshold or steering change prevents this in the experiment.

Append `TERMINAL_ENTRY_ONLY=true` after `RECOVERY_PREVIEW=true` to invoke the
candidate directly. The new flag defaults to false and requires the terminal
preview. The trajectory adds `terminal_entry_only` and
`release_preview_clearance_lower_bound`: the latter is the stopping metric for
`entryline`, and the existing combined metric for `recoverable`. The existing
`preview_clearance_lower_bound` remains the entry metric. Initial rows use
false/zero sentinels. Verification independently checks both metrics and timers;
old logs without a release column retain their original preview-based release rule.

Paired trajectories must agree until the first differing release action, apart
from the two mode/metric columns and the two evidence counters. The known
reference summaries are checked against the prior terminal-preview comparison.
Every candidate entry, release and loss resume is geometrically audited along
with fixed checkpoints. Plotting requires Matplotlib; the simulation and other
verification use Cargo and Python's standard library.

`release-entry-gaps.csv` also describes candidate guard releases whose entry
condition already fails, and guard entries on the immediately following 0.02 s
interval. This event diagnosis was added after observing the re-entry increase;
it includes zero-command releases and has a different denominator from short
re-entries. Counts of audited releases must match the full-trajectory verifier.
See the [release ablation results](results/terminal-entry-only/report.md).

## Compare entry-compatible terminal release

Candidate `aligned` retains the terminal entry rule and requires that same
terminal clearance at release. The stop-only reference is `entryline`.
The fixed [protocol](results/aligned-release/protocol.md) uses known seeds
262–264 and new seeds 265–267 (648 comparison runs):

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 262 263 264 \
  --noise-correlation-time 1 --output /tmp/aligned-release-reference
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 262 263 264 \
  --noise-correlation-time 0 --output /tmp/aligned-release-known-white
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 265 266 267 \
  --noise-correlation-time 0 --output /tmp/aligned-release-white
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 265 266 267 \
  --noise-correlation-time 1 --output /tmp/aligned-release-correlated
python3 experiments/pose_observation_loss/summarize_stopping_preview.py \
  --aligned-release --reference /tmp/aligned-release-reference \
  --known-white /tmp/aligned-release-known-white \
  --white /tmp/aligned-release-white \
  --correlated /tmp/aligned-release-correlated \
  --output /tmp/aligned-release-summary --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_aligned_release.py
```

Both arms enter if `min(Dstop, Dterminal - 0.1) <= m`. Reference release requires
`Dstop > m + 0.05`; candidate release requires both `Dstop > m + 0.05` and
`Dterminal > m + 0.1`. Thus the candidate release metric is
`min(Dstop, Dterminal - 0.05)`, compared with `m + 0.05`. It retains the stopping
hysteresis while requiring the same terminal clearance as entry. The original
`recoverable` required a stricter `Dterminal > m + 0.15` at release.

The candidate changes both independent and joint release; 0.5 s/1 s evidence
windows and all other requirements remain. Release now implies entry clearance
on the same inputs, but motion, observations and planned commands may change
before the next interval. No entry suppression or extra dwell is added.

Append `ALIGNED_TERMINAL_RELEASE=true` after `TERMINAL_ENTRY_ONLY=true` to invoke
this mode directly. It defaults to false and requires that preceding flag.
The trajectory adds `aligned_terminal_release` (false initially); the existing
release metric column records the new minimum. Existing modes retain their
release rules. Known reference summaries are checked against the previous
stop-only comparison. Paired tracks agree until the first different release
action, apart from the three mode/metric columns and two evidence counters.

The verifier audits both entry and release geometry, including same-instant
entry clearance at all candidate releases and loss resumes. Event gap counts
include zero-command releases and are checked against full trajectory exit
counts. See the [comparison results](results/aligned-release/report.md).

## Diagnose remaining aligned-release timeouts

Use the saved aligned-release runs to diagnose all 93 new-seed candidate
timeouts and the four known-seed lost holds, plus their reference tracks:

```bash
python3 experiments/pose_observation_loss/analyze_aligned_stalls.py \
  --known-white /tmp/aligned-release-known-white \
  --white /tmp/aligned-release-white \
  --correlated /tmp/aligned-release-correlated \
  --output /tmp/aligned-stalls
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_aligned_stalls.py
```

The output directory must be new. The analyzer checks pinned summaries,
provenance and fixture hashes and records raw trajectory hashes. It reconstructs
both timers, filtered poses and bicycle integration. There is no policy change
or simulator rerun; this analysis needs only Python's standard library.

Classification uses first true goal entry, the maximum remaining hold budget
(including the first qualifying endpoint's 0.02 s), and travel in the last 30 s.
Release opportunities use pre-command guard/loss state. The triggering interval
of an independent guard is excluded and the release interval included. The
aligned terminal condition is `Dterminal > release_margin + 0.05`, equivalent
to the terminal entry threshold; its stopping condition remains
`Dstop > release_margin`. The combined entry metric is not used for release.

Tail gate categories partition opportunities: other requirements fail, both
geometries fail, stop only fails, terminal only fails, or everything passes.
Per-requirement failure times overlap; nonapplicable independent-route
requirements have zero recorded failure time. Consecutive readiness runs reset
at route changes or ineligible intervals and are clipped to the last 30 s.
Dropping the terminal requirement only from this calculation holds future
recorded inputs fixed and does not simulate an alternative controller.

Geometry checkpoints include the first occurrence of each tail route/category,
fresh stationary samples at observation age 0.5 s, all tail releases/resumes and
the final point. A second forecast substitutes only the true pre-command pose,
retaining logged speed, command, steering and uncertainty margins. Its
`true_pose_release_passes` field refers only to the geometric release condition;
it does not check other release requirements or predict changed-policy success.
Final true center and straight-line clearances are separate static references,
not the forecast terminal pose or a proof of future recovery feasibility.

Outputs include timeout cases, per-case tail gates, gate totals, geometry
checkpoints, matched-horizon waits for lost pairs, classification totals and
`sources.json`. See the [remaining-timeout diagnosis](results/aligned-stalls/report.md).

## Audit forward arcs at the aligned-release stalls

This offline analysis selects the 81 new-seed `no_entry_stationary` cases from
the pinned aligned-stall diagnosis. It uses the same raw trajectories and
requires their hashes to match that diagnosis:

```bash
python3 experiments/pose_observation_loss/analyze_aligned_arcs.py \
  --white /tmp/aligned-release-white \
  --correlated /tmp/aligned-release-correlated \
  --output /tmp/aligned-arcs --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_aligned_arcs.py
```

The output directory must be new. This uses Python's standard library; optional
plots require Matplotlib. Generate missing raw inputs with the aligned-release
comparison above. No new simulator run or controller change is part of this
analysis.

At the final decision and fixed fresh stationary checkpoints
(`150.5, 155.5, ..., 175.5` s with observation age 0.5 s and pre-command speed
at most 0.02), compare the true pose and logged motion-filter pose at the same
pre-command instant. A row at 180 s therefore uses the physical state at
179.98 s, not its post-command endpoint. The filter and every physical interval
are independently reconstructed; actual filter sample counts are retained.

Reuse the tested analytic point-obstacle and wall distance for forward arcs of
length 0.6, wheelbase 0.4 and fixed steering from -35 to 35 degrees in one-degree
steps. The physical radius 0.5, buffered radius 0.6, and recovery margins
`0.6 + 2*sqrt(2)*sigma` (age zero) and that margin plus `0.02*age` are separate.
True-pose evaluation retains the logged sigma and age allowances.

The continuous-steering maximum is bounded by the lesser of the starting
clearance and the grid maximum plus 0.00585236, with a 1e-9 numerical allowance.
Only a grid witness is called `grid_pass`; only an upper bound no greater than
the margin is called `all_fixed_steering_blocked`. The intervening cases are
`unresolved`. This bound covers fixed steering over the full interval, not
steering switches, shorter motions, reversing or replanning. All final grid arcs
and the straight, best-grid and recorded-steering arcs at fresh checkpoints are
checked against independent 120-chord geometry with a sagitta error bound.

The best angle selected in the filtered pose is also evaluated at the paired
true pose and independently checked with chords. This distinguishes existence
of some passing arc in each pose from passing with the same selected steering.
Paired output fields retain the selected angle, its true clearance and the
pass/fail result under both age margins. These diagnostics do not execute that
command or evaluate its release timer.

Outputs are per-pose checkpoints, the full steering grid, independent geometry
audits, paired pose errors/statuses, summary counts and `sources.json`. The plots
compare best grid slack from true and filtered poses; they do not show recovery
success rates. See the [aligned-stall arc diagnosis](results/aligned-arcs/report.md).

## Compare uncertainty-aware arc selection on saved poses

Use the tracked 497 paired checkpoints and map fixture directly; raw simulator
trajectories and a Rust build are unnecessary for this offline comparison:

```bash
python3 experiments/pose_observation_loss/analyze_robust_arc_selection.py \
  --output /tmp/robust-arc-selection --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_robust_arc_selection.py
```

The output directory must be new. Only optional plots need Matplotlib; the
analysis uses Python's standard library. Inputs, helpers and the
[fixed protocol](results/robust-arc-selection/protocol.md) are hashed in
`sources.json`. Repeat into a second new directory to compare the four CSVs
and `sources.json` byte for byte.

All rules retain the same true-side recovery target
`M = 0.6 + E`, where `E = 2*sqrt(2)*sigma + 0.02*age` and sigma is 0.1.
`nominal` reproduces the previous central-heading best steering. `yaw_min`
maximizes the minimum distance over nine heading offsets from -4 to +4 degrees,
minus `2*0.6*sin(radians(0.5)/2)` for headings between samples. `pose_bound`
uses that same angle and subtracts E again from the clearance score. A rule
accepts only when its score exceeds M by more than 1e-9; near ties retain the
previous smaller-absolute-angle preference.

The full pose score bounds true clearance only if position error is at most E
and wrapped heading error is at most 4 degrees. This envelope is an assumption,
with empirical coverage reported separately, not a Gaussian confidence claim.
Retaining the full target M while subtracting E adds conservatism, so count
abstentions and lost correct selections alongside mismatches. True poses never
enter steering selection or acceptance. The analyzer checks selected arcs with
full obstacle scans and independent chords and checks the conditional bound
at every covered checkpoint.

Outputs contain decisions, every candidate's scores, geometry audits, summary
counts, provenance and optional figures. These are selected failed-run poses,
with repeated checkpoints per run; no controller, release timer or hold outcome
is simulated. See the [selection comparison](results/robust-arc-selection/report.md).

## Separate position and heading effects at fixed selected angles

Reuse the tracked paired poses, map and nominal/yaw_min decisions from the
comparison above. Keep every selected angle and acceptance decision fixed:

```bash
python3 experiments/pose_observation_loss/analyze_arc_pose_components.py \
  --output /tmp/arc-pose-components --plot
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_arc_pose_components.py
```

The output directory must be new. This uses Python's standard library, with
Matplotlib needed only for optional plots; no Rust build or raw trajectories
are required. The [fixed protocol](results/arc-pose-components/protocol.md)
defines four poses: filtered position/heading (FF), true position with filtered
heading (TF), filtered position with true heading (FT), and true position/heading
(TT). These truth substitutions are diagnostic references, not an estimator.

For accepted candidates failing the unchanged true-side recovery margin,
partition failures by which substitutions already fail: position alone, heading
alone, either alone, or both required. Also count failures forced by true starting
clearance alone. Decompose signed clearance loss FF-TT by averaging both
substitution orders: position is `((FF-TF)+(FT-TT))/2`, heading is
`((FF-FT)+(TF-TT))/2`. Negative contributions remain negative. These describe
fixed-angle geometry and do not measure attainable control improvement.

Each distinct checkpoint/angle/pose has a full obstacle scan and independent
chord audit. Outputs include `components.csv`, `geometry-audit.csv`, `summary.csv`
and `sources.json`, plus optional figures. Repeat into another new directory to
compare all three CSVs and provenance byte for byte. See the
[pose-component diagnosis](results/arc-pose-components/report.md).

## Decompose the motion filter's remaining position error

Use the same saved raw aligned-release runs as the aligned-arc diagnosis.
The analyzer requires their trajectory hashes and provenance to match the pinned
inputs; it does not run a new controller or modify the filter:

```bash
python3 experiments/pose_observation_loss/analyze_filter_position_components.py \
  --white /tmp/aligned-release-white \
  --correlated /tmp/aligned-release-correlated \
  --output /tmp/filter-position-components
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_filter_position_components.py
```

The output directory must be new. Only Python's standard library is needed.
At every selected track's step, reconstruct the 25-sample window by transporting
each observation with measured speed and applied steering. A scheduled
correction, including a missed one, resets the window; only a received correction
or precision activation replaces the common bias anchor. Compare position and
circular mean heading with the logged filter output before extracting the same
497 checkpoints.

Split filtered minus true position into mean observation noise, common correction
anchor bias, drift since that anchor, and residual motion transport. The vectors
sum to the error. Their signed projections on the total error direction also
sum to its magnitude; component norms and RMS values do not add. Truth and logged
noise/bias are diagnostic references, not estimator inputs.

For the stationary AR(1) noise process alone, the effective sample count is
`n^2 / (n + 2*sum((n-k)*rho^k, k=1..n-1))`. This does not reduce common anchor
variance or establish pose confidence, particularly on selected failed runs.
The [fixed protocol](results/filter-position-components/protocol.md) keeps the
window, resets and all recorded commands unchanged.

Outputs are checkpoint decompositions, retained window samples, four cohort/time
summaries and `sources.json`. Repeat with another new output directory to compare
all three CSVs and provenance byte for byte. See the
[filter position-error diagnosis](results/filter-position-components/report.md).

## Replay a causal three-epoch position prototype while stationary

Use the same saved aligned-release trajectories and fixed paired checkpoints.
The prototype sees reconstructed sensed pose, measured motion and observation
events; separated truth/noise/bias never enter its estimation API:

```bash
python3 experiments/pose_observation_loss/analyze_stationary_epoch_fusion.py \
  --white /tmp/aligned-release-white \
  --correlated /tmp/aligned-release-correlated \
  --output /tmp/stationary-epoch-fusion
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_stationary_epoch_fusion.py
```

Only Python's standard library is needed, and the output directory must be new.
Keep at most three received-correction epochs within 15 seconds, each with at
most 25 local observations from its first 0.5 seconds. Transport retained poses
with their own sensed headings and previous measured speed/applied steering.
Clear history when speed magnitude exceeds 0.02 or precision activates; a missed
correction does not create a new epoch.

Use the equally weighted mean of three epoch means only when stationary, current
observation age is at most 1.5 seconds, each older epoch has at least 10 samples,
and the newest epoch matches the latest receipt. Its current epoch can have one
sample. Otherwise retain the logged baseline pose. The heading always remains
the baseline heading. See the predeclared [protocol](results/stationary-epoch-fusion/protocol.md).

Select steering from each estimated pose with the same grid, tie rule and full
margin as before, then evaluate that fixed angle at the paired true pose. No
uncertainty reduction is assumed. Report availability/fallback subsets, position
errors, accepted true passes, mismatches and paired gains/losses. Each selected
arc has a full obstacle scan and independent chord audit.

Outputs are estimates, retained epoch means/counts, decisions, geometry audits,
summary subsets and provenance. Repeat into another new directory to compare
five CSVs and `sources.json` byte for byte. This replays a position prototype on
selected failed-run observations; it does not rerun commands, timers or hold
outcomes. See the [prototype comparison](results/stationary-epoch-fusion/report.md).

## Compare arc selection using the spread between received epochs

Use only the tracked estimates, retained epoch means, previous decisions and map:

```bash
python3 experiments/pose_observation_loss/analyze_epoch_spread_selection.py \
  --output /tmp/epoch-spread-selection
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_epoch_spread_selection.py
```

The output directory must be new. Only Python's standard library is needed;
there is no raw trajectory replay or Rust build. The
[fixed protocol](results/epoch-spread-selection/protocol.md) retains all previous
availability, fallback, poses, headings and full true-side margins.

Alongside the pinned baseline and reproduced fused-center selector, `epoch_min`
chooses the best worst clearance over the three retained mean positions at the
existing heading. This certifies only those discrete poses. It does not bound
clearance between them. `spread_yaw_bound` uses the fused center, maximizes the
worst clearance over nine heading offsets from -4 to +4 degrees, subtracts the
heading discretization allowance, and subtracts R, the largest distance from
the center to an epoch mean. Its score bounds true clearance only if true
position error is <=R and wrapped heading error is <=4 degrees. Measure empirical
coverage separately; the spread is not a confidence guarantee.

All candidate rules fall back exactly to the previous baseline at unavailable
checkpoints. Score and acceptance use no truth. Report lost/gained accepted true
passes against `fused_center`, mismatches, abstentions and envelope coverage,
including available/fallback subsets. Coverage fields always refer to the fused
center envelope, including on the baseline reference rows.

Outputs are decisions, per-checkpoint spread, full candidate scores, geometry
audits, summary subsets and provenance. Repeat into another new directory to
compare five CSVs and `sources.json` byte for byte. No control or hold outcome is
rerun. See the [spread-based selection comparison](results/epoch-spread-selection/report.md).

## Validate the frozen replay rules on new observation seeds

Generate new observations with the unchanged controller, then replay the frozen
position prototype and four arc selectors. Use the configured Rust toolchain;
the standard runner performs the locked release build and existing verification:

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 268 269 270 \
  --noise-correlation-time 0 --output /tmp/epoch-holdout-white
python3 experiments/pose_observation_loss/run.py \
  --aligned-release-comparison "$fixtures" --seeds 268 269 270 \
  --noise-correlation-time 1 --output /tmp/epoch-holdout-correlated
python3 experiments/pose_observation_loss/analyze_epoch_selection_holdout.py \
  --white /tmp/epoch-holdout-white \
  --correlated /tmp/epoch-holdout-correlated \
  --output /tmp/epoch-selection-holdout
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_epoch_selection_holdout.py
```

All output directories must be new. Generation also runs the entryline comparison
controls (324 cases in total); the replay analyzes all 162 aligned cases. No
candidate estimator output enters the generating controller. The analyzer checks
the prior reports' source hashes to keep the prototype/selection rules frozen,
and validates the simulator, seeds, noise settings and fixture hashes.

The [predeclared protocol](results/epoch-selection-holdout/protocol.md) keeps one
final pre-command checkpoint from every trajectory, including held, timeout and
collision outcomes and moving endpoints. It also keeps fixed late fresh stationary
checkpoints at 150.5 through 175.5 seconds. A coincident final point is counted
once. Logged observation age and sigma, including precision-active 0.03, remain
unchanged. Reconstruct the original filter at every step and evaluate the same
full margin and conditional spread/heading envelope as before.

Outputs contain case outcomes, checkpoint estimates, epoch means, decisions,
spread, full candidate scores, geometry audits, outcome/subset summaries and
position-error summaries, plus provenance. The aggregation uses Python's standard
library. Repeat it into another new directory to compare nine CSVs and
`sources.json` byte for byte on the same generated raw runs. Regenerating the raw
runs records new execution provenance, including its start time and commit.

The held/timeout/contact results belong to the unchanged generating controller.
Replay metrics test the fixed rules on new observations and hypothetical arcs;
they do not establish candidate-control reachability or hold improvement. See
the [new-seed observation validation](results/epoch-selection-holdout/report.md).

## Run the stationary-epoch position estimator in the stopping preview

The optional `STATIONARY_EPOCH_FUSION` simulator argument follows
`ALIGNED_TERMINAL_RELEASE` and defaults to false. It requires the aligned
configuration. The paired runner sets all preceding flags:

```bash
fixtures=experiments/pose_observation_loss/results/planning-clearance/fixtures
epoch_runs=/tmp/live-epoch-preview
python3 experiments/pose_observation_loss/run.py \
  --epoch-fusion-comparison "$fixtures" --seeds 268 269 270 \
  --noise-correlation-time 0 --output "$epoch_runs/known-white"
python3 experiments/pose_observation_loss/run.py \
  --epoch-fusion-comparison "$fixtures" --seeds 268 269 270 \
  --noise-correlation-time 1 --output "$epoch_runs/known-correlated"
python3 experiments/pose_observation_loss/run.py \
  --epoch-fusion-comparison "$fixtures" --seeds 271 272 273 \
  --noise-correlation-time 0 --output "$epoch_runs/new-white"
python3 experiments/pose_observation_loss/run.py \
  --epoch-fusion-comparison "$fixtures" --seeds 271 272 273 \
  --noise-correlation-time 1 --output "$epoch_runs/new-correlated"
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_epoch_preview.py
```

Use the configured Rust toolchain and new output directories. These four runs
produce and verify 648 live cases. `aligned` is the reference; `epoch` replaces
only stopping-preview x,y when the frozen three-epoch estimator is available.
Heading comes from the original motion filter. The estimator retains stationary
fresh observations from received corrections, averages each epoch separately,
and equally weights three epochs. Motion or precision activation clears history.
The [fixed protocol](results/epoch-preview/protocol.md) specifies the exact
windows, sample requirements and inclusive thresholds.

The original position remains in use whenever fusion is unavailable. Nominal
steering, speed-probe inputs, raw recovery means, margins and timers retain their
existing definitions; changed preview decisions can change subsequent motion.
The offline arc selectors are not executed. New CSV columns record enablement,
availability, epoch count, original filtered x,y, and round-trip measured speed.
The latter preserves strict threshold decisions that nine-decimal speed could
otherwise obscure. Python replays the unchanged prototype on every candidate
row and checks the position, history, fallback and preserved heading.

Each runner writes verified outcomes, event metrics and geometry audits without
requiring earlier raw data. The published aggregate additionally checks all old
fields against the prior known-seed observations and against the initial batch
before speed-log precision was improved. Its analyzer command and retained raw
paths appear in the [live comparison](results/epoch-preview/report.md). The
analyzer writes per-case metrics, paired waits at a common time horizon, cohort
summaries and provenance. Repeat aggregation on the same raw inputs to compare
four CSVs and `sources.json` byte for byte.

Historical replay analyzers deliberately pin source hashes. To regenerate their
archived inputs or reports, use the commit named in the corresponding report.
The live estimator remains an opt-in simulation experiment.

## Diagnose where the live epoch-preview commands diverge

Reuse the verified live trajectories and the published epoch-preview case tables:

```bash
python3 experiments/pose_observation_loss/analyze_epoch_divergence.py \
  --output /tmp/epoch-divergence
python3 -m unittest discover -s experiments/pose_observation_loss \
  -p test_epoch_divergence.py
```

The default raw paths come from the prior report's provenance. To use regenerated
live runs, pass `--raw-root DIR`, where DIR contains `known-white`,
`known-correlated`, `new-white` and `new-correlated` from the four preceding
runner commands. The analyzer checks raw trajectory hashes, simulator, fixtures,
seeds and noise settings; a new start time or Git commit does not invalidate
otherwise identical raw observations. An output directory must be new.

The [fixed diagnostic protocol](results/epoch-divergence/protocol.md) retains all
324 pairs in its accounting and examines all 121 with changed applied commands,
including unchanged outcomes. It checks shared physical state and nominal inputs
through the first different guard/loss action. It records the first different
readiness/evidence and traces release evidence back to its last equal value,
so a difference that was subsequently reset is not attributed to that release.
Because unequal timer values can change which arm leads, it also locates the
later arm's last failed requirement within the earlier arm's uninterrupted
ready run. That checkpoint explains the immediate release difference.

Selected checkpoints compare the logged pose, original filtered position,
true position with the estimated heading, and full true pose, using the same
speed, proposed commands and margins. Endpoint and final-window checkpoints
remain local diagnostic substitutions after trajectories separate. They do not
predict a different closed-loop outcome. Tail tables show the final 30 seconds
at each arm's endpoint and at the shorter common endpoint, including overlapping
release failures and maximum consecutive readiness under fixed requirement masks.

Outputs are accounting, first forks, prefix events, geometry, tails, requirements,
cohort/outcome summaries and source fingerprints. Repeat aggregation to compare
seven CSVs and `sources.json` byte for byte. This analysis changes no controller
or estimator. See the [divergence diagnosis](results/epoch-divergence/report.md).

To render the recorded shared-history release example (Matplotlib required):

```bash
python3 experiments/pose_observation_loss/plot_epoch_release_example.py \
  --diagnosis /tmp/epoch-divergence --output /tmp/epoch-release-example
```

This fixed example shows the first new-seed lost-hold case without observation
loss. The numerical series, SVG and provenance are deterministic. The plot uses
confirmation evidence immediately before the controller resets it on release,
and checks those values against the original logged counters.

### Terminal preview with the commanded steering

Compare the existing straight 0.6 m terminal preview with a constant-steering
arc using the already recorded nominal steering, from the same forecast stop
pose. This portable calculation uses all 1,116 published divergence checkpoints
at four pose views. It searches no steering angles and changes no controller.

```bash
python3 experiments/pose_observation_loss/analyze_terminal_steering_preview.py \
  --output /tmp/terminal-steering-preview
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

The analyzer needs only Python's standard library and checked-in checkpoint
tables and map; no raw trajectories are required. Use a new output directory.
It checks pinned inputs, forecast step counts and the prior straight geometry,
then audits both paths at every pose against an independent 120-chord calculation
and its sagitta bound. Repeat aggregation to compare five CSVs and `sources.json`
byte for byte. Entry and release margins are evaluated separately.

The [fixed protocol](results/terminal-steering-preview/protocol.md) keeps all
121 command-changing pairs, including unchanged outcomes. Outputs retain all
324 pairs in accounting, full pose geometry, estimated/true decisions, subgroup
summaries and paired straight-to-arc transitions. A clear terminal arc alone
does not satisfy the separate recovery straight-line gate or confirmation timers
and does not predict a held goal. See the
[terminal-steering comparison](results/terminal-steering-preview/report.md).

### Live comparison of straight and curved terminal previews

The optional `TERMINAL_STEERING_PREVIEW` final simulator argument defaults to
false and requires stationary epoch fusion. The runner's `curved` arm enables
it; `epoch` retains the original straight path. Only the terminal 0.6 m preview
uses the current proposed steering. The stopping prefix, margins, raw recovery
straight line, speed probe, estimator and confirmation timers stay the same.

Reproduce the fixed 648-case matrix with cached Cargo dependencies:

```bash
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 269 270 271 --noise-correlation-time 0 --output /tmp/terminal-arc-live/known-white
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 269 270 271 --noise-correlation-time 1 --output /tmp/terminal-arc-live/known-correlated
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 274 275 276 --noise-correlation-time 0 --output /tmp/terminal-arc-live/new-white
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 274 275 276 --noise-correlation-time 1 --output /tmp/terminal-arc-live/new-correlated
python3 experiments/pose_observation_loss/analyze_terminal_arc_live.py \
  --raw-root /tmp/terminal-arc-live --output /tmp/terminal-arc-summary
python3 experiments/pose_observation_loss/check_terminal_arc_default.py \
  --raw-root /tmp/terminal-arc-live --output /tmp/terminal-arc-default
python3 experiments/pose_observation_loss/plot_terminal_arc_live.py \
  --results /tmp/terminal-arc-summary --output /tmp/terminal-arc-figure
```

All output directories must be new. The runner builds the locked release
example and verifies the full observation/filter/recovery/hold records and
selected exact preview geometry. The analyzer independently audits every
physical transition, swept contact and map boundary, and verifies common inputs
until commands first differ. Its baseline regression needs the pinned prior
epoch-preview raw files at the paths in that report's `sources.json`; those files
are preserved in this cloud workspace. Regenerate historical references with
their recorded simulator version if needed. Aggregation and the default check
use Python's standard library; plotting needs Matplotlib.

The [fixed protocol](results/terminal-arc-live/protocol.md) covers all previously
hold-changing pairs and new seeds 274–276 without tuning after results. Outputs
are cases, pairs, cohort/loss-mode summaries, verification counts and source/raw
fingerprints. Repeat aggregation to compare four CSVs and `sources.json` byte
for byte; figure data, SVG and source fingerprints are deterministic as well.
See the [live terminal-arc results](results/terminal-arc-live/report.md).

### Diagnose terminal-arc control divergences

Reuse the pinned live records to locate the first different guard/loss action,
applied command and physical state in every one of the 211 command-changing
pairs, retaining all 324 pairs in accounting. The six gained and six lost holds
are reported alongside unchanged outcomes. No new control rollout is involved.

```bash
python3 experiments/pose_observation_loss/analyze_terminal_arc_divergence.py \
  --output /tmp/terminal-arc-divergence
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
python3 experiments/pose_observation_loss/plot_terminal_arc_divergence.py \
  --diagnosis /tmp/terminal-arc-divergence --output /tmp/terminal-arc-example
```

The default raw paths are recorded in the previous live report. Both scripts
accept `--raw-root DIR` for relocated or identically regenerated cohorts. Input,
simulator/helper and raw hashes are checked; a different run start time or Git
commit is allowed when the controls and raw bytes match. Use new output folders.

The [fixed diagnostic protocol](results/terminal-arc-divergence/protocol.md)
separates an earlier guard entry from an earlier release and finds the failed
readiness check that survives timer resets or lead reversals. Exact stopping
geometry and both terminal paths are compared at logged, true-x/y and true poses,
using the same nominal controls and margins. Independent chord checks cover the
terminal geometry. Post-fork 5s/30s windows and own/common final windows describe
recorded motion, waits and readiness requirements, without predicting outcomes
under masked requirements or attributing the final hold to a single early gate.

Repeat aggregation to compare seven CSVs and `sources.json` byte for byte. The
standard-library analyzer needs the pinned raw trajectories; the illustrative
plot also needs Matplotlib. Its fixed lost-hold example was chosen from the
previous report, and exports sampled goal-distance data, complete wait spans,
SVG and source fingerprints. See the
[terminal-arc divergence diagnosis](results/terminal-arc-divergence/report.md).

### Recovery-line geometry and positive-speed admission

Analyze every timeout in both live terminal-arc arms, retaining all 648 cases
in accounting. This diagnosis covers all 368 timeouts, independently of whether
their paired commands changed. It checks the final 30 seconds of joint admission
and fixed endpoint, first-usable-epoch and age-0.5 checkpoints.

```bash
python3 experiments/pose_observation_loss/analyze_joint_admission_geometry.py \
  --output /tmp/joint-admission-geometry
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
python3 experiments/pose_observation_loss/plot_joint_admission_geometry.py \
  --results /tmp/joint-admission-geometry --output /tmp/joint-admission-figure
```

Use a new output directory. The default raw paths come from the pinned live
report; `--raw-root DIR` supports relocated or identically regenerated cohorts.
The analyzer checks prior tables, source and raw hashes and uses only Python's
standard library. It exports accounting, per-case readiness/masks, checkpoints,
six-pose geometry, estimated/true gate comparisons, summaries and provenance.
Repeat aggregation to compare six CSVs and `sources.json` byte for byte.
The optional Matplotlib figure exports deterministic numeric data, SVG and
source fingerprints for the fixed readiness masks and age-0.5 geometry subset.

The [fixed protocol](results/joint-admission-geometry/protocol.md) preserves
inclusive recovery wall bounds, strict obstacle clearance, the original speed
probe and the uncapped goal command. The first 0.05 chord determines whether
the speed probe permits positive speed and is checked against the full 40-chord
calculation. Current-age and zero-age margins are recorded separately; a true
starting clearance below the zero-age floor obstructs any path requiring that
same margin at the fixed position and sigma.

Readiness masks describe the recorded requirements. Reusing the stopping-preview
pose for speed would also change its proposed target and stopping forecast, so
these geometry substitutions are not reported as controller rollouts or new
held-goal outcomes. See the
[joint-admission diagnosis](results/joint-admission-geometry/report.md).

### Compare the recovery line with the recorded-steering arc

Use the same 2,120 checkpoints and six poses per checkpoint from all 368 timeout
cases in the previous diagnosis. Compare the original 0.6 m line with a 0.6 m
arc at the logged nominal steering, preserving the same sigma, age, distance
margin and inclusive recovery wall bounds. No steering search is involved.

```bash
python3 experiments/pose_observation_loss/analyze_commanded_recovery_arc.py \
  --output /tmp/commanded-recovery-arc
python3 experiments/pose_observation_loss/plot_commanded_recovery_arc.py \
  --results /tmp/commanded-recovery-arc --output /tmp/commanded-recovery-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. The standard-library analysis consumes only tracked
tables and map data, checking their hashes and the pinned simulator/helpers. It
does not require raw trajectories: the prior provenance chain retains their
validation. Every curve is checked against independent 120-chord geometry with
the circular sagitta bound; identical geometry inputs share the calculation.

The [fixed protocol](results/commanded-recovery-arc/protocol.md) compares actual
recovery and stopping-preview inputs against evaluation-only truth. Both gained
and lost permissions, optimistic permissions, unnecessary blocking and physical
radius comparisons are retained. Summaries preserve cohort, arm and overlapping
checkpoint subsets. No substituted speed target, confirmation timer or live
held-goal outcome is computed.

Repeat aggregation to compare six CSVs and `sources.json` byte for byte. The
optional Matplotlib plot includes all truth/estimate categories at age 0.5 s,
exporting deterministic numeric data, SVG and source fingerprints. See the
[commanded recovery-arc comparison](results/commanded-recovery-arc/report.md).

### Separate position and heading effects on commanded recovery paths

Reuse every recorded line/arc decision from the previous comparison. For both
recovery and preview inputs, evaluate estimated xy/yaw, true xy/estimated yaw,
estimated xy/true yaw and full truth, preserving the nominal steering and margin.
Classify optimistic permissions and conservative blocks in both directions,
including the fixed extra arc permissions that failed the true margin.

```bash
python3 experiments/pose_observation_loss/analyze_commanded_arc_components.py \
  --output /tmp/commanded-arc-components
python3 experiments/pose_observation_loss/plot_commanded_arc_components.py \
  --results /tmp/commanded-arc-components --output /tmp/commanded-components-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. Analysis needs only the tracked tables and map;
source and input hashes are checked, with raw validation inherited from prior
provenance. Every distinct hybrid geometry is checked against independent
120-chord integration. Repeated inputs share geometry, while each margin is
evaluated separately. Component records link to unique geometry-audit IDs.

The [fixed protocol](results/commanded-arc-components/protocol.md) preserves
inclusive recovery wall bounds and strict obstacle clearance. Signed distance
differences are split symmetrically over both coordinate-substitution orders;
negative terms remain. Paired path effects also check that estimated-minus-true
arc improvement equals the change in total position/heading overstatement.
These coordinate and path substitutions do not identify causal error sources
or predict live recovery and held-goal outcomes.

Repeat aggregation to compare five CSVs and `sources.json` byte for byte. The
optional Matplotlib figure includes both mismatch directions for both paths,
inputs and arms at age 0.5 s, exporting numeric data, SVG and source fingerprints.
See the [commanded-arc component diagnosis](results/commanded-arc-components/report.md).

### Decompose recovery and epoch-fused position error

Replay the same 368 timeout trajectories and 2,120 fixed checkpoints from the
component diagnosis. Reconstruct raw recovery, motion-filter and three-epoch
fusion windows, including their actual weights, resets and partial latest epoch.
The position error is split into observation noise, shared received-correction
anchor, drift and transported historical-position residual.

```bash
python3 experiments/pose_observation_loss/analyze_epoch_position_components.py \
  --output /tmp/epoch-position-components
python3 experiments/pose_observation_loss/plot_epoch_position_components.py \
  --results /tmp/epoch-position-components --output /tmp/epoch-position-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. The default raw locations come from pinned prior
provenance; `--raw-root DIR` supports relocated identical cohorts. Source, input,
provenance and raw hashes are checked. Every consumed interval checks window
counts and fusion availability. The fixed checkpoints also check logged xy/yaw
and previous position-error norms. Original physics/contact/timer validation is
inherited, with no new trajectories or control changes.

The [fixed protocol](results/epoch-position-components/protocol.md) retains both
inputs, estimator fallbacks and predeclared decision subgroups. Equal epoch
weights are preserved even when their observation counts differ. Noise covariance
uses actual sample spacing, weights and correlation; shared anchors are grouped
by reset identity. The exported model standard deviations are per axis and
unconditional, excluding drift/transport uncertainty. They are not confidence
intervals calibrated on the selected timeout cases.

Repeat aggregation to compare five CSVs and `sources.json` byte for byte. The
optional Matplotlib figure compares fixed age-0.5 2D error/component RMS by noise
type, arm and actual input estimator, with explicit denominators. Component RMS
values are not additive. See the
[epoch-position decomposition](results/epoch-position-components/report.md).

### Compare model-based three-way admission envelopes

Keep the same 648-case accounting, 368 timeout cases and 2,120 checkpoints.
Compare the original central gate with fixed position/heading envelopes using
the exported observation-noise model, then noise plus shared correction anchors.
Both straight and recorded-steering .6 m paths keep the same full margin.

```bash
python3 experiments/pose_observation_loss/analyze_model_admission_envelopes.py \
  --output /tmp/model-admission-envelopes
python3 experiments/pose_observation_loss/plot_model_admission_envelopes.py \
  --results /tmp/model-admission-envelopes --output /tmp/model-admission-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. Analysis uses only tracked tables and map geometry;
input, source, helper and protocol fingerprints are retained. Raw trajectory
validation and observation-window reconstruction are inherited. Every distinct
estimated heading/steering geometry is checked against independent 120 chords.

The [fixed protocol](results/model-admission-envelopes/protocol.md) preserves
strict obstacle clearance and inclusive wall bounds. Lower component bounds
permit, upper bounds block, and overlapping bounds remain unresolved. A wider
envelope can defer either definite decision but cannot reverse it. Truth is
evaluation-only: empirical pose coverage and the conditional geometry bound
are distinct from statistical guarantees. Drift and transport uncertainty are
excluded from the model, and Gaussian tails remain unbounded.

Repeat aggregation to compare four CSVs and `sources.json` byte for byte. The
optional Matplotlib figure exports counts, SVG and fingerprints for all three
states at age 0.5 s, including both paths, inputs and recorded arms. Unresolved
decisions are counted separately; they do not establish blocking, resumed
movement or held-goal outcomes. See the
[model-based admission comparison](results/model-admission-envelopes/report.md).

### Refine bounds within the same pose envelopes

Keep every row, pose radius, heading interval, steering and margin from the
model-based comparison. For each prior-unresolved envelope, seek both a clear
and blocked pose within it. Such a pair proves that refining the distance bound
alone cannot produce a definite admission decision for that pose set.

```bash
python3 experiments/pose_observation_loss/analyze_admission_envelope_refinement.py \
  --output /tmp/admission-envelope-refinement
python3 experiments/pose_observation_loss/verify_admission_envelope_refinement.py \
  --results /tmp/admission-envelope-refinement
python3 experiments/pose_observation_loss/plot_admission_envelope_refinement.py \
  --results /tmp/admission-envelope-refinement --output /tmp/envelope-refinement-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. The [fixed protocol](results/admission-envelope-refinement/protocol.md)
first checks complete finite heading/position stencils. A bounded binary cover
then either finds both outcomes, certifies the entire pose set, or leaves search
unfinished. Missing examples never establish that a pose cannot exist. Mixed
outcomes and unfinished searches both remain unresolved admission outcomes.

New definite states export prefix-complete cover certificates, including all
outside-disk leaves; mixed states export two checked pose witnesses. Every
published witness/certificate geometry receives an independent 120-chord audit.
Other explored geometries use exact component evaluation. Truth labels and
coverage are evaluation-only, and prior raw/physics validation is inherited.
The standalone verifier reloads the CSVs, recomputes published geometry and
checks cover completeness with both tree reconstruction and binary leaf weights;
`--compare DIR` additionally checks all six CSVs and provenance byte for byte.

Repeat six CSVs and `sources.json` byte for byte. The optional Matplotlib figure
partitions all prior-unresolved age-0.5 envelopes for both recorded arms, inputs,
paths and uncertainty models, with each denominator explicit. See the
[same-envelope refinement report](results/admission-envelope-refinement/report.md).

### Compare information floors at the same estimated centres

Keep all 8,480 path/input rows, current estimated centres, +/-4 degree heading
intervals, steering and full margins. Compare ideal repeats of each weighted
estimator batch with its existing shared anchors versus independent copies of
all anchor contributions. Use K=1,3,10,25 and each mathematical limit.

```bash
python3 experiments/pose_observation_loss/analyze_admission_information_floors.py \
  --output /tmp/admission-information-floors
python3 experiments/pose_observation_loss/plot_admission_information_floors.py \
  --results /tmp/admission-information-floors --output /tmp/information-floors-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. The [fixed protocol](results/admission-information-floors/protocol.md)
preserves every case, fallback and subset. The model already includes temporal
correlation within each batch; independence between repeated batches is an
optimistic assumption. K counts full estimator batches, not samples, correction
events or elapsed time. Actual future observations would also move the centre.

Reuse the pinned nine-heading geometry to bound sufficient and necessary disk
radii. Distinguish a certificate for all poses, a known blocking pose, and the
heading interpolation gap; a blocking pose does not establish that all poses
block. The same-anchor noise-free limit retains anchor variance. A successful
limit need not be achievable at any finite batch count. Recorded truth and error
are evaluation-only, with no new confidence calibration or live outcome claims.

Repeat five CSVs and `sources.json` byte for byte. Compact strategy curves link
to full checkpoint metadata with `base_row`. The optional Matplotlib figure
exports all seven age-0.5 bottlenecks for both inputs, paths and recorded arms.
Geometry, raw-window and physics audits are inherited. See the
[information-floor comparison](results/admission-information-floors/report.md).

### Replay actual centre changes around correction opportunities

Use all 368 recorded timeouts and every scheduled correction or precision
activation in their last 30 seconds. Keep received and missed opportunities,
every control route and both actual/preview inputs. Sample the pre-event input,
the event and +0.18, +0.5 and +1.5 seconds; retain unavailable phases explicitly.

```bash
python3 experiments/pose_observation_loss/analyze_correction_centre_transitions.py \
  --output /tmp/correction-centre-transitions
python3 experiments/pose_observation_loss/plot_correction_centre_transitions.py \
  --results /tmp/correction-centre-transitions --output /tmp/correction-centre-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. Raw trajectories are required at pinned prior
locations; `--raw-root DIR` supports relocated identical cohort directories.
Source/provenance/raw hashes and every replayed window count are checked.
All original 2,120 position checkpoints are also reproduced. Physics/contact/
timer validation is inherited, with no new rollout or control changes.

The [fixed protocol](results/correction-centre-transitions/protocol.md) matches
each input to the preceding physical CSV row. Pair estimated and true movement
to distinguish a centre jump from an actual error change. RMS summaries use
the same available pre/post pairs; empty or censored-only groups keep blank
RMS. Actual releases, resumes and readiness use the recorded control route
and one following 1.5-second interval window, counted once per opportunity.
Both input kinds share these actions and must not be added as separate releases.

Repeat seven CSVs and `sources.json` byte for byte. The optional Matplotlib
figure preserves the full improvement/worsening/tie/censoring partition for
all observed event kinds, both arms/inputs and every post phase. Event
association among repeated, outcome-selected timeouts is descriptive. See the
[recorded correction-centre transitions](results/correction-centre-transitions/report.md).

## Correction gate sequences and uninterrupted evidence

Inspect every recorded decision from the pre-event sample through the following
1.5 seconds for all 2,583 opportunities in the correction-centre analysis.
Reconstruct the actual joint/independent/inactive route, simultaneous failure
sets, and release evidence before the release interval resets its logged timer.
Keep censored end-of-record events, moving events and missed corrections.

```bash
python3 experiments/pose_observation_loss/analyze_correction_gate_sequences.py \
  --output /tmp/correction-gate-sequences
python3 experiments/pose_observation_loss/plot_correction_gate_sequences.py \
  --results /tmp/correction-gate-sequences --output /tmp/correction-gates-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories and the pinned raw trajectories. `--raw-root DIR`
supports relocated identical cohort directories. This audits recorded decisions
and active release timers; full observation/physics validation is inherited.
Every available prior phase and both inputs' shared gate fields are matched,
and the prior action-window counts, delays, movement and runs are reproduced.

The [fixed protocol](results/correction-gate-sequences/protocol.md) distinguishes
conditions seen true at different times, simultaneous readiness, uninterrupted
readiness and actual resume. Intervals are exported per event, including overlaps;
their `event_id` joins the event table and the prior correction-centre tables.
Failure bit positions follow the recorded `requirement_bit_order`; independent
and inactive routes leave absent requirements blank. Marginal failures overlap.
Span boundaries at the observation horizon are not failed requirements.

Repeat seven CSVs and `sources.json` byte for byte. The optional figure includes
all opportunities and a separately labeled joint-start subset, with censoring
and every observed event kind. See the [recorded gate sequences and timer audit](results/correction-gate-sequences/report.md).

## Adjacent correction gate boundaries and nominal commands

Compare every adjacent decision in those same opportunity windows, including
the immediate pre/post pair and unavailable futures at the recorded end.
Keep joint readiness losses, acquisitions and continuations, successful route
exits and all other decisions in the complete transition accounting.

```bash
python3 experiments/pose_observation_loss/analyze_correction_gate_boundaries.py \
  --output /tmp/correction-gate-boundaries
python3 experiments/pose_observation_loss/plot_correction_gate_boundaries.py \
  --results /tmp/correction-gate-boundaries --output /tmp/correction-boundaries-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories; pinned raw trajectories or relocated identical
cohort directories via `--raw-root DIR` are required. The analysis matches all
prior interval gates, reproduces nominal speed-cap arithmetic and keeps actual
paused commands separate. Raw poses use the preceding physical row plus noise
and bias. Recovery and preview poses are the previously validated logged inputs.

The [fixed protocol](results/correction-gate-boundaries/protocol.md) audits exact
speed/recovery/stopping/terminal map geometry at both sides of every adjacent
pair touching joint readiness. Position and error changes keep actual motion,
noise and bias separate. Failure masks preserve simultaneous failed conditions;
geometric depth excludes nominal speed because it has different units.

Repeat nine CSVs and `sources.json` byte for byte. New states link to the prior
`interval_id`; transitions and changes link both states and their event. The
optional figure keeps every post-event joint loss and all observed event kinds,
including zero-loss categories. See the [recorded boundary analysis](results/correction-gate-boundaries/report.md).

## Speed-probe start and first-chord blocks

Separate an unsafe starting pose from an unsafe first 0.05 m chord at all
6,610 previously audited states. Compare raw, recovery, preview and recorded
truth poses with the same nominal steering and speed margin. Preserve all
6,187 detailed adjacent pairs and the full opportunity/case accounting.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_starts.py \
  --output /tmp/speed-probe-starts
python3 experiments/pose_observation_loss/plot_speed_probe_starts.py \
  --results /tmp/speed-probe-starts --output /tmp/speed-probe-starts-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories. The tracked state/geometry tables and map suffice;
raw trajectories are not required again. The [fixed protocol](results/speed-probe-starts/protocol.md)
preserves the speed probe's inclusive blocking equality, separate obstacle/wall
witnesses, start-first precedence, and same-margin truth classifications.
Positive first chords do not imply a clear whole path or new successful release.

Pose queries join the prior `state_id` and `source_interval_id`; three input
views of each source `transition_id` share one recorded action. Keep these
views separate when counting, including the fixed 550 raw-zero-loss pairs
and 509 command-only pairs. Non-audited opportunities remain explicit.

Repeat seven CSVs and `sources.json` byte for byte. The optional figure keeps
the complete raw-zero-loss denominator in both arms; its numeric data includes
empty event-kind groups. See the [start and first-chord comparison](results/speed-probe-starts/report.md).

## First-chord position and heading envelopes

Audit conditional first-chord certificates over a position disk and heading
interval at all 6,610 recorded states. Freeze six disk radii (0 to .30 m) and
two heading bounds (0 and 4 degrees), retaining every raw/recovery/preview
input and the fixed 550 raw-zero-loss / 509 command-only pairs.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_envelopes.py \
  --output /tmp/speed-probe-envelopes
python3 experiments/pose_observation_loss/plot_speed_probe_envelopes.py \
  --results /tmp/speed-probe-envelopes --output /tmp/speed-probe-envelopes-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories; the pinned tracked artifacts suffice. The
[fixed protocol](results/speed-probe-envelopes/protocol.md) defines conservative
Lipschitz bounds, separate clear/blocked/unresolved classifications and exact
position/heading coverage. A certificate applies to its specified pose set;
these radii have no calibrated confidence level. True errors provide a
separately labeled oracle diagnostic, unavailable to an online controller.

Repeat five CSVs and provenance byte for byte. Pose IDs join speed-probe-starts;
individual decisions retain original loss transition/event IDs. Summaries
include all audited states, both fixed loss populations and empty event kinds.
The optional figure shows the entire 550-loss population in both arms and
all three inputs. See the [envelope and truth comparison](results/speed-probe-envelopes/report.md).

## Envelope coverage components and audited persistence

Separate position-only, heading-only and simultaneous coverage failures for
every prior pose/setting. Measure persistence using explicit audited adjacent
pairs, keeping gaps and case boundaries. Each run's elapsed support is its
last minus first decision time; audit boundaries truncate the observation.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_coverage_spans.py \
  --output /tmp/speed-probe-coverage-spans
python3 experiments/pose_observation_loss/plot_speed_probe_coverage_spans.py \
  --results /tmp/speed-probe-coverage-spans --output /tmp/speed-probe-coverage-spans-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and the tracked pinned inputs. The [fixed protocol](results/speed-probe-coverage-spans/protocol.md)
keeps all 6,610 states, six radii, two heading bounds and both fixed loss
populations. It preserves every prior certificate and coverage marginal.
Missing audit edges split chains even at consecutive selected timestamps;
repeated references to the same physical edge count time once.

Loss membership rows join original transition/event/pose IDs to coverage and
clear-conflict spans. Each span family separately partitions every chain.
The full span is retained for event-kind union summaries; these groups can
overlap. No_conflict runs and empty groups remain explicit.

Repeat eight CSVs and provenance byte for byte. The optional figure compares
the entire 550-loss coverage population and all audited conflict spans at
.10 m / +/-4 degrees. See the [coverage and persistence results](results/speed-probe-coverage-spans/report.md).

## Position-error components within audited probe spans

Replay all 368 timeout histories and decompose every one of the 19,830 audited
raw/recovery/preview position errors into observation noise, correction-anchor
offset, within-anchor drift and motion transport. The component vectors and
their signed projections add to the error; component norms do not add.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_error_components.py \
  --output /tmp/speed-probe-error-components
python3 experiments/pose_observation_loss/plot_speed_probe_error_components.py \
  --results /tmp/speed-probe-error-components --output /tmp/speed-probe-error-components-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new output directories and the pinned raw trajectories. `--raw-root DIR`
supports relocated identical cohort directories. All 3,312,000 intervals
check estimator-window counts; every target input matches the tracked pose
and its preceding physical truth. Per-group weights and anchor IDs remain
available. This adds no new controller or geometry.

The [fixed protocol](results/speed-probe-error-components/protocol.md) retains
the full 550/509 loss populations and every original loss membership. All
default-setting spans in both families receive mean-vector and centered-RMS
diagnostics, including no_conflict spans and audit truncation. Within-span
moments describe selected recorded samples, without a calibrated confidence
level or a causal clearance attribution.

Repeat eight CSVs and provenance byte for byte. The optional figure keeps
all 550 loss rows and separately all audited clear-conflict spans, with
signed projections and state-weighted span moments. See the
[error-component results](results/speed-probe-error-components/report.md).

## Shared information and position agreement

Compare all three raw/recovery/preview pairings at every audited state. Use
the recorded observation weights, actual sample spacing and shared anchor IDs
to calculate partial noise-plus-anchor covariance and difference uncertainty.
Position agreement at .02, .05 and .10 m is kept separate from each input's
.10 m position error relative to truth.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_shared_information.py \
  --output /tmp/speed-probe-shared-information
python3 experiments/pose_observation_loss/plot_speed_probe_shared_information.py \
  --results /tmp/speed-probe-shared-information --output /tmp/speed-probe-shared-information-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories; the pinned tracked components and group metadata suffice.
The [fixed protocol](results/speed-probe-shared-information/protocol.md) retains
all 6,610 states and all 550/509 loss members for every pairing, including
shared-single-anchor and three-epoch preview modes. The full three-input
covariance matrix is checked at each state. The model excludes drift and
nonlinear transport; it is not a calibrated posterior or a release condition.

Seven CSVs and provenance repeat byte for byte. Per-anchor weights and source
pose IDs make shared offsets explicit; loss links retain original transition
and event IDs. The figure uses the entire 550-loss population in both arms.
See the [shared-information results](results/speed-probe-shared-information/report.md).

## Agreement persistence and temporal shared information

Partition the original audited chains into maximal agreement-category spans
for all three pairings, retaining every state and explicit edge. Within each
span, split blocks when an anchor-ID/weight signature changes. Calculate the
partial variance of each block's mean using the actual repeated observation
weights, AR covariance and common anchors; compare with independent times.

```bash
python3 experiments/pose_observation_loss/analyze_speed_probe_temporal_information.py \
  --output /tmp/speed-probe-temporal-information
python3 experiments/pose_observation_loss/plot_speed_probe_temporal_information.py \
  --results /tmp/speed-probe-temporal-information --output /tmp/speed-probe-temporal-information-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories; tracked pinned inputs suffice. The
[fixed protocol](results/speed-probe-temporal-information/protocol.md) keeps
four agreement rules, all five categories, singleton and audit-truncated spans,
all 550/509 loss rows and empty summary groups. Time support is last minus
first decision time; missing audit edges never imply continuity.

Repeat nine CSVs and provenance byte for byte. Loss memberships preserve all
three pairings and four rules. Pooled sigmas weight blocks equally, separately
from state and loss-row counts. Common offsets persist in a temporal mean;
the partial model excludes drift/transport and is not calibrated. Observed
mean errors use known truth for offline arithmetic, without changing point
labels or recorded control. The figure keeps the full .05 m span partition
and separate input/difference temporal-model comparisons. See the
[temporal-information results](results/speed-probe-temporal-information/report.md).

## Information retained across correction opportunities

Replay the original 368 timeout histories at every saved correction snapshot.
Match the pinned positions, preceding truth, estimator windows and error
components, then reconstruct exact observation steps and anchor weights.
Compare pre with immediate, +.18 s, +.50 s and +1.50 s for both inputs at all
2,583 opportunities, retaining received/missed/precision events and censoring.

```bash
python3 experiments/pose_observation_loss/analyze_correction_information_renewal.py \
  --output /tmp/correction-information-renewal
python3 experiments/pose_observation_loss/plot_correction_information_renewal.py \
  --results /tmp/correction-information-renewal --output /tmp/correction-information-renewal-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and pinned raw histories; `--raw-root DIR` supports the
same four cohort directories at a relocated path. The
[fixed protocol](results/correction-information-renewal/protocol.md) preserves
all 25,830 source snapshot references and 20,664 requested pre/post pairs.
Exact group sample-step/sigma lists retain window overlap and AR correlation.
Shared anchors, the event's new anchor and other older anchors are distinct.
Unavailable outcomes have blank model fields rather than zero error.

Repeat seven CSVs and provenance byte for byte. The optional figure keeps both
arms, both inputs, every post phase and all requested pairs, including censored
counts; its numeric data include empty event kinds. Partial noise-plus-anchor
variance is not calibrated and excludes drift/nonlinear transport. Error
change subtracts truth displacement from pose change. Recorded actions remain
fixed. See the [information-renewal results](results/correction-information-renewal/report.md).

## Contrasts and remaining common error

Use each pinned pair's partial covariance to project its error midpoint onto
its contrast, retaining the remaining common error and all four signed error
components. Simultaneous contrasts cancel the shared truth. Lagged contrasts
subtract known truth displacement and are offline oracle diagnostics.

```bash
python3 experiments/pose_observation_loss/analyze_contrast_common_errors.py \
  --output /tmp/contrast-common-errors
python3 experiments/pose_observation_loss/plot_contrast_common_errors.py \
  --results /tmp/contrast-common-errors --output /tmp/contrast-common-errors-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories; tracked inputs suffice. The
[fixed protocol](results/contrast-common-errors/protocol.md) keeps all 19,830
simultaneous pairs, 20,664 requested lagged pairs, 550/509 original losses,
source point labels and unavailable outcomes. Projection coefficients come
only from covariance, retaining negative weights and an explicit near-zero
contrast branch. Identical inputs retain their common variance.

Repeat seven CSVs and provenance byte for byte. Both complete summary families
include empty event kinds. The figure retains the complete 550-loss population
for all three simultaneous pairings and all requested lagged phases/inputs,
with missing counts explicit. This partial second-moment decomposition is not
a calibrated posterior or a new position estimator. Observed residual RMS is
separate from model sigma. See the [contrast/common-error results](results/contrast-common-errors/report.md).

## Retrospective transfer of risk diagnostics

Use fixed contrast norm and midpoint/residual model sigma scores to rank
large observed midpoint/residual errors. Fit lower weighted score quantiles
on known seeds 269/270/271 only; evaluate those same cutoffs on the existing
new seeds 274/275/276. Both cohorts were observed in earlier analyses, so this
is retrospective transfer. The evaluation cohort is not untouched validation.

```bash
python3 experiments/pose_observation_loss/analyze_contrast_risk_transfer.py \
  --output /tmp/contrast-risk-transfer
python3 experiments/pose_observation_loss/plot_contrast_risk_transfer.py \
  --results /tmp/contrast-risk-transfer --output /tmp/contrast-risk-transfer-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and tracked pinned inputs. The
[fixed protocol](results/contrast-risk-transfer/protocol.md) keeps every
simultaneous and requested lagged source reference, both arms/noise types,
all eleven views and empty event kinds. Each represented case has total
weight one before retention; selected weights are not renormalized per case.
Scores use fixed 1e-8 m integer ticks, exact tied quantiles and half-credit AUC
ties. Thresholds never consult new-cohort scores or outcome labels.

Keep all 25/50/75% quantile rules and all_available, including achieved
coverage, raw counts, case mass, missing data and both error outcomes. Retained
diagnostic rows are not released actions. Lagged contrast is an offline oracle
quantity. Model sigmas are uncalibrated. Repeat seven CSVs/provenance and both
figures byte for byte. See the [risk-transfer results](results/contrast-risk-transfer/report.md).

## Estimator composition and cutoff transfer

Decompose the preceding simultaneous fixed-cutoff transfer by the recorded
Recovery, filter and three-anchor fusion modes. Reuse every fitted cutoff;
keep the parent case weights within strata and after selection. Report the
change in estimator mixture separately from within-mode rate changes, with
explicit support checks and both error targets. These are descriptive,
order-dependent comparisons of previously observed seed cohorts.

```bash
python3 experiments/pose_observation_loss/analyze_estimator_mode_transfer.py \
  --output /tmp/estimator-mode-transfer
python3 experiments/pose_observation_loss/plot_estimator_mode_transfer.py \
  --results /tmp/estimator-mode-transfer --output /tmp/estimator-mode-transfer-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and the pinned tracked inputs. The
[protocol](results/estimator-mode-transfer/protocol.md) retains all three
simultaneous views, noise types, arms, event kinds, scores, budgets and both
midpoint/residual outcomes. Lagged references remain inherited outside this
stratification. All parent retention aggregates must match the preceding
study. Repeat seven CSVs/provenance and both SVG/PNG figures with their numeric
series/provenance byte for byte. See the
[estimator-mode results](results/estimator-mode-transfer/report.md).

## Observed error components within estimator modes

Apply the existing midpoint/residual weights to each input's recorded noise,
anchor, drift and transport vectors. Compare observed error energies with the
partial model's two-axis 2V reference, preserving signed cancellation terms,
source estimator strata, parent case weights and every existing score cutoff.
The additional component-only outside labels are arithmetic diagnostics in
the saved frame; they do not simulate a different controller trajectory.

```bash
python3 experiments/pose_observation_loss/analyze_estimator_mode_components.py \
  --output /tmp/estimator-mode-components
python3 experiments/pose_observation_loss/plot_estimator_mode_components.py \
  --results /tmp/estimator-mode-components --output /tmp/estimator-mode-components-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and pinned tracked inputs. The
[protocol](results/estimator-mode-components/protocol.md) keeps both targets,
all simultaneous views, scores, cutoffs, event kinds and pooled/structural
strata. Original observed risks/AUC must reproduce the preceding tables.
Repeat eight CSVs/provenance and both SVG/PNG figures/numeric series/provenance
byte for byte. See the
[component results](results/estimator-mode-components/report.md).

## Known-only cutoffs conditional on estimator mode

Compare the existing pooled score cutoffs with quantiles fitted separately
for each source estimator mode. Known seeds alone fit both shared-arm rules.
Preserve the parent case weights before mode conditioning and after selection,
and keep all score ties. Compare coverage, both observed error outcomes and
which records are newly retained or removed. Empty mode fits stay explicit.

```bash
python3 experiments/pose_observation_loss/analyze_mode_conditional_cutoffs.py \
  --output /tmp/mode-conditional-cutoffs
python3 experiments/pose_observation_loss/plot_mode_conditional_cutoffs.py \
  --results /tmp/mode-conditional-cutoffs --output /tmp/mode-conditional-cutoffs-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and pinned tracked inputs. The
[protocol](results/mode-conditional-cutoffs/protocol.md) keeps every simultaneous
view, score, budget, structural stratum and event kind, with both targets.
Original pooled metrics must match the preceding component study. Repeat
seven CSVs/provenance and both SVG/PNG figures/series/provenance byte for byte.
Both seed cohorts were observed previously; conditional quantiles do not
guarantee evaluation coverage or error risk. See the
[conditional-cutoff results](results/mode-conditional-cutoffs/report.md).

## Whole-known-seed exclusion checks

Omit each known seed across both arms/noise types and all of its recorded
cases. Refit pooled/mode-conditional cutoffs on the other two known seeds;
evaluate the omitted seed and the same existing new cohort separately.
Preserve missing mode fits and original case weights. A three-fold range
is reported only when every value is defined.

```bash
python3 experiments/pose_observation_loss/analyze_seed_exclusion_cutoffs.py \
  --output /tmp/seed-exclusion-cutoffs
python3 experiments/pose_observation_loss/plot_seed_exclusion_cutoffs.py \
  --results /tmp/seed-exclusion-cutoffs --output /tmp/seed-exclusion-cutoffs-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py'
```

Use new directories and pinned tracked inputs. The
[protocol](results/seed-exclusion-cutoffs/protocol.md) keeps all simultaneous
views/scores/budgets/event kinds/strata and both targets. Every all-known fit
and original comparison must reproduce the preceding study. Repeat twelve
CSVs/provenance and both SVG/PNG figures/series/provenance byte for byte.
Both cohorts were already observed, and repeated existing-new evaluations
do not add independent cases. See the
[seed-exclusion results](results/seed-exclusion-cutoffs/report.md).

## Additional seeds and an immediate timeout-mode census

The [fixed protocol](results/fresh-seed-mode-census/protocol.md) collects seeds
277/278/279 with the existing straight/curved controller, both noise models,
all loss modes and the same maps/planners: 324 trajectories, 162 paired cases.
It counts every immediate correction/precision opportunity at logged 150--180 s
in every timeout, and compares this census with the same population in the
existing 648 trajectories. This is a wider population than the earlier selected
speed-probe audit. The additional seeds do not refit its cutoffs or establish
their transfer.

With the saved Rust toolchain active and cached dependencies, run:

```bash
export CARGO_NET_OFFLINE=true
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 277 278 279 --noise-correlation-time 0 \
  --output /tmp/fresh-seed-mode-census/white
python3 experiments/pose_observation_loss/run.py \
  --terminal-arc-comparison experiments/pose_observation_loss/results/planning-clearance/fixtures \
  --seeds 277 278 279 --noise-correlation-time 1 \
  --output /tmp/fresh-seed-mode-census/correlated
python3 experiments/pose_observation_loss/repeat_fresh_seed_case.py \
  --raw-root /tmp/fresh-seed-mode-census
python3 experiments/pose_observation_loss/analyze_fresh_seed_modes.py \
  --raw-root /tmp/fresh-seed-mode-census --output /tmp/fresh-seed-mode-summary
MPLCONFIGDIR=/tmp/fresh-seed-mpl python3 experiments/pose_observation_loss/plot_fresh_seed_modes.py \
  --results /tmp/fresh-seed-mode-summary --output /tmp/fresh-seed-mode-figure
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Every output directory must be new. The repeat helper uses the default Cargo
release binary; pass `--binary PATH` when using a different target directory.
It repeats the first white `epoch-none-277` nine-case invocation and requires
identical complete trajectory/result CSV bytes before writing its command and
binary fingerprint. The analyzer requires both completed run.py verifications,
audits all physical transitions/contact/bounds and paired input prefixes, and
replays every timeout's observation history. It validates every immediate
pose/count against the original records. Historical comparison needs only the
pinned repository CSVs, not the original raw histories.

`accounting.csv` preserves every old/fresh case. `events.csv` retains all event
categories and actual/preview modes; `case-support.csv` exposes mixed-mode and
no-event cases. `summary.csv` includes all per-seed/cohort, arm, loss-mode and
event-kind groups, with blank empty fractions. Parent-case-weighted shares use
1/n_events within each reporting group before conditioning on preview mode.
Filter/Fusion case counts can overlap. `fresh-pairs.csv`, `fresh-outcomes.csv`
and `verification.csv` preserve the complete new performance and audit results.

Repeat analysis and plotting into new directories and compare every file byte
for byte with the first outputs. Within a fixed workspace the output paths do
not enter artifacts; source raw paths, invocation paths and raw provenance are
intentionally recorded, so regenerating raw runs elsewhere changes provenance
fingerprints. CSV/SVG/JSON are saved in the repository; large raw CSVs and PNG
previews remain in scratch. See the [fresh-seed results](results/fresh-seed-mode-census/report.md).

## Transfer the fixed cutoffs under the original audit selection

The [protocol](results/fresh-seed-cutoff-transfer/protocol.md) applies the original
joint-ready adjacent-decision selection to the preserved additional-seed
histories. It first reconstructs all historical audit states, pair projections,
score ticks, source-event memberships and comparison metrics. Only after that
regression passes does it evaluate 277/278/279 using the unchanged pooled and
mode-conditional thresholds trained on 269/270/271. No cutoff is refitted.

```bash
python3 experiments/pose_observation_loss/analyze_fresh_seed_cutoffs.py \
  --output /tmp/fresh-seed-cutoff-transfer
MPLCONFIGDIR=/tmp/fresh-cutoff-mpl python3 experiments/pose_observation_loss/plot_fresh_seed_cutoffs.py \
  --results /tmp/fresh-seed-cutoff-transfer --output /tmp/fresh-seed-cutoff-figures
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Outputs must be new. Analysis uses only the standard library and needs the
preserved historical and additional-seed raw trajectories at the paths recorded
in `correction-centre-transitions/sources.json` and
`fresh-seed-mode-census/sources.json`. It verifies provenance and every consumed
raw hash. Those histories are already present in this cloud workspace; plotting
needs Matplotlib. This is a replay of existing runs, with physics/live outcomes
inherited from their verified live studies.

`accounting.csv` retains all 972 cases and `case-support.csv` distinguishes
Filter-only/Fusion-only/mixed/no-audit cases on exactly the original selection.
`scores.csv`, `poses.csv` and `projections.csv` expose the new model and observed
quantities; `loss-links.csv` preserves every selected raw-zero/command-only loss
reference. `events.csv` retains all categories and source IDs. Immutable copies
of the 264-row pooled table (72 simultaneous rules used here) and 216
conditional rules accompany the complete
`comparison.csv` grid: all three views, scores, budgets, event kinds and strata,
including an explicit recovery-fallback stratum with missing training support.
Whole-group conditional metrics stay blank when any represented mode lacks a
fit; all_available remains defined. Parent case weights precede both mode
conditioning and retention.

Repeat analysis and plotting into new directories and compare all files byte
for byte. Source raw paths are preserved, while analysis output paths do not
enter artifacts. Historical regression evidence, raw/helper/protocol hashes
and scope are saved in `sources.json`; plots also export their numeric inputs.
The additional raw trajectories and their broad mode census were previously
observed, which limits claims about untouched validation. See the
[original-audit transfer results](results/fresh-seed-cutoff-transfer/report.md).

## Additional-seed error components and fixed-direction rankings

The [protocol](results/fresh-seed-components/protocol.md) diagnoses the same
13,155 additional-seed pairs using only tracked score/pose/projection artifacts.
For midpoint and residual targets it preserves the four signed input-error
components, their cross terms, and the partial-model two-axis energy reference.
Both the immutable pooled and mode-conditional rules are retained, with no
refitting or new simulation.

```bash
python3 experiments/pose_observation_loss/analyze_fresh_seed_components.py \
  --output /tmp/fresh-seed-components
MPLCONFIGDIR=/tmp/fresh-component-mpl python3 experiments/pose_observation_loss/plot_fresh_seed_components.py \
  --results /tmp/fresh-seed-components --output /tmp/fresh-seed-component-figures
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Every output directory must be new. Aggregation needs only Python's standard
library and repository files; plotting needs Matplotlib/NumPy. Raw trajectories,
Rust compilation and physics replay are unnecessary. The collector checks
case/state/time/event/estimator joins, both saved observed labels, component
closure and every comparable count/mass/fraction/RMS in the preceding cutoff
results. Original source artifacts and all cutoff bytes remain unchanged.

`component-summary.csv` preserves every view, score, budget, event kind,
estimator stratum, method and target, with parent case weights carried through
mode conditioning and retention. Empty energies/fractions stay blank. Unknown
conditional fits make whole-group metrics undefined, while all_available keeps
available records. The copied pooled table has 264 rows (72 simultaneous rules
used here); all 216 conditional rows are retained.

`ranking.csv` uses all_available records and the fixed high-score direction
against observed, noise-plus-anchor and drift-plus-transport outside labels.
Ties receive half credit; one-class AUCs stay blank. `reconstruction.csv` records
per-pair closure checks. Accounting, case support, all opportunities and loss
links are copied unchanged. Plot numeric CSVs expose all signed energy terms,
source/retained counts and masses, and AUCs including undefined values.

Repeat aggregation and plotting into new directories and compare every file
byte for byte. This is retrospective arithmetic diagnosis in the saved frame,
with repeated states and three additional seed streams; component removal is
not a counterfactual rollout or evidence of calibration. See the
[additional-seed component results](results/fresh-seed-components/report.md).

## Additional-seed case and seed influence

The [fixed protocol](results/fresh-seed-case-influence/protocol.md) traces the
same Recovery/Preview states for both targets, all three scores and all original
strata. It keeps every .10 m E/P boundary cell and the original full-parent case
weights through single/omitted case and single/omitted seed subsets. This is
retrospective influence diagnosis of the existing additional-seed artifacts.

```bash
python3 experiments/pose_observation_loss/analyze_fresh_seed_case_influence.py \
  --output /tmp/fresh-seed-case-influence
MPLCONFIGDIR=/tmp/fresh-influence-mpl python3 experiments/pose_observation_loss/plot_fresh_seed_case_influence.py \
  --results /tmp/fresh-seed-case-influence --output /tmp/fresh-seed-case-figures
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output directories. Aggregation uses only the Python standard library
and tracked source artifacts; plotting needs Matplotlib/NumPy. No raw-history
access, Rust build, new trajectory or cutoff refitting is needed.

`cases.csv` maps stable IDs to all 57 full case keys. `boundary-trace.csv` keeps
all 8,770 state/target rows, full joins, signed margins, true component vectors,
fixed score ticks and inherited weights. `component-influence.csv` and
`ranking-influence.csv` retain all subsets, including absent seeds, empty groups,
one-class AUCs and both crossing directions. `case-pairs.csv` decomposes every
complete-group AUC into ordered positive-case/negative-case contributions;
sum numerators and denominators before division rather than averaging AUCs.
Original accounting, support, opportunities, links and cutoffs are copied intact.

All five figures have SVG/PNG versions and complete numeric CSVs. Repeat analysis
and plotting into new directories and compare every artifact byte for byte.
Provenance pins inputs, helpers, protocol and plotted values. Case/seed omission
ranges are deterministic sensitivities, not confidence intervals or untouched
holdouts; repeated states/planners do not create independent seed streams.
See the [case and seed influence results](results/fresh-seed-case-influence/report.md).

## Ranking comparisons within each seed

The [fixed protocol](results/seed-conditioned-rankings/protocol.md) keeps the
known 269/270/271, previous additional 274/275/276 and latest additional
277/278/279 cohorts separate. It decomposes Recovery/Preview AUC into every
ordered positive-seed/negative-seed block with unchanged parent case weights,
both targets, all three scores and E/P/U labels.

```bash
python3 experiments/pose_observation_loss/analyze_seed_conditioned_rankings.py \
  --output /tmp/seed-conditioned-rankings
MPLCONFIGDIR=/tmp/seed-rank-mpl python3 experiments/pose_observation_loss/plot_seed_conditioned_rankings.py \
  --results /tmp/seed-conditioned-rankings --output /tmp/seed-ranking-figures
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output directories. Aggregation needs only Python's standard library
and tracked source CSV/JSON files; plotting needs Matplotlib/NumPy. The existing
historical/latest reconstruction adapters run their original signed-component,
label and join checks. No raw histories, Rust compilation, new simulation,
fitting or cutoff changes are needed.

`component-summary.csv` matches the prior complete-group components;
`seed-support.csv` retains all three seeds, including empty/one-class groups.
`ranking-summary.csv` preserves full AUC, AUC from comparisons within each seed,
comparison-mass shares and supported seed counts. `seed-blocks.csv` exposes all
nine ordered blocks per group/score/label. Sum diagonal numerators/denominators
to obtain the within-seed value; never average seed AUCs or give seeds new equal
weights. AUC stays blank without positive-negative support.

The full accounting/support/opportunity/cutoff tables and separate historical
and latest loss-link tables are copied unchanged. Repeat aggregation and plotting
into new directories and compare every file byte for byte. Four SVG/PNG figures
have complete numeric exports and source hashes. These are retrospective
comparison diagnostics with three observed seed values per cohort: changing
the comparison population is not estimator repair or independent validation.
See the [seed comparison results](results/seed-conditioned-rankings/report.md).

## Planner families and exact diagnostic windows

The [fixed protocol](results/planner-window-sensitivity/protocol.md) audits all
972 cases and 324 planned planner families while reusing the same observed
Recovery/Preview states. A family removes only planner from the full case key.
Exact complete windows must match times, event semantics, both targets' signed
components/vectors/labels, partial variances, coefficients and all score ticks.
There is no tolerance or partial-window matching, and equality concerns saved
diagnostics rather than full physical trajectories.

```bash
python3 experiments/pose_observation_loss/analyze_planner_window_sensitivity.py \
  --output /tmp/planner-window-sensitivity
MPLCONFIGDIR=/tmp/planner-window-mpl python3 experiments/pose_observation_loss/plot_planner_window_sensitivity.py \
  --results /tmp/planner-window-sensitivity --output /tmp/planner-window-figures
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output directories. Aggregation needs Python's standard library and
tracked inputs; plotting needs Matplotlib/NumPy. No raw histories, Rust build,
new simulation, fitting or cutoff changes are needed. Original case weighting
is checked against every preceding component, ranking and seed-block value.

`families.csv` and `cases.csv` keep all planned/represented keys, exact-window
hashes, missing support and row weights. The original accounting partition
`existing_new` is retained as `accounting_partition` while the analysis group is
`new`. Case/window/family weights divide 1/n_case by respectively one, the number
of identical complete-window cases, or represented cases in the family, before
mode/seed conditioning. All observations and labels remain present.

`component-summary.csv`, `ranking-summary.csv` and `seed-blocks.csv` preserve
every weighting, target, score, label and stratum; `unit-summary.csv` records
parent counts/masses. Mass columns explicitly use `analysis_mass` because their
unit changes with the weighting. All original accounting, support, opportunities,
cutoffs and separate historical/latest loss links are copied unchanged.

Repeat aggregation and plotting and compare all artifacts byte for byte. Five
SVG/PNG figures export their full numeric inputs and provenance. Weighting units
are not independent sample counts; unequal windows do not establish independence.
See the [planner weighting results](results/planner-window-sensitivity/report.md).

## Planner-specific rankings on common family support

The [fixed protocol](results/planner-conditioned-rankings/protocol.md) compares
mixed planners and A*/JPS/Theta* separately, using the original 1/n_case weights
before any selection. `source` retains all available states; `common_family`
requires all three planners in that same stratum and full case family. Eligibility
uses availability, never labels or scores. Matching keeps family membership;
times, state counts and inherited mode fractions can still differ.

```bash
python3 experiments/pose_observation_loss/analyze_planner_conditioned_rankings.py \
  --output /tmp/planner-conditioned-rankings
MPLCONFIGDIR=/tmp/planner-conditioned-mpl python3 experiments/pose_observation_loss/plot_planner_conditioned_rankings.py \
  --results /tmp/planner-conditioned-rankings --output /tmp/planner-conditioned-figures
python3 experiments/pose_observation_loss/verify_planner_conditioned_rankings.py \
  --results /tmp/planner-conditioned-rankings --figures /tmp/planner-conditioned-figures \
  --output /tmp/planner-conditioned-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output directories/evidence files. Aggregation needs only the standard
library and tracked inputs; plots need Matplotlib/NumPy. The independent verifier
needs NumPy and imports no analysis helpers. It reconstructs vectors and labels,
audits every planned family/stratum, and directly evaluates weighted pairs and
numeric figure inputs. No raw histories, new simulations, Rust build, fitting or
cutoff changes are needed.

`family-support.csv` keeps every planned family, including zero support;
`component-summary.csv`, `ranking-summary.csv` and `seed-support.csv` retain all
populations, planners, strata, targets and E/P/U labels. `planner-blocks.csv`
contains all nine ordered positive/negative planner pairs and their within-seed
numerators/denominators. Rankings include full, within-seed, within-planner and
jointly within-seed-and-planner AUCs and comparison shares. Sum numerators and
denominators before division. Missing-class AUC stays blank.

Original accounting/support/events/cutoffs/loss links and prior complete
case/family audits are copied unchanged. Six SVG/PNG figures have full numeric
exports and hashes. Repeat aggregation/plotting and compare all artifacts byte
for byte. Planner matching can remove class support or raise a descriptive AUC;
neither supplies independent validation or estimator repair.
See the [planner-conditioned results](results/planner-conditioned-rankings/report.md).

## Ranking comparisons within cases and exact windows

The [fixed protocol](results/case-window-rank-decomposition/protocol.md) divides
every mixed-planner positive/negative comparison into five disjoint bins: same
case, another case with the same exact complete window, another window in the
same family, another family at the same seed, or another seed. Original 1/n_case
weights and source/common-family populations remain unchanged. Cumulative bins
give within-case/window/family/seed AUCs; sum numerators and denominators before
division, never average individual case AUCs.

```bash
python3 experiments/pose_observation_loss/analyze_case_window_ranks.py \
  --output /tmp/case-window-ranks
MPLCONFIGDIR=/tmp/case-window-mpl python3 experiments/pose_observation_loss/plot_case_window_ranks.py \
  --results /tmp/case-window-ranks --output /tmp/case-window-figures
python3 experiments/pose_observation_loss/verify_case_window_ranks.py \
  --results /tmp/case-window-ranks --figures /tmp/case-window-figures \
  --output /tmp/case-window-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output directories and a new evidence filename, without Python -O.
Aggregation needs only the standard library and tracked inputs; figures need
Matplotlib/NumPy and independent verification needs NumPy. No raw histories,
new simulations, Rust build, fitting or cutoff changes are needed.

`case-strata.csv` audits all 972 planned cases in all five strata, with both
targets' class support, inherited mass and common-family eligibility. Missing
parent histories have blank row weights and zero observations. `case-rankings.csv`
keeps all 157 represented parent cases across every stratum/target/score/label,
including empty strata; eligibility specifies whether each unchanged case
survives common-family selection. `ranking-summary.csv` retains previous
mixed-planner metrics and adds within-case/window/family and between-case results.
`hierarchy-bins.csv` exposes all five bin masses, AUCs and contributions, even
when empty. An absent-class AUC stays blank.

Full accounting/support/events/cutoffs/loss links and prior case/family/stratum
audits are copied unchanged. All prior mixed-ranking fields and latest-only
within/between-case masses are checked. The independent verifier imports no
analysis helpers and uses direct state-pair masks to check every new field.
Five SVG/PNG figures export both targets' complete numeric inputs and hashes.
Repeat aggregation/plotting byte for byte. Restricted comparisons and repeated
cases/windows remain retrospective descriptions with limited class support.
See the [case/window comparison results](results/case-window-rank-decomposition/report.md).

## Rankings within saved source events

The [fixed protocol](results/event-conditioned-rankings/protocol.md) splits
comparisons into the same saved event, other events in the same case, and other
cases. Each RP state has one census event_id; the full case/event join is checked.
The ID is a diagnostic reference tag and does not enumerate Fusion's complete
information ancestry. Original case weights and source/common-family populations
remain unchanged. Event AUCs combine exact pair masses, without event weighting.

```bash
python3 experiments/pose_observation_loss/analyze_event_conditioned_rankings.py \
  --output /tmp/event-conditioned-rankings
MPLCONFIGDIR=/tmp/event-ranking-mpl python3 experiments/pose_observation_loss/plot_event_conditioned_rankings.py \
  --results /tmp/event-conditioned-rankings --output /tmp/event-ranking-figures
python3 experiments/pose_observation_loss/verify_event_conditioned_rankings.py \
  --results /tmp/event-conditioned-rankings --figures /tmp/event-ranking-figures \
  --output /tmp/event-ranking-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new directories and a new evidence filename, without Python -O. Aggregation
uses the standard library and tracked inputs. Plotting needs Matplotlib/NumPy;
the independent NumPy verifier imports no analyzers. No raw histories, new
simulations, Rust build, fitting or cutoff changes are needed.

`source-event-map.csv` traces every RP state to its event, times and inherited
weight. `event-coverage.csv` retains the entire census, including events with no
RP references; this does not mean a received observation was missing.
`event-support.csv` keeps all five strata of represented events, including zero
support. `event-rankings.csv` saves only nonempty event/stratum groups across both
targets, all scores and E/P/U; eligibility determines common-family inclusion.
Empty groups stay explicit in support and absent-class AUCs stay blank.

`ranking-summary.csv` preserves every preceding aggregate field and adds event
metrics/counts. `event-pair-bins.csv` exposes all three exact bin masses and
full-denominator contributions. The first two bins additionally report shares
of within-case pair mass. Sum numerators and denominators before division;
never average individual event AUCs or infer independent sample sizes.

All prior accounting/support/events/cutoffs/loss links and planned case/family/
stratum audits are copied unchanged. Five SVG/PNG figures export both targets
and full/conditional shares; nonzero shares below .1% are distinguished from
zero in the focused AUC panels. Repeat aggregation/plotting byte for byte.
See the [saved-event ranking results](results/event-conditioned-rankings/report.md).

## Time proximity within saved source events

The [fixed protocol](results/event-temporal-rankings/protocol.md) restricts the
same full-case/event comparisons to absolute saved state-time gaps of
<=20/100/500 ms. Integer microseconds make the upper endpoints inclusive;
inputs farther than 1e-7 seconds from that grid are rejected. Original case
weights, common-family eligibility and every previous aggregate field remain
unchanged. Time proximity is symmetric in time and does not define forecasts,
continuous chains or independent observations.

```bash
python3 experiments/pose_observation_loss/analyze_event_temporal_rankings.py \
  --output /tmp/event-temporal-rankings
MPLCONFIGDIR=/tmp/event-temporal-mpl python3 experiments/pose_observation_loss/plot_event_temporal_rankings.py \
  --results /tmp/event-temporal-rankings --output /tmp/event-temporal-figures
python3 experiments/pose_observation_loss/verify_event_temporal_rankings.py \
  --results /tmp/event-temporal-rankings --figures /tmp/event-temporal-figures \
  --output /tmp/event-temporal-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new directories and a new evidence filename, without Python -O. Aggregation
needs the standard library; figures use Matplotlib/NumPy. The independent NumPy
verifier imports no analyzers, reconstructs raw error labels/scores and uses
Decimal on original projection timestamps plus direct pair masks. No raw
rollouts, new simulations, Rust build, fitting or cutoff changes are required.

`source-time-audit.csv` preserves each saved time, integer time and rounding
error. `event-time-support.csv` keeps each nonempty event/stratum/target/label's
class support and time-bin pair counts/masses, independent of score. The copied
`event-support.csv` still exposes empty strata. `temporal-pair-bins.csv` retains
all four disjoint gap bins, including zero support; their exact pair masses sum
to the prior same-event comparison. `ranking-summary.csv` adds three cumulative
horizons, supported event/case counts and shares of both same-event and full
comparison mass. Sum masses before division; missing-class AUCs remain blank.

All prior census/accounting/cutoffs/loss references and event/case/family audits
are preserved. Repeat all 20 CSVs and provenance byte for byte. Five SVG/PNG
figures export complete numeric inputs for both targets; tiny nonzero shares
are distinguished from zero. Small supported counts and comparison mass limit
the conclusions. See the [time-proximity results](results/event-temporal-rankings/report.md).

## Saved-score ties and conditional concordance

The [fixed protocol](results/event-score-ties/protocol.md) separates ties in the
original 1e-8 m score ticks into equal saved scores and distinct saved scores
that round to the same tick. All original cases, populations, weights, labels
and time horizons remain unchanged. The original AUC is retained alongside an
AUC using strict saved-score order and C/(C+D) conditional on unequal scores.
All-tie comparisons retain AUC .5 and blank conditional concordance; missing
classes retain blank AUC. Conditional concordance uses a different comparison
population and does not replace the original AUC.

```bash
python3 experiments/pose_observation_loss/analyze_event_score_ties.py \
  --output /tmp/event-score-ties
MPLCONFIGDIR=/tmp/event-score-ties-mpl python3 experiments/pose_observation_loss/plot_event_score_ties.py \
  --results /tmp/event-score-ties --output /tmp/event-score-ties-figures
python3 experiments/pose_observation_loss/verify_event_score_ties.py \
  --results /tmp/event-score-ties --figures /tmp/event-score-ties-figures \
  --output /tmp/event-score-ties-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new directories and a new evidence filename, without Python -O. Collection
uses the standard library, plotting needs Matplotlib/NumPy, and the independent
verifier needs NumPy. Existing tracked source tables and the complete preceding
event-temporal bundle remain required inputs: their hashes are checked and the
tables are referenced in place. No previous results are overwritten or copied
into this bundle. No new simulations, fitting or Rust build are required.

`score-audit.csv` preserves all 10,995 states' three original score strings,
binary64 hex representations, ticks and source/case/event joins.
`precision-summary.csv` keeps 8,640 all-event/cumulative-horizon rows;
`precision-bins.csv` keeps 8,640 disjoint time-bin rows. Both expose the original
pair masses/AUC, saved-score AUC, exact/resolved tie shares and unequal-pair
concordance, counts and supported event/case counts. Empty support stays explicit.

Repeat all three CSVs and provenance byte for byte. Five SVG/PNG figures export
both targets' complete numeric inputs. The standalone verifier imports no
analyzers: it reconstructs raw states and original weights, rechecks every
preceding audit/ranking field, orders original score strings using Decimal,
checks agreement with binary64 order and evaluates direct NumPy pair masks.
Saved precision does not imply physical accuracy or independent information.
See the [saved-score tie results](results/event-score-ties/report.md).

## Delete-one case and planner-family sensitivity

The [fixed protocol](results/event-unit-influence/protocol.md) removes every
selected case or planner family once at each preceding event/time scope. A
family shares the full case key except for planner. Populations, common-family
eligibility, labels, score directions and original 1/n_case weights stay fixed.
Deleting one planner case keeps the family's other selected cases. Pair-free
units remain explicit; empty selections have summaries and no deletion rows.

```bash
python3 experiments/pose_observation_loss/analyze_event_unit_influence.py \
  --output /tmp/event-unit-influence
MPLCONFIGDIR=/tmp/event-unit-influence-mpl python3 experiments/pose_observation_loss/plot_event_unit_influence.py \
  --results /tmp/event-unit-influence --output /tmp/event-unit-influence-figures
python3 experiments/pose_observation_loss/verify_event_unit_influence.py \
  --results /tmp/event-unit-influence --figures /tmp/event-unit-influence-figures \
  --output /tmp/event-unit-influence-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new directories and a new evidence filename, without Python -O. Collection
uses the standard library; figures require Matplotlib/NumPy and the independent
verifier requires NumPy. Pinned preceding bundles and original tracked source
tables remain inputs; no new simulation, fitting or Rust build is required.

`unit-audit.csv` contains 782 selection/unit entries, chosen without class labels:
554 cases and 228 families across selection groups. These reuse the 157 observed
cases and are not independent trials. `delete-one.csv` keeps all 56,304 deletions,
including removed mass/counts, remaining support, tick/native AUC and unequal-score
concordance, and changes from baseline. `influence-summary.csv` keeps all 17,280
scope/unit-kind summaries with defined ranges, support loss, exact side counts
around .5 and maximum removed shares. Missing denominators stay blank; removing
all support is not a zero AUC or sign flip. Ranges are deletion sensitivities,
not confidence intervals. No remaining case weights are renormalized.

Repeat all three CSVs and provenance byte for byte. Five SVG/PNG figures export
complete summary inputs across both targets. The standalone verifier imports no
analyzers, reconstructs raw states/Decimal score order and directly masks the
remaining NumPy state pairs for every deletion. Exact Fraction weights preserve
half comparisons, independently of the collector's contribution subtraction.
All prior and new fields and hashes are checked. See the
[delete-one sensitivity results](results/event-unit-influence/report.md).

## Same-event seed deletion and seed-only sensitivity

The [fixed protocol](results/event-seed-influence/protocol.md) retains each of the
three planned seeds alone and deletes it once within the same case/event/time
scopes. Known uses 269/270/271, previous additional 274/275/276 and latest
additional 277/278/279. All three remain explicit even for absent seeds and empty
selections. Original 1/n_case weights, source/common-family selected populations,
eligibility, labels, score orders and time boundaries stay fixed.

```bash
python3 experiments/pose_observation_loss/analyze_event_seed_influence.py \
  --output /tmp/event-seed-influence
MPLCONFIGDIR=/tmp/event-seed-influence-mpl python3 experiments/pose_observation_loss/plot_event_seed_influence.py \
  --results /tmp/event-seed-influence --output /tmp/event-seed-influence-figures
python3 experiments/pose_observation_loss/verify_event_seed_influence.py \
  --results /tmp/event-seed-influence --figures /tmp/event-seed-influence-figures \
  --output /tmp/event-seed-influence-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with fresh directories and an unused evidence filename, without
Python -O. Aggregation uses the standard library; plotting requires Matplotlib
and NumPy; independent verification requires NumPy. All raw sources, preceding
bundles and helper scripts are pinned. No simulation or Rust build is required.

`seed-audit.csv` has 360 label-blind selection/seed rows including zero records.
`seed-effects.csv` has 25,920 scope/seed rows. Unprefixed metrics describe the
remaining population after deleting a seed; `seed_` metrics describe that seed
alone. Removed C/T/D masses and comparison shares also describe the seed alone.
`sensitivity-summary.csv` has all 8,640 cells, each with three planned units,
represented and supported counts, baseline values, deletion ranges/side counts,
support loss and seed-only ranges. All four tick/native AUC and unequal-score
concordance metrics remain available. Missing denominators stay blank, and
support loss is not a zero or strict-side flip. A seed with no pairs is an
explicit deletion no-op and has undefined seed-only metrics.

Repeat the three CSVs and provenance byte for byte, then repeat all five figures
and numeric exports. The standalone verifier imports no analyzers, reconstructs
raw states and Decimal score order, checks prior case/family deletions too, and
uses direct NumPy pair masks with exact Fraction weights for each retained and
remaining seed selection. These are retrospective sensitivities of already
observed seeds, not held-out prediction or confidence intervals. See the
[seed sensitivity results](results/event-seed-influence/report.md).

## Same-event scenario deletion and seed/scenario decomposition

The [fixed protocol](results/event-scenario-influence/protocol.md) retains each
planned dense_50x50, maze_50x50 and open_50x50 scenario alone and deletes it once
across all three seeds. It also preserves every seed/scenario cell separately.
Original 1/n_case weights, source/common-family eligibility, labels, score
orders and same-case/event/time scopes remain fixed. Empty scenarios and cells
stay explicit; selected records and class-pair support are separate quantities.

```bash
python3 experiments/pose_observation_loss/analyze_event_scenario_influence.py \
  --output /tmp/event-scenario-influence
MPLCONFIGDIR=/tmp/event-scenario-influence-mpl python3 experiments/pose_observation_loss/plot_event_scenario_influence.py \
  --results /tmp/event-scenario-influence --output /tmp/event-scenario-influence-figures
python3 experiments/pose_observation_loss/verify_event_scenario_influence.py \
  --results /tmp/event-scenario-influence --figures /tmp/event-scenario-influence-figures \
  --output /tmp/event-scenario-influence-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with fresh directories and an unused evidence filename, without
Python -O. Collection uses the standard library; figures require Matplotlib and
NumPy; the standalone verifier requires NumPy. Sources, preceding artifacts and
helper scripts are pinned. No simulation or Rust rebuild is required.

`scenario-audit.csv` has 1,080 label-blind selection/seed/scenario rows, including
zero-record cells. `scenario-effects.csv` has all 25,920 scenario comparisons:
unprefixed metrics describe the population after deleting that scenario, and
`scenario_` metrics describe the scenario alone. Removed masses/shares also
describe that scenario alone. `sensitivity-summary.csv` keeps 8,640 cells, each
with three planned scenarios, represented/supported counts, baseline values,
deletion ranges/side counts/support loss and scenario-only ranges. All four
tick/native AUC and unequal-score concordance metrics remain available.
`seed-scenario-pairs.csv` has all 77,760 scope/cell contributions with original
availability, weights, six C/T/D masses, pair/support counts and metrics. Their
nine-cell sums reproduce the same-event baseline masses; do not average AUCs.

Repeat the four CSVs and provenance byte for byte, then repeat six figures,
three numeric exports and plot provenance. The independent verifier imports no
analyzers, reconstructs raw states and Decimal score order, checks prior case/
family/seed results too, and directly masks original state pairs for every
remaining and retained scenario and all nine seed/scenario subsets. Missing
values remain blank; loss of support is not a zero or strict-side flip. These
are retrospective sensitivities of observed environments/seeds, not confidence
intervals, independent map samples or held-out validation. See the
[scenario sensitivity results](results/event-scenario-influence/report.md).

## Same-event planner deletion and seed/scenario/planner subsets

The [fixed protocol](results/event-planner-influence/protocol.md) retains A*, JPS
and Theta* separately and removes each once across all seeds/scenarios. It also
keeps every planned seed/scenario/planner audit and fixed focused leaf result.
Original 1/n_case weights, full case identities (including loss mode), labels,
score order, time scopes and source/common-family eligibility stay fixed.
Deleting one planner keeps the other two planners' already selected cases.

```bash
python3 experiments/pose_observation_loss/analyze_event_planner_influence.py \
  --output /tmp/event-planner-influence
MPLCONFIGDIR=/tmp/event-planner-influence-mpl python3 experiments/pose_observation_loss/plot_event_planner_influence.py \
  --results /tmp/event-planner-influence --output /tmp/event-planner-influence-figures
python3 experiments/pose_observation_loss/verify_event_planner_influence.py \
  --results /tmp/event-planner-influence --figures /tmp/event-planner-influence-figures \
  --output /tmp/event-planner-influence-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with fresh directories and an unused evidence file, without
Python -O. Collection uses the standard library; figures require Matplotlib and
NumPy; the standalone verifier requires NumPy. All inputs, preceding results
and helper scripts are pinned. No new simulation or Rust build is required.

`planner-audit.csv` contains 3,240 label-blind selection/seed/scenario/planner
entries, including zero-record leaves. `unit_id` identifies the planner; seed
and scenario are separate fields. One leaf can contain distinct original loss
modes, which stay separate full cases with no cross-mode pairs. Parent
`planner_selection=all` identifies the original mixed population for every row.
`planner-effects.csv` contains all 25,920 operations: unprefixed fields describe
the population after deletion and `planner_` fields describe the planner alone.
Removed masses/shares also describe that planner alone. `sensitivity-summary.csv`
contains 8,640 summaries, each with all three planned planners, represented and
supported counts, ranges, missing support and exact side counts around .5.

`planner-cells.csv` contains all 20,736 leaf/scope contributions for the fixed
residual_sigma, Filter/Fusion, E/P subset, both targets and all four scopes.
All 27 planned cells remain explicit. Their planner sums reproduce all 6,912
corresponding preceding seed/scenario rows. Full-grid planner effects/summaries
retain all scores, labels and strata; other case contributions remain in pinned
preceding outputs. Neither cell AUCs nor planner AUCs are averaged.

Repeat the four CSVs and provenance byte for byte, then six figures, three
numeric exports and provenance. Independent verification imports no analyzers,
reconstructs raw states and Decimal score order, checks all preceding case/
family/seed/scenario results, and directly masks the remaining/retained planner
and focused leaf pairs with exact Fraction weights. Empty or pair-free subsets
remain undefined; lost support is not a zero or strict-side flip. These are
observed diagnostic subsets, not aligned trajectories, causal planner effects
or held-out prediction. See the [planner sensitivity results](results/event-planner-influence/report.md).

## Same-event loss-mode deletion and complete focused case subsets

The [fixed protocol](results/event-loss-mode-influence/protocol.md) retains none,
random and blackout loss modes separately and deletes each once across all
seeds/scenarios/planners. Loss mode is the original full-case `mode`, distinct
from estimator `stratum`. Original 1/n_case weights, fixed common-family
eligibility, labels, score orders and event/time scopes remain unchanged.

```bash
python3 experiments/pose_observation_loss/analyze_event_loss_mode_influence.py \
  --output /tmp/event-loss-mode-influence
MPLCONFIGDIR=/tmp/event-loss-mode-influence-mpl python3 experiments/pose_observation_loss/plot_event_loss_mode_influence.py \
  --results /tmp/event-loss-mode-influence --output /tmp/event-loss-mode-influence-figures
python3 experiments/pose_observation_loss/verify_event_loss_mode_influence.py \
  --results /tmp/event-loss-mode-influence --figures /tmp/event-loss-mode-influence-figures \
  --output /tmp/event-loss-mode-influence-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with fresh directories and an unused evidence file, without
Python -O. Collection uses the standard library; figures require Matplotlib and
NumPy; independent verification requires NumPy. All source tables, prior results
and helper scripts are pinned. No new simulation or Rust build is required.

`mode-audit.csv` keeps 9,720 label-blind loss-mode/seed/scenario/planner entries,
including absent selections. `unit_kind=loss_mode` and `unit_id` name the mode;
seed, scenario and planner are separate fields. Each nonempty leaf corresponds
to at most one original full case. `mode-effects.csv` keeps all 25,920 operations:
unprefixed fields describe deletion, `mode_` fields describe the mode alone,
and removed masses/shares describe that mode alone. `sensitivity-summary.csv`
keeps all 8,640 baseline/range/support/side-count summaries, with three planned
units each. No missing denominator is replaced with zero or chance AUC.

`mode-cells.csv` keeps 62,208 fixed residual_sigma, Filter/Fusion, E/P leaf/scope
rows across both targets, all scopes, cohorts/noises/arms/populations and all
81 mode/seed/scenario/planner combinations. `case_id` names the selected full
case, or stays blank for no selected states. Summing modes reproduces every
one of the 20,736 preceding focused planner leaves. Full mode effects/summaries
retain every original score, label and stratum; other original case contributions
remain accessible in pinned preceding bundles. AUCs are never averaged.

Repeat the four CSVs and provenance byte for byte, then all six figures, three
numeric exports and plot provenance. Independent verification imports no analyzers,
reconstructs original states/Decimal score order and rechecks preceding case/
family/seed/scenario/planner results too. Direct NumPy pair masks and exact
Fraction weights evaluate retained/remaining modes and every focused full-case
leaf. Missing RP selections, including mode none, do not imply zero error or
unrun experiments. These are retrospective diagnostic subsets, not causal loss-
mode effects, independent replications or held-out prediction. See the
[loss-mode sensitivity results](results/event-loss-mode-influence/report.md).

## Class-pair support attrition by case, event and time

The [fixed protocol](results/event-support-attrition/protocol.md) explains where
comparisons cease to be available, without calculating new AUCs. Keep every
planned full case, all three loss modes, both targets and E/P/U, all five strata,
and source/common_family populations. Class labels, original parent 1/n_case
weights and availability-only common-family eligibility stay fixed.

```bash
python3 experiments/pose_observation_loss/analyze_event_support_attrition.py \
  --output /tmp/event-support-attrition
MPLCONFIGDIR=/tmp/event-support-attrition-mpl python3 experiments/pose_observation_loss/plot_event_support_attrition.py \
  --results /tmp/event-support-attrition --output /tmp/event-support-attrition-figures
python3 experiments/pose_observation_loss/verify_event_support_attrition.py \
  --results /tmp/event-support-attrition --figures /tmp/event-support-attrition-figures \
  --output /tmp/event-support-attrition-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with new directories and an unused evidence filename, without
Python -O. Collection uses the standard library and pinned case-strata/event
support ledgers; figures require Matplotlib/NumPy and independent verification
requires NumPy. No new simulation or Rust build is required.

`case-support.csv` retains 58,320 planned case/stratum/target/label/population
rows, including absent RP selections. `case_id` always names the planned full
case. `event-support.csv` retains 16,500 rows over all 275 RP-referenced events,
including empty strata. Empty event/stratum entries come from the original
label-blind support ledger; a missing nonempty time row is rejected rather than
filled with zero. `event-coverage.csv` keeps all 3,997 census events with their
original flags and RP reference counts, including the 3,722 unreferenced events.

`support-summary.csv` keeps 2,880 aggregate rows across the three original modes
and all_modes. For each of the four existing event/time scopes, every planned
case has exactly one reason in this precedence: no_parent_rp, no_stratum_rp,
family_ineligible, no_outside, no_inside, event_separated, gap_excluded, supported.
The ordering is an audit convention, not a causal account of simulator behavior.
Supported cases can still lose most class pairs. Counts/masses and weighted
event/case and time/case or /event shares are saved separately. Undefined
opportunity denominators stay blank. Original squared parent weights precede
stratum/population selection; sum masses before division, never average shares.

Within-case class opportunities include other event tags solely as an availability
denominator; they do not become new admissible within-event comparisons or AUCs.
Every actual event/time pair stays in one full original case, including its loss
mode. Missing RP diagnostics are not unrun experiments, zero error or failed
sensor observations. None and every empty panel remain explicit.

Repeat four CSVs and provenance byte for byte, then four figures, the 288-row
numeric export and plot provenance. The standalone verifier imports no analyzers,
reconstructs labels from original signed pose/projection components, and uses
direct NumPy event/time masks with exact Fraction parent weights to check all
new fields, original mode/precision support, plot values and hashes. See the
[support attrition report](results/event-support-attrition/report.md). These
retrospective selections do not establish causality, independent replications,
calibration or held-out prediction.

## Coverage from the late-timeout census to RP records

The [fixed protocol](results/census-rp-selection/protocol.md) joins every planned
case and broad late-timeout event to the fixed RP diagnostic selection. Original
status, loss mode and immediate actual/preview identifiers remain explicit.
This describes coverage and identifier differences; it computes no new AUCs,
errors or controller outcomes.

```bash
python3 experiments/pose_observation_loss/analyze_census_rp_selection.py \
  --output /tmp/census-rp-selection
MPLCONFIGDIR=/tmp/census-rp-selection-mpl python3 experiments/pose_observation_loss/plot_census_rp_selection.py \
  --results /tmp/census-rp-selection --output /tmp/census-rp-selection-figures
python3 experiments/pose_observation_loss/verify_census_rp_selection.py \
  --results /tmp/census-rp-selection --figures /tmp/census-rp-selection-figures \
  --output /tmp/census-rp-selection-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Run sequentially with new directories and an unused evidence filename, without
Python -O. Collection and independent SQL verification use the standard library;
figures require Matplotlib/NumPy. All inputs and prior helpers are pinned.
No simulation, Rust build, new label reconstruction or cutoff refit is required.

`case-selection.csv` keeps all 972 planned cases with original outcomes and
held/timeout_no_rp/timeout_with_rp stages. Held cases fall outside the original
late-timeout census by design. `event-selection.csv` keeps all 3,997 census
events, original source IDs/partition aliases, flags and immediate identifiers,
with RP records/masses by all four strata and original time offsets. An immediate
event preview identifier is distinct from a selected window-state stratum.

`selection-summary.csv` keeps 3,024 cells across all cohorts/noises/arms, all
three loss modes plus all_modes, seven event groups and all/two actual and
preview selectors. Every empty planned category stays explicit. `received_any`
uses the received flag, including combined received/precision events if present.

Three parent measures remain separate: 1/n_all_census_events_case for each
event, 1/n_received_census_events_case for received events, and original
1/n_RP_parent_case for RP states. Parent weights precede kind/identifier/stratum
selection. Sum exact masses before shares; never average case rates or equate
the three measures. Blank coverage means no denominator; zero coverage means
observed census events but no RP reference. Different window/event identifiers
do not locate a transition or establish a causal mode effect.

Repeat three CSVs and provenance byte for byte, then three figures, the 252-row
numeric export and plot provenance. Independent verification imports no analyzers
and reads the original broad census plus historical membership/latest RP score
metadata instead of trusting the saved source-event-map. SQL selection/count
queries and exact Fraction weights verify every new field, original case/event
coverage, additive closure and hashes. Missing RP records are not zero error,
unrun experiments or failed observations. See the [coverage report](results/census-rp-selection/report.md).

## Logged preview transitions in the original windows

The [frozen protocol](results/logged-preview-transitions/protocol.md) follows
all 3,997 late-timeout census events through the original inclusive -20..+1,500 ms
windows, clipped at 180 s. It checks whether differing immediate-event and RP
preview identifiers correspond to a sequence actually present in the raw logs.

```bash
python3 experiments/pose_observation_loss/analyze_logged_preview_transitions.py \
  --output /tmp/logged-preview-transitions
MPLCONFIGDIR=/tmp/logged-preview-transitions-mpl python3 experiments/pose_observation_loss/plot_logged_preview_transitions.py \
  --results /tmp/logged-preview-transitions --output /tmp/logged-preview-transitions-figures
python3 experiments/pose_observation_loss/verify_logged_preview_transitions.py \
  --results /tmp/logged-preview-transitions --figures /tmp/logged-preview-transitions-figures \
  --output /tmp/logged-preview-transitions-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new directories and an unused evidence filename, without Python -O.
Collector and independent verifier use the standard library; figures require
Matplotlib/NumPy. Both raw readers need the 108 original CSVs (6,043,185,194 bytes)
pinned in `fresh-seed-cutoff-transfer/sources.json`, including the historical
terminal-arc-live and latest fresh-seed-mode-census roots. Each reader accepts
`--raw-roots /path/to/roots.json`, an optional JSON map from any of the six cohort
names to relocated directories containing the same arm-mode-seed subdirectories.
Every original file hash remains mandatory. Raw CSVs and PNGs stay outside Git.
No new simulation or Rust build is required.

`event-trace.csv` keeps all events with immediate and predecessor modes,
post-event changes, observation horizon and RP disagreement timing.
`mode-intervals.csv` losslessly compresses the two logged mode identifiers into
maximal inclusive runs, preserving all 265,019 checkpoint references.
`rp-state-audit.csv` checks every original RP state against its same-time log
and covering interval. Selected preview depends on `epoch_fusion_available`;
retained group count alone does not define the selected source.

`transition-summary.csv` keeps 1,344 cells over all cohort/noise/arm groups,
none/random/blackout plus all_modes, all/received/five event-kind groups, and
all/full/clipped/no-forward horizon selectors. A pre-event to immediate change
is distinct from a later change. A final 180 s event has no forward sample;
an empty denominator stays blank. Counts from overlapping windows are event
references, not independent physical observations.

Original all-census, received-census and RP-state parent measures remain
separate and precede filtering. Fusion-loss fractions condition on immediate
Fusion; the figure uses only full 1.50 s forward windows. Sum exact masses
before division. The lag plot includes every event with a post-event differing
RP preview, including empty cohort panels. Numeric exports contain 144 full
summary rows and all qualifying event rows.

Repeat four CSVs/provenance and all figures/exports byte for byte. The standalone
verifier imports no analyzers, independently reads and hashes the raw CSVs,
joins original broad-census and historical/latest RP metadata, and uses SQL
LAG/cumulative windows over uncompressed checkpoints to reconstruct all runs
and transition positions. See the [logged transition report](results/logged-preview-transitions/report.md)
for findings, full verification and limits. Logged ordering does not establish
causal mode benefits or an unobserved continuous-time transition instant.

## Epoch retention mechanisms behind the logged losses

The [fixed protocol](results/epoch-retention-mechanisms/protocol.md) reconstructs
only epoch identities and capped sample counts through every original timeout
decision. It connects the preceding preview-loss boundaries to the existing
15 s retention and motion/precision reset rules.

```bash
python3 experiments/pose_observation_loss/analyze_epoch_retention.py \
  --output /tmp/epoch-retention-mechanisms
MPLCONFIGDIR=/tmp/epoch-retention-mechanisms-mpl python3 experiments/pose_observation_loss/plot_epoch_retention.py \
  --results /tmp/epoch-retention-mechanisms --output /tmp/epoch-retention-mechanisms-figures
python3 experiments/pose_observation_loss/verify_epoch_retention.py \
  --results /tmp/epoch-retention-mechanisms --figures /tmp/epoch-retention-mechanisms-figures \
  --work /tmp/epoch-retention-native --output /tmp/epoch-retention-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Collection uses the standard library; plotting needs Matplotlib/NumPy.
The standalone verifier also needs `rustc` on PATH or `--rustc /path/to/rustc`.
In this managed setup, `. /workspace/.cloud-env/rust_robotics/activate.sh`
activates the installed toolchain. Verification compiles a scratch Rust 2021
stdlib harness; no Cargo dependencies or network access are needed. It runs
the six unchanged original epoch-fusion tests and independently reads the raw
CSV files in Rust. This run used Rust 1.99.0; provenance records the compiler,
flags, source-slice hashes and generated harness hash.

Both readers require the same 108 original raw files as the preceding logged
transition study, and accept its `--raw-roots` cohort-to-directory JSON override.
All original hashes remain mandatory. Raw CSVs, generated native source/binaries
and PNGs stay outside Git. No controller setting, retention horizon or simulation
is changed.

`event-mechanisms.csv` keeps all 3,997 events, the oldest immediate epoch's age
and remaining retention time, observed loss operation and original census/RP
parent measures. `snapshots.csv` keeps every immediate event and both endpoints
of each previously logged loss, totaling 4,301 event/role references.
`group-history.csv` keeps every retained group in those snapshots, oldest first,
with its logged epoch time, sample count, age and remaining TTL. Empty histories
have no group rows. Reused physical checkpoints remain separate references.

`retention-summary.csv` keeps 1,344 cells with all original loss modes plus
all_modes, event-kind groups and full/clipped/no-forward horizons. Operation
ordering follows reset, TTL pruning, capacity and availability predicates;
independent flags remain available alongside that audit category. Original
all-census, received-census and RP parent measures precede filtering and remain
separate. One-step denominators require a forward sample; the 18 TTL-limit
events at 180 s do not become observed losses or observed stable events.

The native verifier extracts the pinned original Rust observe/types, mean_pose,
VehicleState struct/new and six original tests without changing their text.
It uses logged speed/age/event flags with dummy zero pose and steering because
geometry does not enter group membership, sample counts or availability.
Fused positions are ignored: this validates metadata, not pose or controller
trajectory replay. Every timeout decision's availability and retained count
must match the original log. Native snapshots independently reconstruct all
saved event/group/summary fields, exact parent weights and 144 plot rows.

Repeat all four CSVs/provenance and the figure/CSV/provenance byte for byte.
See the [retention mechanism report](results/epoch-retention-mechanisms/report.md)
and its native evidence. This explains recorded algorithm operations; it does
not estimate the benefit of changing the horizon or establish held-out prediction.

## Follow-up after Fusion becomes unavailable

The [fixed protocol](results/fusion-return-followup/protocol.md) follows all
152 previously observed Fusion losses through the original 180 s log endpoint.
It distinguishes a first return, a return followed by another loss, and a
right-censored episode with no observed return. All 3,997 original census
anchors remain in the ledger, including the 18 TTL-limit anchors without a
next step; those are outside the observed-loss denominator.

```bash
python3 experiments/pose_observation_loss/analyze_fusion_return.py \
  --output /tmp/fusion-return-followup
MPLCONFIGDIR=/tmp/fusion-return-followup-mpl python3 experiments/pose_observation_loss/plot_fusion_return.py \
  --results /tmp/fusion-return-followup --output /tmp/fusion-return-followup-figures
python3 experiments/pose_observation_loss/verify_fusion_return.py \
  --results /tmp/fusion-return-followup --figures /tmp/fusion-return-followup-figures \
  --work /tmp/fusion-return-native --output /tmp/fusion-return-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Both readers require the same 108 pinned raw files and accept the preceding
`--raw-roots` cohort-to-directory JSON override. Collection uses the standard
library; plotting requires Matplotlib/NumPy. Verification needs `rustc` on PATH
or `--rustc /path/to/rustc`; the managed setup's activation file is
`/workspace/.cloud-env/rust_robotics/activate.sh`. Its scratch stdlib harness
uses the pinned preceding verifier/native helpers, six original Rust tests and
unchanged simulator source. No Cargo dependency installation or network is needed.

`availability-runs.csv` compresses all 855,570 late checkpoints in all 570
timeout histories into maximal inclusive availability runs. It is lossless
for the availability flag only. `loss-followup.csv` retains every original
anchor with fixed parent weights and RP accounting. Observed losses add their
first return or censoring bound, returned-spell endpoint, source groups and
next opportunities. A return at exactly 180 s is observed, with zero subsequent
observed span and a censored returned spell. A missing return has a blank
return wait and a separate elapsed lower bound, not a substituted duration.

`opportunity-followup.csv` keeps every future census opportunity for every
observed-loss anchor, including opportunities after its first return. Reused
events are anchor-opportunity references, not independent observations.
`followup-summary.csv` keeps 528 cells over all original cohort/noise/arm groups,
loss modes plus all_modes and all/ten operation selectors, including empty
operations. Return fractions condition on observed losses; original all-census,
received-census and RP parent measures remain separate and precede filtering.
Case counts can overlap across operation/outcome groups and are not additive.

The standalone verifier imports no analyzers and requests every late checkpoint
from the original Rust implementation, rather than collector-chosen return
times. It reconstructs maximal native runs, first returns and spell endpoints
independently, comparing all new fields, original joins, 396 numeric summary
rows and 152 plotted loss rows. It records all 108 native trace hashes.
Dummy zero geometry limits verification to metadata; fused positions are ignored.

Repeat four CSVs/provenance and both figures/exports byte for byte. See the
[Fusion return report](results/fusion-return-followup/report.md) for observed
waits, one-checkpoint returns and censoring. These selected retrospective
sequences do not estimate eventual-return probabilities or controller benefits.

## Why the first returned Fusion spell ends

The [fixed protocol](results/return-spell-mechanisms/protocol.md) audits all
137 first returns from the preceding study. It compares observed next losses
to native reset/retention/age evidence and strictly later original received
opportunities. All 3,997 census anchors remain in the ledger, including the
15 original losses without a return and 18 original TTL-limit anchors without
a forward checkpoint. A returned spell at 180 s has an unknown future end.

```bash
python3 experiments/pose_observation_loss/analyze_return_spell.py \
  --native-traces /tmp/fusion-return-native --output /tmp/return-spell-mechanisms
MPLCONFIGDIR=/tmp/return-spell-mpl python3 experiments/pose_observation_loss/plot_return_spell.py \
  --results /tmp/return-spell-mechanisms --output /tmp/return-spell-figures
python3 experiments/pose_observation_loss/verify_return_spell.py \
  --results /tmp/return-spell-mechanisms --figures /tmp/return-spell-figures \
  --work /tmp/return-spell-native --output /tmp/return-spell-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. The collector requires the 108 exact full late native traces generated by
the preceding `verify_fusion_return.py` command; `--native-traces` points to that
verifier's work directory. Every cache file is checked against the preceding
committed evidence. It reads 152 MB of cached metadata and does not reread raw
logs. Recreate a missing cache using the preceding verifier and the 108 pinned
raw files. Collection uses the standard library; plotting needs Matplotlib/NumPy.

The new standalone verifier imports no analyzers. It freshly reads/hashes all
original raw files, compiles unchanged original Rust metadata code, runs its
six original tests, and reconstructs every late checkpoint in all 570 timeout
cases. Its fresh traces must exactly match the frozen cache hashes. It accepts
`--raw-roots` as above, and needs `rustc` on PATH or `--rustc /path/to/rustc`.
The managed activation file remains
`/workspace/.cloud-env/rust_robotics/activate.sh`. No Cargo/network is required.

`return-endpoints.csv` preserves original anchors and parent measures, observed
return spans and end operations, independent gate flags, and next opportunity/
receipt joins. Conditional TTL and age deadlines assume unchanged history/no
new receipt; a matching span alone does not classify an operation. Censored end
fields stay blank, even when the oldest epoch is at its inclusive TTL limit.
`snapshots.csv` keeps 363 anchor/role references at returns, before/after observed
ends and censored endpoints. `group-history.csv` keeps their 1,047 retained
group references. Reused physical checkpoints remain separate references.

`spell-summary.csv` keeps all 528 original cohort/noise/arm/mode/first-loss
operation cells, including empty operations and planned case/status counts.
All nine possible subsequent end operations remain columns. Operation shares
condition on observed ends and retain separate original census/received/RP
parent measures. They do not estimate eventual loss probabilities. Independent
verification covers every saved field, exact additive closures, 396 plotted
summary rows and all 137 plotted first-return rows. Dummy geometry limits
verification to metadata; fused positions are ignored.

Repeat all four CSVs/provenance and both figures/exports byte for byte. The
[returned-spell report](results/return-spell-mechanisms/report.md) explains the
20 ms TTL and 1.52 s age-gate endings and preserves endpoint censoring. No
retention setting, controller behavior or original RP selection is changed.

## Logged controls around first Fusion returns

The [fixed protocol](results/fusion-return-controls/protocol.md) joins every
one of the preceding 137 first returns to its original control records. It
keeps recovery-pose and preview source identifiers, loss/guard latches,
resumption flags, nominal commands and final commands separately. All 3,997
original anchors remain in the ledger with their original parent measures.

```bash
python3 experiments/pose_observation_loss/analyze_return_controls.py \
  --output /tmp/fusion-return-controls
MPLCONFIGDIR=/tmp/return-controls-mpl python3 experiments/pose_observation_loss/plot_return_controls.py \
  --results /tmp/fusion-return-controls --output /tmp/return-controls-figures
python3 experiments/pose_observation_loss/verify_return_controls.py \
  --results /tmp/fusion-return-controls --figures /tmp/return-controls-figures \
  --work /tmp/return-controls-independent --output /tmp/return-controls-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Both readers hash and stream the same 108 pinned original raw files and
accept the preceding `--raw-roots` cohort-to-directory JSON override. Collection
and independent verification use the standard library, including SQLite for
the verifier. Plotting requires Matplotlib/NumPy. This audit inherits the
preceding native metadata evidence and does not run another Rust metadata
replay or require Cargo/network access.

`anchor-controls.csv` preserves the original loss/return/end/censoring fields,
weights and RP accounting. Returned anchors add source/latch/command snapshots,
within-available-spell counts, sample-count range and logged recovery evidence.
`control-runs.csv` compresses each inclusive [return-20 ms, next loss/log end]
window into maximal source/latch/resumed/positive-command tuples. Counts are
anchor-checkpoint references; overlapping roles are retained. Within-spell
counts exclude an observed next-loss checkpoint and include an observed 180 s
return without extending its future span. `control-snapshots.csv` keeps all
500 fixed anchor/role references at return predecessors/returns and observed
endpoints. Censored end-change fields remain blank.

`control-summary.csv` retains 336 cells over all original cohort/noise/arm/mode
conditions and the preceding disjoint anchor categories, including empty cells
and planned/timeout/held counts. All-census, received-census and RP masses keep
their original parents. Resume/positive-command shares condition on returned
anchors and use exact masses before division; undefined shares stay blank.
Checkpoint-reference counts remain separate from event counts and masses.

The standalone verifier imports no analyzers. It independently parses the
original unquoted scalar CSVs, rejoins the original census and reconstructs
maximal runs with SQLite window functions over uncompressed checkpoints. Both
readers check final command overwrite invariants at every 150..180 s checkpoint
in all 570 timeout histories. Verification covers every saved field, closure,
252 numeric plot summary rows and all 137 returned rows. Logged scalar and
sample requirements do not constitute a full recovery gate or geometry replay.

Repeat all four CSVs/provenance and the figure/exports byte for byte. See the
[control audit report](results/fusion-return-controls/report.md) for observed
latch, command and recovery-pose behavior. No controller, original RP selection
or simulation outcome is changed.

## Admission conditions and timers after Fusion returns

The [fixed protocol](results/fusion-return-admission/protocol.md) audits all
137 preceding first returned spells, including the 48 censored returns at
180 s. It separates seven joint loss-recovery conditions from the two guard
release conditions and retains simultaneous failures and rounded-input
unknowns. Every original census anchor and parent measure remains represented.

```bash
python3 experiments/pose_observation_loss/analyze_return_admission.py \
  --output /tmp/fusion-return-admission
MPLCONFIGDIR=/tmp/return-admission-mpl python3 experiments/pose_observation_loss/plot_return_admission.py \
  --results /tmp/fusion-return-admission --output /tmp/return-admission-figures
python3 experiments/pose_observation_loss/verify_return_admission.py \
  --results /tmp/fusion-return-admission --figures /tmp/return-admission-figures \
  --work /tmp/return-admission-independent --output /tmp/return-admission-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Both readers hash and stream the same 108 original raw files and accept
`--raw-roots` as above. Collection and independent verification use the standard
library; plotting needs Matplotlib/NumPy. No Cargo, new native metadata replay
or network access is required. Existing native metadata evidence is inherited.

`admission-decisions.csv` saves 3,751 decision references: 3,662 while returned
Fusion is available and 89 at the first next loss. Each return's predecessor
supplies the actual carried latch/timer inputs. The source binary64 clock is
reconstructed and checked against every printed timeout age; exact audited
ages include hex encodings. The pinned source's identical recovery/speed-margin
expressions justify using logged speed-margin intervals. Nine-decimal scalar
inputs receive half-unit rounding intervals with a 1e-12 numerical guard and
sin/cos Lipschitz endpoint bounds. Unknowns stay separate from certain failures.

`admission-runs.csv` losslessly compresses branch, phase, readiness certainty,
timer-derived readiness, simultaneous failure/unknown masks and timer action.
`anchor-admission.csv` keeps all 3,997 original anchors with fixed parent weights,
RP accounting and blank new fields for nonreturns. It adds reference counts,
condition states, carried/maximum counters and available versus next-loss reset
counts. A return at 180 s adds one observed decision without a future endpoint.

`admission-summary.csv` keeps all 336 original condition/category cells, including
empty cells and planned/timeout/held counts. Original census/received/RP weights
precede selection; exact masses precede division. Checkpoint-reference counts
remain separate from anchor masses. Raw and mass-weighted ready/reset shares
condition on returned anchors; empty denominators are blank.

The standalone verifier imports no analyzers. It uses Decimal intervals and
Taylor-series trigonometry independently of the collector's float/libm bounds,
enumerates both ready states to check recorded counter transitions, and uses
SQLite window functions on uncompressed decisions for maximal runs. It checks
all saved fields, original/preceding-control joins, closures, 252 numeric plot
summary rows and all 137 returned-anchor rows. Timer-derived readiness does
not relabel an uncertain individual condition.

Repeat all four CSVs/provenance and the figure/exports byte for byte. See the
[admission report](results/fusion-return-admission/report.md) for observed
counter resets, joint/guard thresholds and remaining nominal-sign uncertainty.
These are selected retrospective inputs and timer records, with no new
controller outcome, causal effect or eventual-release probability.

## Nominal command signs from the source domain

The [fixed protocol](results/fusion-return-command-sign/protocol.md) adds a
source-domain result to the preceding admission audit. It retains the original
interval classifications and all 137 returned spells, 3,751 decision references
and 3,997 census anchors. The new sign inference uses the original speed probe's
41 possible distances and the positive lower bound of an unlatched pre-cap
command. Recorded timers enter only the subsequent consistency comparison.

In the saved results, all 909 previously ambiguous available joint nominal
gates have exact cap-zero commands. The 25 previously unknown joint conjunctions
become failures, giving 101 pass, 3,169 fail and zero unknown references.
Guard-only admission is unchanged. All 5,130,000 timeout decisions pass the
command-chain checks, and the native cap table matches all 41 collector values
bit for bit. See the report for resets, accounting and the limits of this audit.

```bash
python3 experiments/pose_observation_loss/analyze_command_sign.py \
  --output /tmp/fusion-return-command-sign
MPLCONFIGDIR=/tmp/command-sign-mpl python3 experiments/pose_observation_loss/plot_command_sign.py \
  --results /tmp/fusion-return-command-sign --output /tmp/command-sign-figures
python3 experiments/pose_observation_loss/verify_command_sign.py \
  --results /tmp/fusion-return-command-sign --figures /tmp/command-sign-figures \
  --work /tmp/command-sign-independent --output /tmp/command-sign-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Activate the cloud environment's saved Rust toolchain first or pass the
compiler path with `--rustc`. Both readers accept `--raw-roots` as above and
freshly hash all 108 original raw files. Collection and verification use the
standard library; plotting requires Matplotlib. The verifier compiles the
unchanged cap expression and the original pre-steering command block using
stdlib Rust. No Cargo dependencies or new rollouts are required.

`command-lattice.csv` records all 41 free distances and caps, including binary64
bits and nine-place strings. `timeout-command-counts.csv` keeps full timeout-grid
counts by original cohort, arm, mode, seed, pause branch and structural reason,
with zero cells preserved. Checks use decimal nanounits: rounding commutes with
min, so the printed nominal command equals the min of the printed uncapped
command and cap. Stop-latch, speed-cap and simultaneous zeros stay distinct.
The later pause/guard override and exact-cap braking comparison are checked too.

`command-decisions.csv` preserves prior masks/certainty and adds independently
derived source signs, reasons and refined masks/certainty. Guard-only admission
keeps its original two conditions. `anchor-command-sign.csv` retains every
original anchor, fixed parent measures and blank new fields for nonreturns.
`command-summary.csv` keeps 336 fixed cells, with phase/branch counts and raw
and original-case-mass shares. Undefined shares remain blank. Both readers
reconstruct exact parent measures and check mode/category closures.

The standalone verifier imports no analyzers. It uses Decimal nanounits and a
native cap lookup independently of the collector's float calculations. Earlier
geometry, source-clock, timer and Fusion metadata evidence remains inherited
from the pinned admission proof. This verifies a finite command domain and
its logged consistency; unrounded map geometry and controller trajectories
are not replayed. Repeat all five tables/provenance and figure/exports byte
for byte. See the [command-sign report](results/fusion-return-command-sign/report.md)
for the observed refinement and independent evidence.

## Initial geometry of zero speed-probe returns

The [fixed protocol](results/fusion-return-probe-origin/protocol.md) separates
zero speed probes caused by an unsafe start from a first unsafe chord. It
keeps all preceding 3,751 references, 137 returned spells and 3,997 anchors.
A certainly safe start plus a recorded zero identifies the first chord by
source-path exclusion. This does not replay unrounded curved geometry.

The saved 25 previously unresolved conjunctions split into 23 initially
unsafe sensed positions and two first-chord origins. All initial violations
are against the inflated obstacle margin. Across all 3,751 references there
are 1,035 initial-unsafe zeros, 61 first-chord zeros and no ambiguous starts.
These describe the probe's estimated geometry, not true collision labels.

```bash
python3 experiments/pose_observation_loss/analyze_probe_origin.py \
  --output /tmp/fusion-return-probe-origin
MPLCONFIGDIR=/tmp/probe-origin-mpl python3 experiments/pose_observation_loss/plot_probe_origin.py \
  --results /tmp/fusion-return-probe-origin --output /tmp/probe-origin-figures
python3 experiments/pose_observation_loss/verify_probe_origin.py \
  --results /tmp/fusion-return-probe-origin --figures /tmp/probe-origin-figures \
  --work /tmp/probe-origin-independent --output /tmp/probe-origin-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Activate the saved Rust toolchain or pass `--rustc`. Both readers accept
`--raw-roots`; the collector also accepts `--obstacles` with the pinned map
hash. All six raw provenance/compiler/map declarations and all 108 original
raw hashes are checked. No Cargo dependencies or new rollouts are needed.

The current raw row records truth after state.update. Reconstruct the probe's
sensed x/y from the preceding row's truth plus current noise and bias, in the
source addition order. Step-zero noise fields establish the noisy pose branch.
Nine-place inputs retain half-unit rounding intervals and a 1e-12 comparison
guard. Check all walls and original obstacle records; keep duplicate map IDs,
all definite/ambiguous witnesses and simultaneous failure masks.

`probe-decisions.csv` retains prior command/admission/timer fields and adds
pose inputs, rectangles, initial-test bounds and origin labels. Safe starts
with positive probes stay distinct from zero origins; unresolved starts stay
ambiguous. `anchor-probe-origin.csv` retains every preceding anchor column and
adds phase/branch/origin counts and blank fields for nonreturns. `probe-summary.csv`
keeps 336 original cells, exact original measures and new raw/case-mass shares.
The exact original map is saved as `obstacle-map.csv`.

The standalone verifier imports no analyzers. It uses 60-digit Decimal
rectangles and squared obstacle distances independently of the collector's
float enclosures. It compiles the unchanged initial source test and actual
Point2D distance method with stdlib Rust, cross-checking rounded centers only
when the bounded result is certain. Earlier admission/timer/command-domain
and Fusion metadata evidence remains inherited from the pinned proof.
Repeat all four tables/provenance and figure/exports byte for byte. See the
[probe-origin report](results/fusion-return-probe-origin/report.md) for results
and the distinction between direct initial-test evidence and first-chord
inference by exclusion.

## Direct tests of the first speed-probe chord

The [fixed protocol](results/fusion-return-first-chord/protocol.md) directly
bounds all 61 preceding first-chord references, retaining all 3,751 decision
references, 3,997 anchors and 336 summary cells. The two references among the
preceding 25 nominal-only conjunction failures remain included in that scope.
The earlier source-path exclusion result stays in separate inherited columns.

All 61 saved direct tests fail only the inflated obstacle-margin condition,
with one definite original obstacle record each and no unresolved result.
The two preceding nominal-only resets identify obstacle IDs 420 at (3,4)
and 1310 at (34,35), with margin deficits about 9.89 mm and 4.78 mm.

```bash
python3 experiments/pose_observation_loss/analyze_first_chord.py \
  --output /tmp/fusion-return-first-chord
MPLCONFIGDIR=/tmp/first-chord-mpl python3 experiments/pose_observation_loss/plot_first_chord.py \
  --results /tmp/fusion-return-first-chord --output /tmp/first-chord-figures
python3 experiments/pose_observation_loss/verify_first_chord.py \
  --results /tmp/fusion-return-first-chord --figures /tmp/first-chord-figures \
  --work /tmp/first-chord-independent --output /tmp/first-chord-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Activate the saved Rust toolchain or provide `--rustc`. Both readers accept
`--raw-roots`, freshly check all original raw hashes/grids and use the preceding
pinned map. Collection/verification use the standard library; plotting uses
Matplotlib. No new rollout or Cargo dependencies are required.

Sensed yaw uses preceding truth yaw plus current noise and bias. The nominal
steering field supplies the probe command before final pause/guard overrides.
Nine-place rounding intervals and a 1e-12 elemental allowance propagate through
the source's original tan and sin/cos-difference arithmetic. Curvature near zero
uses a fixed coarse endpoint rectangle instead of choosing a branch. Chord
distance uses the endpoint-displacement/Hausdorff bound and a 1e-9 comparison
allowance. These are explicit numerical assumptions of this bounded audit.

`first-chord-decisions.csv` retains preceding columns, adds an audited flag and
new endpoint/witness/direct-test fields for the 61 selected references, and
leaves those fields blank for the other 3,690. `anchor-first-chord.csv` keeps all
preceding anchor fields and original measures. `first-chord-summary.csv` keeps
the same 336 cells and adds phase/branch counts and raw/case-mass shares.
Original weights precede selection; undefined shares stay blank.

The standalone verifier imports no analyzers. It uses 60-digit Decimal Taylor
trigonometry, separate interval arithmetic and projection/clamp distances.
It compiles the unchanged full source probe, original segment/point distances
and extracted first-endpoint arithmetic with stdlib Rust. Native rounded-center
traces cross-check enclosures, predicates and every certain/possible witness;
they do not recover unrounded original poses. Repeat all three tables/provenance
and the figure/exports byte for byte. See the
[first-chord report](results/fusion-return-first-chord/report.md) for direct
results, numerical allowances and inherited evidence.

## Compare the raw probe pose with recovery geometry

The [fixed protocol](results/fusion-return-recovery-geometry/protocol.md)
compares all 1,096 preceding zero-probe references with the logged recovery
pose. It keeps every original obstacle witness, all 3,751 decision references,
3,997 anchors, original weights and 336 summary cells. The speed probe uses
the current sensed pose; recovery uses the stationary mean when samples are
available. Source checks confirm both obstacle margins use the same expression.

For all 25 preceding nominal-only conjunction failures, the original obstacle
and the complete map pass the recovery-preview clearance condition. Raw and
recovery start estimates differ by about 0.05–0.35 coordinate units. Across
all zero references, 276 clear the original obstacles in recovery geometry,
but 150 of those encounter another map obstacle. Geometry alone does not
establish recovery readiness or a safe controller change.

```bash
python3 experiments/pose_observation_loss/analyze_recovery_geometry.py \
  --output /tmp/fusion-return-recovery-geometry
MPLCONFIGDIR=/tmp/recovery-geometry-mpl python3 experiments/pose_observation_loss/plot_recovery_geometry.py \
  --results /tmp/fusion-return-recovery-geometry --output /tmp/recovery-geometry-figures
python3 experiments/pose_observation_loss/verify_recovery_geometry.py \
  --results /tmp/fusion-return-recovery-geometry --figures /tmp/recovery-geometry-figures \
  --work /tmp/recovery-geometry-independent --output /tmp/recovery-geometry-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use new output/work directories and an unused evidence filename, without Python
-O. Activate the saved Rust toolchain or provide `--rustc`. Both readers accept
`--raw-roots`, freshly hash all 108 logs and check unique complete case grids.
The preceding pinned obstacle map and admission inputs are required. Collection
and verification use the standard library; plotting uses Matplotlib.

`recovery-geometry-decisions.csv` retains all preceding columns and adds an
audit flag, recovery inputs, original witness IDs, displacement enclosures,
same-obstacle point/segment gaps and the complete-map clearance enclosure.
Positive probes have blank new geometry. `anchor-recovery-geometry.csv` and
`recovery-geometry-summary.csv` preserve earlier fields and add phase/branch
counts, available-anchor flags and original case-mass shares. Nonreturns keep
blank new fields; empty share denominators stay blank.

The standalone verifier imports no analyzers. Pinned standalone helpers supply
its independent raw/map reader and Decimal Taylor trigonometry; it separately
rebuilds endpoint/Hausdorff enclosures, projection distances and all new fields.
The unchanged Rust recovery fragment and point/segment distance provide native
endpoint/clearance checks on 1,096 rounded logged poses. Earlier raw initial/
first-chord tests and admission/timer results remain inherited. This does not
reconstruct unrounded sample means or add a closed-loop rollout. Repeat the
three tables/provenance and figure/exports byte for byte. See the
[recovery geometry report](results/fusion-return-recovery-geometry/report.md)
for counts, the original 25-reference subgroup and numerical assumptions.

## Reconstruct the stationary recovery window

The [fixed protocol](results/fusion-return-recovery-window/protocol.md)
reconstructs sample membership for all 1,096 preceding zero-probe references.
It replays every recorded step of histories containing a selected reference,
checks their recovery sample counts, and retains all preceding references,
anchors, original measures and summary cells. Other raw histories still receive
complete hash/grid checks; their windows are not claimed as newly replayed.

The recorded audit reproduces all sample counts across 63 selected histories
and 567,000 steps. All 1,096 reference means agree with logged values within
the declared input/output rounding enclosures: 921 use retained samples and
175 use raw fallback. The preceding 25 conjunction failures use 10–25 samples;
18 already evicted older samples. This confirms the recorded window behavior
without changing the nominal-speed predicate or controller.

```bash
python3 experiments/pose_observation_loss/analyze_recovery_window.py \
  --output /tmp/fusion-return-recovery-window
MPLCONFIGDIR=/tmp/recovery-window-mpl python3 experiments/pose_observation_loss/plot_recovery_window.py \
  --results /tmp/fusion-return-recovery-window --output /tmp/recovery-window-figures
python3 experiments/pose_observation_loss/verify_recovery_window.py \
  --results /tmp/fusion-return-recovery-window --figures /tmp/recovery-window-figures \
  --work /tmp/recovery-window-independent --output /tmp/recovery-window-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output/work directories and an unused evidence filename, without
Python -O. Activate the saved Rust toolchain or provide `--rustc`. Both readers
accept `--raw-roots`. Collection and verification use the standard library;
plotting uses Matplotlib. No new controller rollout or Cargo dependencies.

Window admission uses the pause flag before recovery updates, exact roundtrip
measured speed and source binary64 observation-age arithmetic. Preserve every
simultaneous clear reason, clear-then-append order, and removal of the oldest
sample beyond capacity 25. `window-lineage.csv` records the ordered raw inputs
of every retained member; empty windows have an explicit raw-fallback row.
`window-decisions.csv` retains preceding columns and adds count/clear/eviction
metadata, bounded reconstructed means and a rounded-input decomposition of the
current-versus-mean displacement into truth/noise/bias terms. This decomposition
is for auditing recorded inputs and does not supply true pose to the controller.

`anchor-window.csv` and `window-summary.csv` retain every preceding field and
original measure, adding phase/branch counts, anchor flags and original case-
mass shares. Nonreturns and undefined shares keep blank new fields. Positive
probes retain blank new reconstruction fields.

The standalone verifier imports no analyzers. It independently keeps admissions
since the last clear, selects the last 25, and calculates Decimal mean intervals
and circular yaw with Taylor trig and a separate reduced atan series. Native
Rust compiles the unchanged original window-update/mean fragment and checks
all selected-history sample counts, tracing each selected rounded-input mean.
The full native replay input stays scratch and hash-pinned; small checkpoint
queries and traces are tracked. These checks do not recover original unrounded
samples or run the rest of the controller. Repeat all four tables/provenance
and plot artifacts byte for byte. See the
[window reconstruction report](results/fusion-return-recovery-window/report.md)
for evidence and the fixed 25-reference subgroup.

## Compare adjacent recovery-window updates

The [fixed protocol](results/fusion-return-window-turnover/protocol.md) compares
each of the same 1,096 zero-probe references with its immediately preceding
source step. It retains the original population, weights, outcome labels and
follow-up boundaries. A predecessor is historical input context; it does not
become a new available decision. All 63 selected histories are replayed again.

```bash
python3 experiments/pose_observation_loss/analyze_window_turnover.py \
  --output /tmp/fusion-return-window-turnover
MPLCONFIGDIR=/tmp/window-turnover-mpl python3 experiments/pose_observation_loss/plot_window_turnover.py \
  --results /tmp/fusion-return-window-turnover --output /tmp/window-turnover-figures
python3 experiments/pose_observation_loss/verify_window_turnover.py \
  --results /tmp/fusion-return-window-turnover --figures /tmp/window-turnover-figures \
  --work /tmp/window-turnover-independent --output /tmp/window-turnover-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output/work directories and an unused evidence filename, without
Python -O. Activate the saved Rust toolchain or provide `--rustc`. Both readers
accept `--raw-roots`. Collection/verification use the standard library; plotting
uses Matplotlib. No controller rollout or Cargo dependencies are added.

`turnover-decisions.csv` retains earlier columns, adding preceding context,
membership transitions, signed mean/raw/gap steps and their numerical bounds.
When both windows are nonempty, added and removed samples independently predict
the mean update. A full 25-to-25 slide uses (newest−removed_oldest)/25; growth
uses the previous mean and the new count. Empty-side predictions stay blank.
`turnover-members.csv` records the union of both memberships, shared/added/
removed roles and exact raw inputs. Positive probes retain blank new fields.

Among 887 pairs with two nonempty windows, 877 have a certainly larger raw
position step and ten a larger mean step; no comparison is unresolved. All
25 preceding conjunction failures have a larger raw step, with seven growing
windows and 18 slides. These describe recorded adjacent input changes rather
than statistical noise reduction, causal policy benefit or successful motion.

`anchor-turnover.csv` and `turnover-summary.csv` retain original weights and
cells and add phase/branch transition and comparison counts, available-anchor
flags and original case-mass shares. Nonreturns and empty denominators remain
blank. The standalone verifier imports no analyzers, using pinned independent
window replay/means and native helpers plus separate Decimal vector arithmetic.
Both contexts are traced with the original Rust window fragment; duplicate
context queries are distinguished from unique source points. Repeat all four
tables/provenance and plot artifacts byte for byte. See the
[adjacent-window report](results/fusion-return-window-turnover/report.md).

## Compare adjacent point gaps to fixed original blockers

The [fixed protocol](results/fusion-return-point-clearance/protocol.md) follows
all 1,096 preceding zero-probe references and their original definite obstacle
IDs. It compares raw and reconstructed mean start points at n-1 and n, using
each context's own logged speed margin. A predecessor remains input context;
the original population, weights, follow-up and outcome labels are retained.

```bash
python3 experiments/pose_observation_loss/analyze_point_clearance.py \
  --output /tmp/fusion-return-point-clearance
MPLCONFIGDIR=/tmp/point-clearance-mpl python3 experiments/pose_observation_loss/plot_point_clearance.py \
  --results /tmp/fusion-return-point-clearance --output /tmp/point-clearance-figures
python3 experiments/pose_observation_loss/verify_point_clearance.py \
  --results /tmp/fusion-return-point-clearance --figures /tmp/point-clearance-figures \
  --work /tmp/point-clearance-independent --output /tmp/point-clearance-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output/work directories and an unused evidence filename, without
Python -O. Activate the saved Rust toolchain or provide `--rustc`. Both readers
accept `--raw-roots`. Collection and verification use the standard library;
plotting uses Matplotlib. No controller rollout or Cargo dependencies are added.
Both readers hash all 108 raw files and replay the 63 selected histories again.

`point-decisions.csv` retains all preceding fields, adding bounded raw/mean
coordinates, both exact margin strings, point distances and signed gaps,
previous-to-current states, signed distance/margin/gap changes and closure
residuals. Gap changes obey Delta gap = Delta distance - Delta margin. Bounds
use the declared input rounding assumptions. A zero-containing change interval
remains unresolved; inclusive gap equality is blocked. Empty windows use raw
fallback. Positive probes keep blank new fields.

In the fixed 25-reference subgroup, raw point distance decreases in 21 cases
and increases in four, while each logged margin increases by 0.0004 m. Raw
point gaps change from clear to blocked in 15, stay blocked in eight and stay
clear in two; mean point gaps stay clear in all 25. The two clear raw starts
retain their inherited first-chord failures.

Every multi-obstacle set keeps all original IDs and duplicate coordinates.
`point-witnesses.csv` preserves their individual results. Decision-level minima
span only that fixed set, with exact center ties and possible minimum IDs saved
for each context. An obstacle outside the set never enters this diagnostic.
`anchor-point.csv` and `point-summary.csv` retain all prior fields and original
measures, adding phase/branch transition/direction counts, available-anchor
flags and raw/case-mass shares. Nonreturns and empty denominators remain blank.

The standalone verifier imports no analyzers. It uses pinned independent
window/raw readers, new 60-digit Decimal point calculations, and the original
Rust window and Point2D.distance source. Native traces check both contexts and
all rounded raw/mean point-obstacle pairs, including interval closure. These
are point-gap diagnostics: they do not replace the original curved 0.05-m
chord, straight 0.6-m recovery segment, boundary or whole-probe tests. A
first-chord failure can have a clear start point. Repeat all four tables /
provenance and plot artifacts byte for byte. See the
[point-clearance report](results/fusion-return-point-clearance/report.md).

## Inspect the original 25 local point-gap histories

The [fixed protocol](results/fusion-return-point-history/protocol.md) follows
all 25 preceding nominal-only conjunction failures, including the four raw-away
references. Each gets exactly 51 planned slots, offsets -25..25 at 50 Hz.
Positive offsets stop strictly before the original next loss or at source end;
missing slots remain explicit. Negative offsets before the first return are
historical context. Slots do not become new decision references or follow-up.

```bash
python3 experiments/pose_observation_loss/analyze_point_history.py \
  --output /tmp/fusion-return-point-history
MPLCONFIGDIR=/tmp/point-history-mpl python3 experiments/pose_observation_loss/plot_point_history.py \
  --results /tmp/fusion-return-point-history --output /tmp/point-history-figures
python3 experiments/pose_observation_loss/verify_point_history.py \
  --results /tmp/fusion-return-point-history --figures /tmp/point-history-figures \
  --work /tmp/point-history-independent --output /tmp/point-history-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output/work directories and an unused evidence filename, without
Python -O. Activate the saved Rust toolchain or provide `--rustc`. Both readers
accept `--raw-roots`. Collection/verification use the standard library; plotting
uses Matplotlib. The 108 raw files receive fresh hash/grid checks; only the four
histories containing a selected reference receive the new full window replay.
All earlier evidence and the complete reference population remain inherited.

`history-decisions.csv` retains every prior field and adds an audit flag,
pre/post state counts, current-state run lengths, onset censorship and first
certain later-clear fields. Checkpoint count L and span (L-1)/50 s are separate;
a run reaching the left context limit has an unknown earlier onset. Unknown
states break clear/blocked runs. For currently blocked points, a future clear
is an observed point predicate; missing future slots cannot establish a whole
half-second without a clear or any controller recovery.

All 23 currently blocked raw points have a certain clear point 1–3 recorded
steps later (0.02–0.06 s). The four raw-away references were already in blocked
runs of two or three checkpoints and all have a clear point at the next step.
The 60 future slots at/after next loss remain blank. All 565 included positive-
offset mean points are clear; this is a recorded point predicate.

`history-slots.csv` keeps all 1,275 planned slots and exact raw margin/input
strings for included slots. `history-witnesses.csv` preserves every original
fixed obstacle ID; no whole-map minimum is substituted. Reconstructed raw/mean
point gaps reuse each slot's own margin and the preceding rounding enclosures.
The current and preceding point results are checked against inherited fields.
`anchor-history.csv` and `history-summary.csv` retain original weights, cells,
nonreturns and empty share denominators, adding phase/branch counts and flags.

The standalone verifier imports no analyzers. Independent planning and run
scanning use the pinned Decimal window/point helpers and unchanged Rust window
and Point2D.distance source. Every included context and rounded point-obstacle
pair is traced. Repeat the five tables/provenance and figure artifacts byte
for byte. The 5x5 figure shows all 25 references in original order, marks
before-return context and raw-away labels, and breaks lines at missing slots.
See the [point-history report](results/fusion-return-point-history/report.md).

## Check local admission gates and actual timers

The [fixed protocol](results/fusion-return-local-admission/protocol.md) reuses
exactly the preceding 25 references and 1,275 planned slots. It checks whether
a clear point also satisfies the original admission conjunction, and follows
the actual joint/guard evidence counters. The 60 slots at/after next loss stay
missing; the 82 before-return slots remain historical context.

```bash
python3 experiments/pose_observation_loss/analyze_local_admission.py \
  --output /tmp/fusion-return-local-admission
MPLCONFIGDIR=/tmp/local-admission-mpl python3 experiments/pose_observation_loss/plot_local_admission.py \
  --results /tmp/fusion-return-local-admission --output /tmp/local-admission-figures
python3 experiments/pose_observation_loss/verify_local_admission.py \
  --results /tmp/fusion-return-local-admission --figures /tmp/local-admission-figures \
  --work /tmp/local-admission-independent --output /tmp/local-admission-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output/work directories and an unused evidence filename, without
Python -O. Activate the saved Rust toolchain or provide `--rustc`. Both readers
accept `--raw-roots`. Collection/verification use the standard library; plotting
uses Matplotlib. Both readers hash/stream all 108 raw files and independently
replay the four selected window histories. No new rollout is required.

`local-slots.csv` retains every preceding field and adds exact current/previous
raw inputs, binary64 observation age, the original branch, seven source gates,
simultaneous failure/unknown masks and observed timer transitions. Nominal sign
comes from the original 41-value speed-cap domain and stop latch, independently
of counters. Other gates retain the declared rounding intervals. Joint admission
needs 50 consecutive ready ticks; an active guard needs 25. Fusion availability
does not add a gate or reset a timer by itself.

`ready-runs.csv` records maximal known-pass segments, breaking on unknowns,
failures, missing slots, branch changes or a preceding release. A segment with
L checkpoints spans (L-1)/50 s and adds L/50 s of evidence. Actual carried
counts and context-limited endpoints remain explicit; unknown does not imply
that the original unrounded predicate was false. `local-decisions.csv` adds
pre/post counts, segment/counter maxima and checks at the inherited first
later-clear offsets, without adding decision references. `anchor-local.csv`
and `local-summary.csv` preserve the original weights and all summary cells.

The standalone verifier imports no analyzers or plotters. Independent Decimal
gate calculations, timer inference, original-reference joins, run scans and
aggregation check all exports. Unchanged Rust window, speed-cap, recovery and
timer fragments check the native predicates and updates. Native timer inputs
use logged rounded scalars, exact age/speed and source-DT counter reconstruction;
unknown center mismatches are counted explicitly. Guard-entry flags are inputs,
so this is not a new closed-loop replay or whole-map probe audit.

At the inherited first later-clear offsets, 22 of 23 references pass all gates
and advance from zero to one tick; one fails recovery clearance. Across all
included contexts, the longest known-pass segment and largest actual counter
are nine ticks, below the joint threshold of 50. All 25 reference windows have
both known-pass decisions and resets at positive offsets.

The 5x5 figure shows actual active counters for all 25 references, with shared
0..50-tick axes, both thresholds and missing-slot breaks. Repeat all five tables,
provenance and plot artifacts byte for byte. See the
[local-admission report](results/fusion-return-local-admission/report.md).

## Inspect endings of the original known-pass segments

The [fixed protocol](results/fusion-return-run-breaks/protocol.md) reuses the
same 25 reference windows and all 191 known-pass segments. It checks the next
included checkpoint, preserves every simultaneous failed gate and verifies
the actual reset amount, including evidence carried into the context window.
Window/missing boundaries do not establish a reset or an unobserved failure.

```bash
python3 experiments/pose_observation_loss/analyze_run_breaks.py \
  --output /tmp/fusion-return-run-breaks
MPLCONFIGDIR=/tmp/run-breaks-mpl python3 experiments/pose_observation_loss/plot_run_breaks.py \
  --results /tmp/fusion-return-run-breaks --output /tmp/run-breaks-figures
python3 experiments/pose_observation_loss/verify_run_breaks.py \
  --results /tmp/fusion-return-run-breaks --figures /tmp/run-breaks-figures \
  --output /tmp/run-breaks-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output directories and an unused evidence filename, without Python
-O. Collection/verification use the standard library; plotting uses Matplotlib.
This postprocessing audit requires the tracked preceding results. It rechecks
their complete provenance and all 1,215 inherited native admission query/trace
links, without a new raw extraction or dataset replay. Prior raw/native
execution remains inherited evidence. The full existing test suite also runs
its own native unit fixtures with the activated Rust toolchain.

`run-ends.csv` keeps every preceding segment column and adds the next slot,
native row pointers, complete masks and timer decomposition. `reset-edges.csv`
groups only certified observed failures by exact case/source step, requiring
identical inputs, predicates, counters and native outputs across overlapping
windows. It retains multiplicity and all member references. Distinct physical
resets remain descriptive records; they are not independent experiments.

`reference-breaks.csv` contains only the 25 selected original references.
`anchor-breaks.csv` retains all 3,997 anchors and `break-summary.csv` all 336
original cells. Original metadata/denominator strings stay intact; additional
`exact_*` columns reconstruct the original rational weights from case/received
anchor counts and RP parent counts. Aggregation uses these fractions, preserves
nonreturn/empty-denominator blanks and checks additive mode/category closures.

Of 191 segments, 185 end at an observed failed decision and six at the right
window boundary. The failures represent 19 distinct case/step resets: 15 fail
nominal alone, one fails clearance plus nominal and three fail nominal plus
release. All have inherited zero-cap nominal commands. The largest reset loses
nine ticks at C0310 / 175.40 s, with nominal as its only failed gate.

The standalone verifier imports no analyzers or plotters. It independently
rescans segments, classifies end priority, verifies carry/reset arithmetic,
groups physical edges and joins the original census. The frozen two-panel
figure shows all 25 windows and compares overlapping failure endings with
distinct case/step resets by full gate mask. Repeat all five tables/provenance
and all plot artifacts byte for byte. See the
[run-breaks report](results/fusion-return-run-breaks/report.md).

## Link the 19 reset edges to original probe evidence

The [fixed protocol](results/fusion-return-reset-probes/protocol.md) joins all
19 distinct resets to their original decisions by case, event and exact 50-Hz
step. It preserves their 185 overlapping window-ending records and excludes
the six context boundaries. Geometry, native dataset execution and original
weights/outcomes remain inherited; no new extraction or rollout is required.

```bash
python3 experiments/pose_observation_loss/analyze_reset_probes.py \
  --output /tmp/fusion-return-reset-probes
MPLCONFIGDIR=/tmp/reset-probes-mpl python3 experiments/pose_observation_loss/plot_reset_probes.py \
  --results /tmp/fusion-return-reset-probes --output /tmp/reset-probes-figures
python3 experiments/pose_observation_loss/verify_reset_probes.py \
  --results /tmp/fusion-return-reset-probes --figures /tmp/reset-probes-figures \
  --output /tmp/reset-probes-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output directories and an unused proof filename, without Python -O.
Collection/verification use the standard library; plotting uses Matplotlib.
The full test suite also needs the saved Rust toolchain for existing native
unit fixtures. The new audit uses only the tracked preceding results.

`reset-probes.csv` retains every reset-edge field and every original decision
field under `decision_`, adding the original failing stage, aggregate test-gap
bounds and selected blocker IDs. `reset-blockers.csv` keeps every corresponding
point-witness field and the exact global map-coordinate strings. Global IDs
are one-based rows of the entire map; duplicate coordinates stay separate.
The 20 `probe-summary.csv` cells retain empty categories and distinguish
physical-reset counts from overlapping window-ending counts. Only counts
add across partitions; distinct sets and maxima do not.

Seventeen resets have a certainly unsafe initial point; two have a safe initial
point and a directly verified unsafe first chord. All match obstacle IDs 420
(3, 4) or 1310 (34, 35). The inherited mean point gaps to those blockers are
clear in all 19 cases; these point predicates do not replace the original
curved-chord, recovery-segment or whole-probe conditions.

The standalone verifier imports no analyzers or plotters. It independently
joins identities, selects inherited stage evidence, checks map/witness records
and interval states, and verifies all exports. The two-panel figure shows all
19 inherited test-gap intervals and the two counting units by origin. Repeat
all three tables/provenance and all plot artifacts byte for byte. See the
[reset-probes report](results/fusion-return-reset-probes/report.md).

## Compare raw and mean positions at the same reset step

The [fixed protocol](results/fusion-return-reset-displacement/protocol.md)
reuses the same 19 resets, their original single blocker witnesses and all 185
overlapping-ending multiplicities. It compares two position estimates at each
recorded step; it adds no trajectory, decision, window or trial denominator.

```bash
python3 experiments/pose_observation_loss/analyze_reset_displacement.py \
  --output /tmp/fusion-return-reset-displacement
MPLCONFIGDIR=/tmp/reset-displacement-mpl python3 experiments/pose_observation_loss/plot_reset_displacement.py \
  --results /tmp/fusion-return-reset-displacement --output /tmp/reset-displacement-figures
python3 experiments/pose_observation_loss/verify_reset_displacement.py \
  --results /tmp/fusion-return-reset-displacement --figures /tmp/reset-displacement-figures \
  --output /tmp/reset-displacement-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output directories and an unused proof filename, without Python -O.
The standard-library collector/verifier need only tracked preceding results;
Matplotlib produces the figure. Activate the saved Rust toolchain for the full
suite's existing native unit fixtures. New raw reads and native dataset runs
remain zero; preceding geometry/native evidence stays inherited.

`displacements.csv` copies all 469 preceding probe fields and all 72 original
witness fields under `witness_`. It adds position differences, displacement
length, distance/gap gains and the decomposition
`d_mean^2 - d_raw^2 = 2 (raw-obstacle) dot (mean-raw) + ||mean-raw||^2`,
with center/lo/hi values and closure residuals. Both point gaps use the same
current margin, so their gain must agree with the distance gain. The method
does not divide by raw distance and supports a raw point at the obstacle.

Conditional interval arithmetic preserves inherited point rectangles and uses
the frozen 1e-12 elemental padding. A positive lower bound certifies increase;
a negative upper bound certifies decrease; otherwise the sign remains unknown.
The standalone verifier imports no analyzers or plotters and independently
calculates the joins, intervals, signs, extrema and closures with 60-digit
Decimal. Numeric comparison tolerance never resolves an interval's sign.

All 19 conditional distance gains are positive, approximately 0.01743–0.34544 m.
Eighteen linear terms are outward. C0310 at 176.16 s has a negative linear term
but a larger positive quadratic term, yielding a positive distance gain. The
two original first-chord failures retain their direct curved-segment evidence;
these point comparisons do not establish successful motion or policy recovery.

The 20 `displacement-summary.csv` cells keep both counting units, sign counts
and interval enclosures for minimum/maximum gain and displacement length.
Empty extrema remain blank; only counts add across partitions. The two-panel
figure shows every raw/mean point-gap pair and gain/length pair in original
case/step order, marking both first-chord rows. Repeat both tables/provenance
and all five plot artifacts byte for byte. See the
[reset-displacement report](results/fusion-return-reset-displacement/report.md).

## Resolve radial and transverse position differences at the reset edges

The [fixed protocol](results/fusion-return-reset-projection/protocol.md)
projects the same 19 raw-to-mean position differences onto radial and signed
transverse axes of each original blocker. It retains all preceding columns,
origins and 185 overlapping-ending multiplicities. This compares two estimates
at the same recorded step; it adds no motion trajectory or policy experiment.

```bash
python3 experiments/pose_observation_loss/analyze_reset_projection.py \
  --output /tmp/fusion-return-reset-projection
MPLCONFIGDIR=/tmp/reset-projection-mpl python3 experiments/pose_observation_loss/plot_reset_projection.py \
  --results /tmp/fusion-return-reset-projection --output /tmp/reset-projection-figures
python3 experiments/pose_observation_loss/verify_reset_projection.py \
  --results /tmp/fusion-return-reset-projection --figures /tmp/reset-projection-figures \
  --output /tmp/reset-projection-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh directories and an unused proof filename, without Python -O. The
standard-library collector/verifier use only tracked preceding artifacts;
Matplotlib produces the figure. Activate the saved Rust toolchain for the full
suite's existing native unit fixtures. This audit adds no raw extraction or
native dataset replay, and verifies the full inherited input/helper/code chain.

For `u=raw-obstacle`, `v=mean-raw`, `r=||u||` and `d=||u+v||`, calculate
`a=(u dot v)/r` and `b=(u_x*v_y-u_y*v_x)/r`. The new intervals verify
`||v||^2=a^2+b^2`, reconstruct `d=sqrt((r+a)^2+b^2)`, and compare the
distance-gain remainder with `b^2/(d+r+a)`. Signed transverse uses the recorded
x/y convention, with no assigned physical left/right interpretation.

Division requires a certified nonzero interval; these projections require
`r_lo>0`, and the remainder formula also requires `(d+r+a)_lo>0`. Rows with
unresolved denominators remain present with explicit status and blank affected
quantities. Conditional arithmetic preserves inherited boxes and pads each
operation by 1e-12. Sign decisions never use numeric comparison tolerance;
the gain-beyond-radial interval is never clipped to a positive value.

All 19 projections and remainder formulas are complete. Eighteen radial
components are outward and one is inward. The inward case, C0310 at 176.16 s,
has radial component approximately -0.000942784 m and signed transverse
component +0.184340303 m. Their finite-distance reconstruction recovers the
positive distance gain of approximately 0.017426564 m. Every pair has a
positive distance gain beyond its radial component under the frozen bounds.

`projections.csv` retains all 585 preceding fields and adds 12 quantity triples,
status and three sign classifications. The 20 `projection-summary.csv` cells
keep both counting units, status/sign counts and interval extrema, with blank
empty extrema. The standalone verifier imports no analyzers or plotters and
independently performs the new arithmetic with 60-digit Decimal. Repeat both
tables/sources and all five plot artifacts byte for byte. See the
[reset-projection report](results/fusion-return-reset-projection/report.md).

## Decompose point clearance at the same reset edges

The [fixed protocol](results/fusion-return-reset-margin/protocol.md) keeps the
same 19 original reset/blocker pairs and all 185 overlapping-ending records.
It separates each original mean point gap into raw point gap, radial component
and gain beyond radial. Every preceding projection column remains unchanged.

```bash
python3 experiments/pose_observation_loss/analyze_reset_margin.py \
  --output /tmp/fusion-return-reset-margin
MPLCONFIGDIR=/tmp/reset-margin-mpl python3 experiments/pose_observation_loss/plot_reset_margin.py \
  --results /tmp/fusion-return-reset-margin --output /tmp/reset-margin-figures
python3 experiments/pose_observation_loss/verify_reset_margin.py \
  --results /tmp/fusion-return-reset-margin --figures /tmp/reset-margin-figures \
  --output /tmp/reset-margin-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh output directories and an unused proof filename, without Python -O.
The standard-library collector/verifier require only tracked preceding results;
Matplotlib produces the figure. Activate the saved Rust toolchain for the full
suite's existing native unit fixtures. Raw extraction and native dataset
execution remain inherited; the new audit adds neither.

For radial component `a` and gain beyond radial `e`, form
`radial_gap=raw_gap+a` and `rebuilt_mean_gap=radial_gap+e`. Verify the original
mean gap, the endpoint-minus-common-margin expression, the reconstructed
distance-minus-margin expression and the remainder with four residual checks.
Conditional add/subtract intervals retain the frozen 1e-12 padding. Clear
requires lower>0, blocked requires upper<=0, otherwise the state is unknown;
numeric comparison tolerance never changes a point state.

The 17 originally blocked raw points all have a positive `raw_gap+a` interval.
The remaining two pairs already have clear raw and mean points and retain their
original directly unsafe first-chord evidence. These are same-step point-gap
identity categories, not successful controller recoveries or path tests.

`margins.csv` retains all 625 preceding columns and adds nine quantity triples,
status, two gap states and a frozen six-way point category. An incomplete
projection remains as a row with blank new quantities and explicit status.
The 20 `margin-summary.csv` cells retain both counting units, all categories
and interval extrema; empty extrema stay blank, and only counts add across
partitions. The standalone verifier imports no analyzers or plotters and
independently checks the arithmetic/classifications with 60-digit Decimal.
Replay both tables/sources and all five plot artifacts byte for byte. See the
[reset-margin report](results/fusion-return-reset-margin/report.md).

## Measure radial surplus relative to the original point deficit

The [fixed protocol](results/fusion-return-reset-surplus/protocol.md) retains
all 19 original reset/blocker pairs and all 185 overlapping-ending records.
Only the preceding 17 blocked-to-clear/radial-clear point pairs are eligible
for deficit/radial fractions. Both already-clear first-chord rows remain
explicitly not applicable, with blank fractions rather than zeros.

```bash
python3 experiments/pose_observation_loss/analyze_reset_surplus.py \
  --output /tmp/fusion-return-reset-surplus
MPLCONFIGDIR=/tmp/reset-surplus-mpl python3 experiments/pose_observation_loss/plot_reset_surplus.py \
  --results /tmp/fusion-return-reset-surplus --output /tmp/reset-surplus-figures
python3 experiments/pose_observation_loss/verify_reset_surplus.py \
  --results /tmp/fusion-return-reset-surplus --figures /tmp/reset-surplus-figures \
  --output /tmp/reset-surplus-verification.json
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use fresh directories and an unused proof filename, without Python -O. The
standard-library collector/verifier use tracked preceding results; Matplotlib
produces the figure. Activate the saved Rust toolchain for the full suite's
existing native unit fixtures. Raw extraction and native dataset execution
remain inherited; the new audit adds neither.

For original raw gap `h`, radial component `a` and radial gap `H=h+a`, compute
`deficit=0-h`, `surplus=a-deficit`, `required_fraction=deficit/a` and
`surplus_fraction=surplus/a`. Division requires `a_lo>0`. Verify that the two
fractions sum to one, the surplus fraction matches `H/a` and the surplus
matches the preceding radial gap. Conditional intervals retain 1e-12 padding;
fractions are never clipped to [0,1]. Strict full bounds classify relations
to one and signs, without numeric comparison tolerance.

All 17 eligible pairs have required fractions below one and positive surplus
fractions. The raw deficits account for approximately 1.405–91.379% of their
recorded radial components; the remainder accounts for 8.621–98.595%. These
are same-step algebraic fractions, not controller settings, simulated changes
or recovery probabilities. The original unsafe chord evidence is retained.

`surplus.csv` preserves all 656 preceding fields and adds eight quantity
triples, status and two classifications. Incomplete margins or uncertain
radial denominators remain explicit with blank new quantities. The 20
`surplus-summary.csv` cells keep all physical/overlapping counts, computed
multiplicities, every status/classification and interval extrema. Only counts
add; empty extrema stay blank. The standalone verifier imports no analyzers or
plotters and independently checks the new calculations with 60-digit Decimal.
Replay both tables/sources and all five plot artifacts byte for byte. See the
[reset-surplus report](results/fusion-return-reset-surplus/report.md).

## Review and replay the reset evidence

The [consolidated review](results/fusion-return-reset-review/report.md) connects
all 19 original reset decisions to their point gaps, original failing probe
stage, full simultaneous failure mask, zero-cap nominal command and timer
before/after values. Its 39-column CSV copies original strings and keeps both
nonapplicable first-chord rows, including blank ratio values.

```bash
python3 -m pip install --only-binary=:all: -r experiments/pose_observation_loss/requirements-replay.txt
MPLCONFIGDIR=/tmp/reset-review-mpl python3 experiments/pose_observation_loss/review_reset_evidence.py \
  --output /tmp/fusion-return-reset-review
python3 -m unittest discover -s experiments/pose_observation_loss -p 'test_*.py' -q
```

Use Python 3.12 on Linux for this pinned rendering profile and a fresh output
directory outside the checkout, without Python -O. The runner uses the
standard library; the existing plotters require Matplotlib.
The full Python suite also needs NumPy (see [continuous checks](#continuous-checks))
and the activated saved Rust toolchain for its existing native unit fixtures.
The review runner adds no raw extraction,
source observations or native dataset execution.

The command reruns all six collectors, plotters and standalone independent
verifiers: run-breaks, reset-probes, reset-displacement, reset-projection,
reset-margin and reset-surplus. Each keeps its original committed input paths;
the runner compares all replayed tables/sources, SVG/CSV/plot manifests and
independent proof files byte for byte with frozen evidence. It retains fresh
PNGs for comparison between review runs. This repeats existing arithmetic,
including 19 displacement, 19 projection, 19 margin and 17 ratio pairs, with
their original input boxes; it creates no new experimental trials.

`reset-evidence.csv` contains all 19 rows × 39 fields. `study-index.csv` links
the six reports/scripts/proofs and retains each stage's full original audit
JSON with its original units. `review-sources.json` records frozen inputs,
all fresh replay hashes and the two overview CSV hashes. Per-stage logs and
full replay outputs remain under `OUTPUT/stages/`. A failed child preserves
its failure status and logs; a completion manifest is written only after all
stages and exact comparisons pass. Existing output directories are rejected.

For a second replay, choose another fresh output path. Review manifests omit
timestamps and output paths so they can be compared bytewise; their stage
hashes include the PNGs. The saved review was checked with two full runs,
including every generated collector, plot and proof file.

To check a completed replay after copying or receiving it:

```bash
python3 experiments/pose_observation_loss/check_reset_review.py \
  --review /tmp/fusion-return-reset-review
```

This read-only command needs only the Python standard library and the pinned
checkout. It checks all 64 artifact files, six stage records, 91 frozen inputs,
the recorded runner/protocol/catalogue, and both overview exports. Each PNG
must match its own completion-manifest hash; PNGs have no committed image
baseline. The other stage artifacts also match the frozen repository bytes.
The success JSON describes stored-file integrity. Use the replay command above
for fresh collector, plotter and numerical verifier execution.

Keep the three top-level CSV/JSON files and the complete `stages/` tree.
Stage logs may be retained or discarded; their contents are outside the
integrity claim. Missing/extra artifacts, internal symlinks, changed provenance
or mismatched hashes fail with a nonzero status and a diagnostic on stderr.
The checker writes no files and runs without Python -O.

All 19 original commands have cap_zero and a failing nominal gate; four also
have clearance/release failures, preserved in the full masks. Seventeen
original probes fail at the initial point and two at the directly checked
first chord. Mean point gaps to their original blockers are clear in all 19,
while the original timer resets remain zero after the decision. These are
different recorded predicates; a clear mean point does not replace a curved
probe or establish a successful recovery policy.

## Replay the reset probes at mean positions

The [fixed mean-position diagnostic](results/fusion-return-reset-mean-probe/report.md)
keeps the same 19 reset selections, original yaw, nominal steering, margins and
map. It changes only x/y to the inherited stationary mean and evaluates the
whole original 40-chord speed probe. Both raw and mean inputs are reconstructed
rounded centers, rather than bounds on hidden unrounded states.

```bash
python3 -B experiments/pose_observation_loss/analyze_reset_mean_probe.py \
  --output /tmp/reset-mean-probe
python3 -B experiments/pose_observation_loss/verify_reset_mean_probe.py \
  --results /tmp/reset-mean-probe --work /tmp/reset-mean-probe-native \
  --output /tmp/reset-mean-probe-verification.json
```

Use Python 3.12, an activated Rust compiler and fresh paths outside the checkout.
Only the Python and Rust standard libraries are needed. The collector reads the
tracked 19-row input extraction and pinned preceding evidence. The standalone
verifier imports no collector: it compiles the unchanged Rust probe and distance
functions, queries all 38 centers and checks all failure steps/witnesses. It also
checks the existing speed-cap/nominal-target formulas with 60-digit Decimal.

All 19 raw probes reproduce zero. At the mean positions, 18 return the full
2.0-unit horizon; C0310@8795 returns 0.20 before the fifth chord fails against
obstacle 1310. Every mean nominal target is positive, while all original
simultaneous failure masks and ending multiplicities remain recorded. These
same-step outputs do not establish new timer, release, trajectory or hold results.

The two original raw logs can also rebuild the compact input packet:

```bash
python3 -B experiments/pose_observation_loss/analyze_reset_mean_probe.py \
  --extract-raw --output /tmp/reset-mean-probe-inputs
```

The original paths come from the pinned first-chord source manifest. If those
logs were relocated, add `--raw-root /path/to/original/fresh-white`; both full
file hashes must still match. Pass `--inputs /tmp/reset-mean-probe-inputs` to
both replay commands to use that new extraction. No trajectories are regenerated.

Two extractions, collections and native verifications reproduced nine files
byte for byte. The full test suite replays the tracked packet, verifies the new
results with the native function and includes synthetic initial/first/later
chord, inclusive wall/obstacle, duplicate and near-zero-steering cases.

## Run the closed-loop mean-position probe pilot

The [predeclared pilot](results/mean-speed-probe-live-pilot/protocol.md) tests
whether using the existing stationary recovery mean for the speed probe's x/y
changes actual recovery and hold outcomes. It fixes one new seed, two noise
correlation times, three availability modes, three maps and three planners:
108 cases, or 54 raw/mean pairs. Keep the complete fixed grid, including failures.
The [recorded report](results/mean-speed-probe-live-pilot/report.md) retains all
outcomes: both arms held in 24/54 cases, with no gained or lost holds in any
pair. This pilot did not establish an increase in successful holds.

```bash
python3 -B experiments/pose_observation_loss/run_mean_speed_probe.py \
  --output /tmp/mean-speed-probe-raw
python3 -B experiments/pose_observation_loss/analyze_mean_speed_probe.py \
  --raw /tmp/mean-speed-probe-raw --output /tmp/mean-speed-probe-audit
```

Use Python 3.12, the activated Rust/Cargo toolchain, a clean committed checkout
and fresh output paths outside the repository. The runner requires at least
2 GiB free on the output parent's filesystem and uses only standard libraries
and the existing locked Cargo dependencies. It verifies all frozen inputs,
generates the complete candidate source and diff outside the checkout, and
compiles it against the unchanged original example's exact Cargo libraries.
It preserves the original simulator, library defaults, manifests and lockfile.

The generated example adds an optional final `MEAN_SPEED_PROBE` boolean,
default false. The mean arm changes only probe x/y when already paused,
measured absolute speed is at most 0.02, observation age is at most 1.5 seconds,
and the existing window contains at least 10 samples. Raw yaw, original nominal
steering, margins, geometry and every other policy calculation are retained.
The original 82-column trajectories/results keep their format; a separate
`speed-probes.csv` records every decision and full-precision physical values
for independent verification. Physical truth is never a new controller input.

The runner retains all 108 cases and 18 additional original/default regression
cases. The latter must reproduce the complete explicit-false trajectories and
results byte for byte and are excluded from the treatment denominator.
`provenance.json` records source commit, commands, compiler/library/binary/source
hashes and every raw file hash. Preserve the complete raw packet for replay.

The auditor reconstructs every existing averaging window and eligibility
decision, checks full-precision physical dynamics, braking, swept contacts,
bounds, holds and inherited gate/filter histories, and checks paired common
prefixes and latent observations. Independent full-probe geometry replay uses
the protocol's fixed sampled steps; `probe-samples.csv` retains all checks.
`cases.csv`, `pairs.csv`, `summary.csv` and `verification.csv` retain the complete
population. `sources.json` is written only on successful completion; failures
retain sampled evidence and a diagnostic. Repeat with a second fresh output
path to compare all five CSV files and `sources.json` byte for byte.

The small native fixture and selector tests run in the full Python suite; CI
does not rerun the 108-case pilot. This one-seed engineering pilot cannot
establish a population recovery rate or a safety guarantee. The option remains
experimental and disabled by default regardless of its results.

## Diagnose the live pilot's admission gates

The [fixed admission diagnostic](results/mean-speed-probe-admission-audit/protocol.md)
uses every recorded case from the 108-case live pilot. It queries the original
recovery-line geometry at every paused decision, reconstructs all seven joint
admission gates and checks the one-second continuous timer. It retains complete
failure masks and all definite-ready runs, including runs ending in failure.
Cases without a loss pause remain in the exported population.
The [recorded report](results/mean-speed-probe-admission-audit/report.md) shows
that 9,276 of the mean arm's 11,998 eligible positive commands still failed a
different admission gate. It retains all 482 readiness runs and the extra
admission that still ended in a timeout.
The [margin amendment](results/mean-speed-probe-admission-audit/protocol-amendment-1.md)
records a rejected initial assumption: the curved configuration has separate
recovery and guard margins. The native reader reconstructs both original
expressions from the fixed command parameters and recorded precision epochs.

```bash
python3 -B experiments/pose_observation_loss/analyze_mean_probe_admission.py \
  --raw /path/to/mean-speed-probe-raw --output /tmp/mean-probe-admission
```

Use Python 3.12, an activated Rust compiler, a clean committed checkout and a
fresh external output path. Preserve the complete original raw packet. Source,
fixtures, the packet manifest and all raw trial hashes must match the frozen
pilot. This command reads retained observations and runs native geometry queries;
it creates no new simulator steps and changes no control thresholds.

The original release-preview clearance has nine decimal places. Its closed
rounding interval must establish the strict release comparison; a threshold
overlap remains unresolved and is retained. The diagnostic does not infer a
gate's truth from the observed timer. `case-gates.csv`, `gate-patterns.csv`,
`admission-runs.csv` and `summary.csv` preserve the complete case set, overlapping
failures, verified/unverified timers and endpoint brake flags. Native queries
and traces remain under the output's `native/` directory. Completion provenance
is written only after all joins and coverage checks pass. Use a second fresh
output path to compare all tables and provenance byte for byte.

## Inspect mean-probe geometric witnesses

The [fixed witness protocol](results/mean-speed-probe-geometry-witnesses/protocol.md)
retains all 54 mean-arm cases and every eligible positive command, including
ready controls. It replays the recorded speed probe and recovery line using
the original Rust functions, retains all nearest obstacle ties, and compares
three static queries at the same mean position: recorded yaw/steering, mean
yaw with the same steering, and mean yaw with zero steering.
The [recorded report](results/mean-speed-probe-geometry-witnesses/report.md)
retains all populations, the terminal-release component census and three
fixed geometric diagrams.

```bash
python3 -B experiments/pose_observation_loss/analyze_probe_geometry.py \
  --raw /path/to/mean-speed-probe-raw --output /tmp/probe-geometry
python3 -B experiments/pose_observation_loss/plot_probe_geometry.py \
  --data /tmp/probe-geometry --output /tmp/probe-geometry-plot
```

The collector requires the complete frozen pilot packet, an activated Rust
compiler and a clean committed checkout. The plotter uses the existing pinned
Python 3.12 / Matplotlib 3.10.8 replay profile. Both commands require fresh
external output directories and reject Python optimization. Run each twice
and compare all evidence and figure bytes. No new vehicle trajectories are
generated and no control thresholds are changed.

The stopping and terminal-preview logs have nine decimal places. Their closed
rounding intervals are converted outward to binary floats and passed through
the original aligned release function. A component overlapping the strict
release threshold remains unresolved. All case/mask cells, including empty
cells, are exported. Compact representatives select the first and minimum
recovery-gap decision per case/mask; all tied nearest obstacles remain attached.
Full queries, traces and witnesses stay under the output's `native/` directory,
bound by completion hashes. The plots show the first decision of each failure
mask and label the guard-pose marker as rounded. Same-step query differences
cannot establish the outcome of a modified controller.

## Replay the exact terminal-preview input history

The [fixed terminal-input protocol](results/terminal-preview-input-replay/protocol.md)
replays every mean-arm sensor decision in the retained pilot using the original
Rust motion filter and stationary-epoch position fusion. It checks the logged
filter state and rounded preview pose at every step before evaluating the
selected positive commands. Physical truth and rounded poses are excluded from
the estimator inputs.

```bash
python3 -B experiments/pose_observation_loss/analyze_terminal_preview.py \
  --raw /path/to/mean-speed-probe-raw --output /tmp/terminal-inputs-1
python3 -B experiments/pose_observation_loss/analyze_terminal_preview.py \
  --raw /path/to/mean-speed-probe-raw --output /tmp/terminal-inputs-2
```

Use a clean committed checkout, Python 3.12, the activated Rust compiler and
fresh external output paths. The command rejects Python optimization and
unsupported preview configurations. It verifies the full frozen raw packet,
requires exact original nine-decimal filter/preview values and sample counts,
and requires the recorded query's stop, terminal and combined clearances to
match the logs exactly at their recorded precision.

Each selected decision retains four static queries: original preview inputs,
mean x/y with the original preview heading, mean x/y and mean heading, and the
original preview with a zero target. Speed, steering, margins and map stay
fixed except for the indicated factor. The zero target diagnoses the proposed
stopping-motion prefix and is not a positive-command admission candidate.
No controller trajectory, timer or threshold is changed.

All cases, original gate masks, ready controls and both directions of release
flips are retained. `case-summary.csv` and `summary.csv` preserve empty cells.
Compact exports use the previous study's exact 132 representative IDs and
roles, retaining all four variants. Their static inputs can be replayed with
the archived Rust source's `direct` mode. Full sensor histories, native traces
and classifications remain outside the checkout, bound by completion hashes.
Compare both full run directories byte for byte; successful completion
provenance is absent when a reconstruction or join fails.
