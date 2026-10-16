# Fixed live comparison of straight and commanded-steering terminal previews

Start from main f6d6f2d. Compare epoch (stationary epoch fusion with a straight
terminal preview) and curved (identical controls with a constant-steering
terminal arc). The optional TERMINAL_STEERING_PREVIEW flag defaults to false and
requires epoch fusion. Only the .6 terminal geometry from the forecast stopping
pose changes. Use the existing nominal steering; never search angles or read
truth for control. Preserve the stopping prefix, entry/release margins, raw
recovery straight line, speed probe, estimator, noise, losses and .5/1s timers.
Keep the original straight implementation in the disabled path.

Freeze known seeds 269,270,271 so all four previously hold-changing pairs are
included. Compare new seeds 274,275,276, absent from the previous epoch-preview
live matrix. Use independent noise (tau=0) and stationary correlated noise
(tau=1s), all loss modes none/random/blackout, three maps and three planners.
Each of four cohorts has 81 paired cases: 324 pairs, 648 trajectories. Fixed
padded fixtures and all other model parameters remain those of epoch-preview.
Run every case and report every outcome; do not tune geometry, thresholds or
seeds in response to the results. The known group is a diagnostic set; new
seeds assess transfer within this fixed simulator, not real-world safety.

Build the locked release example with existing cached dependencies and suitable
parallelism. Test exact arc geometry, interior wall minima, near-zero steering,
reflection, both improvements and losses of clearance, and stopping-prefix
invariance. Extend Python reconstruction to the new opt-in variant. Reconstruct
noise, fusion, recovery, guard transitions and hold outcomes on all trajectories.
Independently audit Euler motion, every radius-.5 swept contact and map bounds.
Check stopping and terminal geometry at fixed first events, periodic points,
all resumes/releases and endpoints; audit arcs with an independent 120-chord
construction and sagitta bound. Do not call heuristic-margin failures contacts.

For all 162 known baseline trajectories compare every old 81 CSV fields and
summary bytes with the pinned prior epoch trajectories (269/270 in known cohorts,
271 in prior new cohorts). Require the original simulator/input/raw hashes on
those reference bytes. Appending the new flag column must change no old fields.
In one nine-case baseline invocation, omit the new CLI flag and compare the full
new trajectory/summary bytes with explicit false. Preserve raw data and provenance.

Report held/timeout/contact/out-of-bounds, gained/lost holds by cohort and loss
mode, completion differences for both-held pairs, common-horizon wait, short
re-entry bouts, final goal distances, first applied-command differences, and
all four previously hold-changing pairs. Keep collision detection separate from
preview estimates. Save portable case/pair/summary/verification CSVs, source and
raw fingerprints, and exact commands. Repeat aggregation byte for byte. Preserve
the default disabled state and all previous published report artifacts.
