# Fixed diagnostic of recovery-line and positive-speed admission gates

Start from main 1a46864 and the pinned terminal-arc-live raw records. Retain all
648 cases in accounting and analyze all 368 timeouts (184 epoch, 184 curved),
including cases without changed paired commands and without joint recovery in
their final window. Do not select by geometry, error, movement or outcome change.
No controller changes, new trajectories, tuned margins or steering search.
Pin prior tables/provenance, simulator/helpers, map and consumed raw hashes;
reuse previous full physics/contact/filter/timer validation for identical bytes.

For every selected case evaluate every interval in its own final 30s. Reconstruct
the active joint requirements with precise measured speed, including the recovery
line's inclusive wall bounds and strict obstacle clearance. Count overlapping
failures, fresh/stationary/sample opportunities and maximum consecutive readiness
for actual requirements and fixed masks: omit recovery geometry; positive command;
both raw gates; stopping/terminal preview; or all geometry/speed/preview (leaving
stationarity, freshness and samples). Reset runs outside the joint route. These
are descriptions of recorded states, not alternative controller rollouts.

Choose the endpoint plus, in the final 30s while joint-pending, each correction
epoch's first stationary/fresh observation with at least 10 recovery samples,
and every stationary/fresh observation at age .5 with at least 10 samples.
Deduplicate coincident case/time points and retain cases with no such opportunity.
Use the logged recovery pose even with zero samples: the simulator then falls
back to the instantaneous sensed pose. Mark sample count and admission usability.

At each checkpoint retain measured speed, uncapped target, nominal steering,
position sigma and observation age. Evaluate both gates at six poses: sensed;
actual recovery input; stopping-preview input; true x,y with sensed heading;
true x,y with recovery heading; full truth. Truth is evaluation input only.
The recovery path is a .6 straight segment. Positive speed requires uncapped
target>0 and the first .05 chord of the same fixed-steering speed probe to clear
the margin: a blocked initial point or first chord yields free distance zero.
Independently reconstruct the full 40-chord raw speed probe and compare its
logged scalar; check the short-chord criterion against it. Check the recovery
input's exact obstacle distance against the logged obstacle-only clearance.

Use M=.6+2sqrt(2)*sigma+.02*age, also checking its logged speed-margin value.
Keep obstacle and wall clearances separate: recovery permits wall equality but
requires obstacle distance>M; the speed probe requires strict clearance at walls
and obstacles. Record ambiguity within 2e-8 and do not claim a boundary-equal
wall makes recovery impossible. Record the true initial center minimum and the
zero-age floor M0=.6+2sqrt(2)*sigma. C<M0-2e-8 is a necessary-condition obstruction
to any same-margin path from that fixed position and sigma, even after refreshing
age. Keep the physical radius .5 comparison separate from these heuristic margins.

Report all timeout cases, actual/base-only continuous readiness, and checkpoint
comparisons by cohort, variant and overlapping checkpoint subset (all, first
usable epoch, age .5, endpoint). Distinguish excessive estimated blocking from
true same-margin blocking and optimistic permissions. Highlight the three curved
lost holds with zero final-window travel: known-white/random/270/maze/Theta*,
known-correlated/blackout/270/maze/Theta*, new-white/blackout/274/maze/Theta*.
Compare recovery and preview positions without assuming averaged samples are
independent or changing sigma. A speed-input substitution would also change the
stopping forecast: do not label a combined substituted readiness as a rollout.

Save accounting, per-case requirements/masks, checkpoints, pose geometry, decision
comparisons, subgroup summaries and raw/source fingerprints. Repeat aggregation
byte for byte. If a figure helps, render fixed age-.5 truth/estimate comparisons
from those tables rather than selecting favorable examples. Preserve previous
published reports, experimental defaults and application code.
