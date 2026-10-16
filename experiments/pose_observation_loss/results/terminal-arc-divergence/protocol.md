# Fixed diagnosis of the live terminal-arc divergences

Use main 41f4db8's terminal-arc-live experiment, without new trajectories or
controller changes. Retain all 324 pairs in accounting and select all 211 pairs
with a nonempty first_command_change_s. Include all six gained and six lost
holds, and unchanged outcomes. Pin prior case/pair/source tables, simulator,
helpers, map and every consumed raw trajectory. Reuse the prior full physics,
contact, filter and timer validation only after raw hashes match.

For each pair locate the first differing guard/loss action, applied command and
physical state. Verify the published command time and the shared pre-action
physical state, observation/filter inputs, preview pose, nominal controls,
margins and raw recovery inputs. Classify the first action as guard entry,
independent release or joint resume, recording which arm acts first. Do not
assume the curved arm always releases earlier or always avoids entry.

For releases, trace the relevant counter from its last equal value and find the
last failed readiness check on the later arm within the earlier arm's continuous
ready run. A timer difference that was reset does not explain a later action;
an unequal-timer lead can also reverse. Record the decisive checkpoint separately
from the action, including cases where current geometry already agrees. Reuse
the tested generic timing helpers, translating their legacy arm names explicitly.

At first entry/readiness/evidence difference, timer origin, decisive point, action
and command, evaluate both straight and commanded-steering .6 terminal paths.
Also evaluate each arm's first positive-command release/resume at or after the
fork, common/own endpoints, first fresh stationary observation of age .5 in the
own final 30s, and the shared post-fork 5s/30s endpoints. Deduplicate arm/time
checkpoints. Use three pose views: logged, true x,y with logged heading, full true
pose. Keep measured speed, nominal target/steering and margins. Recompute exact
stopping/terminal geometry and audit both terminal paths with independent
120-chord integration and the sagitta bound. No steering search or truth-based
selection. Record ambiguity within 2e-8 and physical radius .5 separately.

At the decisive point use the relevant entry/release gate. Identify the arm whose
geometry permits motion: the arm avoiding entry or the arm releasing first.
Compare its exact logged and true classifications, as well as the denying arm's
true classification. Report true agreement, optimistic permission, conservative
denial, heading effects and which stop/terminal component fails, by cohort and
outcome. These are local comparisons of two geometric paths, not proof that the
first local permission determines the eventual hold.

Describe both arms from the state immediately before the first action over the
same 5s and 30s horizons, clipped to the shorter trajectory: travel, union wait,
positive commands, guard entries/releases, resumes, speed and true goal progress.
Summarize each arm's own and common final-30s windows with active release
requirements and maximum readiness under fixed requirement masks. Masks describe
recorded states only; do not present them as alternative policy rollouts. Report
all 12 changed-hold pairs individually, their first local classifications and
subsequent movement/goal distance, without selecting by estimator accuracy.

Save accounting, forks, geometry, post-fork windows, tails, requirements, summaries
and raw/source fingerprints; repeat aggregation byte for byte. If a figure is
useful, fix the example to the previously reported largest new-seed lost-hold
distance increase: new-white/blackout/274/maze/Theta*. Show its recorded true goal
distance and guard/loss waits for both arms; it is illustrative, not representative.
Keep all prior published results and controller defaults unchanged.
