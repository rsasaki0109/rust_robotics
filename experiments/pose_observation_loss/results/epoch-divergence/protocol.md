# Fixed diagnostic of live epoch-preview divergences

Use the published epoch-preview experiment at main ae2e6da. Keep all 324 pairs
in the case accounting. Select every pair whose recorded first_command_change_s
is nonempty: 121 pairs, including 111 timeout/timeout, six held/held, three
held/timeout and one timeout/held. Do not select checkpoints by estimator error
or successful geometry. No new trajectories, thresholds or controller changes.

Verify the published case/pair/source hashes and the simulator/helper sources.
Verify each consumed raw trajectory hash against the previous report. Earlier
full motion, contact, filter and timer validation applies to these identical
bytes. Preserve current run provenance; when regenerating raw data, require
the same simulator, seeds, noise and input hashes rather than its start time.

For every selected pair find the first changed applied command, physical state
and guard/loss action. Before the first changed action, check shared physical
state, nominal inputs, observation events and original filtered pose. Track the
first changed entry classification, active release readiness and timer evidence
within this shared-history prefix. Distinguish a current geometry difference
from evidence accumulated over preceding steps. Retain cases where fusion is
unavailable at the action itself. Verify the first command time against the
published pairs and keep all four outcome-changing pairs in the summaries.
For a release, locate the last equal value of its relevant timer before the
action and retain the next step as the origin of the surviving evidence
difference. Verify that the two active readiness inputs differ there. An older
difference followed by timer resynchronization does not explain the action.

At the first changed action and first changed command, independently compute
stopping/terminal geometry for each arm at four poses: logged estimate, original
filtered x,y with the logged heading, true x,y with the logged heading, and full
true pose. Retain the same measured speed, nominal steering/target and margins.
Apply the same audit at the first readiness/evidence difference and at the
origin of the surviving evidence difference; deduplicate coincident steps.
Also evaluate the common endpoint, each arm's own endpoint, and its first fresh
stationary observation of age 0.5 in its final 30 seconds. Deduplicate coincident
checkpoints. Record whether the paired pre-command states/inputs still agree.
Only shared-history comparisons identify the initial local effect of x,y;
after divergence these are same-row diagnostic substitutions, not alternate
closed-loop predictions. Truth is evaluation input only.

For each arm summarize the final 30 seconds at its own endpoint and at the
shorter shared endpoint. Count union wait, travel, goal distance/progress,
guard/loss command modes, active independent/joint release requirements and
consecutive ready runs. Count overlapping failures of freshness, stationarity,
samples, raw recovery geometry, positive command, stopping and terminal
clearance. Also show maximum consecutive readiness when individual preview
requirements are omitted; this is a descriptive upper bound with recorded
states/inputs, not a policy rollout or proof of reachability.

Report all selected pairs, cohort/outcome groups, initial direction of the guard
action, initial position error and estimated/true geometry, then the three lost
holds and the gained hold with their subsequent waits and terminal state.
Do not attribute the final outcome solely to the first local difference or
extrapolate a diagnostic pose substitution into a successful controller.
Save portable case, prefix, geometry, tail and requirement tables plus source
and raw fingerprints. Repeat aggregation byte for byte. Keep controller
defaults and the already published result files unchanged.

Diagnostic refinement after the first aggregation: one unequal-timer episode
starts with epoch ahead and later reverses without passing through equal
counters. Therefore the last equal counter identifies the episode's start,
not necessarily the readiness check that delayed the eventual later release.
Keep all original checkpoints and add the last failed readiness check of the
later arm within the earlier arm's uninterrupted ready run ending at the first
different action. Audit its geometry in the same four views. For entry forks
this additional point is the action itself. This changes no case selection,
controller, thresholds, raw data or outcome counts. Preserve the initial
aggregation and require deterministic repetition of the refined diagnostic.
