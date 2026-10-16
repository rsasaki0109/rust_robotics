# Position and heading envelopes for the first speed-probe chord

Start from main c7c40f8. Preserve speed-probe-starts' complete 648-case
accounting, 368 timeout cases, 2,583 opportunities, 6,610 audited distinct
states, all three estimated inputs and the original 550 raw-zero-loss pairs.
Keep the fixed 509 command-only members, all cohorts, both recorded arms,
event kinds and right-censoring. Non-audited opportunities remain explicit;
they do not become blocked or acquire future observations.

Pin the prior accounting, events, poses, pairs, start-blocks and provenance,
including inherited map/simulator/helper hashes. Use tracked artifacts; no new
raw replay is required. Reuse original pose/state/transition/event IDs. Fixed
steering, original speed_margin, pose headings and same-margin truth remain
unchanged. Neither a full speed cap nor another release gate is recomputed.

For every one of the 19,830 estimated poses record Euclidean position error,
absolute wrapped heading error in [0, pi], the true signed first-chord gap,
the estimated signed first-chord gap, their difference and margin shortfall
max(0, -truth_gap). The first chord includes its start. A truth margin
shortfall is not physical contact or an uncertainty-free collision result.

Use deterministic product sets: a closed position disk of radius r and a
closed heading interval +/-d, with the same steering. Freeze r at
0, .05, .10, .15, .20, .30 m and d at 0 and 4 degrees. These are sensitivity
settings, not confidence levels, sensor calibration or recommended margins.
No radius is selected or tuned using truth labels.

For endpoint displacement length c=|end-start|, heading variation moves every
point on the first chord by at most e=2*c*sin(d/2). Distance to the map's
obstacle set and minimum signed wall distance are 1-Lipschitz under chord
Hausdorff displacement. Translation adds at most r. Retain separate start
constraints, whose rotation displacement is zero. At the fixed margin let
so, sw, co, cw be the four source signed gaps. The pose's clear radius budget
is min(so,sw,co-e,cw-e); its block radius budget is
max(-so,-sw,-co-e,-cw-e). At radius r, certify clear if clear_budget-r>2e-8 m;
otherwise certify blocked if block_budget-r>=2e-8 m; otherwise unresolved.
This conservative rounding guard leaves boundaries unresolved. Source exact
blocking equality remains <=0, including walls. Never equate unresolved
with blocked. A clear certificate means every pose in that product set has
a clear first chord; a blocked certificate means every pose in it blocks.

Export both budgets for each yaw setting once per estimated pose, not a huge
duplicate full-state grid. Classify every pose at all 12 frozen settings in
summaries. For the complete 550-loss subset also export all 19,800 individual
decisions, retaining raw/recovery/preview views of each original transition.
The 509 command-only subset is a flag on those same rows, not new attempts.

Measure position and heading coverage separately, their joint coverage, and
clear/blocked certificates versus same-margin truth. Any certificate with
covered truth must agree with its truth classification; assert this for all
poses/settings. Also use each pose's observed error and heading difference
only as an explicitly labeled oracle diagnostic: its Lipschitz interval must
contain the actual true gap, and any oracle certificate must agree with truth.
The oracle is unavailable to an online controller and is not a calibrated
uncertainty estimate. Summarize geometry conflicts and shortfalls separately
from these conditional guarantees. Save all poses, including those with
correct central classifications and states preceding readiness loss.

Summaries use all audited states, raw_zero_loss and command_only_zero_loss,
with cohort/arm/event-kind/input/setting dimensions and explicit empty groups.
Distinct state counts do not sum across overlapping source event kinds;
event-linked loss rows count original transitions, separately for each input.
Export case/opportunity denominators and no false success-rate claims.

Repeat accounting, events, poses, loss-decisions, summary and provenance byte
for byte. Any figure uses the entire fixed raw-zero-loss population in both
arms, and shows both certified-clear count and residual truth conflicts as
the radius changes, alongside coverage and the three-way partition. Preserve
all numerical source rows and reproducible plot provenance.

No rollout, estimator, steering policy, margin, timer, physical safety claim
or release improvement is introduced. These outcome-selected, repeated logs
are descriptive. Keep inherited physics/observation validation and the actual
held-goal result 140->140 unchanged.
