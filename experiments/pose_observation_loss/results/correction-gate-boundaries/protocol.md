# Adjacent gate boundaries, command zeros and recorded position changes

Start from main 7b3a052. Preserve correction-gate-sequences' complete 648-case
accounting, all 368 timeouts, all 2,583 opportunities and all event/interval IDs.
Use every exported pre/immediate/post decision, including moving, inactive,
independent, missed, precision and right-censored opportunities. Never replace
the missing future of an event at 180 s with observed no-action decisions.

For each opportunity compare adjacent decisions at offsets (-1,0), (0,1), ...,
(74,75), through the recorded end. Separate the immediate pair ending at 0
from post-event pairs ending at 1..75. Assign mutually exclusive labels:
joint_retained, joint_lost, joint_acquired, joint_not_ready, joint_route_exit,
joint_route_entry and other. The first four require both routes joint; an exit
after a successful resume is not a lost joint condition. Retain all pairs,
including overlapping opportunities, and keep event/case/pair denominators.

For every distinct saved decision, match the prior gate fields and reconstruct
the raw sensed pose from preceding physical truth plus logged noise and bias.
Retain logged recovery and preview poses, matching each to pre-integration
truth rather than to the integrated position in its own row. Save coordinates,
noise/bias, signed gate gaps, separate recovery-obstacle/wall gaps and the
actual versus nominal command. Do not replay the observation estimator anew.

Reproduce the speed cap clamp(sqrt(.0225+free_distance)-.15,0,1) and the nominal
command min(uncapped_target_speed,speed_cap), within 2e-8 of logged values.
Classify logged nominal zeros as uncapped_zero, speed_cap_zero, both_zero,
or rounded_zero_other; retain positive commands. A paused actual target of 0
does not imply a zero nominal command. Preserve stop-latch flags for context.
Check each active requirement against its signed gap, preserving strict >0
for recovery obstacle, stop and terminal, inclusive >=0 for recovery wall,
stationarity, freshness and samples, and strict >0 for the nominal command.

For every adjacent pair where either side is joint and fully ready, export
position/heading/error changes for raw, recovery and preview inputs, true
motion, noise/bias changes, gate-gap and command changes. This includes every
loss, acquisition and retained interval, not just interruptions or improvements.
Verify delta-estimate minus delta-truth equals delta-error, and raw error
change equals logged noise plus bias change. Keep the observed vectors;
an estimated-position jump is not itself an error improvement.

Independently recompute speed-probe free distance, recovery-line obstacle
clearance, stopping preview and terminal geometry for the union of both sides
of all these pairs, once per distinct recorded decision. Use the full map and
pinned helpers: exact_speed_distance, segment_distance and preview_components.
No steering search or true-pose counterfactual. Check speed/recovery/terminal
values within 2e-8; stop lower bound must not exceed exact clearance beyond
that tolerance. Check stop classification at the actual release margin where
both values are farther than 2e-8 from it; report boundary cases separately.

Describe losses with their complete failed-requirement mask, logged zero-command
reason and geometric failure depth. Use all failed geometric facets (recovery
obstacle/wall, stop, terminal): no_geometric_failure, within_1cm when every
failed gap >=-.01 m, or beyond_1cm otherwise. Nominal speed has different units
and is not combined into a distance. Marginal failures overlap; exclusive masks
partition the observed losses. No sole causal blocker is assigned.

Export accounting, events, states, all transitions, detailed changes, geometry
audits, complete transition summaries, loss-pattern summaries and matched-pair
change summaries. Preserve empty cohort/arm/event-kind groups. Repeat nine CSVs
and provenance byte for byte. If plotted, retain both arms and every observed
opportunity category, label the post-event population, and keep all joint losses
in the denominator including those with no geometric failure.

All quantities are descriptive associations in repeated records selected on
eventual timeout. Logged command zeros and clearance violations are not new
physical collisions or causal explanations of final holds. Inherit previous
observation/physics validation; add only this recorded-input/geometry audit.
Do not change inputs, controller, margins, timers or held-goal result 140->140.
