# Coverage components and persistence within audited first-chord chains

Start from main 398cf49. Preserve the prior 648-case accounting, 368 timeouts,
2,583 opportunities, 6,610 distinct audited states, 19,830 estimated poses,
6,187 source pairs and fixed 550/509 raw-zero/command-only loss populations.
Keep both recorded arms, all cohorts, event kinds and source censoring. Pin
speed-probe-envelopes' six inputs and speed-probe-starts' pair table, together
with inherited simulator/map/helper hashes. Tracked artifacts suffice.

Reuse all six frozen position radii (0,.05,.10,.15,.20,.30 m), both heading
bounds (0 and 4 degrees), original first-chord certificates and same-margin
truth. No threshold tuning, new geometry, new trajectory, changed controller,
full speed-cap calculation or calibrated confidence claim is introduced.

Partition every pose/setting exclusively into covered, position_only_outside,
heading_only_outside or both_outside. Coverage uses the prior closed disk and
closed heading interval without a tolerance. Cross-tabulate these components
for all states, all 550 loss transitions and all 509 command-only transitions,
including certified-clear/truth-block and certified-blocked/truth-clear
conflicts. Covered definite conflicts must remain zero. Reproduce every
original summary denominator, certificate count and coverage marginal.

For persistence, form physical decision chains from the union of the 6,187
audited source pairs, deduplicating repeated pre/post state edges belonging to
overlapping opportunities. Each edge must join the same recorded case and
consecutive .02 s decision times. Each node must have at most one incoming
and outgoing physical edge. Retain every audited state, including isolated
nodes. Do not infer an edge just because two selected timestamps happen to
be consecutive; only explicit source edges connect. Keep all original
transition/event references on deduplicated edges and source event IDs on
nodes. Do not merge across cases, missing audit edges or time gaps.

At each of the 12 settings and each input, partition every chain twice into
maximal constant runs: family coverage uses the four exclusive categories;
family clear_conflict uses conflict/no_conflict for the existing definite
clear certificate versus same-margin truth. Each family's spans cover all
audited states once per setting/input. An internal boundary is a label
change; a chain boundary is audit_start/audit_end, not observed onset or
resolution. A singleton has zero elapsed support. Report elapsed support as
(last_time-first_time)=(states-1)*.02, without adding an extra future interval.
Record left/right audit truncation and source event unions. These spans are
descriptive selected records, not independent trials or a full-error episode
census. Do not claim uninterrupted persistence across unobserved boundaries.

Join all 19,800 prior loss decisions to their coverage and clear-conflict span
IDs, chain IDs and chain positions. Keep the source 550 and 509 denominators
separate for each input. A conflict span can contain states before or after
a loss; its elapsed support describes that audited chain, not a changed
controller's release delay. Retain no_conflict spans as the full comparison
population. Do not form artificial consecutive runs of loss-only records.

Export accounting, events, chains, chain members with incoming edge references,
spans, loss memberships, coverage summary, span summary and provenance. Retain
explicit empty cohort/arm/event-kind/input/setting groups. Event-kind unions
can overlap on physical chains/spans, so those groups must not be summed.
Summaries retain unique state, recorded-case and opportunity denominators.
For durations report span counts, state counts, median/max elapsed support
and audit-truncation counts; no confidence intervals or success rates.

Repeat all eight CSVs and provenance byte for byte. Any figure must show both
arms, all three inputs and the full fixed 550-loss coverage partition at the
previously reported .10 m / +/-4 degree setting. Include a separate view of
all audited clear-conflict spans at that setting, including singletons and
boundary-truncated spans. Preserve numeric data and plot provenance.

Original observation/physics checks remain inherited and recorded held goals
remain 140->140. No new collision, release or physical safety result follows
from an envelope, a coverage category or an elapsed-support summary.
