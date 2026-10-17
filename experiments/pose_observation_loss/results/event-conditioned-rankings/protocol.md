# Saved source-event comparison protocol

Frozen before computing event-conditioned ranking outcomes, from main
429ef9de764ddc59ecdc390ea766c4cbb7afbae4. A label-blind structural audit found
that all 10,995 saved Recovery/Preview states have exactly one source_event_ids
entry, covering 275 received events and 550 nonempty event/stratum groups when
including all. These facts determine the scope, not outcome-dependent choices.

Reuse 32,985 reconstructed pair references, 157 represented cases, all 972
planned cases, 324 planner families and the full 3,997-event census. Keep known
269/270/271, previous additional 274/275/276 and latest additional 277/278/279
separate; both noises/arms, all five strata, both targets, all three scores and
E/P/U labels. Use mixed planners and the unchanged source/common_family
populations. Original weights remain 1/n_case before mode/population selection.
No event normalization, new simulation, fitting, cutoffs or score reversal.

The grouping key is (full case identity, saved census event_id). Check every
state/event full case join and reporting group, including the census partition
alias existing_new -> new. Keep original partition/source_event_id metadata.
Do not group by event_time_s, truth_time_s, underlying source_event_id or event
semantics. Equal timestamps do not merge distinct events or cases. Reject empty,
unknown or multiple source-event tags rather than silently choosing a primary
event or duplicating a state's weight. Retain event_kind and original flags;
do not select received-only data as a new analysis filter.

This ID is the diagnostic reference tag, not the complete ancestry of samples
used by Fusion, a proof of new independent information, or a physical noise
stream identifier. Event-conditioned comparisons may still mix times and
estimator history. Do not infer causal event effects or calibration.

Partition every ordered positive-state/negative-state comparison into exactly
three disjoint bins:

1. same_event: same full case and same saved event_id;
2. other_event_same_case: same full case but different event IDs;
3. other_case: different full case keys.

Compute exact rational concordant/tied/discordant masses, numerator, denominator,
AUC and full-denominator contributions for each bin. Also expose the first two
bins' shares of same-case pair mass. Sum numerators and denominators before
division; do not average individual event AUCs or reweight events equally.
Assert the first two bins reproduce every preceding same-case pair mass and
all three reproduce full pair mass. Preserve and check every preceding aggregate
ranking field, including within-case/window/family/seed and planner intersections.
Save new within-event and other-event/same-case metrics and supported event/case
counts. High score=outside, ties half; norm > .10 m outside, closed inside disk.
Zero-pair/absent-class AUC remains blank, including all empty strata and seeds.

Save one source-state/event mapping row per RP state, with full case IDs, original
source ID, times, tag metadata and inherited row weight. Audit all 3,997 events,
including no-reference events, with RP counts, state-time/age ranges and parent
case weights. No reference is an absence from the saved diagnostic selection,
not evidence that a received observation was missing. For all 275 represented
events save all five stratum-support rows and both targets' E/P/U class masses.
Save event rankings for all 550 nonempty event/stratum groups, both targets,
three scores and E/P/U (9,900 rows); empty groups remain explicit in support.
One source event ranking serves both populations via unchanged stratum-specific
common-family eligibility. Preserve full original accounting/support/events/
cutoffs/loss references, case/family/stratum eligibility and planned case audits.

Freeze five figures to residual_sigma and Filter/Fusion: each cohort's residual
observed-E full/within-case/within-event/other-event-same-case/between-case AUCs
for both populations; latest modeled-P equivalent; and all cohorts' E/P shares
of same-case pair mass from same versus different source events. Show NA and
supported counts; numeric exports retain both targets and both full/conditional
shares. No outcome-based selection of cases, events, time spans or panels.

Pin protocol, inputs, helpers and outputs. Add meaningful event-key/join,
ambiguous-tag, partition-alias, inherited-weight, exact-tie, missing-class,
mass-closure and baseline regressions. Repeat aggregation/plotting byte for byte,
inspect all figures, run the full Python suite, and save an independent NumPy
state-pair verifier importing no analyzers, CSV/JSON/SVG, report and README.
Merge this completed bounded study under the standing instruction.

These cohorts are already observed; conditioning may leave little class support.
Repeated states, events, windows and planners are not independent sample sizes,
confidence intervals, a repair to the estimator or real-world validation. Live
outcomes remain historical 140->140 and latest 62->60.
