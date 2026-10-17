# Frozen late-census to RP-selection coverage

Freeze from main 8247c0db2c7328ba16636214d8fe6a3e42f9b656 before computing new
coverage outcomes. Connect all planned cases and broad late-timeout census
events to the already selected RP state/event references. Quantify coverage,
not new AUCs, errors, fitted thresholds, interventions or live outcomes.

Keep 972 planned full cases, 324 families, 570 timeout histories, 3,997 census
events, 10,995 RP states in 157 cases, 275 RP-referenced events and 32,985
source-view references in pinned preceding accounting. Keep all three cohorts,
white/correlated noise, epoch/curved arms, none/random/blackout modes, three
planned seeds per cohort, dense/maze/open scenarios and A*/JPS/Theta* planners.
Loss mode remains part of full case identity. Add all_modes summaries by sums,
never by constructing cross-case state pairs. Do not choose new cases or times.

The census includes every correction/precision opportunity at logged 150--180 s
in timeout histories, once immediately after update. Held cases have no such
late-timeout census by design; absence is not an unrun experiment or missing
sensor data. Preserve original status/held/collision/out-of-bounds accounting.
The RP audit previously selected both endpoints of adjacent decisions when at
least one was joint and active_ready, in offset -1..+75 intervals clipped at
180 s. This study joins saved artifacts; it does not replay or change that rule
or infer which individual gate prevented an absent RP selection.

Save case-selection.csv for all 972 planned cases, retaining stable planned
case_id/family_id and original outcome fields. Add all/received census counts,
immediate actual/preview counts, RP references, four RP estimator-stratum record
counts and original RP case mass. Classify each case exactly once as held,
timeout_no_rp or timeout_with_rp. These are recorded selection stages, not
independent successes or new causal outcome categories.

Save event-selection.csv for all 3,997 original events, preserving source event
IDs, original and reporting partition, full case identity, kind, time/truth time,
scheduled/received/precision flags and immediate actual/preview metadata.
Retain all five planned event kinds: received, missed, precision_only,
received_and_precision, missed_and_precision. The immediate actual identifiers
are raw_fallback/recovery_mean; preview identifiers motion_filter/three_epoch.
Unknown, duplicate, ambiguous or wrong-case identifiers fail. Validate original
flag/kind semantics, seven scheduled timeout times, and no census in held cases.

Join the original source-event-map by full case and numeric census event ID.
Every original RP source_id has one event and one stratum. Keep all four planned
strata: recovery_single, filter_single, fusion_three, recovery_fallback_single,
including empty categories. Retain RP time offsets with the original inclusive
[-20, 1500] ms bounds and original microsecond rounding/tolerance. Saved event
tags are diagnostic references, not complete Fusion ancestry or independent
physical streams. Do not equate immediate event preview with window-state
preview. Keep same/different/other preview record counts and stratum masses;
different means different identifiers, not a demonstrated temporal transition.

Keep three distinct measures. Each census event gets 1/n_all_census_events_case
before kind/actual/preview filtering. Each received event also gets
1/n_received_census_events_case before those filters; nonreceived contributions
to this second measure are zero. Each RP state retains its original
1/n_RP_parent_case before any event/stratum selection. Event RP mass is the sum
of those state weights. Never equate, subtract or average these differently
defined measures. Use exact Fraction sums before division; no post-selection
renormalization. An empty denominator yields a blank share, while an observed
denominator with zero selected events yields a defined zero coverage.

Save selection-summary.csv with exactly 3,024 cells: 12 cohort/noise/arm groups
x three loss modes plus all_modes x seven event groups (all, received_any and
all five kinds) x three actual selectors (all plus two identifiers) x three
preview selectors (all plus two identifiers). Keep every empty cell. Save
planned/timeout/held/with-RP/no-RP case counts, selected census cases/events,
referenced/unreferenced events and cases, raw coverage, the two census measures
and their referenced coverage fractions, RP records/mass by stratum and
same/different/other preview counts. RP stratum mass shares use the selected
cell's total original RP mass and remain blank without that mass.

Mode sums must reproduce all_modes; exact kind sums reproduce all; disjoint
actual and preview selectors reproduce their all cells. received_any is defined
by the received flag, not a guessed event-kind subset. Census and RP measures
close to their original parent totals before filtering. Check all case and
event record counts/masses against preceding support-attrition coverage and
source case/stratum audits. Pin preceding files rather than copying them.

Freeze three figures, each with all three cohort panels and all noise/arm/loss
conditions, including none. First: exhaustive held/timeout_no_rp/timeout_with_rp
counts per 27 planned cases. Second: received_any census reference coverage for
all four immediate actual/preview combinations, showing raw referenced/total
events and original received-case-weighted coverage with NA distinct from 0%.
Third: immediate-preview to four RP-stratum crosswalk, all event kinds/actual
inputs, showing raw state counts and original RP-mass fractions. Keep 252 full
summary rows in one numeric export for the union of these fixed panels. Track
three SVGs, CSV and plot provenance; PNGs remain scratch outputs.

Test original unequal census/RP parent weights, fixed pre-filter received
weights, summing before shares, mode/kind/actual/preview closure, empty modes,
held/timeouts, flag semantics, original partition aliases, full-case/mode/event
isolation, rejected duplicate/unknown/wrong-case references, preserved statuses,
time-boundary rounding, different immediate/window identifiers and nonmutation.
Run the full Python suite, repeat collector and figure artifacts byte for byte,
and inspect every final figure.

A standalone verifier importing no analyzers joins original broad-census
source IDs and original historical membership/latest score RP metadata, without
trusting the collector's saved source-event-map. Independently reconstruct all
case/event fields and use SQL selection/count queries plus exact Fraction parent
weights for all summary cells. Check every new field, preceding coverage/
case-stratum closure, all 252 plotted rows and input/helper/script/protocol/
output/plot hashes. Do not rerun unrelated raw-vector labels or AUC studies.

These selected retrospective diagnostics do not represent the full live data
distribution, calibration, causal mode benefits, independent trials, confidence
intervals or held-out prediction. No controller selection is made. Historical
held goals stay 140->140 and latest 62->60. Save report, README and evidence,
then merge this bounded study under the standing instruction.
