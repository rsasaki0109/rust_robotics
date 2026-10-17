# Frozen class-pair support attrition census

Freeze from main 5151f46545a17ce3c8f0a26936586ec77eb85ace before computing new
support-attrition outcomes. Explain the loss-mode study's missing comparisons
with score-independent case/event/class/time accounting. Do not compute new
AUCs, fit, reverse scores, choose cutoffs, run simulations or change controllers.

Keep all 972 planned full cases, 324 families, 3,997 census events, 10,995 RP
states in 157 represented cases and the original 32,985 source-view references
in pinned preceding inputs. Retain all cohorts, noises/arms, five estimator
strata, both targets, E/P/U and source/common_family populations. Original
loss modes none/random/blackout stay separate, with an additional all_modes
summary that sums them. Preserve all seeds, scenarios and planners. Absence
from the selected RP diagnostics is not evidence of an unrun experiment,
missing sensor data, zero error or physical failure.

Use pinned case-strata and event-time-support counts for the collector. Each
state retains original full-RP-parent 1/n_case weight before any stratum or
population selection. Common-family eligibility is availability-only and fixed
before class/time selection; do not recompute it. Positive means outside the
closed .10 m disk, using the original observed E, modeled P (noise+anchor) or
unmodeled U (drift+transport) labels. No score precision choice changes support.

For every planned case/stratum/target/label/population save one case-support row
(58,320 total). Keep the full planned case_id even without records. Retain
parent and pre-population stratum record counts, eligibility, selected records/
events, outside/inside counts and original case mass. Case-level opportunity
count is n_outside*n_inside; mass is that count/n_parent**2. It includes
different event tags solely as an availability denominator, not as new ranking
evidence. Within-event counts sum n_outside_event*n_inside_event without ever
crossing full cases. Split case opportunities exactly into same_event and
other_event_same_case counts/masses.

For same_event and existing integer-microsecond <=500/100/20 ms scopes save
pair count/mass and supported-event counts. Counts are cumulative and nested;
their differences are the original time bins. All zero denominators give blank
shares, not artificial zeros or chance AUCs. No cross-event/case/mode/planner
pair becomes an admissible same-event comparison.

For each case and scope assign exactly one reason, in this fixed precedence:
no_parent_rp, no_stratum_rp, family_ineligible, no_outside, no_inside,
event_separated, gap_excluded, supported. The first two inspect original parent
and pre-population stratum availability; family_ineligible applies only to common
population; class reasons inspect selected states. event_separated means a case
has both classes but no event has both. gap_excluded means same-event class
pairs exist but none survives that scope's time bound. A supported case may
still lose many pairs: reason counts and pair-retention masses answer distinct
questions. The precedence is a deterministic audit convention, not a causal
explanation of why the simulator produced a selection.

Save event-support for all 275 RP-referenced events x five strata x both targets
x E/P/U x two populations (16,500 rows), including empty strata and ineligible
selections. Use the same reason precedence where applicable, with one original
case/event per row. Save label-blind event-coverage for all 3,997 census events,
including the 3,722 not referenced by this RP selection; preserve original event
kind/received/scheduled/precision flags, full case identity and per-stratum RP
counts. Do not filter event kinds or fabricate diagnostic states.

Save 2,880 support-summary rows across the fixed aggregate grid with all_modes
and all three original modes. Include planned/represented/selected cases,
records, events, class counts, original case mass, case opportunities, same-event
and time-bounded counts/masses, supported cases/events, exhaustive reason counts
and weighted retention shares event/case and each time scope/case and /event.
Sum masses before division; never average case shares. Mode sums must reproduce
all_modes. Every reason-count sum equals planned cases for each scope. Preserve
all 81 planned leaves per cohort/noise/arm before any selection.

Check every mode's original availability, pair count/mass and supported case/
event counts against preceding mode-effects for every scope/target/label/score/
stratum/population. Check all_modes against preceding precision-summary. Keep
all preceding files pinned in place. Do not repeat unrelated ranking metrics.

Freeze four figures. Three cohort-specific residual-target pair-retention
matrices each retain all four noise/arm panels, Filter/Fusion, E/P, both
populations and all three modes. Show case -> event -> <=20 ms raw counts and
weighted event/case and <=20 ms/case shares; undefined opportunity denominators
stay NA. The fourth shows latest residual E case-reason stacked counts at
<=20 ms, both populations, all modes, Filter/Fusion and four noise/arm conditions;
each row accounts for all 27 planned cases. One numeric export keeps all 288
focused full summary rows used by these figures. Track four SVGs, CSV and plot
provenance; PNGs remain reproducible scratch outputs.

Test unequal original parent weights, no renormalization, all reason stages and
precedence, fixed family eligibility, empty/no-loss selections, single-class
events in two-class cases, pair-free cases, time nesting/boundaries, exact
partition closure, rejected negative/inconsistent/duplicate/unknown identifiers,
source nonmutation and aggregate mode closure. Run the full Python suite.
Repeat all collector and plot artifacts byte for byte and inspect all figures.

A standalone verifier importing no analyzers independently reconstructs labels
from the original signed pose/projection components and full-case event joins.
Use direct NumPy positive/negative pair masks with original Fraction parent
weights and original microsecond rounding/tolerance to check every case/event/
coverage/summary field, all reason counts, preceding support closure, plotted
rows and input/helper/script/protocol/output/plot hashes. This verifier checks
support only; it need not recompute unrelated preceding AUCs or score order.

These are already observed diagnostic subsets. Label/event/time attrition
does not establish causality, independent physical trials, confidence intervals,
held-out prediction or calibration. Event tags are not complete Fusion ancestry.
Historical held goals stay 140->140 and latest 62->60; no new live outcomes.
Save report, README and evidence, then merge under the standing instruction.
