# Frozen logged preview-transition audit

Freeze from main 3887f64164454f34a6da3ff8dded2a8ec7cb54e6 before reading new
raw-window outcomes. Follow the preceding census/RP join's immediate-preview
disagreements back to original logged availability. No new simulation, controller
choice, error labels, AUC, fitting, intervention or causal attribution is made.

Pin the preceding census-rp-selection sources and its complete input/helper
chain. Pin all 108 original trajectory CSVs by the fresh-seed-cutoff-transfer
raw manifest's SHA-256 values. Stream the originals; relocated roots may be
supplied without changing hashes. Keep all 972 planned cases, 324 families,
570 timeout histories, 3,997 census events and 10,995 original RP records in
157 cases. Preserve original full case identity, loss mode, event/source IDs,
all five event kinds, three cohorts, both noise types and both arms. Held goals
remain historical 140->140 and latest 62->60. No new case or time selection.

For every census event, inspect the original inclusive decision-grid window
from event step -1 through min(event step +75, 9000), at 20 ms per step. These
are the original RP candidate windows, including their predecessor endpoints.
Validate a complete unique 0..9000 grid for each timeout history and the full
scenario/planner membership in each raw file. The pre-event sample is distinct
from the immediate sample. A change after the event compares steps +1..+75
to their predecessors; do not count a change from step -1 to step 0 as a
post-event change. Time is logged time; do not claim an unobserved continuous
transition instant or confuse it with truth_time_s = logged time -20 ms.

Read actual mode as recovery_mean when recovery_sample_count >0, otherwise
raw_fallback. Read preview mode as three_epoch when epoch_fusion_available is
true, otherwise motion_filter. Require lowercase raw booleans, nonnegative
sample counts, retained epoch group count 0..3 and three groups when Fusion
is available. Retained group count alone does not define the selected preview.
Check immediate raw flags/counts/age against the pinned census and same-time
raw actual/preview against every original RP stratum. Do not replay gate
eligibility or infer why an unselected candidate was absent.

Save maximal mode-intervals.csv runs of the pair (actual mode, preview mode)
for all windows. Inclusive boundaries and checkpoint counts must reconstruct
every original window without gaps, overlaps or adjacent equal runs. This is
lossless for these two mode identifiers, not for raw numerical trajectories.
Save event-trace.csv for every event: full identity, fixed window, immediate
modes, raw/received parent weights, interval and checkpoint counts, post-event
actual/preview change counts and first change offsets, and first changed
preview's measured speed, observation age and retained group count. These
last fields describe the logged boundary; they are not a causal decomposition.
Include pre-event modes, RP counts/masses, same/different preview counts,
before/at/after-event disagreement counts, first after-event differing RP
offset and its lag from the first post-event preview change. Do not interpolate.

Classify observation horizon explicitly: full (75 forward steps), clipped
(1..74), no_forward (0). A window with no observed change does not establish
stability beyond its observed endpoint. Report all events, and separate full,
clipped and no-forward cells; no-forward events are not full-horizon failures
or successes. Overlapping windows are event references, not independent
physical samples. No time-exposure weights or independence claims are added.

Save rp-state-audit.csv for all 10,995 original states, with original case/event/
source identity, time/offset/stratum, unchanged 1/n_RP_parent_case weight,
same-time raw modes, covering run ordinal and before/at/after-event relation.
For each post-event different-preview record require a logged post-event
preview change at or before that record. This verifies a logged sequence and
same-time agreement, not a causal explanation or complete Fusion ancestry.

Save transition-summary.csv with 1,344 fixed cells: 12 cohort/noise/arm groups
x three loss modes plus all_modes x seven event groups (all, received_any and
five exact kinds) x all/full/clipped/no_forward horizon selectors. Keep empty
cells and all planned case counts. Summarize checkpoint/run counts, immediate
modes, first-change event counts, original all-census and received-census
masses, original RP masses/counts and preview disagreement timing. Fusion-loss
fractions condition on immediate three_epoch within that cell. Raw fractions
use event counts; weighted fractions sum original all-census 1/n_case_events
masses before division. Preserve each original RP 1/n_parent state weight;
never renormalize after event/mode/horizon selection or equate census and RP
measures. Empty denominator is blank; an observed zero numerator is zero.
Require exact additive mode, kind and horizon closure for counts and masses.

Freeze three figures: (1) all-event horizon counts for all 36 cohort/noise/arm/
loss conditions, (2) full-horizon immediate-Fusion loss raw and original-case-
weighted fractions for those same conditions, with counts and NA distinct
from zero, (3) every event with a post-event differing RP preview, plotting
first logged post-event preview change against first differing RP offset,
including empty cohort panels. Export the full 144 all-kind horizon summary
rows and all fixed-criterion scatter event rows, with plot hashes. Track SVGs;
PNGs stay scratch. Inspect every final figure.

Test pre-event versus post-event changes, 180 s clipping/no-forward windows,
exact 20 ms boundaries, malformed grids/flags/counts, maximal runs, reversals,
full-case/event/source isolation, stratum agreement, historical retained group
count versus availability, unequal original parent weights, empty cells and
sum-before-division. Repeat collector and plot artifacts byte for byte. Run
the complete Python suite.

A standalone stdlib verifier imports no analyzers. Independently stream and
hash all raw files with csv.reader, reconstruct original broad-census IDs and
historical membership/latest score RP references, and use SQLite window
functions over uncompressed checkpoints to reconstruct maximal runs and
transition positions. Compare every saved field, summary, numeric plot value
and provenance hash, including preceding counts/masses. Do not use saved runs
or the collector's source-event-map as the verifier's reconstruction input.
Save report/README/evidence and merge this bounded study under the standing
instruction.
