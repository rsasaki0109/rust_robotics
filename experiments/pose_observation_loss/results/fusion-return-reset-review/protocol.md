# Fixed review and replay of the six reset-evidence studies

Consolidate existing results from main eac4b2a1adfcc63f0fe56a27a04ac12b666250fd.
This is a review/export and reproducibility task using known outcomes, not a
new blinded experiment. Freeze the six-stage catalogue, complete artifact/
script hashes and overview schema before assembling the review packet.

Replay these existing stages in order: run_breaks, reset_probes,
reset_displacement, reset_projection, reset_margin, reset_surplus. Each
collector reads its original committed preceding inputs, as designed; these
are six independent replays against pinned inputs, not a rewired pipeline
feeding newly generated parent outputs into later stages.

The runner must use the existing checkout, a fresh output directory outside
the repository and the current Python executable without optimization. Use
argument vectors, preserve the runtime environment, retain child stdout/stderr
in per-stage logs and stop on the original child failure status. Never
overwrite existing output directories or publish a completed review manifest
after a failed/incomplete stage.

Before execution verify all frozen stage artifacts and the three scripts per
stage. For each stage execute its collector, plotter and standalone independent
verifier. Require byte identity with the pinned committed collector tables/
sources, SVG/CSV/plot manifest and independent proof. Require exactly the
expected files, including the freshly generated PNG; PNG has no committed
baseline, so retain its hash and compare it between the two review replays.
Existing verifiers must recheck their complete transitive input/helper/Rust
provenance and original census joins. Do not substitute hash checks for the
fresh standalone verifier executions.

Repeat the full six-stage runner twice. Compare all generated stage collector,
plot and proof files, and the three new overview files bytewise. Logs remain
scratch evidence. Repeated arithmetic re-evaluates 19 displacement, 19
projection, 19 margin and 17 ratio pairs per run; it adds no new source
observations, raw extraction or native dataset execution. Original probe/native
geometry and prior native dataset execution remain inherited evidence; the
existing full Python suite separately runs its native unit fixtures.

Retain all 19 original reset/blocker rows in case/step order, 17 initial-unsafe
and two first-chord origins, and all 185 overlapping-ending multiplicities.
Preserve the original 25 contexts, 60 missing future slots, 972 cases, 324
families, 570 timeouts, 3,997 anchors, 137 returns, 3,751 decisions, 1,096 zero
probes, 336 cells, preselection weights and original outcome labels. Add no
times, objects, selected weights, independent trials or recovery rates.

Create reset-evidence.csv by copying these 39 fields verbatim from the replayed
reset-surplus/surplus.csv, without rounding, numeric conversion or new derived
metrics: edge_id, case_id, decision_event_id, source_step, source_time_s,
decision_probe_origin, stage, witness_obstacle_id, witness_map_x, witness_map_y,
multiplicity, failed_gates, unknown_gates, zero_reason, nominal_positive,
decision_nominal_target_speed, decision_speed_free_distance, decision_speed_cap,
lost_ticks, timer_before_ticks, timer_after_ticks, decision_timer_action,
decision_ready_from_timer, witness_raw_current_state,
witness_raw_current_gap_lo, witness_raw_current_gap_hi,
witness_mean_current_state, witness_mean_current_gap_lo,
witness_mean_current_gap_hi, radial_gap_state, radial_gap_lo, radial_gap_hi,
distance_gain_lo, distance_gain_hi, surplus_status, required_fraction_lo,
required_fraction_hi, surplus_fraction_lo, surplus_fraction_hi.
Keep nonapplicable ratio blanks and every simultaneous failed/unknown gate.
Do not collapse the two curved-chord failures into point failures.

Create study-index.csv with six rows containing the study name, collector/
verifier/plotter script names, report path, three source/proof/plot hashes,
replayed collector/plot file counts, independent/original identity comparison
counts and the full original source audit JSON. Preserve the audit's original
keys/units; do not normalize, add or reinterpret counts across studies.

Create review-sources.json recording this runner/protocol/freeze hash, all
frozen inputs, per-stage replayed output/proof hashes (including PNG hashes),
and the two overview CSV hashes. Keep paths repository-relative and omit
timestamps, output-directory names and timing data so repeatability is
testable. This manifest records fresh replays of existing independent proofs;
it is not a newly independent numerical method.

Track only the protocol/pins, two overview CSVs, review-sources.json and a
human-readable report. Link the six existing figures/reports instead of
creating another geometric figure. Explain the recorded chain from original
probe failure through zero cap/nominal failure to timer reset, retain full
mixed failure masks, and distinguish the same-step mean-point audit from the
original curved probe and recovery policy. Do not make new controller, path
safety or counterfactual success claims.

Test exact string/blank projection, duplicate/missing overview fields, strict
file/byte checks, changed source/proof metadata, safe fresh output destinations,
argument-vector execution and preserved child failure/logging. Run the full
Python suite and both six-stage replays, save evidence/README/review, then merge
and push under standing authority. No controller or dependency changes.
