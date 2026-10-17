# Frozen logged control audit around first Fusion returns

Freeze from main 3e11620342ce2acd03df279cc8231402da6437c2 before reading new
control outcomes at or within the preceding 137 returned spells. The preceding
study already fixes 42 returned spells ended by TTL after 20 ms, 47 ended by
the age gate after 1.52 s, 48 returns at the 180 s endpoint, and 15 original
losses without a return. Keep every original census anchor, not only returns.
No rollout, controller change, retention tuning, new RP selection, error label,
AUC, treatment effect or complete gate-eligibility replay is added.

Pin return-spell-mechanisms/sources.json at SHA-256
e251852bc1bc8c1f2f1f974acaeaaf5660fb0be8b15f16eabd95b3fd3bdf914b,
its complete input/helper/source/raw chain and four output tables. Pin its
independent-verification.json at
b260b4fe2654a5026ca862b827e96cddcfc290fc85e3b09cfb1e6994fcef3be8.
The native metadata proof remains inherited evidence, not a new Rust replay
in this control audit. Both new readers independently hash and stream all
108 original raw files, checking complete unique grids, full case membership
and original event flags. Preserve 972 planned cases, 324 families, 570 timeout
histories, 3,997 census events and 10,995 original RP records in 157 cases.

Read logged safety_paused and guard_active as four disjoint latch states:
loss_and_guard, loss_only, guard_only, clear. Clear describes these two latches,
not motion or a positive command. Keep stop_latched, resumed, guard_triggered,
guard_released, speed_braking, target_speed, nominal_target_speed, steering,
nominal_steering, recovery_evidence and guard_release_evidence separately.
Read recovery_mean versus raw_fallback from positive versus zero original
recovery_sample_count; this names the recovery pose branch, not the complete
controller. Preview is three_epoch versus motion_filter from availability,
not retained group count. Keep the two source identifiers separate from
latch state, release and logged positive final target-speed commands.

At every late checkpoint (150..180 s in all 570 timeout histories), validate
the original final-command overwrite rules against logged flags: a loss pause
forces target speed and steering zero; guard alone forces target speed zero;
without either latch the target/steering equal their nominal values. A resumed
checkpoint clears both latches and uses nominal commands. Compare the printed
numeric commands within 2e-9. These are recorded command invariants, not a
controller/trajectory replay or a claim that a positive command caused motion.

For each of the 137 returned anchors inspect the inclusive raw decision-grid
range [return_step-1, first_next_loss_step] or [return_step-1, 9000] for a
censored returned spell. Save control-runs.csv as maximal runs of the tuple
(recovery pose branch, preview branch, safety_paused, guard_active, resumed,
logged final target speed positive). Inclusive boundaries/checkpoint counts
must reconstruct every reference without gaps, overlap or adjacent equal runs.
Reused physical checkpoints remain separate anchor references.

For within-spell measures use only available checkpoints [return_step,
next_loss_step-1], or [return_step,9000] for a censored spell. Keep counts of
each latch state, recovery pose branch, resumed/guard trigger/guard release,
and logged positive nominal/final commands. Save maximum logged recovery
evidence, minimum/maximum recovery sample counts and whether the exact integer
>=10 sample requirement is ever met. Counts are checkpoint references, not
time exposure or independent trials. A 180 s return has one observed available
checkpoint and zero subsequent observed span; no future end is inferred.

Save control-snapshots.csv at return_before, return, end_before and end_after
for every observed next loss, or return_before, return and censored_endpoint
for a censored returned spell. Keep full identity, original event/source IDs,
roles/times/offsets, raw event/availability/group/count/age/speed flags, latch/
release flags, commands/evidence and release clearance/margin. Retain logged
source identifiers and whether the integer sample requirement is met. Geometry
and release clearance are descriptive logged inputs; do not infer unselected
RP candidates, full gate readiness or causal reasons from rounded scalars.

Save anchor-controls.csv for all 3,997 anchors, retaining identity, original
flags/kind, first loss/return/end/censoring, original horizons/weights and RP
counts/masses. Nonreturned anchors have blank new control fields. For returns
include snapshot source/latch/release/command fields, run/reference counts,
within-spell counts/flags, and observed end-boundary pose/latch/command changes.
Censored end-change fields stay blank. The 15 no-return episodes retain their
lower bounds; the 18 original TTL-limit anchors without a forward step remain
outside observed-loss and returned-spell denominators.

Save control-summary.csv with 336 fixed cells: 12 cohort/noise/arm groups x
three original loss modes plus all_modes x all and six preceding disjoint
anchor categories (ttl_ended, age_ended, returned_censored, no_return,
unobserved_ttl_limit, no_observed_loss). These categories are fixed from the
previously pinned results, not chosen from new control outcomes. Keep empty
cells and planned/timeout/held case counts. Include original census/RP counts
and masses, returned counts, each return latch/pose state, any within-spell
resume/positive command/sample-ready events and observed boundary changes.
Add checkpoint-reference counts separately. Raw and original all-census-mass
shares of any resume/positive command condition on returned anchors. Empty
denominators stay blank. Preserve original all/received census weights and RP
parent weights before selection; sum exact Fraction masses before division.
Check additive mode/category closures; distinct case counts are not additive.

Freeze one figure of the four return latch states for all 36 original
cohort/noise/arm/mode conditions, keeping empty conditions. Export all 252
original-mode category summary rows and all 137 returned anchor rows with
plot hashes. Track SVG/CSV/provenance; PNG stays scratch. Inspect the figure
and repeat all collector/plot artifacts byte for byte.

The standalone verifier imports no analyzers or new collector/plot code. It
may reuse only pinned preceding standalone original-census/hash helpers. Read
the raw CSV with a separate parser, reconstruct source/latch/command fields
and all fixed window references, and use SQLite window functions over
uncompressed checkpoints to reconstruct maximal control runs. Reconstruct all
saved anchors, snapshots, exact weights, summaries and numeric plot values
independently; do not use saved runs or snapshots as reconstruction input.
Rejoin the original census and compare raw same-time flags/counts. Record raw,
source, helper, protocol, script, output, plot and inherited native-proof hashes.

Test distinct pose/preview/control identifiers, all latch combinations,
overwrite invariants, positive nominal versus final command, resumed pulses,
inclusive/strict spell boundaries, a 180 s return and endpoint censoring,
absent returns, repeated references, full-case isolation, duplicate/missing
grids, strict flags, fixed unequal parent measures, empty/zero denominators,
additive closure and nonmutation. Run the full Python suite. Save report and
README commands, then merge/push this bounded audit under standing authority.
