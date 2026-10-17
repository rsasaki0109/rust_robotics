# Frozen epoch retention mechanism audit

Freeze from main a795bd457a02409859a370aac14cf8165b71fa98 before reading new
raw-history group identities. Explain recorded availability transitions using
the existing algorithm, without changing the controller, retention horizon,
simulation, labels, AUCs, thresholds or live outcome accounting.

Pin logged-preview-transitions/sources.json and its input/helper/raw chain,
all 108 raw CSV hashes, the original Rust simulator and VehicleState source.
Preserve all 972 planned cases, 324 families, 570 timeout histories, 3,997
census events and 10,995 RP states in 157 cases. Keep full case identity
including loss mode and original event/source IDs, kinds, horizons, parent
weights and prior RP coverage. Historical held goals remain 140->140 and
latest 62->60. Do not select new cases or windows.

Replay only epoch metadata for every timeout decision step 1..9000 from the
original raw speed, observation age, received and precision flags. Validate
unique complete 0..9000 grids and all planned raw trajectory membership.
Step 0 initializes history and does not call observe. At every later step,
check retained group count and availability against raw logs. Retention uses
strict age >750 steps (15 s inclusive), stationary speed <=0.02, fresh sample
age 0..0.5 s inclusive, at most three groups and 25 samples per group, at least
ten samples in each of the two older groups, current epoch matching latest,
and availability age <=1.5 s inclusive. Received or precision establishes an
epoch; a missed correction alone does not. Motion or precision clears first,
then expired groups are removed, then fresh current-epoch samples are added
and capacity enforced. Pose values do not enter these availability predicates.

Save event-mechanisms.csv for every original event. Keep original full identity,
event flags, immediate modes, fixed horizon, original all/received census
weights and RP counts/masses. Add immediate oldest epoch age/remaining TTL
and whether available Fusion sits exactly at the inclusive TTL limit. For
each previously frozen post-event preview loss, save its offset and the
ordered operation category: motion_and_precision_clear, motion_clear,
precision_clear, ttl_expiry, capacity_eviction, age_gate, latest_epoch_gate,
older_samples_gate, other_gate. Save independent operation flags/counts as
well; this ordering is a deterministic audit convention and is not a causal
decomposition of controller outcomes. No-loss events use no_observed_loss.

Save snapshots.csv for every immediate event, plus loss_before/loss_after for
every previously logged preview loss. Reused physical checkpoints retain
separate event/role references. Each snapshot includes current epoch, speed,
age, raw availability/group count, retained epoch/sample-count lists and
operation evidence (cleared, expired and capacity-evicted epoch lists).
Save group-history.csv for every retained group in every snapshot, oldest
first, with original logged epoch time, capped sample count, age and TTL.
Empty histories have zero group rows, not invented placeholder groups.
Compare every loss's predecessor and changed checkpoint to preceding logs;
do not count pre-event-to-immediate changes as later losses.

Classify one-step opportunities only when a forward sample exists. A final
180 s event has no observed next step. Report immediate available Fusion at
TTL limit separately from other available Fusion, and cross-tabulate their
observed +20 ms losses without fitting a prediction rule or generalizing to
held-out data. Exact original 1/n_case_census_events and received-census
weights precede filtering. Original RP 1/n_parent_case weights stay unchanged;
never equate these measures or renormalize after operation or horizon selection.

Save retention-summary.csv with 1,344 cells: 12 cohort/noise/arm groups x
none/random/blackout plus all_modes x all/received_any/five exact event kinds
x all/full/clipped/no_forward horizons. Keep every empty operation and planned
cell. Save event counts and original masses by loss operation, immediate
Fusion and TTL-limit counts/masses, eligible one-step denominators and their
loss counts/masses, original RP totals and disagreements. Sum exact Fraction
masses before division; absent denominators stay blank. Check additive mode,
kind and horizon closures without adding overlapping case counts.

Freeze one figure with all 36 original cohort/noise/arm/loss conditions in
three cohort panels. Show full-horizon preview-loss event counts by every
operation category, including empty conditions/categories. Export all 144
all-kind original-mode horizon summary rows and plot provenance. Track SVG;
PNG remains scratch. Inspect the final figure and repeat all collector and
figure artifacts byte for byte.

A standalone verifier imports no analyzers. Extract the original Rust
StationaryEpochFusion types/observe, mean_pose, six epoch-fusion tests, and
VehicleState struct/new unchanged from pinned source into a scratch stdlib
harness. Compile with Rust 2021; run all extracted original tests. Independently
read all original unquoted trajectory CSVs in Rust, validate grids and replay
the original observe for every timeout decision, comparing availability/counts.
Use zero dummy pose and steering because these do not enter group membership,
sample counts or availability; ignore fused positions. This verifies metadata,
not trajectory/pose/controller replay. Emit each requested original snapshot
and compare every field and group with the Python metadata reconstruction.

The verifier independently builds snapshot requests from original broad census
IDs and the pinned preceding event trace, reconstructs event/summary fields
and exact weights from native snapshots, and checks all saved fields, plot
values, inherited accounting and input/helper/script/protocol/output hashes.
Record extracted-source hashes, generated harness hash, Rust version/flags,
six original Rust tests and verified decision count. No network, Cargo build
or third-party Rust crate is needed. Support relocated raw roots without
changing pinned bytes. Test TTL/speed/sample/age boundaries, reset ordering,
capacity, missing observations, full identity, duplicate/unknown requests,
empty/no-forward denominators, fixed parent weights and nonmutation. Run the
complete Python suite. Save report/README/evidence and merge this bounded
study under the standing instruction.
