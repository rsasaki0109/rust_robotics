# Frozen audit of the first returned Fusion spell

Freeze from main 96418e817367b7bd53d9296b39519a130250480c before examining new
return-to-loss operation, age or next-receipt comparisons. The preceding study
already established 152 original observed losses, 137 first returns, 42 fully
observed 20 ms returned spells, 47 longer ended spells of 1.52 s, 48 returns at
the 180 s endpoint, and 15 episodes with no observed return. Audit every first
return; do not select only the short spells. Retain all 3,997 original anchors,
972 planned cases, 324 families, 570 timeout histories and the original 10,995
RP records in 157 parent cases. Do not add rollouts, tune retention, change
controller behavior, select new RP states or estimate pose/control benefits.

Pin fusion-return-followup/sources.json at SHA-256
076db5ab572c491d53a88f14493006e830ba75be637bae6a3dd5bd24ebdc3cc7,
its complete input/helper/source/raw chain and output tables. Also pin its
independent-verification.json at
700e730c54999c7aa2ab5297346302106593815413ddb7659bd4bb684b6488b2,
its verifier/native harness and the 108 full late native trace hashes recorded
there. The collector may reuse these exact native traces, produced by the
preceding standalone verifier, as a cache. Require all 855,570 unique 150..180 s
checkpoints in all 570 timeout cases, complete full-case membership and matching
event/role/step keys. A different cache is rejected. Do not claim this cached
collection rereads raw histories. The standalone verifier must freshly replay
all original raw files in the unchanged original Rust metadata implementation.

Save return-endpoints.csv with all original anchors, full identity, event
flags/kind, preceding first loss/return/censoring category and horizons, original
parent measures and RP counts/masses. For every returned anchor independently
locate the first later unavailable checkpoint through 180 s. Save the observed
return span, first subsequent loss time/operation, one-checkpoint flag, and
independent reset/TTL/capacity/age/latest-epoch/older-sample gate flags. Ordered
operation attribution remains: combined motion/precision clear, motion clear,
precision clear, TTL expiry, capacity eviction, age gate, latest-epoch gate,
older-sample gate, other gate. Retain all nine possible operations even when
empty. Report operations as algorithm metadata, not a causal treatment effect.

At the return save group identities/counts, logged age, oldest remaining TTL
and the two conditional deadlines: the first grid offset at which that oldest
epoch would exceed the inclusive 15 s TTL, and the first offset at which the
current age plus elapsed time would exceed the inclusive 1.5 s availability
age gate. These are conditional calculations with unchanged history/no new
receipt, not forecasts; label them explicitly. Preserve logged endpoint inputs
and compare observed ends to these offsets without suppressing other gates.
Do not classify an endpoint using only a matching duration.

For each returned anchor find the next strictly later original census
opportunity and next strictly later received opportunity, if logged. Save their
IDs/kinds/times/offsets, and a three-way end-versus-next-received relation for
observed ends (before/same_checkpoint/after), or no_next_received_in_log. For
censored returned spells leave end comparisons blank. A return at 180 s has
zero subsequent observed span and unknown end/operation/one-checkpoint flag;
even TTL=0 does not prove a loss after the log. The 15 no-return episodes retain
their lower bounds; the 18 original TTL-limit anchors without a next checkpoint
remain outside observed-loss/returned-spell denominators.

Save snapshots.csv at return, the consecutive before/after endpoints of each
observed subsequent loss, and the log endpoint for each censored returned
spell. Save all original native inputs, prior/current epoch identities/counts,
clear/expiry/capacity evidence and independent availability predicates. Preserve
reused physical checkpoints as separate anchor/role references. Save
group-history.csv with all retained groups in these snapshots, oldest first,
including sample count, age and remaining TTL. No pose geometry is replayed.

Save spell-summary.csv with the same 528 fixed cells as the preceding study:
12 cohort/noise/arm groups x three original loss modes plus all_modes x all
plus ten original first-loss operation selectors. Retain empty cells and all
planned/timeout/held case counts. Include original census/RP counts/masses,
observed-loss/returned/no-return counts, ended/censored returned spells,
all nine end operations, short spells, return TTL-limit status, conditional
deadline matches and next-received relations. Original all-census and received
weights remain 1/n_original_case_events and 1/n_original_case_received_events;
RP mass uses original RP parent counts. Sum exact Fraction masses before shares.
Ended-operation fractions condition on observed ends, with blank empty
denominators. Do not estimate eventual durations, failure probabilities or
mode benefits. Distinct case counts are not additive; check additive mode and
original-operation closures and all preceding totals.

Freeze two figures: (1) observed next-loss operations versus censored returned
spells in all 36 original cohort/noise/arm/loss-mode conditions, including empty
conditions; (2) all 137 returned anchors, return time versus observed span to
next loss or censored observed span, distinguishing end operation/censoring and
TTL-limit returns. Export 396 original-mode summary rows and all 137 returned
anchor rows. Track SVG/CSV/provenance, keep PNG scratch, inspect both figures
and repeat collector/plot artifacts byte for byte.

The standalone verifier imports no analyzers or new collector code. Reuse only
the pinned preceding standalone original-census/native helpers. Compile the
unchanged Rust observe/types/mean_pose/VehicleState/tests and run all six
original tests. Freshly read/hash every original raw file, validate every
timeout decision's availability/group count and request every late checkpoint,
not collector-selected endpoints. Independently reconstruct maximal runs and
first returned spells, metadata gates, conditional deadlines, opportunity
joins, group rows, exact parent measures, all summary and numeric plot fields.
Compare fresh native trace hashes to the frozen cache hashes. Save source,
helper, compiler, protocol, raw, cache, output and plot hashes and field checks.
Dummy zero geometry limits this to metadata validation; ignore fused positions.

Test inclusive TTL/age boundaries, coincident/reset gate priority, censored
returns at the final checkpoint, no return, no next receipt, strict opportunity
ordering, repeated anchors, full-case isolation, duplicate/missing native grid,
cache tampering, unequal fixed parent measures, blank/zero denominators,
additive closure and nonmutation. Run the full Python suite. Save a report and
README instructions, then merge/push this bounded study under standing authority.
