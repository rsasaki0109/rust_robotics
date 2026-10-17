# Frozen admission-condition audit at first Fusion returns

Freeze from main dd64cd86192a0d49026ff1e4cc9a97ac374bf36f before inspecting
new condition masks or timer-reset outcomes. Keep all 137 preceding first
returns: 42 TTL-ended spells, 47 age-ended spells and 48 returns at 180 s.
The preceding control audit fixes 120 loss-paused/guarded and 17 guard-only
returns, zero observed resumes/positive final commands, and 42 spells where
the recovery sample requirement is eventually met. Do not select only those
42. Keep all 3,997 original anchors, 972 planned cases, 324 families, 570
timeouts and the original 10,995 RP records in 157 parent cases.

Pin fusion-return-controls/sources.json at SHA-256
d4805319372227496c5b6fece67644ee1d16b04e16cbf9fd846281929508ddf5,
its complete input/helper/source/raw chain and all four output tables. Pin
its independent-verification.json at
61283776151e7bad7af977ff293b0c787b5e8069d806e66fe68af29f7bb38bcb.
No rollout, parameter change, retention tuning, new RP selection, error label,
AUC, counterfactual release or causal controller benefit is introduced.

Read/hash all 108 original raw files and validate full trajectory membership
and unique complete grids. Reconstruct the source observation clock in all
timeout decisions: observation_time = float(step)*0.02, last observation time
initially zero, updated on received correction or precision activation. Use
the original binary64 multiply/subtract ordering, not rounded logged age or
truth_time_s. Verify all printed six-decimal ages. Save exact age and its hex
encoding at audited decisions, since an age printed as 1.500000 need not be
the same side of the source's exact <=1.5 condition in every case.

For every returned anchor audit each decision from the return through the
first observed next-loss checkpoint, inclusive, or through 180 s for a
censored spell. This fixes 3,751 decision references: 3,662 available and 89
next-loss boundary references. The immediately preceding raw decision is an
input for latch/timer recurrence, not an extra audited exposure. Preserve
original anchor roles and reused physical checkpoints. Join the preceding
control snapshots and maximal control runs back to raw inputs/identifiers.

Derive joint_pending from the previous logged loss-pause flag or a current
missed scheduled correction. Joint admission requires seven conditions:
age <=1.5, absolute measured speed <=0.02, recovery sample count >=10,
recovery clearance > recovery margin, recovery segment inside margin-shrunk
map bounds, positive nominal target speed, and release-preview clearance >
guard release margin. Otherwise, an already active guard evaluates stationary
speed and release-preview clearance (consistent_restart is false in these
arms). Record joint and guard branches separately; other branches are not
applicable. Fusion availability is not added to either condition list.

Prove from the pinned source that recovery_margin and the logged speed_margin
use the identical expression in these anticipatory-speed arms; use the logged
speed_margin interval rather than assuming a drift parameter. Other scalar
inputs include the logged recovery x/y/yaw, clearance, nominal speed and release
margin/clearance. Measured speed's 17-place exponential representation round
trips binary64; counts/flags are exact. For nine-decimal quantities use a
half-unit rounding interval (5e-10) with a conservative 1e-12 numerical guard.
Propagate endpoint uncertainty with the global sin/cos Lipschitz bound and
the original 0.6 m segment. Preserve pass/fail/unknown for strict/inclusive
inequalities; never force a side from a rounded equality. A true stop latch
forces the nominal command to exact zero in the pinned source. Positive
nominal commands printed as zero otherwise remain potentially ambiguous.

Save admission-decisions.csv with full identity/anchor IDs, time/offset/phase,
original raw inputs, exact age, branch, each condition's pass/fail/unknown or
not_applicable state, simultaneous failed and unknown condition masks, and
overall certainty. Retain clearance/bounds/nominal interval evidence. Do not
choose one priority cause when several necessary conditions fail.

Decode the logged recovery and guard evidence as integer 20 ms counters and
retain the actual preceding counter at the return. Joint admission uses 50
consecutive ready ticks and clears guard evidence; guard release uses 25 ticks.
Independently infer ready_from_timer from a +1 counter step, a reset to zero,
or the recorded release/reset at the threshold. Check latch/release flags,
counter changes and known condition certainty for consistency. Unknown
conditions stay unknown; timer evidence must not relabel an individual gate.
Save active counter before/after, threshold, ready_from_timer and timer action
(accumulate, reset, stay_zero, release or not_applicable). A reset means a
previously positive counter is cleared by a failed decision; distinguish that
from another failed decision while the counter is already zero.

Save admission-runs.csv as maximal inclusive runs of branch, phase, overall
certainty, ready_from_timer, failure/unknown masks and timer action. Runs must
reconstruct every decision reference without gaps, overlap or adjacent equal
runs. Save anchor-admission.csv for all 3,997 original anchors with full identity,
original loss/return/end/censoring, horizons, parent measures and RP accounting.
Nonreturns have blank new admission fields. Returns add branch/reference counts,
available versus boundary readiness/failure/unknown/reset counts, maximum active
logged counter, and per-condition pass/fail/unknown reference counts. Censored
spells keep unknown future ends; a 180 s return contributes one observed tick.

Save admission-summary.csv with the preceding 336 cells: 12 cohort/noise/arm
groups x three original modes plus all_modes x all and six preceding disjoint
anchor categories. Keep empty cells and planned/timeout/held counts. Retain
original census/received/RP weights before selection, exact Fraction sums before
shares, and checkpoint references separately from anchor masses. Summarize all
branch/condition/counter outcomes, including unknowns, and raw/original-case-mass
fractions of returned anchors with a positive ready tick or a timer reset.
Empty denominators stay blank. Check additive mode/category closures; distinct
case counts and maximum counters are not additive.

Freeze one figure: per original cohort, condition-failure and condition-unknown
reference counts on available checkpoints, with joint and guard branches
separate and denominators explicit. Export all 252 original-mode summary rows
and all 137 returned-anchor rows. Keep all original conditions in the numeric
exports, including empty cells. Track SVG/CSV/provenance; PNG stays scratch.
Inspect the figure and repeat collector/plot artifacts byte for byte.

The standalone verifier imports no analyzers or new collector/plot code. Reuse
only pinned preceding standalone census/hash helpers. Independently read all
raw files and reconstruct the clock using last-observation step IDs. Use
Decimal intervals and a Decimal Taylor-series sin/cos implementation for the
geometry checks, independent of the collector's float/libm interval arithmetic.
Independently reconstruct timer transitions, all fields/weights/summaries/plot
values and maximal runs with SQLite window functions on uncompressed decisions.
Verify original census joins, preceding controls and all input/helper/source/
protocol/raw/output/plot hashes. Native metadata evidence remains inherited;
do not claim a new native metadata or controller trajectory replay.

Test strict/inclusive interval boundaries, ambiguous rounded zeros, stop-latch
zeros, binary64 ages around 1.5, trigonometric bounds, simultaneous failures,
joint versus guard applicability, 49/50 and 24/25 timer boundaries, resets versus
zero waits, carried counters, inconsistent release/counter records, endpoint
censoring, absent returns, case/grid isolation, original unequal parent measures,
blank/zero shares, exact closure and nonmutation. Run the full Python suite.
Save report/README/evidence, then merge/push this bounded audit under standing
authorization.
