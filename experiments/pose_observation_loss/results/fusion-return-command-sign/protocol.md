# Frozen source-domain audit of nominal command signs

Freeze from main d703438680b35e8997cfac2ea64a0e4fd7ab5e9f before reading
new raw speed-probe/cap/uncapped-command outcomes. The preceding audit has
909 ambiguous nominal gates among 3,270 available joint references; 25 have
no other failed gate, in four full cases. Its timers report those 25 as not
ready. Preserve that rounded-input result and add a separately named source-
domain result; do not infer nominal signs from timers.

Pin fusion-return-admission/sources.json at SHA-256
f83beeea16ed0eb61846be0c3ce58299ec4ef8df29f37ea8dc50261938bfc056 and
independent-verification.json at
27857ce67c5446350510faa25badc2edbd6188b62e84c102ada889d07f76c018.
Verify its complete input/helper/code/raw chain and four output hashes.
Retain all 972 planned cases, 324 families, 570 timeout histories, 3,997
original anchors, 137 first returns and 3,751 decision references (3,662
available and 89 next-loss boundaries). Do not select only the 25 uncertain
references or change horizons, arms, parameters, RP selection or controller.

The pinned speed_free_distance function returns only k*0.05 for k=0..40:
the initial unsafe case returns zero, the 40-step loop returns (step-1)*0.05,
and the exhausted loop returns 2.0. Check these source return paths and the
source flag implications from stationary_epoch_fusion through joint_recovery
and anticipatory_speed. Decode each nine-place free-distance field to its
unique lattice member; reject off-lattice values. Calculate all 41 binary64
caps in the collector. Independently extract and compile the unchanged cap
expression with stdlib Rust, retaining bit patterns, nine-place strings and
positive signs for every lattice value. Test zero, first-positive, clamped
and complete lattice cases. This is a finite command-domain audit, not an
unrounded map-geometry or controller trajectory replay.

Before the speed cap, an unlatched command is at least 0.1 (or TARGET_SPEED
when terminal slowdown is off); the recovery min(0.4) preserves positivity.
The nonjoint paused-zero branch is unreachable in these joint arms. A true
stop latch forces exact zero. Verify the pinned source ordering through
uncapped_target_speed, min(speed_cap), nominal_target_speed, and the later
pause/guard overrides. A positive lattice cap has a positive lower bound;
establish the minimum possible positive nominal command independently of
logged timers. Rounded zeros can then be classified using this source domain.
Keep the cap-zero, latch-zero and both-zero reasons separate.

Both readers freshly hash all 108 original raw files and validate trajectory
membership, unique complete grids and all timeout decisions, excluding the
initial row. For every timeout decision check the decoded free-distance
lattice, exact cap's printed value, latch-dependent uncapped command range,
and nominal command. Use integer decimal nanounits for the exact identity
round9(min(a,b)) = min(round9(a),round9(b)); do not reconstruct an unrounded
uncapped command from its printed value. Check the final target against the
later recorded pause/guard flags and check speed_braking with the exact cap
and round-tripping measured speed. Save full-grid counts by original cohort,
arm, mode, branch and zero reason, separate from selected anchor references.

Join every preceding audited decision to the original raw point and retain
full identity, event/phase/time, raw free distance, cap, uncapped/nominal/final
commands, stop/restart flags, decoded lattice and cap bits. Keep prior gate,
certainty, failure/unknown masks and observed timer action/readiness. Add a
source-domain nominal gate and its structural reason, refined conjunction
and all simultaneous masks. Other six gates retain the preceding independently
verified interval states. Only after source-domain inference compare refined
conjunctions with timer-derived readiness, as a consistency check. Timers
are never inputs to the new sign inference. Preserve guard-only applicability.

Save command-lattice.csv (41 rows), timeout-command-counts.csv (including all
zero cells for both pause branches and structural reasons), command-decisions.csv
(all 3,751 references), anchor-command-sign.csv (all 3,997 original anchors,
blank new fields for nonreturns), command-summary.csv (the preceding 336 fixed
cells) and sources.json. Preserve original census/received/RP weights before
selection, exact Fraction sums before shares, all 10,995 RP records and masses
570/570/157. Report available/boundary and joint/guard references separately,
including prior/refined unknowns, source-zero reasons and timer resets. Keep
distinct case counts separate from additive fields and verify mode/category
closures. Undefined denominators remain blank; observed zeros remain zero.

Freeze one figure comparing prior interval versus refined source-domain joint
conjunctions by original partition on available references. Include pass/fail/
unknown and explicit denominators, keeping empty partitions visible. Export
all 252 original-mode summary cells and all 137 returned anchors. Track SVG,
CSV and provenance; keep PNG in scratch. Inspect the figure. Repeat collector
and plot artifacts byte for byte.

The standalone verifier imports no analyzers or new collector/plot code. Reuse
only the pinned original standalone census/hash helper. Independently parse
raw numeric fields using decimal integer nanounits and a native Rust lattice
lookup rather than collector float calculations. Independently rejoin original
census accounting and all prior decision fields, reconstruct refined masks,
anchor weights/counts and summaries, and verify every saved field and input/
helper/code/protocol/raw/output/plot hash. Earlier geometry, clocks, timers
and native Fusion metadata evidence remain explicitly inherited from the
pinned proof; do not claim they are replayed in this audit.

Test invalid/off-lattice inputs, nonfinite/negative values, nanounit parsing,
source flag/order checks, native cap bits/formatting, exact zero versus positive
domain bounds, latch/cap/both reasons, rounded min identity, final overrides,
braking threshold, timer-independent sign inference, simultaneous masks,
guard-only applicability, nonreturns/censoring, reused physical points/case
isolation, unequal original parent weights, blank/zero shares and closures.
Run the complete Python suite. Save report/README/evidence and merge/push this
bounded retrospective audit under the existing authorization.
