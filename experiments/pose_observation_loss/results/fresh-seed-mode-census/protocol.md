# Additional seeds: late-correction estimator-mode census

Start from main 72ffbb6. Before running new outcomes, fix seeds 277,278,279,
the next unused observation seeds after 274--276. Repository protocols,
runner commands and provenance contain no prior simulation using these seeds.
Run both epoch and curved, tau=0 and tau=1 s, all none/random/blackout modes,
all three maps and all three planners: 162 paired cases, 324 trajectories.
Use the existing run.py --terminal-arc-comparison and padded planning fixtures.
Keep simulator SHA-256
81711bb44ae1741c5f0e1f71180b047284bf098066f9b1246915130b094f9228
and every model/controller/geometry/noise parameter unchanged. Do not replace
seeds, stop for favorable results, tune thresholds or refit any prior cutoff.

The primary question is whether additional seeds supply both preview modes at
late correction opportunities. Account for every trajectory and retain its
status. The census includes every timeout's correction or precision activation
at logged times [150,180] s, taking one immediate, post-update sample per event.
Include received, missed, precision_only, received_and_precision and
missed_and_precision separately, plus all and received_any unions. A scheduled
timeout must retain all seven scheduled events, including the 180 s endpoint.
Do not condition on raw speed loss, gate readiness, movement, error, release or
whether recovery averaging is available. Non-timeouts stay in accounting but
do not enter this explicitly outcome-selected timeout census.

Replay full timeout histories with the unchanged observation/fusion Replay
helper. Check logged counts at every interval and actual/preview/motion poses
at every retained event. Export preview motion_filter/three_epoch and actual
raw_fallback/recovery_mean, sample/group counts, event flags and physical truth
time (the preceding row). Use the same immediate timeout-event definition to
compare all prior 269/270/271 and 274/275/276 cases, via pinned original
correction-centre events and immediate snapshots. Do not rerun those old seeds
or conflate their existing data with the fresh trajectories.

This wider census differs from the earlier selected speed-probe audit states.
In particular, it cannot establish that the narrow audit's Filter support or
its cutoff/risk behavior transfers. Report that population distinction next to
the comparison. No score, cutoff, error fraction, AUC or policy is selected from
these new data. Three additional seed streams are not 324 independent trials:
maps/planners/modes/noises and arms reuse the prescribed seeds, event samples
repeat within trajectories, and both arms share inputs until commands diverge.

Export full historical+fresh accounting, immediate event records, case-level
mode support and complete per-cohort/per-seed (plus all-seed) summaries by arm,
loss mode and event kind, including empty groups. Count cases once within a
mode; expose mixed-mode cases rather than adding Filter/Fusion case counts as
if disjoint. Report timeout/non-timeout denominators, event counts, case counts,
Filter/Fusion event shares and parent-case-weighted shares (1/n_events before
conditioning on preview mode). Empty fractions remain blank. Preserve actual
recovery mode counts. Do not pool away the per-seed rows or claim confidence
intervals/causal sensor effects.

Use run.py's existing locked release build and full noise/filter/recovery/hold
and exact-preview verifiers on all fresh trajectories. Independently audit
Euler motion, radius-.5 swept contact and map bounds, and the paired shared
physical/sensed prefix. Report held/timeout/contact/out-of-bounds, gained/lost
holds, both-held time difference and common-horizon waiting for every fresh
pair. These are additional fixed-simulator outcomes, not real-world validation.
Repeat the first white epoch-none-277 nine-case invocation with the unchanged
binary and compare full trajectory/result bytes. Pin raw trajectory hashes,
summary/provenance/verifier hashes, simulator/fixture/helper/protocol hashes
and exact commands. Keep raw data in scratch and portable CSV/JSON/SVG plus a
report in the repository. Repeat all aggregation/plot artifacts byte for byte,
inspect the plot, run meaningful collector regression tests and the existing
Python suite, then merge this bounded experiment to main.
