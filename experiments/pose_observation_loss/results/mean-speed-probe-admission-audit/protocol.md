# Fixed admission diagnostic for the mean speed-probe pilot

Freeze from main 394b7631ca3c4c7db348b6c45bb80428989a95ef. The previous
108-case pilot's endpoints and 28 changed logs / four changed endpoint distances
are known. No admission-gate distribution or new contiguous readiness census
has been extracted. This is a retrospective mechanism diagnostic of that same
one-seed packet, not new observations, a new treatment or a recovery probability.

Keep every one of the 108 cases, all 54 pairs and all 788,988 decisions from
the 12 frozen trial directories. Preserve none-mode cases and held cases as
well as all timeouts. Bind the full raw provenance and its per-file hashes,
all compact pilot evidence, original sources, fixtures and reused helpers.
Do not change or regenerate trajectories, policies, sample selection, time
budgets, margins or thresholds. Keep previous frozen files unchanged.

At every pre-command safety pause, reconstruct the seven joint admission gates:
observation age <=1.5; absolute measured speed <=0.02; sample count >=10;
recovery clearance > recovery margin; both ends of the original 0.6 recovery
line inside the inclusive margin bounds; nominal target >0; and release-preview
clearance > guard-release margin. Preserve all simultaneous failures.
Use full-precision age/speed/count/mean/nominal target from speed-probes.csv.
Use the logged full-precision speed margin as the recovery/base guard margin:
their expressions are identical in the fixed curved configuration. Confirm
the corresponding source expressions and recorded guard margins.

Compile a small, standard-library Rust geometry reader from the original
Point2D constructors/distance and point_segment_distance function. Retain the
original recovery-line endpoint, obstacle scan and bounds expressions, replacing
only the identical margin expression with the full-precision logged margin.
Query every paused decision, including ineligible and unsuccessful ones, in
fixed trial/row order. Recompute full-precision recovery clearance and bounds,
compare the native clearance formatted to nine decimals with the original log,
and retain exact source/query/output/compiler hashes. These are gate queries
on recorded means, not new simulator steps.

The release-preview clearance is only available to nine decimal places. Use
its closed decimal rounding interval of +/-0.0000000005 and the exact f64
guard-release margin. Classify a strict > gate as pass only when the interval's
lower bound is greater; fail only when its upper bound is <=; otherwise mark
it unresolved. Never substitute a rounded center at that threshold or infer
gate truth from the observed timer/release. Preserve unresolved gates, their
counts and effects on conjunction/timer verification. A different failed gate
can still make the conjunction definitely false.

Reconstruct the native joint timer at every paused decision where the gate
conjunction and its inherited timer are known. Preserve its original dt=0.02,
one-second continuous requirement, +1e-9 completion rule and atomic reset on
admission. Check recorded timer rounding, resumed and post-command pause state.
If an unresolved conjunction prevents independent timer reconstruction, retain
that decision as unverified until a definite failure or observed admission
resets the inherited timer; do not guess. No readiness is inferred from these
observations. Keep every definite-ready contiguous run, its length, and its
ending reason: admission, failure mask, ambiguity, pause end or packet endpoint.

Export all 108 case records, observed gate-pattern counts for both all-pending
and the arm-neutral fresh/stationary/>=10-sample subset, all readiness runs,
and complete arm/noise/mode summaries. Case rows include all native endpoint
brake flags, goal distance, final hold and guard state so a non-paused timeout
is not attributed to the joint loss-release mechanism. Preserve all zero-use
cases and empty summary denominators. Report the positive nominal commands
that still fail another gate and the longest observed continuous ready runs.
Counts of steps, runs, overlapping masks and cases are distinct units.

Keep complete per-decision inputs and native traces outside the checkout.
Track compact exports, exact generated geometry source, protocol/pins, build
and analysis provenance, repeat-comparison proof and report. Write completion
provenance only after all counts/joins/checks pass; preserve failure diagnostics.
Use fresh external output paths and reject Python -O.

Before reading the new gate distributions, commit the implemented diagnostic
and its meaningful tests. Test native line/boundary geometry, inclusive input
thresholds, strict rounded release thresholds, simultaneous failures, 49/50-step
timer behavior, resets, unresolved conjunctions and corrupted native/log rows.
Run the whole diagnostic twice on the same retained raw packet and require
all compact tables, native source/query/trace and provenance bytes to match.
Run the full Python/native suite without skips and relevant CI, then merge
under the standing authorization. A bottleneck in these histories does not
show that relaxing that gate would produce a safe or successful controller.
