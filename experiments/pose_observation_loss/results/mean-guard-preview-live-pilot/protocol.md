# Fixed live pilot of an eligible mean-position guard preview

Freeze from main e1362d58fbab65313e5f3fa170f0aa8996862240. Seed1001 outcomes
and retrospective queries are known: mean-position substitution produced 849
release fail-to-pass and 1,027 pass-to-fail changes, with 151 improvements among
153 release-only failures and all original ready controls retained. Heading
replacement added no flips. These are design observations, not the outcomes
of the live candidate below.

Choose the smallest seed >=1001 absent from every tracked results CSV's seed
column, record the catalogue hashes, and freeze that seed before implementation
or new pilot runs. Use correlation times 0 and 1 second, all original modes
none/random/blackout, all three fine-padding maps and all three planners.
Both arms have the original mean-position speed probe enabled. Reference
fused_preview keeps the original guard-preview position. Treatment mean_preview
uses the current recovery-window mean x/y only when the joint pause is pending
and the original mean speed-probe selector is eligible. There are 108 main
cases and 54 pairs, including all 18 no-loss pairs, with the original 180-second
budget. One shared seed is not 54 independent observation seeds.

Generate a new experimental source outside the checkout from the pinned
mean-speed-probe generator, using strict unique source anchors. Add an optional
final MEAN_GUARD_PREVIEW boolean, default false. Enabling it requires the mean
speed probe and the fixed curved joint-recovery/averaged/stopping-filter/fusion
configuration, with consistency filtering disabled. Preserve the original
simulator, library defaults, manifests, lockfile, fixtures and all old evidence.
Use locked Cargo artifacts and the exact same rlibs for legacy and new sources.

Immediately after the existing stationary-epoch fusion, before stopping_preview,
replace only preview_pose.x/y when enabled && joint_pending &&
mean_speed_probe_used. The existing selector uses paused state, measured
|speed|<=0.02, observation_age<=1.5 and at least 10 current-window samples.
Do not create a new window or change eligibility. Preserve preview heading,
speed, other vehicle fields, steering, positive/zero nominal targets, margins,
timers, path-cursor logic, noise streams and all existing geometry functions.
Use the original fused/motion-filter pose when ineligible. Physical truth is
never a new controller input. Retain zero-target eligible uses as well as
positive-target uses; eligibility does not depend on future outcomes.

Keep all 82 trajectory and 35 speed-sidecar columns unchanged. Add a separate
guard-preview-inputs.csv for every decision, retaining enabled/used/pending/
speed-selector flags, full-precision original preview and selected pose/speed,
plus original baseline pose formatted to nine decimals. The log is an output,
not a controller input. Preserve every raw file and its hash, exact commands,
source/generator/compiler/rlib/binary hashes and clean implementation commit.
Require at least 3 GiB free before the full pilot; retain failed runs and
diagnostics rather than reusing output paths.

Commit the implemented generator, runner, auditors and meaningful unit/native
tests before producing new pilot outcomes. Freeze the eligibility, seed, grid,
180-second budget and reports; do not retune after seeing results. Diagnose and
document any implementation/audit defect and bind corrected execution as needed.

Run the legacy mean-speed-probe binary for every one of the six reference
conditions (54 extra cases), requiring complete trajectory/results/speed-sidecar
bytes to match the new explicitly-disabled reference. Also run the new binary
with its final flag omitted and the original unmodified binary for none/tau0
(9 cases each), requiring complete trajectory/results equality to the reference;
the omitted binary's speed sidecar must also match. These 72 regression cases
are excluded from the 108-case treatment denominator. A small synthetic native
fixture in CI must exercise defaults, active eligibility and corrupted-input
detection without rerunning the full pilot.

Audit every physical step, swept contact, boundary, measured-speed predecessor,
braking command, native hold, pause/resume count and endpoint. Reuse the frozen
pilot's full sidecar/window/dynamics/contact/timer/probe auditor. Both arms
receive its mean-speed-enabled label internally; exports retain their actual
arm labels. Only its estimator-check dependency receives the original logged
pre-substitution x/y view, through a separate function namespace. All physics,
gate, outcome and probe checks receive the original native rows. The new
auditor separately requires the actual selected x/y to equal the fixed rule,
and yaw/speed to remain exactly original; original raw logs are never edited.

Independently replay original Rust motion-filter and epoch-fusion history from
full-precision observed sensors and prior applied steering at every decision.
Require the full original preview pose to match the new sidecar, all logged
filter counters/availability to match, actual selected pose to match both full
sidecar and original trajectory rounding, and original ordered margins to match.
Replay original stopping/terminal/aligned-release geometry for every guard use
plus first/last/every-250th decisions and both sides of use transitions.
Require all sampled nine-decimal component clearances and release margins to
match, retaining complete native queries and checks. Independently sampled
Python speed-probe geometry keeps the original fixed sampling rule.

Pair initial states and latent observations, retain the common prefix and first
original-row difference, and require the first changed row to have a treatment
guard use. No-loss original trajectories and speed sidecars must be identical
and guard use must remain zero. Export all native cases, all 54 paired outcomes,
all noise/mode cells, geometry samples and verification counts. Preserve all
gains, losses, failures, contacts and unchanged pairs without selection. Report
native outcomes separately from counterfactual geometry and regression cases.

Run the complete auditor twice on the same retained raw packet into fresh
external paths, requiring table/source/query/native-trace/binary/provenance
bytes to match. Track compact results, source diff, manifests, repeat proof and
report; keep raw trajectories and binaries outside Git. Reject Python -O,
existing or internal output paths. Run the full Python/native suite without
skips and relevant CI, then merge under the user's standing authorization.
The option remains experimental and disabled by default regardless of results.
