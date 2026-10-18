# Fixed closed-loop pilot of a mean-position speed probe

Freeze from main 836697cebf36041de0c7d57d3f1add9f40f83d13 before implementing
or running the candidate. The preceding 19-reset geometric results are known.
This is a small engineering pilot on new observations, not an independent
population estimate or a claim of a generally better recovery controller.

Choose the smallest seed >=1001 absent from the retained cases.csv and
all-results.csv seed columns. Record that audit and pin those catalogues.
Use that one seed, correlation times 0 and 1 second, all three original
availability modes (none/random/blackout), all three fixed maps and all three
planners. Both arms use the existing fine-padding fixtures and complete curved
configuration. This gives 108 cases and 54 paired comparisons. Keep all
outcomes and use a 180-second budget; do not select cases after their results.
One reused seed across conditions is not 54 independent observation seeds.

Pin the original simulator, core/control sources, manifests, Cargo.lock,
fixtures, preceding probe evidence and reused audit helpers. Preserve these
files: historical audits depend on their exact bytes. Generate an experiment
source outside the checkout from the pinned original with strict, uniquely
matched edits. Save its complete text, digest and unified diff. Use the locked
Cargo build to obtain the original example and its exact core/control rlibs,
then compile the generated source against those artifacts. Do not change
workspace membership, dependencies, lockfiles or library defaults.

Add one optional final MEAN_SPEED_PROBE boolean, default false; enabling it
requires joint recovery and averaged recovery. The reference uses false.
The candidate uses true. At the speed-probe decision, substitute only x/y
with the existing current-step recovery_x/recovery_y when safety_paused is
true, measured |speed|<=0.02, observation_age<=1.5 and the existing recovery
window contains >=10 samples. Preserve raw sensed yaw, speed, nominal steering,
the original speed margin, map and speed_free_distance body. Use the original
raw sensed pose whenever ineligible. Do not create another averaging window,
change how the existing window resets, or alter any other policy calculation.

Keep all 82 original trajectory columns and results unchanged in format.
Write a separate per-decision speed-probes.csv sidecar with full-precision
raw sensed inputs, speed, observation age, correction/precision/pause flags,
window count/mean, selected probe pose, steering, margin, distance and speed
cap/nominal target. Record every actual decision, not only changed or successful
ones. This log must not feed the controller. Preserve all raw files outside
the checkout and record their hashes, exact commands, source/generator/compiler/
rlib/binary/fixture digests and the clean implementation commit.

Commit the implemented generator/runner/auditor/tests before producing pilot
outcomes. Do not tune the eligibility, seed, margin, timeout or sample after
observing results. A discovered implementation or audit defect must be
diagnosed and documented, with new execution provenance as needed.

For none, tau=0 and the fixed seed, run the unmodified original example and
the generated example with the final argument omitted. Compare both complete
trajectory/results files against the explicitly false reference from the
pilot, byte for byte. These 18 additional regression cases are controls,
not new cases in the 108-case treatment denominator. Unit-test disabled and
enabled selection, exact sample/speed/age thresholds, yaw/steering preservation,
fallback, strict source edits and native compilation. Include a small native
fixture regression in CI without reproducing the full pilot there.

Audit every trajectory step for dynamics, braking, swept-segment contact and
map bounds. Recompute holds, original joint recovery/guard timers, motion
filter and epoch-fusion history, pause/resume counts and reported endpoint
outcomes. Audit every sidecar decision against the reconstructed original
recovery window and the fixed eligibility rule. Verify raw input/log rounding
overlaps, full-precision selected position, unchanged yaw/steering/margins and
the speed-cap/nominal-target formulas. Comparison tolerances never decide
mean eligibility or native contact/hold outcomes.

Independently replay probe geometry using the existing Python full-probe
implementation at fixed samples: first and last decision, every 250th step,
and each sidecar-use transition (including its immediate predecessor). Keep
all sampled matches/failures and their counts. This sampling supplements the
unchanged native function and full policy/dynamics audit; it is not a fresh
independent evaluation of geometry at every step.

Pair observations and initial states, and verify the common prefix before
the first changed original trajectory row, including the physical predecessor
and current latent observations at the first divergence. Require complete
none-mode equality where mean selection is never used. Report all paired
gain/loss/unchanged outcomes, contacts/boundary exits/timeouts, mean-use counts,
common-horizon paused time and endpoint goal distance by noise/mode and overall.
Do not sum ending multiplicities, select favorable planner subsets, turn
counts into recovery probabilities, or treat absence of contacts as a safety
guarantee. Retain both favorable and unfavorable changes.

Repeat the full aggregation on the same retained raw files and compare all
CSV/provenance bytes. Track compact case/pair/summary/verification tables,
protocol/pins, generated-source diff, raw provenance and report; keep large
logs, binaries and raw trajectories in scratch. Run the complete Python/native
suite without skips, relevant native tests and CI, document the replay commands,
then merge/push under the standing authorization. Keep the option experimental
and disabled by default regardless of the pilot's results.
