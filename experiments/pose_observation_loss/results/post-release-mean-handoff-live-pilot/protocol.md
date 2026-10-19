# One-decision mean-position handoff after joint recovery release

Freeze from main 65358d7bcdf6b404ac4f087faf42af6e1c718daa. Known seed1002
diagnostic: the mean-guard arm had8 immediate re-guards in70 joint resumes;
fixed next-decision inputs with carried mean x/y cleared those8 geometric
thresholds. A current-recovery-position alternative added2 entries after its
window expired. These are design observations, not outcomes for this live pilot.

Choose the smallest seed >=1001 absent from numeric seed columns of all tracked
results CSVs; record every catalogue hash and freeze before implementation or
new outcomes. Use tau0/1, all modes none/random/blackout, all3 fine-padding maps
and3 planners, original180s budget:108 cases/54 pairs, including18 no-loss pairs.
Both arms enable the existing mean speed probe and mean guard preview. Reference
mean_preview explicitly disables the new option; one_step_handoff enables it.
One shared seed and repeated planner paths are not54 independent observations.

Implement exactly ONE following decision, matching the prior query horizon,
with no tunable duration. Generate an experimental source outside the checkout
from the frozen mean-guard generator using strict unique anchors. Add final
POST_RELEASE_MEAN_HANDOFF boolean at argument31, default false, requiring
mean_guard_preview and its original configuration. Keep all preexisting sources,
defaults, fixtures, lockfiles and evidence unchanged. Use locked Cargo libraries
and identical rlibs for old and generated sources.

After a joint resume, seed a per-case optional snapshot of that decision's
sensor-only recovery mean x/y/yaw, measured speed and actual applied steering.
At the next guard decision consume the snapshot once, even when invalid or
disabled; it must never persist to a second following decision. Use it only if
enabled, no joint pause is pending, no scheduled correction/received correction/
precision activation occurs, |measured_speed|<=0.02, observation_age<=1.5, and
the existing current recovery window still has>=10 samples. Do not retain an
expired window, average a new observation into the snapshot or change any reset.
Transport that single old mean using the original MotionPoseFilter.observe with
the stored measured speed and applied steering; read the transported old sample.
Replace only guard-preview x/y, retaining current yaw/speed and all other vehicle
fields. Do not change speed probes, nominal commands, margins, stopping/terminal
geometry, joint/guard gates, timers, braking, path-cursor rules or noise streams.
The new logic must never read true pose or future outcomes.

Preserve82 trajectory,35 speed-sidecar and19 guard-sidecar columns and their
meaning: guard selected fields record the final pose, guard used remains the
original pending-joint selector flag. Add a full-precision handoff-inputs.csv
on every decision with enabled/available/valid/used/seeded flags, pre-handoff
pose, final pose, prior snapshot and event/window inputs. Log output is never
controller input. Preserve raw files, commands, clean implementation commit,
source/compiler/rlib/binary hashes and all failures. Require>=5GiB free.

Freeze and commit meaningful source, native and end-to-end tests, generator,
runner and independent auditors before generating pilot outcomes. No retuning
after seeing results; document any defect/amendment and retain failed artifacts.
Run the old mean-guard-preview binary with its option true for all6 reference
conditions (54 extra cases), comparing full trajectory/results/speed/guard bytes
to the new explicitly-disabled reference. Run the new binary with its final flag
omitted at tau0/random (9 extra cases), comparing those same4 files. Run original
unmodified source at tau0/none (9 extra cases), comparing trajectory/results.
The72 regression cases are outside the108-case treatment denominator.

Audit every physical step, swept contact/boundary, raw sensor recovery window,
measured-speed predecessor, applied command/braking, joint/guard flags and timers,
native hold/endpoint and pause/resume counts with the frozen full pilot auditor.
Only its baseline-estimator dependency receives original pre-substitution x/y;
physics/gates/outcomes always receive untouched actual rows. Independently verify
new availability exactly equals previous enabled resume, one-shot consumption,
all validity/reset gates, logged prior snapshot, transported x/y and unchanged
yaw/speed. Validate guard joint-use separately from the new handoff use.

Independently replay original Rust motion-filter/epoch-fusion at EVERY decision
from full observed sensors and previous applied steering, including both selectors
and a separate reconstruction of one-shot state. Require full selected inputs
and original counters/availability/rounding to match. Replay original ordered
margins and stopping/curved-terminal/aligned-release for ALL original mean-guard
uses, ALL handoff uses, first/last/every250th decisions, both sides of either
selector-use transition, and every resume through its next50 decisions (clipped
only at packet end). Preserve every query/history/native trace and complete source.
Keep the original fixed independent Python speed-geometry sample rule.

Pair initial states, latent observation streams and common prefixes; require the
first original-row difference to have a candidate handoff use with identical
pre-intervention sensor/nominal inputs. No-loss trajectory/speed bytes must match
and both selectors' joint/handoff uses must be zero. Keep all108 cases,54 pairs,
all12 noise(all/white/correlated)/mode(all/none/random/blackout) summary cells,
all gains/losses/contacts/failures, and all joint-release follow-up records.

For EVERY resume, record the first later guard-trigger step until the next joint
pause or packet endpoint; retain no-trigger and censored records. Report entries
at lag1,lag2,and within5/25/50 decisions with available follow-up at each horizon,
not just immediate entries. This tests whether the option merely delays re-guard.
Outcomes and diverged histories remain descriptive; do not select only valid
handoffs, favorable maps, successful holds or positive nominal targets.

Run the complete auditor twice on the same retained raw packet in fresh external
directories, requiring all tables/provenance/native history/trace/source/binary/
logs byte identical. Track compact exports, complete generated/native sources,
diff, raw manifest/provenance, repeat proof, validation and report. Keep bulk raw
trajectories and binaries outside Git. Reject Python-O, changed hashes, mixed/
reordered rows and existing/internal outputs. Run the full suite without skips
and related CI, then merge under standing authorization. The option remains an
experimental default-false source generator regardless of the pilot outcome.
