# Fixed replay of terminal-preview inputs

Freeze from main b8136fce0b6ef0577d82c9218025d0281a301657. The previous
seed1001 diagnostic already established 54 mean-arm cases, 394,494 decisions,
11,998 eligible positive decisions, 2,722 ready controls and 9,276 blocked
decisions. Release fails in 6,248 selected decisions: terminal alone in 6,174,
stop and terminal in 74. Those are known outcomes. Full-precision reconstructed
preview/filter histories, stopped poses and the alternative queries below have
not been extracted. This is another retrospective calculation on that packet,
not a new simulator trial or a controller improvement claim.

Bind every previous frozen input, the entire last geometry-witness study and
its scripts, the original simulator/vehicle functions and the complete raw
pilot manifest. Keep all original bytes. Verify the raw packet with the existing
pilot verifier. Freeze this protocol, then commit the implementation and
meaningful tests before evaluating any new full-precision preview results.

Replay every decision in all six mean-arm trials, preserving original
trial/case/step order, including the 18 cases without selected decisions.
Reconstruct MotionPoseFilter and StationaryEpochFusion from the original Rust
structs/methods and full-precision raw observed pose, measured speed, correction
events, observation age and previous applied steering in the sidecar. Use the
original step*DT clock. Never feed physical truth or rounded preview positions
into the estimator. The fixed commands have consistent_restart=false,
motion_filter=true, stationary_epoch_fusion=true, recovery_guard_buffer=false,
motion_entry_buffer=false and terminal_steering_preview=true; reject another
configuration instead of silently changing the replay.

At every one of the 394,494 decisions, require exact agreement with the logged
nine-decimal motion-filter x/y and final preview x/y/yaw, filter sample count,
fusion availability and fusion group count. Reconstruct precision epochs and
both margins from the original ordered expressions, including the separate
base guard margin; require exact round-trip speed-margin agreement. Reset all
history per case and verify every case's step count. Preserve failures and do
not write completion provenance if a join fails.

Select exactly sidecar used=true and nominal_target>0: all 11,998 decisions,
including all 2,722 ready controls. Evaluate these four fixed static queries:

1. recorded: reconstructed original guard-preview pose, measured speed,
   recorded positive nominal target and nominal steering;
2. mean_position: replace only x/y by the recovery-window mean; keep the
   original preview heading, speed, positive target and steering;
3. mean_pose: additionally replace heading by the recovery-window mean;
4. zero_target: keep the original preview pose, measured speed and steering,
   and set the target to zero, solely to diagnose the proposed-motion prefix.

Hold original margins and map fixed. Use the unchanged original stopping_preview,
terminal_arc_clearance, aligned_release_preview and VehicleState::update.
Retain all 47,992 query results. Extract the stopping-motion prefix without
altering its arithmetic, and require its stopped pose to reproduce the original
terminal-arc result exactly. Retain all stopped poses, steps, travel lengths,
clearances and strict gate comparisons. The recorded query must reproduce all
three logged stopping/terminal/combined values formatted to nine decimals and
the previous complete per-case masks and release counts.

Compare mean_position against recorded to isolate initial x/y, mean_pose against
mean_position to isolate heading, and zero_target against recorded to isolate
the proposed target. These are conditional static comparisons, not additive
attribution, future estimator trajectories or measured recovery improvements.
In particular, zero_target is not a positive-command admission candidate.
Retain favorable and unfavorable release flips and all ready controls. Do not
select new steering angles, change margins/timers or remove a gate.

Export all 54 cases for all four variants, including zero-selected cases, and
all noise/mode/original-mask/variant summary cells. Keep cases, recorded
decisions and native evaluations as separate units. Select compact rows using
exactly the previous study's 132 representative IDs and roles, with all four
variants. Export their full static query inputs, native outputs and identity
join. The representatives do not replace the complete population tables.

Run two complete replays into fresh external directories and require all
query/history/source/binary/trace/table/provenance bytes to match. Recompile
the archived source and replay all 528 compact static inputs independently,
requiring complete output equality. Keep full histories and query results
outside the checkout with hashes; track compact exports, source, receipts and
an explanatory report. Reject Python -O and existing/internal output paths.
Run the full Python/native suite without skips and relevant CI, then merge
under the user's standing authorization. Preserve the original controller,
old outcomes, old proofs, dependencies and lockfile.
