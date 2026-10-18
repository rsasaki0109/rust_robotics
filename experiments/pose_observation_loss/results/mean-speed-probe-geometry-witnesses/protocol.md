# Fixed geometric witness diagnostic of mean-probe admission

Freeze from main dd7ddfc28cf8c94bcde2d82687571f603bb66f61. Existing endpoints,
the 11,998 mean-arm eligible positive commands, 9,276 other-gate failures and
2,722 all-ready decisions are known. No new obstacle witnesses, alternative
heading probes or release-component census have been extracted. This remains
a retrospective diagnostic of the same seed1001 packet, not a new trajectory
trial or a claim that a controller modification would work.

Use all 54 mean-arm cases, including zero-selected cases. Select every decision
with sidecar used=true and nominal_target>0, in frozen trial/case/step order.
Keep all 11,998 selected decisions, including the 2,722 ready controls. Bind
the entire pilot packet, all previous admission evidence and its margin
amendment, reused scripts, original sources and fixtures. Preserve their bytes.
No policy, margin, timer, outcome, raw observation or timeout is changed.

Compile standard-library Rust from the unchanged original point/vehicle types,
point_segment_distance, speed_free_distance, aligned_release_preview and the
previous exact recovery-line/margin reader. Reconstruct both margins with the
fixed command parameters and original precision sigma. Replay the recorded
speed probe at full-precision mean x/y, raw yaw and nominal steering; require
its distance and formatted recovery clearance/release margin to match the log.

For every selected decision evaluate exactly three probe inputs, at fixed x/y
and original margin/map: (A) recorded raw yaw and nominal steering, (B) mean yaw
and the same steering, (C) mean yaw and zero steering. Retain all distances.
These ordered same-step queries isolate inputs; their differences are not an
additive attribution or new vehicle motion. Recompute initial obstacle/wall
clearance. Retain every obstacle tied for the original recovery line's nearest
distance, its original fixture row ID, projection and closest line point.
Never choose only one tied witness in the evidence tables.

The stopping and terminal-preview clearances are logged to nine decimals.
Enclose each in its closed decimal rounding interval +/-0.0000000005, convert
to outward f64 endpoints, and apply the unchanged monotone aligned release
function to both endpoints. Evaluate each component using the other argument
as +infinity. Compare these enclosures to the reconstructed exact release
margin: strict pass only if the entire enclosure is greater, strict fail only
if its upper bound is <=; otherwise unresolved. Require intersection with the
recorded combined-release interval and compatibility with the previous gate
classification. Preserve all unresolved component classifications.

Reproduce all original per-case counts/masks: ready, clearance, release and
clearance;release. Keep every simultaneous failure and all ready controls.
Export all 54 case summaries and complete noise/mode/mask cells, with distinct
units for cases, decisions, obstacles and probe evaluations. Keep the complete
query/trace/witness tables outside the checkout with their full hashes.

For compact evidence, choose the first selected row and the row with minimum
recovery-clearance-minus-margin, separately for every case/mask, breaking ties
by original order. Merge roles when they select the same row. Keep every such
representative, including ready controls, with full native input/output and
all tied witnesses. For plots, choose the first row of each of the three known
failure masks globally in original order. Draw all three full diagnostic paths,
the actual recovery line, obstacles and rounded recorded guard-pose center.
Label the alternatives as static queries and the rounded marker explicitly.
The representative selection does not replace the complete population counts.

Commit the implementation and meaningful native/interval/selection tests before
extracting new witnesses. Run the complete diagnostic twice into fresh external
paths, requiring all tables/source/query/trace/witness/path/provenance bytes to
match. Render the three panels with the existing pinned plotting profile,
compare repeat SVG/PNG bytes and inspect the figure. Track compact exports,
native source/provenance, plots, repeat proof and report. Reject Python -O and
existing output paths, write completion provenance only after all joins and
checks pass, and retain failure diagnostics. Run the full Python/native suite
without skips and relevant CI, then merge under the standing authorization.
