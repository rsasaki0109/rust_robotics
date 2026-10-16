# Position-error components within the audited first-chord population

Start from main 8e837ef. Pin the preceding coverage-span accounting, events,
chains, members, spans, loss memberships and provenance, the envelope pose
diagnostics, and speed-probe-starts' complete pose queries. Preserve all 648
case records, 368 timeouts, 2,583 opportunities, 6,610 audited states, 19,830
raw/recovery/preview inputs and fixed 550/509 loss populations. Keep both
arms, every cohort/event kind and source censoring. No additional outcome
filter chooses which poses to reconstruct.

Replay all 368 timeout cases from their original initial rows through 180 s:
3,312,000 intervals in 72 hash-pinned files. Reuse the validated observation
window/odometry replay and signed component decomposition, including missed
corrections, precision activations, epoch expiry, reset and fallback rules.
Check window counts at every interval. Export diagnostics at exactly the
6,610 target states; the other timeout histories retain explicit replay
accounting with zero target poses. Support relocated identical raw files.

Match truth to the preceding physical CSV row. Raw uses the current sample;
recovery uses its logged recovery mean/raw fallback, and preview uses its
logged three-epoch fusion/motion fallback. Freeze original means, headings,
weights, steering, speed margin and first-chord decisions. Verify all three
input coordinates and headings against recorded raw and tracked poses, and
all error norms against the previous envelope table, within 1e-8.

Decompose each reconstructed position-error vector into observation noise,
correction-anchor offset, within-anchor bias drift and motion transport error.
Weights come from the recorded estimator, never truth. Their vector sum must
equal the full error and their signed projections on its direction must sum
to its norm. Component norms do not add. Retain negative contributions and
an explicit none label for a zero-error direction. Largest projection is a
descriptive arithmetic label, not a causal obstacle-clearance attribution.
Keep per-group weights, sample bounds, anchor IDs and component vectors.
Reuse model covariance diagnostics with actual spacing, AR correlation and
shared anchor IDs; these are model quantities, not calibrated error bounds.

Summarize all poses, raw_zero_loss and command_only_zero_loss separately,
with cohort/arm/event-kind/input dimensions and conditions all,
central_clear_truth_blocks and envelope_clear_truth_blocks at .10 m / +/-4
degrees. Include explicit empty groups and unique case/opportunity counts.
The two conflict conditions are overlapping descriptive subsets. Keep every
prior loss-membership row and all six radii/two heading bounds unchanged;
component vectors themselves do not depend on envelope settings.

For temporal diagnostics use every prior span at .10 m / +/-4 degrees in
both families, including covered, all outside categories and no_conflict.
Preserve original span IDs, maximality, chain boundaries and truncation.
For error and each component vector calculate its RMS, mean vector norm and
centered RMS across each span. Verify RMS^2 = mean_vector_norm^2 + centered_RMS^2.
Mean vector norm describes an observed persistent offset within that audited
span; centered RMS describes variation around it. A singleton has zero
centered RMS. No independent-sample, full-episode or resolution-time claim
follows from these selected, often truncated spans.

Export unchanged accounting/events/loss-memberships, replay accounting,
all pose components, all estimator groups, component summaries, default-setting
span components and provenance. Repeat all eight CSVs and provenance byte
for byte. Any figure must show the complete 550-loss population in both arms
and all three inputs, with a separately labeled view of every default-setting
clear-conflict span. Retain zero categories and numeric plot data. Distinguish
state-weighted pooled moments from span distributions.

No estimator, release condition, margin, controller, geometry or physical
trajectory changes. Component subtraction is not used as a new controller.
Original physics/observation validation remains inherited; recorded held
goals remain 140->140. Findings are descriptive, not new physical safety or
release improvements.
