# Fixed comparison before running entry-compatible release

The previous stop-only release experiment often released while its terminal
entry condition already failed. This comparison restores the terminal entry
requirement at release without restoring the original extra terminal hysteresis.

- Reference `entryline`: entry passes iff `Dstop > m` and
  `Dterminal > m + 0.1`; release requires `Dstop > m + 0.05`.
- Candidate `aligned`: same entry, release requires `Dstop > m + 0.05` and
  `Dterminal > m + 0.1`. Implement the release metric as
  `min(Dstop, Dterminal - 0.05)` against `m + 0.05`.
- Thus release implies entry clearance on the same instantaneous inputs.
  It does not guarantee entry clearance after motion, noise or command changes.
- Retain both timers (independent 0.5 s, joint 1 s), measured stationary recovery
  geometry, observation freshness/samples, positive joint command, filtering,
  speed cap, steering and braking. No additional dwell or entry suppression.
- `ALIGNED_TERMINAL_RELEASE` defaults to false and requires
  `TERMINAL_ENTRY_ONLY=true`, which already requires `RECOVERY_PREVIEW=true`.
- Known seeds 262–264; new seeds 265–267. Run independent and 1 s correlated
  noise, three loss modes, three maps and three planners, both arms: 648 runs.
  Recheck original seed201 defaults separately (54 runs).
- Keep padded fixtures, radius 0.5, all noise/drift parameters, 180 s budget and
  the 10 s hold criterion. Do not tune thresholds or seeds after seeing results.
- Report holds and paired gains/losses, contacts, waits and stationary waits,
  near-stationary timeouts, guard entries, short re-entries, terminal-only entry,
  releases with failed entry clearance and immediate 0.02 s re-entries.
- Check known reference summaries against the prior stop-only comparison.
  Before the first differing release action, both tracks must agree apart from
  the release-mode/metric columns and the two evidence counters; the reference
  may release first because the candidate is more restrictive.
- Verify physical integration, paired randomness, filtering, both guard timers
  and goal holding. Independently audit all candidate entry/release/resume
  geometries, fixed checkpoints and contacts. Repeat aggregation byte for byte.

This comparison changes both independent and joint release; it does not separate
their individual effects. The known group is already used for diagnosis.
