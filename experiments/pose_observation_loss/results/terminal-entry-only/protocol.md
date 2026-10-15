# Fixed comparison before running the release ablation

The previous recorded-input diagnosis found long waits attributable solely to
the terminal-line release gate. This experiment changes that release gate while
retaining the terminal-line entry test. It does not tune the entry threshold.

- Reference `recoverable`: entry uses `min(Dstop, Dterminal - 0.1) <= m`;
  both release timers require `min(Dstop, Dterminal - 0.1) > m + 0.05`.
- Candidate `entryline`: same entry rule; both release timers require
  `Dstop > m + 0.05`. `TERMINAL_ENTRY_ONLY` defaults to false and requires
  `RECOVERY_PREVIEW=true`.
- Keep the independent 0.5 s and joint 1 s evidence windows. Retain the original
  measured recovery line, stationary samples, observation freshness and positive
  command requirements for joint recovery. No added dwell time or steering rule.
- Use the same padded path fixtures, physical radius 0.5, noise/drift, motion
  filter, speed cap, loss patterns, 180 s timeout and 10 s hold criterion.
- Known seeds: 259, 260, 261; new seeds: 262, 263, 264. Evaluate both arms on
  independent noise and 1 s correlated noise, all three loss modes, maps and
  planners. Total: 648 comparison runs. Separately recheck original seed201
  defaults (54 runs).
- Report holds and paired gains/losses, contacts, waiting and stationary waiting,
  near-stationary timeouts, guard entries, short re-entries and terminal-only
  entries. Do not change thresholds or seeds after looking at results.
- Check reference summaries against the pinned previous comparison on the known
  seeds. Before the first differing release action, paired tracks must agree
  apart from the two release-mode columns and the two evidence counters.
- Reconstruct both timers, physical integration and observation filtering;
  independently audit candidate entry/release/resume geometry and contacts.
  Repeating aggregation must give identical CSVs and provenance.

The retained entry condition can still fail immediately after stop-only release.
The comparison measures this possibility; it does not assume that relaxing a
release rule improves progress or holding. The known-seed group is already used
for diagnosis and is not a fresh validation group.
