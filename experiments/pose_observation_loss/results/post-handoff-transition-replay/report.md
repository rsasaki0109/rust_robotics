# Position continuity after the one-decision handoff

All13 observed lag2 guard entries in the one-step-handoff arm clear the entry
and release geometry thresholds when only preview x/y continues from the
release mean. At those decisions all126 anchors still meet the hypothetical
carry validity gates. This locates the delayed entries in the position
transition; it does not demonstrate a benefit from a longer live handoff.
The13 entries belong to9 timeout cases, not13 independent trials.

The usable interval is short:104/126 candidate anchors invalidate at lag3,
3 at lag4,1 at lag5 and18 at lag18. Continuing past invalidation adds30 static
entry thresholds in each arm. All are outside the original validity prefix.
The experiment option remains default false; this study changes no controller.

## Frozen population and computation

The already completed seed1003 pilot contributes all108 cases/54 pairs:
tau0/1, none/random/blackout,3 maps and3 planners, both mean_preview and
one_step_handoff. The pilot's held23/54 and timeout31/54 in each arm are
inherited observations, not outcomes of a new simulation. All36 cases per arm
with loss have resumes; all18 no-loss cases per arm remain in the exports with
zero resumes. One shared seed and repeated paths limit independence.

Freeze commit03bef3a binds194 existing files before these queries. Implementation
a556ddcbc7e40e85bd7c923ca1b8052aba8bd87d was committed before reading the new
query results. The full137-file,2,743,837,500-byte raw packet and both complete
16-file prior audits match their frozen manifests. Their827706 physical and
estimator decisions and45002 native geometry queries are reused by hash; this
study does not claim to repeat the full physical audit.

Every one of251 joint resumes contributes lag0 and its next50 decisions. None
is truncated in this packet:12801 recorded inputs,12550 following decisions,
37901 native variant evaluations. These counts describe anchored windows.
Overlapping windows would remain separate; this packet's12550 following rows
also refer to12550 distinct case/step pairs. All72 resuming cases,108 total
cases,54 pairs,251 resume records and360 noise/mode/arm/outcome/scope summary
cells are retained, including zero cells and held/timeout outcomes.

At lag0 only recorded geometry is queried. At lag1..50 the three variants are:

- recorded: the actual selected pose and nominal command.
- carried_diagnostic_xy: the release recovery mean, transported sequentially
  using each predecessor's measured speed and actual applied steering, even
  after invalidation. Each update calls the original MotionPoseFilter.observe
  with a singleton and reads the old transported sample before averaging.
- gated_carried_xy: the same position only while every following decision so
  far meets the original validity predicate; otherwise exactly recorded.

The predicate excludes joint_pending, scheduled/received correction, precision
activation, |measured_speed|>0.02, age>1.5 and count<10. A failed prefix never
revives. This is hypothetical carry eligibility: the implemented one-shot
snapshot is actually consumed at lag1 and is unavailable at lag2.
All variants retain current selected yaw/speed, nominal target/steering,
precision, age, ordered margins and the obstacle map. No true pose enters the
transport or geometry query. No timers, physical trajectory or future sensor
history are simulated under an alternative.

Every recorded query matches the previous native trace at full precision and
the original trajectory at9 decimals. All12550 transports match separate
Python equations; all126 valid candidate lag1 poses exactly match the actual
handoff. The original entry flag is rechecked when the recorded prior guard is
inactive and no joint pause is pending. Static entry and release comparisons
are exported separately from this readiness condition and from actual timers.

## Complete follow-up results

The table counts static entry thresholds cleared by gated_carried_xy, including
rows whose recorded guard is already active. The ready column restricts those
flips to an inactive recorded prior guard. Counts are window decisions, not
new hold successes. Gated entry additions and release losses are zero in every
row; release gains equal static entry thresholds cleared in this packet.

| Arm | Scope | Eligible / following rows | Static entries cleared | Cleared while prior guard inactive |
| --- | --- | ---: | ---: | ---: |
| mean_preview | lag1 | 125/125 | 12 | 12 |
| mean_preview | lag2 | 125/125 | 12 | 0 |
| mean_preview | within5 | 301/625 | 56 | 13 |
| mean_preview | within25 | 493/3125 | 195 | 16 |
| mean_preview | within50 | 493/6250 | 195 | 16 |
| one_step_handoff | lag1 | 126/126 | 0 | 0 |
| one_step_handoff | lag2 | 126/126 | 13 | 13 |
| one_step_handoff | within5 | 311/630 | 50 | 15 |
| one_step_handoff | within25 | 527/3150 | 203 | 18 |
| one_step_handoff | within50 | 527/6300 | 203 | 18 |

Actual first entries within5/25/50 remain15/24/30 events in each arm. Immediate
entries are12/125 versus0/126, with exact lag2 entries0 versus13. Each arm has31
trigger rows within50 because an event can contain more than one trigger;
31 must not be substituted for the30 first-entry events. All event joins match
the prior complete follow-up census, stopping its first-entry search before a
new joint pause. There are no new joint pauses in these50-decision windows.

The candidate lag2 result spans every noise/mode cell below; each cell retains
9 cases per arm. Every lag2 anchor remains eligible and there are no added
entry thresholds.

| Noise | Loss | Reference / candidate resumes | Candidate lag2 entries | Cleared with carried x/y |
| --- | --- | ---: | ---: | ---: |
| white | none | 0/0 | 0 | 0 |
| white | random | 53/54 | 7 | 7 |
| white | blackout | 9/9 | 1 | 1 |
| correlated | none | 0/0 | 0 | 0 |
| correlated | random | 54/54 | 4 | 4 |
| correlated | blackout | 9/9 | 1 | 1 |

## Invalidation and unfavorable comparisons

| First invalid lag | mean_preview | one_step_handoff | Simultaneous reasons |
| --- | ---: | ---: | --- |
| 3 | 106 | 104 | moving, insufficient_samples |
| 4 | 3 | 3 | moving, insufficient_samples |
| 5 | 0 | 1 | moving, insufficient_samples |
| 18 | 16 | 18 | stale, insufficient_samples |
| Total | 125 | 126 | |

All anchors invalidate by lag18. Only16 reference and18 candidate anchors are
still eligible through lag5, and none through lag25 or50. Correction, precision
and joint-pause invalidations have no observed examples in these windows;
native and boundary tests exercise them and verify permanent invalidation.
Zero targets remain present:304 reference and344 candidate following rows
within50. They were never excluded from transport, thresholds or summaries.

Unconditionally carrying the point across all50 decisions clears472/460 static
entries but also adds30/30 entries and loses30/30 release-clear thresholds.
The added entries all follow invalidation;12 per arm occur with the recorded
prior guard inactive. Under original validity, the gated variant clears195/203
static entries and adds0/0. These are per-decision geometry counts. The fixed
recorded history cannot reveal how a live policy would alter speed, later
validity, guard timers, collisions or held outcomes.

The result supports position discontinuity as the immediate lag2 mechanism.
It does not select a handoff duration. With104/126 candidate anchors already
moving at lag3, extending time alone is not supported as a complete fix.
Keep the experimental option disabled by default. A useful next diagnosis is
to compare the sensor-only release mean and selected estimator against the
recorded pre-step physical pose across all outcomes, using truth only for
evaluation, before designing another live treatment. Any live treatment needs
a separate frozen protocol, unused-seed audit and delayed-entry follow-up.

## Validation, retention and replay

The complete test suite passes1451 tests, with zero failures, errors or skips.
The18 added tests cover original function preservation, independent original
geometry, all validity boundaries/resets, non-revival, sequential steering and
speed, anchor retention beyond25 samples, both flip directions, zero targets,
endpoints/overlaps, all cases/outcomes, corruption and repeatability. Two test
fixture defects (a mismatched margin and an initial-row identity) were fixed
before the implementation commit; failed test logs remain external. Neither
actual retained-packet query failed or required an amendment.

Both complete fresh queries produce14 byte-identical files, including compiler
logs, source, binary, inputs, traces, exports and provenance. See
repeat-verification.json for every size/hash and validation.json for checks.
Compact cases/pairs/summary/verification and every resume are tracked here.
Complete followups.csv (6,786,789 bytes), queries.csv (24,443,052 bytes), native
queries/trace and binary remain under the external query directories and are
included in the repeat manifest. The raw packet and both prior audits remain
unchanged outside Git. A checkout alone does not include these bulk inputs.

From the repository root, with the retained packet and audit directories:

```sh
source /workspace/.cloud-env/rust_robotics/activate.sh
PYTHONDONTWRITEBYTECODE=1 /workspace/.cloud-env/rust_robotics/python-ci/bin/python -B \
  experiments/pose_observation_loss/analyze_handoff_transition_replay.py \
  --raw /workspace/scratch/post-release-mean-handoff-live-pilot/raw \
  --audit-one /workspace/scratch/post-release-mean-handoff-live-pilot/audit-2 \
  --audit-two /workspace/scratch/post-release-mean-handoff-live-pilot/audit-3 \
  --output /workspace/scratch/post-handoff-replay-fresh-1
```

Repeat with a different fresh external output and compare all14 files. Inputs
must match the frozen manifests; regenerating a pilot at a different commit
does not reproduce this packet's provenance bytes. The repository must be
clean. New executions record their current commit, so sources.json differs
from this archived implementation when run from a later commit; two executions
at the same later commit must still match each other. Python-O, changed hashes,
mixed/reordered inputs and existing/internal outputs are rejected.
