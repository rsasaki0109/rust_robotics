# Fixed-input replay after the one-decision handoff

Freeze from main e2dd5d741a9915ddd911e7babf425757b353727d. The seed1003 live
pilot is already known: held23/54 in both arms; immediate guard entries12/125
versus0/126, exact lag2 entries0 versus13, and within5 entries15 versus15.
These observations motivate diagnosis, not a new live treatment or duration.

Use the complete retained seed1003 packet (108 cases,54 pairs,251 joint
resumes). Bind its137-file raw manifest, provenance and both byte-identical
16-file prior audits, including the complete native history/trace. Reuse the
prior independently checked827706 physical/estimator decisions and45002 native
queries by hash, without claiming to rerun the full physical audit here. Read
all cases and all resumes, retaining both held and timeout outcomes, every mode,
no-loss controls and zero nominal targets. No new simulation or seed selection.

At every resume select that decision and the next50 decisions, clipped only at
the packet endpoint. Fifty is the already frozen native follow-up horizon in
the live pilot, not a proposed controller duration. Keep rows after a new joint
pause or invalidation, mark them explicitly and never use them as valid carry.
Keep overlapping resume windows as separate anchored diagnostics and report
counts as window decisions, not unique observations or independent samples.

For lag0 replay only the recorded geometry. For every lag1..50 replay exactly
three variants: recorded, carried_diagnostic_xy, gated_carried_xy. Initialize
the carried point from the release decision's sensor-only recovery mean
x/y/yaw. On each subsequent decision transport that old point with the ORIGINAL
MotionPoseFilter.observe, using the immediately preceding recorded measured
speed and actual applied steering; retain its transported old sample before
averaging in any new observation. Repeat with a singleton so the anchor is not
evicted by the25-sample filter window. Do not use true pose or future outcomes.
Preserve the current recorded selected yaw/speed, nominal target/steering,
vehicle fields, ordered margins, precision, observation age and obstacle map.

carried_diagnostic_xy continues even after invalidation, solely to describe
geometry. gated_carried_xy uses it only while the entire prefix remains valid:
no current joint_pending, scheduled correction, received correction or precision
activation; |measured_speed|<=0.02; observation_age<=1.5; recovery count>=10.
At the first violation invalidate permanently for this anchor; no revival.
Before or at a new pause/reset this is an independent fixed-input comparison,
not an implementable extension of the one-shot policy. Record every reason on
every decision and the first invalid lag, including simultaneous reasons.

Require every recorded stopping, curved-terminal, entry and aligned-release
value to match the retained native trace at full precision and the original
trajectory's9-decimal values. Bind exact selected x/y/yaw/speed, nominal inputs
and both selector-use flags to that trace. Independently check sequential
transport with Python equations. For valid candidate lag1 require carried x/y
to equal the actual handoff pose. Recheck the original entry decision only when
the previous guard is inactive and no joint pause is pending; label static
entry/release thresholds separately from actual trigger/timer behavior.

Export every query variant, every anchored follow-up row, every resume (even
without follow-up), all108 cases,54 pairs and complete summary cells for noise
(all/white/correlated), mode(all/none/random/blackout), arm and outcome
(all/held/timeout), including zero cells. Record both directions of entry and
release flips, both across all window decisions and within original validity.
Report lag1,lag2 and fixed horizons5/25/50, first invalidation and actual trigger
lags, without favorable subgroup selection. Static flips do not establish
future guard timing, causal trajectory effects, safety or hold benefit.

Freeze protocol and inherited inputs before computing new distributions. Commit
implementation and meaningful native/boundary/corruption/end-to-end tests before
querying the retained pilot. Reject changed hashes, incomplete/mixed/reordered
joins, non-finite input, Python-O and existing/internal output directories.
Run twice in fresh external directories; require every exported table, native
source/binary/query/trace/log and provenance byte identical. Keep bulk queries
outside Git, retaining complete manifests and compact all-resume exports.
Run the complete test suite without skips and related CI; merge under standing
authorization. Do not change any preexisting file, policy, default or raw data.
