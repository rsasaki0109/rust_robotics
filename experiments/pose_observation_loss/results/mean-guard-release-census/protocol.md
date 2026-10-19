# Fixed retrospective census of both release timers

Start from main 3df29b57d9b14332512b172d4a3709fda19e6553. Known design
information is the seed1002 mean-guard-preview pilot: both arms held26/54,
both timed out28/54, zero contacts/boundary exits, 24 changed original logs
and nine changed endpoints. This is a retrospective diagnostic of that same
packet, not a new treatment, seed, simulation or causal success estimate.

Bind the entire previous study, its implementation, original simulator,
dependencies, fixtures and inherited diagnostic functions by hashes. Bind raw
provenance b9476bc0796a097bb9f80156d52694b0e32fbde56ee1cec06e7be29e926298fd
and its complete 111-file manifest. Retain all108 cases/54 pairs and all799688
decisions, including all no-loss controls, successful holds and timeouts.
Keep raw trajectories, binaries, previous files and all controller defaults
unchanged. Commit this protocol before extracting new gate distributions;
commit implementation and meaningful tests before collecting those results.

Re-evaluate the original recovery-line geometry and stopping/terminal/aligned
release geometry at EVERY decision, from full-precision sidecar inputs. Use
the actual selected guard pose, measured predecessor speed, nominal target and
steering, current recovery mean/yaw, observation age and precision epoch.
Extract the original functions and ordered recovery/base-preview/release
margin expressions with strict source anchors. Do not derive base margin by
subtracting hysteresis, change buffers or omit zero-target decisions.
Require native nine-decimal recovery/stop/terminal/entry/release values and
margins to match original logs, exact speed margin to match the full sidecar,
and full native values for strict inequalities. Leave no interval-ambiguous
gate or unverified timer decision. Preserve every query, trace and source.

Independently reconstruct the exact branch order of the guard and joint
timers, including guard entry, the skipped guard update on the entry decision,
joint-pending resets, atomic joint/guard release and reset-after-release.
Joint admission requires all seven original gates: fresh observation,
stationary speed, sample count, recovery-line clearance, recovery bounds,
positive nominal target and aligned-release clearance for1s (50 steps).
Ordinary guard release requires stationary speed and aligned-release
clearance for0.5s (25 steps); it has no positive-target requirement.
Use the original <= inclusivity, strict > clearances and +1e-9 timer tolerance.
Require every native triggered/released/active/paused/resumed flag and both
recorded timers to match. Bind pause/resume counts to original outcomes.

Export all cases, all54 pairs, all readiness runs and reset masks for both
timers, all trigger/release/resume events and all per-case simultaneous gate
patterns. Preserve joint-pending/all and joint-eligible phases (fresh,
stationary and >=10 samples), plus guard-only updates. Partition eligible
joint zero targets into goal-latched and unlatched speed-probe-zero cases.
Do not reinterpret latched zero commands as failed arrivals. Retain all
joint runs, including interruptions and packet endpoints, and guard runs,
including interruptions by a joint pause or guard inactivity.

Summarize every noise(all/white/correlated), mode(all/none/random/blackout),
arm(fused_preview/mean_preview) and outcome(all/held/timeout) cell, including
empty cells:72 cells. Report timeout endpoints and never-ready/subthreshold/
admitted joint histories alongside guard-only behavior. Counts from diverged
closed-loop histories and repeated planner paths are descriptions, not
independent trials or same-time treatment effects. Report gains and losses,
all masks and controls without choosing only favorable cases. Do not infer
that any condition may be removed from a blocker count.

Reuse the previous complete physics/estimator audit only as bound evidence:
verify all raw hashes and prior output hashes; this diagnostic adds full
geometry and timer coverage without rerunning the simulator. Reject Python-O,
existing/internal output paths, changed inputs and mixed/reordered cases.
Retain failure diagnostics and use a fresh output after any correction.
Run the complete census twice into fresh external paths; require every
table, query, native trace, source, binary, log and provenance byte identical.
Track compact results, complete native source, provenance, repeat proof,
re-execution instructions and report. Keep bulk queries/traces/binaries outside
Git. Test inclusive boundaries,49/50 and24/25 steps, timer interruptions,
both native geometries and corrupted inputs; run the full suite without skips
and related CI before merging under standing authorization.
