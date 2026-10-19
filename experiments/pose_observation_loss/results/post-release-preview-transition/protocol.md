# Fixed-input query of the decision after joint recovery release

Start from main 67398ea077dd2a7036291ae3f345a2d3d511f439. This retrospective
diagnostic follows the seed1002 mean-guard-preview pilot and complete release
census. Known results: both arms held26/54 and timed out28/54; the reference
had67 joint resumes and the mean-preview arm70. Candidate timeout endpoints
had27 active guards. This is not a new live controller, simulation, seed or
causal estimate of successful arrival.

Freeze this protocol and hashes before extracting new transition or flip
distributions; commit implementation and meaningful tests before querying them.
Bind all inherited frozen inputs, the complete previous census, its scripts,
the raw provenance b9476bc0796a097bb9f80156d52694b0e32fbde56ee1cec06e7be29e926298fd
and all111 raw files. Verify the entire retained census output against its
14-file repeat manifest, including all799688 native input and geometry rows.
Reuse its complete timer, physics and estimator audit as bound evidence.
Keep all old files, raw logs, simulator sources and defaults unchanged.

Retain all108 cases/54 pairs and every joint resume, including held cases,
timeouts and no-loss controls with zero resumes. For each resume, select the
resume decision and exactly the immediately following decision in that case.
Retain a resume at a packet endpoint explicitly with no invented next row.
Do not select transitions by re-entry, geometry, outcome or changed position.
Require the raw resumed decisions to equal the complete census event set.

At the resume decision, query its actual selected pose once (recorded). At
each available following decision, query exactly three position variants:

- recorded: the actual full-precision selected preview pose.
- current_recovery_xy: replace only x/y with that decision's logged recovery
  position. This is an unchanged retained window when available; an empty
  window falls back to the current raw sensor pose. Retain and label that
  fallback, never present it as a fresh mean.
- carried_release_mean_xy: transport the preceding resume's recovery mean
  by one original MotionPoseFilter transport step, using that resume's
  measured predecessor speed and actual applied steering. Use a singleton
  history and the original observe implementation, read the transported old
  sample before the new sample, and replace only next-decision x/y. Preserve
  the actual next yaw. This diagnostic intentionally adds no new sensor sample
  to the carried position. Record any next correction, precision reset,
  motion, stale observation or new joint pause; do not silently discard or
  reset these witnesses. It is not an eligible live admission policy.

For every variant hold the queried decision's measured speed, yaw, nominal
target, nominal steering, observation age, precision state, map and ordered
margins fixed. Use original Rust stopping, curved terminal and aligned-release
functions, strict source anchors and the original ordered base/release margin
expressions. Preserve the original motion update and buffers. Use full f64
values for entry <= base and release > release_margin, not rounded logs.
Require both recorded decisions to match all full native census geometry and
the original nine-decimal logs. Match selected baseline input rows exactly to
the retained all-decision census queries, including global sequence IDs.
All joint resume gates must pass; the guard is inactive after each resume.
Consequently recorded next entry must equal its actual next guard-trigger flag.

Export every transition, every variant query, all108 case rows, all54 pairs,
verification counts and all72 noise(all/white/correlated), mode(all/none/random/
blackout), arm and outcome(all/held/timeout) summary cells, including empties.
Report position jumps and changes in target, steering, speed, yaw, precision,
window state and fusion availability. Retain both directions of entry and
release flips; distinguish immediate actual re-guard from hypothetical entry
clearance and from continuous timer admission. Counts after arm divergence
are descriptive; repeated planner paths are correlated, and these sensor-only
geometry queries do not measure true clearance or future success. Do not
infer that a window may safely persist or that any gate may be removed.

Reject reordered/mixed cases, missing/extra rows, nonfinite inputs, changed
hashes, Python-O and existing/internal output paths. Retain failure diagnostics
and use fresh output after any correction. Run twice into fresh external paths;
require all exports, native source, binary, input, trace, logs and provenance
byte identical. Track compact exports, complete native source, provenance,
repeat proof, validation, report and exact re-execution instructions; keep bulk
retained census/raw data outside Git. Test original transport with curved and
negative-speed witnesses, variant field preservation, native baseline geometry,
inclusive entry and strict release boundaries, endpoint retention, both flip
directions and corrupt inputs. Run the complete suite without skips and related
CI before merging under the user's standing authorization.
