# Correct the guard-margin reconstruction

The first diagnostic from implementation cd2deaf82fd347d679fb7f9478ae55b39cda7bce
stopped at the first paused query, before writing any gate-distribution tables
or completion manifest. Retain that failed output. Its native recovery line
matched, but its assumed guard-release margin did not.

The original protocol incorrectly called the recovery and base guard margins
identical in the curved configuration. The frozen commands have argument 21
(recovery_guard_buffer) false. Recovery and speed margins include +0.1;
the base guard margin does not. The guard still adds its original +0.05 release
hysteresis. At query 1 (tau0/raw_probe-random-1001, dense/A*, step 3000), the
speed/recovery margin is 0.982842712474618962 and the recorded guard-release
margin is 0.932842712. The first implementation wrongly produced 1.032842712.

Replace that assumption with reconstruction of the original separate formulas.
Extract the original effective-position-sigma, recovery-margin, base-preview-
margin and guard-release-margin expressions verbatim into the native reader.
Pass full observation age and cumulative precision activation from the sidecar,
and position sigma, drift speed and recovery_guard_buffer from the already
frozen command arguments. Effective sigma is the original literal 0.03 in
precision mode, otherwise the configured position sigma. Check recomputed
recovery margin exactly against the full-precision sidecar and check the
formatted sigma and guard-release margin against the original trajectory.
Do not recover the guard margin by subtracting 0.1 from a rounded value.

For endpoint-only annotations outside the paused query population, use both
logged clearance and logged guard-margin rounding intervals; mark an overlap
unresolved. Paused joint admission continues to compare the release-clearance
interval with the full-precision native guard-release margin.

All 108 cases, 54 pairs, decisions, seven gates, thresholds, continuity rules,
failure masks and reporting units remain as frozen. This corrects a diagnostic
implementation defect and changes no controller or stored observation. Commit
the correction and tests before repeating the diagnostic in fresh output paths.
Retain the initial protocol/pins and this amendment with its separate pins.
