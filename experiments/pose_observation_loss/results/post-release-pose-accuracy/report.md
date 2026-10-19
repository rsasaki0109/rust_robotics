# Pre-decision position accuracy after joint release

At fixed recorded inputs, continuing the release mean makes position error
WORSE in all13 candidate
lag2 re-guard events. Their selected positions come from epoch fusion. Mean
position error rises from0.0662m to0.1571m (RMSE0.0760m to0.1726m). All13
events, from9 timeout cases, remain in resumes.csv. Repeating the same nominal
preview with aligned physical x/y crosses the entry threshold in12/13 events;
using physical yaw as well gives the same12/13 result.

The previous study established that position substitution removes those13
threshold crossings. The present result shows why that is not evidence for a
longer handoff: it removes12 crossings that also occur in the physical-pose
reference and increases position error in every event. One selected-pose entry
does disagree with both physical references, and is retained below. A reference
threshold crossing is not an actual collision or a proof of an optimal action:
the original nominal command, uncertainty margin and virtual preview still apply.

Keep the handoff generator default false. This study changes no policy, timer,
margin, estimator, command or trajectory and runs no new treatment simulation.

## Frozen population and time alignment

Freeze a5d8f0b starts from main956f4284ad16c53efba225bd8862ccf0641c7aee and
binds209 prior files before computing new error distributions. Implementation
45db5e5 was committed before either evaluation. The retained seed1003 packet
contains108 cases/54 pairs, tau0/1, none/random/blackout,3 maps and3 planners,
with mean_preview and one_step_handoff arms. All251 resumes and their fixed
lag0..50 windows contribute12801 points. There is no truncation or overlap in
this packet. All108 cases remain, including36 no-loss zero-resume controls.
The pilot's held23/54 and timeout31/54 per arm are inherited outcomes.

The speed sidecar writes true_x/y/yaw/v AFTER state.update. Its observations and
nominal commands describe the preceding decision. The aligned pre-state is
therefore the previous sidecar post-state, or the full-precision initial
trajectory checkpoint for step1. Current sidecar truth and rounded previous
trajectory rows must not be substituted. Decision clock n*DT is retained; the
pre-state belongs to the preceding physical checkpoint at(n-1)*DT.

Both evaluations verify all827706 physical transitions: exact measured-speed
predecessors, full-precision post x/y/v and normalized yaw against the original
vehicle dynamics,9-decimal trajectory rounding, ordered case/step/time joins.
An independent streaming check additionally matches every evaluated pre-state
to the preceding full-precision raw sidecar row. Contact, hold, full estimator
history and original controller-gate audits are reused by frozen hashes; those
audits are not claimed as newly rerun here. The137-file raw manifest, both
16-file live audits and both14-file prior position replays all match exactly.

## Comparisons and metrics

Every point receives seven original native geometry queries,89607 in total.
The existing native_guard_release_census backend is reused unchanged, including
original vehicle update, stopping, curved-terminal, entry and aligned-release
functions and ordered margins. All37901 previously evaluated recorded/carried/
gated geometries reproduce at full precision.

The five sensor-only poses are recorded, carried_diagnostic_xy,
gated_carried_xy, base_estimate_xy and current_recovery_xy. Carry uses the
previous replay's sensor-only point, independently rechecked by sequential
transport equations. Gated carry permanently falls back after the first
invalid following decision; lag0 initializes the prefix. Eligibility is this
hypothetical prefix, not actual availability of the consumed one-shot snapshot.
Base uses pre-selector guard x/y. Current recovery uses the current window,
including raw-observation fallback at count0. All five preserve selected yaw.

truth_xy_eval uses aligned physical x/y and selected yaw. truth_pose_eval also
uses physical yaw. These are evaluation references. Neither enters sensor
pose selection, carry, nominal control, estimator history or physical updates.
Speed, target/steering, precision, observation age, map and uncertainty margins
remain the recorded values for every query. Position error is Euclidean;
absolute yaw error wraps through atan2(sin,cos). Closer/farther requires a
difference greater than1e-12m; smaller differences are numerical ties.

Statistics pool window decisions, not independent trials. Case metrics include
lag0..50; within5/25/50 summary scopes exclude lag0. All noise/mode/arm/outcome/
variant cells are exported for lag0/1/2 and those horizons, plus the predeclared
actual-trigger-within50 scope:756 case rows,378 paired rows and3528 summary
cells. Zero-query controls have EMPTY accuracy values, not zero error. Both
directions of entry disagreement against both references and release
disagreement against the full-pose reference are retained. Ready entry counts
require an inactive recorded prior guard and no joint_pending.

## Whole lag2 cohort

The effect is not restricted to the known13 events. Across all lag2 points:

| Arm | Points | Selected position RMSE(m) | Carried position RMSE(m) | Carry closer / farther / tied |
| --- | ---: | ---: | ---: | ---: |
| mean_preview | 125 | 0.2058 | 0.2130 | 37/88/0 |
| one_step_handoff | 126 | 0.2050 | 0.2125 | 37/89/0 |

At lag2 every anchor remains eligible and base_estimate_xy equals recorded.
The gated and unconditional carry positions also coincide. The fusion position
is not claimed to be universally better: many individual carried points are
closer, including8/9 in each correlated-blackout arm, whose RMSE still rises.

| Noise | Loss | Points R/C | Selected RMSE R/C(m) | Carry RMSE R/C(m) | Carry closer/farther R; C |
| --- | --- | ---: | ---: | ---: | ---: |
| white | none | 0/0 | empty | empty | 0/0;0/0 |
| white | random | 53/54 | 0.1890/0.1873 | 0.1944/0.1936 | 6/47;6/48 |
| white | blackout | 9/9 | 0.0936/0.0936 | 0.1385/0.1385 | 0/9;0/9 |
| correlated | none | 0/0 | empty | empty | 0/0;0/0 |
| correlated | random | 54/54 | 0.2384/0.2384 | 0.2418/0.2418 | 23/31;23/31 |
| correlated | blackout | 9/9 | 0.1681/0.1681 | 0.1936/0.1936 | 8/1;8/1 |

R/C denote reference/candidate. Each noise/loss cell retains9 cases per arm,
even when it contributes no resume points. Shared seeds and repeated planner
paths limit independence; numerical differences are descriptive.

## All13 original candidate lag2 entries

Each row below is an event; errors are metres at the lag2 decision. Resume step
identifies its anchor. All selected sources are epoch_fusion, every carried
position is farther from physical truth, and carry removes every entry. The
physical-x/y reference and full-pose reference agree on every entry flag here.

| Tau/loss | Map/planner | Resume step | Selected error | Carry error | Physical reference enters |
| --- | --- | ---: | ---: | ---: | --- |
| 0/random | dense/A* | 2308 | 0.048432 | 0.123192 | yes |
| 0/random | dense/A* | 3808 | 0.055018 | 0.096904 | yes |
| 0/random | dense/JPS | 2308 | 0.048432 | 0.123192 | yes |
| 0/random | dense/JPS | 3808 | 0.055018 | 0.096904 | yes |
| 0/random | dense/Theta* | 2308 | 0.048432 | 0.123192 | yes |
| 0/random | dense/Theta* | 3808 | 0.055018 | 0.096904 | yes |
| 0/random | dense/Theta* | 5558 | 0.009433 | 0.145450 | yes |
| 0/blackout | maze/A* | 4558 | 0.049920 | 0.278676 | no |
| 1/random | dense/A* | 3808 | 0.080813 | 0.165165 | yes |
| 1/random | dense/JPS | 3808 | 0.080813 | 0.165165 | yes |
| 1/random | dense/Theta* | 3808 | 0.080813 | 0.165165 | yes |
| 1/random | maze/JPS | 5558 | 0.068739 | 0.117059 | yes |
| 1/blackout | maze/JPS | 6308 | 0.179133 | 0.345199 | yes |

There are no reference-arm lag2 trigger events. The complete251-row resume
export retains all non-trigger, held and unfavorable events as well. The one
physical-reference disagreement above does not justify selecting that case
for tuning. Carry also worsens error there, from0.0499m to0.2787m.

## Full follow-up, including invalid carry and opposite disagreements

The following table includes every following decision through50. Reference has
6250 rows with493 eligible; candidate has6300 with527 eligible. Eligibility is
identical for all variants on a point. Oracles are not ranked as estimators.

| Arm | Pose | All-row RMSE(m) | Eligible-row RMSE(m) | Closer / farther / tied versus recorded |
| --- | --- | ---: | ---: | ---: |
| reference | recorded | 0.2032 | 0.1762 | 0/0/6250 |
| reference | carried | 0.2129 | 0.2077 | 2233/4017/0 |
| reference | gated | 0.2056 | 0.2077 | 104/389/5757 |
| reference | base | 0.2032 | 0.1762 | 0/0/6250 |
| reference | current recovery | 0.2236 | 0.2079 | 2689/3561/0 |
| candidate | recorded | 0.2029 | 0.1764 | 0/0/6300 |
| candidate | carried | 0.2124 | 0.2066 | 2201/3973/126 |
| candidate | gated | 0.2052 | 0.2066 | 40/361/5899 |
| candidate | base | 0.2028 | 0.1747 | 62/64/6174 |
| candidate | current recovery | 0.2234 | 0.2067 | 2627/3547/126 |

Among all31 actual trigger rows per arm through50, selected-position RMSE is
0.1524/0.1471m versus carried0.2041/0.2018m. Carry is closer in4 and farther in27
rows in EACH arm. The full-pose reference enters in30/31 rows per arm.
Unconditional carry misses23 of those reference entry thresholds per arm;
gated carry misses15/17. These are geometry disagreements under fixed inputs,
not observed contacts, releases or counterfactual hold outcomes.

The whole follow-up also contains opposite entry disagreement. Recorded
crosses where the physical reference does not in7/6 rows, while not crossing
where the reference does in1361/1422. Carry yields0/0 and1796/1846 respectively;
gated carry yields0/0 and1549/1619. Counts include already active guard rows.
All ready counts, release disagreements, signed errors, maxima, outcomes,
prefix-validity counts and zero-target rows remain in the full exports.

## Interpretation and next step

Position continuity alone is an inadequate objective. In the13 events that
motivated this study, the switch to fusion is a switch to a more accurate
position, and12 original entries also occur under the physical reference.
The frozen results do not support extending the handoff to suppress re-guard.
They do not establish that every entry is necessary, that fusion always wins,
or that either physical-reference preview predicts a collision.

A useful next bounded diagnosis is to decompose stopping versus curved-terminal
clearance at the already recorded physical-reference entry rows, while retaining
all outcomes and non-entry controls. Determine what constrains nominal-command
geometry before considering a sensor-only path or steering change. Freeze that
question separately; do not tune duration, margin or a special-case rule from
these13 events. Truth remains evaluation-only in any subsequent work.

## Validation and reproducibility

All1463 tests pass, with zero failures, errors or skips. The12 added tests cover
pre/post-state alignment, exact predecessors, dynamics/time/rounding corruption,
wrapped yaw, truth isolation, numerical ties and empty cells, complete native
exports, both favorable/unfavorable errors and disagreement directions, expired
prefixes, zero targets, full case collection, frozen inputs and Python-O.
There were no failed test attempts, query failures or audit amendments.

Both fresh evaluations produce13 byte-identical files, including all tables,
native source/binary/query/trace/logs and provenance. repeat-verification.json
contains every size/hash; validation.json records conservation and independent
checks. Compact case/pair/summary/verification/resume tables, sources and the
exact native source are tracked. Bulk samples.csv (77,172,398 bytes), native
queries.tsv (24,613,957 bytes), trace.csv (25,845,305 bytes) and binary remain
external and are covered by the repeat manifest. The137-file raw packet and
all four prior audited directories are unchanged. A checkout alone lacks them.

From a clean checkout with the retained external evidence:

```sh
source /workspace/.cloud-env/rust_robotics/activate.sh
PYTHONDONTWRITEBYTECODE=1 /workspace/.cloud-env/rust_robotics/python-ci/bin/python -B \
  experiments/pose_observation_loss/analyze_release_pose_accuracy.py \
  --raw /workspace/scratch/post-release-mean-handoff-live-pilot/raw \
  --live-one /workspace/scratch/post-release-mean-handoff-live-pilot/audit-2 \
  --live-two /workspace/scratch/post-release-mean-handoff-live-pilot/audit-3 \
  --replay-one /workspace/scratch/post-handoff-transition-replay/query-1 \
  --replay-two /workspace/scratch/post-handoff-transition-replay/query-2 \
  --output /workspace/scratch/pose-accuracy-fresh-1
```

Repeat with another fresh external output and compare all13 files. Inputs must
match frozen manifests. Later commits record their own implementation commit
in sources.json, so that file differs from this archive; two executions at the
same later commit must still agree. Python-O, changed hashes, uncommitted code,
mixed/reordered joins, nonfinite inputs and existing/internal outputs fail.
