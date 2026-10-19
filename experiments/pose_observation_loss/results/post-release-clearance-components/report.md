# Stopping and terminal clearance components after joint release

All13 original candidate lag2 re-guards are caused by the terminal component
alone at their recorded pose: the stopping bound exceeds the entry margin by
0.0410..0.3983m while the reserved terminal score falls below it by
0.0098..0.1166m. At the aligned physical-pose reference,12/13 still cross:
9 are terminal-only and3 cross both stopping and terminal thresholds. The
terminal component binds in all13, including the one non-crossing reference.

At the SAME frozen bounds, the recorded13 entries comprise12 requiring both
the uncertainty/age allowance and terminal entry reserve, and1 for which the
allowance alone suffices. The12 physical-reference entries comprise8 jointly
required and4 allowance-only. None crosses the raw0.5m radius-level check.
These are properties of fixed scalar predictions, not observations of collision
or evidence that a smaller margin or longer handoff would be a better policy.
The prior finding that carry worsens position error in all13 remains unchanged.

## Frozen evidence and definitions

Protocol freeze de3fab8 starts at main262868eaba4f2e1725be198a78a148448b8555a5, binds223 inherited files,
and precedes the new component distributions. Implementation
ba2b59ff730bbce01bb3ead28d337788b967c165 was committed before either annotation.
Both complete13-file pose-accuracy evaluations match the previous repeat
manifest byte counts and SHA256. Their full physical alignment,827706 dynamics
checks, estimator/contact/hold audits and89607 original native evaluations are
inherited by those hashes; none is claimed as newly executed for this study.
Only the small unit-test witnesses call native geometry again.

The seed1003 packet retains108 cases/54 pairs,251 resumes,12801 lag0..50 points
and89607 seven-variant rows. It includes tau0/1, none/random/blackout,3 maps,
3 planners and both mean_preview/reference and one_step_handoff/candidate arms.
The inherited outcomes are23 held and31 timeout per54 cases in each arm.
All36 no-loss zero-resume controls, zero targets, already-active guards and
invalid hypothetical carry prefixes remain. The fixed50 following decisions
exclude lag0 in within scopes; endpoint clipping and overlapping anchors remain
supported, although this packet has neither truncated nor overlapping windows.

Let S be the original stopping lower bound, T the curved-terminal clearance,
B the recorded preview margin, G the release margin, and R=0.5m. Validate exact
full-precision identities E=min(S,T-0.1), L=min(S,T-0.05), G=B+0.05 and the
ordered expression B=0.5+0+2*sqrt(2)*effective_sigma+0.02*observation_age.
Entry uses E<=B; release geometry is clear only for L>G. Use B directly rather
than recovering it from G-0.05, and preserve subtraction order at boundaries.
Stop-only/terminal-only/both/clear classify the two component predicates.
Binding compares S with T-0.1, with1e-12m tolerance for tie labels only.

The allowance labels use four checks on the SAME already computed S/T:

| Layer | Fixed-score condition |
| --- | --- |
| radius_level | min(S,T)<=R |
| margin_only | Entry; raw minimum<=B, but E>R and raw minimum>R |
| reserve_only | Entry; E<=R, but raw minimum>B |
| either_sufficient | Entry; both raw minimum<=B and E<=R, but raw minimum>R |
| jointly_required | Entry; raw minimum>B and E>R |
| clear | E>B |

S depends on B in its conservative far-obstacle shortcut. Checking its frozen
value against R does not recompute geometry for a policy using margin R.
T evaluates a virtual0.6m fixed-steering arc at the stopping endpoint AFTER
0.2s nominal motion and bounded braking; it is not an observed swept path.
Truth references retain the recorded commands, speed, uncertainty and margins,
and remain evaluation-only. No command, timer, margin, estimator, default or
trajectory changes, and no new treatment simulation, are included.

## Whole lag2 cohort

| Arm | Variant | Points | Terminal-only / both / clear | Margin-only / jointly-required | Ready entries | Release geometry clear |
| --- | --- | --- | --- | --- | --- | --- |
| reference | recorded | 125 | 12/0/113 | 1/11 | 0 | 113 |
| reference | carried_diagnostic_xy | 125 | 0/0/125 | 0/0 | 0 | 125 |
| reference | gated_carried_xy | 125 | 0/0/125 | 0/0 | 0 | 125 |
| reference | base_estimate_xy | 125 | 12/0/113 | 1/11 | 0 | 113 |
| reference | current_recovery_xy | 125 | 0/0/125 | 0/0 | 0 | 125 |
| reference | truth_xy_eval | 125 | 29/6/90 | 8/27 | 24 | 90 |
| reference | truth_pose_eval | 125 | 28/6/91 | 8/26 | 23 | 91 |
| candidate | recorded | 126 | 13/0/113 | 1/12 | 13 | 113 |
| candidate | carried_diagnostic_xy | 126 | 0/0/126 | 0/0 | 0 | 126 |
| candidate | gated_carried_xy | 126 | 0/0/126 | 0/0 | 0 | 126 |
| candidate | base_estimate_xy | 126 | 13/0/113 | 1/12 | 13 | 113 |
| candidate | current_recovery_xy | 126 | 0/0/126 | 0/0 | 0 | 126 |
| candidate | truth_xy_eval | 126 | 30/6/90 | 8/28 | 36 | 90 |
| candidate | truth_pose_eval | 126 | 29/6/91 | 8/27 | 35 | 91 |

There are no stop-only entries in this cohort, and the terminal score binds at
every lag2 point across all seven variants. Radius-level, reserve-only and
either-sufficient counts are zero. All251 lag2 points are prefix-eligible;
candidate has126 ready rows, reference113. The12 reference recorded crossings
already have an active guard from lag1 and are not new triggers at lag2.
The complete candidate cohort includes35 physical-pose crossings, beyond the
12 in the original13-trigger subset. The non-trigger controls are retained.

## All13 original candidate lag2 trigger events

All rows are timeout events from9 cases. Every recorded mechanism is
terminal_only and every binding label, recorded or truth, is terminal.
Recorded layers are jointly_required except the final correlated-blackout row,
which is margin_only. B-R=0.30684271247461914m in every row; E-B equals the
reserved terminal gap. Signed gaps are metres; positive means above threshold.

| Tau/loss | Map/planner | Resume step | Recorded S-B / E-B (m) | Truth S-B / E-B (m) | Truth mechanism / layer | First truth entry lag |
| --- | --- | --- | --- | --- | --- | --- |
| 0/random | dense/A* | 2308 | 0.397159/-0.027977 | 0.367174/-0.071387 | terminal_only/jointly_required | 1 |
| 0/random | dense/A* | 3808 | 0.069660/-0.074202 | 0.122488/-0.014679 | terminal_only/jointly_required | 1 |
| 0/random | dense/JPS | 2308 | 0.397159/-0.027977 | 0.367174/-0.071387 | terminal_only/jointly_required | 1 |
| 0/random | dense/JPS | 3808 | 0.069660/-0.074202 | 0.122488/-0.014679 | terminal_only/jointly_required | 1 |
| 0/random | dense/Theta* | 2308 | 0.398347/-0.026454 | 0.368258/-0.070024 | terminal_only/jointly_required | 1 |
| 0/random | dense/Theta* | 3808 | 0.051388/-0.064100 | 0.101028/-0.006614 | terminal_only/jointly_required | 1 |
| 0/random | dense/Theta* | 5558 | 0.066984/-0.009770 | 0.066471/-0.009333 | terminal_only/jointly_required | 1 |
| 0/blackout | maze/A* | 4558 | 0.178873/-0.023030 | 0.223640/0.003130 | clear/clear | 3 |
| 1/random | dense/A* | 3808 | 0.048451/-0.048088 | -0.014845/-0.112571 | both/margin_only | 1 |
| 1/random | dense/JPS | 3808 | 0.048451/-0.048088 | -0.014845/-0.112571 | both/margin_only | 1 |
| 1/random | dense/Theta* | 3808 | 0.041045/-0.056255 | -0.020861/-0.119431 | both/margin_only | 1 |
| 1/random | maze/JPS | 5558 | 0.055338/-0.053176 | 0.072770/-0.043790 | terminal_only/jointly_required | 1 |
| 1/blackout | maze/JPS | 6308 | 0.242553/-0.116598 | 0.085882/-0.210952 | terminal_only/margin_only | 1 |

The white-blackout maze/A* event is the sole physical-reference non-entry at
lag2, with0.003130m combined entry gap; its same frozen follow-up first crosses
at lag3. All other12 physical references first cross at lag1, before the
recorded lag2 trigger. Repeated planner paths and shared seed create correlated
events, so13 rows do not constitute13 independent trials. No row is removed
on outcome, target sign, prefix validity or unfavorable reference disagreement.

## Full follow-up and all original triggers

The complete following50 decisions contain6250/6300 rows in the reference/
candidate arms. Their prefix-eligible counts are493/527; ready counts5199/5267;
zero-target counts304/344. Eligibility and readiness are inherited flags, not
newly granted availability of the consumed one-shot handoff. Ready entry
requires an inactive recorded prior guard and no joint pause.

| Arm | Pose | Rows | Terminal-only / both / clear | Margin-only / jointly-required | Ready entries | Release geometry clear |
| --- | --- | --- | --- | --- | --- | --- |
| reference | recorded | 6250 | 501/0/5749 | 21/480 | 31 | 5749 |
| reference | truth_pose_eval | 6250 | 1546/309/4395 | 588/1267 | 901 | 4395 |
| candidate | recorded | 6300 | 483/0/5817 | 17/466 | 31 | 5817 |
| candidate | truth_pose_eval | 6300 | 1590/309/4401 | 588/1311 | 962 | 4401 |

Across all89607 annotated rows, including lag0 and all seven variants,11212
cross the entry threshold:9841 terminal-only and1371 both, with no stop-only
entry. Their layers are2490 margin-only and8722 jointly-required. The remaining
78395 rows are clear. No row has radius-level, reserve-only or
either-sufficient classification. Binding labels need not imply an entry:
some clear following rows bind on stopping, and all such controls remain.

All original actual-trigger rows within50 are pooled below. There are31 rows
per arm, spanning30 distinct first-trigger windows per arm; multiple triggers
within a window remain separate. All are ready;3/4 have zero nominal target.

| Arm | Pose | Rows | Terminal-only / both / clear | Margin-only / jointly-required | Stop/terminal/combined mean entry gap (m) |
| --- | --- | --- | --- | --- | --- |
| reference | recorded | 31 | 31/0/0 | 1/30 | 0.225017/-0.029263/-0.029263 |
| reference | truth_pose_eval | 31 | 24/6/1 | 14/16 | 0.157747/-0.092386/-0.092386 |
| candidate | recorded | 31 | 31/0/0 | 1/30 | 0.219456/-0.029451/-0.029451 |
| candidate | truth_pose_eval | 31 | 24/6/1 | 14/16 | 0.156593/-0.091715/-0.091715 |

All31 recorded triggers per arm are terminal-only:30 jointly-required and1
margin-only. Physical references cross30/31 per arm:24 terminal-only and6 both,
with14 margin-only and16 jointly-required. Their lone clear row is retained.
Release blocking is separately exported: at recorded triggers it is terminal-
only28/28 and both3/3, even though no stopping component causes entry.
Geometric release-clear counts do not apply the stationary0.5s release timer.

## All noise, loss and outcome groups

The following table reports physical-pose reference entries through50. R/C
denote reference/candidate. Each noise/loss cell retains9 cases per arm.

| Noise | Loss | Resumes R/C | Truth entries R/C | Truth terminal-only R/C | Truth both R/C | Margin-only R/C | Jointly-required R/C |
| --- | --- | --- | --- | --- | --- | --- | --- |
| white | none | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| white | random | 53/54 | 813/857 | 813/857 | 0/0 | 130/130 | 683/727 |
| white | blackout | 9/9 | 192/192 | 192/192 | 0/0 | 10/10 | 182/182 |
| correlated | none | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| correlated | random | 54/54 | 650/650 | 341/341 | 309/309 | 398/398 | 252/252 |
| correlated | blackout | 9/9 | 200/200 | 200/200 | 0/0 | 50/50 | 150/150 |

No-loss groups have zero rows and EMPTY gap statistics, not perfect clearance
or zero error. Held and timeout groups remain separate:

| Arm | Outcome | Cases | Resumes | Rows | Truth entries | Terminal-only / both | Margin-only / jointly-required |
| --- | --- | --- | --- | --- | --- | --- | --- |
| reference | held | 23 | 70 | 3500 | 0 | 0/0 | 0/0 |
| reference | timeout | 31 | 55 | 2750 | 1855 | 1546/309 | 588/1267 |
| candidate | held | 23 | 70 | 3500 | 0 | 0/0 | 0/0 |
| candidate | timeout | 31 | 56 | 2800 | 1899 | 1590/309 | 588/1311 |

All physical-reference following entries occur in inherited timeout cases in
this packet. That association does not establish a new treatment effect.
The complete resume census, including no physical-reference entry, is:

| Arm | Resumes | First truth entry at lag1 / lag2 / lag3 / lag6 / lag27 / none | Actual first trigger within50 |
| --- | --- | --- | --- |
| reference | 125 | 37/0/3/1/1/83 | 30 |
| candidate | 126 | 37/1/3/1/1/83 | 30 |

Case tables retain lag0..50 per variant (756 rows); pairs retain both arms
(378 rows);3528 summary cells cover every noise/mode/arm/outcome/variant cell
for lag0/1/2,within5/25/50 and original triggers_within50. All count partitions,
ready/eligible/zero-target counts, entry/release gaps, mean/min/max entry gap
and mean allowance are retained, including empty cells and unfavorable groups.

## Interpretation and next bounded question

The original lag2 re-guards are terminal-preview constraints rather than
stopping-only constraints. Their physical-reference counterparts generally
persist and can additionally cross the stopping threshold. The fixed-score
layers show where the current allowance and reserve enter the classification;
they do not justify shrinking either. Retain the existing guard, handoff and
uncertainty settings. No claim of optimality, collision prevention or contact
improvement follows from these scalar annotations.

A useful next diagnosis is to attribute the limiting terminal-arc clearance to
its wall/obstacle and location along the arc, using the same complete frozen
population before selecting any steering alternative. This requires separate
native geometric evidence; it has not been run here. Do not retune the13 known
events or feed physical truth into an operational controller.

## Verification and reproduction

All1476 tests pass, with zero failures, errors or skips. The13 added tests cover
all six layers and four component outcomes, inclusive entry/strict release,
exact boundary neighbours and a real G-0.05 rounding counterexample, original
Rust native witnesses, complete case/pair/summary matrices and zero controls,
fresh repeat exports, first-entry/lag2 joins, empty endpoint follow-up,
corrupt scalar/unused numeric/boolean fields, missing/mixed/reordered/truncated
windows, frozen inputs, clean Git, external fresh output and Python-O rejection.

Both fresh annotations succeed on their first attempt and produce7 byte-
identical files including provenance. Independent checks rejoin every89607
row to the parent samples and recompute the component, binding, four-check
truth table and all gaps. They conserve every case/pair/summary count and
statistic and all251 resume records, including their first entries and lag2
fields. There were no test failures, annotation failures or audit amendments.

Track compact cases/pairs/summary/verification/resumes tables, sources.json,
repeat-verification.json and validation.json. Bulk annotations.csv remains
external with its complete size/hash. Both parent13-file evaluations remain
external and are bound by the parent manifest. A checkout alone lacks them.
Source manifests retain the original physical/raw/native audit chain without
claiming to rerun it. No preexisting tracked file changes.

From a clean checkout with both retained parent evaluations:

```sh
source /workspace/.cloud-env/rust_robotics/activate.sh
PYTHONDONTWRITEBYTECODE=1 /workspace/.cloud-env/rust_robotics/python-ci/bin/python -B \
  experiments/pose_observation_loss/analyze_clearance_components.py \
  --input-one /workspace/scratch/post-release-pose-accuracy/evaluation-1 \
  --input-two /workspace/scratch/post-release-pose-accuracy/evaluation-2 \
  --output /workspace/scratch/clearance-components-fresh-1
```

Repeat with another fresh external output and compare all7 files. At the same
commit they must match byte-for-byte. A later commit records its own hash in
sources.json, so comparison to this archive must account for that provenance
change. Changed input hashes, nonfinite data, incorrect joins, uncommitted code,
existing/internal output directories and Python-O fail without overwriting
earlier artifacts. Related CI must pass for the exact final data commit before
merge and the exact main merge commit after push; CI identities are retained
externally to avoid changing the validated tree.
