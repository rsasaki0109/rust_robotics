# Fixed position/heading decomposition of commanded recovery-arc decisions

Start from main e55b0f3 and the pinned commanded-recovery-arc accounting,
geometry, decisions, transitions and provenance. Retain all 648 cases, all 368
timeouts, exactly the same 2,120 checkpoints, both actual recovery and preview
inputs, and both straight and commanded .6 m recovery paths. Preserve the
overlapping all / first-usable-epoch / age-.5 / endpoint subsets. Do not select
by new hybrid geometry, error size or movement. Highlight the previously defined
optimistic extra arc permissions (43 age-.5 points for actual recovery input),
with the full input/arm denominators and all other decisions retained.

For each input use four poses: FF=estimated xy/yaw; TF=true xy/estimated yaw;
FT=estimated xy/true yaw; TT=full truth. Evaluate the original line and the
recorded nominal-steering arc at each pose. Steering is fixed, not recomputed
from true observations or optimized. This compares zero steering with the
recorded steering's path shape; it does not isolate steering estimation error.
Do not change sigma, observation age, speed commands or M=.6+2sqrt(2)*sigma+.02*age.
Retain strict obstacle clearance and inclusive map-wall equality separately.

Reproduce every prior FF/TT decision and distance, and the existing TF geometry
for actual recovery input. Classify each optimistic permission and conservative
block by which truth substitution changes the decision toward TT: position only,
heading only, either alone, or both required. This classification must work in
both mismatch directions and use the recovery predicate, not scalar slack>0.
Do not classify matching FF/TT decisions as a mismatch.

Split the signed distance overstatement FF-TT symmetrically over both orders:
position=((FF-TF)+(FT-TT))/2, heading=((FF-FT)+(TF-TT))/2. Keep negative terms;
verify their sum. For each paired path compare estimated arc-minus-line distance
with its true counterpart. Their difference equals the change in FF-TT, and also
the sum of changes in the two signed contributions. These are descriptive
coordinate/path substitutions, not additive causal error sources or probabilities.

Audit every distinct hybrid pose/steering geometry with exact point-obstacle and
interior-wall calculations, the existing combined-minimum helper and independent
120-chord integration with the sagitta bound. Cache geometry only; evaluate each
row's margin separately. Record classification ambiguity within 2e-8 m. Pin all
consumed tracked tables, protocol, map, simulator and helpers. The original raw
physics/filter/timer validation is inherited through the prior source chain;
do not reload raw trajectories or claim new physical validation.

Export full accounting, per-decision components, unique geometry audits linked
by IDs, paired path effects, cohort/arm/input/path/subset/group summaries and
source fingerprints. Include all, optimistic, conservative and fixed optimistic
extra-arc-permission groups, explicitly handling empty groups. Repeat byte for
byte. If plotted, use all age-.5 optimistic and conservative classifications
for both paths, inputs and arms. Keep previous reports and control defaults.
Do not infer a live one-second confirmation, recovery, contact or held-goal result.
