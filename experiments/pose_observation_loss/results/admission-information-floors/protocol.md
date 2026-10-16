# Fixed-centre information floors for the unchanged recovery geometry

Start from main 926c8ec. Preserve all 648 accounting cases, 368 timeouts,
2,120 checkpoints, actual/preview inputs and straight/commanded .6 m paths.
Use all 8,480 noise_anchor_envelope rows of model-admission-envelopes and
join the existing epoch-position-components covariance rows. No selection
by truth, movement, error, availability or decision result. Retain every
cohort, arm, estimator/fallback, subset and fixed optimistic-arc-gain group.

Keep the estimated centre, yaw, recorded steering, heading interval +/-4 deg
and the full original true/estimated margin M unchanged. Compare conditional
information requirements at those centres; do not simulate future estimates,
actual observation waits, control changes or new held-goal outcomes.

Use only the previously exported per-axis noise and shared-anchor variances
Vn and Va. Define C=2sqrt(2). Compare exactly two ideal batch strategies:
- same_anchors: radius C*sqrt(Va+Vn/K); existing shared anchor weights persist.
- independent_anchors: radius C*sqrt((Va+Vn)/K); K independent copies of the
  entire current estimator batch, including its anchor variables.
Use K=1,3,10,25 and the mathematical K->infinity limit (C*sqrt(Va) or zero).
K is a batch count, not the observation count or number of new corrections.
For a three-epoch estimate, an independent batch also needs independent
copies of its three anchor contributions. Assume independent noise between
batches while preserving each batch's already modelled temporal correlation.
This independence is an optimistic information assumption, not a forecast
for adjacent samples of the correlated process or overlapping epoch windows.
The limit can be unattainable at any finite K, notably at an inclusive wall
threshold. Never infer a finite successful K solely from a successful limit.

Reuse all nine previously audited heading samples -4..4. Let do and dw be
the minimum obstacle/wall distances over these samples, kept separate.
Let E=2*.6*sin(radians(.5)/2). The true continuous-heading minima lie in
[do-E,do] and [dw-E,dw]. Record both sufficient and necessary radius caps.
For any nonnegative radius r classify:
- certified_clear if do-E-r>M and dw-E-r>=M;
- some_pose_blocks if do-r<=M or dw-r<M;
- angle_gap otherwise, keeping the <=E interpolation gap explicit.
These states describe robust permission of the entire disk, not the previous
all-permitted/all-blocked/unresolved admission states. some_pose_blocks
does not mean every pose is blocked. Moving the whole path toward a nearest
point obstacle attains max(0,d-r); moving toward a wall reduces its minimum
by r. Thus a sampled-heading violation witnesses the failure to permit the
entire disk, regardless of finer heading interpolation. Preserve strict
obstacle clearance and inclusive wall equality. Test these geometric facts.

K=1 for both strategies must reproduce the prior certified-permit mask and
radius, with noise-only and zero-noise anchor limits checking the variance
decomposition. Radii cannot increase with K; independent_anchors cannot
exceed same_anchors. Definite permission and known blocking-pose statuses
must be monotone in the corresponding direction as radius decreases.

Classify each base row, in this precedence order: already_certified;
fixed_centre_not_clear (original central gate fails); heading_interval_blocks
(r=0 has a blocking sampled heading); heading_grid_gap (r=0 angle gap);
shared_anchor_floor (same-anchor limit has a blocking pose);
anchor_floor_grid_gap; noise_limit_certifies. The last label describes an
ideal limit only. Verify this is an exhaustive partition without using truth.

Only after each counterfactual classification attach recorded truth and pose
coverage. Label these evaluations as fixed-centre comparisons: shrinking a
radius while leaving its recorded error unchanged is not a measured change
in real error or a calibrated confidence statement. Keep model omissions of
drift/transport and the unbounded Gaussian tails explicit. Report false
fixed-centre permits and uncovered recorded poses as well as additional
certified permits. Do not interpret some_pose_blocks/angle_gap as actions.

Pin input/protocol/source/helper/map/simulator hashes. Inherit raw-window,
physics and heading-geometry audits; no new geometry or raw trajectories are
required. Export accounting, base checkpoints/caps/bottlenecks, compact
strategy curves linked by base row, subgroup summaries and provenance.
Repeat byte for byte. If plotted show all age-.5 base rows for both inputs,
paths and recorded arms, including every bottleneck or non-permitted state.
Do not tune batch counts, geometry, margins or categories to the results.
