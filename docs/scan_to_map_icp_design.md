# Scan-to-Map ICP Design

Status: design draft, no code merged yet.
Audience: next agent picking up corrected-SLAM work on `dev/corrected-slam-eval`.
Companion to `docs/corrected_slam_evaluation_plan.md`.

## Why this exists

The 2026-05-17/18 tuning sprint established a structural ceiling
for the current scan-to-scan ICP path:

- Under `odom_xy_scale_3pct`, every per-scan motion fed to ICP is
  already biased by the same factor as raw odom. ICP cannot
  recover a signal that the bias exists, so corrected SLAM tracks
  raw odom on XY by construction.
- Under `odom_yaw_drift_*deg_per_m`, scan-to-scan ICP does extract
  a yaw correction signal (the LIDAR does not rotate when the
  drift says it should), but each per-scan correction is at the
  ICP yaw noise floor. Lowering blend alpha reduces yaw damage
  but does not deliver consistent yaw improvement.

The single change that unblocks both regimes is to match each new
scan against a **local accumulated reference** rather than against
the immediately preceding scan. That reference accumulates in the
**corrected frame**, so it is anchored against the running output
of the corrected-SLAM pose, not against per-scan biased motion.
Even under cumulative bias, the submap stays globally consistent
(modulo the corrections we already made), so the residual ICP
transform sees the accumulated odom error rather than only the
per-scan delta.

## Non-goals

- Full graph-SLAM or loop closure. The target is a **local**
  submap, not a global map. Loop closure would be a separate,
  larger change.
- Replacing the occupancy grid mapping pipeline. The occupancy
  grid (`st.map`) keeps consuming corrected poses as today.
- Changing the ROS topic interface. `/slam_diagnostics`,
  `/corrected_pose`, `/corrected_odom` keep their current
  fields; new fields are additive.
- Changing the EKF / odom blending stack outside of `slam_node`.

## High-level approach

Today `slam_node` keeps exactly one `previous_scan: Option<DMatrix<f64>>`
in body frame at the previous pose. ICP runs `icp_matching(previous, current)`
and returns a body-frame motion delta from `previous` to `current`.

The new path keeps two things:

1. A **local submap** as a point cloud in the **corrected world
   frame**, populated by transforming each accepted scan from its
   body frame into the corrected pose at the time of that scan.
2. The previous corrected pose, so that each new scan can be
   transformed into a **predicted world-frame point cloud** using
   `predicted_pose = compose_pose_local(prev_corrected_pose, odom_delta)`.

ICP then runs between (submap_world, predicted_current_scan_world)
and returns a **residual world-frame transform**. That residual is
the correction we should apply on top of the odom-only prediction.

```text
        body-frame scan
              │
              ▼
   predicted_pose = compose(prev_corrected, odom_delta)
              │
              ▼
   current_scan_world = predicted_pose * current_scan_body
              │
              ▼
   ICP(submap_world, current_scan_world)
              │
              ▼
   residual_world  ─►  blend / gate  ─►  applied_correction_world
              │                                │
              ▼                                ▼
   corrected_pose = predicted_pose ⊕ applied_correction_world
              │
              ▼
   submap ← submap ∪ (corrected_pose * current_scan_body), pruned
```

This is a structural change but it reuses every existing
component: `icp_matching`, `compute_icp_blend_decision`,
`blend_motion_delta`, `compose_pose_local`. The new code is in
two seams: building the predicted world-frame scan, and
maintaining the submap.

## Data structures

### Submap

A bounded `DMatrix<f64>` of points in the corrected world frame.

- Storage: `Option<DMatrix<f64>>` so the first scan can bootstrap
  it the same way `previous_scan` is bootstrapped today.
- Capacity policy: rolling buffer of the last `N` accepted scans,
  with two complementary cap checks:
  - `SLAM_SUBMAP_MAX_SCANS` (default ~20): bounds memory and
    ICP cost at high scan rate.
  - `SLAM_SUBMAP_RADIUS_M` (default ~3.0 m around the current
    corrected pose): drops points that have left the local
    region, so the submap stays a *local* reference rather than
    a growing global cloud.
- Decimation: each scan is subsampled by the existing
  `subsample_points_for_icp` stride before insertion, so submap
  point count stays bounded.
- Initialization gate: hold the first `SLAM_SUBMAP_BOOTSTRAP_SCANS`
  (default 3) scans before allowing ICP to run against the
  submap. Until then, fall back to scan-to-scan ICP (or pure
  odom). This avoids running ICP against an effectively
  single-scan submap that is no better than scan-to-scan.

### State additions in `SlamNodeState`

```rust
// existing:
previous_scan: Option<DMatrix<f64>>,
previous_raw_odom_at_scan: Option<Pose2D>,

// new:
local_submap_world: Option<DMatrix<f64>>,
local_submap_scan_count: usize,
previous_corrected_pose_at_scan: Option<Pose2D>,
```

`previous_scan` is kept for the scan-to-scan fallback during
bootstrap and for diagnostics. After bootstrap it is no longer
read by the ICP path.

## ICP target swap

In `main.rs` around the existing block:

```rust
if let Some(previous) = st.previous_scan.as_ref() {
    let icp_result = icp_matching(previous, &current_scan);
    ...
}
```

The new branch is:

```rust
let predicted_pose = match st.previous_corrected_pose_at_scan {
    Some(prev) => compose_pose_local(prev, odom_delta_or_zero(odom_delta)),
    None => raw_pose,
};
if let Some(submap) = st.local_submap_world.as_ref() {
    if st.local_submap_scan_count >= bootstrap_scans {
        let current_world = transform_scan_to_world(&current_scan, predicted_pose);
        let icp_result = icp_matching(submap, &current_world);
        // icp_result.translation / rotation are now in the world frame.
        // Treat them as the residual on top of predicted_pose.
        icp_delta = icp_world_residual_to_motion_delta(&icp_result);
    } else {
        // bootstrap fallback: keep scan-to-scan
        ...
    }
}
```

Two helpers are new:

- `transform_scan_to_world(scan_body: &DMatrix<f64>, pose: Pose2D) -> DMatrix<f64>`
  builds a `2×N` matrix where each column is the body-frame point
  rotated by `pose.yaw` and translated by `(pose.x, pose.y)`.
  This is a pure utility, easy to unit-test on synthetic input.
- `icp_world_residual_to_motion_delta(result: &ICPResult) -> Option<MotionDelta>`
  reuses the existing `icp_motion_delta` logic but interprets
  the result as a world-frame residual (because the input was in
  world frame). The `MotionDelta` returned is in world frame,
  not body frame.

That last point matters: today `icp_delta` is a body-frame motion
delta because both inputs to ICP were body-frame scans separated
by motion. After the swap, both inputs are world-frame point
clouds at the same predicted pose, so the residual is a
world-frame transform.

The blending stage adapts:

- Today: `blended_delta = blend_motion_delta(odom_delta, icp_delta, alpha, alpha_yaw, ...)`
  where both deltas are in body frame.
- New: `applied_correction = blend(zero_delta, icp_residual_world, alpha, alpha_yaw, ...)`
  and `corrected_pose = compose_world(predicted_pose, applied_correction)`.

The simplest way to preserve gate behavior is to compute the
*equivalent body-frame delta* and reuse the existing
`compute_icp_blend_decision` and `blend_motion_delta` unchanged.
That keeps gating math identical and isolates the change to the
target-selection seam.

## Drift consolidation (submap update)

After applying the correction, the corrected pose is the new
truth-of-record. Insert the current scan into the submap using
that pose:

```rust
let current_world_corrected =
    transform_scan_to_world(&current_scan, st.corrected_pose);
st.local_submap_world =
    Some(append_and_prune(
        st.local_submap_world.take(),
        current_world_corrected,
        st.corrected_pose,
        SubmapBudget {
            max_scans,
            max_radius_m,
        },
    ));
st.local_submap_scan_count += 1;
```

`append_and_prune` does two things:

1. Concatenate the new world-frame scan onto the existing submap
   (column-append on the `DMatrix`).
2. Drop points whose Euclidean distance from
   `st.corrected_pose.{x,y}` is greater than `max_radius_m`.
   This is the local-window prune that keeps the submap
   bounded as the robot drives.

Optionally also drop the oldest scan once the buffer reaches
`max_scans` scans. The simplest implementation keeps a parallel
`VecDeque<DMatrix<f64>>` of per-scan world clouds and rebuilds
the concatenated submap each insert. At 5 Hz scan rate × 20-scan
buffer × 320-point decimated scans, that is 6 400 points and a
20-iteration insert cost, which is negligible compared to the ICP
solve itself.

The submap stores **corrected** world-frame points, so a per-scan
correction immediately benefits the next scan's ICP target.
That is the consolidation we want: each correction "sticks" by
modifying the reference used by the next match.

## Gating semantics carry-over

The existing per-axis gate parameters (`SLAM_ICP_REJECT_ERROR`,
`SLAM_ICP_REJECT_ERROR_YAW`, `SLAM_ICP_BLEND_ALPHA*`,
`SLAM_ICP_FULL_WEIGHT_ERROR*`, etc.) are defined as **mean
nearest-neighbor distance** thresholds. That metric is well-defined
for either scan-to-scan or scan-to-map inputs, but the numeric
magnitude differs:

- Scan-to-scan: error is dominated by per-frame LIDAR noise and
  small structural changes; floor is ~1.0 cm in the current
  setup.
- Scan-to-map: error is dominated by submap point density and
  accumulated map noise; floor depends on submap point count
  and decimation stride. Likely lower (~5-8 mm) when the submap
  is well-populated because there are more candidate matches
  per current point.

So the gate thresholds will need re-tuning under scan-to-map.
Concretely, the first matrix sweep with scan-to-map should treat
the gate as if all thresholds are unknown:

1. Run `default` profile and read the per-scenario mean error
   distribution from diagnostics. That sets the new noise floor.
2. Derive a new `SLAM_ICP_REJECT_ERROR*` from that floor (same
   rule of thumb: noise floor + small margin).
3. Add the new profile to the matrix script as a separate ICP
   profile, not as a replacement for `default`, so the
   scan-to-scan and scan-to-map paths can be A/B-compared on the
   same biased odom matrix.

## Backward compatibility / rollout

Add a single environment flag:

```text
SLAM_ICP_MODE = scan_to_scan | scan_to_map   # default: scan_to_scan
```

This lets:

- The default smoke test, the synthetic ICP acceptance smoke,
  and the existing tuning matrix keep their current behavior
  with zero code-path change.
- A new matrix profile set (or per-profile env addition) opts in
  to scan-to-map for evaluation.
- Once scan-to-map demonstrates non-regression on baseline
  scenarios and improvement on biased scenarios, the default
  can flip.

This is the same incremental promotion rule already documented
in `corrected_slam_evaluation_plan.md` § Regression Promotion.

## Implementation phases

This is a multi-week effort. Suggested phasing:

1. **Helpers + unit tests** (single PR, no behavior change):
   - `transform_scan_to_world(scan_body, pose) -> scan_world`
   - `append_and_prune(existing, new_world, pose, budget) -> submap`
   - `icp_world_residual_to_body_delta(residual_world, predicted_pose) -> MotionDelta`
   - Unit tests on synthetic point clouds for each.
2. **Submap state + dual-mode plumbing** (single PR, scan_to_map
   gated off by default):
   - Add `SLAM_ICP_MODE` env, add submap state to `SlamNodeState`.
   - In the scan callback, branch on `SLAM_ICP_MODE`: existing
     path or new submap path.
   - Diagnostics gain a new field (e.g., `submap_points`) so the
     matrix script can see whether the new path is engaged.
3. **Synthetic positive-control extension** (single PR):
   - Extend `run_slam_icp_acceptance_test.sh` (or add a sibling
     script) that drives synthetic odom with known bias plus
     clean LIDAR. Require that scan-to-map mode produces
     `improvement_xy > 0` and `improvement_yaw > 0` against the
     same odom under scan-to-scan.
4. **Matrix profile + first sweep**:
   - Add an ICP profile entry in `run_navigation_revaluation_matrix.sh`
     with `SLAM_ICP_MODE=scan_to_map` plus reasonable starting
     thresholds.
   - Run the biased odom × scan-to-map combinations to verify
     the structural claim (xy_scale improvement, smaller yaw
     paradox).
5. **Gate retune + default flip** (only if step 4 shows
   non-regression + improvement on the agreed scenarios; see
   Default-Mode Gate in the evaluation plan):
   - Pick scan-to-map default thresholds.
   - Flip `SLAM_ICP_MODE` default to `scan_to_map`.
   - Remove scan-to-scan from the tuning matrix, or keep one
     profile entry for regression detection.

## Open questions to resolve during implementation

- **Submap pruning when stationary**: if the robot stops, the
  same scan repeatedly appends near-duplicate points. Probably
  resolved by the existing low-motion gate (don't update submap
  when `odom_delta` is below the low-motion threshold), but
  needs validation.
- **Predicted-pose seeding for ICP**: ICP already runs from
  identity. For scan-to-map we are providing a seeded prediction
  via `transform_scan_to_world`; ICP then refines from there.
  This is a *prior*, not just an initial guess — and it makes
  ICP convergence more sensitive to a bad odom_delta. May need
  to bound the max residual per scan more aggressively than
  scan-to-scan.
- **Memory profile for long missions**: 20-scan submap with
  bounded radius keeps memory flat, but should be measured at
  the end of the longest scenario (`rich_geometry_turns` or
  longer) to confirm no slow growth.
- **Submap reset on large jumps**: if the corrected pose ever
  jumps (it shouldn't, but defensive guard), the submap
  becomes stale instantly. Consider a guard: if the corrected
  pose moves by more than `SLAM_SUBMAP_RESET_DIST_M` (e.g.,
  2 m) within one scan, drop the submap.

## Out of scope for the first PR

- Submap loop closure / re-anchoring.
- Multi-resolution submap (coarse + fine).
- Switching ICP algorithm itself (point-to-plane, GICP).
- Persisting the submap across node restarts.

These are explicitly future work; mention them only so that the
first PR's review does not get derailed into discussing them.
