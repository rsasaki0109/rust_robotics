# API Traits: 0.3.0 Coherence Pass

The 0.3.0 milestone makes the public API coherent enough to declare a stability
contract (Phase 3 pillar 1). This document is the design record: what the trait
architecture is, what was cleaned up, and what is protected by semver.

## Trait map

All shared traits live in `rust_robotics_core::traits` and are re-exported from
the crate root. There is exactly one trait per role; convenience 2D accessors
are inherent methods on the concrete types, not separate traits.

| Role | Trait | Notes |
| --- | --- | --- |
| Path planning | `PathPlanner` | world coordinates; samplers expose their trees through inherent `get_tree()` |
| Estimation | `StateEstimator` | GAT (`State` / `Measurement` / `Control`). **The only estimation trait.** |
| Path tracking | `PathTracker` | `compute_control(state, path)` specialization |
| Generic control | `Controller` | GAT (`State` / `Reference` / `Output`), `compute` / `reset` |
| Models | `MotionModel`, `ObservationModel` | EKF/UKF building blocks |

### Estimator unification (done in 0.3.0)

`Estimator2D` was a second estimation trait with a hard-coded 2D shape
(`predict(ControlInput, dt)` / `update(Point2D)` / `get_state() -> State2D`).
It was never implemented by any localizer — every estimator implements
`StateEstimator`, and 2D views are inherent methods (`EKFLocalizer::state_2d`,
`with_initial_state_2d`). **`Estimator2D` was removed as dead, duplicate
surface.** New localizers implement `StateEstimator` and add 2D convenience
accessors as inherent methods.

### Controller family (documented, not merged)

`Controller` is the generic base (used by `PIDController`). `PathTracker` is
the path-tracking specialization used by Pure Pursuit / Stanley / LQR Steer.
`docs/controller_arena_project.md` introduces a third, arena-local abstraction
(`ArenaControllerKind`) to compare controllers in a shared simulation. These
three coexist deliberately; unifying them into one trait is deferred until
external demand (see Deprioritize in `plan.md`).

### Tier 1 trait conformance (0.3.0)

Every Tier 1 algorithm is reachable through its role trait, and contract tests
exercise them uniformly through trait objects:

| Algorithm | Type | Trait |
| --- | --- | --- |
| A\* | `AStarPlanner` | `PathPlanner` |
| Dijkstra | `DijkstraPlanner` (new: A\* grid with a zero heuristic) | `PathPlanner` |
| JPS | `JPSPlanner` | `PathPlanner` |
| Theta\* | `ThetaStarPlanner` | `PathPlanner` |
| RRT | `RRTPlanner` | `PathPlanner` |
| RRT\* | `RRTStar` (new impl; plans on a clone, `plan_from` keeps the tree) | `PathPlanner` |
| DWA | `DWAPlanner` | none (inherent `try_plan_input` / `try_step` API, see below) |
| PID | `PIDController` | `Controller` |
| Pure Pursuit / Stanley / LQR Steer / Rear Wheel Feedback | `*Controller` | `PathTracker` (unicycle `(v, ω)` output) |
| EKF / UKF / PF family | `*Localizer` | `StateEstimator` |

Contract tests:

- `crates/rust_robotics_planning/tests/tier1_path_planner_contract.rs` — the
  six planners as `Box<dyn PathPlanner>` on one map: endpoints, ≥ 0.3 m
  clearance along every segment, a detour around the wall, and Dijkstra
  matching A\*'s optimal length.
- `crates/rust_robotics_control/tests/tier1_path_tracker_contract.rs` — the
  four trackers as `Box<dyn PathTracker>` under one unicycle model:
  convergence from a 1 m offset, following a replaced path, goal check.

They found two Tier 1 bugs, fixed in 0.3.0:

- All `PathTracker` impls adopted a new path only when its *length* differed
  (`self.path.len() != path.len()`), silently tracking the old path when a
  different path with the same point count was passed. They now compare
  contents (`Path2D: PartialEq`).
- Rear Wheel Feedback evaluated `sin(θe)·e/θe` as `0` at `θe = 0`, so a
  vehicle parallel to but offset from the path never corrected the offset.
  It now uses `sinc(θe) → 1`.

### Removed traits (0.3.0)

`GridPathPlanner`, `SamplingBasedPlanner`, and `TrajectoryTracker` had no
implementation anywhere in the workspace and duplicated roles covered by
`PathPlanner` / `PathTracker` / `Controller`. `SamplingBasedPlanner` could not
be implemented coherently: `PathPlanner::plan(&self)` cannot record a tree and
each sampler has its own node type behind an inherent `get_tree()`. They
shipped in 0.1.0, but 0.3.0 is a breaking minor release for `0.x`, nothing
implemented them, and the crates had double-digit downloads, so they were
removed outright — the `Estimator2D` precedent — instead of spending a release
on deprecation. `MotionModel` and `ObservationModel` stay as extension points
for user-defined filters; the built-in filters do not use them.

### Local planning: no trait yet (decision)

DWA is the only obstacle-aware local planner in the workspace that turns
(state, goal, obstacles) into a velocity command; `move_to_pose` and the
tracking controllers ignore obstacles, and the potential-field / elastic-band
modules produce paths. A `LocalPlanner` trait with a single implementer would
be speculative surface, so DWA keeps its inherent API as its Tier 1 contract:

- `try_new` / `try_set_state` / `try_set_goal` / `set_obstacles_from_obstacles`
  configure it; `try_plan_input() -> ControlInput` returns the unicycle
  command without moving; `try_step()` plans and advances the internal state;
  `is_goal_reached()` / `distance_to_goal()` report progress.

`crates/rust_robotics_planning/tests/tier1_dwa_contract.rs` pins that
contract. Introduce a `LocalPlanner` trait when a second implementer (e.g. a
velocity-obstacle or MPPI local planner with obstacles) lands, shaped after
`try_plan_input`.

## Stability tiers

Tier 1 is semver-checked: a breaking change requires a minor version bump and
an entry in the release notes.

- **Tier 1** — `rust_robotics_core`, `rust_robotics_localization`, and the
  high-traffic planners / controllers: A*, Dijkstra, JPS, Theta*, RRT/RRT\*,
  DWA, PID, Pure Pursuit, Stanley, LQR Steer, EKF/UKF/PF family.
- **Tier 2 (experimental)** — research-reproduction modules (`mppi`,
  `pusher_slider`, `admm_consensus`, racing `*`, `person_following_mppi`,
  `cbf_safety_filter`, ...). API may change without notice.

## Semver enforcement

CI runs `obi1kenobi/cargo-semver-checks-action` over the published crates
(`rust_robotics_core`, `rust_robotics_optimization`, `rust_robotics_planning`,
`rust_robotics_localization`, `rust_robotics_control`, `rust_robotics_mapping`,
`rust_robotics_slam`, `rust_robotics_viz`) with `continue-on-error: true` —
advisory today. Once 0.3.0 declares the stability contract, flip it to a hard
gate so a Tier 1 break fails CI. `rust_robotics_playground` and
`rust_robotics_embedded_demo` are not published and are excluded from the check.

## Frozen corpus

The `experiments_*` / `decisions_*` documentation corpus (~70 files) is frozen
(do not extend) per the Phase 3 deprioritization in `plan.md`. It lives under
`docs/archive/`, is regenerated only by the `update_*_docs` example binaries,
and is guarded by `rust_robotics_core::tests::workspace_summary_guard`.
