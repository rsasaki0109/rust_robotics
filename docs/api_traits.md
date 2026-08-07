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
| Path planning | `PathPlanner`, `GridPathPlanner`, `SamplingBasedPlanner` | `SamplingBasedPlanner: PathPlanner` |
| Estimation | `StateEstimator` | GAT (`State` / `Measurement` / `Control`). **The only estimation trait.** |
| Path tracking | `PathTracker` | `compute_control(state, path)` specialization |
| Trajectory tracking | `TrajectoryTracker` | time-parameterized `compute_control(state, time)` |
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
