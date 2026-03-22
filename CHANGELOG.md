# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-03-23

### Added
- Cargo workspace with 8 domain crates:
  - `rust_robotics_core` — shared types, traits, error handling
  - `rust_robotics_planning` — path planning algorithms (A\*, JPS, Theta\*, RRT, DWA, etc.)
  - `rust_robotics_localization` — EKF, UKF, Particle Filter, Histogram Filter
  - `rust_robotics_control` — path tracking, LQR, MPC, behavior tree, state machine
  - `rust_robotics_mapping` — NDT, Gaussian Grid, Ray Casting
  - `rust_robotics_slam` — EKF-SLAM, FastSLAM, Graph SLAM, ICP
  - `rust_robotics_viz` — Visualizer with gnuplot backend
  - `rust_robotics` — umbrella crate with feature-gated re-exports
- **Dubins Path** planner (6 path types: LSL, RSR, LSR, RSL, RLR, LRL)
- Top-level re-exports for planning (19 types), control (9), localization (7), mapping (3), slam (2)
- Unified planner APIs: `plan_from()` for RRT\*/Informed RRT\*, `plan_with_obstacles()` for Potential Field
- `#[must_use]` on core types (Point2D, Point3D, Pose2D, State2D, ControlInput, GridNode)
- Headless examples (grid planners, localizers, navigation loop)
- Criterion benchmarks for A\* vs JPS vs Theta\*
- Proptest property-based tests for EKF/UKF/PF non-divergence
- 230 unit tests across all crates
- CI pipeline: build, test, clippy (-D warnings), rustdoc (-D warnings), fmt, cargo-deny, cargo-tarpaulin coverage, cargo-semver-checks (PRs)
- GitHub Pages with showcase gallery and API docs
- README badges (CI, codecov, docs)
- CLAUDE.md with build/test/feature documentation
- `deny.toml` for dependency auditing
- Workspace dependency versions for crates.io publish readiness

### Fixed
- Clippy warnings (identical if blocks in ICP, collapsible else-if in JPS)
- Rustdoc warnings (escaped `[m]`/`[rad]` in doc comments, bare URLs)
- Broken README example commands (30 removed, 5 updated with `--features`)
- Showcase `--bin` commands updated to `--example`

### Removed
- Unused `plotlib` dependency from `rust_robotics_viz`
