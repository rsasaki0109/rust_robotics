# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- Migrated from single-crate monolith to Cargo workspace with 8 domain crates
- Separated visualization (gnuplot) from algorithm logic
- Converted binary targets to examples

### Added
- `rust_robotics_core` — shared types, traits, error handling
- `rust_robotics_planning` — all path planning algorithms
- `rust_robotics_localization` — EKF, UKF, Particle Filter, Histogram Filter
- `rust_robotics_control` — path tracking, LQR, MPC, behavior tree, state machine
- `rust_robotics_mapping` — NDT, Gaussian Grid, Ray Casting
- `rust_robotics_slam` — EKF-SLAM, FastSLAM, Graph SLAM, ICP
- `rust_robotics_viz` — Visualizer with gnuplot backend
- `rust_robotics` — umbrella crate with feature-gated re-exports
- Headless examples that work without visualization dependencies
- 199 unit tests across all crates
