# RustRobotics Library-First Plan

## Status

- Date: 2026-03-11
- Scope: direction-setting document for shifting `rust_robotics` from showcase-first to library-first
- Intended audience: maintainers and contributors deciding how to evolve the crate architecture

## Summary

`rust_robotics` already has the beginnings of a reusable library:

- a public `lib.rs`
- shared types and traits
- a meaningful test suite
- some algorithms already migrated toward reusable module APIs

At the same time, the repository still behaves mostly like a showcase project:

- the README is demo-first
- many legacy `[[bin]]` targets remain in `Cargo.toml`
- visualization utilities are part of the default public surface
- examples, demos, and core algorithms still share one broad dependency boundary

The recommendation is to invert the layering:

1. make the library the product
2. make demos/examples thin wrappers around public APIs
3. keep visualization and showcase assets as optional layers

## Why This Direction Matters

Staying sample-first is attractive in the short term, but it creates limits:

- users copy code instead of depending on the crate
- API quality drifts toward demo convenience
- fixes do not accumulate well for downstream users
- headless reuse becomes harder because plotting and file output assumptions leak into the codebase
- semver, publishing, and support levels remain vague

A library-first direction makes the repository more useful for:

- robotics application development
- simulation loops
- evaluation tooling
- future ROS2 or adapter crates
- downstream crates that want stable types and tested behavior

## Product Thesis

`rust_robotics` should become a library-first robotics algorithms project with:

- a stable, typed, headless core
- explicit public APIs
- examples as documentation, not architecture
- visualization as an optional concern
- clear maturity levels for each algorithm family

## Current Observations

Observed from the current repository state:

- there is one crate with a wide surface area
- the project contains both clean library modules and legacy binary-oriented paths
- tests already exist for many modules, which is a strong base
- CI currently validates the whole repository, but it does not distinguish core library guarantees from showcase guarantees

This means the right question is not whether the project can become a library. It already can. The real question is whether the architecture and documentation will catch up to that direction.

## Target Principles

### Principle 1: Headless-first core

Core algorithms should run without:

- plotting
- filesystem side effects
- image generation
- interactive UI assumptions

### Principle 2: Public API before demo wrappers

Preferred interfaces should use:

- typed config structs
- typed state and measurement structs
- typed result objects where appropriate
- structured error handling

Legacy helper APIs can remain temporarily, but they should not be the primary documented path.

### Principle 3: Algorithm maturity is explicit

Not every algorithm needs the same support level immediately. The repository should classify modules such as:

- Stable
- Experimental
- Showcase

### Principle 4: Examples prove ergonomics

Every example should be expressible as a thin client of public library APIs. If an example requires private logic or demo-only interfaces, the library surface is incomplete.

## Recommended End State

The long-term target should be a workspace with clearer crate boundaries.

Example target shape:

```text
rust_robotics/
  Cargo.toml
  crates/
    rust_robotics_core/
    rust_robotics_planning/
    rust_robotics_localization/
    rust_robotics_mapping/
    rust_robotics_control/
    rust_robotics_slam/
    rust_robotics_viz/
    rust_robotics/
  examples/
  benches/
  docs/
```

This does not need to happen in one PR, but it should be the architectural direction if the library-first goal is accepted.

## Suggested Maturity Tiers

### Tier 1: Stable candidates

These are the most natural first candidates for library-quality support:

- A*
- Theta*
- JPS
- DWA
- EKF
- UKF
- particle filter
- pure pursuit
- Stanley control
- LQR steer control
- grid map utilities
- behavior tree
- state machine

### Tier 2: Experimental candidates

- RRT family
- PRM
- state lattice
- Frenet optimal trajectory
- MPC variants
- histogram filter
- some mapping modules

### Tier 3: Showcase/research candidates

- inverted pendulum demos
- some arm navigation examples
- graph-based SLAM until the API is clarified
- FastSLAM and ICP if they remain more demonstrative than integration-oriented

## Migration Roadmap

### Phase 0: Inventory and ownership

Goal:

- classify every module

Tasks:

- inventory algorithms and current API style
- mark each module with a maturity target
- decide which modules are first-publish candidates

### Phase 1: Documentation and direction-setting

Goal:

- make the architectural direction visible

Tasks:

- add roadmap and ADRs
- link them from the README
- define public support language

### Phase 2: Dependency boundary cleanup

Goal:

- separate headless algorithm code from visualization and showcase concerns

Tasks:

- reduce legacy `[[bin]]` usage
- move more runnable content to examples
- isolate visualization behind a crate or feature boundary

### Phase 3: Stable subset hardening

Goal:

- make a small subset truly reusable

Tasks:

- standardize configs and result types
- add config validation
- document assumptions and failure modes
- add deterministic paths for stochastic algorithms

### Phase 4: Publishing readiness

Goal:

- make at least one stable subset publishable with confidence

Tasks:

- fill in crate metadata
- document semver expectations
- split CI by concern where necessary
- benchmark representative algorithms

## Immediate Priorities

The highest-value near-term work is:

1. write and accept the architectural direction
2. pick the first stable subset
3. separate visualization from core execution
4. migrate examples to thin wrappers over public APIs
5. improve docs so the crate is described as a library first

## Risks

### Risk: over-splitting too early

Mitigation:

- split by dependency boundary, not by folder aesthetics

### Risk: preserving every legacy API forever

Mitigation:

- use deprecation windows and migration guides

### Risk: treating every algorithm as equally mature

Mitigation:

- publish support tiers and focus on a stable subset first

## First Deliverables

The first concrete outputs after accepting this direction should be:

- accepted ADRs for product direction and crate boundaries
- a stable-subset shortlist
- an implementation plan for visualization separation
- a small PR sequence rather than a single giant refactor

## Recommendation

`rust_robotics` should move toward being a practical library rather than remaining primarily a showcase repository.

The right approach is not to discard the showcase value. The right approach is to subordinate it to a stronger architecture:

- library first
- examples second
- visualization optional
- maturity explicit
- stable subset delivered early
