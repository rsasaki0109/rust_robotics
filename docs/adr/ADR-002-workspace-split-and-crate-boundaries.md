# ADR-002: Move Toward Explicit Crate Boundaries

- Status: Proposed
- Date: 2026-03-11
- Related: [Library-first roadmap](../roadmaps/library-first-plan.md)
- Depends on: [ADR-001](./ADR-001-library-first-product.md)

## Context

The current repository is a single crate that mixes:

- reusable algorithm logic
- legacy binary entry points
- visualization helpers
- showcase-oriented assets

This is workable for a demo-focused repository, but it makes library growth harder:

- the dependency boundary is broad
- visualization remains close to core execution paths
- stable and experimental APIs are difficult to isolate
- users cannot depend on a smaller subset cleanly

## Decision

The project should move toward explicit crate boundaries, likely through a workspace, as the library-first migration progresses.

The intended shape is approximately:

```text
rust_robotics/
  crates/
    rust_robotics_core/
    rust_robotics_planning/
    rust_robotics_localization/
    rust_robotics_mapping/
    rust_robotics_control/
    rust_robotics_slam/
    rust_robotics_viz/
    rust_robotics/
```

## Boundary Rules

### `rust_robotics_core`

Owns:

- shared types
- traits
- common error types

Must not own:

- plotting
- algorithm-family-specific implementations

### Domain crates

Planning, localization, mapping, control, and SLAM should own their respective algorithm implementations and public configs/results.

### `rust_robotics_viz`

Owns:

- plotting
- rendering helpers
- showcase-oriented output adapters

Must not own:

- core algorithm behavior

### Umbrella crate

The top-level `rust_robotics` crate can remain as a friendly re-export layer for stable subsets.

## Rationale

This direction is preferred because it:

- keeps core execution headless by default
- supports smaller dependency footprints
- allows maturity to evolve by domain
- makes future publishing strategy more realistic

## Migration Notes

This is a direction, not a demand for one giant refactor. A staged migration is preferred:

1. define the boundaries
2. migrate a stable subset first
3. keep compatibility shims only where they reduce churn meaningfully
