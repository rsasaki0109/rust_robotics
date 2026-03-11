# ADR-001: Adopt a Library-First Product Direction

- Status: Proposed
- Date: 2026-03-11
- Related: [Library-first roadmap](../roadmaps/library-first-plan.md)

## Context

`rust_robotics` began as a Rust implementation of PythonRobotics-style examples and remains highly valuable as a showcase repository. The project now also contains:

- a public library crate surface
- shared types and traits
- reusable algorithm modules
- a growing automated test suite

This creates a fork in the road. The project can continue to optimize mainly for demos, or it can treat the reusable library surface as the primary product while still keeping demos as a major strength.

## Decision

The project will be developed as a library-first robotics algorithms project.

This means:

1. the library is the product
2. examples and demos are thin clients of public APIs
3. headless execution is the default assumption for core algorithms
4. visualization is optional and should not define core architecture
5. algorithm maturity must be explicit in code structure and documentation

## Consequences

### Positive

- better reuse by downstream crates
- clearer public API ownership
- easier semver and publishing strategy
- cleaner future integration paths such as ROS2 adapters
- easier testing and benchmarking

### Negative

- more up-front architectural work
- migration pressure on legacy demo-oriented APIs
- examples may need to be rewritten to use cleaner public interfaces

## Non-Goals

This decision does not imply:

- every algorithm becomes production-ready immediately
- demos should be removed
- all legacy signatures remain forever
- ROS2-specific concerns belong in the core crate

## Follow-Up

The immediate follow-up is to define:

- architecture boundaries
- maturity tiers
- migration sequence for the first stable subset
