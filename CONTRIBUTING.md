# Contributing to RustRobotics

## Getting Started

```bash
git clone https://github.com/rsasaki0109/rust_robotics.git
cd rust_robotics
cargo build --workspace --lib --tests --examples
cargo test --workspace --lib --tests
```

## Code Quality Checks

All of these must pass before submitting a PR:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -W clippy::all -D warnings
cargo build --workspace --lib --tests --examples
cargo test --workspace --lib --tests
cargo doc --workspace --no-deps
cargo deny check
```

If the change touches ROS2/Gazebo integration, also run the relevant smoke
script from `ros2_nodes/launch/` after sourcing the ROS2 environment.

## Adding a New Algorithm

1. Add the implementation in the appropriate crate under `crates/`:
   - `rust_robotics_planning` for path planners
   - `rust_robotics_localization` for filters
   - `rust_robotics_control` for controllers
   - `rust_robotics_mapping` for mapping algorithms
   - `rust_robotics_slam` for SLAM algorithms

2. Follow existing patterns:
   - Config struct with `validate()` method
   - `try_new()` returning `RoboticsResult<Self>`
   - Implement `PathPlanner` trait (for planners) or equivalent
   - Use `plan()` returning `RoboticsResult<Path2D>` (not the legacy `planning()`)

3. Add tests in the same file under `#[cfg(test)] mod tests`

4. Register the module in `lib.rs` and add re-exports

5. Add a section to `README.md` with description and source link
6. Add a runnable example or a focused comparison test when the behavior is
   easier to understand through output than through API docs alone

## Conventions

- `#![forbid(unsafe_code)]` in all crates
- Return `Result` from library code, never `unwrap()` (use `expect()` only for true invariants)
- Use seeded RNG (`StdRng::seed_from_u64`) for deterministic tests
- Escape `[m]` as `\[m\]` in doc comments (rustdoc treats `[...]` as links)
- Long-running tests (>10s) should be `#[ignore]`
- Keep examples deterministic where possible so CI output is stable
- Prefer small, reviewable PRs that cover one algorithm, experiment, or docs
  change at a time

## Running Benchmarks

```bash
cargo bench --bench unified_planning_benchmark -p rust_robotics_planning
cargo bench --bench jps_crossover_benchmark -p rust_robotics_planning
```

## Documentation And Showcase Updates

When adding a visual example, include the generated image under `img/` or
`docs/assets/`, link the source file from the README, and add the card to
`docs/app.js` if it is a good public demo.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
