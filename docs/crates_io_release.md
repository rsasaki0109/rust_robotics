# crates.io Release Checklist

This workspace has several crates that depend on each other. Publish them in
dependency order and wait for the crates.io index to update between layers.

## Preflight

```bash
cargo fmt --all -- --check
cargo check -p rust_robotics --all-features
cargo test -p rust_robotics --lib
cargo package --manifest-path vendor/nearest_neighbor/Cargo.toml
cargo package -p rust_robotics_core
cargo package --workspace --no-verify --exclude rust_robotics_playground
```

Before the first publish, confirm each package name is still absent from the
registry:

```bash
cargo info rust_robotics
cargo info rust_robotics_core
cargo info rust_robotics_planning
cargo info rust_robotics_localization
cargo info rust_robotics_control
cargo info rust_robotics_mapping
cargo info rust_robotics_slam
cargo info rust_robotics_viz
cargo info rust_robotics_nearest_neighbor
```

Each command should report that the crate cannot be found. After the first
release, the same commands should resolve to the published versions.

`cargo publish --dry-run` for dependent workspace crates will fail until their
internal dependencies already exist on crates.io. That is expected before the
first release; run each dependent dry-run only after the previous dependency
layer has been published and the index has caught up.

## Publish Order

Run the matching `--dry-run` first, then publish for real.

```bash
# 1. Vendored KD-tree helper used by rust_robotics_slam.
cargo publish --manifest-path vendor/nearest_neighbor/Cargo.toml --dry-run
cargo publish --manifest-path vendor/nearest_neighbor/Cargo.toml

# 2. Shared core crate.
cargo publish -p rust_robotics_core --dry-run
cargo publish -p rust_robotics_core

# Wait until `cargo info rust_robotics_core` resolves from crates.io.

# 3. Domain crates that depend on core.
cargo publish -p rust_robotics_planning --dry-run
cargo publish -p rust_robotics_planning

cargo publish -p rust_robotics_localization --dry-run
cargo publish -p rust_robotics_localization

cargo publish -p rust_robotics_control --dry-run
cargo publish -p rust_robotics_control

cargo publish -p rust_robotics_mapping --dry-run
cargo publish -p rust_robotics_mapping

cargo publish -p rust_robotics_viz --dry-run
cargo publish -p rust_robotics_viz

# 4. SLAM depends on core plus rust_robotics_nearest_neighbor.
cargo publish -p rust_robotics_slam --dry-run
cargo publish -p rust_robotics_slam

# 5. Umbrella crate depends on all domain crates.
cargo publish -p rust_robotics --dry-run
cargo publish -p rust_robotics
```

If a command reports `no matching package named ... found`, wait a minute and
retry after the crates.io index catches up.

## After Publish

```bash
git tag v0.1.0
git push origin v0.1.0
gh release create v0.1.0 --title "v0.1.0" --notes "Initial crates.io release of RustRobotics."
```

Then update the root README library section from the Git dependency to the
published version.
