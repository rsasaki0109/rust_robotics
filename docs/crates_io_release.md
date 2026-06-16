# crates.io Release Checklist

This workspace has several crates that depend on each other. Publish them in
dependency order and wait for the crates.io index to update between layers.

> **Release status (2026-06-17):** `0.1.0` is live on crates.io for all nine
> crates. The next release is **`0.2.0`** (no_std localization stack + pure-Rust
> GIF gallery + RRT `get_tree` fix). For subsequent releases the workflow is the
> same as below, with two differences: the "confirm name is absent" preflight no
> longer applies (the crates already exist), and `vendor/nearest_neighbor` only
> needs re-publishing if it actually changed (it is unchanged for 0.2.0, so it
> stays at `0.1.0` — skip step 1). Bump the version in `[workspace.package]` and
> the internal dependency pins in `[workspace.dependencies]` together before
> publishing, so a `0.2.0` crate resolves its siblings to `0.2.0` instead of the
> already-published `0.1.0`. To ship as a backward-compatible `0.1.1` instead,
> set both to `0.1.1` — but `0.2.0` is preferred because the no_std default-
> feature restructuring is a semver-significant change for `0.x` consumers.

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
git tag v0.2.0
git push origin v0.2.0
gh release create v0.2.0 --title "v0.2.0" \
  --notes "no_std localization stack, pure-Rust GIF gallery, RRT get_tree fix. See CHANGELOG.md."
```

Then update the root README library section from the Git dependency to the
published version.
