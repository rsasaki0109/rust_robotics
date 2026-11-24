# crates.io Release Checklist

This workspace has several crates that depend on each other. Publish them in
dependency order and wait for the crates.io index to update between layers.

> **Release status (2026-11-24):** `0.3.0` is live on crates.io for all nine
> crates. (`0.2.0` was tagged on GitHub but never published to crates.io, so
> crates.io went `0.1.0` -> `0.3.0`.) `vendor/nearest_neighbor` stays at
> `0.1.0`; it only needs re-publishing if it actually changed (skip step 1). For
> subsequent releases the "confirm name is absent" preflight does not apply (the
> crates already exist). Bump the version in `[workspace.package]`, the internal
> dependency pins in `[workspace.dependencies]`, and
> `crates/rust_robotics_embedded_demo/Cargo.toml` together before publishing, so
> a new crate resolves its siblings to the new version instead of the
> already-published one.
>
> A crate that has never been published (e.g. `rust_robotics_optimization` in
> 0.3.0) is subject to the crates.io new-crate rate limit (HTTP 429). Publish
> the already-published crates first, then that crate on its own with
> `cargo publish -p <crate>`, retrying after the time in the error message. If
> `cargo publish` verification fails on the release machine, `--no-verify` is
> acceptable after a separate `cargo build` and `cargo test`.

## Preflight

The preferred path is the manually triggered `Release` GitHub Actions
workflow. Run it once with `publish=false`; after the preflight succeeds, run
the same version with `publish=true`. The publishing run requires a
`CARGO_REGISTRY_TOKEN` secret in the protected `crates-io` environment. It
publishes in dependency order, waits for crates.io propagation, and then
creates the matching tag and GitHub Release. Re-running it is safe: versions
already visible on crates.io are skipped.

For a local preflight, run:

```bash
bash scripts/publish_workspace.sh --check
```

The commands below document the manual fallback.

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
cargo info rust_robotics_optimization
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

# 3. Shared optimization crate.
cargo publish -p rust_robotics_optimization --dry-run
cargo publish -p rust_robotics_optimization

# Wait until `cargo info rust_robotics_optimization` resolves from crates.io.

# 4. Domain crates that depend on core.
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

# 5. SLAM depends on core, optimization, and rust_robotics_nearest_neighbor.
cargo publish -p rust_robotics_slam --dry-run
cargo publish -p rust_robotics_slam

# 6. Umbrella crate depends on all domain crates.
cargo publish -p rust_robotics --dry-run
cargo publish -p rust_robotics
```

If a command reports `no matching package named ... found`, wait a minute and
retry after the crates.io index catches up.

## After Publish

```bash
git tag v0.3.0
git push origin v0.3.0
gh release create v0.3.0 --title "v0.3.0" \
  --generate-notes
```

Then verify that `cargo add rust_robotics` resolves the published version and
that docs.rs has built it. The root README intentionally uses `cargo add`
instead of hard-coding a version, so no post-release version edit is required.
