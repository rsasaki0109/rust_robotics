# rust_robotics_playground

Interactive browser-ready demos for RustRobotics algorithms. This crate is
**not** published to crates.io; it ships with the repository for local debug
and GitHub Pages deployment.

## Run locally (native)

```bash
cargo run -p rust_robotics_playground
```

## Run in the browser (WASM)

Install [Trunk](https://trunkrs.dev/) and serve from this directory:

```bash
rustup target add wasm32-unknown-unknown
cd crates/rust_robotics_playground
RUSTFLAGS='--cfg getrandom_backend="wasm_js"' trunk serve --public-url /
```

Release build (matches GitHub Pages):

```bash
RUSTFLAGS='--cfg getrandom_backend="wasm_js"' trunk build --release --public-url /rust_robotics/playground/
```

Live demo: https://rsasaki0109.github.io/rust_robotics/playground/

## Reproducible links

Use **Copy share link** in the header to copy a URL for the active demo. Grid
planner links preserve the selected planner, start and goal cells, and the full
obstacle map, so another visitor opens the same planning problem. For example,
`?tab=grid&planner=theta&start=2,12&goal=29,12&map=...` opens a reproducible
Theta* scenario. The other tabs currently preserve the selected demo tab.

## Tabs

- **Grid Planners** — A\*, Dijkstra, JPS, Theta\* with click-to-edit obstacles.
- **Localization** — PF / EKF with arrow-key driving and noise slider.
- **SLAM** — EKF-SLAM, FastSLAM 1.0, ICP scan matching on a canned loop (timeline scrubber).
- **ADMM Formation** — receding-horizon consensus ADMM with four agents past an L-corner.
