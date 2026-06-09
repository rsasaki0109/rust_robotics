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
