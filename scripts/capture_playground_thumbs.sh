#!/usr/bin/env bash
# Rebuild the playground (like the Pages workflow) and recapture the
# screenshots on the landing page (docs/assets/playground/*.png).
#
# Needs trunk, the wasm32-unknown-unknown target, Node 18+, and Chromium
# (Playwright's, or set CHROMIUM_PATH).
#
# Usage: ./scripts/capture_playground_thumbs.sh [name...]   (e.g. grid hero)

set -euo pipefail
cd "$(dirname "$0")/.."

(
    cd crates/rust_robotics_playground
    RUSTFLAGS='--cfg getrandom_backend="wasm_js"' trunk build index.html --release
)
(
    cd scripts/web_smoke
    [ -d node_modules ] || npm install --silent
    node capture_thumbs.mjs ../../crates/rust_robotics_playground/dist ../../docs/assets/playground "$@"
)
ls -lh docs/assets/playground/
