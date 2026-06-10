#!/usr/bin/env bash
# Regenerate every animated GIF in media/gallery/.
#
# Pure Rust pipeline (rust_robotics_viz "gif" feature) — no gnuplot or other
# system packages required.
#
# Usage: ./scripts/generate_gallery_gifs.sh

set -euo pipefail
cd "$(dirname "$0")/.."

run() {
    local example="$1" features="$2"
    echo "==> ${example}"
    cargo run --release -p rust_robotics --example "${example}" --features "${features}"
}

run render_gif_ekf_localization "localization,gif"
run render_gif_particle_filter "localization,gif"
run render_gif_pure_pursuit "control,gif"
run render_gif_dwa "planning,gif"
run render_gif_rrt "planning,gif"
run render_gif_slam "slam,gif"

echo
echo "Gallery GIFs:"
ls -lh media/gallery/*.gif
