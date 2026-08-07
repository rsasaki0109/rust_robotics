#!/usr/bin/env bash
# Build and run the bare-metal RustRobotics reference demo under QEMU.
#
# The demo is an excluded workspace crate targeting thumbv7em-none-eabihf and
# runs on the QEMU `netduinoplus2` machine (Cortex-M4 / STM32F405) using
# semihosting for console output and exit. CI greps for the PASS line and
# relies on the QEMU exit status.
#
# Commands run from inside the demo directory so the crate's own
# `.cargo/config.toml` (target + `-Tlink.x` rustflags) applies.
#
# Usage: scripts/run_embedded_demo.sh
set -euo pipefail

DEMO_DIR="$(git rev-parse --show-toplevel)/crates/rust_robotics_embedded_demo"
cd "$DEMO_DIR"

echo "== Building embedded demo (thumbv7em-none-eabihf) =="
cargo build

BIN="target/thumbv7em-none-eabihf/debug/rust_robotics_embedded_demo"

echo "== Running under QEMU (netduinoplus2, semihosting) =="
OUTPUT="$(qemu-system-arm -machine netduinoplus2 -semihosting -nographic -monitor none -kernel "$BIN")"
echo "$OUTPUT"

if ! printf '%s\n' "$OUTPUT" | grep -q "embedded demo PASS"; then
  echo "!! embedded demo did not report PASS" >&2
  exit 1
fi

echo ""
echo "== PASS: embedded demo ran on emulated Cortex-M4 and converged =="
