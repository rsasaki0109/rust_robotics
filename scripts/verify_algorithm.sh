#!/bin/bash
# Verify a newly added algorithm compiles, passes tests, and has clean docs.
# Usage: ./scripts/verify_algorithm.sh <crate_suffix> <module_name>
# Example: ./scripts/verify_algorithm.sh control pid_controller

set -euo pipefail

CRATE="${1:?Usage: verify_algorithm.sh <crate_suffix> <module_name>}"
MODULE="${2:?Usage: verify_algorithm.sh <crate_suffix> <module_name>}"
PKG="rust_robotics_${CRATE}"

echo "=== Verifying ${PKG}::${MODULE} ==="

echo "[1/5] cargo fmt"
cargo fmt --all

echo "[2/5] cargo check"
cargo check -p "${PKG}"

echo "[3/5] cargo clippy"
cargo clippy -p "${PKG}" --all-targets -- -D warnings

echo "[4/5] cargo test (${MODULE})"
cargo test -p "${PKG}" -- "${MODULE}" --nocapture

echo "[5/5] cargo doc"
RUSTDOCFLAGS="-D warnings" cargo doc -p "${PKG}" --no-deps

echo ""
echo "=== PASS: ${PKG}::${MODULE} ==="
