#!/usr/bin/env bash
set -euo pipefail

mode="${1:---check}"
case "$mode" in
  --check|--publish) ;;
  *) echo "usage: $0 [--check|--publish]" >&2; exit 2 ;;
esac

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

cargo_bin="${CARGO:-cargo}"
if ! command -v "$cargo_bin" >/dev/null 2>&1; then
  if [[ -x "$HOME/.cargo/bin/cargo.exe" ]]; then
    cargo_bin="$HOME/.cargo/bin/cargo.exe"
  else
    echo "cargo was not found; set CARGO to its executable path" >&2
    exit 1
  fi
fi

version="$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\([^"]*\)"/\1/p' Cargo.toml | head -n 1)"
if [[ -z "$version" ]]; then
  echo "could not read workspace.package.version" >&2
  exit 1
fi

packages=(
  rust_robotics_core
  rust_robotics_optimization
  rust_robotics_planning
  rust_robotics_localization
  rust_robotics_control
  rust_robotics_mapping
  rust_robotics_viz
  rust_robotics_slam
  rust_robotics
)

wait_for_version() {
  local package="$1"
  local attempts=30
  for ((i = 1; i <= attempts; i++)); do
    if curl --fail --silent --show-error "https://crates.io/api/v1/crates/${package}/${version}" >/dev/null; then
      echo "${package} ${version} is visible on crates.io"
      return 0
    fi
    echo "waiting for ${package} ${version} to reach crates.io (${i}/${attempts})"
    sleep 10
  done
  echo "timed out waiting for ${package} ${version}" >&2
  return 1
}

echo "validating RustRobotics ${version}"
"$cargo_bin" fmt --all -- --check
"$cargo_bin" check -p rust_robotics --all-features
"$cargo_bin" test -p rust_robotics --lib
"$cargo_bin" package --manifest-path vendor/nearest_neighbor/Cargo.toml --allow-dirty
"$cargo_bin" package -p rust_robotics_core --allow-dirty
"$cargo_bin" package --workspace --no-verify --allow-dirty --exclude rust_robotics_playground

if [[ "$mode" == "--check" ]]; then
  echo "release preflight passed; no crates were published"
  exit 0
fi

: "${CARGO_REGISTRY_TOKEN:?CARGO_REGISTRY_TOKEN is required for --publish}"

for package in "${packages[@]}"; do
  if curl --fail --silent "https://crates.io/api/v1/crates/${package}/${version}" >/dev/null; then
    echo "skipping ${package} ${version}; already published"
    continue
  fi
  "$cargo_bin" publish -p "$package"
  wait_for_version "$package"
done

echo "all RustRobotics ${version} crates are published"
