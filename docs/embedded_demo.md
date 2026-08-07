# Embedded Reference Demo

Phase 3 pillar 3: prove the "no_std Kalman / controller on a $5 MCU" claim
with a runnable artifact, not just a `cargo build` check.

## What it is

`crates/rust_robotics_embedded_demo/` is a `no_std` binary for
`thumbv7em-none-eabihf` (Cortex-M4F). It is deliberately **excluded** from the
workspace so normal host builds never try to compile it; CI builds it with an
explicit target.

Inside the demo, three of the library's own crates run on the bare-metal target:

- `rust_robotics_localization` — an **EKF** fuses noisy position measurements
  into a state estimate (`no_std` Kalman).
- `rust_robotics_control` — a **Pure Pursuit** path tracker steers and a
  **PID** speed controller accelerates (`no_std` Tier 1 controllers).
- `rust_robotics_core` — `Path2D` / `Point2D` / `StateEstimator`.

The plant is simulated on-chip: 400 steps of a bicycle model driving a straight
path while the EKF tracks it. A tiny deterministic LCG supplies measurement
noise (no entropy source required on the MCU).

## Running it

Prerequisites: `rustup target add thumbv7em-none-eabihf` and `qemu-system-arm`.

```bash
./scripts/run_embedded_demo.sh
```

This builds the demo and runs it under QEMU on the `netduinoplus2` machine
(STM32F405, Cortex-M4) with semihosting for console output and exit.
`memory.x` targets 128 KB SRAM to match what QEMU models for this board
(the real F405 has 192 KB):

```
rust_robotics embedded demo: EKF + PID/Pure Pursuit on Cortex-M
step=  0 true=(+0.00,+0.00) est=(+0.02,-0.07) err=0.068
step=100 true=(+9.91,+0.00) est=(+9.92,+0.02) err=0.026
...
final EKF position error = 0.0363 m
embedded demo PASS
```

The script fails unless QEMU prints `embedded demo PASS` (and QEMU exits `0`).
CI runs this on every push/PR (`embedded-demo` job, installs QEMU itself).

## Why it matters

- The same `rust_robotics_localization` / `rust_robotics_control` crates used by
  the host examples and the browser playground also build and execute on a
  microcontroller with zero `std` — every `f64` math call goes through libm.
- It is a screenshot, not a compile check: the EKF actually converges under QEMU.
- It is the seed for a real-board (RP2040/STM32) bring-up, and the CI gate keeps
  the no_std surface from rotting.

## CI coverage

- `embedded-check`: compiles `rust_robotics_core` + `rust_robotics_localization`
  + `rust_robotics_control` for `thumbv7em-none-eabihf`.
- `embedded-demo`: builds and *runs* the demo under QEMU, plus fmt/clippy for the
  demo crate.
