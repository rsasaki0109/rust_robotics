# RustRobotics

[![CI](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/rsasaki0109/rust_robotics/branch/main/graph/badge.svg)](https://codecov.io/gh/rsasaki0109/rust_robotics)
[![crates.io](https://img.shields.io/crates/v/rust_robotics.svg)](https://crates.io/crates/rust_robotics)
[![docs.rs](https://docs.rs/rust_robotics/badge.svg)](https://docs.rs/rust_robotics)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/rust_robotics/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Robotics algorithms in Rust: planning, localization, SLAM, control, and manipulation.
Try them in your browser, then use the same library in your own project.

<p align="center">
  <a href="https://rsasaki0109.github.io/rust_robotics/playground/"><b>Open the playground</b></a>
  ·
  <a href="#quick-start"><b>Run locally</b></a>
  ·
  <a href="https://docs.rs/rust_robotics"><b>API docs</b></a>
</p>

## Try in your browser

Edit a map, move a robot, or inspect an algorithm step by step.
Pause, replay, and share your experiment. No installation or account required.

<table>
  <tr>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=grid"><img src="docs/assets/playground/grid-search.png" width="260" alt="Grid planner search replay"/></a><br/><b>Grid planners</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=localization&amp;loc_view=belief&amp;belief_event=resample"><img src="docs/assets/playground/belief-lab.png" width="260" alt="Particle filter belief update"/></a><br/><b>Localization</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=slam&amp;algorithm=loop"><img src="docs/assets/playground/slam-loop.png" width="260" alt="SLAM loop closure comparison"/></a><br/><b>SLAM</b></td>
  </tr>
  <tr>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=arena"><img src="docs/assets/playground/arena.png" width="260" alt="Path tracking controller comparison"/></a><br/><b>Controller Arena</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=arm"><img src="docs/assets/playground/arm.png" width="260" alt="Two-link robot arm inverse kinematics"/></a><br/><b>Arm Lab</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=drone&amp;mode=manual&amp;course=climb&amp;cinema=1&amp;intro=1"><img src="docs/assets/playground/drone-v1.png" width="260" alt="Manual drone flight beside an autonomous ghost"/></a><br/><b>Flight Lab</b></td>
  </tr>
</table>

[Explore all experiments](https://rsasaki0109.github.io/rust_robotics/)
· [Playground guide](crates/rust_robotics_playground/README.md)

## Quick Start

Clone the repository and launch the native playground:

```bash
git clone https://github.com/rsasaki0109/rust_robotics.git
cd rust_robotics
cargo run -p rust_robotics_playground
```

### Rust navigation loop

Run planning, localization, and path tracking together without a GUI:

```bash
cargo run -p rust_robotics --example headless_navigation_loop \
  --features "planning,localization,control"
```

## Use As A Library

```bash
cargo add rust_robotics --features "planning,localization,control"
```

```rust
use rust_robotics::planning::{AStarConfig, AStarPlanner};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::control::PurePursuitController;
```

See the [Rust guide](docs/rust_guide.md) for workspace crates, more examples,
build instructions, and benchmarks.

## Embedded / no_std

Core types and Kalman filters support `no_std` with `alloc`:

```bash
rustup target add thumbv7em-none-eabihf
cargo build -p rust_robotics_core -p rust_robotics_localization \
  --no-default-features --target thumbv7em-none-eabihf
```

[Run the embedded demo](docs/embedded_demo.md) with EKF, Pure Pursuit, and PID
on Cortex-M under QEMU.

## Documentation

- [Algorithm gallery](docs/algorithm_gallery.md) — animations, source, and commands.
- [Rust guide](docs/rust_guide.md) — library APIs, examples, and benchmarks.
- [Playground guide](crates/rust_robotics_playground/README.md) — controls and browser builds.
- [ROS2 / Gazebo](docs/ros2_integration.md) · [dora-rs](docs/dora_integration.md).
- [Datasets and VIO](docs/datasets.md) · [Interfaces](docs/interfaces.md).
- [Pose-observation loss experiment](experiments/pose_observation_loss/README.md).

## Community

[Contributions](CONTRIBUTING.md) and [algorithm requests](.github/ISSUE_TEMPLATE/algorithm_request.yml)
are welcome. See the [security policy](SECURITY.md) and [MIT license](LICENSE).

Inspired by [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and
[MathematicalRobotics](https://github.com/scomup/MathematicalRobotics).
Attributions are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
