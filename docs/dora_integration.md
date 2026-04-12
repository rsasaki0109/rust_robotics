# dora-rs Integration Guide

This workspace includes a dora-rs planning demo and an EKF localization node that can be used in larger navigation pipelines.

## 1) Install dora-rs

Use one of the following installation methods.

### Linux/macOS installer (recommended)

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.sh | sh
```

### Cargo install

```bash
cargo install dora-cli --locked
```

Verify installation:

```bash
dora --help
```

## 2) Run the existing planning demo

The current demo dataflow is defined at:

- `crates/rust_robotics/examples/dora_path_planning_dataflow.yml`

Run it from the workspace root:

```bash
dora run crates/rust_robotics/examples/dora_path_planning_dataflow.yml
```

What it does:

- `dora_path_planning_node`: runs A* planning and publishes a path JSON report.
- `dora_path_metrics_node`: computes path quality metrics from the report.
- `dora_path_planning_sink`: consumes and prints the path report.

## 3) EKF localization node

New example:

- `crates/rust_robotics/examples/dora_ekf_node.rs`

Inputs and output:

- input `control`: velocity and yaw-rate command (`v`, `omega`, `dt`)
- input `measurement`: noisy position measurement (`x`, `y`)
- output `state_estimate`: EKF state (`x`, `y`, `yaw`, `v`, covariance trace)

Build as a dora node:

```bash
cargo build -p rust_robotics --example dora_ekf_node --features "localization,dora"
```

### Fallback mode (without dora feature)

If dora is not available, this example can run as a stdin/stdout process:

```bash
cargo run -p rust_robotics --example dora_ekf_node --features localization
```

Line protocol:

- `control <v> <omega> <dt>`
- `measurement <x> <y>`
- output: `state_estimate <x> <y> <yaw> <v>`

Example:

```text
control 1.0 0.1 0.1
measurement 0.12 -0.03
measurement 0.22 0.01
```

## 4) Architecture and dataflow

Current planning-only demo:

```text
timer(tick)
   |
   v
[dora_path_planning_node] --path-report--> [dora_path_metrics_node]
              \--path-report-------------> [dora_path_planning_sink]
```

Expanded navigation pipeline (planner + localization loop):

```text
          goal/map
            |
            v
      [planner node] --planned_path--> [controller node] --control(v,omega,dt)----+
            ^                                                                     |
            |                                                                     v
      [replan policy] <--state_estimate-- [dora_ekf_node] <--measurement-- [sensor node]
```

Notes:

- EKF node fuses control and measurement streams, then publishes `state_estimate`.
- Planner can use `state_estimate` for closed-loop replanning.
- The same EKF logic is testable without dora via fallback stdin/stdout mode.
