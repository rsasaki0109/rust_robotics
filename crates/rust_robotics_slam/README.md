# rust_robotics_slam

SLAM algorithms for the RustRobotics workspace, including EKF-SLAM, FastSLAM,
graph-based SLAM, SE(2)/SE(3) pose graph optimization, g2o I/O, IMU
preintegration, bundle adjustment, point-to-line/point-to-plane ICP, robust
ICP, and correlative scan matching.

```toml
[dependencies]
rust_robotics_slam = "0.2"
```

The advanced estimation modules share `rust_robotics_optimization` for
Levenberg-Marquardt, block-sparse PCG and Schur solvers,
information-weighted factors, manifold retractions, and
Huber/Pseudo-Huber/Cauchy losses. Camera poses and 3D pose graphs use
world-from-local SE(3) matrices; tangent vectors are translation-first. SE(3)
pose graph and IMU factors use analytic Jacobians checked against finite
differences.

`dataset` loads standard EuRoC MAV and KITTI odometry layouts. `vio_pipeline`
connects timestamped EuRoC IMU samples, optional pre-extracted visual tracks,
Schur bundle adjustment, and block-sparse SE(3) pose-graph fusion. See the
[dataset guide](../../docs/datasets.md) and pinned
[MathematicalRobotics parity report](../../docs/mathematical-robotics-parity.md).

```bash
cargo run -p rust_robotics --example headless_euroc_vio \
  --no-default-features --features slam
```

Run its focused test suite with:

```bash
cargo test -p rust_robotics_slam
```

Project site: https://rsasaki0109.github.io/rust_robotics/

Repository: https://github.com/rsasaki0109/rust_robotics
