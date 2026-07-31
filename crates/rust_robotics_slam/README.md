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
differences. IMU support includes body-from-sensor extrinsics with lever-arm
acceleration, navigation-state/bias priors, bias random walks,
position/velocity factors, and relative navigation-state factors.

`dataset` loads standard EuRoC MAV and KITTI odometry layouts.
`visual_frontend` provides decoder-independent Shi-Tomasi detection,
pyramidal Lucas-Kanade tracking, forward/backward validation and metric
triangulation. `vio_pipeline` connects timestamped EuRoC IMU samples,
generated visual tracks, Schur bundle adjustment, joint navigation-state/bias
refinement, and block-sparse SE(3) pose-graph fusion. See the
[dataset guide](../../docs/datasets.md) and pinned
[MathematicalRobotics porting matrix](../../docs/mathematical-robotics-porting.md)
and [parity report](../../docs/mathematical-robotics-parity.md).

```bash
cargo run --release -p rust_robotics \
  --example generate_euroc_feature_tracks \
  --no-default-features --features slam -- /datasets/EuRoC/MH_01_easy

cargo run -p rust_robotics --example headless_euroc_vio \
  --no-default-features --features slam
```

Run its focused test suite with:

```bash
cargo test -p rust_robotics_slam
```

Project site: https://rsasaki0109.github.io/rust_robotics/

Repository: https://github.com/rsasaki0109/rust_robotics
