# MathematicalRobotics porting matrix

This table tracks the RustRobotics counterpart of
[scomup/MathematicalRobotics](https://github.com/scomup/MathematicalRobotics)
at commit
[`79600010f0c86179905a6960e5fce2bb7cc85d77`](https://github.com/scomup/MathematicalRobotics/tree/79600010f0c86179905a6960e5fce2bb7cc85d77).

For this first-pass inventory, **ported** means that RustRobotics has an
executable Rust implementation of the corresponding functionality. It does
not require a line-by-line translation or a dedicated upstream parity test.
The parity column records the smaller set that is also checked against pinned
MathematicalRobotics numerical output.

| Status | Meaning |
|---|---|
| ✅ Ported | Corresponding functionality is implemented in RustRobotics |
| 🟡 Partial | The main functionality exists, but some APIs or variants in the grouped row are missing |
| ⬜ Not ported | No corresponding Rust implementation was found |
| ➖ Support | Demo GUI, plotting code, or bundled data rather than a core algorithm |

## Summary

| Ported | Partial | Not ported | Support |
|---:|---:|---:|---:|
| 19 | 3 | 3 | 2 |

## Correspondence table

| MathematicalRobotics source | Functionality | RustRobotics counterpart | Status | Pinned parity |
|---|---|---|---|---|
| [`filter/ekf.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/filter/ekf.py) | 2D odometry/GPS EKF | [`localization/ekf.rs`](../crates/rust_robotics_localization/src/ekf.rs) | ✅ Ported | — |
| [`filter/particle_filter.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/filter/particle_filter.py) | 2D particle-filter localization | [`localization/particle_filter.rs`](../crates/rust_robotics_localization/src/particle_filter.rs) | ✅ Ported | — |
| [`graph_optimization/graph_solver.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/graph_optimization/graph_solver.py) | Variables, factors, Hessian assembly, nonlinear solve | [`optimization/graph.rs`](../crates/rust_robotics_optimization/src/graph.rs), [`solver.rs`](../crates/rust_robotics_optimization/src/solver.rs), [`sparse.rs`](../crates/rust_robotics_optimization/src/sparse.rs) | ✅ Ported | SE(3) graph |
| [`demo_pose2d_graph.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/graph_optimization/demo_pose2d_graph.py) | SE(2) prior/between factors and pose graph | [`pose_graph_optimization.rs`](../crates/rust_robotics_slam/src/pose_graph_optimization.rs) | ✅ Ported | — |
| [`demo_pose3d_graph.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/graph_optimization/demo_pose3d_graph.py) | SE(3) prior/between factors and pose graph | [`pose_graph_optimization_3d.rs`](../crates/rust_robotics_slam/src/pose_graph_optimization_3d.rs) | ✅ Ported | ✅ 12-node loop |
| [`demo_g2o_se2.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/graph_optimization/demo_g2o_se2.py), [`demo_g2o_se3.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/graph_optimization/demo_g2o_se3.py) | Solve imported SE(2)/SE(3) g2o graphs | [`g2o.rs`](../crates/rust_robotics_slam/src/g2o.rs), pose-graph modules above | ✅ Ported | — |
| [`imls/imls.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/imls/imls.py) | PCA normals and 2D IMLS surface projection | [`mapping/imls.rs`](../crates/rust_robotics_mapping/src/imls.rs) | ✅ Ported | — |
| [`imu_preintegration/preintegration.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/imu_preintegration/preintegration.py) | Navigation state, bias-aware IMU preintegration and prediction | [`imu_preintegration.rs`](../crates/rust_robotics_slam/src/imu_preintegration.rs) | ✅ Ported | ✅ Integration/prediction |
| [`imu_preintegration/imu_factor.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/imu_preintegration/imu_factor.py) | IMU, navigation-state, bias, bias-change, position/velocity and transform factors | [`ImuFactor`, `NavStatePriorFactor`, `BiasPriorFactor`, `BiasBetweenFactor`, `PositionVelocityFactor`, `NavStateBetweenFactor`](../crates/rust_robotics_slam/src/imu_preintegration.rs) | ✅ Ported | Jacobians + IMU core |
| [`kinematics/transfrom_velocity.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/kinematics/transfrom_velocity.py) | 2D/3D rigid-body velocity transforms | [`se2_adjoint`, `se3_adjoint`](../crates/rust_robotics_core/src/lie.rs) | ✅ Ported | — |
| [`kinematics/transfrom_imu.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/kinematics/transfrom_imu.py) | IMU frame transform with centripetal and tangential lever-arm acceleration | [`ImuExtrinsics::transform`](../crates/rust_robotics_slam/src/imu_preintegration.rs) | ✅ Ported | ✅ Frame transform |
| [`utilities/math_tools.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/math_tools.py) | Skew/unskew, SO(2), SE(2), SO(3), SE(3), inverses and Jacobians | [`core/lie.rs`](../crates/rust_robotics_core/src/lie.rs) | ✅ Ported | ✅ SE(3) exp/log |
| [`utilities/math_tools.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/math_tools.py) | Numerical derivative and higher SO(3) differential helpers (`HSO3`, `dHinvSO3`, `dLogSO3`) | SO(3) left Jacobian and inverse in [`core/lie.rs`](../crates/rust_robotics_core/src/lie.rs); factor-specific finite-difference tests | 🟡 Partial | — |
| [`optimization/gauss_newton.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/optimization/gauss_newton.py) | Gauss–Newton nonlinear least squares | [`optimization/solver.rs`](../crates/rust_robotics_optimization/src/solver.rs) | ✅ Ported | Through graph case |
| [`utilities/robust_kernel.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/robust_kernel.py) | L2, L1, L4, Huber, Pseudo-Huber, Cauchy and Gaussian kernels | [`optimization/loss.rs`](../crates/rust_robotics_optimization/src/loss.rs): L2, Huber, Pseudo-Huber, Cauchy | 🟡 Partial | — |
| [`robot_geometry/basic_geometry.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/robot_geometry/basic_geometry.py) | Line/plane estimation and point residuals | [`mapping/line_extraction.rs`](../crates/rust_robotics_mapping/src/line_extraction.rs), [`normal_vector_estimation.rs`](../crates/rust_robotics_mapping/src/normal_vector_estimation.rs), [`geometric_icp.rs`](../crates/rust_robotics_slam/src/geometric_icp.rs) | ✅ Ported | — |
| [`demo_p2line_matching.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/robot_geometry/demo_p2line_matching.py) | Point-to-line ICP | [`point_to_line_icp_2d`](../crates/rust_robotics_slam/src/geometric_icp.rs) | ✅ Ported | — |
| [`demo_p2plane_matching.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/robot_geometry/demo_p2plane_matching.py) | Point-to-plane ICP | [`point_to_plane_icp_3d`](../crates/rust_robotics_slam/src/geometric_icp.rs) | ✅ Ported | — |
| [`demo_plane_cross_cube.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/robot_geometry/demo_plane_cross_cube.py) | Plane/cube intersection polygon | — | ⬜ Not ported | — |
| [`slam/projection.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/slam/projection.py) | Pose/point transforms, pinhole projection and reprojection factors | [`bundle_adjustment.rs`](../crates/rust_robotics_slam/src/bundle_adjustment.rs), [`core/lie.rs`](../crates/rust_robotics_core/src/lie.rs) | ✅ Ported | ✅ Reprojection |
| [`slam/demo_bundle_adjustment.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/slam/demo_bundle_adjustment.py) | Joint camera/landmark bundle adjustment | [`bundle_adjustment.rs`](../crates/rust_robotics_slam/src/bundle_adjustment.rs) | ✅ Ported | Projection core |
| [`slam/load_ba_datasets.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/slam/load_ba_datasets.py) | BAL dataset loader | — | ⬜ Not ported | — |
| [`utilities/math_tools.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/math_tools.py) | Quaternion/matrix conversion | `nalgebra::UnitQuaternion`, used by [`g2o.rs`](../crates/rust_robotics_slam/src/g2o.rs) and [`dataset.rs`](../crates/rust_robotics_slam/src/dataset.rs) | ✅ Ported | Round-trip tests |
| [`utilities/pcd_io.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/pcd_io.py) | ASCII/binary PCD loader | — | ⬜ Not ported | — |
| [`utilities/polygon.py`](https://github.com/scomup/MathematicalRobotics/blob/79600010f0c86179905a6960e5fce2bb7cc85d77/mathR/utilities/polygon.py) | Point-in-polygon and polygon boundary residual | Private point-in-polygon helper in [`grid_based_sweep_cpp.rs`](../crates/rust_robotics_planning/src/grid_based_sweep_cpp.rs); no reusable boundary-residual API | 🟡 Partial | — |
| Plotting, Qt/OpenGL GUI and drawing helpers | Interactive visualization | Rust-native visualization and generated SVG/GIF examples exist, but exact GUIs are not port targets | ➖ Support | — |
| Bundled BAL, g2o, PCD and NumPy samples | Demo/test input data | RustRobotics uses generated fixtures and EuRoC/KITTI loaders; upstream assets are not copied | ➖ Support | — |

The pinned numerical checks and their tolerances are documented in the
[MathematicalRobotics numerical parity report](./mathematical-robotics-parity.md).

## Remaining port queue

1. Add BAL and PCD readers.
2. Add the remaining robust kernels and reusable polygon-boundary residual.
3. Add plane/cube intersection if it is needed outside the original demo.
