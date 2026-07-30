# MathematicalRobotics numerical parity

RustRobotics is independently implemented in Rust and numerically compared
against [scomup/MathematicalRobotics](https://github.com/scomup/MathematicalRobotics)
at commit `79600010f0c86179905a6960e5fce2bb7cc85d77`.
Both projects are MIT-licensed.

The golden regression test is
`crates/rust_robotics_slam/tests/mathematical_robotics_parity.rs`.

| Rust operation | Upstream reference | Tolerance |
|---|---|---:|
| `se3_exp` / `se3_log` | `mathR/utilities/math_tools.py` | `2e-12` |
| IMU integration and prediction | `mathR/imu_preintegration/preintegration.py` | `1e-6` |
| pinhole reprojection | `mathR/slam/projection.py` | `2e-12` |
| 12-node SE(3) loop closure optimum | `mathR/graph_optimization/demo_pose3d_graph.py` | `2e-4` |

The IMU tolerance is intentionally wider. MathematicalRobotics switches to
`I + skew(ω)` when `|ω|² <= 1e-5`; RustRobotics retains stable Rodrigues
second-order terms. Ten 10 ms steps consequently differ below one microradian,
while the Rust rotation remains more nearly orthonormal.

```bash
cargo test -p rust_robotics_slam --test mathematical_robotics_parity
```

Golden values were produced by checking out the pinned upstream commit,
setting `PYTHONPATH` to that checkout, and evaluating the exact vectors listed
in the test through its `expSE3`, `ImuIntegration`, `reproj_error`, and
`GraphSolver` APIs. Changing the pinned commit or a test vector requires
regenerating the values and documenting the numerical difference.
