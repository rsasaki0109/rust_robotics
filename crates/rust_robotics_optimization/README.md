# rust_robotics_optimization

Nonlinear least-squares building blocks for RustRobotics, including factor
graphs, robust loss functions, Gauss-Newton, and Levenberg-Marquardt. Linear
systems can use dense LU, block-sparse PCG, or trailing-block Schur elimination.
See the workspace README for examples and scaling results.

```toml
[dependencies]
rust_robotics_optimization = "0.2"
```

Variables may use Euclidean addition or a custom manifold retraction. Factors
return residuals, information matrices, and Jacobian blocks. Available robust
losses are L2, Huber, Pseudo-Huber, and Cauchy.

```bash
cargo test -p rust_robotics_optimization
```
