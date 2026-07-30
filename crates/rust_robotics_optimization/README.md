# rust_robotics_optimization

Dense nonlinear least-squares building blocks for RustRobotics, including
factor graphs, robust loss functions, Gauss-Newton, and
Levenberg-Marquardt. See the workspace README for examples.

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
