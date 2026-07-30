# rust_robotics_core

Core types, traits, error definitions, and `no_std` Lie-group mathematics
shared by the RustRobotics domain crates.

```toml
[dependencies]
rust_robotics_core = "0.2"
```

Use this crate when building integrations that need common geometry, pose, and
workspace-level abstractions without pulling in planner or filter implementations.

The `lie` module provides SO(2), SE(2), SO(3), and SE(3) exponential/logarithm
maps, inverse transforms, adjoints, skew operations, and stable SO(3) left
Jacobians. SE(2)/SE(3) tangent vectors use translation-first ordering.

```rust
use rust_robotics_core::{se3_exp, se3_log, Vector6};

let xi = Vector6::new(1.0, 2.0, 3.0, 0.1, 0.2, -0.1);
assert!((se3_log(&se3_exp(&xi)) - xi).norm() < 1.0e-9);
```

Project site: https://rsasaki0109.github.io/rust_robotics/

Repository: https://github.com/rsasaki0109/rust_robotics
