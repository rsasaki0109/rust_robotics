# rust_robotics

Umbrella crate for the RustRobotics workspace. It exposes feature-gated re-exports
for planning, localization, mapping, SLAM, control, and visualization crates.

```toml
[dependencies]
rust_robotics = "0.1"
```

```rust
use rust_robotics::planning::{AStarConfig, AStarPlanner};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
```

Project site: https://rsasaki0109.github.io/rust_robotics/

Repository: https://github.com/rsasaki0109/rust_robotics
