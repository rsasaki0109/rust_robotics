# Adaptive Costmap-NAMO Slice

This pure-Rust slice reproduces the planning-side idea from adaptive
costmap-based navigation among movable obstacles:

1. Keep semantic cell states: free, unknown, static obstacle, movable obstacle.
2. Treat movable obstacles as soft costs at first.
3. Raise movable-obstacle costs when commanded motion produces little odom
   progress.
4. Decay costs when the robot makes progress again.
5. Convert the adaptive costmap to traversal-risk cells for graph planning.

Run:

```bash
cargo run -p rust_robotics --example headless_adaptive_costmap_namo --no-default-features --features planning
```

The demo starts with a short center route through soft movable-obstacle costs.
After repeated stuck observations, those movable cells become lethal and the
same planning query detours around them. A progress observation then decays the
costs back toward the soft movable baseline.

This is intentionally not a ROS2/Nav2 plugin. It is the deterministic cost
adaptation core that can be tested without middleware.
