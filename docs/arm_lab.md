# Arm lab

[Open the arm lab](https://rsasaki0109.github.io/rust_robotics/playground/?tab=arm).
It runs the Rust library's two-link arm in WebAssembly.

![Current arm, two IK solutions, and the reachable workspace](assets/playground/arm.png)

## Try it

1. Choose **Reach**, then **Snap**. The cyan arm reaches the gold target.
2. Choose **Bend -**, then **Snap** again. The elbow changes sides while the
   tip reaches the same point. **Other solution** shows the alternate pose.
3. Choose **Reach**, then **Play**. Drag the target; watch the joint controller
   follow it and leave a cyan tip trail. **Pause** freezes motion, and
   **Step 0.05 s** advances one controller step.
4. Choose **Stretch**, then **Snap**. At the 2 m boundary the two IK solutions
   meet and the Jacobian determinant vanishes. **Fold**, then **Snap**, shows
   the other singular case, with the tip at the shoulder.
5. Choose **Outside**. No IK solution exists, so the arm holds its pose.
   Drag the target back inside the disk to make it reachable.
6. Choose **Joints (FK)** and move the two angle sliders. The target stays
   fixed while the actual tip position and error change. **Target (IK)**
   returns to target tracking; **Snap** solves the selected branch.

The scene and controls work with mouse or touch. On phones, scroll below the
scene for the controls. The **Share** button keeps the target, current joint
angles, selected branch, mode, and alternate-pose visibility. Shared links open
paused at that exact pose with a new trail, including manual poses away from
the target.

## Model

The arm has two 1 m links and unrestricted revolute joints. Its reachable
workspace is a radius-2 m disk, with no obstacles or joint limits. Analytic IK
uses both signs of the elbow angle. At the origin there are infinitely many
shoulder orientations; the solver returns shoulder angle zero as a deterministic
representative. At the outer boundary the two branches coincide.

The displayed error is the actual distance between the current tip and target.
`|det J| = |sin(elbow angle)|` in m² describes local Cartesian dexterity;
it is zero when the links are straight or fully folded. It is measured at the
current pose, including during tracking and manual joint edits.

Play integrates a gain-4 proportional joint-angle controller in elapsed time,
using the shortest signed angular errors. It models geometry and kinematic
tracking, without inertia, motor limits, or collision avoidance. Background
gaps of at least one second are discarded. Snap and manual angle edits start
a new trail instead of inventing a motion between teleported poses.

Implementation: [library IK](../crates/rust_robotics_control/src/two_joint_arm_control.rs),
[playground model](../crates/rust_robotics_playground/src/arm.rs), and
[interactive browser checks](../scripts/web_smoke/arm.mjs).

```sh
cargo test --locked -p rust_robotics_control two_joint_arm_control::tests:: --lib
cargo test --locked -p rust_robotics_playground arm::tests:: --bin rust_robotics_playground
```
