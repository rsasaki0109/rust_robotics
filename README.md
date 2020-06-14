RustRobotics
====

This package is a rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics).

Build
```
git clone https://github.com/rsasaki0109/RustRobotics.git
cd RustRobotics
cargo build
```

Run (Example)
```
cargo run --bin ekf
```

# Table of Contents
   * [Localization](#localization)
      * [Extended Kalman Filter localization](#extended-kalman-filter-localization)
      * Particle filter localization
   * [SLAM](#slam)
      * Iterative Closest Point 
      * FastSLAM 1.0
   * [Path Planning](#path-planning)
      * [Dynamic Window Approach](#dynamic-window-approach)
      * Dijkstra algorithm
      * Potential Field algorithm
      * State Lattice Planner
      * Rapidly-Exploring Random Trees (RRT)
   * [Path Tracking](#path-tracking)
      * [Pure Pursuit](#pure-pursuit)
      * Linear–quadratic regulator (LQR) speed and steering control
      * Nonlinear Model predictive control with C-GMRES

# Localization
## Extended Kalman Filter localization
<img src="./img/ekf.svg" width="640px">   

Red:GPS, Brue:Ground Truth, Green:EKF, Yellow:Dead Reckoning

## Particle Filter localization

# SLAM
## FastSLAM 1.0


# Path Planning
## Dynamic Window Approach

<img src="./img/dwa.svg" width="640px">  

Brack: Obstacles, Green: Trajectry, Yellow: Predected trajectry
# Path Tracking
## Pure Pursuit

<img src="./img/pure_pursuit.svg" width="640px">  

Brack: Planned path, Green: Tracked path
## Linear–quadratic regulator (LQR) speed and steering control


