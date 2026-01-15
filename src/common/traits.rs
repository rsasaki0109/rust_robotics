//! Common traits defining interfaces for robotics algorithms

use crate::common::types::*;
use crate::common::error::RoboticsError;

/// Trait for path planning algorithms
pub trait PathPlanner {
    /// Plan a path from start to goal
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError>;
}

/// Trait for grid-based path planning algorithms
pub trait GridPathPlanner {
    /// Plan a path on a grid from start to goal
    fn plan(&self, start: GridNode, goal: GridNode) -> Result<Path2D, RoboticsError>;

    /// Set obstacles for the planner
    fn set_obstacles(&mut self, obstacles: &Obstacles);
}

/// Trait for sampling-based path planning algorithms (RRT, PRM, etc.)
pub trait SamplingBasedPlanner: PathPlanner {
    /// Get the tree/graph built during planning
    fn get_tree(&self) -> &[(Point2D, Option<usize>)];

    /// Set maximum iterations for planning
    fn set_max_iterations(&mut self, max_iter: usize);
}

/// Trait for state estimation algorithms (EKF, UKF, Particle Filter, etc.)
pub trait StateEstimator {
    /// State type used by this estimator
    type State;
    /// Measurement type used by this estimator
    type Measurement;
    /// Control input type
    type Control;

    /// Prediction step
    fn predict(&mut self, control: &Self::Control, dt: f64);

    /// Update step with measurement
    fn update(&mut self, measurement: &Self::Measurement);

    /// Get current state estimate
    fn get_state(&self) -> &Self::State;

    /// Get current covariance estimate (if applicable)
    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        None
    }
}

/// Simplified state estimator for 2D localization
pub trait Estimator2D {
    /// Prediction step
    fn predict(&mut self, control: &ControlInput, dt: f64);

    /// Update step with position measurement
    fn update(&mut self, measurement: Point2D);

    /// Get current state estimate
    fn get_state(&self) -> State2D;
}

/// Trait for path tracking/following algorithms
pub trait PathTracker {
    /// Compute control input to follow the path
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput;

    /// Check if the goal has been reached
    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool;
}

/// Trait for trajectory tracking with time-parameterized paths
pub trait TrajectoryTracker {
    /// Trajectory point with time
    type TrajectoryPoint;

    /// Compute control input to follow the trajectory at given time
    fn compute_control(&mut self, current_state: &State2D, time: f64) -> ControlInput;
}

/// Trait for vehicle/robot motion models
pub trait MotionModel {
    /// State type
    type State;
    /// Control type
    type Control;

    /// Propagate state forward in time
    fn propagate(&self, state: &Self::State, control: &Self::Control, dt: f64) -> Self::State;

    /// Compute Jacobian with respect to state (for EKF)
    fn jacobian_state(&self, state: &Self::State, control: &Self::Control, dt: f64)
        -> nalgebra::DMatrix<f64>;
}

/// Trait for observation/measurement models
pub trait ObservationModel {
    /// State type
    type State;
    /// Measurement type
    type Measurement;

    /// Predict measurement from state
    fn predict(&self, state: &Self::State) -> Self::Measurement;

    /// Compute Jacobian with respect to state (for EKF)
    fn jacobian(&self, state: &Self::State) -> nalgebra::DMatrix<f64>;
}

/// Trait for controllers (PID, LQR, MPC, etc.)
pub trait Controller {
    /// State type
    type State;
    /// Reference/target type
    type Reference;
    /// Output control type
    type Output;

    /// Compute control output
    fn compute(&mut self, state: &Self::State, reference: &Self::Reference) -> Self::Output;

    /// Reset controller state
    fn reset(&mut self);
}

/// Trait for visualizable algorithms
pub trait Visualizable {
    /// Draw current state to visualizer
    fn visualize(&self, vis: &mut crate::utils::Visualizer);
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test that traits compile correctly
    struct DummyPlanner;

    impl PathPlanner for DummyPlanner {
        fn plan(&self, _start: Point2D, _goal: Point2D) -> Result<Path2D, RoboticsError> {
            Ok(Path2D::new())
        }
    }

    #[test]
    fn test_path_planner_trait() {
        let planner = DummyPlanner;
        let result = planner.plan(Point2D::origin(), Point2D::new(1.0, 1.0));
        assert!(result.is_ok());
    }
}
