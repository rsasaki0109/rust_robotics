//! Motion model for State Lattice Planner
//!
//! Implements a bicycle kinematic model for vehicle motion.
//! Based on PythonRobotics implementation.

use std::f64::consts::PI;

/// Vehicle state
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
}

impl VehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64) -> Self {
        Self { x, y, yaw, v }
    }

    pub fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            v: 0.0,
        }
    }
}

/// Motion model configuration
#[derive(Debug, Clone)]
pub struct MotionModelConfig {
    /// Wheelbase length [m]
    pub wheelbase: f64,
    /// Distance step for trajectory generation [m]
    pub ds: f64,
    /// Default velocity [m/s]
    pub default_velocity: f64,
}

impl Default for MotionModelConfig {
    fn default() -> Self {
        Self {
            wheelbase: 1.0,
            ds: 0.1,
            default_velocity: 10.0 / 3.6, // ~2.78 m/s
        }
    }
}

/// Bicycle kinematic motion model
pub struct MotionModel {
    config: MotionModelConfig,
}

impl MotionModel {
    pub fn new(config: MotionModelConfig) -> Self {
        Self { config }
    }

    pub fn with_defaults() -> Self {
        Self::new(MotionModelConfig::default())
    }

    /// Get wheelbase
    pub fn wheelbase(&self) -> f64 {
        self.config.wheelbase
    }

    /// Get distance step
    pub fn ds(&self) -> f64 {
        self.config.ds
    }

    /// Update state using bicycle kinematic model
    ///
    /// # Arguments
    /// * `state` - Current vehicle state
    /// * `v` - Linear velocity [m/s]
    /// * `delta` - Steering angle [rad]
    /// * `dt` - Time step [s]
    ///
    /// # Returns
    /// Updated vehicle state
    pub fn update(&self, state: &VehicleState, v: f64, delta: f64, dt: f64) -> VehicleState {
        let l = self.config.wheelbase;

        VehicleState {
            x: state.x + v * state.yaw.cos() * dt,
            y: state.y + v * state.yaw.sin() * dt,
            yaw: state.yaw + v / l * delta.tan() * dt,
            v,
        }
    }

    /// Generate trajectory given curvature parameters
    ///
    /// # Arguments
    /// * `s` - Arc length [m]
    /// * `k0` - Initial curvature [1/m]
    /// * `km` - Middle curvature [1/m]
    /// * `kf` - Final curvature [1/m]
    ///
    /// # Returns
    /// Tuple of (x_coords, y_coords, yaw_angles)
    pub fn generate_trajectory(
        &self,
        s: f64,
        k0: f64,
        km: f64,
        kf: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = (s / self.config.ds) as usize;
        if n == 0 {
            return (vec![0.0], vec![0.0], vec![0.0]);
        }

        let ds = self.config.ds;
        let v = self.config.default_velocity;
        let dt = ds / v;

        let mut x_list = vec![0.0];
        let mut y_list = vec![0.0];
        let mut yaw_list = vec![0.0];

        let mut state = VehicleState::origin();

        for i in 0..n {
            // Interpolate curvature using quadratic interpolation
            let t = i as f64 / n as f64;
            let k = self.interpolate_curvature(t, k0, km, kf);

            // Convert curvature to steering angle: delta = atan(L * k)
            let delta = (self.config.wheelbase * k).atan();

            state = self.update(&state, v, delta, dt);

            x_list.push(state.x);
            y_list.push(state.y);
            yaw_list.push(state.yaw);
        }

        (x_list, y_list, yaw_list)
    }

    /// Interpolate curvature using quadratic polynomial
    /// k(t) = a*t^2 + b*t + c where t in [0, 1]
    /// Boundary conditions: k(0)=k0, k(0.5)=km, k(1)=kf
    fn interpolate_curvature(&self, t: f64, k0: f64, km: f64, kf: f64) -> f64 {
        // Solve for coefficients:
        // c = k0
        // a + b + c = kf
        // 0.25*a + 0.5*b + c = km
        //
        // From these:
        // a + b = kf - k0
        // 0.25*a + 0.5*b = km - k0
        //
        // a = 2*(kf + k0 - 2*km)
        // b = kf - k0 - a = -kf - 3*k0 + 4*km

        let a = 2.0 * (kf + k0 - 2.0 * km);
        let b = -kf - 3.0 * k0 + 4.0 * km;
        let c = k0;

        a * t * t + b * t + c
    }

    /// Generate trajectory and return final state
    pub fn generate_trajectory_final_state(
        &self,
        s: f64,
        k0: f64,
        km: f64,
        kf: f64,
    ) -> (f64, f64, f64) {
        let (x, y, yaw) = self.generate_trajectory(s, k0, km, kf);
        let n = x.len();
        if n == 0 {
            (0.0, 0.0, 0.0)
        } else {
            (x[n - 1], y[n - 1], yaw[n - 1])
        }
    }
}

/// Normalize angle to [-PI, PI]
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vehicle_state() {
        let state = VehicleState::new(1.0, 2.0, 0.5, 1.0);
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.5);
    }

    #[test]
    fn test_motion_model_update() {
        let model = MotionModel::with_defaults();
        let state = VehicleState::origin();
        let next = model.update(&state, 1.0, 0.0, 0.1);

        // Moving straight at 1 m/s for 0.1s should move 0.1m forward
        assert!((next.x - 0.1).abs() < 1e-10);
        assert!(next.y.abs() < 1e-10);
        assert!(next.yaw.abs() < 1e-10);
    }

    #[test]
    fn test_curvature_interpolation() {
        let model = MotionModel::with_defaults();

        // Test boundary conditions
        let k0 = 0.0;
        let km = 0.1;
        let kf = 0.2;

        let k_start = model.interpolate_curvature(0.0, k0, km, kf);
        let k_mid = model.interpolate_curvature(0.5, k0, km, kf);
        let k_end = model.interpolate_curvature(1.0, k0, km, kf);

        assert!((k_start - k0).abs() < 1e-10);
        assert!((k_mid - km).abs() < 1e-10);
        assert!((k_end - kf).abs() < 1e-10);
    }

    #[test]
    fn test_generate_trajectory_straight() {
        let model = MotionModel::with_defaults();
        let (x, y, yaw) = model.generate_trajectory(1.0, 0.0, 0.0, 0.0);

        // Straight line should have y ≈ 0
        assert!(x.len() > 1);
        for yi in &y {
            assert!(yi.abs() < 1e-6);
        }
    }

    #[test]
    fn test_generate_trajectory_turn() {
        let model = MotionModel::with_defaults();
        let (x, y, _yaw) = model.generate_trajectory(2.0, 0.1, 0.1, 0.1);

        // Constant positive curvature should turn left (positive y)
        assert!(x.len() > 1);
        let final_y = y.last().unwrap();
        assert!(*final_y > 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 1e-10);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }
}
