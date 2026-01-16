// EKF SLAM (Extended Kalman Filter SLAM)
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port
//
// Reference:
// - Probabilistic Robotics (Thrun, Burgard, Fox)
// - https://github.com/AtsushiSakai/PythonRobotics

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSize, PointSymbol};
use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, Vector2, Vector3};
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const SIM_TIME: f64 = 50.0; // simulation time [s]
const MAX_RANGE: f64 = 20.0; // maximum observation range [m]
const M_DIST_TH: f64 = 4.0; // Mahalanobis distance threshold for data association (chi-square 95% for 2 DOF)

// State dimension
const STATE_SIZE: usize = 3; // robot state [x, y, yaw]
const LM_SIZE: usize = 2; // landmark state [x, y]

// Noise parameters
const Q_SIM: [[f64; 2]; 2] = [[0.2, 0.0], [0.0, (5.0 * PI / 180.0) * (5.0 * PI / 180.0)]]; // process noise (reduced)
const R_SIM: [[f64; 2]; 2] = [[0.3, 0.0], [0.0, (5.0 * PI / 180.0) * (5.0 * PI / 180.0)]]; // observation noise (reduced)

const SHOW_ANIMATION: bool = false;

/// Normalize angle to [-pi, pi]
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Process noise covariance for control input
fn get_q_control() -> Matrix2<f64> {
    Matrix2::new(Q_SIM[0][0], Q_SIM[0][1], Q_SIM[1][0], Q_SIM[1][1])
}

/// Observation noise covariance
fn get_r() -> Matrix2<f64> {
    Matrix2::new(R_SIM[0][0], R_SIM[0][1], R_SIM[1][0], R_SIM[1][1])
}

/// EKF SLAM state
/// State vector: [x, y, yaw, lm1_x, lm1_y, lm2_x, lm2_y, ...]
pub struct EKFSLAMState {
    /// State vector
    pub x: DVector<f64>,
    /// Covariance matrix
    pub p: DMatrix<f64>,
    /// Number of observed landmarks
    pub n_lm: usize,
}

impl EKFSLAMState {
    /// Create a new EKF SLAM state
    pub fn new() -> Self {
        EKFSLAMState {
            x: DVector::zeros(STATE_SIZE),
            p: DMatrix::identity(STATE_SIZE, STATE_SIZE),
            n_lm: 0,
        }
    }

    /// Get robot pose [x, y, yaw]
    pub fn get_robot_pose(&self) -> Vector3<f64> {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    /// Get landmark position by index
    pub fn get_landmark(&self, idx: usize) -> Option<Vector2<f64>> {
        if idx < self.n_lm {
            let lm_idx = STATE_SIZE + idx * LM_SIZE;
            Some(Vector2::new(self.x[lm_idx], self.x[lm_idx + 1]))
        } else {
            None
        }
    }

    /// Get number of landmarks
    pub fn n_landmarks(&self) -> usize {
        self.n_lm
    }
}

impl Default for EKFSLAMState {
    fn default() -> Self {
        Self::new()
    }
}

/// Motion model for robot
/// x_t = f(x_{t-1}, u_t)
fn motion_model(x: &Vector3<f64>, u: &Vector2<f64>) -> Vector3<f64> {
    Vector3::new(
        x[0] + u[0] * DT * x[2].cos(),
        x[1] + u[0] * DT * x[2].sin(),
        normalize_angle(x[2] + u[1] * DT),
    )
}

/// Jacobian of motion model with respect to state
fn jacob_motion(x: &Vector3<f64>, u: &Vector2<f64>) -> (Matrix3<f64>, nalgebra::Matrix3x2<f64>) {
    let yaw = x[2];
    let v = u[0];

    // Jacobian with respect to state (G matrix)
    let g = Matrix3::new(
        1.0, 0.0, -DT * v * yaw.sin(),
        0.0, 1.0, DT * v * yaw.cos(),
        0.0, 0.0, 1.0,
    );

    // Jacobian with respect to control (V matrix): 3x2
    let v_mat = nalgebra::Matrix3x2::new(
        DT * yaw.cos(), 0.0,
        DT * yaw.sin(), 0.0,
        0.0, DT,
    );

    (g, v_mat)
}

/// Calculate observation from robot pose to landmark
fn calc_observation(robot_pose: &Vector3<f64>, landmark: &Vector2<f64>) -> Vector2<f64> {
    let dx = landmark[0] - robot_pose[0];
    let dy = landmark[1] - robot_pose[1];
    let d = (dx * dx + dy * dy).sqrt();
    let angle = normalize_angle(dy.atan2(dx) - robot_pose[2]);
    Vector2::new(d, angle)
}

/// Jacobian of observation model with respect to robot pose and landmark position
fn jacob_observation(
    robot_pose: &Vector3<f64>,
    landmark: &Vector2<f64>,
) -> (nalgebra::Matrix2x3<f64>, Matrix2<f64>) {
    let dx = landmark[0] - robot_pose[0];
    let dy = landmark[1] - robot_pose[1];
    let d2 = dx * dx + dy * dy;
    let d = d2.sqrt();

    // Jacobian with respect to robot pose [x, y, yaw]
    let h_robot = nalgebra::Matrix2x3::new(
        -dx / d, -dy / d, 0.0,
        dy / d2, -dx / d2, -1.0,
    );

    // Jacobian with respect to landmark position [lm_x, lm_y]
    let h_lm = Matrix2::new(
        dx / d, dy / d,
        -dy / d2, dx / d2,
    );

    (h_robot, h_lm)
}

/// EKF SLAM prediction step
fn ekf_slam_predict(state: &mut EKFSLAMState, u: &Vector2<f64>) {
    let n = state.x.len();

    // Get current robot pose
    let robot_pose = state.get_robot_pose();

    // Jacobians (computed before state update, using current state)
    let (g, v_mat) = jacob_motion(&robot_pose, u);

    // Predict robot pose using motion model
    let new_pose = motion_model(&robot_pose, u);
    state.x[0] = new_pose[0];
    state.x[1] = new_pose[1];
    state.x[2] = new_pose[2];

    // Process noise in control space
    let q_control = get_q_control();

    // Process noise in state space (robot part only): V * Q * V^T (3x2 * 2x2 * 2x3 = 3x3)
    let q_robot = v_mat * q_control * v_mat.transpose();

    // Update covariance following EKF formula:
    // P_new = F @ P @ F^T + Q_augmented
    // where F is the Jacobian of full state transition (identity for landmarks)

    // For EKF-SLAM, we need to propagate covariance properly:
    // P_rr = G @ P_rr @ G^T + Q
    // P_rm = G @ P_rm (cross-covariance between robot and landmarks)
    // P_mr = P_rm^T
    // P_mm = P_mm (landmarks covariance unchanged)

    // Extract robot covariance (3x3)
    let mut p_rr = Matrix3::zeros();
    for i in 0..STATE_SIZE {
        for j in 0..STATE_SIZE {
            p_rr[(i, j)] = state.p[(i, j)];
        }
    }

    // Update P_rr
    let p_rr_new = g * p_rr * g.transpose() + q_robot;

    // Update state covariance
    for i in 0..STATE_SIZE {
        for j in 0..STATE_SIZE {
            state.p[(i, j)] = p_rr_new[(i, j)];
        }
    }

    // Update cross-covariance P_rm = G @ P_rm
    for lm in 0..state.n_lm {
        let lm_idx = STATE_SIZE + lm * LM_SIZE;

        // Extract P_rm for this landmark (3x2)
        let mut p_rm = nalgebra::Matrix3x2::zeros();
        for i in 0..STATE_SIZE {
            for j in 0..LM_SIZE {
                p_rm[(i, j)] = state.p[(i, lm_idx + j)];
            }
        }

        // Update: P_rm_new = G @ P_rm
        let p_rm_new = g * p_rm;

        // Write back
        for i in 0..STATE_SIZE {
            for j in 0..LM_SIZE {
                state.p[(i, lm_idx + j)] = p_rm_new[(i, j)];
                state.p[(lm_idx + j, i)] = p_rm_new[(i, j)]; // Keep symmetry
            }
        }
    }
}

/// Calculate innovation (measurement residual) for a landmark observation
fn calc_innovation(
    state: &EKFSLAMState,
    lm_idx: usize,
    z: &Vector2<f64>,
) -> (Vector2<f64>, Matrix2<f64>, DMatrix<f64>) {
    let robot_pose = state.get_robot_pose();
    let landmark = state.get_landmark(lm_idx).unwrap();

    // Predicted observation
    let z_pred = calc_observation(&robot_pose, &landmark);

    // Innovation
    let y = Vector2::new(z[0] - z_pred[0], normalize_angle(z[1] - z_pred[1]));

    // Jacobians
    let (h_robot, h_lm) = jacob_observation(&robot_pose, &landmark);

    // Build full Jacobian H
    let n = state.x.len();
    let mut h_full = DMatrix::zeros(2, n);

    // Robot part
    for i in 0..2 {
        for j in 0..STATE_SIZE {
            h_full[(i, j)] = h_robot[(i, j)];
        }
    }

    // Landmark part
    let lm_state_idx = STATE_SIZE + lm_idx * LM_SIZE;
    for i in 0..2 {
        for j in 0..LM_SIZE {
            h_full[(i, lm_state_idx + j)] = h_lm[(i, j)];
        }
    }

    // Innovation covariance
    let r = get_r();
    let s = &h_full * &state.p * h_full.transpose()
        + DMatrix::from_fn(2, 2, |i, j| r[(i, j)]);

    (y, Matrix2::new(s[(0, 0)], s[(0, 1)], s[(1, 0)], s[(1, 1)]), h_full)
}

/// Search for corresponding landmark using Mahalanobis distance
fn search_correspond_landmark_id(state: &EKFSLAMState, z: &Vector2<f64>) -> Option<usize> {
    let mut min_dist = f64::MAX;
    let mut min_id = None;

    for i in 0..state.n_lm {
        let (y, s, _) = calc_innovation(state, i, z);

        // Mahalanobis distance
        if let Some(s_inv) = s.try_inverse() {
            let mahal = (y.transpose() * s_inv * y)[(0, 0)];
            if mahal < min_dist {
                min_dist = mahal;
                min_id = Some(i);
            }
        }
    }

    // Return match only if below threshold
    if min_dist < M_DIST_TH * M_DIST_TH {
        min_id
    } else {
        None
    }
}

/// Add a new landmark to the state
fn add_new_landmark(state: &mut EKFSLAMState, z: &Vector2<f64>) {
    let robot_pose = state.get_robot_pose();

    // Calculate landmark position from observation
    let lm_x = robot_pose[0] + z[0] * (robot_pose[2] + z[1]).cos();
    let lm_y = robot_pose[1] + z[0] * (robot_pose[2] + z[1]).sin();

    // Extend state vector
    let old_n = state.x.len();
    let new_n = old_n + LM_SIZE;

    let mut new_x = DVector::zeros(new_n);
    for i in 0..old_n {
        new_x[i] = state.x[i];
    }
    new_x[old_n] = lm_x;
    new_x[old_n + 1] = lm_y;
    state.x = new_x;

    // Extend covariance matrix
    let mut new_p = DMatrix::zeros(new_n, new_n);

    // Copy old covariance
    for i in 0..old_n {
        for j in 0..old_n {
            new_p[(i, j)] = state.p[(i, j)];
        }
    }

    // Initialize new landmark covariance with large uncertainty
    let r = get_r();

    // Jacobian of landmark initialization with respect to robot pose and observation
    let c = (robot_pose[2] + z[1]).cos();
    let s = (robot_pose[2] + z[1]).sin();

    // G_r: Jacobian w.r.t. robot pose [x, y, yaw]
    let g_r = nalgebra::Matrix2x3::new(
        1.0, 0.0, -z[0] * s,
        0.0, 1.0, z[0] * c,
    );

    // G_z: Jacobian w.r.t. observation [d, angle]
    let g_z = Matrix2::new(
        c, -z[0] * s,
        s, z[0] * c,
    );

    // Initial landmark covariance
    let p_rr = state.p.fixed_view::<3, 3>(0, 0);
    let p_lm = g_r * p_rr * g_r.transpose() + g_z * r * g_z.transpose();

    // Set landmark-landmark covariance
    new_p[(old_n, old_n)] = p_lm[(0, 0)];
    new_p[(old_n, old_n + 1)] = p_lm[(0, 1)];
    new_p[(old_n + 1, old_n)] = p_lm[(1, 0)];
    new_p[(old_n + 1, old_n + 1)] = p_lm[(1, 1)];

    // Cross-covariance between robot and new landmark
    let p_rl = p_rr * g_r.transpose();
    for i in 0..STATE_SIZE {
        for j in 0..LM_SIZE {
            new_p[(i, old_n + j)] = p_rl[(i, j)];
            new_p[(old_n + j, i)] = p_rl[(i, j)];
        }
    }

    // Cross-covariance between existing landmarks and new landmark
    for k in 0..state.n_lm {
        let lm_idx = STATE_SIZE + k * LM_SIZE;
        for i in 0..LM_SIZE {
            for j in 0..STATE_SIZE {
                let p_lk_r = state.p[(lm_idx + i, j)];
                for l in 0..LM_SIZE {
                    new_p[(lm_idx + i, old_n + l)] += p_lk_r * g_r[(l, j)];
                    new_p[(old_n + l, lm_idx + i)] = new_p[(lm_idx + i, old_n + l)];
                }
            }
        }
    }

    state.p = new_p;
    state.n_lm += 1;
}

/// EKF SLAM update step for a single observation
fn ekf_slam_update(state: &mut EKFSLAMState, z: &Vector2<f64>, lm_idx: usize) {
    let (y, s, h_full) = calc_innovation(state, lm_idx, z);

    // Kalman gain
    let s_dmatrix = DMatrix::from_fn(2, 2, |i, j| s[(i, j)]);
    let s_inv = s_dmatrix.try_inverse().unwrap_or_else(|| DMatrix::identity(2, 2));
    let k = &state.p * h_full.transpose() * s_inv;

    // State update
    let y_dvec = DVector::from_vec(vec![y[0], y[1]]);
    state.x = &state.x + &k * y_dvec;

    // Normalize yaw
    state.x[2] = normalize_angle(state.x[2]);

    // Covariance update
    let n = state.x.len();
    let i_kh = DMatrix::identity(n, n) - &k * h_full;
    state.p = &i_kh * &state.p;

    // Ensure symmetry
    state.p = (&state.p + state.p.transpose()) * 0.5;
}

/// Full EKF SLAM step (prediction + update) with unknown data association
pub fn ekf_slam(
    state: &mut EKFSLAMState,
    u: &Vector2<f64>,
    observations: &[(f64, f64)], // (distance, angle)
) {
    // Prediction step
    ekf_slam_predict(state, u);

    // Update step for each observation
    for (d, angle) in observations {
        let z = Vector2::new(*d, *angle);

        // Data association
        let lm_idx = search_correspond_landmark_id(state, &z);

        match lm_idx {
            Some(idx) => {
                // Update existing landmark
                ekf_slam_update(state, &z, idx);
            }
            None => {
                // Add new landmark
                add_new_landmark(state, &z);
            }
        }
    }
}

/// Full EKF SLAM step (prediction + update) with known data association
/// This version uses landmark IDs from observations (ideal case)
pub fn ekf_slam_known_correspondences(
    state: &mut EKFSLAMState,
    u: &Vector2<f64>,
    observations: &[(f64, f64, usize)], // (distance, angle, landmark_id)
    n_landmarks: usize,
) {
    // Prediction step
    ekf_slam_predict(state, u);

    // Pre-allocate space for all landmarks on first observation
    if state.n_lm == 0 && !observations.is_empty() {
        // Initialize state to hold all potential landmarks
        let n = STATE_SIZE + n_landmarks * LM_SIZE;
        let mut new_x = DVector::zeros(n);
        for i in 0..STATE_SIZE {
            new_x[i] = state.x[i];
        }
        // Initialize landmarks to (0, 0) with large covariance
        state.x = new_x;

        let mut new_p = DMatrix::identity(n, n) * 1e6; // Large initial uncertainty
        // Copy robot covariance
        for i in 0..STATE_SIZE {
            for j in 0..STATE_SIZE {
                new_p[(i, j)] = state.p[(i, j)];
            }
        }
        state.p = new_p;
        state.n_lm = n_landmarks;
    }

    // Update step for each observation
    for (d, angle, lm_id) in observations {
        let z = Vector2::new(*d, *angle);

        // Check if this is a valid landmark ID
        if *lm_id < state.n_lm {
            // Check if landmark is initialized (covariance is reasonable)
            let lm_idx = STATE_SIZE + lm_id * LM_SIZE;
            if state.p[(lm_idx, lm_idx)] > 1e5 {
                // First observation of this landmark - initialize it
                let robot_pose = state.get_robot_pose();
                let lm_x = robot_pose[0] + z[0] * (robot_pose[2] + z[1]).cos();
                let lm_y = robot_pose[1] + z[0] * (robot_pose[2] + z[1]).sin();
                state.x[lm_idx] = lm_x;
                state.x[lm_idx + 1] = lm_y;

                // Initialize with observation covariance
                let r = get_r();
                let c = (robot_pose[2] + z[1]).cos();
                let s = (robot_pose[2] + z[1]).sin();
                let g_z = Matrix2::new(c, -z[0] * s, s, z[0] * c);
                let p_lm = g_z * r * g_z.transpose();
                state.p[(lm_idx, lm_idx)] = p_lm[(0, 0)] + 0.1;
                state.p[(lm_idx, lm_idx + 1)] = p_lm[(0, 1)];
                state.p[(lm_idx + 1, lm_idx)] = p_lm[(1, 0)];
                state.p[(lm_idx + 1, lm_idx + 1)] = p_lm[(1, 1)] + 0.1;
            } else {
                // Update existing landmark
                ekf_slam_update(state, &z, *lm_id);
            }
        }
    }
}

/// Simulate observations from true robot pose to landmarks (without IDs)
fn get_observations(x_true: &Vector3<f64>, landmarks: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let normal = Normal::new(0.0, 1.0).unwrap();
    let r = get_r();
    let mut z = Vec::new();

    for (lx, ly) in landmarks.iter() {
        let dx = lx - x_true[0];
        let dy = ly - x_true[1];
        let d = (dx * dx + dy * dy).sqrt();

        if d <= MAX_RANGE {
            let angle = normalize_angle(dy.atan2(dx) - x_true[2]);

            // Add noise
            let d_noisy = d + normal.sample(&mut rand::thread_rng()) * r[(0, 0)].sqrt();
            let angle_noisy = angle + normal.sample(&mut rand::thread_rng()) * r[(1, 1)].sqrt();

            z.push((d_noisy, angle_noisy));
        }
    }

    z
}

/// Simulate observations from true robot pose to landmarks (with IDs - known correspondences)
fn get_observations_with_id(x_true: &Vector3<f64>, landmarks: &[(f64, f64)]) -> Vec<(f64, f64, usize)> {
    let normal = Normal::new(0.0, 1.0).unwrap();
    let r = get_r();
    let mut z = Vec::new();

    for (lm_id, (lx, ly)) in landmarks.iter().enumerate() {
        let dx = lx - x_true[0];
        let dy = ly - x_true[1];
        let d = (dx * dx + dy * dy).sqrt();

        if d <= MAX_RANGE {
            let angle = normalize_angle(dy.atan2(dx) - x_true[2]);

            // Add noise
            let d_noisy = d + normal.sample(&mut rand::thread_rng()) * r[(0, 0)].sqrt();
            let angle_noisy = angle + normal.sample(&mut rand::thread_rng()) * r[(1, 1)].sqrt();

            z.push((d_noisy, angle_noisy, lm_id));
        }
    }

    z
}

fn main() {
    println!("EKF SLAM start!");

    // Landmark positions [x, y]
    let landmarks: Vec<(f64, f64)> = vec![
        (10.0, -2.0),
        (15.0, 10.0),
        (3.0, 15.0),
        (-5.0, 20.0),
        (-5.0, 5.0),
    ];

    let n_landmarks = landmarks.len();

    // Initialize EKF SLAM state
    let mut state = EKFSLAMState::new();

    // True state: [x, y, yaw]
    let mut x_true = Vector3::new(0.0, 0.0, 0.0);
    let mut x_dr = Vector3::new(0.0, 0.0, 0.0); // dead reckoning

    // Control input: [v, yaw_rate]
    let u = Vector2::new(1.0, 0.1);

    // History for plotting
    let mut h_true: Vec<(f64, f64)> = vec![(0.0, 0.0)];
    let mut h_dr: Vec<(f64, f64)> = vec![(0.0, 0.0)];
    let mut h_est: Vec<(f64, f64)> = vec![(0.0, 0.0)];

    let normal = Normal::new(0.0, 1.0).unwrap();
    let q = get_q_control();

    let mut time = 0.0;
    let mut fig = Figure::new();

    while time <= SIM_TIME {
        time += DT;

        // True state update
        x_true = motion_model(&x_true, &u);

        // Dead reckoning with noise
        let u_noisy = Vector2::new(
            u[0] + normal.sample(&mut rand::thread_rng()) * q[(0, 0)].sqrt(),
            u[1] + normal.sample(&mut rand::thread_rng()) * q[(1, 1)].sqrt(),
        );
        x_dr = motion_model(&x_dr, &u_noisy);

        // Get observations with known landmark IDs
        let z = get_observations_with_id(&x_true, &landmarks);

        // EKF SLAM update with known data association
        ekf_slam_known_correspondences(&mut state, &u, &z, n_landmarks);

        // Store history
        h_true.push((x_true[0], x_true[1]));
        h_dr.push((x_dr[0], x_dr[1]));
        h_est.push((state.x[0], state.x[1]));

        // Animation
        if SHOW_ANIMATION && (time * 10.0).fract() < 0.15 {
            fig.clear_axes();

            let true_x: Vec<f64> = h_true.iter().map(|p| p.0).collect();
            let true_y: Vec<f64> = h_true.iter().map(|p| p.1).collect();
            let dr_x: Vec<f64> = h_dr.iter().map(|p| p.0).collect();
            let dr_y: Vec<f64> = h_dr.iter().map(|p| p.1).collect();
            let est_x: Vec<f64> = h_est.iter().map(|p| p.0).collect();
            let est_y: Vec<f64> = h_est.iter().map(|p| p.1).collect();
            let lm_x: Vec<f64> = landmarks.iter().map(|p| p.0).collect();
            let lm_y: Vec<f64> = landmarks.iter().map(|p| p.1).collect();

            // Estimated landmark positions
            let mut est_lm_x: Vec<f64> = Vec::new();
            let mut est_lm_y: Vec<f64> = Vec::new();
            for i in 0..state.n_lm {
                if let Some(lm) = state.get_landmark(i) {
                    est_lm_x.push(lm[0]);
                    est_lm_y.push(lm[1]);
                }
            }

            fig.axes2d()
                .set_title("EKF SLAM", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .points(
                    &lm_x,
                    &lm_y,
                    &[
                        Caption("True Landmarks"),
                        Color("black"),
                        PointSymbol('*'),
                        PointSize(2.0),
                    ],
                )
                .points(
                    &est_lm_x,
                    &est_lm_y,
                    &[
                        Caption("Est. Landmarks"),
                        Color("cyan"),
                        PointSymbol('O'),
                        PointSize(1.5),
                    ],
                )
                .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
                .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")])
                .lines(&est_x, &est_y, &[Caption("EKF SLAM"), Color("green")]);

            fig.show_and_keep_running().unwrap();
        }
    }

    println!("Done!");
    println!("Number of landmarks detected: {}", state.n_lm);

    // Print final landmark estimates
    println!("\nLandmark estimates vs true positions:");
    for (i, (true_x, true_y)) in landmarks.iter().enumerate() {
        if let Some(est_lm) = state.get_landmark(i) {
            let err = ((est_lm[0] - true_x).powi(2) + (est_lm[1] - true_y).powi(2)).sqrt();
            println!(
                "  LM{}: True=({:.2}, {:.2}), Est=({:.2}, {:.2}), Error={:.3}m",
                i, true_x, true_y, est_lm[0], est_lm[1], err
            );
        }
    }

    // Save final plot
    fig.clear_axes();

    let true_x: Vec<f64> = h_true.iter().map(|p| p.0).collect();
    let true_y: Vec<f64> = h_true.iter().map(|p| p.1).collect();
    let dr_x: Vec<f64> = h_dr.iter().map(|p| p.0).collect();
    let dr_y: Vec<f64> = h_dr.iter().map(|p| p.1).collect();
    let est_x: Vec<f64> = h_est.iter().map(|p| p.0).collect();
    let est_y: Vec<f64> = h_est.iter().map(|p| p.1).collect();
    let lm_x: Vec<f64> = landmarks.iter().map(|p| p.0).collect();
    let lm_y: Vec<f64> = landmarks.iter().map(|p| p.1).collect();

    let mut est_lm_x: Vec<f64> = Vec::new();
    let mut est_lm_y: Vec<f64> = Vec::new();
    for i in 0..state.n_lm {
        if let Some(lm) = state.get_landmark(i) {
            est_lm_x.push(lm[0]);
            est_lm_y.push(lm[1]);
        }
    }

    fig.axes2d()
        .set_title("EKF SLAM", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .points(
            &lm_x,
            &lm_y,
            &[
                Caption("True Landmarks"),
                Color("black"),
                PointSymbol('*'),
                PointSize(2.0),
            ],
        )
        .points(
            &est_lm_x,
            &est_lm_y,
            &[
                Caption("Est. Landmarks"),
                Color("cyan"),
                PointSymbol('O'),
                PointSize(1.5),
            ],
        )
        .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
        .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")])
        .lines(&est_x, &est_y, &[Caption("EKF SLAM"), Color("green")]);

    match fig.save_to_svg("./img/slam/ekf_slam.svg", 640, 480) {
        Ok(_) => println!("Plot saved to ./img/slam/ekf_slam.svg"),
        Err(e) => eprintln!("Failed to save SVG: {:?}", e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ekf_slam_state_creation() {
        let state = EKFSLAMState::new();
        assert_eq!(state.x.len(), STATE_SIZE);
        assert_eq!(state.n_lm, 0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(-2.0 * PI) - 0.0).abs() < 1e-10);
        // 3π = π (after normalization), both π and -π are equivalent at boundary
        assert!((normalize_angle(3.0 * PI).abs() - PI).abs() < 1e-10);
    }

    #[test]
    fn test_motion_model() {
        let x = Vector3::new(0.0, 0.0, 0.0);
        let u = Vector2::new(1.0, 0.0); // move forward at 1 m/s
        let new_x = motion_model(&x, &u);

        // Should move in x direction (yaw is 0)
        assert!(new_x[0] > 0.0);
        assert!(new_x[1].abs() < 1e-10);
        assert!(new_x[2].abs() < 1e-10);
    }

    #[test]
    fn test_add_landmark() {
        let mut state = EKFSLAMState::new();
        let z = Vector2::new(5.0, 0.0); // landmark at 5m ahead

        add_new_landmark(&mut state, &z);

        assert_eq!(state.n_lm, 1);
        assert_eq!(state.x.len(), STATE_SIZE + LM_SIZE);

        // Landmark should be at (5, 0) since robot is at origin facing +x
        let lm = state.get_landmark(0).unwrap();
        assert!((lm[0] - 5.0).abs() < 0.1);
        assert!(lm[1].abs() < 0.1);
    }

    #[test]
    fn test_ekf_slam_prediction() {
        let mut state = EKFSLAMState::new();
        let u = Vector2::new(1.0, 0.1);

        ekf_slam_predict(&mut state, &u);

        // Robot should have moved forward and turned slightly
        assert!(state.x[0] > 0.0);
        assert!(state.x[2].abs() > 0.0);
    }

    #[test]
    fn test_ekf_slam_full() {
        let mut state = EKFSLAMState::new();
        let u = Vector2::new(1.0, 0.0);
        let observations = vec![(5.0, 0.0), (5.0, PI / 2.0)];

        ekf_slam(&mut state, &u, &observations);

        // Should have added 2 landmarks
        assert_eq!(state.n_lm, 2);
        assert_eq!(state.x.len(), STATE_SIZE + 2 * LM_SIZE);
    }
}
