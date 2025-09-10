/*!
 * Unscented Kalman Filter (UKF) implementation for localization
 * 
 * This module implements the Unscented Kalman Filter for nonlinear state estimation.
 * UKF uses the unscented transform to handle nonlinearities more accurately than EKF
 * by propagating a set of sigma points through the nonlinear functions.
 * 
 * Ported from PythonRobotics
 * Original author: Atsushi Sakai (@Atsushi_twi)
 */

use nalgebra::{Matrix4, Matrix2, Vector4, Vector2, DMatrix, DVector, OMatrix, Const, Dyn};
use gnuplot::{Figure, Caption, Color, PointSymbol, AxesCommon};
use rand::Rng;
use std::f64;

// UKF Parameters
const ALPHA: f64 = 0.001;
const BETA: f64 = 2.0;
const KAPPA: f64 = 0.0;

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const SIM_TIME: f64 = 50.0; // simulation time [s]

// Noise parameters
const INPUT_NOISE_V: f64 = 1.0;
const INPUT_NOISE_YAW: f64 = 30.0_f64.to_radians();
const GPS_NOISE_X: f64 = 0.5;
const GPS_NOISE_Y: f64 = 0.5;

/// UKF weights structure
#[derive(Debug, Clone)]
pub struct UKFWeights {
    pub wm: DVector<f64>, // mean weights
    pub wc: DVector<f64>, // covariance weights
    pub gamma: f64,       // scaling parameter
}

/// UKF state structure
#[derive(Debug, Clone)]
pub struct UKFState {
    pub x: Vector4<f64>,     // state vector [x, y, yaw, v]
    pub p: Matrix4<f64>,     // covariance matrix
    pub q: Matrix4<f64>,     // process noise covariance
    pub r: Matrix2<f64>,     // observation noise covariance
}

impl UKFState {
    /// Create a new UKF state
    pub fn new() -> Self {
        let x = Vector4::zeros();
        let p = Matrix4::identity();
        
        // Process noise covariance Q
        let q = Matrix4::from_diagonal(&Vector4::new(
            0.1_f64.powi(2),           // variance of location on x-axis
            0.1_f64.powi(2),           // variance of location on y-axis
            1.0_f64.to_radians().powi(2), // variance of yaw angle
            1.0_f64.powi(2),           // variance of velocity
        ));
        
        // Observation noise covariance R
        let r = Matrix2::from_diagonal(&Vector2::new(
            1.0_f64.powi(2), // x position variance
            1.0_f64.powi(2), // y position variance
        ));
        
        UKFState { x, p, q, r }
    }
}

/// Motion model for the vehicle
/// State: [x, y, yaw, v]
/// Input: [v, yaw_rate]
pub fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>) -> Vector4<f64> {
    let mut x_next = x.clone();
    
    // Update position
    x_next[0] += DT * x[3] * x[2].cos(); // x += dt * v * cos(yaw)
    x_next[1] += DT * x[3] * x[2].sin(); // y += dt * v * sin(yaw)
    x_next[2] += DT * u[1];              // yaw += dt * yaw_rate
    x_next[3] = u[0];                    // v = input_v
    
    x_next
}

/// Observation model
/// Returns [x, y] position from state [x, y, yaw, v]
pub fn observation_model(x: &Vector4<f64>) -> Vector2<f64> {
    Vector2::new(x[0], x[1])
}

/// Setup UKF weights and parameters
pub fn setup_ukf(nx: usize) -> UKFWeights {
    let lambda = ALPHA.powi(2) * (nx as f64 + KAPPA) - nx as f64;
    
    let mut wm = Vec::new();
    let mut wc = Vec::new();
    
    // First weight (mean point)
    wm.push(lambda / (lambda + nx as f64));
    wc.push((lambda / (lambda + nx as f64)) + (1.0 - ALPHA.powi(2) + BETA));
    
    // Remaining weights (sigma points)
    let weight = 1.0 / (2.0 * (nx as f64 + lambda));
    for _ in 0..(2 * nx) {
        wm.push(weight);
        wc.push(weight);
    }
    
    let gamma = (nx as f64 + lambda).sqrt();
    
    UKFWeights {
        wm: DVector::from_vec(wm),
        wc: DVector::from_vec(wc),
        gamma,
    }
}

/// Generate sigma points for UKF
pub fn generate_sigma_points(x_est: &Vector4<f64>, p_est: &Matrix4<f64>, gamma: f64) -> DMatrix<f64> {
    let nx = x_est.len();
    let n_sigma = 2 * nx + 1;
    
    let mut sigma = DMatrix::zeros(nx, n_sigma);
    
    // Mean point
    sigma.set_column(0, x_est);
    
    // Matrix square root using Cholesky decomposition
    let chol = p_est.cholesky();
    if let Some(l) = chol {
        let sqrt_p = l.l();
        
        // Positive direction
        for i in 0..nx {
            let col = x_est + gamma * sqrt_p.column(i);
            sigma.set_column(i + 1, &col);
        }
        
        // Negative direction  
        for i in 0..nx {
            let col = x_est - gamma * sqrt_p.column(i);
            sigma.set_column(i + 1 + nx, &col);
        }
    } else {
        // Fallback: use identity if Cholesky fails
        for i in 0..nx {
            let mut col = x_est.clone();
            col[i] += gamma * p_est[(i, i)].sqrt();
            sigma.set_column(i + 1, &col);
            
            col = x_est.clone();
            col[i] -= gamma * p_est[(i, i)].sqrt();
            sigma.set_column(i + 1 + nx, &col);
        }
    }
    
    sigma
}

/// Predict sigma points through motion model
pub fn predict_sigma_motion(sigma: &DMatrix<f64>, u: &Vector2<f64>) -> DMatrix<f64> {
    let mut sigma_pred = sigma.clone();
    
    for i in 0..sigma.ncols() {
        let x_sigma = Vector4::new(
            sigma[(0, i)], sigma[(1, i)], sigma[(2, i)], sigma[(3, i)]
        );
        let x_pred = motion_model(&x_sigma, u);
        
        sigma_pred[(0, i)] = x_pred[0];
        sigma_pred[(1, i)] = x_pred[1];
        sigma_pred[(2, i)] = x_pred[2];
        sigma_pred[(3, i)] = x_pred[3];
    }
    
    sigma_pred
}

/// Predict sigma points through observation model
pub fn predict_sigma_observation(sigma: &DMatrix<f64>) -> DMatrix<f64> {
    let mut z_sigma = DMatrix::zeros(2, sigma.ncols());
    
    for i in 0..sigma.ncols() {
        let x_sigma = Vector4::new(
            sigma[(0, i)], sigma[(1, i)], sigma[(2, i)], sigma[(3, i)]
        );
        let z_pred = observation_model(&x_sigma);
        
        z_sigma[(0, i)] = z_pred[0];
        z_sigma[(1, i)] = z_pred[1];
    }
    
    z_sigma
}

/// Calculate covariance from sigma points
pub fn calc_sigma_covariance(
    x_mean: &DVector<f64>,
    sigma: &DMatrix<f64>,
    wc: &DVector<f64>,
    noise: &DMatrix<f64>,
) -> DMatrix<f64> {
    let _nx = x_mean.len();
    let mut p = noise.clone();
    
    for i in 0..sigma.ncols() {
        let diff = sigma.column(i) - x_mean;
        p += wc[i] * &diff * diff.transpose();
    }
    
    p
}

/// Calculate cross-covariance between state and observation
pub fn calc_cross_covariance(
    x_sigma: &DMatrix<f64>,
    x_mean: &DVector<f64>,
    z_sigma: &DMatrix<f64>,
    z_mean: &DVector<f64>,
    wc: &DVector<f64>,
) -> DMatrix<f64> {
    let nx = x_mean.len();
    let nz = z_mean.len();
    let mut pxz = DMatrix::zeros(nx, nz);
    
    for i in 0..x_sigma.ncols() {
        let dx = x_sigma.column(i) - x_mean;
        let dz = z_sigma.column(i) - z_mean;
        pxz += wc[i] * &dx * dz.transpose();
    }
    
    pxz
}

/// UKF estimation step
pub fn ukf_estimation(
    state: &mut UKFState,
    z: &Vector2<f64>,
    u: &Vector2<f64>,
    weights: &UKFWeights,
) {
    let nx = 4;
    let nz = 2;
    
    // Predict step
    let sigma = generate_sigma_points(&state.x, &state.p, weights.gamma);
    let sigma_pred = predict_sigma_motion(&sigma, u);
    
    // Calculate predicted mean
    let mut x_pred = DVector::zeros(nx);
    for i in 0..sigma_pred.ncols() {
        x_pred += weights.wm[i] * sigma_pred.column(i);
    }
    
    // Calculate predicted covariance
    let q_dyn = DMatrix::from_fn(4, 4, |i, j| state.q[(i, j)]);
    let p_pred = calc_sigma_covariance(&x_pred, &sigma_pred, &weights.wc, &q_dyn);
    
    // Update step
    let sigma_update = generate_sigma_points(
        &Vector4::new(x_pred[0], x_pred[1], x_pred[2], x_pred[3]),
        &Matrix4::from_fn(|i, j| p_pred[(i, j)]),
        weights.gamma,
    );
    let z_sigma = predict_sigma_observation(&sigma_update);
    
    // Calculate predicted observation mean
    let mut z_pred = DVector::zeros(nz);
    for i in 0..z_sigma.ncols() {
        z_pred += weights.wm[i] * z_sigma.column(i);
    }
    
    // Calculate innovation covariance
    let r_dyn = DMatrix::from_fn(2, 2, |i, j| state.r[(i, j)]);
    let s = calc_sigma_covariance(&z_pred, &z_sigma, &weights.wc, &r_dyn);
    
    // Calculate cross-covariance
    let pxz = calc_cross_covariance(&sigma_update, &x_pred, &z_sigma, &z_pred, &weights.wc);
    
    // Kalman gain
    let s_inv = s.clone().try_inverse().unwrap_or_else(|| DMatrix::identity(nz, nz));
    let k = &pxz * &s_inv;
    
    // Update state and covariance
    let innovation = DVector::from_vec(vec![z[0] - z_pred[0], z[1] - z_pred[1]]);
    let x_update = &x_pred + &k * &innovation;
    let p_update = &p_pred - &k * &s * k.transpose();
    
    // Store results
    state.x = Vector4::new(x_update[0], x_update[1], x_update[2], x_update[3]);
    state.p = Matrix4::from_fn(|i, j| p_update[(i, j)]);
}

/// Generate control input
pub fn calc_input() -> Vector2<f64> {
    Vector2::new(1.0, 0.1) // [v, yaw_rate]
}

/// Simulate observation with noise
pub fn observation(
    x_true: &mut Vector4<f64>,
    x_dr: &mut Vector4<f64>,
    u: &Vector2<f64>,
) -> (Vector2<f64>, Vector2<f64>) {
    let mut rng = rand::thread_rng();
    
    // Update true state
    *x_true = motion_model(x_true, u);
    
    // Add noise to GPS observation
    let z = observation_model(x_true) + Vector2::new(
        GPS_NOISE_X * rng.gen::<f64>() - GPS_NOISE_X / 2.0,
        GPS_NOISE_Y * rng.gen::<f64>() - GPS_NOISE_Y / 2.0,
    );
    
    // Add noise to input for dead reckoning
    let u_noisy = u + Vector2::new(
        INPUT_NOISE_V * rng.gen::<f64>() - INPUT_NOISE_V / 2.0,
        INPUT_NOISE_YAW * rng.gen::<f64>() - INPUT_NOISE_YAW / 2.0,
    );
    
    // Update dead reckoning
    *x_dr = motion_model(x_dr, &u_noisy);
    
    (z, u_noisy)
}

/// Plot covariance ellipse
pub fn plot_covariance_ellipse(
    x_est: &Vector4<f64>,
    p_est: &Matrix4<f64>,
    x_data: &mut Vec<f64>,
    y_data: &mut Vec<f64>,
) {
    let p_xy = p_est.fixed_view::<2, 2>(0, 0);
    
    // Eigenvalue decomposition
    let eigen = p_xy.symmetric_eigen();
    let eigenvals = eigen.eigenvalues;
    let eigenvecs = eigen.eigenvectors;
    
    let (big_idx, small_idx) = if eigenvals[0] >= eigenvals[1] {
        (0, 1)
    } else {
        (1, 0)
    };
    
    let a = eigenvals[big_idx].sqrt();
    let b = eigenvals[small_idx].sqrt();
    let angle = eigenvecs[(1, big_idx)].atan2(eigenvecs[(0, big_idx)]);
    
    // Generate ellipse points
    let n_points = 50;
    x_data.clear();
    y_data.clear();
    
    for i in 0..=n_points {
        let t = 2.0 * f64::consts::PI * i as f64 / n_points as f64;
        let x_local = a * t.cos();
        let y_local = b * t.sin();
        
        // Rotate and translate
        let x_global = x_local * angle.cos() - y_local * angle.sin() + x_est[0];
        let y_global = x_local * angle.sin() + y_local * angle.cos() + x_est[1];
        
        x_data.push(x_global);
        y_data.push(y_global);
    }
}

/// Run UKF localization demo
pub fn demo_ukf_localization() {
    println!("=== Unscented Kalman Filter Localization Demo ===\n");
    
    // Create output directory
    std::fs::create_dir_all("img/localization").unwrap_or_default();
    
    let nx = 4; // State dimension
    let mut state = UKFState::new();
    let weights = setup_ukf(nx);
    
    // Initialize states
    let mut x_true = Vector4::zeros();
    let mut x_dr = Vector4::zeros();
    
    // History storage
    let mut h_x_est = Vec::new();
    let mut h_x_true = Vec::new();
    let mut h_x_dr = Vec::new();
    let mut h_z = Vec::new();
    
    let mut time = 0.0;
    let mut step = 0;
    
    println!("Starting simulation...");
    
    while time <= SIM_TIME {
        time += DT;
        step += 1;
        
        let u = calc_input();
        let (z, u_noisy) = observation(&mut x_true, &mut x_dr, &u);
        
        // UKF estimation
        ukf_estimation(&mut state, &z, &u_noisy, &weights);
        
        // Store history
        h_x_est.push(state.x.clone());
        h_x_true.push(x_true.clone());
        h_x_dr.push(x_dr.clone());
        h_z.push(z.clone());
        
        if step % 50 == 0 {
            println!("Time: {:.1}s, Position: ({:.2}, {:.2})", time, state.x[0], state.x[1]);
        }
    }
    
    println!("Simulation completed. Creating visualization...");
    
    // Create visualization
    create_ukf_visualization(&h_x_est, &h_x_true, &h_x_dr, &h_z, &state);
    
    // Calculate and display final errors
    let final_true = &h_x_true.last().unwrap();
    let final_est = &h_x_est.last().unwrap();
    let final_dr = &h_x_dr.last().unwrap();
    
    let ukf_error = ((final_est[0] - final_true[0]).powi(2) + (final_est[1] - final_true[1]).powi(2)).sqrt();
    let dr_error = ((final_dr[0] - final_true[0]).powi(2) + (final_dr[1] - final_true[1]).powi(2)).sqrt();
    
    println!("\n=== Final Results ===");
    println!("True position: ({:.3}, {:.3})", final_true[0], final_true[1]);
    println!("UKF estimate: ({:.3}, {:.3})", final_est[0], final_est[1]);
    println!("Dead reckoning: ({:.3}, {:.3})", final_dr[0], final_dr[1]);
    println!("UKF error: {:.3} m", ukf_error);
    println!("Dead reckoning error: {:.3} m", dr_error);
    println!("Improvement: {:.1}%", (1.0 - ukf_error / dr_error) * 100.0);
}

/// Create visualization of UKF results
fn create_ukf_visualization(
    h_x_est: &[Vector4<f64>],
    h_x_true: &[Vector4<f64>],
    h_x_dr: &[Vector4<f64>],
    h_z: &[Vector2<f64>],
    final_state: &UKFState,
) {
    let mut fg = Figure::new();
    let axes = fg.axes2d();
    
    // Extract trajectory data
    let true_x: Vec<f64> = h_x_true.iter().map(|x| x[0]).collect();
    let true_y: Vec<f64> = h_x_true.iter().map(|x| x[1]).collect();
    
    let est_x: Vec<f64> = h_x_est.iter().map(|x| x[0]).collect();
    let est_y: Vec<f64> = h_x_est.iter().map(|x| x[1]).collect();
    
    let dr_x: Vec<f64> = h_x_dr.iter().map(|x| x[0]).collect();
    let dr_y: Vec<f64> = h_x_dr.iter().map(|x| x[1]).collect();
    
    let obs_x: Vec<f64> = h_z.iter().map(|z| z[0]).collect();
    let obs_y: Vec<f64> = h_z.iter().map(|z| z[1]).collect();
    
    // Plot trajectories
    axes.lines(&true_x, &true_y, &[Caption("Ground Truth"), Color("blue")]);
    axes.lines(&est_x, &est_y, &[Caption("UKF Estimate"), Color("red")]);
    axes.lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("black")]);
    axes.points(&obs_x, &obs_y, &[Caption("GPS Observations"), Color("green"), PointSymbol('.')]);
    
    // Plot final covariance ellipse
    let mut ellipse_x = Vec::new();
    let mut ellipse_y = Vec::new();
    plot_covariance_ellipse(&h_x_est.last().unwrap(), &final_state.p, &mut ellipse_x, &mut ellipse_y);
    axes.lines(&ellipse_x, &ellipse_y, &[Caption("Uncertainty Ellipse"), Color("red")]);
    
    axes.set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
    axes.set_title("Unscented Kalman Filter Localization", &[]);
    axes.set_x_label("X [m]", &[]);
    axes.set_y_label("Y [m]", &[]);
    
    // Save plot
    fg.set_terminal("pngcairo", "img/localization/ukf_result.png");
    fg.show().unwrap();
    
    println!("Visualization saved to img/localization/ukf_result.png");
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_motion_model() {
        let x = Vector4::new(0.0, 0.0, 0.0, 1.0);
        let u = Vector2::new(1.0, 0.1);
        let x_next = motion_model(&x, &u);
        
        assert!((x_next[0] - DT).abs() < 1e-10); // x should increase by dt*v*cos(0) = dt
        assert!(x_next[1].abs() < 1e-10);        // y should remain 0
        assert!((x_next[2] - DT * 0.1).abs() < 1e-10); // yaw should increase by dt*yaw_rate
        assert!((x_next[3] - 1.0).abs() < 1e-10); // velocity should be input velocity
    }
    
    #[test]
    fn test_observation_model() {
        let x = Vector4::new(1.0, 2.0, 0.5, 1.0);
        let z = observation_model(&x);
        
        assert_eq!(z[0], 1.0);
        assert_eq!(z[1], 2.0);
    }
    
    #[test]
    fn test_ukf_weights() {
        let weights = setup_ukf(4);
        
        // Check that weights sum to 1
        let wm_sum: f64 = weights.wm.iter().sum();
        let wc_sum: f64 = weights.wc.iter().sum();
        
        assert!((wm_sum - 1.0).abs() < 1e-10);
        assert!((wc_sum - 1.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_sigma_points_generation() {
        let x = Vector4::zeros();
        let p = Matrix4::identity();
        let gamma = 2.0;
        
        let sigma = generate_sigma_points(&x, &p, gamma);
        
        // Should have 2*nx + 1 = 9 sigma points
        assert_eq!(sigma.ncols(), 9);
        assert_eq!(sigma.nrows(), 4);
        
        // First column should be the mean
        for i in 0..4 {
            assert_eq!(sigma[(i, 0)], x[i]);
        }
    }
}

fn main() {
    demo_ukf_localization();
}
