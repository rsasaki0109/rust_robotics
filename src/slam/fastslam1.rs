// FastSLAM 1.0
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::{Matrix2, Matrix2x3, Matrix3x2, Vector2, Vector3, DMatrix, DVector};
use rand_distr::{Distribution, Normal, Uniform};
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};
use std::f64::consts::PI;

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const SIM_TIME: f64 = 50.0; // simulation time [s]
const MAX_RANGE: f64 = 20.0; // maximum observation range [m]
const M_DIST_TH: f64 = 2.0; // Mahalanobis distance threshold for data association

// Particle filter parameters
const N_PARTICLE: usize = 100; // number of particles
const NTH: f64 = N_PARTICLE as f64 / 1.5; // resampling threshold

// Noise parameters
// (10.0 * PI / 180.0)^2 ≈ 0.0305
const Q_SIM: [[f64; 2]; 2] = [[0.3, 0.0], [0.0, 0.0305]]; // simulation noise
const R_SIM: [[f64; 2]; 2] = [[0.5, 0.0], [0.0, 0.0305]]; // observation noise

const SHOW_ANIMATION: bool = true;

/// Landmark for EKF
#[derive(Clone)]
struct Landmark {
    x: f64,
    y: f64,
    cov: Matrix2<f64>, // covariance
}

impl Landmark {
    fn new() -> Self {
        Landmark {
            x: 0.0,
            y: 0.0,
            cov: Matrix2::identity() * 1000.0, // large initial uncertainty
        }
    }
}

/// Particle for FastSLAM
#[derive(Clone)]
struct Particle {
    weight: f64,
    x: f64,
    y: f64,
    yaw: f64,
    landmarks: Vec<Landmark>,
}

impl Particle {
    fn new(n_landmarks: usize) -> Self {
        Particle {
            weight: 1.0 / N_PARTICLE as f64,
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            landmarks: vec![Landmark::new(); n_landmarks],
        }
    }

    fn pose(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.yaw)
    }
}

/// Motion model for robot
fn motion_model(x: Vector3<f64>, u: Vector2<f64>) -> Vector3<f64> {
    let yaw = x[2];
    Vector3::new(
        x[0] + u[0] * DT * yaw.cos(),
        x[1] + u[0] * DT * yaw.sin(),
        normalize_angle(x[2] + u[1] * DT),
    )
}

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

/// Observation model: predict observation from particle pose and landmark
fn observation_model(particle: &Particle, lm_id: usize) -> Vector2<f64> {
    let lm = &particle.landmarks[lm_id];
    let dx = lm.x - particle.x;
    let dy = lm.y - particle.y;
    let d = (dx * dx + dy * dy).sqrt();
    let angle = normalize_angle(dy.atan2(dx) - particle.yaw);
    Vector2::new(d, angle)
}

/// Compute Jacobian of observation model w.r.t. landmark position
fn compute_jacobian(particle: &Particle, lm_id: usize) -> Matrix2<f64> {
    let lm = &particle.landmarks[lm_id];
    let dx = lm.x - particle.x;
    let dy = lm.y - particle.y;
    let d2 = dx * dx + dy * dy;
    let d = d2.sqrt();

    Matrix2::new(
        dx / d, dy / d,
        -dy / d2, dx / d2,
    )
}

/// Process noise covariance
fn get_q() -> Matrix2<f64> {
    Matrix2::new(Q_SIM[0][0], Q_SIM[0][1], Q_SIM[1][0], Q_SIM[1][1])
}

/// Observation noise covariance
fn get_r() -> Matrix2<f64> {
    Matrix2::new(R_SIM[0][0], R_SIM[0][1], R_SIM[1][0], R_SIM[1][1])
}

/// Update particle with control input (motion update)
fn predict_particle(particle: &mut Particle, u: Vector2<f64>) {
    let normal = Normal::new(0.0, 1.0).unwrap();
    let q = get_q();

    // Add noise to control
    let u_noisy = Vector2::new(
        u[0] + normal.sample(&mut rand::thread_rng()) * q[(0, 0)].sqrt(),
        u[1] + normal.sample(&mut rand::thread_rng()) * q[(1, 1)].sqrt(),
    );

    let pose = motion_model(particle.pose(), u_noisy);
    particle.x = pose[0];
    particle.y = pose[1];
    particle.yaw = pose[2];
}

/// Update landmark with EKF
fn update_landmark(particle: &mut Particle, z: &Vector2<f64>, lm_id: usize, r: &Matrix2<f64>) {
    let lm = &particle.landmarks[lm_id];

    // First observation of this landmark
    if lm.cov[(0, 0)] > 100.0 {
        // Initialize landmark position
        particle.landmarks[lm_id].x = particle.x + z[0] * (particle.yaw + z[1]).cos();
        particle.landmarks[lm_id].y = particle.y + z[0] * (particle.yaw + z[1]).sin();
        return;
    }

    // Predicted observation
    let z_pred = observation_model(particle, lm_id);

    // Innovation
    let y = Vector2::new(z[0] - z_pred[0], normalize_angle(z[1] - z_pred[1]));

    // Jacobian
    let h = compute_jacobian(particle, lm_id);

    // Innovation covariance
    let s = h * particle.landmarks[lm_id].cov * h.transpose() + r;

    // Kalman gain
    let s_inv = s.try_inverse().unwrap_or(Matrix2::identity());
    let k = particle.landmarks[lm_id].cov * h.transpose() * s_inv;

    // Update landmark position
    let delta = k * y;
    particle.landmarks[lm_id].x += delta[0];
    particle.landmarks[lm_id].y += delta[1];

    // Update covariance
    let i = Matrix2::identity();
    particle.landmarks[lm_id].cov = (i - k * h) * particle.landmarks[lm_id].cov;

    // Update weight using likelihood
    let det_s = s.determinant();
    if det_s > 0.0 {
        let mahal = y.transpose() * s_inv * y;
        let likelihood = (-0.5 * mahal[(0, 0)]).exp() / (2.0 * PI * det_s.sqrt());
        particle.weight *= likelihood;
    }
}

/// Compute effective number of particles
fn compute_neff(particles: &[Particle]) -> f64 {
    let sum_w2: f64 = particles.iter().map(|p| p.weight * p.weight).sum();
    if sum_w2 > 0.0 {
        1.0 / sum_w2
    } else {
        0.0
    }
}

/// Normalize particle weights
fn normalize_weights(particles: &mut [Particle]) {
    let sum_w: f64 = particles.iter().map(|p| p.weight).sum();
    if sum_w > 0.0 {
        for p in particles.iter_mut() {
            p.weight /= sum_w;
        }
    }
}

/// Low variance resampling
fn resample(particles: &mut Vec<Particle>) {
    normalize_weights(particles);

    let n = particles.len();
    let mut new_particles = Vec::with_capacity(n);

    // Cumulative sum of weights
    let mut cum_sum = vec![0.0; n + 1];
    for (i, p) in particles.iter().enumerate() {
        cum_sum[i + 1] = cum_sum[i] + p.weight;
    }

    // Systematic resampling
    let uniform = Uniform::new(0.0, 1.0 / n as f64);
    let mut r = uniform.sample(&mut rand::thread_rng());

    let mut j = 0;
    for _ in 0..n {
        while r > cum_sum[j + 1] && j < n - 1 {
            j += 1;
        }
        let mut new_p = particles[j].clone();
        new_p.weight = 1.0 / n as f64;
        new_particles.push(new_p);
        r += 1.0 / n as f64;
    }

    *particles = new_particles;
}

/// FastSLAM update step
fn fastslam_update(
    particles: &mut Vec<Particle>,
    u: Vector2<f64>,
    z: &[(f64, f64, usize)], // (distance, angle, landmark_id)
) {
    let r = get_r();

    // Prediction step
    for particle in particles.iter_mut() {
        predict_particle(particle, u);
    }

    // Update step for each observation
    for (d, angle, lm_id) in z {
        let z_vec = Vector2::new(*d, *angle);

        for particle in particles.iter_mut() {
            update_landmark(particle, &z_vec, *lm_id, &r);
        }
    }

    // Normalize weights
    normalize_weights(particles);

    // Resample if needed
    let neff = compute_neff(particles);
    if neff < NTH {
        resample(particles);
    }
}

/// Get best particle (highest weight)
fn get_best_particle(particles: &[Particle]) -> &Particle {
    particles.iter().max_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap()).unwrap()
}

/// Simulate observations
fn get_observations(
    x_true: &Vector3<f64>,
    landmarks: &[(f64, f64)],
) -> Vec<(f64, f64, usize)> {
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
    println!("FastSLAM 1.0 start!");

    // Landmark positions [x, y]
    let landmarks: Vec<(f64, f64)> = vec![
        (10.0, -2.0),
        (15.0, 10.0),
        (3.0, 15.0),
        (-5.0, 20.0),
        (-5.0, 5.0),
    ];
    let n_landmarks = landmarks.len();

    // Initialize particles
    let mut particles: Vec<Particle> = (0..N_PARTICLE)
        .map(|_| Particle::new(n_landmarks))
        .collect();

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
    let q = get_q();

    let mut time = 0.0;
    let mut fig = Figure::new();

    while time <= SIM_TIME {
        time += DT;

        // True state update
        x_true = motion_model(x_true, u);

        // Dead reckoning with noise
        let u_noisy = Vector2::new(
            u[0] + normal.sample(&mut rand::thread_rng()) * q[(0, 0)].sqrt(),
            u[1] + normal.sample(&mut rand::thread_rng()) * q[(1, 1)].sqrt(),
        );
        x_dr = motion_model(x_dr, u_noisy);

        // Get observations
        let z = get_observations(&x_true, &landmarks);

        // FastSLAM update
        fastslam_update(&mut particles, u_noisy, &z);

        // Get best estimate
        let best = get_best_particle(&particles);

        // Store history
        h_true.push((x_true[0], x_true[1]));
        h_dr.push((x_dr[0], x_dr[1]));
        h_est.push((best.x, best.y));

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

            // Particle positions
            let p_x: Vec<f64> = particles.iter().map(|p| p.x).collect();
            let p_y: Vec<f64> = particles.iter().map(|p| p.y).collect();

            // Estimated landmark positions from best particle
            let est_lm_x: Vec<f64> = best.landmarks.iter()
                .filter(|lm| lm.cov[(0, 0)] < 100.0)
                .map(|lm| lm.x).collect();
            let est_lm_y: Vec<f64> = best.landmarks.iter()
                .filter(|lm| lm.cov[(0, 0)] < 100.0)
                .map(|lm| lm.y).collect();

            fig.axes2d()
                .set_title("FastSLAM 1.0", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .points(&lm_x, &lm_y, &[Caption("True Landmarks"), Color("black"), PointSymbol('*'), PointSize(2.0)])
                .points(&est_lm_x, &est_lm_y, &[Caption("Est. Landmarks"), Color("cyan"), PointSymbol('O'), PointSize(1.5)])
                .points(&p_x, &p_y, &[Caption("Particles"), Color("gray"), PointSymbol('.'), PointSize(0.5)])
                .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
                .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")])
                .lines(&est_x, &est_y, &[Caption("FastSLAM"), Color("green")]);

            fig.show_and_keep_running().unwrap();
        }
    }

    println!("Done!");

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

    let best = get_best_particle(&particles);
    let est_lm_x: Vec<f64> = best.landmarks.iter()
        .filter(|lm| lm.cov[(0, 0)] < 100.0)
        .map(|lm| lm.x).collect();
    let est_lm_y: Vec<f64> = best.landmarks.iter()
        .filter(|lm| lm.cov[(0, 0)] < 100.0)
        .map(|lm| lm.y).collect();

    fig.axes2d()
        .set_title("FastSLAM 1.0", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .points(&lm_x, &lm_y, &[Caption("True Landmarks"), Color("black"), PointSymbol('*'), PointSize(2.0)])
        .points(&est_lm_x, &est_lm_y, &[Caption("Est. Landmarks"), Color("cyan"), PointSymbol('O'), PointSize(1.5)])
        .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
        .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")])
        .lines(&est_x, &est_y, &[Caption("FastSLAM"), Color("green")]);

    fig.save_to_svg("./img/slam/fastslam1.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/slam/fastslam1.svg");
}
