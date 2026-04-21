#![allow(dead_code)]

// FastSLAM 1.0
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::{Matrix2, Vector2, Vector3};
use rand_distr::{Distribution, Normal, Uniform};
use std::f64::consts::PI;

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const MAX_RANGE: f64 = 20.0; // maximum observation range [m]

// Particle filter parameters
const N_PARTICLE: usize = 100; // number of particles
const NTH: f64 = N_PARTICLE as f64 / 1.5; // resampling threshold

// Noise parameters
// (10.0 * PI / 180.0)^2 ≈ 0.0305
const Q_SIM: [[f64; 2]; 2] = [[0.3, 0.0], [0.0, 0.0305]]; // simulation noise
const R_SIM: [[f64; 2]; 2] = [[0.5, 0.0], [0.0, 0.0305]]; // observation noise

/// Landmark for EKF
#[derive(Clone)]
pub struct Landmark {
    pub x: f64,
    pub y: f64,
    pub cov: Matrix2<f64>, // covariance
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
pub struct Particle {
    pub weight: f64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub landmarks: Vec<Landmark>,
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

    Matrix2::new(dx / d, dy / d, -dy / d2, dx / d2)
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
        u[0] + normal.sample(&mut rand::rng()) * q[(0, 0)].sqrt(),
        u[1] + normal.sample(&mut rand::rng()) * q[(1, 1)].sqrt(),
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
    let uniform = Uniform::new(0.0, 1.0 / n as f64).expect("valid resampling range");
    let mut r = uniform.sample(&mut rand::rng());

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
pub fn fastslam_update(
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
pub fn get_best_particle(particles: &[Particle]) -> &Particle {
    particles
        .iter()
        .max_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap())
        .unwrap()
}

/// Simulate observations
pub fn get_observations(x_true: &Vector3<f64>, landmarks: &[(f64, f64)]) -> Vec<(f64, f64, usize)> {
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
            let d_noisy = d + normal.sample(&mut rand::rng()) * r[(0, 0)].sqrt();
            let angle_noisy = angle + normal.sample(&mut rand::rng()) * r[(1, 1)].sqrt();

            z.push((d_noisy, angle_noisy, lm_id));
        }
    }

    z
}

/// Create a new set of particles for FastSLAM
pub fn create_particles(n_particles: usize, n_landmarks: usize) -> Vec<Particle> {
    (0..n_particles)
        .map(|_| Particle::new(n_landmarks))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_particles() {
        let n_particles = 50;
        let n_landmarks = 5;
        let particles = create_particles(n_particles, n_landmarks);

        assert_eq!(particles.len(), n_particles);
        for p in &particles {
            assert_eq!(p.landmarks.len(), n_landmarks);
        }
    }

    #[test]
    fn test_get_observations() {
        // Robot at origin facing +x
        let x_true = Vector3::new(0.0, 0.0, 0.0);
        // One landmark within range, one outside
        let landmarks = vec![(5.0, 0.0), (100.0, 100.0)];
        let observations = get_observations(&x_true, &landmarks);

        // Only the first landmark should be observed (within MAX_RANGE=20)
        assert_eq!(observations.len(), 1);
        let (d, _angle, lm_id) = observations[0];
        assert_eq!(lm_id, 0);
        // Distance should be roughly 5.0 (noisy)
        assert!(
            (d - 5.0).abs() < 5.0,
            "distance {d} too far from expected 5.0"
        );
    }

    #[test]
    fn test_get_best_particle() {
        let n_landmarks = 3;
        let mut particles = create_particles(5, n_landmarks);
        // Assign distinct weights
        particles[0].weight = 0.1;
        particles[1].weight = 0.5;
        particles[2].weight = 0.9;
        particles[3].weight = 0.3;
        particles[4].weight = 0.2;

        let best = get_best_particle(&particles);
        assert!(
            (best.weight - 0.9).abs() < f64::EPSILON,
            "expected best weight 0.9, got {}",
            best.weight
        );
    }

    #[test]
    fn test_fastslam_update_does_not_panic() {
        let n_landmarks = 3;
        let mut particles = create_particles(20, n_landmarks);
        let u = Vector2::new(1.0, 0.1); // linear vel, angular vel

        // Simulate a few observations
        let landmarks = vec![(10.0, 0.0), (0.0, 10.0), (10.0, 10.0)];
        let x_true = Vector3::new(0.0, 0.0, 0.0);

        for _ in 0..5 {
            let z = get_observations(&x_true, &landmarks);
            fastslam_update(&mut particles, u, &z);
        }

        // All particles should still exist
        assert_eq!(particles.len(), 20);
    }

    #[test]
    fn test_particle_initial_state() {
        let n_landmarks = 4;
        let particles = create_particles(10, n_landmarks);

        for p in &particles {
            assert_eq!(p.x, 0.0);
            assert_eq!(p.y, 0.0);
            assert_eq!(p.yaw, 0.0);
            // Weight should be 1/N_PARTICLE
            assert!((p.weight - 1.0 / N_PARTICLE as f64).abs() < f64::EPSILON);
            // Each landmark should have large initial covariance
            for lm in &p.landmarks {
                assert_eq!(lm.x, 0.0);
                assert_eq!(lm.y, 0.0);
                assert_eq!(lm.cov[(0, 0)], 1000.0);
                assert_eq!(lm.cov[(1, 1)], 1000.0);
            }
        }
    }
}
