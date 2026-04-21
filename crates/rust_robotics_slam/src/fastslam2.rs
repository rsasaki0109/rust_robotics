#![allow(dead_code)]

//! FastSLAM 2.0
//!
//! Improves on FastSLAM 1.0 by incorporating observations into the proposal
//! distribution for particle pose sampling, resulting in better particle
//! efficiency.
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109)
//!         Rust port

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Vector2, Vector3};
use rand::Rng;
use rand_distr::{Distribution, Normal, Uniform};
use std::f64::consts::PI;

// Simulation parameters
const DT: f64 = 0.1;
const MAX_RANGE: f64 = 20.0;

// Particle filter parameters
const N_PARTICLE: usize = 100;
const NTH: f64 = N_PARTICLE as f64 / 1.5;

// Noise parameters
const Q_SIM: [[f64; 2]; 2] = [[0.3, 0.0], [0.0, 0.0305]];
const R_SIM: [[f64; 2]; 2] = [[0.5, 0.0], [0.0, 0.0305]];

// Motion noise (for proposal distribution)
const MOTION_COV: [[f64; 3]; 3] = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.01]];

#[derive(Clone)]
pub struct Landmark {
    pub x: f64,
    pub y: f64,
    pub cov: Matrix2<f64>,
}

impl Landmark {
    fn new() -> Self {
        Landmark {
            x: 0.0,
            y: 0.0,
            cov: Matrix2::identity() * 1000.0,
        }
    }

    fn is_initialized(&self) -> bool {
        self.cov[(0, 0)] < 100.0
    }
}

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

    fn set_pose(&mut self, pose: &Vector3<f64>) {
        self.x = pose[0];
        self.y = pose[1];
        self.yaw = normalize_angle(pose[2]);
    }
}

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

fn motion_model(x: Vector3<f64>, u: Vector2<f64>) -> Vector3<f64> {
    let yaw = x[2];
    Vector3::new(
        x[0] + u[0] * DT * yaw.cos(),
        x[1] + u[0] * DT * yaw.sin(),
        normalize_angle(x[2] + u[1] * DT),
    )
}

/// Jacobian of motion model w.r.t. state
fn motion_jacobian(x: &Vector3<f64>, u: &Vector2<f64>) -> Matrix3<f64> {
    let yaw = x[2];
    let v = u[0];
    Matrix3::new(
        1.0,
        0.0,
        -v * DT * yaw.sin(),
        0.0,
        1.0,
        v * DT * yaw.cos(),
        0.0,
        0.0,
        1.0,
    )
}

fn observation_model(px: f64, py: f64, pyaw: f64, lm_x: f64, lm_y: f64) -> Vector2<f64> {
    let dx = lm_x - px;
    let dy = lm_y - py;
    let d = (dx * dx + dy * dy).sqrt();
    let angle = normalize_angle(dy.atan2(dx) - pyaw);
    Vector2::new(d, angle)
}

/// Jacobian of observation model w.r.t. landmark position
fn obs_jacobian_landmark(px: f64, py: f64, lm_x: f64, lm_y: f64) -> Matrix2<f64> {
    let dx = lm_x - px;
    let dy = lm_y - py;
    let d2 = dx * dx + dy * dy;
    let d = d2.sqrt();
    Matrix2::new(dx / d, dy / d, -dy / d2, dx / d2)
}

/// Jacobian of observation model w.r.t. particle pose [x, y, yaw]
fn obs_jacobian_pose(px: f64, py: f64, pyaw: f64, lm_x: f64, lm_y: f64) -> Matrix2x3<f64> {
    let dx = lm_x - px;
    let dy = lm_y - py;
    let d2 = dx * dx + dy * dy;
    let d = d2.sqrt();
    let _ = pyaw; // yaw only affects angle via subtraction
    Matrix2x3::new(-dx / d, -dy / d, 0.0, dy / d2, -dx / d2, -1.0)
}

fn get_q() -> Matrix2<f64> {
    Matrix2::new(Q_SIM[0][0], Q_SIM[0][1], Q_SIM[1][0], Q_SIM[1][1])
}

fn get_r() -> Matrix2<f64> {
    Matrix2::new(R_SIM[0][0], R_SIM[0][1], R_SIM[1][0], R_SIM[1][1])
}

fn get_motion_cov() -> Matrix3<f64> {
    Matrix3::new(
        MOTION_COV[0][0],
        MOTION_COV[0][1],
        MOTION_COV[0][2],
        MOTION_COV[1][0],
        MOTION_COV[1][1],
        MOTION_COV[1][2],
        MOTION_COV[2][0],
        MOTION_COV[2][1],
        MOTION_COV[2][2],
    )
}

/// FastSLAM 2.0: compute the improved proposal distribution for a particle
/// given an observation of a known landmark, and sample from it.
fn compute_proposal(
    particle: &Particle,
    u: &Vector2<f64>,
    z: &Vector2<f64>,
    lm_id: usize,
    r: &Matrix2<f64>,
) -> (Vector3<f64>, Matrix3<f64>) {
    let lm = &particle.landmarks[lm_id];

    // Prior: motion model prediction
    let x_pred = motion_model(particle.pose(), *u);
    let g = motion_jacobian(&particle.pose(), u);
    let motion_cov = get_motion_cov();
    let p_pred = g * motion_cov * g.transpose();

    if !lm.is_initialized() {
        // No observation update possible for uninitialized landmark
        return (x_pred, p_pred);
    }

    // Observation Jacobian w.r.t. pose at predicted pose
    let h_pose = obs_jacobian_pose(x_pred[0], x_pred[1], x_pred[2], lm.x, lm.y);

    // Innovation covariance contribution from landmark
    let h_lm = obs_jacobian_landmark(x_pred[0], x_pred[1], lm.x, lm.y);
    let q_obs = h_lm * lm.cov * h_lm.transpose() + r;

    // Fuse prior (motion) with observation likelihood
    // Posterior precision = prior precision + observation precision
    let h_pose_t = h_pose.transpose();
    let q_obs_inv = q_obs.try_inverse().unwrap_or(Matrix2::identity());

    let p_pred_inv = p_pred.try_inverse().unwrap_or(Matrix3::identity() * 1e-6);
    let p_post_inv = p_pred_inv + h_pose_t * q_obs_inv * h_pose;
    let p_post = p_post_inv.try_inverse().unwrap_or(p_pred);

    // Posterior mean
    let z_pred = observation_model(x_pred[0], x_pred[1], x_pred[2], lm.x, lm.y);
    let innovation = Vector2::new(z[0] - z_pred[0], normalize_angle(z[1] - z_pred[1]));

    let x_post = x_pred + p_post * h_pose_t * q_obs_inv * innovation;

    (x_post, p_post)
}

/// Sample a 3D pose from a Gaussian distribution
fn sample_pose_with_rng<R: Rng + ?Sized>(
    mean: &Vector3<f64>,
    cov: &Matrix3<f64>,
    rng: &mut R,
) -> Vector3<f64> {
    let normal = Normal::new(0.0, 1.0).unwrap();

    // Cholesky decomposition for sampling
    let l = cov.cholesky().map(|c| c.l()).unwrap_or_else(|| {
        // Fallback: use diagonal sqrt
        Matrix3::from_diagonal(&Vector3::new(
            cov[(0, 0)].max(0.0).sqrt(),
            cov[(1, 1)].max(0.0).sqrt(),
            cov[(2, 2)].max(0.0).sqrt(),
        ))
    });

    let noise = Vector3::new(normal.sample(rng), normal.sample(rng), normal.sample(rng));

    mean + l * noise
}

/// Update landmark EKF and compute weight (FastSLAM 2.0 version)
fn update_landmark_and_weight(
    particle: &mut Particle,
    z: &Vector2<f64>,
    lm_id: usize,
    r: &Matrix2<f64>,
) -> f64 {
    let lm = &particle.landmarks[lm_id];

    if !lm.is_initialized() {
        // Initialize landmark
        particle.landmarks[lm_id].x = particle.x + z[0] * (particle.yaw + z[1]).cos();
        particle.landmarks[lm_id].y = particle.y + z[0] * (particle.yaw + z[1]).sin();
        particle.landmarks[lm_id].cov = Matrix2::identity() * 10.0;
        return 1.0; // neutral weight for new landmark
    }

    let z_pred = observation_model(particle.x, particle.y, particle.yaw, lm.x, lm.y);
    let innovation = Vector2::new(z[0] - z_pred[0], normalize_angle(z[1] - z_pred[1]));

    let h = obs_jacobian_landmark(particle.x, particle.y, lm.x, lm.y);
    let s = h * particle.landmarks[lm_id].cov * h.transpose() + r;
    let s_inv = s.try_inverse().unwrap_or(Matrix2::identity());
    let k = particle.landmarks[lm_id].cov * h.transpose() * s_inv;

    // Update landmark
    let delta = k * innovation;
    particle.landmarks[lm_id].x += delta[0];
    particle.landmarks[lm_id].y += delta[1];
    particle.landmarks[lm_id].cov = (Matrix2::identity() - k * h) * particle.landmarks[lm_id].cov;

    // Compute weight from likelihood
    let det_s = s.determinant();
    if det_s > 0.0 {
        let mahal = innovation.transpose() * s_inv * innovation;
        (-0.5 * mahal[(0, 0)]).exp() / (2.0 * PI * det_s.sqrt())
    } else {
        1e-10
    }
}

fn compute_neff(particles: &[Particle]) -> f64 {
    let sum_w2: f64 = particles.iter().map(|p| p.weight * p.weight).sum();
    if sum_w2 > 0.0 {
        1.0 / sum_w2
    } else {
        0.0
    }
}

fn normalize_weights(particles: &mut [Particle]) {
    let sum_w: f64 = particles.iter().map(|p| p.weight).sum();
    if sum_w > 0.0 {
        for p in particles.iter_mut() {
            p.weight /= sum_w;
        }
    }
}

fn resample_with_rng<R: Rng + ?Sized>(particles: &mut Vec<Particle>, rng: &mut R) {
    normalize_weights(particles);
    let n = particles.len();
    let mut new_particles = Vec::with_capacity(n);

    let mut cum_sum = vec![0.0; n + 1];
    for (i, p) in particles.iter().enumerate() {
        cum_sum[i + 1] = cum_sum[i] + p.weight;
    }

    let uniform = Uniform::new(0.0, 1.0 / n as f64).expect("valid resampling range");
    let mut r = uniform.sample(rng);
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

/// FastSLAM 2.0 update step.
///
/// Key difference from FastSLAM 1.0: the particle pose is sampled from a
/// proposal distribution that incorporates the observation, not just the
/// motion model.
fn fastslam2_update_with_rng<R: Rng + ?Sized>(
    particles: &mut Vec<Particle>,
    u: Vector2<f64>,
    z: &[(f64, f64, usize)], // (distance, angle, landmark_id)
    rng: &mut R,
) {
    let r = get_r();

    for particle in particles.iter_mut() {
        // FastSLAM 2.0: compute proposal using first observation (if any)
        if let Some(&(d, angle, lm_id)) = z.first() {
            let z_vec = Vector2::new(d, angle);
            let (proposal_mean, proposal_cov) = compute_proposal(particle, &u, &z_vec, lm_id, &r);

            // Sample pose from improved proposal
            let sampled_pose = sample_pose_with_rng(&proposal_mean, &proposal_cov, rng);
            particle.set_pose(&sampled_pose);
        } else {
            // No observations: fall back to motion model sampling
            let normal = Normal::new(0.0, 1.0).unwrap();
            let q = get_q();
            let u_noisy = Vector2::new(
                u[0] + normal.sample(rng) * q[(0, 0)].sqrt(),
                u[1] + normal.sample(rng) * q[(1, 1)].sqrt(),
            );
            let pose = motion_model(particle.pose(), u_noisy);
            particle.set_pose(&pose);
        }

        // Update landmarks and compute weights
        for &(d, angle, lm_id) in z {
            let z_vec = Vector2::new(d, angle);
            let w = update_landmark_and_weight(particle, &z_vec, lm_id, &r);
            particle.weight *= w;
        }
    }

    normalize_weights(particles);

    let neff = compute_neff(particles);
    if neff < NTH {
        resample_with_rng(particles, rng);
    }
}

pub fn fastslam2_update(
    particles: &mut Vec<Particle>,
    u: Vector2<f64>,
    z: &[(f64, f64, usize)], // (distance, angle, landmark_id)
) {
    let mut rng = rand::thread_rng();
    fastslam2_update_with_rng(particles, u, z, &mut rng);
}

pub fn get_best_particle(particles: &[Particle]) -> &Particle {
    particles
        .iter()
        .max_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap())
        .unwrap()
}

fn get_observations_with_rng<R: Rng + ?Sized>(
    x_true: &Vector3<f64>,
    landmarks: &[(f64, f64)],
    rng: &mut R,
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
            let d_noisy = d + normal.sample(rng) * r[(0, 0)].sqrt();
            let angle_noisy = angle + normal.sample(rng) * r[(1, 1)].sqrt();
            z.push((d_noisy, angle_noisy, lm_id));
        }
    }

    z
}

pub fn get_observations(x_true: &Vector3<f64>, landmarks: &[(f64, f64)]) -> Vec<(f64, f64, usize)> {
    let mut rng = rand::thread_rng();
    get_observations_with_rng(x_true, landmarks, &mut rng)
}

pub fn create_particles(n_particles: usize, n_landmarks: usize) -> Vec<Particle> {
    (0..n_particles)
        .map(|_| Particle::new(n_landmarks))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, SeedableRng};

    #[test]
    fn test_create_particles() {
        let particles = create_particles(50, 5);
        assert_eq!(particles.len(), 50);
        for p in &particles {
            assert_eq!(p.landmarks.len(), 5);
        }
    }

    #[test]
    fn test_fastslam2_update_does_not_panic() {
        let mut particles = create_particles(20, 3);
        let u = Vector2::new(1.0, 0.1);
        let landmarks = vec![(10.0, 0.0), (0.0, 10.0), (10.0, 10.0)];
        let x_true = Vector3::new(0.0, 0.0, 0.0);
        let mut rng = StdRng::seed_from_u64(7);

        for _ in 0..5 {
            let z = get_observations_with_rng(&x_true, &landmarks, &mut rng);
            fastslam2_update_with_rng(&mut particles, u, &z, &mut rng);
        }

        assert_eq!(particles.len(), 20);
    }

    #[test]
    fn test_proposal_improves_with_observation() {
        let mut p = Particle::new(1);
        // Initialize a landmark at (5, 0)
        p.landmarks[0].x = 5.0;
        p.landmarks[0].y = 0.0;
        p.landmarks[0].cov = Matrix2::identity() * 0.5;

        let u = Vector2::new(1.0, 0.0);
        let z = Vector2::new(5.0, 0.0); // observe landmark at distance=5, angle=0
        let r = get_r();

        let (mean, cov) = compute_proposal(&p, &u, &z, 0, &r);

        // Proposal covariance should be smaller than motion-only covariance
        let g = motion_jacobian(&p.pose(), &u);
        let motion_only_cov = g * get_motion_cov() * g.transpose();

        assert!(
            cov.determinant() < motion_only_cov.determinant(),
            "proposal should have less uncertainty than motion-only"
        );

        // Mean should be close to predicted (no large deviation for consistent obs)
        let x_pred = motion_model(p.pose(), u);
        let diff = (mean - x_pred).norm();
        assert!(
            diff < 1.0,
            "proposal mean should be near prediction: diff={diff}"
        );
    }

    #[test]
    fn test_landmark_convergence() {
        let mut particles = create_particles(120, 1);
        let landmarks = vec![(5.0, 5.0)];
        let mut x_true = Vector3::new(0.0, 0.0, PI / 4.0);
        let u = Vector2::new(0.5, 0.0);
        let mut rng = StdRng::seed_from_u64(17);

        for _ in 0..60 {
            x_true = motion_model(x_true, u);
            let z = get_observations_with_rng(&x_true, &landmarks, &mut rng);
            fastslam2_update_with_rng(&mut particles, u, &z, &mut rng);
        }

        let initialized: Vec<&Particle> = particles
            .iter()
            .filter(|particle| particle.landmarks[0].is_initialized())
            .collect();
        assert!(
            !initialized.is_empty(),
            "at least one particle should initialize the landmark"
        );

        let total_weight: f64 = initialized.iter().map(|particle| particle.weight).sum();
        let (mean_x, mean_y) = if total_weight > 0.0 {
            let weighted_x = initialized
                .iter()
                .map(|particle| particle.weight * particle.landmarks[0].x)
                .sum::<f64>()
                / total_weight;
            let weighted_y = initialized
                .iter()
                .map(|particle| particle.weight * particle.landmarks[0].y)
                .sum::<f64>()
                / total_weight;
            (weighted_x, weighted_y)
        } else {
            let avg_x = initialized
                .iter()
                .map(|particle| particle.landmarks[0].x)
                .sum::<f64>()
                / initialized.len() as f64;
            let avg_y = initialized
                .iter()
                .map(|particle| particle.landmarks[0].y)
                .sum::<f64>()
                / initialized.len() as f64;
            (avg_x, avg_y)
        };

        let lm_err = ((mean_x - 5.0).powi(2) + (mean_y - 5.0).powi(2)).sqrt();
        assert!(
            lm_err < 6.0,
            "landmark estimate should converge: err={lm_err}"
        );
    }
}
