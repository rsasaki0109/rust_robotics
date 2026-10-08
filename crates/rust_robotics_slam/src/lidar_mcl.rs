//! Monte Carlo localization of a 2D LiDAR in a known occupancy grid.
//!
//! - **Motion:** every particle applies the odometry step with noise that
//!   grows with the distance and rotation travelled (odometry motion model).
//! - **Measurement:** the likelihood-field model scores a subsampled scan by
//!   the distance from each beam end point to the nearest occupied cell,
//!   mixed with a uniform term for unexplained returns.
//! - **Global initialization:** the first scan after
//!   [`LidarMcl::initialize_global`] scores `global_oversampling` times as
//!   many uniform poses as there are particles and keeps the best by weight.
//! - **Recovery:** augmented MCL tracks short- and long-term averages of the
//!   measurement likelihood and injects new particles when the short-term
//!   average drops; sensor resetting also injects while the scan fit at the
//!   particles stays poor. Each injected particle is the best-scoring of a few
//!   uniform free-space draws, so the filter recovers from a kidnapping
//!   without needing a lucky sample. A small fraction of scored particles is
//!   injected on every update, so a filter that converged on a look-alike
//!   place (long uniform corridors) keeps probing for the true one.
//! - **Resampling:** weights accumulate across updates and the particles are
//!   resampled (low variance) only when the effective sample size halves.

use nalgebra::Vector2;
use rand::{rngs::StdRng, Rng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::Pose2D;

use crate::lidar_occupancy::{CellState, OccupancyGrid};
use crate::scan_to_map::compose_pose;

/// Tuning parameters for [`LidarMcl`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarMclConfig {
    /// Number of particles.
    pub particles: usize,
    /// Translation noise per meter travelled \[m/m\].
    pub translation_noise: f64,
    /// Rotation noise per radian turned \[rad/rad\].
    pub rotation_noise: f64,
    /// Rotation noise per meter travelled \[rad/m\].
    pub rotation_noise_per_meter: f64,
    /// Standard deviation of a beam end point around the nearest wall \[m\].
    pub hit_sigma: f64,
    /// Weight of the uniform "unexplained return" term per beam.
    pub random_weight: f64,
    /// Use every `beam_stride`-th beam of a scan.
    pub beam_stride: usize,
    /// Distances beyond this are treated as equally unlikely \[m\].
    pub max_field_distance: f64,
    /// Effective number of independent beams: the per-beam geometric-mean
    /// likelihood is raised to this power. Beams are correlated, so a value
    /// below the beam count keeps the filter from collapsing too early.
    pub independent_beams: f64,
    /// A per-beam likelihood below this at the weighted particle mean counts
    /// as a poor fit (sensor-resetting localization).
    pub poor_fit: f64,
    /// Fraction of particles re-drawn per update during a poor fit.
    pub reset_fraction: f64,
    /// A re-drawn particle is the best-scoring of this many uniform free
    /// poses, so recovery does not need a lucky draw.
    pub reset_candidates: usize,
    /// Fraction of particles replaced by scored free poses on every update,
    /// so a filter confidently locked onto a look-alike place keeps probing
    /// for the true one.
    pub min_injection: f64,
    /// Global (re)initialization scores this many times `particles` uniform
    /// poses against the first scan and keeps `particles` of them.
    pub global_oversampling: usize,
    /// Smoothing factors of the long- and short-term likelihood averages
    /// (augmented MCL, `alpha_slow < alpha_fast`).
    pub alpha_slow: f64,
    pub alpha_fast: f64,
    /// RNG seed.
    pub seed: u64,
}

impl Default for LidarMclConfig {
    fn default() -> Self {
        Self {
            particles: 600,
            translation_noise: 0.1,
            rotation_noise: 0.1,
            rotation_noise_per_meter: 0.05,
            hit_sigma: 0.12,
            random_weight: 0.05,
            beam_stride: 6,
            max_field_distance: 1.5,
            independent_beams: 8.0,
            poor_fit: 0.6,
            reset_fraction: 0.1,
            reset_candidates: 20,
            min_injection: 0.01,
            global_oversampling: 10,
            alpha_slow: 0.01,
            alpha_fast: 0.3,
            seed: 3,
        }
    }
}

/// A weighted pose hypothesis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Particle {
    pub pose: Pose2D,
    pub weight: f64,
}

/// Particle-filter localization against an [`OccupancyGrid`].
#[derive(Debug, Clone)]
pub struct LidarMcl {
    config: LidarMclConfig,
    grid: OccupancyGrid,
    field: Vec<f32>,
    free_cells: Vec<(usize, usize)>,
    particles: Vec<Particle>,
    rng: StdRng,
    w_slow: f64,
    w_fast: f64,
    fit: f64,
    /// Particles are uniform and the next scan should seed them.
    global_pending: bool,
}

impl LidarMcl {
    /// Creates a filter on `grid` with particles spread uniformly over the
    /// free cells (global localization).
    pub fn new(grid: OccupancyGrid, config: LidarMclConfig) -> Self {
        let field = grid.distance_field(config.max_field_distance);
        let (width, height) = grid.size();
        let free_cells = (0..height)
            .flat_map(|y| (0..width).map(move |x| (x, y)))
            .filter(|&(x, y)| grid.state(x, y) == CellState::Free)
            .collect();
        let mut mcl = Self {
            config,
            grid,
            field,
            free_cells,
            particles: Vec::new(),
            rng: StdRng::seed_from_u64(config.seed),
            w_slow: 0.0,
            w_fast: 0.0,
            fit: 0.0,
            global_pending: false,
        };
        mcl.initialize_global();
        mcl
    }

    /// The map being localized in.
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// Current particles.
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    fn random_free_pose(&mut self) -> Pose2D {
        if self.free_cells.is_empty() {
            return Pose2D::origin();
        }
        let (x, y) = self.free_cells[self.rng.random_range(0..self.free_cells.len())];
        let center = self.grid.cell_center(x, y);
        let half = self.grid.resolution() / 2.0;
        Pose2D::new(
            center.x + self.rng.random_range(-half..half),
            center.y + self.rng.random_range(-half..half),
            self.rng
                .random_range(-std::f64::consts::PI..std::f64::consts::PI),
        )
    }

    /// Spreads the particles uniformly over the free space.
    pub fn initialize_global(&mut self) {
        let count = self.config.particles.max(1);
        let weight = 1.0 / count as f64;
        self.particles = (0..count)
            .map(|_| Particle {
                pose: self.random_free_pose(),
                weight,
            })
            .collect();
        self.w_slow = 0.0;
        self.w_fast = 0.0;
        self.global_pending = true;
    }

    /// Spreads the particles around `pose` (position σ `xy`, heading σ `yaw`).
    pub fn initialize_at(&mut self, pose: Pose2D, xy: f64, yaw: f64) {
        let count = self.config.particles.max(1);
        let position = Normal::new(0.0, xy.max(1.0e-9)).expect("finite sigma");
        let heading = Normal::new(0.0, yaw.max(1.0e-9)).expect("finite sigma");
        self.particles = (0..count)
            .map(|_| Particle {
                pose: Pose2D::new(
                    pose.x + position.sample(&mut self.rng),
                    pose.y + position.sample(&mut self.rng),
                    pose.yaw + heading.sample(&mut self.rng),
                ),
                weight: 1.0 / count as f64,
            })
            .collect();
        self.global_pending = false;
    }

    /// Motion update with a body-frame odometry step.
    pub fn predict(&mut self, odom_delta: Pose2D) {
        let distance = odom_delta.x.hypot(odom_delta.y);
        let turn = odom_delta.yaw.abs();
        let sigma_xy = self.config.translation_noise * distance;
        let sigma_yaw =
            self.config.rotation_noise * turn + self.config.rotation_noise_per_meter * distance;
        let xy = Normal::new(0.0, sigma_xy.max(1.0e-9)).expect("finite sigma");
        let yaw = Normal::new(0.0, sigma_yaw.max(1.0e-9)).expect("finite sigma");
        for particle in &mut self.particles {
            let noisy = Pose2D::new(
                odom_delta.x + xy.sample(&mut self.rng),
                odom_delta.y + xy.sample(&mut self.rng),
                odom_delta.yaw + yaw.sample(&mut self.rng),
            );
            particle.pose = compose_pose(particle.pose, noisy);
        }
    }

    /// Per-beam likelihood of the scan from `pose` (geometric mean, in (0, 1]).
    fn beam_likelihood(&self, pose: Pose2D, beams: &[Vector2<f64>]) -> f64 {
        if beams.is_empty() {
            return 1.0;
        }
        let (width, _) = self.grid.size();
        let (sin, cos) = pose.yaw.sin_cos();
        let sigma_sq = self.config.hit_sigma * self.config.hit_sigma;
        let log_sum: f64 = beams
            .iter()
            .map(|beam| {
                let world = Vector2::new(
                    pose.x + cos * beam.x - sin * beam.y,
                    pose.y + sin * beam.x + cos * beam.y,
                );
                let distance = self
                    .grid
                    .cell_of(world)
                    .map_or(self.config.max_field_distance, |(x, y)| {
                        f64::from(self.field[y * width + x])
                    });
                let hit = (-distance * distance / (2.0 * sigma_sq)).exp();
                (hit + self.config.random_weight).ln()
            })
            .sum();
        (log_sum / beams.len() as f64).exp() / (1.0 + self.config.random_weight)
    }

    /// Measurement update and resampling with a beam-ordered body-frame scan.
    pub fn update(&mut self, scan_body: &[Vector2<f64>]) {
        let beams: Vec<Vector2<f64>> = scan_body
            .iter()
            .step_by(self.config.beam_stride.max(1))
            .copied()
            .collect();
        if beams.is_empty() || self.particles.is_empty() {
            return;
        }
        if self.global_pending {
            self.seed_from_scan(&beams);
            return;
        }
        let probes =
            (self.particles.len() as f64 * self.config.min_injection.clamp(0.0, 1.0)) as usize;
        for _ in 0..probes {
            let slot = self.rng.random_range(0..self.particles.len());
            let pose = self.scored_free_pose(&beams);
            self.particles[slot].pose = pose;
        }
        // Sharpen the per-beam geometric mean back to a scan likelihood. A
        // particle standing outside the mapped free space is implausible.
        let exponent = self.config.independent_beams.max(1.0);
        let likelihoods: Vec<f64> = self
            .particles
            .iter()
            .map(|particle| {
                let position = Vector2::new(particle.pose.x, particle.pose.y);
                let inside = self.grid.state_at(position) == CellState::Free;
                let likelihood = self.beam_likelihood(particle.pose, &beams);
                if inside {
                    likelihood
                } else {
                    likelihood * 1.0e-3
                }
            })
            .collect();
        let average = likelihoods.iter().sum::<f64>() / likelihoods.len() as f64;
        let best = likelihoods
            .iter()
            .copied()
            .fold(0.0, f64::max)
            .max(1.0e-300);
        // Weights accumulate between resamplings.
        for (particle, likelihood) in self.particles.iter_mut().zip(&likelihoods) {
            particle.weight *= (likelihood / best).powf(exponent);
        }
        let total: f64 = self.particles.iter().map(|p| p.weight).sum();
        if total > 0.0 && total.is_finite() {
            for particle in &mut self.particles {
                particle.weight /= total;
            }
        } else {
            let uniform = 1.0 / self.particles.len() as f64;
            for particle in &mut self.particles {
                particle.weight = uniform;
            }
        }

        if self.w_slow == 0.0 {
            self.w_slow = average;
            self.w_fast = average;
        } else {
            self.w_slow += self.config.alpha_slow * (average - self.w_slow);
            self.w_fast += self.config.alpha_fast * (average - self.w_fast);
        }
        // Posterior-weighted per-beam likelihood: how well the filter explains the scan.
        let fit: f64 = self
            .particles
            .iter()
            .zip(&likelihoods)
            .map(|(particle, likelihood)| particle.weight * likelihood)
            .sum();
        self.fit = fit;
        let reset = if fit < self.config.poor_fit {
            self.config.reset_fraction
        } else {
            0.0
        };
        let inject = (1.0 - self.w_fast / self.w_slow).max(reset);
        let effective = 1.0
            / self
                .particles
                .iter()
                .map(|p| p.weight * p.weight)
                .sum::<f64>();
        if inject > 0.0 || effective < 0.5 * self.particles.len() as f64 {
            self.resample(inject, &beams);
        }
    }

    /// Global initialization from a scan: scores `global_oversampling ×
    /// particles` uniform free poses and resamples `particles` of them.
    fn seed_from_scan(&mut self, beams: &[Vector2<f64>]) {
        let count = self.config.particles.max(1);
        let exponent = self.config.independent_beams.max(1.0);
        let candidates: Vec<(Pose2D, f64)> = (0..count * self.config.global_oversampling.max(1))
            .map(|_| {
                let pose = self.random_free_pose();
                (pose, self.beam_likelihood(pose, beams))
            })
            .collect();
        let best = candidates
            .iter()
            .map(|(_, likelihood)| *likelihood)
            .fold(1.0e-300, f64::max);
        let total: f64 = candidates
            .iter()
            .map(|(_, likelihood)| (likelihood / best).powf(exponent))
            .sum();
        self.particles = candidates
            .iter()
            .map(|(pose, likelihood)| Particle {
                pose: *pose,
                weight: (likelihood / best).powf(exponent) / total,
            })
            .collect();
        self.resample_to(count, 0.0, beams);
        let average = candidates.iter().map(|(_, l)| l).sum::<f64>() / candidates.len() as f64;
        self.w_slow = average;
        self.w_fast = average;
        self.fit = best;
        self.global_pending = false;
    }

    /// The best-scoring of `reset_candidates` uniform free poses.
    fn scored_free_pose(&mut self, beams: &[Vector2<f64>]) -> Pose2D {
        let mut best = (f64::NEG_INFINITY, Pose2D::origin());
        for _ in 0..self.config.reset_candidates.max(1) {
            let pose = self.random_free_pose();
            let score = self.beam_likelihood(pose, beams);
            if score > best.0 {
                best = (score, pose);
            }
        }
        best.1
    }

    /// Low-variance resampling; each slot is replaced by a scored free pose
    /// with probability `inject`.
    fn resample(&mut self, inject: f64, beams: &[Vector2<f64>]) {
        self.resample_to(self.particles.len(), inject, beams);
    }

    /// Low-variance resampling of the current weighted particles into
    /// `count` equally weighted ones.
    fn resample_to(&mut self, count: usize, inject: f64, beams: &[Vector2<f64>]) {
        let step = 1.0 / count as f64;
        let mut target = self.rng.random_range(0.0..step);
        let mut cumulative = self.particles[0].weight;
        let mut index = 0;
        let mut next = Vec::with_capacity(count);
        for _ in 0..count {
            while target > cumulative && index + 1 < self.particles.len() {
                index += 1;
                cumulative += self.particles[index].weight;
            }
            let pose = if self.rng.random_bool(inject.clamp(0.0, 1.0)) {
                self.scored_free_pose(beams)
            } else {
                self.particles[index].pose
            };
            next.push(Particle { pose, weight: step });
            target += step;
        }
        self.particles = next;
    }

    /// Posterior-weighted per-beam likelihood of the last scan, in (0, 1]:
    /// how well the particles explain what the LiDAR sees.
    pub fn fit(&self) -> f64 {
        self.fit
    }

    /// Weighted mean pose and the RMS distance of particles from it \[m\].
    pub fn estimate(&self) -> (Pose2D, f64) {
        let (mut x, mut y, mut sin, mut cos, mut total) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for particle in &self.particles {
            let w = particle.weight;
            x += w * particle.pose.x;
            y += w * particle.pose.y;
            sin += w * particle.pose.yaw.sin();
            cos += w * particle.pose.yaw.cos();
            total += w;
        }
        let total = total.max(1.0e-300);
        let mean = Pose2D::new(x / total, y / total, sin.atan2(cos));
        let spread = (self
            .particles
            .iter()
            .map(|p| p.weight * ((p.pose.x - mean.x).powi(2) + (p.pose.y - mean.y).powi(2)))
            .sum::<f64>()
            / total)
            .sqrt();
        (mean, spread)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lidar_occupancy::OccupancyConfig;
    use crate::scan_to_map::{ranges_to_points, ray_cast_ranges, relative_pose, LineSegment};

    fn polygon(corners: &[(f64, f64)]) -> Vec<LineSegment> {
        (0..corners.len())
            .map(|i| {
                let (ax, ay) = corners[i];
                let (bx, by) = corners[(i + 1) % corners.len()];
                LineSegment::new(Vector2::new(ax, ay), Vector2::new(bx, by))
            })
            .collect()
    }

    /// An asymmetric room, so the pose is globally identifiable.
    fn world() -> Vec<LineSegment> {
        let mut walls = polygon(&[(-6.0, -4.0), (6.0, -4.0), (6.0, 4.0), (-6.0, 4.0)]);
        walls.extend(polygon(&[(1.0, 1.0), (2.0, 1.4), (1.6, 2.4), (0.6, 2.0)]));
        walls.extend(polygon(&[(-4.5, -3.0), (-3.7, -3.0), (-3.7, -2.2)]));
        walls.push(LineSegment::new(
            Vector2::new(3.5, -4.0),
            Vector2::new(3.5, -1.5),
        ));
        walls
    }

    fn scan(pose: Pose2D) -> Vec<Vector2<f64>> {
        ranges_to_points(&ray_cast_ranges(pose, &world(), 180, 12.0))
    }

    fn map() -> OccupancyGrid {
        let poses = [
            Pose2D::new(-3.0, 0.0, 0.0),
            Pose2D::new(0.0, -2.0, 1.0),
            Pose2D::new(3.0, 2.0, -2.0),
            Pose2D::new(-1.0, 3.0, 0.5),
        ];
        let scans: Vec<_> = poses.iter().map(|pose| scan(*pose)).collect();
        OccupancyGrid::from_scans(
            poses.iter().copied().zip(scans.iter().map(Vec::as_slice)),
            0.5,
            OccupancyConfig::default(),
        )
    }

    /// Drives a slow circle from `start`, returning the final truth.
    fn drive(mcl: &mut LidarMcl, start: Pose2D, steps: usize) -> Pose2D {
        let mut truth = start;
        let step = Pose2D::new(0.1, 0.0, 0.04);
        for _ in 0..steps {
            truth = compose_pose(truth, step);
            mcl.predict(step);
            mcl.update(&scan(truth));
        }
        truth
    }

    #[test]
    fn global_localization_converges_from_uniform_particles() {
        let mut mcl = LidarMcl::new(map(), LidarMclConfig::default());
        let truth = drive(&mut mcl, Pose2D::new(-2.0, -1.0, 0.3), 40);
        let (estimate, spread) = mcl.estimate();
        let error = relative_pose(truth, estimate);
        assert!(error.x.hypot(error.y) < 0.2, "error {error:?}");
        assert!(error.yaw.abs() < 0.1, "yaw error {}", error.yaw);
        assert!(spread < 0.3, "spread {spread}");
    }

    #[test]
    fn recovers_from_a_kidnapping() {
        let mut mcl = LidarMcl::new(map(), LidarMclConfig::default());
        let start = Pose2D::new(-2.0, -1.0, 0.3);
        mcl.initialize_at(start, 0.05, 0.02);
        let _ = drive(&mut mcl, start, 10);
        // Teleport the robot; the particles still believe the old pose.
        let truth = drive(&mut mcl, Pose2D::new(3.0, 1.0, 2.5), 80);
        let error = relative_pose(truth, mcl.estimate().0);
        assert!(error.x.hypot(error.y) < 0.3, "error {error:?}");
    }
}
