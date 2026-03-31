#![allow(clippy::excessive_precision, clippy::too_many_arguments)]

//! Eta^3 spline path and trajectory planner
//!
//! Generates smooth paths for wheeled mobile robots using eta^3 polynomial
//! splines. Each segment is a 7th-degree parametric curve connecting two
//! poses (position + heading), shaped by eta parameters and curvature
//! constraints.
//!
//! Reference:
//! - \[eta^3-Splines for the Smooth Path Generation of Wheeled Mobile Robots\]
//!   (<https://ieeexplore.ieee.org/document/4339545/>)

/// A pose in 2D: position (x, y) and heading angle theta \[rad\].
#[derive(Debug, Clone, Copy)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// Shaping parameters for an eta^3 spline segment.
///
/// Six values `[eta0..eta5]` controlling the curve shape.
/// `eta[0..2]` affect the start side, `eta[3..5]` affect the end side.
#[derive(Debug, Clone, Copy)]
pub struct EtaParams {
    pub values: [f64; 6],
}

impl EtaParams {
    pub fn new(values: [f64; 6]) -> Self {
        Self { values }
    }

    pub fn zeros() -> Self {
        Self { values: [0.0; 6] }
    }
}

/// Curvature parameters at the segment endpoints.
///
/// `[kappa_a, kappa_dot_a, kappa_b, kappa_dot_b]`
/// where `_a` is the start and `_b` is the end of the segment.
#[derive(Debug, Clone, Copy)]
pub struct KappaParams {
    pub values: [f64; 4],
}

impl KappaParams {
    pub fn new(values: [f64; 4]) -> Self {
        Self { values }
    }

    pub fn zeros() -> Self {
        Self { values: [0.0; 4] }
    }
}

/// A single eta^3 path segment connecting two poses.
///
/// Internally stores 2x8 polynomial coefficients (x and y, degree 0..7).
#[derive(Debug, Clone)]
pub struct Eta3PathSegment {
    pub start_pose: Pose2D,
    pub end_pose: Pose2D,
    /// Polynomial coefficients: `coeffs[dim][degree]` where dim 0=x, 1=y.
    coeffs: [[f64; 8]; 2],
    /// Precomputed total arc length of this segment.
    pub segment_length: f64,
}

impl Eta3PathSegment {
    /// Create a new segment from start/end poses with shaping and curvature
    /// parameters.
    pub fn new(start_pose: Pose2D, end_pose: Pose2D, eta: &EtaParams, kappa: &KappaParams) -> Self {
        let e = &eta.values;
        let k = &kappa.values;

        let ca = start_pose.theta.cos();
        let sa = start_pose.theta.sin();
        let cb = end_pose.theta.cos();
        let sb = end_pose.theta.sin();

        let dx = end_pose.x - start_pose.x;
        let dy = end_pose.y - start_pose.y;

        let mut coeffs = [[0.0f64; 8]; 2];

        // u^0 (constant)
        coeffs[0][0] = start_pose.x;
        coeffs[1][0] = start_pose.y;

        // u^1 (linear)
        coeffs[0][1] = e[0] * ca;
        coeffs[1][1] = e[0] * sa;

        // u^2 (quadratic)
        coeffs[0][2] = 0.5 * e[2] * ca - 0.5 * e[0].powi(2) * k[0] * sa;
        coeffs[1][2] = 0.5 * e[2] * sa + 0.5 * e[0].powi(2) * k[0] * ca;

        // u^3 (cubic)
        let cubic_curv = e[0].powi(3) * k[1] + 3.0 * e[0] * e[2] * k[0];
        coeffs[0][3] = (1.0 / 6.0) * e[4] * ca - (1.0 / 6.0) * cubic_curv * sa;
        coeffs[1][3] = (1.0 / 6.0) * e[4] * sa + (1.0 / 6.0) * cubic_curv * ca;

        // u^4 (quartic)
        {
            let t1 = 35.0 * dx;
            let t2 = (20.0 * e[0] + 5.0 * e[2] + (2.0 / 3.0) * e[4]) * ca;
            let t3 = (5.0 * e[0].powi(2) * k[0]
                + (2.0 / 3.0) * e[0].powi(3) * k[1]
                + 2.0 * e[0] * e[2] * k[0])
                * sa;
            let t4 = (15.0 * e[1] - 2.5 * e[3] + (1.0 / 6.0) * e[5]) * cb;
            let t5 = (2.5 * e[1].powi(2) * k[2]
                - (1.0 / 6.0) * e[1].powi(3) * k[3]
                - 0.5 * e[1] * e[3] * k[2])
                * sb;
            coeffs[0][4] = t1 - t2 + t3 - t4 - t5;

            let t1 = 35.0 * dy;
            let t2 = (20.0 * e[0] + 5.0 * e[2] + (2.0 / 3.0) * e[4]) * sa;
            let t3 = (5.0 * e[0].powi(2) * k[0]
                + (2.0 / 3.0) * e[0].powi(3) * k[1]
                + 2.0 * e[0] * e[2] * k[0])
                * ca;
            let t4 = (15.0 * e[1] - 2.5 * e[3] + (1.0 / 6.0) * e[5]) * sb;
            let t5 = (2.5 * e[1].powi(2) * k[2]
                - (1.0 / 6.0) * e[1].powi(3) * k[3]
                - 0.5 * e[1] * e[3] * k[2])
                * cb;
            coeffs[1][4] = t1 - t2 - t3 - t4 + t5;
        }

        // u^5 (quintic)
        {
            let t1 = -84.0 * dx;
            let t2 = (45.0 * e[0] + 10.0 * e[2] + e[4]) * ca;
            let t3 =
                (10.0 * e[0].powi(2) * k[0] + e[0].powi(3) * k[1] + 3.0 * e[0] * e[2] * k[0]) * sa;
            let t4 = (39.0 * e[1] - 7.0 * e[3] + 0.5 * e[5]) * cb;
            let t5 =
                (7.0 * e[1].powi(2) * k[2] - 0.5 * e[1].powi(3) * k[3] - 1.5 * e[1] * e[3] * k[2])
                    * sb;
            coeffs[0][5] = t1 + t2 - t3 + t4 + t5;

            let t1 = -84.0 * dy;
            let t2 = (45.0 * e[0] + 10.0 * e[2] + e[4]) * sa;
            let t3 =
                (10.0 * e[0].powi(2) * k[0] + e[0].powi(3) * k[1] + 3.0 * e[0] * e[2] * k[0]) * ca;
            let t4 = (39.0 * e[1] - 7.0 * e[3] + 0.5 * e[5]) * sb;
            let t5 =
                -(7.0 * e[1].powi(2) * k[2] - 0.5 * e[1].powi(3) * k[3] - 1.5 * e[1] * e[3] * k[2])
                    * cb;
            coeffs[1][5] = t1 + t2 + t3 + t4 + t5;
        }

        // u^6 (sextic)
        {
            let t1 = 70.0 * dx;
            let t2 = (36.0 * e[0] + 7.5 * e[2] + (2.0 / 3.0) * e[4]) * ca;
            let t3 = (7.5 * e[0].powi(2) * k[0]
                + (2.0 / 3.0) * e[0].powi(3) * k[1]
                + 2.0 * e[0] * e[2] * k[0])
                * sa;
            let t4 = (34.0 * e[1] - 6.5 * e[3] + 0.5 * e[5]) * cb;
            let t5 =
                -(6.5 * e[1].powi(2) * k[2] - 0.5 * e[1].powi(3) * k[3] - 1.5 * e[1] * e[3] * k[2])
                    * sb;
            coeffs[0][6] = t1 - t2 + t3 - t4 + t5;

            let t1 = 70.0 * dy;
            let t2 = -(36.0 * e[0] + 7.5 * e[2] + (2.0 / 3.0) * e[4]) * sa;
            let t3 = -(7.5 * e[0].powi(2) * k[0]
                + (2.0 / 3.0) * e[0].powi(3) * k[1]
                + 2.0 * e[0] * e[2] * k[0])
                * ca;
            let t4 = -(34.0 * e[1] - 6.5 * e[3] + 0.5 * e[5]) * sb;
            let t5 =
                (6.5 * e[1].powi(2) * k[2] - 0.5 * e[1].powi(3) * k[3] - 1.5 * e[1] * e[3] * k[2])
                    * cb;
            coeffs[1][6] = t1 + t2 + t3 + t4 + t5;
        }

        // u^7 (septic)
        {
            let t1 = -20.0 * dx;
            let t2 = (10.0 * e[0] + 2.0 * e[2] + (1.0 / 6.0) * e[4]) * ca;
            let t3 = -(2.0 * e[0].powi(2) * k[0]
                + (1.0 / 6.0) * e[0].powi(3) * k[1]
                + 0.5 * e[0] * e[2] * k[0])
                * sa;
            let t4 = (10.0 * e[1] - 2.0 * e[3] + (1.0 / 6.0) * e[5]) * cb;
            let t5 = (2.0 * e[1].powi(2) * k[2]
                - (1.0 / 6.0) * e[1].powi(3) * k[3]
                - 0.5 * e[1] * e[3] * k[2])
                * sb;
            coeffs[0][7] = t1 + t2 + t3 + t4 + t5;

            let t1 = -20.0 * dy;
            let t2 = (10.0 * e[0] + 2.0 * e[2] + (1.0 / 6.0) * e[4]) * sa;
            let t3 = (2.0 * e[0].powi(2) * k[0]
                + (1.0 / 6.0) * e[0].powi(3) * k[1]
                + 0.5 * e[0] * e[2] * k[0])
                * ca;
            let t4 = (10.0 * e[1] - 2.0 * e[3] + (1.0 / 6.0) * e[5]) * sb;
            let t5 = -(2.0 * e[1].powi(2) * k[2]
                - (1.0 / 6.0) * e[1].powi(3) * k[3]
                - 0.5 * e[1] * e[3] * k[2])
                * cb;
            coeffs[1][7] = t1 + t2 + t3 + t4 + t5;
        }

        let segment_length =
            gauss_legendre_integrate(|u| Self::s_dot_from_coeffs(&coeffs, u), 0.0, 1.0);

        Self {
            start_pose,
            end_pose,
            coeffs,
            segment_length,
        }
    }

    /// Evaluate the position (x, y) at parameter `u` in \[0, 1\].
    pub fn calc_point(&self, u: f64) -> (f64, f64) {
        let powers = [
            1.0,
            u,
            u.powi(2),
            u.powi(3),
            u.powi(4),
            u.powi(5),
            u.powi(6),
            u.powi(7),
        ];
        let x: f64 = self.coeffs[0].iter().zip(&powers).map(|(c, p)| c * p).sum();
        let y: f64 = self.coeffs[1].iter().zip(&powers).map(|(c, p)| c * p).sum();
        (x, y)
    }

    /// First derivative (dx/du, dy/du) at parameter `u`.
    pub fn calc_first_deriv(&self, u: f64) -> (f64, f64) {
        let dpowers = [
            1.0,
            2.0 * u,
            3.0 * u.powi(2),
            4.0 * u.powi(3),
            5.0 * u.powi(4),
            6.0 * u.powi(5),
            7.0 * u.powi(6),
        ];
        let dx: f64 = self.coeffs[0][1..]
            .iter()
            .zip(&dpowers)
            .map(|(c, p)| c * p)
            .sum();
        let dy: f64 = self.coeffs[1][1..]
            .iter()
            .zip(&dpowers)
            .map(|(c, p)| c * p)
            .sum();
        (dx, dy)
    }

    /// Second derivative (d^2x/du^2, d^2y/du^2) at parameter `u`.
    pub fn calc_second_deriv(&self, u: f64) -> (f64, f64) {
        let ddpowers = [
            2.0,
            6.0 * u,
            12.0 * u.powi(2),
            20.0 * u.powi(3),
            30.0 * u.powi(4),
            42.0 * u.powi(5),
        ];
        let ddx: f64 = self.coeffs[0][2..]
            .iter()
            .zip(&ddpowers)
            .map(|(c, p)| c * p)
            .sum();
        let ddy: f64 = self.coeffs[1][2..]
            .iter()
            .zip(&ddpowers)
            .map(|(c, p)| c * p)
            .sum();
        (ddx, ddy)
    }

    /// Rate of change of arc length with respect to u: ||dr/du||.
    /// Clamped to a minimum of 1e-6 to avoid division by zero.
    pub fn s_dot(&self, u: f64) -> f64 {
        Self::s_dot_from_coeffs(&self.coeffs, u)
    }

    fn s_dot_from_coeffs(coeffs: &[[f64; 8]; 2], u: f64) -> f64 {
        let dpowers = [
            1.0,
            2.0 * u,
            3.0 * u.powi(2),
            4.0 * u.powi(3),
            5.0 * u.powi(4),
            6.0 * u.powi(5),
            7.0 * u.powi(6),
        ];
        let dx: f64 = coeffs[0][1..]
            .iter()
            .zip(&dpowers)
            .map(|(c, p)| c * p)
            .sum();
        let dy: f64 = coeffs[1][1..]
            .iter()
            .zip(&dpowers)
            .map(|(c, p)| c * p)
            .sum();
        (dx * dx + dy * dy).sqrt().max(1e-6)
    }

    /// Arc length from u=0 to `u_end`.
    pub fn arc_length(&self, u_end: f64) -> f64 {
        gauss_legendre_integrate(|u| self.s_dot(u), 0.0, u_end)
    }
}

/// A multi-segment eta^3 path composed of contiguous segments.
#[derive(Debug, Clone)]
pub struct Eta3Path {
    pub segments: Vec<Eta3PathSegment>,
}

impl Eta3Path {
    /// Create a path from a list of segments.
    ///
    /// Panics if the segment list is empty.
    pub fn new(segments: Vec<Eta3PathSegment>) -> Self {
        assert!(!segments.is_empty(), "At least one segment is required");
        Self { segments }
    }

    /// Evaluate the path at a normalised parameter `u` in \[0, num_segments\].
    ///
    /// Integer values correspond to segment boundaries.
    pub fn calc_path_point(&self, u: f64) -> (f64, f64) {
        let n = self.segments.len();
        let (seg_idx, local_u) = if (u - n as f64).abs() < 1e-12 || u >= n as f64 {
            (n - 1, 1.0)
        } else {
            let idx = u.floor() as usize;
            (idx.min(n - 1), u - idx as f64)
        };
        self.segments[seg_idx].calc_point(local_u)
    }

    /// Generate a sampled path as a vector of (x, y) points.
    ///
    /// `num_points` is the total number of samples across all segments.
    pub fn sample(&self, num_points: usize) -> Vec<(f64, f64)> {
        let n = self.segments.len() as f64;
        (0..num_points)
            .map(|i| {
                let u = n * i as f64 / (num_points - 1) as f64;
                self.calc_path_point(u)
            })
            .collect()
    }

    /// Total arc length of all segments.
    pub fn total_length(&self) -> f64 {
        self.segments.iter().map(|s| s.segment_length).sum()
    }
}

// ---------------------------------------------------------------------------
// Trajectory (velocity-profiled path)
// ---------------------------------------------------------------------------

/// Configuration for the trapezoidal-with-jerk velocity profile.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryConfig {
    pub max_vel: f64,
    pub v0: f64,
    pub a0: f64,
    pub max_accel: f64,
    pub max_jerk: f64,
}

impl TrajectoryConfig {
    pub fn new(max_vel: f64, max_accel: f64, max_jerk: f64) -> Self {
        Self {
            max_vel,
            v0: 0.0,
            a0: 0.0,
            max_accel,
            max_jerk,
        }
    }
}

/// Seven-section velocity profile (jerk-limited S-curve).
#[derive(Debug, Clone)]
struct VelocityProfile {
    /// Duration of each of the 7 sections.
    times: [f64; 7],
    /// Velocity at the end of each section.
    vels: [f64; 7],
    /// Arc-length traversed in each section.
    seg_lengths: [f64; 7],
    max_vel: f64,
    v0: f64,
    max_accel: f64,
    max_jerk: f64,
    total_time: f64,
    total_length: f64,
}

impl VelocityProfile {
    fn compute(config: &TrajectoryConfig, total_length: f64) -> Self {
        let max_jerk = config.max_jerk;
        let max_accel = config.max_accel;
        let v0 = config.v0;
        let a0 = config.a0;

        // Section 0: max jerk up to max acceleration
        let delta_a = max_accel - a0;
        let t_s1 = delta_a / max_jerk;
        let v_s1 = v0 + a0 * t_s1 + max_jerk * t_s1.powi(2) / 2.0;
        let s_s1 = v0 * t_s1 + a0 * t_s1.powi(2) / 2.0 + max_jerk * t_s1.powi(3) / 6.0;

        // Final section parameters
        let t_sf = max_accel / max_jerk;
        let v_sf = max_jerk * t_sf.powi(2) / 2.0;
        let s_sf = max_jerk * t_sf.powi(3) / 6.0;

        // Solve quadratic for achievable max velocity
        let a_coeff = 1.0 / max_accel;
        let b_coeff = 1.5 * max_accel / max_jerk + v_s1 / max_accel
            - (max_accel.powi(2) / max_jerk + v_s1) / max_accel;
        let c_coeff = s_s1 + s_sf
            - total_length
            - 7.0 * max_accel.powi(3) / (3.0 * max_jerk.powi(2))
            - v_s1 * (max_accel / max_jerk + v_s1 / max_accel)
            + (max_accel.powi(2) / max_jerk + v_s1 / max_accel).powi(2) / (2.0 * max_accel);

        let discriminant = b_coeff.powi(2) - 4.0 * a_coeff * c_coeff;
        let v_max_achievable = (-b_coeff + discriminant.max(0.0).sqrt()) / (2.0 * a_coeff);
        let max_vel = config.max_vel.min(v_max_achievable);

        let mut times = [0.0f64; 7];
        let mut vels = [0.0f64; 7];
        let mut seg_lengths = [0.0f64; 7];

        // Section 0
        times[0] = t_s1;
        vels[0] = v_s1;
        seg_lengths[0] = s_s1;

        // Section 1: accelerate at max_accel
        let dv1 = (max_vel - max_jerk * (max_accel / max_jerk).powi(2) / 2.0) - vels[0];
        times[1] = dv1 / max_accel;
        vels[1] = vels[0] + max_accel * times[1];
        seg_lengths[1] = vels[0] * times[1] + max_accel * times[1].powi(2) / 2.0;

        // Section 2: decrease acceleration to 0
        times[2] = max_accel / max_jerk;
        vels[2] = vels[1] + max_accel * times[2] - max_jerk * times[2].powi(2) / 2.0;
        seg_lengths[2] = vels[1] * times[2] + max_accel * times[2].powi(2) / 2.0
            - max_jerk * times[2].powi(3) / 6.0;

        // Section 4: negative jerk
        times[4] = max_accel / max_jerk;
        vels[4] = max_vel - max_jerk * times[4].powi(2) / 2.0;
        seg_lengths[4] = max_vel * times[4] - max_jerk * times[4].powi(3) / 6.0;

        // Section 5: decelerate at max rate
        let dv5 = vels[4] - v_sf;
        times[5] = dv5 / max_accel;
        vels[5] = vels[4] - max_accel * times[5];
        seg_lengths[5] = vels[4] * times[5] - max_accel * times[5].powi(2) / 2.0;

        // Section 6: final jerk to zero velocity
        times[6] = t_sf;
        vels[6] = vels[5] - max_jerk * t_sf.powi(2) / 2.0;
        seg_lengths[6] = s_sf;

        // Section 3: cruise (fill remaining distance)
        let used: f64 = seg_lengths.iter().sum();
        if used < total_length {
            seg_lengths[3] = total_length - used;
            vels[3] = max_vel;
            times[3] = seg_lengths[3] / max_vel;
        }

        let total_time: f64 = times.iter().sum();

        Self {
            times,
            vels,
            seg_lengths,
            max_vel,
            v0,
            max_accel,
            max_jerk,
            total_time,
            total_length,
        }
    }

    /// Compute (linear_velocity, arc_length, linear_acceleration) at a given time.
    fn query(&self, time: f64) -> (f64, f64, f64) {
        let cum_time = |n: usize| -> f64 { self.times[..n].iter().sum() };

        if time <= self.times[0] {
            let v = self.v0 + self.max_jerk * time.powi(2) / 2.0;
            let s = self.v0 * time + self.max_jerk * time.powi(3) / 6.0;
            let a = self.max_jerk * time;
            (v, s, a)
        } else if time <= cum_time(2) {
            let dt = time - cum_time(1);
            let v = self.vels[0] + self.max_accel * dt;
            let s = self.seg_lengths[0] + self.vels[0] * dt + self.max_accel * dt.powi(2) / 2.0;
            (v, s, self.max_accel)
        } else if time <= cum_time(3) {
            let dt = time - cum_time(2);
            let v = self.vels[1] + self.max_accel * dt - self.max_jerk * dt.powi(2) / 2.0;
            let s = self.seg_lengths[..2].iter().sum::<f64>()
                + self.vels[1] * dt
                + self.max_accel * dt.powi(2) / 2.0
                - self.max_jerk * dt.powi(3) / 6.0;
            let a = self.max_accel - self.max_jerk * dt;
            (v, s, a)
        } else if time <= cum_time(4) {
            let dt = time - cum_time(3);
            let v = self.vels[3];
            let s = self.seg_lengths[..3].iter().sum::<f64>() + self.vels[3] * dt;
            (v, s, 0.0)
        } else if time <= cum_time(5) {
            let dt = time - cum_time(4);
            let v = self.vels[3] - self.max_jerk * dt.powi(2) / 2.0;
            let s = self.seg_lengths[..4].iter().sum::<f64>() + self.vels[3] * dt
                - self.max_jerk * dt.powi(3) / 6.0;
            let a = -self.max_jerk * dt;
            (v, s, a)
        } else if time <= cum_time(6) {
            let dt = time - cum_time(5);
            let v = self.vels[4] - self.max_accel * dt;
            let s = self.seg_lengths[..5].iter().sum::<f64>() + self.vels[4] * dt
                - self.max_accel * dt.powi(2) / 2.0;
            (v, s, -self.max_accel)
        } else if time < self.total_time {
            let dt = time - cum_time(6);
            let v = self.vels[5] - self.max_accel * dt + self.max_jerk * dt.powi(2) / 2.0;
            let s = self.seg_lengths[..6].iter().sum::<f64>() + self.vels[5] * dt
                - self.max_accel * dt.powi(2) / 2.0
                + self.max_jerk * dt.powi(3) / 6.0;
            let a = -self.max_accel + self.max_jerk * dt;
            (v, s, a)
        } else {
            (0.0, self.total_length, 0.0)
        }
    }
}

/// A trajectory state at a single instant.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub linear_velocity: f64,
    pub angular_velocity: f64,
}

/// Eta^3 spline trajectory: a path with a jerk-limited velocity profile.
#[derive(Debug, Clone)]
pub struct Eta3Trajectory {
    path: Eta3Path,
    profile: VelocityProfile,
    /// Cumulative arc lengths at segment boundaries (starts with 0).
    cum_lengths: Vec<f64>,
}

impl Eta3Trajectory {
    /// Build a trajectory from path segments and kinematic constraints.
    pub fn new(segments: Vec<Eta3PathSegment>, config: TrajectoryConfig) -> Self {
        let path = Eta3Path::new(segments);
        let total_length = path.total_length();
        let profile = VelocityProfile::compute(&config, total_length);

        let mut cum_lengths = Vec::with_capacity(path.segments.len() + 1);
        cum_lengths.push(0.0);
        let mut acc = 0.0;
        for seg in &path.segments {
            acc += seg.segment_length;
            cum_lengths.push(acc);
        }

        Self {
            path,
            profile,
            cum_lengths,
        }
    }

    /// Total trajectory time \[s\].
    pub fn total_time(&self) -> f64 {
        self.profile.total_time
    }

    /// Evaluate the trajectory state at a given time.
    pub fn calc_traj_point(&self, time: f64) -> TrajectoryState {
        let (linear_velocity, s, linear_accel) = self.profile.query(time);

        // Find which path segment contains arc-length s
        let n = self.path.segments.len();
        let mut seg_id = 0;
        for i in (0..n).rev() {
            if s >= self.cum_lengths[i] {
                seg_id = i;
                break;
            }
        }
        if seg_id >= n {
            seg_id = n - 1;
        }

        let ui = if seg_id == n - 1 && (s - self.cum_lengths[n]).abs() < 1e-9 {
            1.0
        } else {
            let local_s = s - self.cum_lengths[seg_id];
            self.get_interp_param(seg_id, local_s)
        };

        let seg = &self.path.segments[seg_id];
        let pos = seg.calc_point(ui);
        let d = seg.calc_first_deriv(ui);
        let dd = seg.calc_second_deriv(ui);
        let su = seg.s_dot(ui);

        let angular_velocity = if su.abs() > 1e-6 && linear_velocity.abs() > 1e-6 {
            let ut = linear_velocity / su;
            let utt = linear_accel / su - (d.0 * dd.0 + d.1 * dd.1) / su.powi(2) * ut;
            let xt = d.0 * ut;
            let yt = d.1 * ut;
            let xtt = dd.0 * ut.powi(2) + d.0 * utt;
            let ytt = dd.1 * ut.powi(2) + d.1 * utt;
            (ytt * xt - xtt * yt) / linear_velocity.powi(2)
        } else {
            0.0
        };

        TrajectoryState {
            x: pos.0,
            y: pos.1,
            theta: d.1.atan2(d.0),
            linear_velocity,
            angular_velocity,
        }
    }

    /// Newton's method to find `u` such that arc_length(u) == target_s.
    fn get_interp_param(&self, seg_id: usize, target_s: f64) -> f64 {
        let seg = &self.path.segments[seg_id];
        // Initial guess proportional to target fraction
        let mut ui = if seg.segment_length > 1e-9 {
            (target_s / seg.segment_length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let tol = 1e-3;
        for _ in 0..50 {
            let f = seg.arc_length(ui) - target_s;
            if f.abs() < tol {
                break;
            }
            let fp = seg.s_dot(ui);
            ui -= f / fp;
            ui = ui.clamp(0.0, 1.0);
        }
        ui
    }

    /// Sample the full trajectory at uniform time intervals.
    ///
    /// Returns a vector of `TrajectoryState` at `num_points` evenly spaced times
    /// from 0 to `total_time`.
    pub fn sample(&self, num_points: usize) -> Vec<TrajectoryState> {
        let dt = self.total_time() / (num_points - 1).max(1) as f64;
        (0..num_points)
            .map(|i| self.calc_traj_point(dt * i as f64))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Numerical integration (Gauss-Legendre 5-point, adaptive subdivision)
// ---------------------------------------------------------------------------

/// 5-point Gauss-Legendre quadrature on \[a, b\].
fn gauss_legendre_5(f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    // Nodes and weights for [-1, 1]
    const NODES: [f64; 5] = [
        -0.906_179_845_938_664,
        -0.538_469_310_105_683,
        0.0,
        0.538_469_310_105_683,
        0.906_179_845_938_664,
    ];
    const WEIGHTS: [f64; 5] = [
        0.236_926_885_056_189_1,
        0.478_628_670_499_366_5,
        0.568_888_888_888_889,
        0.478_628_670_499_366_5,
        0.236_926_885_056_189_1,
    ];

    let half = (b - a) / 2.0;
    let mid = (a + b) / 2.0;
    let mut sum = 0.0;
    for i in 0..5 {
        sum += WEIGHTS[i] * f(half * NODES[i] + mid);
    }
    sum * half
}

/// Adaptive Gauss-Legendre integration with subdivision.
fn gauss_legendre_integrate(f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    // Use 16 sub-intervals for good accuracy on polynomial-like integrands
    let n = 16;
    let h = (b - a) / n as f64;
    let mut total = 0.0;
    for i in 0..n {
        let lo = a + i as f64 * h;
        let hi = lo + h;
        total += gauss_legendre_5(&f, lo, hi);
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // Path segment tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_segment_endpoints() {
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let end = Pose2D::new(4.0, 3.0, 0.0);
        let eta = EtaParams::new([4.0, 4.0, 0.0, 0.0, 0.0, 0.0]);
        let kappa = KappaParams::zeros();
        let seg = Eta3PathSegment::new(start, end, &eta, &kappa);

        let p0 = seg.calc_point(0.0);
        assert!(approx_eq(p0.0, 0.0, 1e-12));
        assert!(approx_eq(p0.1, 0.0, 1e-12));

        let p1 = seg.calc_point(1.0);
        assert!(approx_eq(p1.0, 4.0, 1e-10));
        assert!(approx_eq(p1.1, 3.0, 1e-10));
    }

    #[test]
    fn test_segment_with_heading() {
        let start = Pose2D::new(0.0, 0.0, PI / 4.0);
        let end = Pose2D::new(5.0, 5.0, PI / 4.0);
        let eta = EtaParams::new([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]);
        let kappa = KappaParams::zeros();
        let seg = Eta3PathSegment::new(start, end, &eta, &kappa);

        let p0 = seg.calc_point(0.0);
        assert!(approx_eq(p0.0, 0.0, 1e-12));
        assert!(approx_eq(p0.1, 0.0, 1e-12));

        let p1 = seg.calc_point(1.0);
        assert!(approx_eq(p1.0, 5.0, 1e-10));
        assert!(approx_eq(p1.1, 5.0, 1e-10));
    }

    #[test]
    fn test_segment_arc_length_positive() {
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let end = Pose2D::new(4.0, 3.0, 0.0);
        let eta = EtaParams::new([4.27, 4.27, 0.0, 0.0, 0.0, 0.0]);
        let kappa = KappaParams::zeros();
        let seg = Eta3PathSegment::new(start, end, &eta, &kappa);

        assert!(seg.segment_length > 0.0);
        // Arc length should be at least the straight-line distance
        let straight = ((4.0f64).powi(2) + (3.0f64).powi(2)).sqrt();
        assert!(seg.segment_length >= straight - 1e-6);
    }

    #[test]
    fn test_segment_derivatives_at_start() {
        let start = Pose2D::new(1.0, 2.0, 0.5);
        let end = Pose2D::new(5.0, 4.0, 0.0);
        let eta = EtaParams::new([3.0, 3.0, 0.0, 0.0, 0.0, 0.0]);
        let kappa = KappaParams::zeros();
        let seg = Eta3PathSegment::new(start, end, &eta, &kappa);

        // At u=0, the first derivative direction should match the start heading
        let d = seg.calc_first_deriv(0.0);
        let heading = d.1.atan2(d.0);
        assert!(approx_eq(heading, 0.5, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Multi-segment path tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_path_continuity() {
        let segments = vec![
            Eta3PathSegment::new(
                Pose2D::new(0.0, 0.0, 0.0),
                Pose2D::new(4.0, 1.5, 0.0),
                &EtaParams::new([4.27, 4.27, 0.0, 0.0, 0.0, 0.0]),
                &KappaParams::zeros(),
            ),
            Eta3PathSegment::new(
                Pose2D::new(4.0, 1.5, 0.0),
                Pose2D::new(5.5, 1.5, 0.0),
                &EtaParams::zeros(),
                &KappaParams::zeros(),
            ),
        ];
        let path = Eta3Path::new(segments);

        // At the junction u=1.0, path should be continuous
        let p_end_seg0 = path.calc_path_point(1.0 - 1e-12);
        let p_start_seg1 = path.calc_path_point(1.0);

        assert!(approx_eq(p_end_seg0.0, p_start_seg1.0, 1e-4));
        assert!(approx_eq(p_end_seg0.1, p_start_seg1.1, 1e-4));
    }

    #[test]
    fn test_path_sample() {
        let seg = Eta3PathSegment::new(
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(4.0, 3.0, 0.0),
            &EtaParams::new([4.0, 4.0, 0.0, 0.0, 0.0, 0.0]),
            &KappaParams::zeros(),
        );
        let path = Eta3Path::new(vec![seg]);
        let pts = path.sample(101);
        assert_eq!(pts.len(), 101);

        // First and last should match endpoints
        assert!(approx_eq(pts[0].0, 0.0, 1e-12));
        assert!(approx_eq(pts[0].1, 0.0, 1e-12));
        assert!(approx_eq(pts[100].0, 4.0, 1e-10));
        assert!(approx_eq(pts[100].1, 3.0, 1e-10));
    }

    #[test]
    fn test_varying_eta_produces_different_paths() {
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let end = Pose2D::new(4.0, 3.0, 0.0);
        let kappa = KappaParams::zeros();

        let seg1 = Eta3PathSegment::new(
            start,
            end,
            &EtaParams::new([2.0, 2.0, 0.0, 0.0, 0.0, 0.0]),
            &kappa,
        );
        let seg2 = Eta3PathSegment::new(
            start,
            end,
            &EtaParams::new([2.0, 2.0, 5.0, 5.0, 0.0, 0.0]),
            &kappa,
        );

        // Mid-point should differ when higher-order eta params change
        let mid1 = seg1.calc_point(0.5);
        let mid2 = seg2.calc_point(0.5);
        let dist = ((mid1.0 - mid2.0).powi(2) + (mid1.1 - mid2.1).powi(2)).sqrt();
        assert!(
            dist > 1e-6,
            "Different eta should produce different paths, dist={dist}"
        );
    }

    #[test]
    fn test_curvature_params_effect() {
        let start = Pose2D::new(5.5, 1.5, 0.0);
        let end = Pose2D::new(7.4377, 1.8235, 0.6667);
        let eta = EtaParams::new([1.88, 1.88, 0.0, 0.0, 0.0, 0.0]);

        let seg_no_curv = Eta3PathSegment::new(start, end, &eta, &KappaParams::zeros());
        let seg_with_curv =
            Eta3PathSegment::new(start, end, &eta, &KappaParams::new([0.0, 0.0, 1.0, 1.0]));

        let mid1 = seg_no_curv.calc_point(0.5);
        let mid2 = seg_with_curv.calc_point(0.5);
        let dist = ((mid1.0 - mid2.0).powi(2) + (mid1.1 - mid2.1).powi(2)).sqrt();
        assert!(dist > 1e-6, "Curvature params should affect the path shape");
    }

    // -----------------------------------------------------------------------
    // Reference test (Table 1 from the paper, matches Python test3)
    // -----------------------------------------------------------------------

    #[test]
    fn test_reference_multi_segment_path() {
        let segments = vec![
            // Lane-change
            Eta3PathSegment::new(
                Pose2D::new(0.0, 0.0, 0.0),
                Pose2D::new(4.0, 1.5, 0.0),
                &EtaParams::new([4.27, 4.27, 0.0, 0.0, 0.0, 0.0]),
                &KappaParams::zeros(),
            ),
            // Line
            Eta3PathSegment::new(
                Pose2D::new(4.0, 1.5, 0.0),
                Pose2D::new(5.5, 1.5, 0.0),
                &EtaParams::zeros(),
                &KappaParams::zeros(),
            ),
            // Cubic spiral
            Eta3PathSegment::new(
                Pose2D::new(5.5, 1.5, 0.0),
                Pose2D::new(7.4377, 1.8235, 0.6667),
                &EtaParams::new([1.88, 1.88, 0.0, 0.0, 0.0, 0.0]),
                &KappaParams::new([0.0, 0.0, 1.0, 1.0]),
            ),
            // Generic twirl arc
            Eta3PathSegment::new(
                Pose2D::new(7.4377, 1.8235, 0.6667),
                Pose2D::new(7.8, 4.3, 1.8),
                &EtaParams::new([7.0, 10.0, 10.0, -10.0, 4.0, 4.0]),
                &KappaParams::new([1.0, 1.0, 0.5, 0.0]),
            ),
            // Circular arc
            Eta3PathSegment::new(
                Pose2D::new(7.8, 4.3, 1.8),
                Pose2D::new(5.4581, 5.8064, 3.3416),
                &EtaParams::new([2.98, 2.98, 0.0, 0.0, 0.0, 0.0]),
                &KappaParams::new([0.5, 0.0, 0.5, 0.0]),
            ),
        ];

        let path = Eta3Path::new(segments);
        let pts = path.sample(1001);

        // Check endpoints
        assert!(approx_eq(pts[0].0, 0.0, 1e-10));
        assert!(approx_eq(pts[0].1, 0.0, 1e-10));
        assert!(approx_eq(pts[1000].0, 5.4581, 1e-4));
        assert!(approx_eq(pts[1000].1, 5.8064, 1e-4));

        // Total path length should be reasonable (roughly 20 units)
        let total_len = path.total_length();
        assert!(total_len > 10.0 && total_len < 40.0);
    }

    // -----------------------------------------------------------------------
    // Trajectory tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_trajectory_basic() {
        let seg = Eta3PathSegment::new(
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(4.0, 3.0, 0.0),
            &EtaParams::new([4.27, 4.27, 0.0, 0.0, 0.0, 0.0]),
            &KappaParams::zeros(),
        );

        let config = TrajectoryConfig::new(0.5, 0.5, 5.0);
        let traj = Eta3Trajectory::new(vec![seg], config);

        assert!(traj.total_time() > 0.0);

        // At t=0, should be at start
        let s0 = traj.calc_traj_point(0.0);
        assert!(approx_eq(s0.x, 0.0, 1e-6));
        assert!(approx_eq(s0.y, 0.0, 1e-6));
        assert!(approx_eq(s0.linear_velocity, 0.0, 1e-6));

        // At t=total_time, should be at end with zero velocity
        let sf = traj.calc_traj_point(traj.total_time());
        assert!(approx_eq(sf.x, 4.0, 0.1));
        assert!(approx_eq(sf.y, 3.0, 0.1));
        assert!(approx_eq(sf.linear_velocity, 0.0, 1e-6));
    }

    #[test]
    fn test_trajectory_sample() {
        let seg = Eta3PathSegment::new(
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(4.0, 3.0, 0.0),
            &EtaParams::new([4.0, 4.0, 0.0, 0.0, 0.0, 0.0]),
            &KappaParams::zeros(),
        );

        let config = TrajectoryConfig::new(1.0, 0.5, 5.0);
        let traj = Eta3Trajectory::new(vec![seg], config);
        let states = traj.sample(101);

        assert_eq!(states.len(), 101);

        // Velocity should never exceed max_vel (with some numerical tolerance)
        for s in &states {
            assert!(s.linear_velocity <= 1.0 + 1e-6);
            assert!(s.linear_velocity >= -1e-6);
        }
    }

    #[test]
    fn test_trajectory_monotonic_position() {
        // For a straight-ish segment, x should increase monotonically
        let seg = Eta3PathSegment::new(
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(10.0, 0.0, 0.0),
            &EtaParams::new([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]),
            &KappaParams::zeros(),
        );

        let config = TrajectoryConfig::new(2.0, 1.0, 5.0);
        let traj = Eta3Trajectory::new(vec![seg], config);
        let states = traj.sample(200);

        for i in 1..states.len() {
            assert!(
                states[i].x >= states[i - 1].x - 1e-6,
                "x should increase monotonically for a straight segment: x[{}]={} < x[{}]={}",
                i,
                states[i].x,
                i - 1,
                states[i - 1].x,
            );
        }
    }

    // -----------------------------------------------------------------------
    // Integration test
    // -----------------------------------------------------------------------

    #[test]
    fn test_gauss_legendre_accuracy() {
        // Integrate x^2 from 0 to 1, exact answer = 1/3
        let result = gauss_legendre_integrate(|x| x * x, 0.0, 1.0);
        assert!(approx_eq(result, 1.0 / 3.0, 1e-12));

        // Integrate sin(x) from 0 to pi, exact answer = 2
        let result = gauss_legendre_integrate(|x| x.sin(), 0.0, PI);
        assert!(approx_eq(result, 2.0, 1e-10));
    }
}
