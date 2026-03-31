#![allow(dead_code)]

//! Frenet Optimal Trajectory planning
//!
//! Generates optimal trajectories in the Frenet frame, considering
//! lateral and longitudinal motion with jerk minimization.

// Parameters
const MAX_SPEED: f64 = 50.0 / 3.6;
const MAX_ACCEL: f64 = 5.0;
const MAX_CURVATURE: f64 = 1.0;
const MAX_ROAD_WIDTH: f64 = 7.0;
const D_ROAD_W: f64 = 1.0;
const DT: f64 = 0.2;
const MAX_T: f64 = 5.0;
const MIN_T: f64 = 4.0;
const TARGET_SPEED: f64 = 30.0 / 3.6;
const D_T_S: f64 = 5.0 / 3.6;
const N_S_SAMPLE: usize = 1;
const ROBOT_RADIUS: f64 = 2.0;

// Cost weights
const K_J: f64 = 0.1;
const K_T: f64 = 0.1;
const K_S_DOT: f64 = 1.0;
const K_D: f64 = 1.0;
const K_S: f64 = 1.0;
const K_LAT: f64 = 1.0;
const K_LON: f64 = 1.0;

/// Quintic polynomial for trajectory generation
struct QuinticPolynomial {
    a0: f64,
    a1: f64,
    a2: f64,
    a3: f64,
    a4: f64,
    a5: f64,
}

impl QuinticPolynomial {
    fn new(xs: f64, vxs: f64, axs: f64, xe: f64, vxe: f64, axe: f64, time: f64) -> Self {
        let a0 = xs;
        let a1 = vxs;
        let a2 = axs / 2.0;

        let t2 = time * time;
        let t3 = t2 * time;
        let t4 = t3 * time;
        let t5 = t4 * time;

        let a = nalgebra::Matrix3::new(
            t3,
            t4,
            t5,
            3.0 * t2,
            4.0 * t3,
            5.0 * t4,
            6.0 * time,
            12.0 * t2,
            20.0 * t3,
        );

        let b = nalgebra::Vector3::new(
            xe - a0 - a1 * time - a2 * t2,
            vxe - a1 - 2.0 * a2 * time,
            axe - 2.0 * a2,
        );

        let x = a
            .try_inverse()
            .map(|inv| inv * b)
            .unwrap_or(nalgebra::Vector3::zeros());

        QuinticPolynomial {
            a0,
            a1,
            a2,
            a3: x[0],
            a4: x[1],
            a5: x[2],
        }
    }

    fn calc_point(&self, t: f64) -> f64 {
        self.a0
            + self.a1 * t
            + self.a2 * t.powi(2)
            + self.a3 * t.powi(3)
            + self.a4 * t.powi(4)
            + self.a5 * t.powi(5)
    }

    fn calc_first_derivative(&self, t: f64) -> f64 {
        self.a1
            + 2.0 * self.a2 * t
            + 3.0 * self.a3 * t.powi(2)
            + 4.0 * self.a4 * t.powi(3)
            + 5.0 * self.a5 * t.powi(4)
    }

    fn calc_second_derivative(&self, t: f64) -> f64 {
        2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t.powi(2) + 20.0 * self.a5 * t.powi(3)
    }

    fn calc_third_derivative(&self, t: f64) -> f64 {
        6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t.powi(2)
    }
}

/// Quartic polynomial for velocity keeping
struct QuarticPolynomial {
    a0: f64,
    a1: f64,
    a2: f64,
    a3: f64,
    a4: f64,
}

impl QuarticPolynomial {
    fn new(xs: f64, vxs: f64, axs: f64, vxe: f64, axe: f64, time: f64) -> Self {
        let a0 = xs;
        let a1 = vxs;
        let a2 = axs / 2.0;

        let t2 = time * time;
        let t3 = t2 * time;

        let a = nalgebra::Matrix2::new(3.0 * t2, 4.0 * t3, 6.0 * time, 12.0 * t2);

        let b = nalgebra::Vector2::new(vxe - a1 - 2.0 * a2 * time, axe - 2.0 * a2);

        let x = a
            .try_inverse()
            .map(|inv| inv * b)
            .unwrap_or(nalgebra::Vector2::zeros());

        QuarticPolynomial {
            a0,
            a1,
            a2,
            a3: x[0],
            a4: x[1],
        }
    }

    fn calc_point(&self, t: f64) -> f64 {
        self.a0 + self.a1 * t + self.a2 * t.powi(2) + self.a3 * t.powi(3) + self.a4 * t.powi(4)
    }

    fn calc_first_derivative(&self, t: f64) -> f64 {
        self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t.powi(2) + 4.0 * self.a4 * t.powi(3)
    }

    fn calc_second_derivative(&self, t: f64) -> f64 {
        2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t.powi(2)
    }

    fn calc_third_derivative(&self, t: f64) -> f64 {
        6.0 * self.a3 + 24.0 * self.a4 * t
    }
}

/// Frenet path
#[derive(Clone)]
pub struct FrenetPath {
    pub t: Vec<f64>,
    pub d: Vec<f64>,
    pub d_d: Vec<f64>,
    pub d_dd: Vec<f64>,
    pub d_ddd: Vec<f64>,
    pub s: Vec<f64>,
    pub s_d: Vec<f64>,
    pub s_dd: Vec<f64>,
    pub s_ddd: Vec<f64>,
    pub cd: f64,
    pub cv: f64,
    pub cf: f64,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub yaw: Vec<f64>,
    pub v: Vec<f64>,
    pub a: Vec<f64>,
    pub ds: Vec<f64>,
    pub c: Vec<f64>,
}

impl FrenetPath {
    pub fn new() -> Self {
        FrenetPath {
            t: Vec::new(),
            d: Vec::new(),
            d_d: Vec::new(),
            d_dd: Vec::new(),
            d_ddd: Vec::new(),
            s: Vec::new(),
            s_d: Vec::new(),
            s_dd: Vec::new(),
            s_ddd: Vec::new(),
            cd: 0.0,
            cv: 0.0,
            cf: 0.0,
            x: Vec::new(),
            y: Vec::new(),
            yaw: Vec::new(),
            v: Vec::new(),
            a: Vec::new(),
            ds: Vec::new(),
            c: Vec::new(),
        }
    }

    fn pop_front(&mut self) {
        self.x.remove(0);
        self.y.remove(0);
        self.yaw.remove(0);
        self.v.remove(0);
        self.a.remove(0);
        self.s.remove(0);
        self.s_d.remove(0);
        self.s_dd.remove(0);
        self.s_ddd.remove(0);
        self.d.remove(0);
        self.d_d.remove(0);
        self.d_dd.remove(0);
        self.d_ddd.remove(0);
    }
}

impl Default for FrenetPath {
    fn default() -> Self {
        Self::new()
    }
}

/// Cubic spline for reference path
pub struct CubicSpline2D {
    pub s: Vec<f64>,
    sx: CubicSpline1D,
    sy: CubicSpline1D,
}

struct CubicSpline1D {
    x: Vec<f64>,
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
}

impl CubicSpline1D {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        let a = y.to_vec();
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];

        let h: Vec<f64> = (0..n - 1).map(|i| x[i + 1] - x[i]).collect();

        let mut alpha = vec![0.0; n];
        for i in 1..n - 1 {
            alpha[i] = 3.0 / h[i] * (a[i + 1] - a[i]) - 3.0 / h[i - 1] * (a[i] - a[i - 1]);
        }

        let mut l = vec![1.0; n];
        let mut mu = vec![0.0; n];
        let mut z = vec![0.0; n];

        for i in 1..n - 1 {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        for j in (0..n - 1).rev() {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        CubicSpline1D {
            x: x.to_vec(),
            a,
            b,
            c,
            d,
        }
    }

    fn calc(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        let dx = t - self.x[i];
        self.a[i] + self.b[i] * dx + self.c[i] * dx.powi(2) + self.d[i] * dx.powi(3)
    }

    fn calc_d(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        let dx = t - self.x[i];
        self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx.powi(2)
    }

    fn calc_dd(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        let dx = t - self.x[i];
        2.0 * self.c[i] + 6.0 * self.d[i] * dx
    }

    fn calc_ddd(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        6.0 * self.d[i]
    }

    fn search_index(&self, t: f64) -> usize {
        for i in 0..self.x.len() - 1 {
            if self.x[i] <= t && t < self.x[i + 1] {
                return i;
            }
        }
        self.x.len() - 2
    }
}

impl CubicSpline2D {
    pub fn new(x: &[f64], y: &[f64]) -> Self {
        let mut s = vec![0.0];
        for i in 1..x.len() {
            let ds = ((x[i] - x[i - 1]).powi(2) + (y[i] - y[i - 1]).powi(2)).sqrt();
            s.push(s[i - 1] + ds);
        }

        let sx = CubicSpline1D::new(&s, x);
        let sy = CubicSpline1D::new(&s, y);

        CubicSpline2D { s, sx, sy }
    }

    pub fn calc_position(&self, s: f64) -> (f64, f64) {
        (self.sx.calc(s), self.sy.calc(s))
    }

    pub fn calc_curvature(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let ddx = self.sx.calc_dd(s);
        let dy = self.sy.calc_d(s);
        let ddy = self.sy.calc_dd(s);
        (ddy * dx - ddx * dy) / (dx.powi(2) + dy.powi(2)).powf(1.5)
    }

    pub fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }

    pub fn calc_curvature_rate(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        let ddx = self.sx.calc_dd(s);
        let ddy = self.sy.calc_dd(s);
        let dddx = self.sx.calc_ddd(s);
        let dddy = self.sy.calc_ddd(s);
        let a = dx * ddy - dy * ddx;
        let b = dx * dddy - dy * dddx;
        let c = dx * ddx + dy * ddy;
        let d = dx * dx + dy * dy;
        (b * d - 3.0 * a * c) / (d * d * d)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PathCheckStatus {
    MaxSpeedError,
    MaxAccelError,
    MaxCurvatureError,
    CollisionError,
    Ok,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct PathCheckCounts {
    max_speed_error: usize,
    max_accel_error: usize,
    max_curvature_error: usize,
    collision_error: usize,
    ok: usize,
}

#[derive(Clone, Copy, Debug)]
struct ReferencePathState {
    s: f64,
    x: f64,
    y: f64,
    theta: f64,
    kappa: f64,
    dkappa: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FrenetInitialState {
    pub s0: f64,
    pub speed: f64,
    pub accel: f64,
    pub d: f64,
    pub d_d: f64,
    pub d_dd: f64,
}

#[derive(Clone, Copy, Debug)]
struct FrenetPlannerConfig {
    max_road_width: f64,
    d_road_w: f64,
    dt: f64,
    max_t: f64,
    min_t: f64,
    target_speed: f64,
    d_t_s: f64,
    n_s_sample: usize,
    robot_radius: f64,
    stop_s: f64,
    d_s: f64,
    n_stop_s_sample: usize,
}

impl FrenetPlannerConfig {
    fn high_speed() -> Self {
        Self {
            max_road_width: MAX_ROAD_WIDTH,
            d_road_w: D_ROAD_W,
            dt: DT,
            max_t: MAX_T,
            min_t: MIN_T,
            target_speed: TARGET_SPEED,
            d_t_s: D_T_S,
            n_s_sample: N_S_SAMPLE,
            robot_radius: ROBOT_RADIUS,
            stop_s: 25.0,
            d_s: 2.0,
            n_stop_s_sample: 4,
        }
    }

    fn low_speed() -> Self {
        Self {
            max_road_width: 1.0,
            d_road_w: 0.2,
            dt: DT,
            max_t: MAX_T,
            min_t: MIN_T,
            target_speed: 3.0 / 3.6,
            d_t_s: 0.5 / 3.6,
            n_s_sample: N_S_SAMPLE,
            robot_radius: 0.5,
            stop_s: 4.0,
            d_s: 0.3,
            n_stop_s_sample: 3,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum LateralStrategyKind {
    HighSpeed,
    LowSpeed,
}

#[derive(Clone, Copy, Debug)]
enum LongitudinalStrategyKind {
    VelocityKeeping,
    MergingAndStopping,
}

fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    debug_assert!(step > 0.0);
    if stop <= start {
        return Vec::new();
    }

    let count = ((stop - start) / step).ceil() as usize;
    (0..count).map(|i| start + step * i as f64).collect()
}

fn accumulating_arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    debug_assert!(step > 0.0);
    let mut values = Vec::new();
    let mut current = start;

    while current < stop {
        values.push(current);
        current += step;
    }

    values
}

fn target_speed_samples(config: FrenetPlannerConfig) -> Vec<f64> {
    arange(
        config.target_speed - config.d_t_s * config.n_s_sample as f64,
        config.target_speed + config.d_t_s * config.n_s_sample as f64,
        config.d_t_s,
    )
}

fn stop_position_samples(config: FrenetPlannerConfig) -> Vec<f64> {
    accumulating_arange(
        config.stop_s - config.d_s * config.n_stop_s_sample as f64,
        config.stop_s + config.d_s * config.n_stop_s_sample as f64,
        config.d_s,
    )
}

fn road_width_samples(
    config: FrenetPlannerConfig,
    s0: f64,
    longitudinal: LongitudinalStrategyKind,
) -> Vec<f64> {
    match longitudinal {
        LongitudinalStrategyKind::VelocityKeeping => accumulating_arange(
            -config.max_road_width,
            config.max_road_width,
            config.d_road_w,
        ),
        LongitudinalStrategyKind::MergingAndStopping => {
            if s0 < config.stop_s / 3.0 {
                accumulating_arange(
                    -config.max_road_width,
                    config.max_road_width,
                    config.d_road_w,
                )
            } else {
                vec![0.0]
            }
        }
    }
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = (angle + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI);
    if normalized < 0.0 {
        normalized += 2.0 * std::f64::consts::PI;
    }
    normalized - std::f64::consts::PI
}

fn frenet_to_cartesian(
    reference: ReferencePathState,
    s_condition: [f64; 3],
    d_condition: [f64; 3],
) -> (f64, f64, f64, f64, f64, f64) {
    debug_assert!((reference.s - s_condition[0]).abs() < 1e-6);

    let cos_theta_r = reference.theta.cos();
    let sin_theta_r = reference.theta.sin();

    let x = reference.x - sin_theta_r * d_condition[0];
    let y = reference.y + cos_theta_r * d_condition[0];

    let one_minus_kappa_r_d = 1.0 - reference.kappa * d_condition[0];
    let tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    let delta_theta = d_condition[1].atan2(one_minus_kappa_r_d);
    let cos_delta_theta = delta_theta.cos();

    let theta = normalize_angle(delta_theta + reference.theta);
    let kappa_r_d_prime = reference.dkappa * d_condition[0] + reference.kappa * d_condition[1];
    let kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta.powi(2))
        / one_minus_kappa_r_d
        + reference.kappa)
        * cos_delta_theta
        / one_minus_kappa_r_d;

    let d_dot = d_condition[1] * s_condition[1];
    let v = ((one_minus_kappa_r_d * s_condition[1]).powi(2) + d_dot.powi(2)).sqrt();

    let delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - reference.kappa;
    let a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta
        + s_condition[1].powi(2) / cos_delta_theta
            * (d_condition[1] * delta_theta_prime - kappa_r_d_prime);

    (x, y, theta, kappa, v, a)
}

fn build_velocity_keeping_path(
    state: FrenetInitialState,
    ti: f64,
    target_speed: f64,
    config: FrenetPlannerConfig,
) -> FrenetPath {
    let lon_qp = QuarticPolynomial::new(state.s0, state.speed, state.accel, target_speed, 0.0, ti);
    let mut fp = FrenetPath::new();
    fp.t = arange(0.0, ti, config.dt);

    for &t in &fp.t {
        fp.s.push(lon_qp.calc_point(t));
        fp.s_d.push(lon_qp.calc_first_derivative(t));
        fp.s_dd.push(lon_qp.calc_second_derivative(t));
        fp.s_ddd.push(lon_qp.calc_third_derivative(t));
    }

    fp
}

fn build_merging_stop_path(
    state: FrenetInitialState,
    ti: f64,
    stop_position: f64,
    config: FrenetPlannerConfig,
) -> FrenetPath {
    let lon_qp = QuinticPolynomial::new(
        state.s0,
        state.speed,
        state.accel,
        stop_position,
        0.0,
        0.0,
        ti,
    );
    let mut fp = FrenetPath::new();
    fp.t = arange(0.0, ti, config.dt);

    for &t in &fp.t {
        fp.s.push(lon_qp.calc_point(t));
        fp.s_d.push(lon_qp.calc_first_derivative(t));
        fp.s_dd.push(lon_qp.calc_second_derivative(t));
        fp.s_ddd.push(lon_qp.calc_third_derivative(t));
    }

    fp
}

fn calc_longitudinal_candidates(
    state: FrenetInitialState,
    ti: f64,
    config: FrenetPlannerConfig,
    longitudinal: LongitudinalStrategyKind,
) -> Vec<FrenetPath> {
    match longitudinal {
        LongitudinalStrategyKind::VelocityKeeping => target_speed_samples(config)
            .into_iter()
            .map(|target_speed| build_velocity_keeping_path(state, ti, target_speed, config))
            .collect(),
        LongitudinalStrategyKind::MergingAndStopping => {
            if state.s0 >= config.stop_s {
                return Vec::new();
            }

            stop_position_samples(config)
                .into_iter()
                .map(|stop_position| build_merging_stop_path(state, ti, stop_position, config))
                .collect()
        }
    }
}

fn apply_lateral_strategy(
    fp: &FrenetPath,
    di: f64,
    state: FrenetInitialState,
    ti: f64,
    lateral: LateralStrategyKind,
) -> FrenetPath {
    let mut tp = fp.clone();
    tp.d.clear();
    tp.d_d.clear();
    tp.d_dd.clear();
    tp.d_ddd.clear();

    match lateral {
        LateralStrategyKind::HighSpeed => {
            let s0_d = fp.s_d[0];
            let s0_dd = fp.s_dd[0];
            let lat_qp = QuinticPolynomial::new(
                state.d,
                state.d_d * s0_d,
                state.d_dd * s0_d.powi(2) + state.d_d * s0_dd,
                di,
                0.0,
                0.0,
                ti,
            );

            for i in 0..tp.t.len() {
                let t = tp.t[i];
                let s_d = tp.s_d[i];
                let s_dd = tp.s_dd[i];
                let s_d_inv = 1.0 / (s_d + 1e-6) + 1e-6;
                let s_d_inv_sq = s_d_inv * s_d_inv;

                let d = lat_qp.calc_point(t);
                let d_d_t = lat_qp.calc_first_derivative(t);
                let d_dd_t = lat_qp.calc_second_derivative(t);
                let d_ddd_t = lat_qp.calc_third_derivative(t);
                let d_d = d_d_t * s_d_inv;

                tp.d.push(d);
                tp.d_d.push(d_d);
                tp.d_dd.push((d_dd_t - d_d * s_dd) * s_d_inv_sq);
                tp.d_ddd.push(d_ddd_t);
            }
        }
        LateralStrategyKind::LowSpeed => {
            let s0 = fp.s[0];
            let s1 = *fp.s.last().unwrap_or(&s0);
            let lat_qp =
                QuinticPolynomial::new(state.d, state.d_d, state.d_dd, di, 0.0, 0.0, s1 - s0);

            for &s in &fp.s {
                let shifted_s = s - s0;
                tp.d.push(lat_qp.calc_point(shifted_s));
                tp.d_d.push(lat_qp.calc_first_derivative(shifted_s));
                tp.d_dd.push(lat_qp.calc_second_derivative(shifted_s));
                tp.d_ddd.push(lat_qp.calc_third_derivative(shifted_s));
            }
        }
    }

    tp
}

fn destination_cost(
    fp: &FrenetPath,
    config: FrenetPlannerConfig,
    longitudinal: LongitudinalStrategyKind,
) -> f64 {
    match longitudinal {
        LongitudinalStrategyKind::VelocityKeeping => {
            let speed_error = (config.target_speed - fp.s_d.last().unwrap_or(&0.0)).powi(2);
            K_S_DOT * speed_error
        }
        LongitudinalStrategyKind::MergingAndStopping => {
            let stop_error = (config.stop_s - fp.s.last().unwrap_or(&0.0)).powi(2);
            K_S * stop_error
        }
    }
}

fn classify_path_with_config(
    fp: &FrenetPath,
    ob: &[(f64, f64)],
    config: FrenetPlannerConfig,
) -> PathCheckStatus {
    if fp.v.iter().any(|&v| v > MAX_SPEED) {
        return PathCheckStatus::MaxSpeedError;
    }

    if fp.a.iter().any(|&a| a.abs() > MAX_ACCEL) {
        return PathCheckStatus::MaxAccelError;
    }

    if fp.c.iter().any(|&c| c.abs() > MAX_CURVATURE) {
        return PathCheckStatus::MaxCurvatureError;
    }

    for (ox, oy) in ob {
        for (&x, &y) in fp.x.iter().zip(fp.y.iter()) {
            let distance_sq = (x - ox).powi(2) + (y - oy).powi(2);
            if distance_sq <= config.robot_radius.powi(2) {
                return PathCheckStatus::CollisionError;
            }
        }
    }

    PathCheckStatus::Ok
}

fn classify_path(fp: &FrenetPath, ob: &[(f64, f64)]) -> PathCheckStatus {
    classify_path_with_config(fp, ob, FrenetPlannerConfig::high_speed())
}

fn summarize_paths_with_config(
    fplist: &[FrenetPath],
    ob: &[(f64, f64)],
    config: FrenetPlannerConfig,
) -> PathCheckCounts {
    let mut counts = PathCheckCounts::default();

    for fp in fplist {
        match classify_path_with_config(fp, ob, config) {
            PathCheckStatus::MaxSpeedError => counts.max_speed_error += 1,
            PathCheckStatus::MaxAccelError => counts.max_accel_error += 1,
            PathCheckStatus::MaxCurvatureError => counts.max_curvature_error += 1,
            PathCheckStatus::CollisionError => counts.collision_error += 1,
            PathCheckStatus::Ok => counts.ok += 1,
        }
    }

    counts
}

fn summarize_paths(fplist: &[FrenetPath], ob: &[(f64, f64)]) -> PathCheckCounts {
    summarize_paths_with_config(fplist, ob, FrenetPlannerConfig::high_speed())
}

/// Generate Frenet candidate paths
pub fn calc_frenet_paths(
    c_speed: f64,
    c_d: f64,
    c_d_d: f64,
    c_d_dd: f64,
    s0: f64,
) -> Vec<FrenetPath> {
    calc_frenet_paths_with_accel(c_speed, 0.0, c_d, c_d_d, c_d_dd, s0)
}

pub fn calc_frenet_paths_with_accel(
    c_speed: f64,
    c_accel: f64,
    c_d: f64,
    c_d_d: f64,
    c_d_dd: f64,
    s0: f64,
) -> Vec<FrenetPath> {
    calc_frenet_paths_for_strategy(
        FrenetInitialState {
            s0,
            speed: c_speed,
            accel: c_accel,
            d: c_d,
            d_d: c_d_d,
            d_dd: c_d_dd,
        },
        FrenetPlannerConfig::high_speed(),
        LateralStrategyKind::HighSpeed,
        LongitudinalStrategyKind::VelocityKeeping,
    )
}

fn calc_frenet_paths_for_strategy(
    state: FrenetInitialState,
    config: FrenetPlannerConfig,
    lateral: LateralStrategyKind,
    longitudinal: LongitudinalStrategyKind,
) -> Vec<FrenetPath> {
    let mut fplist = Vec::new();

    for ti in accumulating_arange(config.min_t, config.max_t, config.dt) {
        for fp in calc_longitudinal_candidates(state, ti, config, longitudinal) {
            for di in road_width_samples(config, state.s0, longitudinal) {
                let mut tp = apply_lateral_strategy(&fp, di, state, ti, lateral);
                let jp: f64 = tp.d_ddd.iter().map(|x| x.powi(2)).sum();
                let js: f64 = tp.s_ddd.iter().map(|x| x.powi(2)).sum();

                tp.cd = K_J * jp + K_T * ti + K_D * tp.d.last().unwrap_or(&0.0).powi(2);
                tp.cv = K_J * js + K_T * ti + destination_cost(&tp, config, longitudinal);
                tp.cf = K_LAT * tp.cd + K_LON * tp.cv;
                fplist.push(tp);
            }
        }
    }

    fplist
}

/// Calculate global positions from Frenet paths
pub fn calc_global_paths(fplist: &mut [FrenetPath], csp: &CubicSpline2D) {
    for fp in fplist.iter_mut() {
        fp.x.clear();
        fp.y.clear();
        fp.yaw.clear();
        fp.v.clear();
        fp.a.clear();
        fp.ds.clear();
        fp.c.clear();

        for i in 0..fp.s.len() {
            let s = fp.s[i];
            if s > *csp.s.last().unwrap_or(&0.0) {
                break;
            }

            let (ix, iy) = csp.calc_position(s);
            let i_yaw = csp.calc_yaw(s);
            let i_kappa = csp.calc_curvature(s);
            let i_dkappa = csp.calc_curvature_rate(s);
            let reference = ReferencePathState {
                s: fp.s[i],
                x: ix,
                y: iy,
                theta: i_yaw,
                kappa: i_kappa,
                dkappa: i_dkappa,
            };
            let s_condition = [fp.s[i], fp.s_d[i], fp.s_dd[i]];
            let d_condition = [fp.d[i], fp.d_d[i], fp.d_dd[i]];
            let (x, y, yaw, kappa, v, a) = frenet_to_cartesian(reference, s_condition, d_condition);
            fp.x.push(x);
            fp.y.push(y);
            fp.yaw.push(yaw);
            fp.v.push(v);
            fp.a.push(a);
            fp.c.push(kappa);
        }

        for i in 0..fp.x.len().saturating_sub(1) {
            let dx = fp.x[i + 1] - fp.x[i];
            let dy = fp.y[i + 1] - fp.y[i];
            fp.ds.push((dx.powi(2) + dy.powi(2)).sqrt());
        }

        if let Some(last_ds) = fp.ds.last().copied() {
            fp.ds.push(last_ds);
        }
    }
}

/// Check constraints and collisions
pub fn check_paths(fplist: &mut Vec<FrenetPath>, ob: &[(f64, f64)]) {
    fplist.retain(|fp| classify_path(fp, ob) == PathCheckStatus::Ok);
}

fn check_paths_with_config(
    fplist: &mut Vec<FrenetPath>,
    ob: &[(f64, f64)],
    config: FrenetPlannerConfig,
) {
    fplist.retain(|fp| classify_path_with_config(fp, ob, config) == PathCheckStatus::Ok);
}

/// Frenet optimal planning
pub fn frenet_optimal_planning(
    csp: &CubicSpline2D,
    s0: f64,
    c_speed: f64,
    c_d: f64,
    c_d_d: f64,
    c_d_dd: f64,
    ob: &[(f64, f64)],
) -> Option<FrenetPath> {
    frenet_optimal_planning_with_accel(
        csp,
        FrenetInitialState {
            s0,
            speed: c_speed,
            accel: 0.0,
            d: c_d,
            d_d: c_d_d,
            d_dd: c_d_dd,
        },
        ob,
    )
}

pub fn frenet_optimal_planning_with_accel(
    csp: &CubicSpline2D,
    state: FrenetInitialState,
    ob: &[(f64, f64)],
) -> Option<FrenetPath> {
    frenet_optimal_planning_for_strategy(
        csp,
        state,
        ob,
        FrenetPlannerConfig::high_speed(),
        LateralStrategyKind::HighSpeed,
        LongitudinalStrategyKind::VelocityKeeping,
    )
}

fn frenet_optimal_planning_for_strategy(
    csp: &CubicSpline2D,
    state: FrenetInitialState,
    ob: &[(f64, f64)],
    config: FrenetPlannerConfig,
    lateral: LateralStrategyKind,
    longitudinal: LongitudinalStrategyKind,
) -> Option<FrenetPath> {
    let mut fplist = calc_frenet_paths_for_strategy(state, config, lateral, longitudinal);
    calc_global_paths(&mut fplist, csp);
    check_paths_with_config(&mut fplist, ob, config);

    fplist
        .into_iter()
        .min_by(|a, b| a.cf.partial_cmp(&b.cf).unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;

    const UPSTREAM_WX: [f64; 5] = [0.0, 10.0, 20.5, 35.0, 70.5];
    const UPSTREAM_WY: [f64; 5] = [0.0, -6.0, 5.0, 6.5, 0.0];
    const UPSTREAM_OBSTACLES: [(f64, f64); 5] = [
        (20.0, 10.0),
        (30.0, 6.0),
        (30.0, 8.0),
        (35.0, 8.0),
        (50.0, 3.0),
    ];
    const LOW_SPEED_WX: [f64; 6] = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0];
    const LOW_SPEED_WY: [f64; 6] = [0.0, 0.0, 1.0, 0.0, -1.0, -2.0];
    const LOW_SPEED_OBSTACLES: [(f64, f64); 4] = [(3.0, 1.0), (5.0, 0.0), (6.0, 0.5), (8.0, -1.5)];
    const LOW_SPEED_STATE: FrenetInitialState = FrenetInitialState {
        s0: 0.0,
        speed: 1.0 / 3.6,
        accel: 0.0,
        d: 0.5,
        d_d: 0.0,
        d_dd: 0.0,
    };

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-6,
            "actual={actual}, expected={expected}"
        );
    }

    #[derive(Clone, Copy, Debug)]
    struct SimulationSnapshot {
        fallback: bool,
        cf: f64,
        len: usize,
        x1: f64,
        y1: f64,
        yaw1: f64,
        v1: f64,
        a1: f64,
        c1: f64,
        s1: f64,
        d1: f64,
        s_d1: f64,
        s_dd1: f64,
        d_d1: f64,
        d_dd1: f64,
        counts: PathCheckCounts,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum SimulationOutcomeKind {
        Goal,
        Finish,
    }

    fn sampled_course_goal(csp: &CubicSpline2D) -> (f64, f64) {
        let mut goal = csp.calc_position(0.0);
        for s in arange(0.0, *csp.s.last().unwrap_or(&0.0), 0.1) {
            goal = csp.calc_position(s);
        }
        goal
    }

    fn run_simulation_for_strategy(
        csp: &CubicSpline2D,
        initial_state: FrenetInitialState,
        obstacles: &[(f64, f64)],
        config: FrenetPlannerConfig,
        lateral: LateralStrategyKind,
        longitudinal: LongitudinalStrategyKind,
        max_steps: usize,
    ) -> (Vec<SimulationSnapshot>, SimulationOutcomeKind, usize) {
        let goal = sampled_course_goal(csp);
        let mut state = initial_state;
        let mut last_path: Option<FrenetPath> = None;
        let mut trace = Vec::new();

        for step in 0..max_steps {
            let mut candidates =
                calc_frenet_paths_for_strategy(state, config, lateral, longitudinal);
            calc_global_paths(&mut candidates, csp);
            let counts = summarize_paths_with_config(&candidates, obstacles, config);
            check_paths_with_config(&mut candidates, obstacles, config);

            let mut fallback = false;
            let path = if let Some(path) = candidates
                .into_iter()
                .min_by(|a, b| a.cf.partial_cmp(&b.cf).unwrap())
            {
                path
            } else {
                let mut path = last_path.clone().expect("expected fallback path");
                path.pop_front();
                fallback = true;
                path
            };

            if path.x.len() <= 1 {
                return (trace, SimulationOutcomeKind::Finish, step);
            }

            last_path = Some(path.clone());
            trace.push(SimulationSnapshot {
                fallback,
                cf: path.cf,
                len: path.x.len(),
                x1: path.x[1],
                y1: path.y[1],
                yaw1: path.yaw[1],
                v1: path.v[1],
                a1: path.a[1],
                c1: path.c[1],
                s1: path.s[1],
                d1: path.d[1],
                s_d1: path.s_d[1],
                s_dd1: path.s_dd[1],
                d_d1: path.d_d[1],
                d_dd1: path.d_dd[1],
                counts,
            });

            state = FrenetInitialState {
                s0: path.s[1],
                speed: path.s_d[1],
                accel: path.s_dd[1],
                d: path.d[1],
                d_d: path.d_d[1],
                d_dd: path.d_dd[1],
            };

            if ((path.x[1] - goal.0).powi(2) + (path.y[1] - goal.1).powi(2)).sqrt() <= 1.0 {
                return (trace, SimulationOutcomeKind::Goal, step);
            }
        }

        panic!("simulation did not terminate within {max_steps} steps");
    }

    fn assert_snapshot_matches(actual: SimulationSnapshot, expected: SimulationSnapshot) {
        assert_eq!(actual.fallback, expected.fallback);
        assert_eq!(actual.len, expected.len);
        assert_eq!(actual.counts, expected.counts);
        assert_close(actual.cf, expected.cf);
        assert_close(actual.x1, expected.x1);
        assert_close(actual.y1, expected.y1);
        assert_close(actual.yaw1, expected.yaw1);
        assert_close(actual.v1, expected.v1);
        assert_close(actual.a1, expected.a1);
        assert_close(actual.c1, expected.c1);
        assert_close(actual.s1, expected.s1);
        assert_close(actual.d1, expected.d1);
        assert_close(actual.s_d1, expected.s_d1);
        assert_close(actual.s_dd1, expected.s_dd1);
        assert_close(actual.d_d1, expected.d_d1);
        assert_close(actual.d_dd1, expected.d_dd1);
    }

    #[test]
    fn test_cubic_spline_2d() {
        let wx = UPSTREAM_WX.to_vec();
        let wy = UPSTREAM_WY.to_vec();
        let csp = CubicSpline2D::new(&wx, &wy);
        assert!(!csp.s.is_empty());

        let (x, y) = csp.calc_position(0.0);
        assert!((x - 0.0).abs() < 1e-6);
        assert!((y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_frenet_path_creation() {
        let fp = FrenetPath::new();
        assert!(fp.t.is_empty());
        assert_eq!(fp.cf, 0.0);
    }

    #[test]
    fn test_calc_frenet_paths_matches_upstream_initial_candidate_grid() {
        let paths = calc_frenet_paths_with_accel(10.0 / 3.6, 0.0, 2.0, 0.0, 0.0, 0.0);
        assert_eq!(paths.len(), 210);

        let reference = [
            (
                0usize,
                20usize,
                3.8000000000000003,
                -6.989576874999997,
                -0.022026714671375677,
                0.030317350631561317,
                18.05758680555556,
                6.9142361111111095,
                0.2968749999999991,
                82.49117269655986,
            ),
            (
                105usize,
                22usize,
                4.2,
                0.0017525504343343101,
                -0.0030929646585680258,
                0.003563824972365069,
                22.780021287252694,
                8.299941564404374,
                0.32870022539443955,
                3.955233569168209,
            ),
            (
                209usize,
                25usize,
                4.800000000000001,
                6.0000000000000036,
                1.8271274675916536e-15,
                7.517396749979645e-17,
                30.000000000000014,
                9.722222222222223,
                0.0,
                44.61225288072495,
            ),
        ];

        for (index, t_len, t_last, d_last, d_d_last, d_dd_last, s_last, s_d_last, s_dd_last, cf) in
            reference
        {
            let path = &paths[index];
            assert_eq!(path.t.len(), t_len);
            assert_close(*path.t.last().unwrap(), t_last);
            assert_close(*path.d.last().unwrap(), d_last);
            assert_close(*path.d_d.last().unwrap(), d_d_last);
            assert_close(*path.d_dd.last().unwrap(), d_dd_last);
            assert_close(*path.s.last().unwrap(), s_last);
            assert_close(*path.s_d.last().unwrap(), s_d_last);
            assert_close(*path.s_dd.last().unwrap(), s_dd_last);
            assert_close(path.cf, cf);
        }
    }

    #[test]
    fn test_frenet_optimal_planning_matches_upstream_initial_window() {
        let csp = CubicSpline2D::new(&UPSTREAM_WX, &UPSTREAM_WY);
        let mut all_paths = calc_frenet_paths_with_accel(10.0 / 3.6, 0.0, 2.0, 0.0, 0.0, 0.0);
        calc_global_paths(&mut all_paths, &csp);
        let counts = summarize_paths(&all_paths, &UPSTREAM_OBSTACLES);
        assert_eq!(
            counts,
            PathCheckCounts {
                max_speed_error: 30,
                max_accel_error: 150,
                max_curvature_error: 0,
                collision_error: 0,
                ok: 30,
            }
        );

        let best = frenet_optimal_planning_with_accel(
            &csp,
            FrenetInitialState {
                s0: 0.0,
                speed: 10.0 / 3.6,
                accel: 0.0,
                d: 2.0,
                d_d: 0.0,
                d_dd: 0.0,
            },
            &UPSTREAM_OBSTACLES,
        )
        .expect("expected a valid path");

        assert_eq!(best.x.len(), 25);
        assert_close(best.cf, 3.542294469271419);

        let reference = [
            (
                0usize,
                1.3527664541170894,
                1.4730997660089002,
                -0.7428410682357911,
                2.777777777777779,
                -0.07124228337505333,
                -1.70188803340216e-16,
                0.0,
                2.0,
                2.7777777777777777,
                0.0,
                0.0,
                0.0,
            ),
            (
                1usize,
                1.8605645611714472,
                1.005284701078841,
                -0.7488798286101788,
                2.787960815155609,
                0.2033476908615227,
                -0.021106708455623728,
                0.557444380144033,
                1.9986421561535495,
                2.80590920781893,
                0.27729552469135804,
                -0.007103103907306435,
                -0.02396404262355845,
            ),
            (
                2usize,
                2.3695265289095406,
                0.5284173023840572,
                -0.7638860397722307,
                2.8502342514931054,
                0.4520558979283277,
                -0.03273816922509124,
                1.125900205761317,
                1.9898244598765433,
                2.8870884773662553,
                0.5304783950617284,
                -0.025264540804632068,
                -0.03816888277632631,
            ),
            (
                12usize,
                8.344424474847328,
                -4.899558751414678,
                -0.45189185643671337,
                4.679227014638239,
                0.1761834255770116,
                0.20268006969898653,
                9.16666666666667,
                1.0000000000000047,
                5.555555555555556,
                1.7361111111111112,
                -0.14062575593750407,
                0.007910283815139495,
            ),
            (
                24usize,
                20.33573711360301,
                4.87317525607841,
                0.6678035273152476,
                8.333333333333332,
                -1.1131604943389124e-14,
                -0.09773284704041481,
                26.66666666666667,
                -1.7763568394002505e-15,
                8.333333333333334,
                -8.881784197001252e-16,
                -1.0658228575266703e-15,
                -5.1159917351284273e-17,
            ),
        ];

        for (index, x, y, yaw, v, a, c, s, d, s_d, s_dd, d_d, d_dd) in reference {
            assert_close(best.x[index], x);
            assert_close(best.y[index], y);
            assert_close(best.yaw[index], yaw);
            assert_close(best.v[index], v);
            assert_close(best.a[index], a);
            assert_close(best.c[index], c);
            assert_close(best.s[index], s);
            assert_close(best.d[index], d);
            assert_close(best.s_d[index], s_d);
            assert_close(best.s_dd[index], s_dd);
            assert_close(best.d_d[index], d_d);
            assert_close(best.d_dd[index], d_dd);
        }
    }

    #[test]
    fn test_calc_low_speed_velocity_candidates_match_upstream_reference() {
        let paths = calc_frenet_paths_for_strategy(
            LOW_SPEED_STATE,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::VelocityKeeping,
        );
        assert_eq!(paths.len(), 100);

        let reference = [
            (
                0usize,
                20usize,
                3.8000000000000003,
                -0.9999999999999947,
                1.4210854715202004e-14,
                7.105427357601002e-15,
                1.8057586805555559,
                0.6914236111111112,
                0.029687499999999978,
                122.17132690502265,
            ),
            (
                50usize,
                22usize,
                4.2,
                -0.9999999999999964,
                0.0,
                -7.105427357601002e-15,
                2.2780021287252694,
                0.8299941564404372,
                0.032870022539444155,
                35.03551917278308,
            ),
            (
                99usize,
                25usize,
                4.800000000000001,
                0.7999999999999992,
                4.440892098500626e-16,
                0.0,
                2.666666666666667,
                0.8333333333333333,
                0.0,
                2.1965106963760093,
            ),
        ];

        for (index, t_len, t_last, d_last, d_d_last, d_dd_last, s_last, s_d_last, s_dd_last, cf) in
            reference
        {
            let path = &paths[index];
            assert_eq!(path.t.len(), t_len);
            assert_close(*path.t.last().unwrap(), t_last);
            assert_close(*path.d.last().unwrap(), d_last);
            assert_close(*path.d_d.last().unwrap(), d_d_last);
            assert_close(*path.d_dd.last().unwrap(), d_dd_last);
            assert_close(*path.s.last().unwrap(), s_last);
            assert_close(*path.s_d.last().unwrap(), s_d_last);
            assert_close(*path.s_dd.last().unwrap(), s_dd_last);
            assert_close(path.cf, cf);
        }
    }

    #[test]
    fn test_low_speed_velocity_planning_matches_upstream_initial_window() {
        let csp = CubicSpline2D::new(&LOW_SPEED_WX, &LOW_SPEED_WY);
        let mut all_paths = calc_frenet_paths_for_strategy(
            LOW_SPEED_STATE,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::VelocityKeeping,
        );
        calc_global_paths(&mut all_paths, &csp);
        let counts = summarize_paths_with_config(
            &all_paths,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
        );
        assert_eq!(
            counts,
            PathCheckCounts {
                max_speed_error: 0,
                max_accel_error: 0,
                max_curvature_error: 57,
                collision_error: 0,
                ok: 43,
            }
        );

        let best = frenet_optimal_planning_for_strategy(
            &csp,
            LOW_SPEED_STATE,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::VelocityKeeping,
        )
        .expect("expected a valid low-speed velocity path");

        assert_eq!(best.x.len(), 25);
        assert_close(best.cf, 1.2030755468845455);

        let reference = [
            (
                0usize,
                0.0837531641911651,
                0.4929355003324144,
                -0.1682997130258217,
                0.2777777777777778,
                -0.00857959813235128,
                0.0,
                0.0,
                0.5,
                0.2777777777777778,
                0.0,
                0.0,
                0.0,
            ),
            (
                1usize,
                0.14080514241144224,
                0.4832403177978342,
                -0.16839917882997923,
                0.2787788814960548,
                0.01878657598085877,
                -0.003816200753394419,
                0.0557444380144033,
                0.499991149250527,
                0.280590920781893,
                0.0277295524691358,
                -0.0004712686472257448,
                -0.016547187583003557,
            ),
            (
                12usize,
                0.9665238390623796,
                0.3501445932331652,
                -0.12806954066719456,
                0.495180889632367,
                0.11811618453721201,
                0.1750548062537276,
                0.9166666666666669,
                0.47744540572166433,
                0.5555555555555555,
                0.1736111111111111,
                -0.05725014209747345,
                -0.05948066711425816,
            ),
            (
                24usize,
                2.402278614362034,
                0.6586442169705189,
                0.5615637714479615,
                0.7923271276537857,
                0.12135132580938711,
                0.12938533923789106,
                2.666666666666667,
                0.39999999999999947,
                0.8333333333333333,
                0.0,
                -2.220446049250313e-16,
                -2.220446049250313e-16,
            ),
        ];

        for (index, x, y, yaw, v, a, c, s, d, s_d, s_dd, d_d, d_dd) in reference {
            assert_close(best.x[index], x);
            assert_close(best.y[index], y);
            assert_close(best.yaw[index], yaw);
            assert_close(best.v[index], v);
            assert_close(best.a[index], a);
            assert_close(best.c[index], c);
            assert_close(best.s[index], s);
            assert_close(best.d[index], d);
            assert_close(best.s_d[index], s_d);
            assert_close(best.s_dd[index], s_dd);
            assert_close(best.d_d[index], d_d);
            assert_close(best.d_dd[index], d_dd);
        }
    }

    #[test]
    fn test_calc_low_speed_stop_candidates_match_upstream_reference() {
        let paths = calc_frenet_paths_for_strategy(
            LOW_SPEED_STATE,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::MergingAndStopping,
        );
        assert_eq!(paths.len(), 350);

        let reference = [
            (
                0usize,
                20usize,
                3.8000000000000003,
                -1.0,
                3.552713678800501e-15,
                7.105427357601002e-15,
                3.0969177986111163,
                0.04507065972221902,
                -0.4276979166666628,
                11.64042021329846,
            ),
            (
                175usize,
                22usize,
                4.2,
                -4.440892098500626e-16,
                0.0,
                -1.7763568394002505e-15,
                3.9969182251791864,
                0.04516803769544708,
                -0.43070595649825094,
                3.583659512530786,
            ),
            (
                349usize,
                25usize,
                4.800000000000001,
                0.7999999999999996,
                0.0,
                2.220446049250313e-16,
                4.899999999999999,
                7.105427357601002e-15,
                0.0,
                5.541471593686472,
            ),
        ];

        for (index, t_len, t_last, d_last, d_d_last, d_dd_last, s_last, s_d_last, s_dd_last, cf) in
            reference
        {
            let path = &paths[index];
            assert_eq!(path.t.len(), t_len);
            assert_close(*path.t.last().unwrap(), t_last);
            assert_close(*path.d.last().unwrap(), d_last);
            assert_close(*path.d_d.last().unwrap(), d_d_last);
            assert_close(*path.d_dd.last().unwrap(), d_dd_last);
            assert_close(*path.s.last().unwrap(), s_last);
            assert_close(*path.s_d.last().unwrap(), s_d_last);
            assert_close(*path.s_dd.last().unwrap(), s_dd_last);
            assert_close(path.cf, cf);
        }
    }

    #[test]
    fn test_low_speed_stop_planning_matches_upstream_initial_window() {
        let csp = CubicSpline2D::new(&LOW_SPEED_WX, &LOW_SPEED_WY);
        let mut all_paths = calc_frenet_paths_for_strategy(
            LOW_SPEED_STATE,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::MergingAndStopping,
        );
        calc_global_paths(&mut all_paths, &csp);
        let counts = summarize_paths_with_config(
            &all_paths,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
        );
        assert_eq!(
            counts,
            PathCheckCounts {
                max_speed_error: 0,
                max_accel_error: 0,
                max_curvature_error: 90,
                collision_error: 179,
                ok: 81,
            }
        );

        let best = frenet_optimal_planning_for_strategy(
            &csp,
            LOW_SPEED_STATE,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::MergingAndStopping,
        )
        .expect("expected a valid low-speed stop path");

        assert_eq!(best.x.len(), 23);
        assert_close(best.cf, 3.278202056506379);

        let reference = [
            (
                0usize,
                0.0837531641911651,
                0.4929355003324144,
                -0.1682997130258217,
                0.2777777777777778,
                -0.00857959813235128,
                0.0,
                0.0,
                0.5,
                0.2777777777777778,
                0.0,
                0.0,
                0.0,
            ),
            (
                1usize,
                0.14290658233093098,
                0.4828665878589613,
                -0.16925077183662696,
                0.30869555293037393,
                0.3016699961830643,
                -0.033086079297872974,
                0.057807503767624294,
                0.4999738741535292,
                0.31077654569441754,
                0.31452125324375435,
                -0.001345121009882328,
                -0.0457988066141507,
            ),
            (
                11usize,
                1.8763619373663174,
                0.09290809871307339,
                -0.05377592062450898,
                1.3921141884482757,
                0.2728327787831233,
                0.5788942882515342,
                1.9106947376794559,
                0.12803574707574247,
                1.3909637817624072,
                0.0037598990606935168,
                -0.3541694804931467,
                0.025694926643622717,
            ),
            (
                22usize,
                3.6101138784518745,
                0.6878357985060032,
                0.42438076874742103,
                0.03273264868551453,
                -0.3131098033086571,
                -0.5079631926311426,
                3.697542577965301,
                -0.20000000000000018,
                0.03605804483142805,
                -0.3446574138968863,
                2.6645352591003757e-15,
                0.0,
            ),
        ];

        for (index, x, y, yaw, v, a, c, s, d, s_d, s_dd, d_d, d_dd) in reference {
            assert_close(best.x[index], x);
            assert_close(best.y[index], y);
            assert_close(best.yaw[index], yaw);
            assert_close(best.v[index], v);
            assert_close(best.a[index], a);
            assert_close(best.c[index], c);
            assert_close(best.s[index], s);
            assert_close(best.d[index], d);
            assert_close(best.s_d[index], s_d);
            assert_close(best.s_dd[index], s_dd);
            assert_close(best.d_d[index], d_d);
            assert_close(best.d_dd[index], d_dd);
        }
    }

    #[test]
    fn test_high_speed_simulation_matches_upstream_main_loop() {
        let csp = CubicSpline2D::new(&UPSTREAM_WX, &UPSTREAM_WY);
        let (trace, outcome, step) = run_simulation_for_strategy(
            &csp,
            FrenetInitialState {
                s0: 0.0,
                speed: 10.0 / 3.6,
                accel: 0.0,
                d: 2.0,
                d_d: 0.0,
                d_dd: 0.0,
            },
            &UPSTREAM_OBSTACLES,
            FrenetPlannerConfig::high_speed(),
            LateralStrategyKind::HighSpeed,
            LongitudinalStrategyKind::VelocityKeeping,
            500,
        );
        assert_eq!(outcome, SimulationOutcomeKind::Goal);
        assert_eq!(step, 56);
        assert_eq!(trace.len(), 57);

        let reference = [
            (
                0usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 3.542294469271419,
                    len: 25,
                    x1: 1.8605645611714472,
                    y1: 1.005284701078841,
                    yaw1: -0.7488798286101788,
                    v1: 2.787960815155609,
                    a1: 0.2033476908615227,
                    c1: -0.021106708455623728,
                    s1: 0.557444380144033,
                    d1: 1.9986421561535495,
                    s_d1: 2.80590920781893,
                    s_dd1: 0.27729552469135804,
                    d_d1: -0.007103103907306435,
                    d_dd1: -0.02396404262355845,
                    counts: PathCheckCounts {
                        max_speed_error: 30,
                        max_accel_error: 150,
                        max_curvature_error: 0,
                        collision_error: 0,
                        ok: 30,
                    },
                },
            ),
            (
                1usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 2.8489891521827695,
                    len: 23,
                    x1: 2.3695265115199833,
                    y1: 0.5284172832738651,
                    yaw1: -0.7638861092260947,
                    v1: 2.8502342570516377,
                    a1: 0.4520559398699433,
                    c1: -0.03273823640059959,
                    s1: 1.125900205761317,
                    d1: 1.9898244340386615,
                    s_d1: 2.887088477366255,
                    s_dd1: 0.5304783950617284,
                    d_d1: -0.02526460939871889,
                    d_dd1: -0.038168947823938225,
                    counts: PathCheckCounts {
                        max_speed_error: 21,
                        max_accel_error: 155,
                        max_curvature_error: 0,
                        collision_error: 0,
                        ok: 34,
                    },
                },
            ),
            (
                2usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 2.323344485225353,
                    len: 23,
                    x1: 2.8841546208047726,
                    y1: 0.033418184614008695,
                    yaw1: -0.7819891459756945,
                    v1: 2.956948411602434,
                    a1: 0.6512431510400924,
                    c1: -0.032212309902136096,
                    s1: 1.715336759017426,
                    d1: 1.9680584361960702,
                    s_d1: 3.0141498321879223,
                    s_dd1: 0.7366617048543571,
                    d_d1: -0.048825425109529065,
                    d_dd1: -0.04062488602072928,
                    counts: PathCheckCounts {
                        max_speed_error: 15,
                        max_accel_error: 157,
                        max_curvature_error: 0,
                        collision_error: 0,
                        ok: 38,
                    },
                },
            ),
            (
                10usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 1.166097731781801,
                    len: 21,
                    x1: 7.548253592986458,
                    y1: -4.294826068828912,
                    yaw1: -0.5856688254921298,
                    v1: 4.189278559604101,
                    a1: 0.21881208796186158,
                    c1: 0.1254113257858499,
                    s1: 7.876732837857354,
                    d1: 1.2776621126305698,
                    s_d1: 4.8229460558777975,
                    s_dd1: 1.286834078533147,
                    d_d1: -0.1255759074190509,
                    d_dd1: 0.004692247248534772,
                    counts: PathCheckCounts {
                        max_speed_error: 2,
                        max_accel_error: 88,
                        max_curvature_error: 0,
                        collision_error: 69,
                        ok: 51,
                    },
                },
            ),
            (
                20usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 17.5303184314179,
                    len: 23,
                    x1: 14.6771670455895,
                    y1: -0.5409502249017437,
                    yaw1: 1.0432729339961222,
                    v1: 6.751977179466637,
                    a1: 1.1716781306769781,
                    c1: 0.022496673620090123,
                    s1: 19.67714300233158,
                    d1: 0.855537905656363,
                    s_d1: 6.762010642962067,
                    s_dd1: 0.7188928361205661,
                    d_d1: 0.0927110353342166,
                    d_dd1: 0.016517891487136914,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 3,
                        max_curvature_error: 0,
                        collision_error: 150,
                        ok: 57,
                    },
                },
            ),
            (
                56usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 0.8381388542917644,
                    len: 2,
                    x1: 70.17562512818523,
                    y1: 0.3562246796885216,
                    yaw1: -0.25232078404059566,
                    v1: 8.365218464633804,
                    a1: -0.02230330452044404,
                    c1: 0.005745317758332535,
                    s1: 77.13369022391397,
                    d1: 0.28528511463172584,
                    s_d1: 8.352995857586786,
                    s_dd1: 0.0014166156483617837,
                    d_d1: -0.05393428352839567,
                    d_dd1: 0.005806714817959832,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 0,
                        collision_error: 0,
                        ok: 210,
                    },
                },
            ),
        ];

        for (index, expected) in reference {
            assert_snapshot_matches(trace[index], expected);
        }
    }

    #[test]
    fn test_low_speed_velocity_simulation_matches_upstream_reference() {
        let csp = CubicSpline2D::new(&LOW_SPEED_WX, &LOW_SPEED_WY);
        let (trace, outcome, step) = run_simulation_for_strategy(
            &csp,
            LOW_SPEED_STATE,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::VelocityKeeping,
            500,
        );
        assert_eq!(outcome, SimulationOutcomeKind::Goal);
        assert_eq!(step, 72);
        assert_eq!(trace.len(), 73);

        let reference = [
            (
                0usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 1.2030755468845455,
                    len: 25,
                    x1: 0.14080514241144224,
                    y1: 0.4832403177978342,
                    yaw1: -0.16839917882997923,
                    v1: 0.2787788814960548,
                    a1: 0.01878657598085877,
                    c1: -0.003816200753394419,
                    s1: 0.0557444380144033,
                    d1: 0.499991149250527,
                    s_d1: 0.280590920781893,
                    s_dd1: 0.0277295524691358,
                    d_d1: -0.0004712686472257448,
                    d_dd1: -0.016547187583003557,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 57,
                        collision_error: 0,
                        ok: 43,
                    },
                },
            ),
            (
                1usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 1.1593824675794724,
                    len: 21,
                    x1: 0.19862397869158743,
                    y1: 0.4734005087839725,
                    yaw1: -0.1689488569133979,
                    v1: 0.2854845708264037,
                    a1: 0.04834696890578991,
                    c1: -0.016702081709852195,
                    s1: 0.1126273382131937,
                    d1: 0.4999233647275333,
                    s_d1: 0.2892620076314542,
                    s_dd1: 0.058447345232429636,
                    d_d1: -0.002151504794264144,
                    d_dd1: -0.0418106087316688,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 52,
                        collision_error: 0,
                        ok: 48,
                    },
                },
            ),
            (
                2usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 1.09349999893079,
                    len: 20,
                    x1: 0.2583590326193169,
                    y1: 0.46318616576505733,
                    yaw1: -0.17013710434144036,
                    v1: 0.2976978528813822,
                    a1: 0.0739238866402707,
                    c1: -0.02614265471946679,
                    s1: 0.1718374528054004,
                    d1: 0.4997092754709174,
                    s_d1: 0.30375627055724636,
                    s_dd1: 0.08596131323144582,
                    d_d1: -0.005295480363446369,
                    d_dd1: -0.06364996071308254,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 44,
                        collision_error: 3,
                        ok: 53,
                    },
                },
            ),
            (
                10usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 1.5495892077371207,
                    len: 23,
                    x1: 0.8720531487807729,
                    y1: 0.35911979640746683,
                    yaw1: -0.16214830237709377,
                    v1: 0.45631948013996126,
                    a1: 0.08683536859186874,
                    c1: -0.04279669202248163,
                    s1: 0.8123519120139755,
                    d1: 0.47926480753857237,
                    s_d1: 0.5037646387918939,
                    s_dd1: 0.12428623137720253,
                    d_d1: -0.06826002731440607,
                    d_dd1: -0.20972236566824146,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 18,
                        collision_error: 40,
                        ok: 42,
                    },
                },
            ),
            (
                72usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 0.8058978467091669,
                    len: 7,
                    x1: 9.125666057110516,
                    y1: -1.5235516112544554,
                    yaw1: -0.5413983263388418,
                    v1: 0.8329541837630935,
                    a1: -0.0025774853620610617,
                    c1: 0.08692867844324033,
                    s1: 9.956315978981115,
                    d1: 0.021489724603800633,
                    s_d1: 0.8305162740025163,
                    s_dd1: 0.003961111852914451,
                    d_d1: -0.07081502791017966,
                    d_dd1: 0.10801481070105634,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 0,
                        collision_error: 0,
                        ok: 100,
                    },
                },
            ),
        ];

        for (index, expected) in reference {
            assert_snapshot_matches(trace[index], expected);
        }
    }

    #[test]
    fn test_low_speed_stop_simulation_matches_upstream_reference() {
        let csp = CubicSpline2D::new(&LOW_SPEED_WX, &LOW_SPEED_WY);
        let (trace, outcome, step) = run_simulation_for_strategy(
            &csp,
            LOW_SPEED_STATE,
            &LOW_SPEED_OBSTACLES,
            FrenetPlannerConfig::low_speed(),
            LateralStrategyKind::LowSpeed,
            LongitudinalStrategyKind::MergingAndStopping,
            500,
        );
        assert_eq!(outcome, SimulationOutcomeKind::Finish);
        assert_eq!(step, 28);
        assert_eq!(trace.len(), 28);

        let reference = [
            (
                0usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 3.278202056506379,
                    len: 23,
                    x1: 0.14290658233093098,
                    y1: 0.4828665878589613,
                    yaw1: -0.16925077183662696,
                    v1: 0.30869555293037393,
                    a1: 0.3016699961830643,
                    c1: -0.033086079297872974,
                    s1: 0.057807503767624294,
                    d1: 0.4999738741535292,
                    s_d1: 0.31077654569441754,
                    s_dd1: 0.31452125324375435,
                    d_d1: -0.001345121009882328,
                    d_dd1: -0.0457988066141507,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 90,
                        collision_error: 179,
                        ok: 81,
                    },
                },
            ),
            (
                1usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 2.5409670415058536,
                    len: 23,
                    x1: 0.21398675876644052,
                    y1: 0.47062703598230726,
                    yaw1: -0.17233523193211964,
                    v1: 0.3908914702453267,
                    a1: 0.507139996062664,
                    c1: -0.05720254792836169,
                    s1: 0.1278355663792582,
                    d1: 0.49973517177727805,
                    s_d1: 0.3967668021582189,
                    s_dd1: 0.5325274854225438,
                    d_d1: -0.00591896882202722,
                    d_dd1: -0.08407951439775528,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 92,
                        collision_error: 178,
                        ok: 80,
                    },
                },
            ),
            (
                2usize,
                SimulationSnapshot {
                    fallback: false,
                    cf: 2.081139888747688,
                    len: 23,
                    x1: 0.3051413175662653,
                    y1: 0.4545276729333922,
                    yaw1: -0.1784908329442092,
                    v1: 0.5018045282261229,
                    a1: 0.5936073747094394,
                    c1: -0.08344253603936327,
                    s1: 0.21863965454953663,
                    d1: 0.4987893954606454,
                    s_d1: 0.5148189105129078,
                    s_dd1: 0.6389665942262637,
                    d_d1: -0.015563825269794466,
                    d_dd1: -0.127144484147799,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 91,
                        collision_error: 179,
                        ok: 80,
                    },
                },
            ),
            (
                10usize,
                SimulationSnapshot {
                    fallback: true,
                    cf: 1.4304565347697635,
                    len: 19,
                    x1: 1.6926439871748995,
                    y1: 0.15261800304196171,
                    yaw1: -0.11885428163574252,
                    v1: 1.1361148263712337,
                    a1: 0.20978218334193272,
                    c1: 0.27909204598500814,
                    s1: 1.7325635495402325,
                    d1: 0.23379710386591446,
                    s_d1: 1.2016568272191095,
                    s_dd1: 0.043622677081437716,
                    d_d1: -0.3162049915823444,
                    d_dd1: -0.08257892976684256,
                    counts: PathCheckCounts {
                        max_speed_error: 0,
                        max_accel_error: 0,
                        max_curvature_error: 14,
                        collision_error: 21,
                        ok: 0,
                    },
                },
            ),
            (
                27usize,
                SimulationSnapshot {
                    fallback: true,
                    cf: 1.4304565347697635,
                    len: 2,
                    x1: 3.8421875321885457,
                    y1: 0.7718572517229931,
                    yaw1: 0.2541519104288392,
                    v1: 0.01058009224298756,
                    a1: -0.103322941903366,
                    c1: 0.27909204598500814,
                    s1: 3.999157563333649,
                    d1: -0.19999999999999984,
                    s_d1: 0.01248764848273487,
                    s_dd1: -0.12190755851008128,
                    d_d1: 6.661338147750939e-16,
                    d_dd1: 2.220446049250313e-16,
                    counts: PathCheckCounts {
                        max_speed_error: 5,
                        max_accel_error: 0,
                        max_curvature_error: 30,
                        collision_error: 0,
                        ok: 0,
                    },
                },
            ),
        ];

        for (index, expected) in reference {
            assert_snapshot_matches(trace[index], expected);
        }
    }
}
