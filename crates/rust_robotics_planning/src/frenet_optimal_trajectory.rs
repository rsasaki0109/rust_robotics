#![allow(dead_code)]

//! Frenet Optimal Trajectory planning
//!
//! Generates optimal trajectories in the Frenet frame, considering
//! lateral and longitudinal motion with jerk minimization.

// Parameters
const MAX_SPEED: f64 = 50.0 / 3.6;
const MAX_ACCEL: f64 = 2.0;
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
const K_D: f64 = 1.0;
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
            ds: Vec::new(),
            c: Vec::new(),
        }
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
}

/// Generate Frenet candidate paths
pub fn calc_frenet_paths(
    c_speed: f64,
    c_d: f64,
    c_d_d: f64,
    c_d_dd: f64,
    s0: f64,
) -> Vec<FrenetPath> {
    let mut fplist = Vec::new();

    let mut di = -MAX_ROAD_WIDTH;
    while di <= MAX_ROAD_WIDTH {
        let mut ti = MIN_T;
        while ti <= MAX_T {
            let mut fp = FrenetPath::new();

            let lat_qp = QuinticPolynomial::new(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);

            let mut t = 0.0;
            while t <= ti {
                fp.t.push(t);
                fp.d.push(lat_qp.calc_point(t));
                fp.d_d.push(lat_qp.calc_first_derivative(t));
                fp.d_dd.push(lat_qp.calc_second_derivative(t));
                fp.d_ddd.push(lat_qp.calc_third_derivative(t));
                t += DT;
            }

            for tv in [
                TARGET_SPEED - D_T_S * N_S_SAMPLE as f64,
                TARGET_SPEED,
                TARGET_SPEED + D_T_S * N_S_SAMPLE as f64,
            ] {
                let mut tfp = fp.clone();

                let lon_qp = QuarticPolynomial::new(s0, c_speed, 0.0, tv, 0.0, ti);

                for &t in &tfp.t {
                    tfp.s.push(lon_qp.calc_point(t));
                    tfp.s_d.push(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push(lon_qp.calc_third_derivative(t));
                }

                let jp: f64 = tfp.d_ddd.iter().map(|x| x.powi(2)).sum();
                let js: f64 = tfp.s_ddd.iter().map(|x| x.powi(2)).sum();

                let ds = (TARGET_SPEED - tfp.s_d.last().unwrap_or(&0.0)).powi(2);
                tfp.cd = K_J * jp + K_T * ti + K_D * tfp.d.last().unwrap_or(&0.0).powi(2);
                tfp.cv = K_J * js + K_T * ti + K_D * ds;
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv;

                fplist.push(tfp);
            }

            ti += DT;
        }

        di += D_ROAD_W;
    }

    fplist
}

/// Calculate global positions from Frenet paths
pub fn calc_global_paths(fplist: &mut [FrenetPath], csp: &CubicSpline2D) {
    for fp in fplist.iter_mut() {
        for i in 0..fp.s.len() {
            let s = fp.s[i];
            if s > *csp.s.last().unwrap_or(&0.0) {
                break;
            }

            let (ix, iy) = csp.calc_position(s);
            let iyaw = csp.calc_yaw(s);
            let d = fp.d[i];

            let x = ix - d * iyaw.sin();
            let y = iy + d * iyaw.cos();

            fp.x.push(x);
            fp.y.push(y);
        }

        for i in 0..fp.x.len().saturating_sub(1) {
            let dx = fp.x[i + 1] - fp.x[i];
            let dy = fp.y[i + 1] - fp.y[i];
            fp.yaw.push(dy.atan2(dx));
            fp.ds.push((dx.powi(2) + dy.powi(2)).sqrt());
        }

        if !fp.yaw.is_empty() {
            fp.yaw.push(*fp.yaw.last().unwrap());
            fp.ds.push(*fp.ds.last().unwrap());
        }

        for i in 0..fp.yaw.len().saturating_sub(1) {
            let dyaw = fp.yaw[i + 1] - fp.yaw[i];
            if fp.ds[i] > 0.0 {
                fp.c.push(dyaw / fp.ds[i]);
            } else {
                fp.c.push(0.0);
            }
        }
    }
}

/// Check constraints and collisions
pub fn check_paths(fplist: &mut Vec<FrenetPath>, ob: &[(f64, f64)]) {
    fplist.retain(|fp| {
        if fp.s_d.iter().any(|&v| v > MAX_SPEED) {
            return false;
        }

        if fp.s_dd.iter().any(|&a| a.abs() > MAX_ACCEL) {
            return false;
        }

        if fp.c.iter().any(|&c| c.abs() > MAX_CURVATURE) {
            return false;
        }

        for (ox, oy) in ob {
            for (&x, &y) in fp.x.iter().zip(fp.y.iter()) {
                let d = ((x - ox).powi(2) + (y - oy).powi(2)).sqrt();
                if d <= ROBOT_RADIUS {
                    return false;
                }
            }
        }

        true
    });
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
    let mut fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(&mut fplist, csp);
    check_paths(&mut fplist, ob);

    fplist
        .into_iter()
        .min_by(|a, b| a.cf.partial_cmp(&b.cf).unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cubic_spline_2d() {
        let wx = vec![0.0, 10.0, 20.5, 35.0, 70.5];
        let wy = vec![0.0, -6.0, 5.0, 6.5, 0.0];
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
}
