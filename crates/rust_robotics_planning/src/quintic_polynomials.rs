#![allow(
    dead_code,
    clippy::too_many_arguments,
    clippy::needless_borrows_for_generic_args
)]

//! Quintic Polynomials path planning
//!
//! Generates smooth trajectories using quintic polynomial curves
//! that satisfy position, velocity, and acceleration boundary conditions.

// Parameters
const MAX_T: f64 = 100.0; // maximum time to the goal [s]
const MIN_T: f64 = 5.0; // minimum time to the goal [s]

#[derive(Debug, Clone)]
pub struct QuinticPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub a4: f64,
    pub a5: f64,
}

impl QuinticPolynomial {
    pub fn new(xs: f64, vxs: f64, axs: f64, xe: f64, vxe: f64, axe: f64, time: f64) -> Self {
        let a0 = xs;
        let a1 = vxs;
        let a2 = axs / 2.0;

        let t2 = time * time;
        let t3 = t2 * time;
        let t4 = t3 * time;
        let t5 = t4 * time;

        let a_matrix = [
            [t3, t4, t5],
            [3.0 * t2, 4.0 * t3, 5.0 * t4],
            [6.0 * time, 12.0 * t2, 20.0 * t3],
        ];

        let b = [
            xe - a0 - a1 * time - a2 * t2,
            vxe - a1 - 2.0 * a2 * time,
            axe - 2.0 * a2,
        ];

        let det_a = Self::determinant_3x3(&a_matrix);

        let mut a3_matrix = a_matrix;
        a3_matrix[0][0] = b[0];
        a3_matrix[1][0] = b[1];
        a3_matrix[2][0] = b[2];
        let a3 = Self::determinant_3x3(&a3_matrix) / det_a;

        let mut a4_matrix = a_matrix;
        a4_matrix[0][1] = b[0];
        a4_matrix[1][1] = b[1];
        a4_matrix[2][1] = b[2];
        let a4 = Self::determinant_3x3(&a4_matrix) / det_a;

        let mut a5_matrix = a_matrix;
        a5_matrix[0][2] = b[0];
        a5_matrix[1][2] = b[1];
        a5_matrix[2][2] = b[2];
        let a5 = Self::determinant_3x3(&a5_matrix) / det_a;

        QuinticPolynomial {
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
        }
    }

    fn determinant_3x3(matrix: &[[f64; 3]; 3]) -> f64 {
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    }

    pub fn calc_point(&self, t: f64) -> f64 {
        self.a0
            + self.a1 * t
            + self.a2 * t.powi(2)
            + self.a3 * t.powi(3)
            + self.a4 * t.powi(4)
            + self.a5 * t.powi(5)
    }

    pub fn calc_first_derivative(&self, t: f64) -> f64 {
        self.a1
            + 2.0 * self.a2 * t
            + 3.0 * self.a3 * t.powi(2)
            + 4.0 * self.a4 * t.powi(3)
            + 5.0 * self.a5 * t.powi(4)
    }

    pub fn calc_second_derivative(&self, t: f64) -> f64 {
        2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t.powi(2) + 20.0 * self.a5 * t.powi(3)
    }

    pub fn calc_third_derivative(&self, t: f64) -> f64 {
        6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t.powi(2)
    }
}

#[derive(Debug)]
pub struct QuinticPolynomialsPlanner {
    pub time: Vec<f64>,
    pub rx: Vec<f64>,
    pub ry: Vec<f64>,
    pub ryaw: Vec<f64>,
    pub rv: Vec<f64>,
    pub ra: Vec<f64>,
    pub rj: Vec<f64>,
}

impl QuinticPolynomialsPlanner {
    pub fn new() -> Self {
        QuinticPolynomialsPlanner {
            time: Vec::new(),
            rx: Vec::new(),
            ry: Vec::new(),
            ryaw: Vec::new(),
            rv: Vec::new(),
            ra: Vec::new(),
            rj: Vec::new(),
        }
    }

    pub fn planning(
        &mut self,
        sx: f64,
        sy: f64,
        syaw: f64,
        sv: f64,
        sa: f64,
        gx: f64,
        gy: f64,
        gyaw: f64,
        gv: f64,
        ga: f64,
        max_accel: f64,
        max_jerk: f64,
        dt: f64,
    ) -> bool {
        let vxs = sv * syaw.cos();
        let vys = sv * syaw.sin();
        let vxg = gv * gyaw.cos();
        let vyg = gv * gyaw.sin();

        let axs = sa * syaw.cos();
        let ays = sa * syaw.sin();
        let axg = ga * gyaw.cos();
        let ayg = ga * gyaw.sin();

        let mut t = MIN_T;
        while t < MAX_T {
            let xqp = QuinticPolynomial::new(sx, vxs, axs, gx, vxg, axg, t);
            let yqp = QuinticPolynomial::new(sy, vys, ays, gy, vyg, ayg, t);

            self.time.clear();
            self.rx.clear();
            self.ry.clear();
            self.ryaw.clear();
            self.rv.clear();
            self.ra.clear();
            self.rj.clear();

            let mut current_t = 0.0;
            while current_t <= t + dt {
                self.time.push(current_t);
                self.rx.push(xqp.calc_point(current_t));
                self.ry.push(yqp.calc_point(current_t));

                let vx = xqp.calc_first_derivative(current_t);
                let vy = yqp.calc_first_derivative(current_t);
                let v = (vx * vx + vy * vy).sqrt();
                let yaw = vy.atan2(vx);
                self.rv.push(v);
                self.ryaw.push(yaw);

                let ax = xqp.calc_second_derivative(current_t);
                let ay = yqp.calc_second_derivative(current_t);
                let mut a = (ax * ax + ay * ay).sqrt();
                if self.rv.len() >= 2
                    && self.rv[self.rv.len() - 1] - self.rv[self.rv.len() - 2] < 0.0
                {
                    a *= -1.0;
                }
                self.ra.push(a);

                let jx = xqp.calc_third_derivative(current_t);
                let jy = yqp.calc_third_derivative(current_t);
                let mut j = (jx * jx + jy * jy).sqrt();
                if self.ra.len() >= 2
                    && self.ra[self.ra.len() - 1] - self.ra[self.ra.len() - 2] < 0.0
                {
                    j *= -1.0;
                }
                self.rj.push(j);

                current_t += dt;
            }

            let max_a = self.ra.iter().map(|x| x.abs()).fold(0.0, f64::max);
            let max_j = self.rj.iter().map(|x| x.abs()).fold(0.0, f64::max);

            if max_a <= max_accel && max_j <= max_jerk {
                return true;
            }

            t += MIN_T;
        }

        false
    }
}

impl Default for QuinticPolynomialsPlanner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quintic_polynomial() {
        let qp = QuinticPolynomial::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0);
        assert!((qp.calc_point(0.0) - 0.0).abs() < 1e-10);
        assert!((qp.calc_point(10.0) - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_quintic_polynomials_planner() {
        let mut planner = QuinticPolynomialsPlanner::new();
        let result = planner.planning(
            10.0, 10.0,
            10.0_f64.to_radians(),
            1.0, 0.1,
            30.0, -10.0,
            20.0_f64.to_radians(),
            1.0, 0.1,
            1.0, 0.5, 0.1,
        );
        assert!(result);
        assert!(!planner.rx.is_empty());
    }
}
