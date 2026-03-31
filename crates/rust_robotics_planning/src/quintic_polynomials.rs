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
            while current_t < t + dt - 1e-12 {
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
#[allow(clippy::excessive_precision)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn scalar_fingerprint(values: &[f64]) -> (f64, f64) {
        let sum: f64 = values.iter().sum();
        let weighted_sum: f64 = values
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        (sum, weighted_sum)
    }

    #[test]
    fn test_quintic_polynomial() {
        let qp = QuinticPolynomial::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0);
        assert!((qp.calc_point(0.0) - 0.0).abs() < 1e-10);
        assert!((qp.calc_point(10.0) - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_quintic_polynomial_matches_upstream_simple_reference() {
        let qp = QuinticPolynomial::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0);

        assert!(approx_eq(qp.a0, 0.0, 1e-12));
        assert!(approx_eq(qp.a1, 0.0, 1e-12));
        assert!(approx_eq(qp.a2, 0.0, 1e-12));
        assert!(approx_eq(qp.a3, 0.099_999_999_999_999_99, 1e-12));
        assert!(approx_eq(qp.a4, -0.014_999_999_999_999_996, 1e-12));
        assert!(approx_eq(qp.a5, 0.000_599_999_999_999_999_8, 1e-12));

        let expected_states = [
            (0.0, 0.0, 0.0, 0.0, 0.6),
            (
                5.0,
                5.0,
                1.875_000_000_000_001_3,
                1.332_267_629_550_187_8e-15,
                -0.299_999_999_999_999_5,
            ),
            (
                10.0,
                10.0,
                7.105_427_357_601_002e-15,
                3.552_713_678_800_501e-15,
                0.600_000_000_000_000_5,
            ),
        ];

        for (t, point, velocity, accel, jerk) in expected_states {
            assert!(approx_eq(qp.calc_point(t), point, 1e-12));
            assert!(approx_eq(qp.calc_first_derivative(t), velocity, 1e-12));
            assert!(approx_eq(qp.calc_second_derivative(t), accel, 1e-12));
            assert!(approx_eq(qp.calc_third_derivative(t), jerk, 1e-12));
        }
    }

    #[test]
    fn test_quintic_polynomials_planner() {
        let mut planner = QuinticPolynomialsPlanner::new();
        let result = planner.planning(
            10.0,
            10.0,
            10.0_f64.to_radians(),
            1.0,
            0.1,
            30.0,
            -10.0,
            20.0_f64.to_radians(),
            1.0,
            0.1,
            1.0,
            0.5,
            0.1,
        );
        assert!(result);
        assert!(!planner.rx.is_empty());
    }

    #[test]
    fn test_quintic_polynomials_planner_matches_upstream_main_example() {
        let mut planner = QuinticPolynomialsPlanner::new();
        let result = planner.planning(
            10.0,
            10.0,
            10.0_f64.to_radians(),
            1.0,
            0.1,
            30.0,
            -10.0,
            20.0_f64.to_radians(),
            1.0,
            0.1,
            1.0,
            0.5,
            0.1,
        );

        assert!(result);
        assert_eq!(planner.time.len(), 151);
        assert_eq!(planner.rx.len(), 151);
        assert_eq!(planner.ry.len(), 151);
        assert_eq!(planner.ryaw.len(), 151);
        assert_eq!(planner.rv.len(), 151);
        assert_eq!(planner.ra.len(), 151);
        assert_eq!(planner.rj.len(), 151);
        assert!(approx_eq(*planner.time.last().unwrap(), 15.0, 1e-12));

        let expected_samples = [
            (
                0usize,
                0.0,
                10.0,
                10.0,
                10.0_f64.to_radians(),
                1.0,
                0.099_999_999_999_999_99,
                0.427_280_794_153_487_6,
            ),
            (
                1,
                0.1,
                10.098_982_615_546_902,
                10.017_381_774_411_378,
                0.172_447_377_225_515_92,
                1.009_916_853_081_542_3,
                0.106_821_796_693_630_91,
                0.410_270_512_683_564_35,
            ),
            (
                10,
                1.0,
                11.042_264_078_940_175,
                10.118_588_516_101_896,
                0.005_806_154_972_050_835,
                1.106_637_942_606_725_2,
                0.354_433_544_260_436_83,
                0.267_611_399_963_759_2,
            ),
            (
                50,
                5.0,
                16.610_137_614_251_27,
                6.064_611_076_798_1,
                -0.912_947_184_636_906_5,
                2.664_216_406_735_696_4,
                0.467_382_916_097_198_84,
                -0.146_179_174_681_691_67,
            ),
            (
                150,
                15.0,
                30.000_000_000_000_007,
                -10.0,
                20.0_f64.to_radians(),
                1.000_000_000_000_003,
                0.099_999_999_999_999_62,
                -0.433_897_236_756_954_63,
            ),
        ];

        for (index, time, rx, ry, yaw, v, a, j) in expected_samples {
            assert!(approx_eq(planner.time[index], time, 1e-12));
            assert!(approx_eq(planner.rx[index], rx, 1e-12));
            assert!(approx_eq(planner.ry[index], ry, 1e-12));
            assert!(approx_eq(planner.ryaw[index], yaw, 1e-12));
            assert!(approx_eq(planner.rv[index], v, 1e-12));
            assert!(approx_eq(planner.ra[index], a, 1e-12));
            assert!(approx_eq(planner.rj[index], j, 1e-12));
        }

        let time_fp = scalar_fingerprint(&planner.time);
        assert!(approx_eq(time_fp.0, 1_132.500_000_000_000_2, 1e-9));
        assert!(approx_eq(time_fp.1, 114_759.999_999_999_97, 1e-6));

        let rx_fp = scalar_fingerprint(&planner.rx);
        assert!(approx_eq(rx_fp.0, 3_084.277_101_694_296, 1e-9));
        assert!(approx_eq(rx_fp.1, 275_626.252_367_386_25, 1e-6));

        let ry_fp = scalar_fingerprint(&planner.ry);
        assert!(approx_eq(ry_fp.0, -23.379_117_661_761_498, 1e-9));
        assert!(approx_eq(ry_fp.1, -52_763.587_503_238_03, 1e-6));

        let yaw_fp = scalar_fingerprint(&planner.ryaw);
        assert!(approx_eq(yaw_fp.0, -91.931_046_512_332_49, 1e-9));
        assert!(approx_eq(yaw_fp.1, -6_931.617_159_195_794, 1e-6));

        let velocity_fp = scalar_fingerprint(&planner.rv);
        assert!(approx_eq(velocity_fp.0, 302.345_686_787_654_7, 1e-9));
        assert!(approx_eq(velocity_fp.1, 22_418.061_584_864_976, 1e-6));

        let accel_fp = scalar_fingerprint(&planner.ra);
        assert!(approx_eq(accel_fp.0, 3.130_947_174_813_961, 1e-9));
        assert!(approx_eq(accel_fp.1, -1_921.398_399_412_191_8, 1e-6));

        let jerk_fp = scalar_fingerprint(&planner.rj);
        assert!(approx_eq(jerk_fp.0, -6.835_206_957_044_357, 1e-9));
        assert!(approx_eq(jerk_fp.1, -1_017.425_338_206_766_7, 1e-6));
    }
}
