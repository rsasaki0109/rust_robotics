#![allow(
    dead_code,
    clippy::needless_borrows_for_generic_args,
    clippy::new_without_default,
    clippy::ptr_arg,
    clippy::type_complexity
)]

//! Cubic spline planner
//!
//! Path planner using cubic spline interpolation through waypoints.
//!
//! Reference:
//! - PythonRobotics CubicSpline by Atsushi Sakai
//! - CppRobotics cubic_spline by TAI Lei

#[derive(Debug, Clone)]
struct Spline {
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
    x: Vec<f64>,
    y: Vec<f64>,
}

impl Spline {
    fn new(x: &Vec<f64>, y: &Vec<f64>) -> Spline {
        let nx = x.len();
        let mut b: Vec<f64> = Vec::with_capacity(nx);
        let mut d: Vec<f64> = Vec::with_capacity(nx);
        let mut h: Vec<f64> = Vec::with_capacity(nx - 1);
        for i in 0..nx - 1 {
            h.push(x[i + 1] - x[i]);
        }
        let a = y.clone();
        let a_mat = Spline::__calc_a(&h);
        let b_mat = Spline::__calc_b(&h, &a);

        let a_mat_inv = a_mat.try_inverse().unwrap();

        let c_na = a_mat_inv * b_mat;
        let mut c: Vec<f64> = Vec::with_capacity(c_na.len());
        for i in 0..c_na.len() {
            c.push(c_na[i]);
        }
        for i in 0..nx - 1 {
            d.push((c[i + 1] - c[i]) / (3. * h[i]));
            let tb = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
            b.push(tb);
        }

        Spline {
            a,
            b,
            c,
            d,
            x: x.to_vec(),
            y: y.to_vec(),
        }
    }

    fn calc(&self, t: f64) -> f64 {
        let i = self.__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        self.a[i] + self.b[i] * dx + self.c[i] * dx.powi(2) + self.d[i] * dx.powi(3)
    }

    fn calcd(&self, t: f64) -> f64 {
        let i = self.__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        let b = self.b[i];
        let c = self.c[i];
        let d = self.d[i];
        b + 2. * c * dx + 3. * d * dx.powi(2)
    }

    fn calcdd(&self, t: f64) -> f64 {
        let i = self.__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        2. * self.c[i] + 6. * self.d[i] * dx
    }

    fn __search_index(&self, t: f64) -> usize {
        let nx = self.x.len();
        self.bisect(t, 0, nx)
    }

    fn __calc_a(h: &Vec<f64>) -> nalgebra::DMatrix<f64> {
        let nx = h.len() + 1;
        let mut a = nalgebra::DMatrix::from_diagonal_element(nx, nx, 0.0);
        a[(0, 0)] = 1.;
        for i in 0..nx - 1 {
            if i != nx - 2 {
                a[(i + 1, i + 1)] = 2.0 * (h[i] + h[i + 1]);
            }
            a[(i + 1, i)] = h[i];
            a[(i, i + 1)] = h[i];
        }
        a[(0, 1)] = 0.;
        a[(nx - 1, nx - 2)] = 0.;
        a[(nx - 1, nx - 1)] = 1.;
        a
    }

    fn __calc_b(h: &Vec<f64>, a: &Vec<f64>) -> nalgebra::DVector<f64> {
        let nx = h.len() + 1;
        let mut b = nalgebra::DVector::zeros(nx);
        for i in 0..nx - 2 {
            b[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
        }
        b
    }

    fn bisect(&self, t: f64, s: usize, e: usize) -> usize {
        let mid = (s + e) / 2;
        if t == self.x[mid] || e - s <= 1 {
            mid
        } else if t > self.x[mid] {
            self.bisect(t, mid, e)
        } else {
            self.bisect(t, s, mid)
        }
    }
}

#[derive(Debug, Clone)]
pub struct Spline2D {
    pub s: Vec<f64>,
    sx: Spline,
    sy: Spline,
}

impl Spline2D {
    pub fn new(x: Vec<f64>, y: Vec<f64>) -> Spline2D {
        let s = Spline2D::__calc_s(&x, &y);
        let sx = Spline::new(&s, &x);
        let sy = Spline::new(&s, &y);

        Spline2D { s, sx, sy }
    }

    fn __calc_s(x: &Vec<f64>, y: &Vec<f64>) -> Vec<f64> {
        let nx = x.len();
        let mut dx: Vec<f64> = Vec::with_capacity(nx - 1);
        let mut dy: Vec<f64> = Vec::with_capacity(nx - 1);

        for i in 0..nx - 1 {
            dx.push(x[i + 1] - x[i]);
            dy.push(y[i + 1] - y[i]);
        }
        let mut ds: Vec<f64> = Vec::with_capacity(nx - 1);
        for i in 0..nx - 1 {
            let dsi = (dx[i].powi(2) + dy[i].powi(2)).sqrt();
            ds.push(dsi);
        }
        let mut s: Vec<f64> = Vec::with_capacity(nx);
        s.push(0.);
        for i in 0..nx - 1 {
            s.push(s[i] + ds[i]);
        }
        s
    }

    pub fn calc_position(&self, is: f64) -> (f64, f64) {
        let x = self.sx.calc(is);
        let y = self.sy.calc(is);
        (x, y)
    }

    pub fn calc_curvature(&self, is: f64) -> f64 {
        let dx = self.sx.calcd(is);
        let ddx = self.sx.calcdd(is);
        let dy = self.sy.calcd(is);
        let ddy = self.sy.calcdd(is);
        (ddy * dx - ddx * dy) / ((dx.powi(2) + dy.powi(2)).powf(3. / 2.))
    }

    pub fn calc_yaw(&self, is: f64) -> f64 {
        let dx = self.sx.calcd(is);
        let dy = self.sy.calcd(is);
        dy.atan2(dx)
    }
}

pub fn calc_spline_course(
    x: Vec<f64>,
    y: Vec<f64>,
    ds: f64,
) -> (Vec<(f64, f64)>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let sp = Spline2D::new(x, y);
    let s_end = sp.s[sp.s.len() - 1];
    let mut r: Vec<(f64, f64)> = Vec::new();
    let mut ryaw: Vec<f64> = Vec::new();
    let mut rk: Vec<f64> = Vec::new();
    let mut s: Vec<f64> = Vec::new();

    let mut is = 0.0;
    while is < s_end {
        let pair = sp.clone().calc_position(is);
        r.push(pair);
        ryaw.push(sp.clone().calc_yaw(is));
        rk.push(sp.clone().calc_curvature(is));
        s.push(is);
        is += ds;
    }
    (r, ryaw, rk, s)
}

pub struct CubicSplinePlanner {
    pub path: Vec<(f64, f64)>,
    pub yaw: Vec<f64>,
    pub curvature: Vec<f64>,
    pub s: Vec<f64>,
}

impl CubicSplinePlanner {
    pub fn new() -> Self {
        CubicSplinePlanner {
            path: Vec::new(),
            yaw: Vec::new(),
            curvature: Vec::new(),
            s: Vec::new(),
        }
    }

    pub fn planning(&mut self, waypoints_x: Vec<f64>, waypoints_y: Vec<f64>, ds: f64) -> bool {
        if waypoints_x.len() != waypoints_y.len() || waypoints_x.len() < 2 {
            return false;
        }

        let (path, yaw, curvature, s) = calc_spline_course(waypoints_x, waypoints_y, ds);

        self.path = path;
        self.yaw = yaw;
        self.curvature = curvature;
        self.s = s;

        true
    }
}

impl Default for CubicSplinePlanner {
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

    fn path_fingerprint(path: &[(f64, f64)]) -> (f64, f64, f64, f64) {
        let sum_x: f64 = path.iter().map(|point| point.0).sum();
        let sum_y: f64 = path.iter().map(|point| point.1).sum();
        let weighted_sum_x: f64 = path
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.0)
            .sum();
        let weighted_sum_y: f64 = path
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.1)
            .sum();
        (sum_x, sum_y, weighted_sum_x, weighted_sum_y)
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
    fn test_cubic_spline_planning() {
        let waypoints_x = vec![0.0, 10.0, 20.5, 30.0, 40.5, 50.0];
        let waypoints_y = vec![0.0, -6.0, 5.0, 6.5, 0.0, -4.0];
        let ds = 0.1;

        let mut planner = CubicSplinePlanner::new();
        let result = planner.planning(waypoints_x, waypoints_y, ds);

        assert!(result);
        assert!(!planner.path.is_empty());
        assert!(!planner.yaw.is_empty());
        assert!(!planner.curvature.is_empty());
    }

    #[test]
    fn test_spline2d() {
        let x = vec![0.0, 10.0, 20.5, 30.0];
        let y = vec![0.0, -6.0, 5.0, 6.5];
        let sp = Spline2D::new(x, y);
        assert!(!sp.s.is_empty());
    }

    #[test]
    fn test_calc_spline_course_matches_upstream_main2d_example() {
        let x = vec![-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
        let y = vec![0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0];
        let (path, yaw, curvature, s) = calc_spline_course(x, y, 0.1);

        assert_eq!(path.len(), 432);
        assert_eq!(yaw.len(), 432);
        assert_eq!(curvature.len(), 432);
        assert_eq!(s.len(), 432);

        let expected_samples = [
            (
                0usize,
                (-2.5, 0.7),
                -1.261_911_977_390_588_5,
                3.639_626_203_496_368e-17,
                0.0,
            ),
            (
                1,
                (-2.456_214_414_403_149, 0.562_786_288_975_573_1),
                -1.261_892_330_947_575,
                0.000_272_891_300_476_531_77,
                0.1,
            ),
            (
                10,
                (-2.063_853_173_620_386_4, -0.663_709_921_270_961_1),
                -1.259_911_718_126_614_6,
                0.002_880_022_595_677_534_5,
                1.0,
            ),
            (
                50,
                (-0.526_420_969_786_195_5, -5.097_072_155_044_978_5),
                -1.172_930_102_001_000_1,
                0.080_811_423_731_918_38,
                5.0,
            ),
            (
                431,
                (-0.999_817_756_777_344, -1.999_280_944_483_088),
                -1.819_017_577_858_351_5,
                4.894_060_652_105_897e-06,
                43.1,
            ),
        ];

        for (index, expected_pos, expected_yaw, expected_curvature, expected_s) in expected_samples
        {
            assert!(approx_eq(path[index].0, expected_pos.0, 1e-12));
            assert!(approx_eq(path[index].1, expected_pos.1, 1e-12));
            assert!(approx_eq(yaw[index], expected_yaw, 1e-12));
            assert!(approx_eq(curvature[index], expected_curvature, 1e-12));
            assert!(approx_eq(s[index], expected_s, 1e-12));
        }

        let path_fp = path_fingerprint(&path);
        assert!(approx_eq(path_fp.0, 1_022.623_164_017_587_4, 1e-9));
        assert!(approx_eq(path_fp.1, 357.291_977_281_188_57, 1e-9));
        assert!(approx_eq(path_fp.2, 291_306.562_899_991_5, 1e-6));
        assert!(approx_eq(path_fp.3, 199_642.229_890_529_74, 1e-6));

        let yaw_fp = scalar_fingerprint(&yaw);
        assert!(approx_eq(yaw_fp.0, 9.657_666_682_428_65, 1e-9));
        assert!(approx_eq(yaw_fp.1, -7_330.389_498_653_044, 1e-6));

        let curvature_fp = scalar_fingerprint(&curvature);
        assert!(approx_eq(curvature_fp.0, 62.207_543_933_499_61, 1e-9));
        assert!(approx_eq(curvature_fp.1, -3_950.521_036_466_579_5, 1e-6));

        let s_fp = scalar_fingerprint(&s);
        assert!(approx_eq(s_fp.0, 9_309.6, 1e-9));
        assert!(approx_eq(s_fp.1, 2_687_371.200_000_001_6, 1e-3));
    }

    #[test]
    fn test_spline2d_matches_upstream_main2d_reference_states() {
        let x = vec![-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
        let y = vec![0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0];
        let sp = Spline2D::new(x, y);

        assert!(approx_eq(
            sp.s[sp.s.len() - 1],
            43.100_477_702_041_04,
            1e-12
        ));

        let expected_states = [
            (
                0.0,
                (-2.5, 0.7),
                -1.261_911_977_390_588_5,
                3.639_626_203_496_368e-17,
            ),
            (
                0.1,
                (-2.456_214_414_403_149, 0.562_786_288_975_573_1),
                -1.261_892_330_947_575,
                0.000_272_891_300_476_531_77,
            ),
            (
                1.0,
                (-2.063_853_173_620_386_4, -0.663_709_921_270_961_1),
                -1.259_911_718_126_614_6,
                0.002_880_022_595_677_534_5,
            ),
            (
                5.0,
                (-0.526_420_969_786_195_5, -5.097_072_155_044_978_5),
                -1.172_930_102_001_000_1,
                0.080_811_423_731_918_38,
            ),
            (
                10.0,
                (0.277_082_035_120_894_85, -4.891_782_706_378_051),
                1.505_135_127_918_666_9,
                0.043_260_837_806_513_984,
            ),
            (
                43.000_477_702_041_04,
                (-0.961_848_258_550_467_4, -1.849_485_946_939_665_8),
                -1.819_097_133_814_203_5,
                0.001_025_039_188_263_279_5,
            ),
        ];

        for (arc, expected_pos, expected_yaw, expected_curvature) in expected_states {
            let position = sp.calc_position(arc);
            assert!(approx_eq(position.0, expected_pos.0, 1e-12));
            assert!(approx_eq(position.1, expected_pos.1, 1e-12));
            assert!(approx_eq(sp.calc_yaw(arc), expected_yaw, 1e-12));
            assert!(approx_eq(sp.calc_curvature(arc), expected_curvature, 1e-12));
        }
    }
}
