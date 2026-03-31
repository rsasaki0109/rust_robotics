//! Motion model for State Lattice Planner
//!
//! Implements a bicycle kinematic model for vehicle motion.
//! Based on PythonRobotics implementation.

use std::f64::consts::PI;

/// Vehicle state
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
}

impl VehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64) -> Self {
        Self { x, y, yaw, v }
    }

    pub fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            v: 0.0,
        }
    }
}

/// Motion model configuration
#[derive(Debug, Clone)]
pub struct MotionModelConfig {
    /// Wheelbase length \[m\]
    pub wheelbase: f64,
    /// Distance step for trajectory generation \[m\]
    pub ds: f64,
    /// Default velocity \[m/s\]
    pub default_velocity: f64,
}

impl Default for MotionModelConfig {
    fn default() -> Self {
        Self {
            wheelbase: 1.0,
            ds: 0.1,
            default_velocity: 10.0 / 3.6, // ~2.78 m/s
        }
    }
}

/// Bicycle kinematic motion model
pub struct MotionModel {
    config: MotionModelConfig,
}

impl MotionModel {
    pub fn new(config: MotionModelConfig) -> Self {
        Self { config }
    }

    pub fn with_defaults() -> Self {
        Self::new(MotionModelConfig::default())
    }

    /// Get wheelbase
    pub fn wheelbase(&self) -> f64 {
        self.config.wheelbase
    }

    /// Get distance step
    pub fn ds(&self) -> f64 {
        self.config.ds
    }

    /// Update state using bicycle kinematic model
    pub fn update(&self, state: &VehicleState, v: f64, delta: f64, dt: f64) -> VehicleState {
        let l = self.config.wheelbase;

        VehicleState {
            x: state.x + v * state.yaw.cos() * dt,
            y: state.y + v * state.yaw.sin() * dt,
            yaw: normalize_angle(state.yaw + v / l * delta.tan() * dt),
            v,
        }
    }

    /// Generate trajectory given curvature parameters
    pub fn generate_trajectory(
        &self,
        s: f64,
        k0: f64,
        km: f64,
        kf: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if s <= 0.0 {
            return (vec![0.0], vec![0.0], vec![0.0]);
        }

        let v = self.config.default_velocity;
        let dt = self.config.ds / v;
        let time = s / v;

        let mut x_list = vec![0.0];
        let mut y_list = vec![0.0];
        let mut yaw_list = vec![0.0];

        let mut state = VehicleState::origin();
        let mut step = 0usize;

        loop {
            let t = step as f64 * dt;
            if t >= time {
                break;
            }
            let tau = if time > 0.0 { t / time } else { 0.0 };
            let delta = self.interpolate_curvature(tau, k0, km, kf);

            state = self.update(&state, v, delta, dt);

            x_list.push(state.x);
            y_list.push(state.y);
            yaw_list.push(state.yaw);
            step += 1;
        }

        (x_list, y_list, yaw_list)
    }

    /// Interpolate curvature using quadratic polynomial
    fn interpolate_curvature(&self, t: f64, k0: f64, km: f64, kf: f64) -> f64 {
        let a = 2.0 * (kf + k0 - 2.0 * km);
        let b = -kf - 3.0 * k0 + 4.0 * km;
        let c = k0;

        a * t * t + b * t + c
    }

    /// Generate trajectory and return final state
    pub fn generate_trajectory_final_state(
        &self,
        s: f64,
        k0: f64,
        km: f64,
        kf: f64,
    ) -> (f64, f64, f64) {
        let (x, y, yaw) = self.generate_trajectory(s, k0, km, kf);
        let n = x.len();
        if n == 0 {
            (0.0, 0.0, 0.0)
        } else {
            (x[n - 1], y[n - 1], yaw[n - 1])
        }
    }
}

/// Normalize angle to [-PI, PI]
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    type TrajectorySample = (f64, f64, f64);
    type TrajectoryParams = (f64, f64, f64);

    #[derive(Debug, Clone, Copy)]
    struct TrajectoryFingerprint {
        len: usize,
        sum_x: f64,
        sum_y: f64,
        sum_yaw: f64,
        weighted_sum_x: f64,
        weighted_sum_y: f64,
        weighted_sum_yaw: f64,
        sum_x_sq: f64,
        sum_y_sq: f64,
        sum_yaw_sq: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct TrajectoryFingerprintTolerance {
        sum_x: f64,
        sum_y: f64,
        sum_yaw: f64,
        weighted_sum_x: f64,
        weighted_sum_y: f64,
        weighted_sum_yaw: f64,
        sum_x_sq: f64,
        sum_y_sq: f64,
        sum_yaw_sq: f64,
    }

    fn parse_reference_trajectory_samples(csv: &str) -> Vec<TrajectorySample> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let values: Vec<f64> = line
                    .split(',')
                    .skip(1)
                    .map(|value| value.parse::<f64>().unwrap())
                    .collect();
                assert_eq!(values.len(), 3);
                (values[0], values[1], values[2])
            })
            .collect()
    }

    fn parse_grouped_reference_trajectory_samples(csv: &str) -> Vec<Vec<TrajectorySample>> {
        let mut trajectories = Vec::new();
        let mut current_row = None;
        let mut current_samples = Vec::new();

        for line in csv.lines().skip(1).filter(|line| !line.trim().is_empty()) {
            let values: Vec<&str> = line.split(',').collect();
            assert_eq!(values.len(), 5);

            let row = values[0].parse::<usize>().unwrap();
            let index = values[1].parse::<usize>().unwrap();
            let sample = (
                values[2].parse::<f64>().unwrap(),
                values[3].parse::<f64>().unwrap(),
                values[4].parse::<f64>().unwrap(),
            );

            match current_row {
                Some(current_row_index) if current_row_index != row => {
                    trajectories.push(std::mem::take(&mut current_samples));
                    current_row = Some(row);
                }
                None => current_row = Some(row),
                _ => {}
            }

            assert_eq!(index, current_samples.len());
            current_samples.push(sample);
        }

        if !current_samples.is_empty() {
            trajectories.push(current_samples);
        }

        trajectories
    }

    fn parse_terminal_trajectory_params(csv: &str) -> Vec<TrajectoryParams> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let values: Vec<f64> = line
                    .split(',')
                    .map(|value| value.parse::<f64>().unwrap())
                    .collect();
                assert_eq!(values.len(), 6);
                (values[3], values[4], values[5])
            })
            .collect()
    }

    fn parse_reference_trajectory_fingerprints(csv: &str) -> Vec<TrajectoryFingerprint> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .enumerate()
            .map(|(index, line)| {
                let values: Vec<&str> = line.split(',').collect();
                assert_eq!(values.len(), 11);
                assert_eq!(values[0].parse::<usize>().unwrap(), index);

                TrajectoryFingerprint {
                    len: values[1].parse::<usize>().unwrap(),
                    sum_x: values[2].parse::<f64>().unwrap(),
                    sum_y: values[3].parse::<f64>().unwrap(),
                    sum_yaw: values[4].parse::<f64>().unwrap(),
                    weighted_sum_x: values[5].parse::<f64>().unwrap(),
                    weighted_sum_y: values[6].parse::<f64>().unwrap(),
                    weighted_sum_yaw: values[7].parse::<f64>().unwrap(),
                    sum_x_sq: values[8].parse::<f64>().unwrap(),
                    sum_y_sq: values[9].parse::<f64>().unwrap(),
                    sum_yaw_sq: values[10].parse::<f64>().unwrap(),
                }
            })
            .collect()
    }

    fn trajectory_fingerprint(x: &[f64], y: &[f64], yaw: &[f64]) -> TrajectoryFingerprint {
        assert_eq!(x.len(), y.len());
        assert_eq!(x.len(), yaw.len());

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_yaw = 0.0;
        let mut weighted_sum_x = 0.0;
        let mut weighted_sum_y = 0.0;
        let mut weighted_sum_yaw = 0.0;
        let mut sum_x_sq = 0.0;
        let mut sum_y_sq = 0.0;
        let mut sum_yaw_sq = 0.0;

        for (index, ((&x, &y), &yaw)) in x.iter().zip(y.iter()).zip(yaw.iter()).enumerate() {
            let weight = (index + 1) as f64;
            sum_x += x;
            sum_y += y;
            sum_yaw += yaw;
            weighted_sum_x += weight * x;
            weighted_sum_y += weight * y;
            weighted_sum_yaw += weight * yaw;
            sum_x_sq += x * x;
            sum_y_sq += y * y;
            sum_yaw_sq += yaw * yaw;
        }

        TrajectoryFingerprint {
            len: x.len(),
            sum_x,
            sum_y,
            sum_yaw,
            weighted_sum_x,
            weighted_sum_y,
            weighted_sum_yaw,
            sum_x_sq,
            sum_y_sq,
            sum_yaw_sq,
        }
    }

    fn assert_trajectory_samples_match(
        x: &[f64],
        y: &[f64],
        yaw: &[f64],
        expected: &[TrajectorySample],
        tolerance: TrajectorySample,
    ) {
        assert_eq!(x.len(), expected.len());
        assert_eq!(y.len(), expected.len());
        assert_eq!(yaw.len(), expected.len());

        for (((&x, &y), &yaw), expected) in x
            .iter()
            .zip(y.iter())
            .zip(yaw.iter())
            .zip(expected.iter().copied())
        {
            let observed = (x, y, yaw);
            assert!(
                (x - expected.0).abs() < tolerance.0,
                "x mismatch: observed={observed:?}, expected={expected:?}"
            );
            assert!(
                (y - expected.1).abs() < tolerance.1,
                "y mismatch: observed={observed:?}, expected={expected:?}"
            );
            assert!(
                (yaw - expected.2).abs() < tolerance.2,
                "yaw mismatch: observed={observed:?}, expected={expected:?}"
            );
        }
    }

    fn assert_trajectory_fingerprint_matches(
        actual: &TrajectoryFingerprint,
        expected: &TrajectoryFingerprint,
        tolerance: TrajectoryFingerprintTolerance,
    ) {
        assert_eq!(actual.len, expected.len);
        assert!(
            (actual.sum_x - expected.sum_x).abs() < tolerance.sum_x,
            "sum_x mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.sum_y - expected.sum_y).abs() < tolerance.sum_y,
            "sum_y mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.sum_yaw - expected.sum_yaw).abs() < tolerance.sum_yaw,
            "sum_yaw mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.weighted_sum_x - expected.weighted_sum_x).abs() < tolerance.weighted_sum_x,
            "weighted_sum_x mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.weighted_sum_y - expected.weighted_sum_y).abs() < tolerance.weighted_sum_y,
            "weighted_sum_y mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.weighted_sum_yaw - expected.weighted_sum_yaw).abs()
                < tolerance.weighted_sum_yaw,
            "weighted_sum_yaw mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.sum_x_sq - expected.sum_x_sq).abs() < tolerance.sum_x_sq,
            "sum_x_sq mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.sum_y_sq - expected.sum_y_sq).abs() < tolerance.sum_y_sq,
            "sum_y_sq mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.sum_yaw_sq - expected.sum_yaw_sq).abs() < tolerance.sum_yaw_sq,
            "sum_yaw_sq mismatch: actual={actual:?}, expected={expected:?}"
        );
    }

    fn assert_terminal_table_fingerprint_matches(
        terminal_csv: &str,
        fingerprint_csv: &str,
        k0: f64,
        tolerance: TrajectoryFingerprintTolerance,
    ) {
        let model = MotionModel::with_defaults();
        let params = parse_terminal_trajectory_params(terminal_csv);
        let expected = parse_reference_trajectory_fingerprints(fingerprint_csv);

        assert_eq!(params.len(), expected.len());

        for ((s, km, kf), expected) in params.iter().copied().zip(expected.iter()) {
            let (x, y, yaw) = model.generate_trajectory(s, k0, km, kf);
            let actual = trajectory_fingerprint(&x, &y, &yaw);
            assert_trajectory_fingerprint_matches(&actual, expected, tolerance);
        }
    }

    fn assert_terminal_table_exact_samples_match(
        terminal_csv: &str,
        sample_csv: &str,
        k0: f64,
        tolerance: TrajectorySample,
    ) {
        let model = MotionModel::with_defaults();
        let params = parse_terminal_trajectory_params(terminal_csv);
        let expected = parse_grouped_reference_trajectory_samples(sample_csv);

        assert_eq!(params.len(), expected.len());

        for ((s, km, kf), expected_samples) in params.iter().copied().zip(expected.iter()) {
            let (x, y, yaw) = model.generate_trajectory(s, k0, km, kf);
            assert_trajectory_samples_match(&x, &y, &yaw, expected_samples, tolerance);
        }
    }

    #[test]
    fn test_vehicle_state() {
        let state = VehicleState::new(1.0, 2.0, 0.5, 1.0);
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.5);
    }

    #[test]
    fn test_motion_model_update() {
        let model = MotionModel::with_defaults();
        let state = VehicleState::origin();
        let next = model.update(&state, 1.0, 0.0, 0.1);

        assert!((next.x - 0.1).abs() < 1e-10);
        assert!(next.y.abs() < 1e-10);
        assert!(next.yaw.abs() < 1e-10);
    }

    #[test]
    fn test_curvature_interpolation() {
        let model = MotionModel::with_defaults();

        let k0 = 0.0;
        let km = 0.1;
        let kf = 0.2;

        let k_start = model.interpolate_curvature(0.0, k0, km, kf);
        let k_mid = model.interpolate_curvature(0.5, k0, km, kf);
        let k_end = model.interpolate_curvature(1.0, k0, km, kf);

        assert!((k_start - k0).abs() < 1e-10);
        assert!((k_mid - km).abs() < 1e-10);
        assert!((k_end - kf).abs() < 1e-10);
    }

    #[test]
    fn test_generate_trajectory_straight() {
        let model = MotionModel::with_defaults();
        let (x, y, _yaw) = model.generate_trajectory(1.0, 0.0, 0.0, 0.0);

        assert!(x.len() > 1);
        for yi in &y {
            assert!(yi.abs() < 1e-6);
        }
    }

    #[test]
    fn test_generate_trajectory_turn() {
        let model = MotionModel::with_defaults();
        let (x, y, _yaw) = model.generate_trajectory(2.0, 0.1, 0.1, 0.1);

        assert!(x.len() > 1);
        let final_y = y.last().unwrap();
        assert!(*final_y > 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 1e-10);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }

    #[test]
    fn test_generate_trajectory_matches_upstream_uniform1_row0_samples() {
        let model = MotionModel::with_defaults();
        let expected = parse_reference_trajectory_samples(include_str!(
            "testdata/uniform1_row0_trajectory.csv"
        ));
        let (x, y, yaw) = model.generate_trajectory(
            23.380_658_812_582,
            0.0,
            -0.101556678511287,
            0.004757708055363,
        );

        assert_trajectory_samples_match(&x, &y, &yaw, &expected, (1e-9, 1e-9, 1e-9));
    }

    #[test]
    fn test_generate_trajectory_matches_upstream_uniform2_row9_samples() {
        let model = MotionModel::with_defaults();
        let expected = parse_reference_trajectory_samples(include_str!(
            "testdata/uniform2_row9_trajectory.csv"
        ));
        let (x, y, yaw) = model.generate_trajectory(
            20.421202670847993,
            0.1,
            0.028733328960710,
            -0.157833468427762,
        );

        assert_trajectory_samples_match(&x, &y, &yaw, &expected, (1e-9, 1e-9, 1e-9));
    }

    #[test]
    fn test_generate_trajectory_matches_upstream_lane1_row2_samples() {
        let model = MotionModel::with_defaults();
        let expected =
            parse_reference_trajectory_samples(include_str!("testdata/lane1_row2_trajectory.csv"));
        let (x, y, yaw) = model.generate_trajectory(
            15.865930382436854,
            0.0,
            0.147107848059632,
            -0.569190510845161,
        );

        assert_trajectory_samples_match(&x, &y, &yaw, &expected, (1e-9, 1e-9, 1e-9));
    }

    #[test]
    fn test_generate_trajectory_exact_samples_match_upstream_lane_state_sampling_test1_all_rows() {
        assert_terminal_table_exact_samples_match(
            include_str!("testdata/lane_state_sampling_test1.csv"),
            include_str!("testdata/lane_state_sampling_test1_all_samples.csv"),
            0.0,
            (1e-9, 1e-9, 1e-9),
        );
    }

    #[test]
    fn test_generate_trajectory_matches_upstream_biased1_row29_samples() {
        let model = MotionModel::with_defaults();
        let expected = parse_reference_trajectory_samples(include_str!(
            "testdata/biased1_row29_trajectory.csv"
        ));
        let (x, y, yaw) = model.generate_trajectory(
            19.979_736_701_974_8,
            0.0,
            -0.006547264051110,
            0.119218956522636,
        );

        assert_trajectory_samples_match(&x, &y, &yaw, &expected, (1e-9, 1e-9, 1e-9));
    }

    #[test]
    fn test_generate_trajectory_matches_upstream_biased2_row14_samples() {
        let model = MotionModel::with_defaults();
        let expected = parse_reference_trajectory_samples(include_str!(
            "testdata/biased2_row14_trajectory.csv"
        ));
        let (x, y, yaw) = model.generate_trajectory(
            21.125_744_384_337_9,
            0.0,
            0.073986407526083,
            -0.049246868802640,
        );

        assert_trajectory_samples_match(&x, &y, &yaw, &expected, (1e-9, 1e-9, 1e-9));
    }

    #[test]
    fn test_generate_trajectory_fingerprints_match_upstream_uniform_terminal_state_sampling_test1()
    {
        assert_terminal_table_fingerprint_matches(
            include_str!("testdata/uniform_terminal_state_sampling_test1.csv"),
            include_str!("testdata/uniform_terminal_state_sampling_test1_fingerprints.csv"),
            0.0,
            TrajectoryFingerprintTolerance {
                sum_x: 1e-6,
                sum_y: 1e-6,
                sum_yaw: 1e-6,
                weighted_sum_x: 1e-4,
                weighted_sum_y: 1e-4,
                weighted_sum_yaw: 1e-4,
                sum_x_sq: 1e-4,
                sum_y_sq: 1e-4,
                sum_yaw_sq: 1e-4,
            },
        );
    }

    #[test]
    fn test_generate_trajectory_exact_samples_match_upstream_uniform_terminal_state_sampling_test1_all_rows(
    ) {
        assert_terminal_table_exact_samples_match(
            include_str!("testdata/uniform_terminal_state_sampling_test1.csv"),
            include_str!("testdata/uniform_terminal_state_sampling_test1_all_samples.csv"),
            0.0,
            (1e-9, 1e-9, 1e-9),
        );
    }

    #[test]
    fn test_generate_trajectory_fingerprints_match_upstream_uniform_terminal_state_sampling_test2()
    {
        assert_terminal_table_fingerprint_matches(
            include_str!("testdata/uniform_terminal_state_sampling_test2.csv"),
            include_str!("testdata/uniform_terminal_state_sampling_test2_fingerprints.csv"),
            0.1,
            TrajectoryFingerprintTolerance {
                sum_x: 1e-6,
                sum_y: 1e-6,
                sum_yaw: 1e-6,
                weighted_sum_x: 1e-4,
                weighted_sum_y: 1e-4,
                weighted_sum_yaw: 1e-4,
                sum_x_sq: 1e-4,
                sum_y_sq: 1e-4,
                sum_yaw_sq: 1e-4,
            },
        );
    }

    #[test]
    fn test_generate_trajectory_exact_samples_match_upstream_uniform_terminal_state_sampling_test2_all_rows(
    ) {
        assert_terminal_table_exact_samples_match(
            include_str!("testdata/uniform_terminal_state_sampling_test2.csv"),
            include_str!("testdata/uniform_terminal_state_sampling_test2_all_samples.csv"),
            0.1,
            (1e-9, 1e-9, 1e-9),
        );
    }

    #[test]
    fn test_generate_trajectory_fingerprints_match_upstream_biased_terminal_state_sampling_test1() {
        assert_terminal_table_fingerprint_matches(
            include_str!("testdata/biased_terminal_state_sampling_test1.csv"),
            include_str!("testdata/biased_terminal_state_sampling_test1_fingerprints.csv"),
            0.0,
            TrajectoryFingerprintTolerance {
                sum_x: 1e-6,
                sum_y: 1e-6,
                sum_yaw: 1e-6,
                weighted_sum_x: 1e-4,
                weighted_sum_y: 1e-4,
                weighted_sum_yaw: 1e-4,
                sum_x_sq: 1e-4,
                sum_y_sq: 1e-4,
                sum_yaw_sq: 1e-4,
            },
        );
    }

    #[test]
    fn test_generate_trajectory_exact_samples_match_upstream_biased_terminal_state_sampling_test1_all_rows(
    ) {
        assert_terminal_table_exact_samples_match(
            include_str!("testdata/biased_terminal_state_sampling_test1.csv"),
            include_str!("testdata/biased_terminal_state_sampling_test1_all_samples.csv"),
            0.0,
            (1e-9, 1e-9, 1e-9),
        );
    }

    #[test]
    fn test_generate_trajectory_fingerprints_match_upstream_biased_terminal_state_sampling_test2() {
        assert_terminal_table_fingerprint_matches(
            include_str!("testdata/biased_terminal_state_sampling_test2.csv"),
            include_str!("testdata/biased_terminal_state_sampling_test2_fingerprints.csv"),
            0.0,
            TrajectoryFingerprintTolerance {
                sum_x: 1e-6,
                sum_y: 1e-6,
                sum_yaw: 1e-6,
                weighted_sum_x: 1e-4,
                weighted_sum_y: 1e-4,
                weighted_sum_yaw: 1e-4,
                sum_x_sq: 1e-4,
                sum_y_sq: 1e-4,
                sum_yaw_sq: 1e-4,
            },
        );
    }

    #[test]
    fn test_generate_trajectory_fingerprints_match_upstream_lane_state_sampling_test1() {
        assert_terminal_table_fingerprint_matches(
            include_str!("testdata/lane_state_sampling_test1.csv"),
            include_str!("testdata/lane_state_sampling_test1_fingerprints.csv"),
            0.0,
            TrajectoryFingerprintTolerance {
                sum_x: 1e-6,
                sum_y: 1e-6,
                sum_yaw: 1e-6,
                weighted_sum_x: 1e-4,
                weighted_sum_y: 1e-4,
                weighted_sum_yaw: 1e-4,
                sum_x_sq: 1e-4,
                sum_y_sq: 1e-4,
                sum_yaw_sq: 1e-4,
            },
        );
    }

    #[test]
    fn test_generate_trajectory_exact_samples_match_upstream_biased_terminal_state_sampling_test2_all_rows(
    ) {
        assert_terminal_table_exact_samples_match(
            include_str!("testdata/biased_terminal_state_sampling_test2.csv"),
            include_str!("testdata/biased_terminal_state_sampling_test2_all_samples.csv"),
            0.0,
            (1e-9, 1e-9, 1e-9),
        );
    }
}
