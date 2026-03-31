//! Model Predictive Trajectory Generator
//!
//! Optimizes trajectory parameters to reach target states using Newton-Raphson method.
//! Based on PythonRobotics implementation.

use nalgebra::{Matrix3, Vector3};

use super::motion_model::{normalize_angle, MotionModel};

const DEFAULT_LOOKUP_TABLE_CSV: &str = include_str!("lookup_table.csv");

/// Trajectory parameters: [s, km, kf]
/// - s: arc length
/// - km: middle curvature
/// - kf: final curvature
pub type TrajectoryParams = Vector3<f64>;
pub type GeneratedTrajectory = (Vec<f64>, Vec<f64>, Vec<f64>, TrajectoryParams);

/// Target state: [x, y, yaw]
pub type TargetState = Vector3<f64>;

/// Trajectory generator configuration
#[derive(Debug, Clone)]
pub struct TrajectoryGeneratorConfig {
    /// Maximum optimization iterations
    pub max_iter: usize,
    /// Convergence threshold for cost
    pub cost_threshold: f64,
    /// Finite difference step for Jacobian
    pub h: Vector3<f64>,
    /// Initial curvature (steering at start)
    pub k0: f64,
}

impl Default for TrajectoryGeneratorConfig {
    fn default() -> Self {
        Self {
            max_iter: 100,
            cost_threshold: 0.1,
            h: Vector3::new(0.5, 0.02, 0.02), // [ds, dkm, dkf]
            k0: 0.0,
        }
    }
}

/// Model Predictive Trajectory Generator
pub struct TrajectoryGenerator {
    pub(crate) motion_model: MotionModel,
    config: TrajectoryGeneratorConfig,
}

impl TrajectoryGenerator {
    pub fn new(motion_model: MotionModel, config: TrajectoryGeneratorConfig) -> Self {
        Self {
            motion_model,
            config,
        }
    }

    pub fn with_defaults() -> Self {
        Self::new(
            MotionModel::with_defaults(),
            TrajectoryGeneratorConfig::default(),
        )
    }

    /// Set initial curvature
    pub fn set_k0(&mut self, k0: f64) {
        self.config.k0 = k0;
    }

    /// Generate trajectory with given parameters
    pub fn generate(&self, params: &TrajectoryParams) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        self.motion_model.generate_trajectory(
            params[0], // s
            self.config.k0,
            params[1], // km
            params[2], // kf
        )
    }

    /// Calculate difference between current trajectory endpoint and target
    fn calc_diff(&self, params: &TrajectoryParams, target: &TargetState) -> Vector3<f64> {
        let (x_final, y_final, yaw_final) = self.motion_model.generate_trajectory_final_state(
            params[0],
            self.config.k0,
            params[1],
            params[2],
        );

        Vector3::new(
            x_final - target[0],
            y_final - target[1],
            normalize_angle(yaw_final - target[2]),
        )
    }

    /// Calculate cost (L2 norm of difference)
    fn calc_cost(&self, params: &TrajectoryParams, target: &TargetState) -> f64 {
        let diff = self.calc_diff(params, target);
        diff.norm()
    }

    /// Calculate Jacobian matrix using finite differences
    fn calc_jacobian(&self, params: &TrajectoryParams, target: &TargetState) -> Matrix3<f64> {
        let h = &self.config.h;
        let mut jacobian = Matrix3::zeros();
        let diff_current = self.calc_diff(params, target);

        for i in 0..3 {
            let mut params_plus = *params;
            params_plus[i] += h[i];
            let mut params_minus = *params;
            params_minus[i] -= h[i];

            let diff_plus = self.calc_diff(&params_plus, target);
            let diff_minus = self.calc_diff(&params_minus, target);

            for j in 0..3 {
                let delta = if i == 0 && params_minus[i] <= 0.0 {
                    diff_plus[j] - diff_current[j]
                } else {
                    diff_plus[j] - diff_minus[j]
                };
                let denom = if i == 0 && params_minus[i] <= 0.0 {
                    h[i]
                } else {
                    2.0 * h[i]
                };
                jacobian[(j, i)] = delta / denom;
            }
        }

        jacobian
    }

    /// Line search to find optimal step size
    fn line_search(
        &self,
        params: &TrajectoryParams,
        dp: &Vector3<f64>,
        target: &TargetState,
    ) -> f64 {
        let alphas = [1.0, 1.5];
        let mut best_alpha = 1.0;
        let mut min_cost = f64::MAX;

        for &alpha in &alphas {
            let new_params = params + alpha * dp;
            if new_params[0] > 0.0 {
                let cost = self.calc_cost(&new_params, target);
                if cost < min_cost {
                    min_cost = cost;
                    best_alpha = alpha;
                }
            }
        }

        best_alpha
    }

    /// Optimize trajectory to reach target state
    pub fn optimize(
        &self,
        target: &TargetState,
        init_params: &TrajectoryParams,
    ) -> Option<TrajectoryParams> {
        let mut params = *init_params;

        for _iter in 0..self.config.max_iter {
            let cost = self.calc_cost(&params, target);

            if cost < self.config.cost_threshold {
                return Some(params);
            }

            let jacobian = self.calc_jacobian(&params, target);

            let diff = self.calc_diff(&params, target);

            if let Some(j_inv) = jacobian.try_inverse() {
                let dp = -j_inv * diff;

                let alpha = self.line_search(&params, &dp, target);
                params += alpha * dp;

                if params[0] < 0.1 {
                    params[0] = 0.1;
                }
            } else {
                return None;
            }
        }

        None
    }

    /// Generate optimized trajectory to reach target
    pub fn generate_optimized(
        &self,
        target: &TargetState,
        init_params: &TrajectoryParams,
    ) -> Option<GeneratedTrajectory> {
        let params = self.optimize(target, init_params)?;
        let (x, y, yaw) = self.generate(&params);
        Some((x, y, yaw, params))
    }
}

/// Lookup table entry for trajectory generation
#[derive(Debug, Clone)]
pub struct LookupTableEntry {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub s: f64,
    pub km: f64,
    pub kf: f64,
}

impl LookupTableEntry {
    pub fn new(x: f64, y: f64, yaw: f64, s: f64, km: f64, kf: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            s,
            km,
            kf,
        }
    }

    /// Get target state from entry
    pub fn target(&self) -> TargetState {
        Vector3::new(self.x, self.y, self.yaw)
    }

    /// Get trajectory parameters from entry
    pub fn params(&self) -> TrajectoryParams {
        Vector3::new(self.s, self.km, self.kf)
    }

    /// Calculate distance to a target state
    pub fn distance_to(&self, target: &TargetState) -> f64 {
        let dx = self.x - target[0];
        let dy = self.y - target[1];
        let dyaw = self.yaw - target[2];
        (dx * dx + dy * dy + dyaw * dyaw).sqrt()
    }
}

/// Lookup table for efficient trajectory initialization
#[derive(Debug, Clone)]
pub struct LookupTable {
    entries: Vec<LookupTableEntry>,
}

impl LookupTable {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Create lookup table from CSV data
    pub fn from_csv(csv_data: &str) -> Self {
        let mut entries = Vec::new();

        for line in csv_data.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with("x,") {
                continue;
            }

            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 6 {
                if let (Ok(x), Ok(y), Ok(yaw), Ok(s), Ok(km), Ok(kf)) = (
                    parts[0].trim().parse::<f64>(),
                    parts[1].trim().parse::<f64>(),
                    parts[2].trim().parse::<f64>(),
                    parts[3].trim().parse::<f64>(),
                    parts[4].trim().parse::<f64>(),
                    parts[5].trim().parse::<f64>(),
                ) {
                    entries.push(LookupTableEntry::new(x, y, yaw, s, km, kf));
                }
            }
        }

        Self { entries }
    }

    /// Generate default lookup table with common trajectories
    pub fn generate_default() -> Self {
        Self::from_csv(DEFAULT_LOOKUP_TABLE_CSV)
    }

    /// Find nearest entry to target state
    pub fn find_nearest(&self, target: &TargetState) -> Option<&LookupTableEntry> {
        self.entries.iter().min_by(|a, b| {
            a.distance_to(target)
                .partial_cmp(&b.distance_to(target))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Add entry to table
    pub fn add(&mut self, entry: LookupTableEntry) {
        self.entries.push(entry);
    }

    /// Get number of entries
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Check if table is empty
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Convert to CSV string
    pub fn to_csv(&self) -> String {
        let mut csv = String::from("x,y,yaw,s,km,kf\n");
        for entry in &self.entries {
            csv.push_str(&format!(
                "{},{},{},{},{},{}\n",
                entry.x, entry.y, entry.yaw, entry.s, entry.km, entry.kf
            ));
        }
        csv
    }
}

impl Default for LookupTable {
    fn default() -> Self {
        Self::generate_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trajectory_generator_straight() {
        let generator = TrajectoryGenerator::with_defaults();
        let params = Vector3::new(5.0, 0.0, 0.0);

        let (x, y, _yaw) = generator.generate(&params);

        assert!(x.len() > 1);
        let final_x = x.last().unwrap();
        let final_y = y.last().unwrap();
        assert!(*final_x > 4.0);
        assert!(final_y.abs() < 0.1);
    }

    #[test]
    fn test_trajectory_generator_turn() {
        let generator = TrajectoryGenerator::with_defaults();
        let params = Vector3::new(5.0, 0.1, 0.1);

        let (x, y, _yaw) = generator.generate(&params);

        assert!(x.len() > 1);
        let final_y = y.last().unwrap();
        assert!(*final_y > 0.0);
    }

    #[test]
    fn test_calc_diff() {
        let generator = TrajectoryGenerator::with_defaults();
        let target = Vector3::new(5.0, 0.0, 0.0);
        let params = Vector3::new(5.0, 0.0, 0.0);

        let diff = generator.calc_diff(&params, &target);

        assert!(diff.norm() < 1.0);
    }

    #[test]
    fn test_optimize_straight() {
        let generator = TrajectoryGenerator::with_defaults();
        let target = Vector3::new(10.0, 0.0, 0.0);
        let init_params = Vector3::new(10.0, 0.0, 0.0);

        let result = generator.optimize(&target, &init_params);

        assert!(result.is_some());
        let params = result.unwrap();
        assert!(params[0] > 0.0);
    }

    #[test]
    fn test_optimize_turn() {
        let generator = TrajectoryGenerator::with_defaults();
        let target = Vector3::new(8.0, 3.0, 0.5);
        let init_params = Vector3::new(10.0, 0.05, 0.05);

        let result = generator.optimize(&target, &init_params);

        if let Some(params) = result {
            assert!(params[0] > 0.0);
        }
    }

    #[test]
    fn test_optimize_matches_upstream_lane_reference() {
        let generator = TrajectoryGenerator::with_defaults();
        let target = Vector3::new(10.0, 9.0, 0.0);
        let init_params = Vector3::new(13.45362404707371, 0.1482242831571022, -0.5606578442626601);

        let params = generator.optimize(&target, &init_params).unwrap();

        assert!((params[0] - 14.806296460297).abs() < 1e-9);
        assert!((params[1] - 0.148478839778).abs() < 1e-9);
        assert!((params[2] - -0.57288113757).abs() < 1e-9);
    }

    #[test]
    fn test_lookup_table_default() {
        let table = LookupTable::generate_default();
        assert!(!table.is_empty());
        assert_eq!(table.len(), 81);
    }

    #[test]
    fn test_lookup_table_default_matches_upstream_reference_rows() {
        let table = LookupTable::generate_default();
        let first = &table.entries[0];
        let last = table.entries.last().unwrap();

        assert!((first.x - 1.0).abs() < 1e-12);
        assert!((first.y - 0.0).abs() < 1e-12);
        assert!((first.yaw - 0.0).abs() < 1e-12);
        assert!((first.s - 1.0).abs() < 1e-12);
        assert!((first.km - 0.0).abs() < 1e-12);
        assert!((first.kf - 0.0).abs() < 1e-12);

        assert!((last.x - 24.960019173190652).abs() < 1e-12);
        assert!((last.y - 17.98909417109214).abs() < 1e-12);
        assert!((last.yaw - 0.011594018486178026).abs() < 1e-12);
        assert!((last.s - 33.0995680641525).abs() < 1e-12);
        assert!((last.km - 0.05634561447882407).abs() < 1e-12);
        assert!((last.kf - -0.22402297280749597).abs() < 1e-12);
    }

    #[test]
    fn test_lookup_table_find_nearest() {
        let table = LookupTable::generate_default();
        let target = Vector3::new(10.0, 0.0, 0.0);

        let nearest = table.find_nearest(&target);
        assert!(nearest.is_some());
    }

    #[test]
    fn test_lookup_table_csv() {
        let table = LookupTable::generate_default();
        let csv = table.to_csv();

        assert!(csv.contains("x,y,yaw,s,km,kf"));

        let parsed = LookupTable::from_csv(&csv);
        assert_eq!(table.len(), parsed.len());
    }

    #[test]
    fn test_lookup_entry_distance() {
        let entry = LookupTableEntry::new(10.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let target = Vector3::new(10.0, 0.0, 0.0);

        assert!(entry.distance_to(&target) < 0.001);
    }
}
