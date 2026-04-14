#![allow(
    dead_code,
    clippy::items_after_test_module,
    clippy::needless_borrows_for_generic_args,
    clippy::ptr_arg,
    clippy::legacy_numeric_constants
)]

/*!
 * Iterative Closest Point (ICP) SLAM implementation
 *
 * This module implements the ICP algorithm for point cloud registration.
 * It finds the optimal transformation (rotation and translation) between
 * two point sets by iteratively minimizing the distance between corresponding points.
 *
 * Ported from PythonRobotics
 * Original authors: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı, Shamil Gemuev
 */

use nalgebra::{DMatrix, DVector, Matrix2, SVector, Vector2};
use nearest_neighbor::KdTree;
use rand::Rng;
use std::f64;

// ICP parameters
const EPS: f64 = 0.0001;
const MAX_ITER: usize = 100;

pub struct ICPResult {
    pub rotation: DMatrix<f64>,
    pub translation: DVector<f64>,
    pub iterations: usize,
    /// Sum of nearest-neighbor distances (legacy; scales with point count).
    pub final_error: f64,
    /// Mean nearest-neighbor distance \[m/point\]: `final_error / point_count`.
    pub final_error_mean: f64,
    pub point_count: usize,
    pub converged: bool,
}

/// Main ICP matching function
///
/// # Arguments
/// * `previous_points` - Points from the previous frame (2×N or 3×N matrix)
/// * `current_points` - Points from the current frame (2×N or 3×N matrix)
///
/// # Returns
/// * `ICPResult` containing rotation matrix, translation vector, and convergence info
pub fn icp_matching(previous_points: &DMatrix<f64>, current_points: &DMatrix<f64>) -> ICPResult {
    let mut h_matrix: Option<DMatrix<f64>> = None;
    let mut d_error = f64::INFINITY;
    let mut pre_error = f64::INFINITY;
    let mut count = 0;
    let mut current_pts = current_points.clone();

    while d_error >= EPS {
        count += 1;

        let (indexes, error) = nearest_neighbor_association(previous_points, &current_pts);
        let previous_indexed = select_columns(previous_points, &indexes);
        let (rt, tt) = svd_motion_estimation(&previous_indexed, &current_pts);

        // Update current points: current_points = (Rt @ current_points) + Tt
        let rotated_pts = &rt * &current_pts;
        current_pts = rotated_pts.add_scalar_to_each_column(&tt);

        d_error = pre_error - error;

        if d_error < 0.0 {
            break;
        }

        pre_error = error;
        h_matrix = Some(update_homogeneous_matrix(h_matrix, &rt, &tt));

        if d_error <= EPS || count >= MAX_ITER {
            break;
        }
    }

    let h = h_matrix.unwrap_or_else(|| {
        DMatrix::identity(previous_points.nrows() + 1, previous_points.nrows() + 1)
    });
    let dim = previous_points.nrows();

    let rotation = h.view((0, 0), (dim, dim)).into_owned();
    let translation = h.column(dim).rows(0, dim).into_owned();

    let point_count = current_points.ncols().max(1);
    let final_error_mean = pre_error / point_count as f64;

    ICPResult {
        rotation,
        translation,
        iterations: count,
        final_error: pre_error,
        final_error_mean,
        point_count,
        converged: d_error <= EPS && count < MAX_ITER,
    }
}

/// Update the homogeneous transformation matrix
fn update_homogeneous_matrix(
    h_in: Option<DMatrix<f64>>,
    r: &DMatrix<f64>,
    t: &DVector<f64>,
) -> DMatrix<f64> {
    let r_size = r.nrows();
    let mut h = DMatrix::zeros(r_size + 1, r_size + 1);

    // Set rotation part
    h.view_mut((0, 0), (r_size, r_size)).copy_from(r);
    // Set translation part
    h.view_mut((0, r_size), (r_size, 1)).copy_from(&t.column(0));
    // Set homogeneous coordinate
    h[(r_size, r_size)] = 1.0;

    match h_in {
        None => h,
        Some(h_prev) => &h_prev * &h,
    }
}

/// Find nearest neighbor associations between point sets using KdTree
fn nearest_neighbor_association(
    previous_points: &DMatrix<f64>,
    current_points: &DMatrix<f64>,
) -> (Vec<usize>, f64) {
    let dim = previous_points.nrows();

    // Build KdTree from previous_points (reference point cloud)
    if dim == 2 {
        let prev_svecs: Vec<SVector<f64, 2>> = (0..previous_points.ncols())
            .map(|i| SVector::<f64, 2>::from([previous_points[(0, i)], previous_points[(1, i)]]))
            .collect();
        let tree = KdTree::new(&prev_svecs, 2);

        let mut indexes = Vec::with_capacity(current_points.ncols());
        let mut error = 0.0;

        for j in 0..current_points.ncols() {
            let query = SVector::<f64, 2>::from([current_points[(0, j)], current_points[(1, j)]]);
            let (argmin, sq_dist) = tree.search(&query);
            indexes.push(argmin.unwrap_or(0));
            error += sq_dist.sqrt();
        }
        (indexes, error)
    } else {
        // Fallback to brute-force for non-2D cases
        let mut indexes = Vec::with_capacity(current_points.ncols());
        let mut error = 0.0;

        for j in 0..current_points.ncols() {
            let current_point = current_points.column(j);
            let mut min_dist = f64::INFINITY;
            let mut best_idx = 0;

            for i in 0..previous_points.ncols() {
                let prev_point = previous_points.column(i);
                let dist = (current_point - prev_point).norm();
                if dist < min_dist {
                    min_dist = dist;
                    best_idx = i;
                }
            }
            indexes.push(best_idx);
            error += min_dist;
        }
        (indexes, error)
    }
}

/// Select columns from matrix based on indices
fn select_columns(matrix: &DMatrix<f64>, indices: &[usize]) -> DMatrix<f64> {
    let mut result = DMatrix::zeros(matrix.nrows(), indices.len());
    for (j, &idx) in indices.iter().enumerate() {
        result.set_column(j, &matrix.column(idx));
    }
    result
}

/// Add scalar to each column of a matrix (for translation)
trait AddScalarToEachColumn {
    fn add_scalar_to_each_column(&self, scalar: &DVector<f64>) -> Self;
}

impl AddScalarToEachColumn for DMatrix<f64> {
    fn add_scalar_to_each_column(&self, scalar: &DVector<f64>) -> Self {
        let mut result = self.clone();
        for j in 0..result.ncols() {
            for i in 0..result.nrows() {
                result[(i, j)] += scalar[i];
            }
        }
        result
    }
}

/// SVD-based motion estimation for 2D points
fn svd_motion_estimation(
    previous_points: &DMatrix<f64>,
    current_points: &DMatrix<f64>,
) -> (DMatrix<f64>, DVector<f64>) {
    let n_points = previous_points.ncols();

    // Calculate centroids
    let mut pm_x = 0.0;
    let mut pm_y = 0.0;
    let mut cm_x = 0.0;
    let mut cm_y = 0.0;

    for j in 0..n_points {
        pm_x += previous_points[(0, j)];
        pm_y += previous_points[(1, j)];
        cm_x += current_points[(0, j)];
        cm_y += current_points[(1, j)];
    }

    pm_x /= n_points as f64;
    pm_y /= n_points as f64;
    cm_x /= n_points as f64;
    cm_y /= n_points as f64;

    // Shift points to centroids
    let mut p_shift = DMatrix::zeros(2, n_points);
    let mut c_shift = DMatrix::zeros(2, n_points);

    for j in 0..n_points {
        p_shift[(0, j)] = previous_points[(0, j)] - pm_x;
        p_shift[(1, j)] = previous_points[(1, j)] - pm_y;
        c_shift[(0, j)] = current_points[(0, j)] - cm_x;
        c_shift[(1, j)] = current_points[(1, j)] - cm_y;
    }

    // Calculate cross-covariance matrix W = c_shift * p_shift^T
    let w = &c_shift * p_shift.transpose();

    // SVD decomposition
    let svd = w.svd(true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();

    // Calculate rotation: R = V * U^T
    let r = &v_t.transpose() * &u.transpose();

    // Calculate translation: t = pm - R * cm
    let cm_vec = DVector::from_vec(vec![cm_x, cm_y]);
    let pm_vec = DVector::from_vec(vec![pm_x, pm_y]);
    let r_cm = &r * &cm_vec;
    let t = &pm_vec - &r_cm;

    (r, t)
}

/// Generate random 2D point cloud for testing
pub fn generate_2d_points(n_points: usize, field_length: f64) -> DMatrix<f64> {
    let mut rng = rand::thread_rng();
    let mut points = DMatrix::zeros(2, n_points);

    for j in 0..n_points {
        points[(0, j)] = (rng.gen::<f64>() - 0.5) * field_length;
        points[(1, j)] = (rng.gen::<f64>() - 0.5) * field_length;
    }

    points
}

/// Apply 2D transformation to points
pub fn apply_2d_transformation(
    points: &DMatrix<f64>,
    translation: &Vector2<f64>,
    rotation_angle: f64,
) -> DMatrix<f64> {
    let cos_theta = rotation_angle.cos();
    let sin_theta = rotation_angle.sin();
    let rotation = Matrix2::new(cos_theta, -sin_theta, sin_theta, cos_theta);

    let mut transformed = DMatrix::zeros(2, points.ncols());

    for j in 0..points.ncols() {
        let point = Vector2::new(points[(0, j)], points[(1, j)]);
        let rotated = rotation * point;
        transformed[(0, j)] = rotated.x + translation.x;
        transformed[(1, j)] = rotated.y + translation.y;
    }

    transformed
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;
    use rand::{rngs::StdRng, Rng, SeedableRng};

    fn generate_seeded_2d_points(n_points: usize, field_length: f64, seed: u64) -> DMatrix<f64> {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut points = DMatrix::zeros(2, n_points);

        for j in 0..n_points {
            points[(0, j)] = (rng.gen::<f64>() - 0.5) * field_length;
            points[(1, j)] = (rng.gen::<f64>() - 0.5) * field_length;
        }

        points
    }

    #[test]
    fn test_icp_simple_translation() {
        let n_points = 50;
        let previous_points = generate_seeded_2d_points(n_points, 20.0, 7);
        let translation = Vector2::new(1.0, 2.0);
        let current_points = apply_2d_transformation(&previous_points, &translation, 0.0);

        let result = icp_matching(&previous_points, &current_points);

        assert!(result.converged);
        assert!((result.translation[0] + translation.x).abs() < 0.1);
        assert!((result.translation[1] + translation.y).abs() < 0.1);
    }

    #[test]
    fn test_icp_rotation_and_translation() {
        let n_points = 50;
        let previous_points = generate_seeded_2d_points(n_points, 20.0, 42);
        let translation = Vector2::new(0.5, 1.5);
        let rotation_angle = 0.2; // ~11.5 degrees
        let current_points =
            apply_2d_transformation(&previous_points, &translation, rotation_angle);

        let result = icp_matching(&previous_points, &current_points);

        assert!(result.converged);
        assert!(result.final_error_mean < 0.5);
    }
}
