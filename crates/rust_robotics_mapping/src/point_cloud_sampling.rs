//! Point cloud sampling algorithms.
//!
//! Provides three sampling strategies for N-dimensional point clouds:
//! - Voxel point sampling
//! - Farthest point sampling
//! - Poisson disk sampling

use nalgebra::DVector;
use rand::Rng;
use rand::SeedableRng;
use std::collections::HashMap;

/// Voxel point sampling.
///
/// Points that fall within the same voxel grid cell are merged by computing
/// their mean. This produces a downsampled point cloud with at most one
/// point per voxel.
///
/// # Arguments
/// * `points` - Original N-dimensional points.
/// * `voxel_size` - Side length of each voxel cell.
///
/// # Returns
/// A vector of sampled points (one per occupied voxel).
pub fn voxel_point_sampling(points: &[DVector<f64>], voxel_size: f64) -> Vec<DVector<f64>> {
    assert!(voxel_size > 0.0, "voxel_size must be positive");
    if points.is_empty() {
        return Vec::new();
    }

    let dim = points[0].len();
    let mut voxel_map: HashMap<Vec<i64>, Vec<&DVector<f64>>> = HashMap::new();

    for p in points {
        let key: Vec<i64> = (0..dim)
            .map(|d| (p[d] / voxel_size).floor() as i64)
            .collect();
        voxel_map.entry(key).or_default().push(p);
    }

    voxel_map
        .values()
        .map(|group| {
            let mut mean = DVector::zeros(dim);
            for &p in group {
                mean += p;
            }
            mean / group.len() as f64
        })
        .collect()
}

/// Farthest point sampling.
///
/// Iteratively selects the point that is farthest from all previously
/// selected points, producing a well-distributed subset.
///
/// # Arguments
/// * `points` - Original N-dimensional points.
/// * `n_points` - Number of points to sample.
/// * `seed` - Random seed for selecting the initial point.
///
/// # Returns
/// A vector of `n_points` sampled points.
///
/// # Panics
/// Panics if `n_points` is 0 or exceeds the number of input points.
pub fn farthest_point_sampling(
    points: &[DVector<f64>],
    n_points: usize,
    seed: u64,
) -> Vec<DVector<f64>> {
    assert!(n_points > 0, "n_points must be positive");
    assert!(
        n_points <= points.len(),
        "n_points ({}) exceeds input size ({})",
        n_points,
        points.len()
    );

    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let n = points.len();
    let first = rng.gen_range(0..n);

    let mut min_distances = vec![f64::INFINITY; n];
    let mut selected_ids: Vec<usize> = vec![first];

    while selected_ids.len() < n_points {
        let last = *selected_ids.last().unwrap();
        let base = &points[last];

        for i in 0..n {
            let dist = (&points[i] - base).norm();
            if dist < min_distances[i] {
                min_distances[i] = dist;
            }
        }

        // Select the point with maximum minimum distance
        let mut best_idx = 0;
        let mut best_dist = -1.0f64;
        for i in 0..n {
            if !selected_ids.contains(&i) && min_distances[i] > best_dist {
                best_dist = min_distances[i];
                best_idx = i;
            }
        }
        selected_ids.push(best_idx);
    }

    selected_ids.iter().map(|&i| points[i].clone()).collect()
}

/// Poisson disk sampling.
///
/// Randomly selects points while enforcing a minimum distance between
/// all selected points. If the desired number of points cannot be reached
/// within `max_iter` attempts, fewer points are returned.
///
/// # Arguments
/// * `points` - Original N-dimensional points.
/// * `n_points` - Desired number of points to sample.
/// * `min_distance` - Minimum distance between any pair of selected points.
/// * `seed` - Random seed.
/// * `max_iter` - Maximum number of random selection attempts.
///
/// # Returns
/// A vector of sampled points (at most `n_points`).
pub fn poisson_disk_sampling(
    points: &[DVector<f64>],
    n_points: usize,
    min_distance: f64,
    seed: u64,
    max_iter: usize,
) -> Vec<DVector<f64>> {
    if points.is_empty() || n_points == 0 {
        return Vec::new();
    }

    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let n = points.len();
    let first = rng.gen_range(0..n);
    let mut selected_ids: Vec<usize> = vec![first];

    let mut loop_count = 0;
    while selected_ids.len() < n_points && loop_count <= max_iter {
        let candidate = rng.gen_range(0..n);
        let base = &points[candidate];

        let min_dist_to_selected = selected_ids
            .iter()
            .map(|&i| (&points[i] - base).norm())
            .fold(f64::INFINITY, f64::min);

        if min_dist_to_selected >= min_distance {
            selected_ids.push(candidate);
        }
        loop_count += 1;
    }

    selected_ids.iter().map(|&i| points[i].clone()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_3d_point(x: f64, y: f64, z: f64) -> DVector<f64> {
        DVector::from_vec(vec![x, y, z])
    }

    fn make_grid_points() -> Vec<DVector<f64>> {
        let mut pts = Vec::new();
        for x in 0..10 {
            for y in 0..10 {
                for z in 0..2 {
                    pts.push(make_3d_point(x as f64, y as f64, z as f64));
                }
            }
        }
        pts
    }

    #[test]
    fn test_voxel_sampling_reduces_points() {
        let pts = make_grid_points();
        let sampled = voxel_point_sampling(&pts, 3.0);
        assert!(sampled.len() < pts.len());
        assert!(!sampled.is_empty());
    }

    #[test]
    fn test_voxel_sampling_preserves_single_voxel() {
        let pts = vec![
            make_3d_point(0.1, 0.1, 0.1),
            make_3d_point(0.2, 0.2, 0.2),
            make_3d_point(0.3, 0.3, 0.3),
        ];
        let sampled = voxel_point_sampling(&pts, 10.0);
        assert_eq!(sampled.len(), 1);
        // Mean should be (0.2, 0.2, 0.2)
        assert!((sampled[0][0] - 0.2).abs() < 1e-10);
        assert!((sampled[0][1] - 0.2).abs() < 1e-10);
        assert!((sampled[0][2] - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_sampling_empty() {
        let pts: Vec<DVector<f64>> = vec![];
        let sampled = voxel_point_sampling(&pts, 1.0);
        assert!(sampled.is_empty());
    }

    #[test]
    fn test_farthest_point_sampling_count() {
        let pts = make_grid_points();
        let sampled = farthest_point_sampling(&pts, 10, 42);
        assert_eq!(sampled.len(), 10);
    }

    #[test]
    fn test_farthest_point_sampling_spread() {
        let pts = make_grid_points();
        let sampled = farthest_point_sampling(&pts, 5, 42);

        // All pairs should have reasonable distance (not all clustered)
        let mut min_pair_dist = f64::INFINITY;
        for i in 0..sampled.len() {
            for j in (i + 1)..sampled.len() {
                let d = (&sampled[i] - &sampled[j]).norm();
                if d < min_pair_dist {
                    min_pair_dist = d;
                }
            }
        }
        // With 200 points in a 10x10x2 grid, 5 farthest points should be at least 3 apart
        assert!(
            min_pair_dist > 2.0,
            "Minimum pair distance {} is too small",
            min_pair_dist
        );
    }

    #[test]
    #[should_panic]
    fn test_farthest_point_sampling_too_many() {
        let pts = vec![make_3d_point(0.0, 0.0, 0.0)];
        farthest_point_sampling(&pts, 5, 42);
    }

    #[test]
    fn test_poisson_disk_sampling_min_distance() {
        let pts = make_grid_points();
        let min_dist = 2.0;
        let sampled = poisson_disk_sampling(&pts, 20, min_dist, 42, 5000);

        // Verify minimum distance constraint
        for i in 0..sampled.len() {
            for j in (i + 1)..sampled.len() {
                let d = (&sampled[i] - &sampled[j]).norm();
                assert!(
                    d >= min_dist - 1e-10,
                    "Points {} and {} are too close: {}",
                    i,
                    j,
                    d
                );
            }
        }
    }

    #[test]
    fn test_poisson_disk_sampling_returns_at_least_one() {
        let pts = make_grid_points();
        let sampled = poisson_disk_sampling(&pts, 5, 1.0, 42, 1000);
        assert!(!sampled.is_empty());
    }

    #[test]
    fn test_poisson_disk_sampling_empty() {
        let pts: Vec<DVector<f64>> = vec![];
        let sampled = poisson_disk_sampling(&pts, 5, 1.0, 42, 100);
        assert!(sampled.is_empty());
    }
}
