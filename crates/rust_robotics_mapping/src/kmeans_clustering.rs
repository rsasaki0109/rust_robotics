//! K-means clustering for 2D point clouds.
//!
//! Based on the work by Atsushi Sakai (@Atsushi_twi).

use rand::Rng;
use rand::SeedableRng;

/// Default maximum number of iterations.
pub const DEFAULT_MAX_ITERATIONS: usize = 10;
/// Default cost-change threshold for convergence.
pub const DEFAULT_COST_THRESHOLD: f64 = 0.1;

/// Configuration for the k-means algorithm.
#[derive(Debug, Clone)]
pub struct KMeansConfig {
    /// Number of clusters.
    pub k: usize,
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Convergence threshold on cost change.
    pub cost_threshold: f64,
}

impl KMeansConfig {
    /// Create a new configuration with the given number of clusters and default parameters.
    pub fn new(k: usize) -> Self {
        Self {
            k,
            max_iterations: DEFAULT_MAX_ITERATIONS,
            cost_threshold: DEFAULT_COST_THRESHOLD,
        }
    }

    /// Set maximum iterations.
    #[must_use]
    pub fn with_max_iterations(mut self, max_iterations: usize) -> Self {
        self.max_iterations = max_iterations;
        self
    }

    /// Set convergence cost threshold.
    #[must_use]
    pub fn with_cost_threshold(mut self, cost_threshold: f64) -> Self {
        self.cost_threshold = cost_threshold;
        self
    }
}

/// Result of k-means clustering.
#[derive(Debug, Clone)]
pub struct KMeansResult {
    /// X-coordinates of the input points.
    pub x: Vec<f64>,
    /// Y-coordinates of the input points.
    pub y: Vec<f64>,
    /// Cluster label for each point (0-indexed).
    pub labels: Vec<usize>,
    /// X-coordinates of cluster centroids.
    pub center_x: Vec<f64>,
    /// Y-coordinates of cluster centroids.
    pub center_y: Vec<f64>,
    /// Number of clusters.
    pub k: usize,
}

impl KMeansResult {
    /// Return the (x, y) coordinates of points assigned to the given label.
    pub fn get_cluster_points(&self, label: usize) -> (Vec<f64>, Vec<f64>) {
        let mut cx = Vec::new();
        let mut cy = Vec::new();
        for (i, &l) in self.labels.iter().enumerate() {
            if l == label {
                cx.push(self.x[i]);
                cy.push(self.y[i]);
            }
        }
        (cx, cy)
    }
}

/// Run k-means clustering on 2D points.
///
/// # Arguments
/// * `x` - X-coordinates of data points.
/// * `y` - Y-coordinates of data points.
/// * `config` - Algorithm configuration (k, max iterations, threshold).
///
/// # Panics
/// Panics if `x` and `y` have different lengths, if either is empty,
/// or if `k` is zero or greater than the number of points.
pub fn kmeans_clustering(x: &[f64], y: &[f64], config: &KMeansConfig) -> KMeansResult {
    assert_eq!(x.len(), y.len(), "x and y must have the same length");
    assert!(!x.is_empty(), "input must not be empty");
    assert!(config.k > 0, "k must be positive");
    assert!(
        config.k <= x.len(),
        "k must not exceed the number of points"
    );

    let n = x.len();
    let k = config.k;

    // Use seeded RNG for deterministic results. Seed from data to ensure
    // reproducibility while varying with different inputs.
    let seed = n as u64 ^ k as u64 ^ x[0].to_bits() ^ y[0].to_bits();
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let mut labels = vec![0; n];
    let mut center_x = vec![0.0; k];
    let mut center_y = vec![0.0; k];

    init_centroids(x, y, k, &mut rng, &mut center_x, &mut center_y);

    let mut pre_cost = f64::INFINITY;
    for _ in 0..config.max_iterations {
        let cost = update_labels(x, y, &center_x, &center_y, &mut labels);
        calc_centroids(x, y, &labels, k, &mut center_x, &mut center_y);

        let d_cost = (cost - pre_cost).abs();
        if d_cost < config.cost_threshold {
            break;
        }
        pre_cost = cost;
    }

    KMeansResult {
        x: x.to_vec(),
        y: y.to_vec(),
        labels,
        center_x,
        center_y,
        k,
    }
}

/// Initialize centroids with a deterministic farthest-point spread.
fn init_centroids(
    x: &[f64],
    y: &[f64],
    k: usize,
    rng: &mut rand::rngs::StdRng,
    center_x: &mut [f64],
    center_y: &mut [f64],
) {
    let n = x.len();
    let first = rng.random_range(0..n);
    center_x[0] = x[first];
    center_y[0] = y[first];

    for label in 1..k {
        let mut farthest_index = 0;
        let mut farthest_dist = f64::NEG_INFINITY;
        for i in 0..n {
            let nearest_dist = (0..label)
                .map(|existing| {
                    (x[i] - center_x[existing]).powi(2) + (y[i] - center_y[existing]).powi(2)
                })
                .fold(f64::INFINITY, f64::min);
            if nearest_dist > farthest_dist {
                farthest_dist = nearest_dist;
                farthest_index = i;
            }
        }
        center_x[label] = x[farthest_index];
        center_y[label] = y[farthest_index];
    }
}

/// Compute centroids from current label assignments.
fn calc_centroids(
    x: &[f64],
    y: &[f64],
    labels: &[usize],
    k: usize,
    center_x: &mut [f64],
    center_y: &mut [f64],
) {
    for label in 0..k {
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut count = 0usize;
        for (i, &l) in labels.iter().enumerate() {
            if l == label {
                sx += x[i];
                sy += y[i];
                count += 1;
            }
        }
        if count > 0 {
            center_x[label] = sx / count as f64;
            center_y[label] = sy / count as f64;
        }
    }
}

/// Assign each point to the nearest centroid and return total cost.
fn update_labels(
    x: &[f64],
    y: &[f64],
    center_x: &[f64],
    center_y: &[f64],
    labels: &mut [usize],
) -> f64 {
    let mut cost = 0.0;
    for i in 0..x.len() {
        let mut min_dist = f64::INFINITY;
        let mut min_label = 0;
        for (j, (&cx, &cy)) in center_x.iter().zip(center_y.iter()).enumerate() {
            let dist = ((x[i] - cx).powi(2) + (y[i] - cy).powi(2)).sqrt();
            if dist < min_dist {
                min_dist = dist;
                min_label = j;
            }
        }
        labels[i] = min_label;
        cost += min_dist;
    }
    cost
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a simple point cloud around given centers.
    fn generate_test_data(
        centers: &[(f64, f64)],
        points_per_cluster: usize,
        spread: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let mut x = Vec::new();
        let mut y = Vec::new();
        for &(cx, cy) in centers {
            for _ in 0..points_per_cluster {
                x.push(cx + spread * (rng.random::<f64>() - 0.5));
                y.push(cy + spread * (rng.random::<f64>() - 0.5));
            }
        }
        (x, y)
    }

    #[test]
    fn test_two_clusters_well_separated() {
        let centers = [(0.0, 0.0), (100.0, 100.0)];
        let (x, y) = generate_test_data(&centers, 20, 2.0);

        let config = KMeansConfig::new(2).with_max_iterations(50);
        let result = kmeans_clustering(&x, &y, &config);

        assert_eq!(result.labels.len(), 40);
        assert_eq!(result.k, 2);

        // All points from the first group should share one label,
        // all points from the second group should share another.
        let first_label = result.labels[0];
        for &l in &result.labels[..20] {
            assert_eq!(l, first_label);
        }
        let second_label = result.labels[20];
        assert_ne!(first_label, second_label);
        for &l in &result.labels[20..] {
            assert_eq!(l, second_label);
        }
    }

    #[test]
    fn test_three_clusters() {
        let centers = [(0.0, 0.0), (50.0, 0.0), (25.0, 50.0)];
        let (x, y) = generate_test_data(&centers, 30, 3.0);

        let config = KMeansConfig::new(3).with_max_iterations(50);
        let result = kmeans_clustering(&x, &y, &config);

        assert_eq!(result.labels.len(), 90);
        assert_eq!(result.center_x.len(), 3);
        assert_eq!(result.center_y.len(), 3);

        // Each cluster should have roughly 30 points.
        for label in 0..3 {
            let (cx, cy) = result.get_cluster_points(label);
            assert!(!cx.is_empty(), "cluster {label} should not be empty");
            assert_eq!(cx.len(), cy.len());
        }
    }

    #[test]
    fn test_single_cluster() {
        let (x, y) = generate_test_data(&[(5.0, 5.0)], 10, 1.0);
        let config = KMeansConfig::new(1);
        let result = kmeans_clustering(&x, &y, &config);

        // All points should be in cluster 0.
        for &l in &result.labels {
            assert_eq!(l, 0);
        }

        // Centroid should be near (5, 5).
        assert!((result.center_x[0] - 5.0).abs() < 1.0);
        assert!((result.center_y[0] - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_get_cluster_points() {
        let x = vec![0.0, 1.0, 10.0, 11.0];
        let y = vec![0.0, 1.0, 10.0, 11.0];
        let config = KMeansConfig::new(2).with_max_iterations(50);
        let result = kmeans_clustering(&x, &y, &config);

        let mut total = 0;
        for label in 0..2 {
            let (cx, _cy) = result.get_cluster_points(label);
            total += cx.len();
        }
        assert_eq!(total, 4);
    }

    #[test]
    fn test_convergence_with_cost_threshold() {
        let centers = [(0.0, 0.0), (20.0, 20.0)];
        let (x, y) = generate_test_data(&centers, 15, 1.0);

        let config = KMeansConfig::new(2)
            .with_max_iterations(100)
            .with_cost_threshold(0.01);
        let result = kmeans_clustering(&x, &y, &config);

        // Should still converge correctly.
        assert_eq!(result.labels.len(), 30);
    }

    #[test]
    #[should_panic(expected = "x and y must have the same length")]
    fn test_mismatched_lengths() {
        let config = KMeansConfig::new(2);
        kmeans_clustering(&[1.0, 2.0], &[1.0], &config);
    }

    #[test]
    #[should_panic(expected = "k must be positive")]
    fn test_zero_k() {
        let config = KMeansConfig::new(0);
        kmeans_clustering(&[1.0], &[1.0], &config);
    }
}
