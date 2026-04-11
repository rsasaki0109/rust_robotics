//! DBSCAN (Density-Based Spatial Clustering of Applications with Noise) for 2D point clouds.
//!
//! Based on the work by Atsushi Sakai (@Atsushi_twi).

/// Configuration for the DBSCAN algorithm.
#[derive(Debug, Clone)]
pub struct DBSCANConfig {
    /// Neighborhood radius \[m\]. Points within this distance are considered neighbors.
    pub eps: f64,
    /// Minimum number of neighbors (including the point itself) required to form a core point.
    pub min_points: usize,
}

impl Default for DBSCANConfig {
    fn default() -> Self {
        Self {
            eps: 1.0,
            min_points: 5,
        }
    }
}

/// Result of DBSCAN clustering.
#[derive(Debug, Clone)]
pub struct DBSCANResult {
    /// X-coordinates of the input points.
    pub x: Vec<f64>,
    /// Y-coordinates of the input points.
    pub y: Vec<f64>,
    /// Cluster label for each point. `None` means the point is noise.
    pub labels: Vec<Option<usize>>,
    /// Number of clusters found (excluding noise).
    pub n_clusters: usize,
}

impl DBSCANResult {
    /// Return the (x, y) coordinates of points assigned to the given cluster label.
    pub fn get_cluster_points(&self, label: usize) -> (Vec<f64>, Vec<f64>) {
        let mut cx = Vec::new();
        let mut cy = Vec::new();
        for (i, &l) in self.labels.iter().enumerate() {
            if l == Some(label) {
                cx.push(self.x[i]);
                cy.push(self.y[i]);
            }
        }
        (cx, cy)
    }

    /// Return the (x, y) coordinates of noise points (label `None`).
    pub fn get_noise_points(&self) -> (Vec<f64>, Vec<f64>) {
        let mut nx = Vec::new();
        let mut ny = Vec::new();
        for (i, &l) in self.labels.iter().enumerate() {
            if l.is_none() {
                nx.push(self.x[i]);
                ny.push(self.y[i]);
            }
        }
        (nx, ny)
    }
}

/// Run DBSCAN clustering on 2D points.
///
/// # Arguments
/// * `x` - X-coordinates of data points.
/// * `y` - Y-coordinates of data points.
/// * `config` - Algorithm configuration (`eps`, `min_points`).
///
/// # Panics
/// Panics if `x` and `y` have different lengths.
pub fn dbscan_clustering(x: &[f64], y: &[f64], config: &DBSCANConfig) -> DBSCANResult {
    assert_eq!(x.len(), y.len(), "x and y must have the same length");

    let n = x.len();
    let eps2 = config.eps * config.eps;

    // None = unvisited; will be set to Some(cluster_id) or remain None (noise).
    let mut labels: Vec<Option<usize>> = vec![None; n];
    // Track whether a point has been visited.
    let mut visited = vec![false; n];
    let mut cluster_id = 0usize;

    for i in 0..n {
        if visited[i] {
            continue;
        }
        visited[i] = true;

        let neighbors = region_query(x, y, i, eps2);

        if neighbors.len() < config.min_points {
            // Mark as noise for now; may be absorbed into a cluster later.
            // labels[i] stays None.
        } else {
            expand_cluster(
                x,
                y,
                &mut labels,
                &mut visited,
                i,
                &neighbors,
                cluster_id,
                config.min_points,
                eps2,
            );
            cluster_id += 1;
        }
    }

    DBSCANResult {
        x: x.to_vec(),
        y: y.to_vec(),
        labels,
        n_clusters: cluster_id,
    }
}

/// Return indices of all points within `eps^2` squared distance of point `idx`.
fn region_query(x: &[f64], y: &[f64], idx: usize, eps2: f64) -> Vec<usize> {
    let mut neighbors = Vec::new();
    let px = x[idx];
    let py = y[idx];
    for j in 0..x.len() {
        let dx = x[j] - px;
        let dy = y[j] - py;
        if dx * dx + dy * dy <= eps2 {
            neighbors.push(j);
        }
    }
    neighbors
}

/// Grow a cluster starting from `core_idx` using the provided initial neighbor set.
#[allow(clippy::too_many_arguments)]
fn expand_cluster(
    x: &[f64],
    y: &[f64],
    labels: &mut [Option<usize>],
    visited: &mut [bool],
    core_idx: usize,
    initial_neighbors: &[usize],
    cluster_id: usize,
    min_points: usize,
    eps2: f64,
) {
    labels[core_idx] = Some(cluster_id);

    let mut queue: Vec<usize> = initial_neighbors.to_vec();
    let mut head = 0;

    while head < queue.len() {
        let q = queue[head];
        head += 1;

        if !visited[q] {
            visited[q] = true;
            let q_neighbors = region_query(x, y, q, eps2);
            if q_neighbors.len() >= min_points {
                for &nb in &q_neighbors {
                    if !visited[nb] {
                        queue.push(nb);
                    }
                }
            }
        }

        if labels[q].is_none() {
            labels[q] = Some(cluster_id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_cluster(cx: f64, cy: f64, n: usize, spread: f64) -> (Vec<f64>, Vec<f64>) {
        // Deterministic points arranged in a grid-like pattern around (cx, cy).
        let side = (n as f64).sqrt().ceil() as usize;
        let step = spread / side.max(1) as f64;
        let mut xs = Vec::with_capacity(n);
        let mut ys = Vec::with_capacity(n);
        'outer: for row in 0..side {
            for col in 0..side {
                xs.push(cx + col as f64 * step);
                ys.push(cy + row as f64 * step);
                if xs.len() == n {
                    break 'outer;
                }
            }
        }
        (xs, ys)
    }

    #[test]
    fn test_two_separated_clusters() {
        let (mut x, mut y) = make_cluster(0.0, 0.0, 10, 0.5);
        let (x2, y2) = make_cluster(100.0, 100.0, 10, 0.5);
        x.extend(x2);
        y.extend(y2);

        let config = DBSCANConfig {
            eps: 2.0,
            min_points: 3,
        };
        let result = dbscan_clustering(&x, &y, &config);

        assert_eq!(result.n_clusters, 2);
        assert_eq!(result.labels.len(), 20);

        // All points should be assigned to one of the two clusters.
        for &l in &result.labels {
            assert!(l.is_some(), "expected no noise in well-separated clusters");
        }

        // First 10 points belong to one cluster, last 10 to another.
        let first_label = result.labels[0];
        for &l in &result.labels[..10] {
            assert_eq!(l, first_label);
        }
        let second_label = result.labels[10];
        assert_ne!(first_label, second_label);
        for &l in &result.labels[10..] {
            assert_eq!(l, second_label);
        }
    }

    #[test]
    fn test_noise_detection() {
        // Dense cluster + isolated outliers.
        let (mut x, mut y) = make_cluster(0.0, 0.0, 10, 0.4);
        // Three isolated outliers far away.
        x.extend([50.0, 51.0, 200.0]);
        y.extend([50.0, 51.0, 200.0]);

        let config = DBSCANConfig {
            eps: 1.0,
            min_points: 5,
        };
        let result = dbscan_clustering(&x, &y, &config);

        let (noise_x, _) = result.get_noise_points();
        // At least the lone outlier at (200, 200) should be noise.
        assert!(!noise_x.is_empty(), "expected some noise points");
    }

    #[test]
    fn test_single_cluster() {
        let (x, y) = make_cluster(5.0, 5.0, 15, 0.5);

        let config = DBSCANConfig {
            eps: 2.0,
            min_points: 3,
        };
        let result = dbscan_clustering(&x, &y, &config);

        assert_eq!(result.n_clusters, 1);
        for &l in &result.labels {
            assert_eq!(l, Some(0), "all points should be in cluster 0");
        }
    }

    #[test]
    fn test_all_noise_high_min_points() {
        // Spread points far apart so no point has enough neighbors.
        let x: Vec<f64> = (0..10).map(|i| i as f64 * 100.0).collect();
        let y: Vec<f64> = vec![0.0; 10];

        let config = DBSCANConfig {
            eps: 1.0,
            min_points: 5,
        };
        let result = dbscan_clustering(&x, &y, &config);

        assert_eq!(result.n_clusters, 0);
        for &l in &result.labels {
            assert!(l.is_none(), "all points should be noise");
        }
    }

    #[test]
    fn test_get_cluster_points_correctness() {
        // Two tiny well-separated clusters.
        let x = vec![0.0, 0.1, 0.2, 10.0, 10.1, 10.2];
        let y = vec![0.0, 0.1, 0.2, 10.0, 10.1, 10.2];

        let config = DBSCANConfig {
            eps: 0.5,
            min_points: 2,
        };
        let result = dbscan_clustering(&x, &y, &config);

        assert_eq!(result.n_clusters, 2);

        let (c0x, c0y) = result.get_cluster_points(0);
        let (c1x, c1y) = result.get_cluster_points(1);

        assert_eq!(c0x.len(), c0y.len());
        assert_eq!(c1x.len(), c1y.len());
        assert_eq!(c0x.len() + c1x.len(), 6, "all points must be in a cluster");

        let (nx, _) = result.get_noise_points();
        assert!(nx.is_empty(), "no noise expected");
    }
}
