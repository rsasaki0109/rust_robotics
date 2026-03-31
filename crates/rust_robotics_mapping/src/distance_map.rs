//! Distance Map computation using the Euclidean Distance Transform (EDT).
//!
//! Computes unsigned distance fields (UDF) and signed distance fields (SDF)
//! from a 2D boolean obstacle grid.
//!
//! Reference: <https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf>

use nalgebra::DMatrix;

const INF: f64 = 1e20;

/// Compute 1D distance transform in-place under squared Euclidean distance.
///
/// Uses the algorithm from Felzenszwalb & Huttenlocher (2012).
fn dt_1d(d: &mut [f64]) {
    let n = d.len();
    if n == 0 {
        return;
    }
    let mut v = vec![0usize; n + 1];
    let mut z = vec![0.0f64; n + 1];
    let mut k: usize = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;

    for q in 1..n {
        let vk = v[k];
        let mut s = ((d[q] + (q * q) as f64) - (d[vk] + (vk * vk) as f64))
            / (2.0 * q as f64 - 2.0 * vk as f64);
        while s <= z[k] {
            k -= 1;
            let vk2 = v[k];
            s = ((d[q] + (q * q) as f64) - (d[vk2] + (vk2 * vk2) as f64))
                / (2.0 * q as f64 - 2.0 * vk2 as f64);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        if k + 1 < z.len() {
            z[k + 1] = INF;
        }
    }

    k = 0;
    for q in 0..n {
        while k + 1 < z.len() && z[k + 1] < q as f64 {
            k += 1;
        }
        let dx = q as f64 - v[k] as f64;
        d[q] = dx * dx + d[v[k]];
    }
}

/// Compute the unsigned distance field (UDF) from a boolean obstacle grid.
///
/// # Arguments
/// * `obstacles` - A 2D matrix where `true` represents an obstacle cell and
///   `false` represents free space.
///
/// # Returns
/// A matrix of Euclidean distances from each cell to the nearest obstacle.
pub fn compute_udf(obstacles: &DMatrix<bool>) -> DMatrix<f64> {
    let nrows = obstacles.nrows();
    let ncols = obstacles.ncols();

    // Initialize: obstacle cells = 0, free cells = INF
    let mut edt = DMatrix::from_fn(
        nrows,
        ncols,
        |r, c| {
            if obstacles[(r, c)] {
                0.0
            } else {
                INF
            }
        },
    );

    // Transform rows
    for r in 0..nrows {
        let mut row: Vec<f64> = (0..ncols).map(|c| edt[(r, c)]).collect();
        dt_1d(&mut row);
        for c in 0..ncols {
            edt[(r, c)] = row[c];
        }
    }

    // Transform columns
    for c in 0..ncols {
        let mut col: Vec<f64> = (0..nrows).map(|r| edt[(r, c)]).collect();
        dt_1d(&mut col);
        for r in 0..nrows {
            edt[(r, c)] = col[r];
        }
    }

    // Take square root to get Euclidean distance
    edt.map(|v| v.sqrt())
}

/// Compute the signed distance field (SDF) from a boolean obstacle grid.
///
/// Positive values indicate distance to the nearest obstacle (free space),
/// negative values indicate distance to the nearest free space (inside obstacle).
///
/// # Arguments
/// * `obstacles` - A 2D matrix where `true` represents an obstacle cell and
///   `false` represents free space.
///
/// # Returns
/// A matrix of signed Euclidean distances.
pub fn compute_sdf(obstacles: &DMatrix<bool>) -> DMatrix<f64> {
    let udf_obstacle = compute_udf(obstacles);
    let inverted = obstacles.map(|v| !v);
    let udf_free = compute_udf(&inverted);
    udf_obstacle - udf_free
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::DMatrix;

    fn make_obstacles() -> DMatrix<bool> {
        // 5x5 grid with obstacles at specific cells
        #[rustfmt::skip]
        let data: Vec<bool> = vec![
            true,  false, false, false, false,
            false, true,  true,  true,  false,
            false, true,  true,  true,  false,
            false, false, true,  true,  false,
            false, false, true,  false, false,
        ];
        DMatrix::from_row_slice(5, 5, &data)
    }

    #[test]
    fn test_udf_obstacle_cells_are_zero() {
        let obs = make_obstacles();
        let udf = compute_udf(&obs);

        // All obstacle cells should have distance 0
        for r in 0..obs.nrows() {
            for c in 0..obs.ncols() {
                if obs[(r, c)] {
                    assert!(
                        udf[(r, c)].abs() < 1e-10,
                        "Obstacle cell ({}, {}) should be 0, got {}",
                        r,
                        c,
                        udf[(r, c)]
                    );
                }
            }
        }
    }

    #[test]
    fn test_udf_non_negative() {
        let obs = make_obstacles();
        let udf = compute_udf(&obs);

        for r in 0..udf.nrows() {
            for c in 0..udf.ncols() {
                assert!(
                    udf[(r, c)] >= 0.0,
                    "UDF should be non-negative at ({}, {}), got {}",
                    r,
                    c,
                    udf[(r, c)]
                );
            }
        }
    }

    #[test]
    fn test_udf_free_cells_positive() {
        let obs = make_obstacles();
        let udf = compute_udf(&obs);

        for r in 0..obs.nrows() {
            for c in 0..obs.ncols() {
                if !obs[(r, c)] {
                    assert!(
                        udf[(r, c)] > 0.0,
                        "Free cell ({}, {}) should have positive distance",
                        r,
                        c
                    );
                }
            }
        }
    }

    #[test]
    fn test_sdf_sign_convention() {
        let obs = make_obstacles();
        let sdf = compute_sdf(&obs);

        // Obstacle cells: UDF=0, so SDF = 0 - udf_free <= 0
        for r in 0..obs.nrows() {
            for c in 0..obs.ncols() {
                if obs[(r, c)] {
                    assert!(
                        sdf[(r, c)] <= 0.0,
                        "Obstacle cell ({}, {}) should have non-positive SDF, got {}",
                        r,
                        c,
                        sdf[(r, c)]
                    );
                }
            }
        }
    }

    #[test]
    fn test_udf_known_distance() {
        // Single obstacle at (0,0) in a 3x3 grid
        #[rustfmt::skip]
        let data = vec![
            true, false, false,
            false, false, false,
            false, false, false,
        ];
        let obs = DMatrix::from_row_slice(3, 3, &data);
        let udf = compute_udf(&obs);

        assert!((udf[(0, 0)] - 0.0).abs() < 1e-10);
        assert!((udf[(0, 1)] - 1.0).abs() < 1e-10);
        assert!((udf[(1, 0)] - 1.0).abs() < 1e-10);
        // Diagonal: sqrt(2)
        assert!((udf[(1, 1)] - std::f64::consts::SQRT_2).abs() < 1e-10);
    }

    #[test]
    fn test_all_obstacles() {
        let obs = DMatrix::from_element(4, 4, true);
        let udf = compute_udf(&obs);
        for r in 0..4 {
            for c in 0..4 {
                assert!((udf[(r, c)] - 0.0).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_dt_1d_basic() {
        let mut d = vec![0.0, INF, INF, INF];
        dt_1d(&mut d);
        assert!((d[0] - 0.0).abs() < 1e-10);
        assert!((d[1] - 1.0).abs() < 1e-10);
        assert!((d[2] - 4.0).abs() < 1e-10);
        assert!((d[3] - 9.0).abs() < 1e-10);
    }
}
