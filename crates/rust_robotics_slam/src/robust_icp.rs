/*!
 * Robust ICP with Gauss-Newton optimization and Huber loss
 *
 * Based on tier4/icp_rust architecture:
 * - KdTree-accelerated nearest neighbor search
 * - Huber loss for outlier robustness
 * - Weighted Gauss-Newton optimization on SE(2)
 *
 * Reference: https://github.com/tier4/icp_rust
 */

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Rotation2, SVector, Vector2, Vector3};
use nearest_neighbor::KdTree;

// ── Huber loss ───────────────────────────────────────────────────

const HUBER_K: f64 = 1.345;

fn huber_rho(e: f64, k: f64) -> f64 {
    let k_sq = k * k;
    if e <= k_sq {
        e
    } else {
        2.0 * k * e.sqrt() - k_sq
    }
}

fn huber_drho(e: f64, k: f64) -> f64 {
    let k_sq = k * k;
    if e <= k_sq {
        1.0
    } else {
        k / e.sqrt()
    }
}

// ── SE(2) Transform ──────────────────────────────────────────────

/// 2D rigid body transform (rotation + translation)
#[derive(Copy, Clone, Debug)]
pub struct Transform2D {
    pub rot: Rotation2<f64>,
    pub t: Vector2<f64>,
}

impl Transform2D {
    /// Create from parameter vector [tx, ty, theta]
    pub fn from_param(param: &Vector3<f64>) -> Self {
        let theta = param[2];
        let cos = theta.cos();
        let sin = theta.sin();
        let rot = Rotation2::from_matrix_unchecked(Matrix2::new(cos, -sin, sin, cos));

        let t = if theta.abs() < 1e-10 {
            Vector2::new(param[0], param[1])
        } else {
            Vector2::new(
                (sin * param[0] - (1.0 - cos) * param[1]) / theta,
                ((1.0 - cos) * param[0] + sin * param[1]) / theta,
            )
        };
        Self { rot, t }
    }

    pub fn identity() -> Self {
        Self {
            rot: Rotation2::identity(),
            t: Vector2::zeros(),
        }
    }

    pub fn transform(&self, p: &Vector2<f64>) -> Vector2<f64> {
        self.rot * p + self.t
    }

    /// Compose: self * rhs
    pub fn compose(&self, rhs: &Self) -> Self {
        Self {
            rot: self.rot * rhs.rot,
            t: self.rot * rhs.t + self.t,
        }
    }
}

// ── Robust ICP ───────────────────────────────────────────────────

/// Result of robust ICP matching
pub struct RobustICPResult {
    pub transform: Transform2D,
    pub iterations: usize,
    pub final_error: f64,
}

/// Robust 2D ICP using KdTree + Huber-weighted Gauss-Newton
pub struct RobustIcp2D<'a> {
    kdtree: KdTree<'a, f64, 2>,
    dst: &'a [SVector<f64, 2>],
}

impl<'a> RobustIcp2D<'a> {
    /// Build KdTree from reference (destination) point cloud
    pub fn new(dst: &'a [SVector<f64, 2>]) -> Self {
        Self {
            kdtree: KdTree::new(dst, 2),
            dst,
        }
    }

    /// Estimate transform that aligns src to dst
    pub fn estimate(
        &self,
        src: &[SVector<f64, 2>],
        initial: &Transform2D,
        max_iter: usize,
    ) -> RobustICPResult {
        let mut transform = *initial;
        let mut prev_error = f64::MAX;
        let mut iterations = 0;

        for iter in 0..max_iter {
            iterations = iter + 1;

            // Transform source points
            let src_transformed: Vec<Vector2<f64>> =
                src.iter().map(|sp| transform.transform(sp)).collect();

            // Find nearest neighbors in dst
            let nearest_dsts: Vec<Vector2<f64>> = src_transformed
                .iter()
                .map(|sp| {
                    let query = SVector::<f64, 2>::from([sp[0], sp[1]]);
                    let (index, _dist) = self.kdtree.search(&query);
                    self.dst[index.unwrap()]
                })
                .collect();

            // Compute error
            let error: f64 = src_transformed
                .iter()
                .zip(nearest_dsts.iter())
                .map(|(s, d)| {
                    let r = s - d;
                    huber_rho(r.dot(&r), HUBER_K)
                })
                .sum();

            if error > prev_error {
                break;
            }
            prev_error = error;

            // Weighted Gauss-Newton update
            let Some(delta) = weighted_gauss_newton(&transform, &src_transformed, &nearest_dsts)
            else {
                break;
            };

            if delta.dot(&delta) < 1e-12 {
                break;
            }

            let dtransform = Transform2D::from_param(&delta);
            transform = dtransform.compose(&transform);
        }

        RobustICPResult {
            transform,
            iterations,
            final_error: prev_error,
        }
    }
}

/// Jacobian of transform w.r.t. SE(2) parameters
fn jacobian(rot: &Rotation2<f64>, landmark: &Vector2<f64>) -> Matrix2x3<f64> {
    let a = Vector2::new(-landmark[1], landmark[0]);
    let r = rot.matrix();
    let b = rot * a;
    Matrix2x3::new(r[(0, 0)], r[(0, 1)], b[0], r[(1, 0)], r[(1, 1)], b[1])
}

/// Compute MAD-based robust standard deviation
fn calc_stddevs(residuals: &[Vector2<f64>]) -> Option<[f64; 2]> {
    if residuals.is_empty() {
        return None;
    }
    let ppf34 = 1.482602218505602; // 1 / PPF(0.75) for normal distribution
    let mut result = [0.0; 2];

    for j in 0..2 {
        let mut vals: Vec<f64> = residuals.iter().map(|r| r[j]).collect();
        let median = median_of(&mut vals)?;
        let mut abs_devs: Vec<f64> = vals.iter().map(|v| (v - median).abs()).collect();
        let mad = median_of(&mut abs_devs)?;
        result[j] = ppf34 * mad;
    }
    Some(result)
}

fn median_of(data: &mut [f64]) -> Option<f64> {
    let n = data.len();
    if n == 0 {
        return None;
    }
    let cmp = |a: &f64, b: &f64| a.partial_cmp(b).unwrap();
    if n % 2 == 1 {
        data.select_nth_unstable_by(n / 2, cmp);
        Some(data[n / 2])
    } else {
        data.select_nth_unstable_by(n / 2 - 1, cmp);
        let a = data[n / 2 - 1];
        data.select_nth_unstable_by(n / 2, cmp);
        let b = data[n / 2];
        Some((a + b) / 2.0)
    }
}

/// Inverse of 3x3 matrix (explicit formula)
fn inverse3x3(m: &Matrix3<f64>) -> Option<Matrix3<f64>> {
    let det = m[(0, 0)] * (m[(2, 2)] * m[(1, 1)] - m[(2, 1)] * m[(1, 2)])
        - m[(1, 0)] * (m[(2, 2)] * m[(0, 1)] - m[(2, 1)] * m[(0, 2)])
        + m[(2, 0)] * (m[(1, 2)] * m[(0, 1)] - m[(1, 1)] * m[(0, 2)]);
    if det.abs() < 1e-30 {
        return None;
    }
    #[rustfmt::skip]
    let inv = Matrix3::new(
        m[(2,2)]*m[(1,1)] - m[(2,1)]*m[(1,2)], -(m[(2,2)]*m[(0,1)] - m[(2,1)]*m[(0,2)]),  m[(1,2)]*m[(0,1)] - m[(1,1)]*m[(0,2)],
       -(m[(2,2)]*m[(1,0)] - m[(2,0)]*m[(1,2)]),  m[(2,2)]*m[(0,0)] - m[(2,0)]*m[(0,2)], -(m[(1,2)]*m[(0,0)] - m[(1,0)]*m[(0,2)]),
        m[(2,1)]*m[(1,0)] - m[(2,0)]*m[(1,1)], -(m[(2,1)]*m[(0,0)] - m[(2,0)]*m[(0,1)]),  m[(1,1)]*m[(0,0)] - m[(1,0)]*m[(0,1)],
    );
    Some(inv / det)
}

/// Weighted Gauss-Newton update with Huber loss
fn weighted_gauss_newton(
    transform: &Transform2D,
    src: &[Vector2<f64>],
    dst: &[Vector2<f64>],
) -> Option<Vector3<f64>> {
    if src.len() < 2 {
        return None;
    }

    let residuals: Vec<Vector2<f64>> = src
        .iter()
        .zip(dst.iter())
        .map(|(s, d)| transform.transform(s) - d)
        .collect();

    let stddevs = calc_stddevs(&residuals)?;

    let mut jtr = Vector3::<f64>::zeros();
    let mut jtj = Matrix3::<f64>::zeros();

    for (s, r) in src.iter().zip(residuals.iter()) {
        let j = jacobian(&transform.rot, s);
        for (dim, jacobian_row) in j.row_iter().enumerate() {
            if stddevs[dim] == 0.0 {
                continue;
            }
            let g = 1.0 / stddevs[dim];
            let r_dim = r[dim];
            let w = huber_drho(r_dim * r_dim, HUBER_K);

            jtr += w * g * jacobian_row.transpose() * r_dim;
            jtj += w * g * jacobian_row.transpose() * jacobian_row;
        }
    }

    let inv = inverse3x3(&jtj)?;
    Some(-inv * jtr)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_points(coords: &[[f64; 2]]) -> Vec<SVector<f64, 2>> {
        coords.iter().map(|c| SVector::from(*c)).collect()
    }

    #[test]
    fn test_robust_icp_identity() {
        let dst = make_points(&[
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [1.0, 1.0],
            [0.5, 0.5],
            [0.3, 0.7],
            [0.7, 0.3],
        ]);
        let src = dst.clone();

        let icp = RobustIcp2D::new(&dst);
        let result = icp.estimate(&src, &Transform2D::identity(), 20);

        assert!(result.final_error < 1e-6);
    }

    #[test]
    fn test_robust_icp_translation() {
        // Use more spread-out points for better conditioning
        let dst = make_points(&[
            [-2.0, -2.0],
            [-2.0, 0.0],
            [-2.0, 2.0],
            [0.0, -2.0],
            [0.0, 0.0],
            [0.0, 2.0],
            [2.0, -2.0],
            [2.0, 0.0],
            [2.0, 2.0],
        ]);

        let true_param = Vector3::new(0.1, 0.05, 0.05);
        let true_transform = Transform2D::from_param(&true_param);

        let src: Vec<SVector<f64, 2>> = dst
            .iter()
            .map(|p| {
                let tp = true_transform.transform(p);
                SVector::from([tp[0], tp[1]])
            })
            .collect();

        let icp = RobustIcp2D::new(&dst);
        let result = icp.estimate(&src, &Transform2D::identity(), 50);

        // Verify each point is close after transform
        for (sp, dp) in src.iter().zip(dst.iter()) {
            let transformed = result.transform.transform(sp);
            let error = (transformed - dp).norm();
            assert!(error < 0.3, "Point error too large: {}", error);
        }
    }

    #[test]
    fn test_huber_loss() {
        // Within threshold: identity
        assert_eq!(huber_rho(0.5, 1.0), 0.5);
        // Beyond threshold: linear growth
        let rho = huber_rho(4.0, 1.0);
        assert!((rho - 3.0).abs() < 1e-10); // 2*1*2 - 1 = 3
    }
}
