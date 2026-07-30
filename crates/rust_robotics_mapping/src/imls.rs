//! Implicit Moving Least Squares (IMLS) surfaces for 2D point sets.

use nalgebra::{Matrix2, SymmetricEigen, Vector2};

/// Local IMLS surface evaluation and first-order projection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImlsProjection {
    pub signed_distance: f64,
    pub normal: Vector2<f64>,
    pub projected_point: Vector2<f64>,
    pub support_count: usize,
}

/// Estimates a PCA normal around every sample using neighbors within `radius`.
pub fn estimate_normals_2d(
    samples: &[Vector2<f64>],
    radius: f64,
    min_neighbors: usize,
) -> Vec<Option<Vector2<f64>>> {
    let radius_squared = radius * radius;
    samples
        .iter()
        .map(|sample| {
            let neighbors = samples
                .iter()
                .filter(|candidate| (*candidate - sample).norm_squared() <= radius_squared)
                .copied()
                .collect::<Vec<_>>();
            if neighbors.len() < min_neighbors.max(2) {
                return None;
            }
            let center = neighbors.iter().copied().sum::<Vector2<f64>>() / neighbors.len() as f64;
            let covariance = neighbors.iter().fold(Matrix2::zeros(), |sum, point| {
                let centered = point - center;
                sum + centered * centered.transpose()
            }) / neighbors.len() as f64;
            let eigen = SymmetricEigen::new(covariance);
            let index = if eigen.eigenvalues[0] <= eigen.eigenvalues[1] {
                0
            } else {
                1
            };
            let mut normal = eigen.eigenvectors.column(index).into_owned().normalize();
            // Deterministic orientation makes signed distances reproducible.
            if normal.y < 0.0 || (normal.y.abs() < 1.0e-12 && normal.x < 0.0) {
                normal = -normal;
            }
            Some(normal)
        })
        .collect()
}

/// Evaluates and projects onto an IMLS surface.
pub fn project_to_imls_2d(
    query: Vector2<f64>,
    samples: &[Vector2<f64>],
    normals: &[Option<Vector2<f64>>],
    bandwidth: f64,
    support_radius: f64,
) -> Option<ImlsProjection> {
    if samples.len() != normals.len() || bandwidth <= 0.0 || support_radius <= 0.0 {
        return None;
    }
    let radius_squared = support_radius * support_radius;
    let bandwidth_squared = bandwidth * bandwidth;
    let mut weight_sum = 0.0;
    let mut distance_sum = 0.0;
    let mut gradient = Vector2::zeros();
    let mut support_count = 0;

    for (sample, normal) in samples.iter().zip(normals) {
        let Some(normal) = normal else {
            continue;
        };
        let displacement = query - sample;
        let squared_distance = displacement.norm_squared();
        if squared_distance > radius_squared {
            continue;
        }
        let weight = (-squared_distance / bandwidth_squared).exp();
        weight_sum += weight;
        distance_sum += weight * displacement.dot(normal);
        gradient += weight * normal;
        support_count += 1;
    }
    if weight_sum <= f64::EPSILON || support_count == 0 {
        return None;
    }
    let signed_distance = distance_sum / weight_sum;
    let gradient = gradient / weight_sum;
    let gradient_norm_squared = gradient.norm_squared();
    if gradient_norm_squared <= 1.0e-12 {
        return None;
    }
    let normal = gradient / gradient_norm_squared.sqrt();
    let projected_point = query - gradient * (signed_distance / gradient_norm_squared);
    Some(ImlsProjection {
        signed_distance,
        normal,
        projected_point,
        support_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_normals_and_projection_are_correct() {
        let samples = (-10..=10)
            .map(|index| Vector2::new(index as f64 * 0.1, 0.0))
            .collect::<Vec<_>>();
        let normals = estimate_normals_2d(&samples, 0.35, 3);
        let projection =
            project_to_imls_2d(Vector2::new(0.1, 0.25), &samples, &normals, 0.3, 0.5).unwrap();
        assert!((projection.signed_distance - 0.25).abs() < 1.0e-9);
        assert!(projection.projected_point.y.abs() < 1.0e-9);
        assert!(projection.normal.y > 0.99);
    }

    #[test]
    fn projection_requires_local_support() {
        let samples = vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)];
        let normals = vec![Some(Vector2::y()), Some(Vector2::y())];
        assert!(
            project_to_imls_2d(Vector2::new(10.0, 10.0), &samples, &normals, 1.0, 1.0).is_none()
        );
    }
}
