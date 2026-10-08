//! Relative-pose SLAM metric of Kümmerle et al., "On measuring the accuracy of
//! SLAM algorithms" (Autonomous Robots, 2009).
//!
//! Instead of comparing a trajectory against a global ground truth, the
//! benchmark provides *relations*: verified relative poses `δ*ᵢⱼ` between the
//! poses at two timestamps. For an estimate the error of each relation is
//! `δᵢⱼ ⊖ δ*ᵢⱼ` with `δᵢⱼ = xᵢ⁻¹ ⊕ xⱼ`, and translational and rotational parts
//! are averaged separately (absolute and squared). The metric is invariant to
//! the global frame, so any SLAM output can be scored.
//!
//! Relations files have one relation per line:
//!
//! ```text
//! timestamp_i timestamp_j x y z roll pitch yaw
//! ```

use rust_robotics_core::Pose2D;

use crate::scan_to_map::relative_pose;

/// A reference relative pose between the poses at two timestamps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Relation {
    pub timestamp_from: f64,
    pub timestamp_to: f64,
    /// Relative pose `x_from⁻¹ ⊕ x_to`.
    pub relative: Pose2D,
}

/// Aggregated relation errors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelationErrors {
    /// Relations whose both timestamps matched a trajectory pose.
    pub matched: usize,
    /// Relations skipped because a timestamp had no pose within tolerance.
    pub unmatched: usize,
    /// Mean absolute translational error \[m\].
    pub translation_mean: f64,
    /// Standard deviation of the absolute translational error \[m\].
    pub translation_std: f64,
    /// Mean squared translational error \[m²\].
    pub translation_mean_squared: f64,
    /// Mean absolute rotational error \[rad\].
    pub rotation_mean: f64,
    /// Standard deviation of the absolute rotational error \[rad\].
    pub rotation_std: f64,
    /// Mean squared rotational error \[rad²\].
    pub rotation_mean_squared: f64,
}

/// Parses a relations file; blank lines and `#` comments are skipped.
pub fn parse_relations(text: &str) -> Result<Vec<Relation>, String> {
    text.lines()
        .enumerate()
        .filter(|(_, line)| {
            let line = line.trim();
            !line.is_empty() && !line.starts_with('#')
        })
        .map(|(index, line)| {
            let values: Vec<f64> = line
                .split_whitespace()
                .map(str::parse::<f64>)
                .collect::<Result<_, _>>()
                .map_err(|error| format!("line {}: {error}", index + 1))?;
            let [t1, t2, x, y, _z, _roll, _pitch, yaw] = values.as_slice() else {
                return Err(format!(
                    "line {}: expected 8 values, found {}",
                    index + 1,
                    values.len()
                ));
            };
            Ok(Relation {
                timestamp_from: *t1,
                timestamp_to: *t2,
                relative: Pose2D::new(*x, *y, *yaw),
            })
        })
        .collect()
}

/// Formats relations in the benchmark file format.
pub fn format_relations(relations: &[Relation]) -> String {
    relations
        .iter()
        .map(|relation| {
            format!(
                "{:.6} {:.6} {:.6} {:.6} 0 0 0 {:.6}\n",
                relation.timestamp_from,
                relation.timestamp_to,
                relation.relative.x,
                relation.relative.y,
                relation.relative.yaw
            )
        })
        .collect()
}

/// Pose of a trajectory sorted by timestamp closest to `timestamp`, if within
/// `tolerance` seconds.
fn pose_at(trajectory: &[(f64, Pose2D)], timestamp: f64, tolerance: f64) -> Option<Pose2D> {
    let index = trajectory.partition_point(|(time, _)| *time < timestamp);
    [index.checked_sub(1), Some(index)]
        .into_iter()
        .flatten()
        .filter_map(|index| trajectory.get(index))
        .min_by(|a, b| (a.0 - timestamp).abs().total_cmp(&(b.0 - timestamp).abs()))
        .filter(|(time, _)| (time - timestamp).abs() <= tolerance)
        .map(|(_, pose)| *pose)
}

fn mean_and_std(values: &[f64]) -> (f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0);
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64;
    (mean, variance.sqrt())
}

/// Scores `trajectory` (timestamped poses, any order) against `relations`.
///
/// A relation is used when both of its timestamps have a pose within
/// `time_tolerance` seconds.
pub fn evaluate_relations(
    trajectory: &[(f64, Pose2D)],
    relations: &[Relation],
    time_tolerance: f64,
) -> RelationErrors {
    let mut sorted = trajectory.to_vec();
    sorted.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut translation = Vec::new();
    let mut rotation = Vec::new();
    for relation in relations {
        let (Some(from), Some(to)) = (
            pose_at(&sorted, relation.timestamp_from, time_tolerance),
            pose_at(&sorted, relation.timestamp_to, time_tolerance),
        ) else {
            continue;
        };
        let error = relative_pose(relation.relative, relative_pose(from, to));
        translation.push(error.x.hypot(error.y));
        rotation.push(error.yaw.abs());
    }

    let (translation_mean, translation_std) = mean_and_std(&translation);
    let (rotation_mean, rotation_std) = mean_and_std(&rotation);
    let squared =
        |values: &[f64]| values.iter().map(|v| v * v).sum::<f64>() / values.len().max(1) as f64;
    RelationErrors {
        matched: translation.len(),
        unmatched: relations.len() - translation.len(),
        translation_mean,
        translation_std,
        translation_mean_squared: squared(&translation),
        rotation_mean,
        rotation_std,
        rotation_mean_squared: squared(&rotation),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scan_to_map::compose_pose;

    fn trajectory() -> Vec<(f64, Pose2D)> {
        let mut pose = Pose2D::origin();
        (0..20)
            .map(|i| {
                pose = compose_pose(pose, Pose2D::new(0.5, 0.0, 0.1));
                (f64::from(i), pose)
            })
            .collect()
    }

    #[test]
    fn exact_trajectory_scores_zero_in_any_frame() {
        let truth = trajectory();
        let relations: Vec<Relation> = (0..15)
            .map(|i| Relation {
                timestamp_from: truth[i].0,
                timestamp_to: truth[i + 5].0,
                relative: relative_pose(truth[i].1, truth[i + 5].1),
            })
            .collect();
        // Same trajectory expressed in a different global frame.
        let offset = Pose2D::new(3.0, -1.0, 0.7);
        let moved: Vec<(f64, Pose2D)> = truth
            .iter()
            .map(|(time, pose)| (time + 0.01, compose_pose(offset, *pose)))
            .collect();
        let errors = evaluate_relations(&moved, &relations, 0.05);
        assert_eq!(errors.matched, 15);
        assert_eq!(errors.unmatched, 0);
        assert!(errors.translation_mean < 1e-9);
        assert!(errors.rotation_mean < 1e-9);
    }

    #[test]
    fn measures_a_known_error_and_skips_unmatched_relations() {
        let truth = trajectory();
        let relations = vec![
            Relation {
                timestamp_from: 0.0,
                timestamp_to: 1.0,
                relative: relative_pose(truth[0].1, truth[1].1),
            },
            Relation {
                timestamp_from: 0.0,
                timestamp_to: 99.0,
                relative: Pose2D::origin(),
            },
        ];
        let mut estimate = truth.clone();
        // Push the second pose 0.3 m forward in its own frame.
        estimate[1].1 = compose_pose(estimate[1].1, Pose2D::new(0.3, 0.0, 0.0));
        let errors = evaluate_relations(&estimate, &relations, 0.1);
        assert_eq!(errors.matched, 1);
        assert_eq!(errors.unmatched, 1);
        assert!((errors.translation_mean - 0.3).abs() < 1e-9);
        assert!((errors.translation_mean_squared - 0.09).abs() < 1e-9);
    }

    #[test]
    fn relations_round_trip_through_the_file_format() {
        let relations = vec![Relation {
            timestamp_from: 1.5,
            timestamp_to: 7.25,
            relative: Pose2D::new(0.25, -1.5, 0.125),
        }];
        let parsed = parse_relations(&format!("# header\n\n{}", format_relations(&relations)))
            .expect("valid file");
        assert_eq!(parsed, relations);
        assert!(parse_relations("1 2 3\n").is_err());
    }
}
