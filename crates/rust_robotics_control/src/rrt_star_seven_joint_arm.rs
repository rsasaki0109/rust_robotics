//! RRT\* path planner for 7-DOF robotic arms
//!
//! Plans collision-free paths in joint space using RRT\* with:
//! - Random sampling in joint-angle space with configurable limits
//! - Collision checking via forward kinematics + sphere obstacle model
//! - Near-node rewiring for asymptotic optimality
//! - Goal biasing for faster convergence

use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;

use crate::n_joint_arm_3d::NLinkArm3D;

/// Spherical obstacle in 3D workspace.
#[derive(Debug, Clone)]
pub struct SphereObstacle {
    pub center: Vector3<f64>,
    pub radius: f64,
}

/// Configuration for the RRT\* arm planner.
#[derive(Debug, Clone)]
pub struct RRTStarArmConfig {
    /// Number of DOF (typically 7).
    pub num_joints: usize,
    /// Length of each link.
    pub link_lengths: Vec<f64>,
    /// (min, max) angle per joint in radians.
    pub joint_limits: Vec<(f64, f64)>,
    /// Maximum number of tree expansion iterations.
    pub max_iterations: usize,
    /// Maximum joint-space step per extend.
    pub step_size: f64,
    /// Probability of sampling the goal configuration \[0..1\].
    pub goal_bias: f64,
    /// Neighbourhood radius for rewiring.
    pub rewire_radius: f64,
    /// Number of intermediate collision checks per edge.
    pub collision_resolution: usize,
    /// RNG seed for reproducibility.
    pub seed: u64,
}

impl Default for RRTStarArmConfig {
    fn default() -> Self {
        let n = 7;
        Self {
            num_joints: n,
            link_lengths: vec![1.0; n],
            joint_limits: vec![(-std::f64::consts::PI, std::f64::consts::PI); n],
            max_iterations: 5000,
            step_size: 0.3,
            goal_bias: 0.1,
            rewire_radius: 1.0,
            collision_resolution: 10,
            seed: 42,
        }
    }
}

/// Internal tree node.
struct Node {
    angles: Vec<f64>,
    parent: Option<usize>,
    cost: f64,
}

/// Result of a successful RRT\* plan.
pub struct RRTStarArmPath {
    /// Sequence of joint-angle configurations from start to goal.
    pub waypoints: Vec<Vec<f64>>,
    /// Total joint-space cost of the path.
    pub cost: f64,
}

/// RRT\* planner operating in joint space with FK-based collision checking.
pub struct RRTStarArmPlanner {
    config: RRTStarArmConfig,
    obstacles: Vec<SphereObstacle>,
}

impl RRTStarArmPlanner {
    /// Creates a new planner with the given configuration and obstacle set.
    pub fn new(config: RRTStarArmConfig, obstacles: Vec<SphereObstacle>) -> Self {
        Self { config, obstacles }
    }

    /// Plans a collision-free path from `start` to `goal` joint configurations.
    ///
    /// Returns `None` if no path is found within `max_iterations`.
    pub fn plan(&self, start: &[f64], goal: &[f64]) -> Option<RRTStarArmPath> {
        assert_eq!(start.len(), self.config.num_joints);
        assert_eq!(goal.len(), self.config.num_joints);

        if !self.config_collision_free(start) || !self.config_collision_free(goal) {
            return None;
        }

        let mut rng = StdRng::seed_from_u64(self.config.seed);

        let mut tree = vec![Node {
            angles: start.to_vec(),
            parent: None,
            cost: 0.0,
        }];

        let mut best_goal_idx: Option<usize> = None;
        let mut best_goal_cost = f64::INFINITY;
        let goal_threshold = self.config.step_size;

        for _ in 0..self.config.max_iterations {
            // Sample random config or goal with bias
            let sample = if rng.random::<f64>() < self.config.goal_bias {
                goal.to_vec()
            } else {
                self.sample_random(&mut rng)
            };

            // Find nearest node
            let nearest_idx = self.nearest(&tree, &sample);
            let new_angles = self.steer(&tree[nearest_idx].angles, &sample);

            // Check collision along edge
            if !self.collision_free(&tree[nearest_idx].angles, &new_angles) {
                continue;
            }

            let edge_cost = self.joint_distance(&tree[nearest_idx].angles, &new_angles);
            let mut min_cost = tree[nearest_idx].cost + edge_cost;
            let mut min_parent = nearest_idx;

            // Find near nodes for potential better parent
            let near_indices = self.near_nodes(&tree, &new_angles);
            for &ni in &near_indices {
                let c = tree[ni].cost + self.joint_distance(&tree[ni].angles, &new_angles);
                if c < min_cost && self.collision_free(&tree[ni].angles, &new_angles) {
                    min_cost = c;
                    min_parent = ni;
                }
            }

            // Add new node
            let new_idx = tree.len();
            tree.push(Node {
                angles: new_angles.clone(),
                parent: Some(min_parent),
                cost: min_cost,
            });

            // Rewire nearby nodes through the new node
            for &ni in &near_indices {
                let rewire_cost = min_cost + self.joint_distance(&new_angles, &tree[ni].angles);
                if rewire_cost < tree[ni].cost && self.collision_free(&new_angles, &tree[ni].angles)
                {
                    tree[ni].parent = Some(new_idx);
                    tree[ni].cost = rewire_cost;
                }
            }

            // Check if new node is close to goal
            let dist_to_goal = self.joint_distance(&new_angles, goal);
            if dist_to_goal < goal_threshold {
                let total = min_cost + dist_to_goal;
                if total < best_goal_cost {
                    // Add goal node connected through new node
                    let goal_idx = tree.len();
                    tree.push(Node {
                        angles: goal.to_vec(),
                        parent: Some(new_idx),
                        cost: total,
                    });
                    if self.collision_free(&new_angles, goal) {
                        best_goal_idx = Some(goal_idx);
                        best_goal_cost = total;
                    }
                }
            }
        }

        // Extract path
        let goal_idx = best_goal_idx?;
        let mut path = Vec::new();
        let mut idx = goal_idx;
        loop {
            path.push(tree[idx].angles.clone());
            match tree[idx].parent {
                Some(p) => idx = p,
                None => break,
            }
        }
        path.reverse();

        Some(RRTStarArmPath {
            cost: best_goal_cost,
            waypoints: path,
        })
    }

    /// Samples a random joint configuration within the configured limits.
    fn sample_random(&self, rng: &mut impl Rng) -> Vec<f64> {
        self.config
            .joint_limits
            .iter()
            .map(|&(lo, hi)| rng.random_range(lo..=hi))
            .collect()
    }

    /// Returns the index of the tree node nearest to `sample` in joint space.
    fn nearest(&self, tree: &[Node], sample: &[f64]) -> usize {
        tree.iter()
            .enumerate()
            .map(|(i, n)| (i, self.joint_distance(&n.angles, sample)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap()
            .0
    }

    /// Steers from `from` toward `to`, clamping the step to `step_size`.
    fn steer(&self, from: &[f64], to: &[f64]) -> Vec<f64> {
        let dist = self.joint_distance(from, to);
        if dist <= self.config.step_size {
            return to.to_vec();
        }
        let ratio = self.config.step_size / dist;
        from.iter()
            .zip(to.iter())
            .map(|(&f, &t)| f + ratio * (t - f))
            .collect()
    }

    /// Computes the L2 distance between two joint configurations.
    fn joint_distance(&self, a: &[f64], b: &[f64]) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(&ai, &bi)| (ai - bi).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// Checks whether the straight-line edge in joint space between `from`
    /// and `to` is collision-free by interpolating intermediate configurations.
    fn collision_free(&self, from: &[f64], to: &[f64]) -> bool {
        let n = self.config.collision_resolution;
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let interp: Vec<f64> = from
                .iter()
                .zip(to.iter())
                .map(|(&f, &tt)| f + t * (tt - f))
                .collect();
            if !self.config_collision_free(&interp) {
                return false;
            }
        }
        true
    }

    /// Checks whether a single joint configuration is collision-free by
    /// computing forward kinematics and testing every link segment against
    /// every sphere obstacle.
    fn config_collision_free(&self, angles: &[f64]) -> bool {
        let arm = NLinkArm3D::with_angles(self.config.link_lengths.clone(), angles.to_vec());
        let points = arm.forward_kinematics();

        for obs in &self.obstacles {
            // Check each link segment (between consecutive joint positions)
            for seg in points.windows(2) {
                if segment_sphere_intersects(&seg[0], &seg[1], &obs.center, obs.radius) {
                    return false;
                }
            }
        }
        true
    }

    /// Returns indices of tree nodes within `rewire_radius` of `point`.
    fn near_nodes(&self, tree: &[Node], point: &[f64]) -> Vec<usize> {
        tree.iter()
            .enumerate()
            .filter(|(_, n)| self.joint_distance(&n.angles, point) <= self.config.rewire_radius)
            .map(|(i, _)| i)
            .collect()
    }
}

/// Tests whether a line segment (from `a` to `b`) intersects a sphere
/// centred at `center` with the given `radius`.
///
/// The closest point on the segment to the sphere centre is found by
/// projecting onto the line and clamping to \[0, 1\].
fn segment_sphere_intersects(
    a: &Vector3<f64>,
    b: &Vector3<f64>,
    center: &Vector3<f64>,
    radius: f64,
) -> bool {
    let ab = b - a;
    let ac = center - a;
    let ab_sq = ab.dot(&ab);

    if ab_sq < 1e-12 {
        // Degenerate segment (zero-length link)
        return ac.norm() < radius;
    }

    let t = (ac.dot(&ab) / ab_sq).clamp(0.0, 1.0);
    let closest = a + t * ab;
    (closest - center).norm() < radius
}

#[cfg(test)]
mod tests {
    use super::*;

    /// With no obstacles the planner should find a path quickly.
    #[test]
    fn test_plan_no_obstacles() {
        let config = RRTStarArmConfig {
            num_joints: 7,
            link_lengths: vec![1.0; 7],
            max_iterations: 3000,
            step_size: 0.5,
            goal_bias: 0.15,
            rewire_radius: 1.5,
            collision_resolution: 5,
            seed: 123,
            ..Default::default()
        };

        let planner = RRTStarArmPlanner::new(config, vec![]);
        let start = vec![0.0; 7];
        let goal = vec![0.3, -0.2, 0.1, 0.4, -0.1, 0.2, 0.0];

        let result = planner.plan(&start, &goal);
        assert!(result.is_some(), "Should find a path with no obstacles");
        let path = result.unwrap();
        assert!(
            path.waypoints.len() >= 2,
            "Path must have at least start and goal"
        );
        assert_eq!(path.waypoints.first().unwrap(), &start);
        assert_eq!(path.waypoints.last().unwrap(), &goal);
        assert!(path.cost > 0.0);
    }

    /// Place an obstacle in the workspace and verify the planner still finds
    /// a path (which must avoid the obstacle).
    #[test]
    fn test_plan_with_obstacle() {
        let config = RRTStarArmConfig {
            num_joints: 7,
            link_lengths: vec![1.0; 7],
            max_iterations: 8000,
            step_size: 0.4,
            goal_bias: 0.1,
            rewire_radius: 1.2,
            collision_resolution: 10,
            seed: 77,
            ..Default::default()
        };

        // Place a small sphere obstacle off to the side (not blocking too much)
        let obs = SphereObstacle {
            center: Vector3::new(2.0, 2.0, 0.0),
            radius: 0.5,
        };

        let planner = RRTStarArmPlanner::new(config.clone(), vec![obs.clone()]);
        let start = vec![0.0; 7];
        // Goal: arm rotated to a different configuration
        let goal = vec![0.5, -0.3, 0.5, -0.3, 0.5, -0.3, 0.5];

        let result = planner.plan(&start, &goal);
        assert!(result.is_some(), "Should find a path around the obstacle");
        let path = result.unwrap();

        // Every waypoint along the path must be collision-free.
        for wp in &path.waypoints {
            assert!(
                planner.config_collision_free(wp),
                "Waypoint must not collide with obstacle"
            );
        }

        // Verify the path cost is positive and reasonable
        assert!(path.cost > 0.0, "Path should have positive cost");
        assert!(
            path.waypoints.len() >= 2,
            "Path should have at least start and goal"
        );
    }

    /// Direct test of segment-sphere intersection geometry.
    #[test]
    fn test_collision_check() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(2.0, 0.0, 0.0);
        let center = Vector3::new(1.0, 0.3, 0.0);

        // Sphere radius large enough to touch the segment
        assert!(segment_sphere_intersects(&a, &b, &center, 0.5));

        // Sphere radius too small to touch the segment
        assert!(!segment_sphere_intersects(&a, &b, &center, 0.2));

        // Sphere beyond the segment endpoint
        let far_center = Vector3::new(3.0, 0.0, 0.0);
        assert!(!segment_sphere_intersects(&a, &b, &far_center, 0.5));

        // Sphere before the segment start
        let behind = Vector3::new(-1.0, 0.0, 0.0);
        assert!(!segment_sphere_intersects(&a, &b, &behind, 0.5));

        // Sphere exactly touching (boundary)
        let on_segment = Vector3::new(1.0, 0.0, 0.0);
        assert!(segment_sphere_intersects(&a, &b, &on_segment, 0.1));
    }

    /// Verify that rewiring actually improves cost compared to a planner
    /// with zero rewire radius (effectively no rewiring).
    #[test]
    fn test_rewiring_improves_cost() {
        let base = RRTStarArmConfig {
            num_joints: 7,
            link_lengths: vec![1.0; 7],
            max_iterations: 4000,
            step_size: 0.4,
            goal_bias: 0.1,
            collision_resolution: 5,
            seed: 99,
            ..Default::default()
        };

        let start = vec![0.0; 7];
        let goal = vec![0.5, -0.3, 0.2, 0.4, -0.2, 0.3, 0.1];

        // Planner with rewiring enabled
        let config_rewire = RRTStarArmConfig {
            rewire_radius: 1.5,
            ..base.clone()
        };
        let planner_rewire = RRTStarArmPlanner::new(config_rewire, vec![]);
        let result_rewire = planner_rewire.plan(&start, &goal);

        // Planner with effectively no rewiring
        let config_no_rewire = RRTStarArmConfig {
            rewire_radius: 0.0,
            ..base
        };
        let planner_no = RRTStarArmPlanner::new(config_no_rewire, vec![]);
        let result_no = planner_no.plan(&start, &goal);

        assert!(result_rewire.is_some(), "Rewiring planner should find path");
        assert!(result_no.is_some(), "No-rewire planner should find path");

        let cost_rewire = result_rewire.unwrap().cost;
        let cost_no = result_no.unwrap().cost;

        // With the same seed and enough iterations, rewiring should yield
        // equal or better cost.
        assert!(
            cost_rewire <= cost_no + 1e-9,
            "Rewiring should not increase cost: rewire={cost_rewire}, none={cost_no}"
        );
    }
}
