//! 2D LiDAR graph SLAM: scan-to-map odometry plus loop closure.
//!
//! [`ScanToMapMatcher`] keeps local drift small but cannot remove it: once the
//! robot leaves the submap radius, errors are frozen into the trajectory. This
//! module adds the global layer on top of it:
//!
//! 1. **Pose-graph nodes** are created every `node_translation` / `node_yaw`
//!    of front-end motion and keep their body-frame scan. Consecutive nodes are
//!    linked by odometry edges measured by the front end.
//! 2. **Loop detection**: each new node searches older nodes (at least
//!    `loop_min_node_gap` back) within `loop_search_radius` of its current
//!    estimate, and registers its scan against the candidate's neighborhood
//!    map with coarse-to-fine point-to-line ICP.
//! 3. **Verification**: a closure is accepted only if the inlier ratio, mean
//!    residual, and correction size pass the gates; it then becomes a loop
//!    edge and the whole graph is re-optimized with
//!    [`optimize_pose_graph`].
//!
//! The front end keeps running in its own (drifting) frame; only its relative
//! motion between nodes is used, so optimized node poses never disturb the
//! local matcher.

use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use rust_robotics_core::Pose2D;

use crate::pose_graph_optimization::{optimize_pose_graph, Edge2D, Pose2DNode, PoseGraphConfig};
use crate::scan_to_map::{
    compose_pose, register_point_to_line, relative_pose, scan_points_with_normals,
    transform_scan_to_world, translational_observability, ScanToMapConfig, ScanToMapMatcher,
    ScanToMapUpdate,
};

/// Tuning parameters for [`LidarGraphSlam`].
#[derive(Debug, Clone, Copy)]
pub struct LidarGraphSlamConfig {
    /// Local scan-to-map front end.
    pub front_end: ScanToMapConfig,
    /// Front-end translation between pose-graph nodes \[m\].
    pub node_translation: f64,
    /// Front-end rotation between pose-graph nodes \[rad\].
    pub node_yaw: f64,
    /// Older nodes within this distance of the new node are loop candidates \[m\].
    pub loop_search_radius: f64,
    /// Candidates must be at least this many nodes older than the new node.
    pub loop_min_node_gap: usize,
    /// At most this many nearest candidates are verified per new node.
    pub loop_max_candidates: usize,
    /// The candidate's map uses nodes `candidate ± loop_map_neighbors`.
    pub loop_map_neighbors: usize,
    /// Correspondence radii for the coarse-to-fine loop registration \[m\].
    pub loop_correspondence_schedule: [f64; 3],
    /// Minimum fraction of scan points matched at the finest radius.
    pub loop_min_inlier_ratio: f64,
    /// Maximum mean point-to-line residual at the finest radius \[m\].
    pub loop_max_mean_residual: f64,
    /// Maximum loop correction translation relative to the estimate \[m\].
    pub loop_max_correction_translation: f64,
    /// Maximum loop correction rotation relative to the estimate \[rad\].
    pub loop_max_correction_yaw: f64,
    /// Base standard deviations `(xy [m], yaw [rad])` of an odometry edge.
    pub odometry_sigma: (f64, f64),
    /// Odometry translation uncertainty per meter travelled in a direction
    /// the front end cannot observe (no match, or a degenerate match).
    pub odometry_scale_sigma: f64,
    /// Odometry heading uncertainty per meter travelled without any accepted
    /// scan match \[rad/m\].
    pub odometry_yaw_sigma: f64,
    /// A match is treated as fully degenerate along its weakest translation
    /// direction when `λ_min / λ_max` of the translational Hessian is 0, and
    /// fully constrained at or above this ratio (linear in between).
    pub degeneracy_ratio: f64,
    /// Standard deviations `(xy [m], yaw [rad])` of a loop edge.
    pub loop_sigma: (f64, f64),
    /// A new loop edge triggers re-optimization only when it disagrees with
    /// the current graph by more than `(xy [m], yaw [rad])`; consistent
    /// edges are kept and folded in by the next optimization.
    pub reoptimize_threshold: (f64, f64),
    /// Back-end optimizer settings.
    pub pose_graph: PoseGraphConfig,
}

impl Default for LidarGraphSlamConfig {
    fn default() -> Self {
        Self {
            front_end: ScanToMapConfig::default(),
            node_translation: 1.0,
            node_yaw: 0.5,
            loop_search_radius: 4.0,
            loop_min_node_gap: 15,
            loop_max_candidates: 3,
            loop_map_neighbors: 2,
            loop_correspondence_schedule: [1.0, 0.5, 0.2],
            loop_min_inlier_ratio: 0.6,
            loop_max_mean_residual: 0.05,
            loop_max_correction_translation: 3.0,
            loop_max_correction_yaw: 0.6,
            odometry_sigma: (0.005, 0.0005),
            odometry_scale_sigma: 0.05,
            odometry_yaw_sigma: 0.05,
            degeneracy_ratio: 0.05,
            loop_sigma: (0.02, 0.003),
            reoptimize_threshold: (0.02, 0.005),
            pose_graph: PoseGraphConfig {
                max_iterations: 50,
                ..PoseGraphConfig::default()
            },
        }
    }
}

/// An accepted loop closure edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopClosure {
    /// Older node the new node was matched against.
    pub from: usize,
    /// New node.
    pub to: usize,
    /// Measured relative pose `from⁻¹ ⊕ to`.
    pub relative: Pose2D,
    /// Fraction of scan points matched at the finest correspondence radius.
    pub inlier_ratio: f64,
    /// Mean point-to-line residual at the finest radius \[m\].
    pub mean_residual: f64,
}

/// Diagnostics returned by [`LidarGraphSlam::update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarGraphSlamUpdate {
    /// Front-end matcher diagnostics for this scan.
    pub front_end: ScanToMapUpdate,
    /// Best current pose estimate in the graph frame.
    pub pose: Pose2D,
    /// Index of the node created by this scan, if any.
    pub new_node: Option<usize>,
    /// Whether the pose graph was re-optimized during this update.
    pub optimized: bool,
    /// Loop closure accepted for the new node, if any.
    pub loop_closure: Option<LoopClosure>,
}

#[derive(Debug, Clone)]
struct Node {
    /// Optimized pose in the graph frame.
    pose: Pose2D,
    /// Front-end pose when the node was created.
    front_end_pose: Pose2D,
    scan: Vec<Vector2<f64>>,
}

/// Scan-to-map odometry with pose-graph loop closure.
#[derive(Debug, Clone)]
pub struct LidarGraphSlam {
    config: LidarGraphSlamConfig,
    front_end: ScanToMapMatcher,
    nodes: Vec<Node>,
    edges: Vec<Edge2D>,
    loop_closures: Vec<LoopClosure>,
    /// Distance-weighted weak directions `Σ dₖ wₖ vₖ vₖᵀ` (front-end world
    /// frame) travelled since the last node without scan constraints.
    unobserved_translation: Matrix2<f64>,
    /// Distance travelled since the last node without any accepted match.
    unobserved_yaw_distance: f64,
}

impl LidarGraphSlam {
    /// Creates a SLAM instance whose graph frame starts at `initial_pose`.
    pub fn new(config: LidarGraphSlamConfig, initial_pose: Pose2D) -> Self {
        Self {
            config,
            front_end: ScanToMapMatcher::new(config.front_end, initial_pose),
            nodes: Vec::new(),
            edges: Vec::new(),
            loop_closures: Vec::new(),
            unobserved_translation: Matrix2::zeros(),
            unobserved_yaw_distance: 0.0,
        }
    }

    /// Configuration.
    pub fn config(&self) -> &LidarGraphSlamConfig {
        &self.config
    }

    /// Best current pose estimate: last optimized node plus front-end motion since.
    pub fn pose(&self) -> Pose2D {
        match self.nodes.last() {
            Some(node) => compose_pose(
                node.pose,
                relative_pose(node.front_end_pose, self.front_end.pose()),
            ),
            None => self.front_end.pose(),
        }
    }

    /// Front-end (odometry-only) pose, without loop closure.
    pub fn front_end_pose(&self) -> Pose2D {
        self.front_end.pose()
    }

    /// Optimized pose-graph node poses.
    pub fn node_poses(&self) -> Vec<Pose2D> {
        self.nodes.iter().map(|node| node.pose).collect()
    }

    /// Front-end poses recorded when each node was created.
    pub fn node_front_end_poses(&self) -> Vec<Pose2D> {
        self.nodes.iter().map(|node| node.front_end_pose).collect()
    }

    /// Accepted loop closures, oldest first.
    pub fn loop_closures(&self) -> &[LoopClosure] {
        &self.loop_closures
    }

    /// Pose-graph edges (odometry and loop) in insertion order.
    pub fn edges(&self) -> &[Edge2D] {
        &self.edges
    }

    /// All node scans rendered at their optimized poses.
    pub fn map_points(&self) -> Vec<Vector2<f64>> {
        self.nodes
            .iter()
            .flat_map(|node| transform_scan_to_world(&node.scan, node.pose))
            .collect()
    }

    /// Integrates one odometry step and the beam-ordered body-frame scan at its end.
    pub fn update(
        &mut self,
        odom_delta: Pose2D,
        scan_body: &[Vector2<f64>],
    ) -> LidarGraphSlamUpdate {
        let front_end = self.front_end.update(odom_delta, scan_body);
        self.accumulate_unobservable_motion(odom_delta, &front_end);
        let mut new_node = None;
        let mut loop_closure = None;
        let mut optimized = false;

        if self.is_new_node() {
            let index = self.add_node(scan_body);
            new_node = Some(index);
            loop_closure = self.detect_loop(index);
            if let Some(closure) = loop_closure {
                let current =
                    relative_pose(self.nodes[closure.from].pose, self.nodes[closure.to].pose);
                let innovation = relative_pose(current, closure.relative);
                let information = diagonal_information(self.config.loop_sigma);
                self.add_edge(closure.from, closure.to, closure.relative, information);
                self.loop_closures.push(closure);
                let (xy, yaw) = self.config.reoptimize_threshold;
                if innovation.x.hypot(innovation.y) > xy || innovation.yaw.abs() > yaw {
                    self.optimize();
                    optimized = true;
                }
            }
        }

        LidarGraphSlamUpdate {
            front_end,
            pose: self.pose(),
            new_node,
            optimized,
            loop_closure,
        }
    }

    fn is_new_node(&self) -> bool {
        let Some(last) = self.nodes.last() else {
            return true;
        };
        let delta = relative_pose(last.front_end_pose, self.front_end.pose());
        delta.x.hypot(delta.y) >= self.config.node_translation
            || delta.yaw.abs() >= self.config.node_yaw
    }

    fn add_node(&mut self, scan_body: &[Vector2<f64>]) -> usize {
        let front_end_pose = self.front_end.pose();
        let pose = self.pose();
        let index = self.nodes.len();
        self.nodes.push(Node {
            pose,
            front_end_pose,
            scan: scan_body.to_vec(),
        });
        if index > 0 {
            let odometry = relative_pose(self.nodes[index - 1].front_end_pose, front_end_pose);
            let information = self.odometry_information(front_end_pose.yaw);
            self.add_edge(index - 1, index, odometry, information);
        }
        self.unobserved_translation = Matrix2::zeros();
        self.unobserved_yaw_distance = 0.0;
        index
    }

    /// Inflates the pending edge covariance along directions the front end
    /// could not observe during this step.
    fn accumulate_unobservable_motion(&mut self, odom_delta: Pose2D, update: &ScanToMapUpdate) {
        let step = odom_delta.x.hypot(odom_delta.y);
        if step == 0.0 {
            return;
        }
        if !update.status.is_accepted() {
            self.unobserved_translation += Matrix2::identity() * step;
            self.unobserved_yaw_distance += step;
            return;
        }
        let (ratio, direction) = translational_observability(&update.hessian);
        let weakness = (1.0 - ratio / self.config.degeneracy_ratio.max(1.0e-12)).clamp(0.0, 1.0);
        self.unobserved_translation += direction * direction.transpose() * (step * weakness);
    }

    /// Information of the odometry edge ending at a node whose front-end yaw
    /// is `to_yaw`.
    ///
    /// Odometry scale and heading errors are systematic, so unobserved motion
    /// grows the standard deviation linearly with distance:
    /// `Σ_xy = σ_xy² I + κ² M Mᵀ` with `M = Σ dₖ wₖ vₖ vₖᵀ` over the weak
    /// directions `vₖ`. [`optimize_pose_graph`] expresses the translational
    /// error in the `to` frame, so `Σ_xy` is rotated into it.
    fn odometry_information(&self, to_yaw: f64) -> Matrix3<f64> {
        let (xy, yaw) = self.config.odometry_sigma;
        let (sin, cos) = to_yaw.sin_cos();
        let rotation = Matrix2::new(cos, -sin, sin, cos);
        let spread = self.unobserved_translation * self.config.odometry_scale_sigma;
        let covariance = Matrix2::identity() * (xy * xy)
            + rotation.transpose() * spread * spread.transpose() * rotation;
        let translational = covariance
            .try_inverse()
            .unwrap_or_else(|| Matrix2::identity() / (xy * xy));
        let yaw_sigma = yaw + self.config.odometry_yaw_sigma * self.unobserved_yaw_distance;
        let mut information = Matrix3::zeros();
        information
            .fixed_view_mut::<2, 2>(0, 0)
            .copy_from(&translational);
        information[(2, 2)] = 1.0 / (yaw_sigma * yaw_sigma);
        information
    }

    fn add_edge(&mut self, from: usize, to: usize, relative: Pose2D, information: Matrix3<f64>) {
        self.edges.push(Edge2D {
            from,
            to,
            measurement: Pose2DNode::new(relative.x, relative.y, relative.yaw),
            information,
        });
    }

    fn detect_loop(&self, index: usize) -> Option<LoopClosure> {
        if index < self.config.loop_min_node_gap {
            return None;
        }
        let current = self.nodes[index].pose;
        let radius_sq = self.config.loop_search_radius * self.config.loop_search_radius;
        let mut candidates: Vec<(f64, usize)> = self.nodes
            [..=index - self.config.loop_min_node_gap]
            .iter()
            .enumerate()
            .filter_map(|(candidate, node)| {
                let distance_sq =
                    (node.pose.x - current.x).powi(2) + (node.pose.y - current.y).powi(2);
                (distance_sq <= radius_sq).then_some((distance_sq, candidate))
            })
            .collect();
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));

        candidates
            .into_iter()
            .take(self.config.loop_max_candidates)
            .find_map(|(_, candidate)| self.verify_loop(candidate, index))
    }

    fn verify_loop(&self, candidate: usize, index: usize) -> Option<LoopClosure> {
        let front = &self.config.front_end;
        let first = candidate.saturating_sub(self.config.loop_map_neighbors);
        let last = (candidate + self.config.loop_map_neighbors).min(index - 1);
        let mut target = Vec::new();
        let mut normals = Vec::new();
        for node in &self.nodes[first..=last] {
            let (points, node_normals) = scan_points_with_normals(
                &node.scan,
                node.pose,
                front.normal_neighbor_distance,
                front.voxel_size,
            );
            target.extend(points);
            normals.extend(node_normals);
        }
        let source = scan_points_with_normals(
            &self.nodes[index].scan,
            Pose2D::origin(),
            front.normal_neighbor_distance,
            front.voxel_size,
        )
        .0;
        if source.len() < front.min_correspondences || target.len() < front.min_correspondences {
            return None;
        }

        let seed = self.nodes[index].pose;
        let mut pose = seed;
        let mut registration = None;
        for radius in self.config.loop_correspondence_schedule {
            let config = ScanToMapConfig {
                max_correspondence_distance: radius,
                max_iterations: front.max_iterations.max(30),
                ..*front
            };
            let result = register_point_to_line(&source, &target, &normals, pose, &config);
            pose = result.pose;
            registration = Some(result);
        }
        let registration = registration?;
        let inlier_ratio = registration.correspondences as f64 / source.len() as f64;
        let correction = relative_pose(seed, registration.pose);
        let accepted = inlier_ratio >= self.config.loop_min_inlier_ratio
            && registration.mean_residual <= self.config.loop_max_mean_residual
            && correction.x.hypot(correction.y) <= self.config.loop_max_correction_translation
            && correction.yaw.abs() <= self.config.loop_max_correction_yaw;
        accepted.then(|| LoopClosure {
            from: candidate,
            to: index,
            relative: relative_pose(self.nodes[candidate].pose, registration.pose),
            inlier_ratio,
            mean_residual: registration.mean_residual,
        })
    }

    /// Re-optimizes the pose graph over all odometry and loop edges.
    ///
    /// [`Self::update`] calls this when a loop closure disagrees with the
    /// graph; call it explicitly to fold in consistent loop edges, e.g.
    /// before reading the final trajectory.
    pub fn optimize(&mut self) {
        let initial: Vec<Pose2DNode> = self
            .nodes
            .iter()
            .map(|node| Pose2DNode::new(node.pose.x, node.pose.y, node.pose.yaw))
            .collect();
        let result = optimize_pose_graph(&initial, &self.edges, &self.config.pose_graph);
        for (node, pose) in self.nodes.iter_mut().zip(result.poses) {
            node.pose = Pose2D::new(pose.x, pose.y, pose.yaw);
        }
    }
}

fn diagonal_information((xy, yaw): (f64, f64)) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        1.0 / (xy * xy),
        1.0 / (xy * xy),
        1.0 / (yaw * yaw),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scan_to_map::{ranges_to_points, ray_cast_ranges, LineSegment};

    fn polygon(corners: &[(f64, f64)]) -> Vec<LineSegment> {
        (0..corners.len())
            .map(|i| {
                let (ax, ay) = corners[i];
                let (bx, by) = corners[(i + 1) % corners.len()];
                LineSegment::new(Vector2::new(ax, ay), Vector2::new(bx, by))
            })
            .collect()
    }

    fn world() -> Vec<LineSegment> {
        let mut walls = polygon(&[(-6.0, -4.0), (6.0, -4.0), (6.0, 4.0), (-6.0, 4.0)]);
        walls.extend(polygon(&[(1.0, 1.0), (2.0, 1.4), (1.6, 2.4), (0.6, 2.0)]));
        walls.extend(polygon(&[(-4.5, -3.0), (-3.7, -3.0), (-3.7, -2.2)]));
        walls
    }

    fn scan_at(pose: Pose2D) -> Vec<Vector2<f64>> {
        ranges_to_points(&ray_cast_ranges(pose, &world(), 360, 15.0))
    }

    #[test]
    fn creates_nodes_and_odometry_edges() {
        let config = LidarGraphSlamConfig {
            node_translation: 0.5,
            ..LidarGraphSlamConfig::default()
        };
        let mut slam = LidarGraphSlam::new(config, Pose2D::origin());
        let mut truth = Pose2D::origin();
        let step = Pose2D::new(0.1, 0.0, 0.0);
        for _ in 0..=20 {
            slam.update(step, &scan_at(truth));
            truth = compose_pose(truth, step);
        }
        let nodes = slam.node_poses();
        assert!(nodes.len() >= 4, "nodes: {}", nodes.len());
        assert_eq!(slam.edges().len(), nodes.len() - 1);
        assert!(slam.loop_closures().is_empty());
    }

    #[test]
    fn closes_a_loop_after_a_blind_odometry_stretch() {
        let config = LidarGraphSlamConfig {
            node_translation: 0.5,
            loop_min_node_gap: 6,
            ..LidarGraphSlamConfig::default()
        };
        let start = Pose2D::new(-1.0, -2.5, 0.0);
        let mut slam = LidarGraphSlam::new(config, start);

        // Drive a closed circle of radius 2 m back to the start.
        let steps = 126;
        let blind = 40..55;
        let step = Pose2D::new(0.1, 0.0, 0.1 / 2.0);
        let mut truth = start;
        slam.update(Pose2D::origin(), &scan_at(truth));
        let mut closures = 0;
        for i in 0..steps {
            truth = compose_pose(truth, step);
            if blind.contains(&i) {
                // LiDAR blind: biased odometry alone carries the front end.
                let odom = Pose2D::new(step.x * 1.15, 0.0, step.yaw + 0.003);
                slam.update(odom, &[]);
                if i + 1 == blind.end {
                    // The front end lost its map, so it cannot snap back.
                    slam.front_end.reset(slam.front_end.pose());
                }
                continue;
            }
            let update = slam.update(step, &scan_at(truth));
            closures += usize::from(update.loop_closure.is_some());
        }

        let drifted = relative_pose(truth, slam.front_end_pose());
        let corrected = relative_pose(truth, slam.pose());
        assert!(closures >= 1, "no loop closure accepted");
        assert!(
            drifted.x.hypot(drifted.y) > 0.05 && drifted.yaw.abs() > 0.03,
            "front end did not drift: {drifted:?}"
        );
        assert!(
            corrected.x.hypot(corrected.y) < 0.05,
            "loop closure left {:.3} m error",
            corrected.x.hypot(corrected.y)
        );
        assert!(corrected.yaw.abs() < 0.01, "yaw error {}", corrected.yaw);
    }
}
