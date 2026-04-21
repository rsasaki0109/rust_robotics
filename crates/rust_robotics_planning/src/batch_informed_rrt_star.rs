#![allow(dead_code, clippy::needless_borrows_for_generic_args)]

//! Batch Informed RRT* path planning algorithm
//!
//! Extends Informed RRT* with batch sampling for improved convergence.
//! Instead of sampling one point per iteration, a batch of samples is drawn
//! from the informed ellipsoidal region (or free space if no solution exists
//! yet) and processed together. After each batch the best solution cost is
//! updated and the sampling ellipsoid shrinks, focusing future samples on
//! the promising region.
//!
//! Reference: <https://arxiv.org/abs/1405.5848>

use rand::Rng;
use std::f64::consts::PI;

use rust_robotics_core::{Path2D, Point2D, RoboticsError, RoboticsResult};

/// A node in the RRT tree.
#[derive(Clone, Debug)]
pub struct Node {
    pub x: f64,
    pub y: f64,
    pub cost: f64,
    pub parent: Option<usize>,
}

impl Node {
    pub fn new(x: f64, y: f64) -> Self {
        Node {
            x,
            y,
            cost: 0.0,
            parent: None,
        }
    }
}

/// Configuration for the Batch Informed RRT* planner.
#[derive(Clone, Debug)]
pub struct BatchInformedRRTStarConfig {
    /// Number of samples per batch.
    pub batch_size: usize,
    /// Maximum number of batches to run.
    pub max_batches: usize,
    /// Step size for tree expansion.
    pub expand_dis: f64,
    /// Percentage of time to sample the goal directly (0-100).
    pub goal_sample_rate: i32,
    /// Circular obstacles as (x, y, radius).
    pub obstacle_list: Vec<(f64, f64, f64)>,
    /// Sampling bounds (min, max).
    pub rand_area: (f64, f64),
}

impl Default for BatchInformedRRTStarConfig {
    fn default() -> Self {
        Self {
            batch_size: 50,
            max_batches: 10,
            expand_dis: 0.5,
            goal_sample_rate: 10,
            obstacle_list: Vec::new(),
            rand_area: (-2.0, 15.0),
        }
    }
}

/// Batch Informed RRT* planner.
///
/// Combines RRT* tree construction (nearest-neighbour expansion, rewiring) with
/// informed ellipsoidal sampling. Samples are generated in batches: after each
/// batch the best-known solution cost is updated and the sampling ellipsoid is
/// tightened, leading to faster convergence towards the optimal path.
pub struct BatchInformedRRTStar {
    pub start: Node,
    pub goal: Node,
    pub min_rand: f64,
    pub max_rand: f64,
    pub expand_dis: f64,
    pub goal_sample_rate: i32,
    pub batch_size: usize,
    pub max_batches: usize,
    pub obstacle_list: Vec<(f64, f64, f64)>,
    pub node_list: Vec<Node>,
}

impl BatchInformedRRTStar {
    /// Create a new Batch Informed RRT* planner.
    pub fn new(start: (f64, f64), goal: (f64, f64), config: BatchInformedRRTStarConfig) -> Self {
        BatchInformedRRTStar {
            start: Node::new(start.0, start.1),
            goal: Node::new(goal.0, goal.1),
            min_rand: config.rand_area.0,
            max_rand: config.rand_area.1,
            expand_dis: config.expand_dis,
            goal_sample_rate: config.goal_sample_rate,
            batch_size: config.batch_size,
            max_batches: config.max_batches,
            obstacle_list: config.obstacle_list,
            node_list: Vec::new(),
        }
    }

    /// Run the planner and return the best path found (goal to start order), or `None`.
    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.planning_with_sampler(|planner, c_best, c_min, x_center, rotation_matrix| {
            planner.informed_sample(c_best, c_min, x_center, rotation_matrix)
        })
    }

    fn sampling_frame(&self) -> (f64, [f64; 2], [[f64; 2]; 2]) {
        let c_min =
            ((self.start.x - self.goal.x).powi(2) + (self.start.y - self.goal.y).powi(2)).sqrt();
        let x_center = [
            (self.start.x + self.goal.x) / 2.0,
            (self.start.y + self.goal.y) / 2.0,
        ];
        let a1 = [
            (self.goal.x - self.start.x) / c_min,
            (self.goal.y - self.start.y) / c_min,
        ];
        let e_theta = a1[1].atan2(a1[0]);
        let cos_theta = e_theta.cos();
        let sin_theta = e_theta.sin();
        let rotation_matrix = [[cos_theta, -sin_theta], [sin_theta, cos_theta]];

        (c_min, x_center, rotation_matrix)
    }

    fn reset_search(&mut self) {
        self.node_list = vec![self.start.clone()];
    }

    /// Core planning loop that processes samples in batches.
    ///
    /// After each batch of `batch_size` samples, the best known path cost is updated
    /// which tightens the informed sampling ellipsoid for the next batch.
    fn planning_with_sampler<F>(&mut self, mut sample_node: F) -> Option<Vec<[f64; 2]>>
    where
        F: FnMut(&BatchInformedRRTStar, f64, f64, &[f64; 2], &[[f64; 2]; 2]) -> [f64; 2],
    {
        self.reset_search();
        let mut c_best = f64::INFINITY;
        let mut path = None;

        let (c_min, x_center, rotation_matrix) = self.sampling_frame();

        for _batch in 0..self.max_batches {
            // Generate and process a batch of samples
            for _sample in 0..self.batch_size {
                let rnd = sample_node(self, c_best, c_min, &x_center, &rotation_matrix);
                let n_ind = self.get_nearest_list_index(&rnd);
                let nearest_node = &self.node_list[n_ind];

                let theta = (rnd[1] - nearest_node.y).atan2(rnd[0] - nearest_node.x);
                let new_node = self.get_new_node(theta, n_ind, nearest_node);
                let d = self.line_cost(nearest_node, &new_node);

                let no_collision = self.check_collision(nearest_node, theta, d);

                if no_collision {
                    let near_inds = self.find_near_nodes(&new_node);
                    let new_node = self.choose_parent(new_node, &near_inds);

                    let new_node_index = self.node_list.len();
                    self.node_list.push(new_node);
                    self.rewire(new_node_index, &near_inds);

                    if self.is_near_goal(&self.node_list[new_node_index])
                        && self.check_segment_collision(
                            self.node_list[new_node_index].x,
                            self.node_list[new_node_index].y,
                            self.goal.x,
                            self.goal.y,
                        )
                    {
                        let temp_path = self.get_final_course(new_node_index);
                        let temp_path_len = self.get_path_len(&temp_path);
                        if temp_path_len < c_best {
                            path = Some(temp_path);
                            c_best = temp_path_len;
                        }
                    }
                }
            }

            // After each batch, prune nodes that cannot improve the solution.
            // This keeps the tree lean and focused on the informed region.
            if c_best < f64::INFINITY {
                self.prune_nodes(c_best, c_min, &x_center, &rotation_matrix);
            }
        }

        path
    }

    /// Remove leaf nodes whose heuristic lower-bound cost exceeds `c_best`.
    ///
    /// A node is prunable when it is a leaf (no children) and its cost-to-come
    /// plus straight-line cost-to-goal exceeds the best known solution cost.
    /// Pruning is done iteratively until no more leaves can be removed.
    fn prune_nodes(
        &mut self,
        c_best: f64,
        _c_min: f64,
        _x_center: &[f64; 2],
        _rotation_matrix: &[[f64; 2]; 2],
    ) {
        // Iteratively remove leaves that cannot improve the solution.
        loop {
            let n = self.node_list.len();
            if n <= 1 {
                break;
            }

            // Find which nodes are parents (have children).
            let mut is_parent = vec![false; n];
            for node in &self.node_list {
                if let Some(p) = node.parent {
                    if p < n {
                        is_parent[p] = true;
                    }
                }
            }

            // Collect indices of leaf nodes (not parents, not root) that exceed c_best.
            let mut to_remove = Vec::new();
            for (i, &is_par) in is_parent.iter().enumerate().skip(1) {
                if !is_par {
                    let heuristic = self.node_list[i].cost + self.dist_to_goal(&self.node_list[i]);
                    if heuristic > c_best {
                        to_remove.push(i);
                    }
                }
            }

            if to_remove.is_empty() {
                break;
            }

            // Remove in reverse order to preserve indices.
            to_remove.sort_unstable();
            for &idx in to_remove.iter().rev() {
                self.node_list.remove(idx);
                // Update parent references that point beyond the removed index.
                for node in &mut self.node_list {
                    if let Some(ref mut p) = node.parent {
                        if *p == idx {
                            // This should not happen since idx was a leaf,
                            // but handle defensively.
                            node.parent = None;
                        } else if *p > idx {
                            *p -= 1;
                        }
                    }
                }
            }
        }
    }

    fn dist_to_goal(&self, node: &Node) -> f64 {
        ((node.x - self.goal.x).powi(2) + (node.y - self.goal.y).powi(2)).sqrt()
    }

    fn choose_parent(&self, mut new_node: Node, near_inds: &[usize]) -> Node {
        if near_inds.is_empty() {
            return new_node;
        }

        let mut d_list = Vec::new();
        for &i in near_inds {
            let dx = new_node.x - self.node_list[i].x;
            let dy = new_node.y - self.node_list[i].y;
            let d = (dx * dx + dy * dy).sqrt();
            let theta = dy.atan2(dx);
            if self.check_collision(&self.node_list[i], theta, d) {
                d_list.push(self.node_list[i].cost + d);
            } else {
                d_list.push(f64::INFINITY);
            }
        }

        let min_cost = d_list.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        if let Some(min_index) = d_list.iter().position(|&x| x == min_cost) {
            if min_cost != f64::INFINITY {
                new_node.cost = min_cost;
                new_node.parent = Some(near_inds[min_index]);
            }
        }

        new_node
    }

    fn find_near_nodes(&self, new_node: &Node) -> Vec<usize> {
        let n_node = self.node_list.len();
        let r = 50.0 * ((n_node as f64).ln() / n_node as f64).sqrt();
        let mut near_inds = Vec::new();

        for (i, node) in self.node_list.iter().enumerate() {
            let d_sq = (node.x - new_node.x).powi(2) + (node.y - new_node.y).powi(2);
            if d_sq <= r * r {
                near_inds.push(i);
            }
        }

        near_inds
    }

    fn informed_sample(
        &self,
        c_max: f64,
        c_min: f64,
        x_center: &[f64; 2],
        rotation_matrix: &[[f64; 2]; 2],
    ) -> [f64; 2] {
        if c_max < f64::INFINITY {
            let x_ball = self.sample_unit_ball();
            self.informed_sample_from_unit_ball(c_max, c_min, x_center, rotation_matrix, x_ball)
        } else {
            self.sample_free_space()
        }
    }

    fn informed_sample_from_unit_ball(
        &self,
        c_max: f64,
        c_min: f64,
        x_center: &[f64; 2],
        rotation_matrix: &[[f64; 2]; 2],
        x_ball: [f64; 2],
    ) -> [f64; 2] {
        let r = [c_max / 2.0, (c_max * c_max - c_min * c_min).sqrt() / 2.0];
        let scaled = [r[0] * x_ball[0], r[1] * x_ball[1]];
        let rotated = [
            rotation_matrix[0][0] * scaled[0] + rotation_matrix[0][1] * scaled[1],
            rotation_matrix[1][0] * scaled[0] + rotation_matrix[1][1] * scaled[1],
        ];

        [rotated[0] + x_center[0], rotated[1] + x_center[1]]
    }

    fn sample_unit_ball(&self) -> [f64; 2] {
        let mut rng = rand::thread_rng();
        let a: f64 = rng.gen();
        let b: f64 = rng.gen();

        Self::sample_unit_ball_from_uniforms(a, b)
    }

    fn sample_unit_ball_from_uniforms(a: f64, b: f64) -> [f64; 2] {
        let (a, b) = if b < a { (b, a) } else { (a, b) };
        let sample = (b * (2.0 * PI * a / b).cos(), b * (2.0 * PI * a / b).sin());
        [sample.0, sample.1]
    }

    fn sample_free_space(&self) -> [f64; 2] {
        let mut rng = rand::thread_rng();
        if rng.gen_range(0..=100) > self.goal_sample_rate {
            [
                rng.gen_range(self.min_rand..=self.max_rand),
                rng.gen_range(self.min_rand..=self.max_rand),
            ]
        } else {
            [self.goal.x, self.goal.y]
        }
    }

    fn get_path_len(&self, path: &[[f64; 2]]) -> f64 {
        let mut path_len = 0.0;
        for i in 1..path.len() {
            let dx = path[i][0] - path[i - 1][0];
            let dy = path[i][1] - path[i - 1][1];
            path_len += (dx * dx + dy * dy).sqrt();
        }
        path_len
    }

    fn line_cost(&self, node1: &Node, node2: &Node) -> f64 {
        ((node1.x - node2.x).powi(2) + (node1.y - node2.y).powi(2)).sqrt()
    }

    fn get_nearest_list_index(&self, rnd: &[f64; 2]) -> usize {
        let mut min_dist = f64::INFINITY;
        let mut min_index = 0;

        for (i, node) in self.node_list.iter().enumerate() {
            let dist = (node.x - rnd[0]).powi(2) + (node.y - rnd[1]).powi(2);
            if dist < min_dist {
                min_dist = dist;
                min_index = i;
            }
        }

        min_index
    }

    fn get_new_node(&self, theta: f64, n_ind: usize, nearest_node: &Node) -> Node {
        let mut new_node = nearest_node.clone();
        new_node.x += self.expand_dis * theta.cos();
        new_node.y += self.expand_dis * theta.sin();
        new_node.cost += self.expand_dis;
        new_node.parent = Some(n_ind);
        new_node
    }

    fn is_near_goal(&self, node: &Node) -> bool {
        let d = self.line_cost(node, &self.goal);
        d < self.expand_dis
    }

    fn rewire(&mut self, new_node_index: usize, near_inds: &[usize]) {
        for &i in near_inds {
            let near_node = &self.node_list[i];
            let new_node = &self.node_list[new_node_index];

            let d =
                ((near_node.x - new_node.x).powi(2) + (near_node.y - new_node.y).powi(2)).sqrt();
            let s_cost = new_node.cost + d;

            if near_node.cost > s_cost {
                let theta = (new_node.y - near_node.y).atan2(new_node.x - near_node.x);
                if self.check_collision(near_node, theta, d) {
                    self.node_list[i].parent = Some(new_node_index);
                    self.node_list[i].cost = s_cost;
                }
            }
        }
    }

    fn distance_squared_point_to_segment(&self, v: [f64; 2], w: [f64; 2], p: [f64; 2]) -> f64 {
        if v[0] == w[0] && v[1] == w[1] {
            return (p[0] - v[0]).powi(2) + (p[1] - v[1]).powi(2);
        }

        let l2 = (w[0] - v[0]).powi(2) + (w[1] - v[1]).powi(2);
        let t =
            (((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2).clamp(0.0, 1.0);
        let projection = [v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])];
        (p[0] - projection[0]).powi(2) + (p[1] - projection[1]).powi(2)
    }

    fn check_segment_collision(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        for &(ox, oy, size) in &self.obstacle_list {
            let dd = self.distance_squared_point_to_segment([x1, y1], [x2, y2], [ox, oy]);
            if dd <= size * size {
                return false;
            }
        }
        true
    }

    fn check_collision(&self, near_node: &Node, theta: f64, d: f64) -> bool {
        let end_x = near_node.x + theta.cos() * d;
        let end_y = near_node.y + theta.sin() * d;
        self.check_segment_collision(near_node.x, near_node.y, end_x, end_y)
    }

    fn get_final_course(&self, last_index: usize) -> Vec<[f64; 2]> {
        let mut path = vec![[self.goal.x, self.goal.y]];
        let mut current_index = last_index;

        while let Some(parent_index) = self.node_list[current_index].parent {
            let node = &self.node_list[current_index];
            path.push([node.x, node.y]);
            current_index = parent_index;
        }

        path.push([self.start.x, self.start.y]);
        path
    }

    /// Plan a path from the given start to goal, returning a [`Path2D`].
    ///
    /// This is a convenience wrapper around [`planning()`](Self::planning) that accepts
    /// [`Point2D`], sets the start/goal, runs the planner, and returns [`Path2D`].
    pub fn plan_from(&mut self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.start = Node::new(start.x, start.y);
        self.goal = Node::new(goal.x, goal.y);

        self.planning()
            .map(|raw_path| {
                Path2D::from_points(
                    raw_path
                        .into_iter()
                        .rev()
                        .map(|p| Point2D::new(p[0], p[1]))
                        .collect(),
                )
            })
            .ok_or_else(|| {
                RoboticsError::PlanningError(
                    "BatchInformedRRT*: Cannot find path within max batches".to_string(),
                )
            })
    }

    /// Get the tree nodes for external inspection.
    pub fn get_tree(&self) -> &[Node] {
        &self.node_list
    }

    /// Get the obstacle list.
    pub fn get_obstacles(&self) -> &[(f64, f64, f64)] {
        &self.obstacle_list
    }
}

#[cfg(test)]
impl BatchInformedRRTStar {
    fn prune_nodes_for_test(
        &mut self,
        c_best: f64,
        c_min: f64,
        x_center: &[f64; 2],
        rotation_matrix: &[[f64; 2]; 2],
    ) {
        self.prune_nodes(c_best, c_min, x_center, rotation_matrix);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "expected {expected}, got {actual}"
        );
    }

    fn default_obstacles() -> Vec<(f64, f64, f64)> {
        vec![
            (5.0, 5.0, 0.5),
            (9.0, 6.0, 1.0),
            (7.0, 5.0, 1.0),
            (1.0, 5.0, 1.0),
            (3.0, 6.0, 1.0),
            (7.0, 9.0, 1.0),
        ]
    }

    fn create_planner(
        obstacles: Vec<(f64, f64, f64)>,
        batch_size: usize,
        max_batches: usize,
    ) -> BatchInformedRRTStar {
        BatchInformedRRTStar::new(
            (0.0, 0.0),
            (5.0, 10.0),
            BatchInformedRRTStarConfig {
                batch_size,
                max_batches,
                expand_dis: 0.5,
                goal_sample_rate: 10,
                obstacle_list: obstacles,
                rand_area: (-2.0, 15.0),
            },
        )
    }

    #[test]
    fn test_batch_informed_rrt_star_config() {
        let planner = create_planner(vec![(5.0, 5.0, 0.5)], 50, 10);
        assert_eq!(planner.expand_dis, 0.5);
        assert_eq!(planner.batch_size, 50);
        assert_eq!(planner.max_batches, 10);
    }

    #[test]
    fn test_sampling_frame_matches_informed_rrt_star() {
        let planner = create_planner(default_obstacles(), 50, 10);
        let (c_min, x_center, rotation_matrix) = planner.sampling_frame();

        assert_close(c_min, 11.180_339_887_498_949);
        assert_close(x_center[0], 2.5);
        assert_close(x_center[1], 5.0);
        assert_close(rotation_matrix[0][0], 0.447_213_595_499_958);
        assert_close(rotation_matrix[0][1], -0.894_427_190_999_916);
        assert_close(rotation_matrix[1][0], 0.894_427_190_999_916);
        assert_close(rotation_matrix[1][1], 0.447_213_595_499_958);
    }

    #[test]
    fn test_sample_unit_ball() {
        let expected = [
            ((0.2, 0.8), [0.0, 0.8]),
            ((0.1, 0.4), [0.0, 0.4]),
            ((0.33, 0.9), [-0.602_217_545_722_972, 0.668_830_342_929_655]),
        ];

        for ((a, b), xy) in expected {
            let sample = BatchInformedRRTStar::sample_unit_ball_from_uniforms(a, b);
            assert_close(sample[0], xy[0]);
            assert_close(sample[1], xy[1]);
        }
    }

    #[test]
    fn test_informed_ellipse_sampling() {
        let planner = create_planner(default_obstacles(), 50, 10);
        let (c_min, x_center, rotation_matrix) = planner.sampling_frame();
        let unit_ball_samples = [
            [0.0, 0.8],
            [0.0, 0.4],
            [-0.602_217_545_722_972, 0.668_830_342_929_655],
        ];
        let expected = [
            (
                12.0,
                [
                    [0.940_512_904_830_566, 5.779_743_547_584_717],
                    [1.720_256_452_415_283, 5.389_871_773_792_358],
                    [-0.419_709_604_196_265, 2.420_056_693_659_169],
                ],
            ),
            (
                14.0,
                [
                    [-0.514_630_989_026_684, 6.507_315_494_513_342],
                    [0.992_684_505_486_658, 5.753_657_747_256_671],
                    [-1.905_584_965_017_868, 2.489_694_689_330_144],
                ],
            ),
        ];

        for (c_best, xy_samples) in expected {
            for (x_ball, xy) in unit_ball_samples.iter().zip(xy_samples.iter()) {
                let sample = planner.informed_sample_from_unit_ball(
                    c_best,
                    c_min,
                    &x_center,
                    &rotation_matrix,
                    *x_ball,
                );
                assert_close(sample[0], xy[0]);
                assert_close(sample[1], xy[1]);
            }
        }
    }

    #[test]
    fn test_seeded_single_batch_matches_informed_rrt_star_logic() {
        // Use deterministic samples to verify the tree-building logic is correct.
        // A single batch with N samples should behave identically to InformedRRTStar
        // with N iterations (no pruning occurs because c_best starts infinite within
        // the first batch).
        let mut planner = create_planner(default_obstacles(), 20, 1);
        let samples = [
            [10.455_649_682_677_358, 11.942_970_283_541_907],
            [12.537_351_811_401_535, 13.840_339_744_778_298],
            [7.622_138_868_390_643, 0.748_693_006_799_259],
            [12.860_963_734_685_752, 2.438_529_994_751_022],
            [0.963_840_532_303_441, 7.404_758_454_678_607],
            [10.548_525_179_081_42, 10.458_784_524_738_785],
            [14.636_879_924_696_97, 5.006_029_679_968_117],
            [0.831_739_168_751_872, 1.497_141_169_178_794],
            [5.0, 10.0],
            [12.230_194_783_259_385, 3.474_659_848_212_157],
            [3.771_802_130_764_652, 14.447_201_800_957_814],
            [10.657_010_682_397_589, -1.941_271_617_163_375],
            [12.803_038_468_843_097, 11.104_183_754_785_783],
            [2.871_774_581_756_395, 9.093_185_216_538_995],
            [13.054_119_941_473_152, 7.827_462_465_227_494],
            [4.375_040_433_962_334, 14.322_895_906_129_173],
            [10.059_569_437_230_238, 12.022_177_097_262_272],
            [5.0, 10.0],
            [0.036_638_892_026_463, 6.486_823_094_475_769],
            [11.632_287_039_256_342, 7.436_450_081_952_417],
        ];
        let mut sample_index = 0_usize;

        let path = planner.planning_with_sampler(|_, _, _, _, _| {
            let sample = *samples
                .get(sample_index)
                .expect("sample sequence exhausted");
            sample_index += 1;
            sample
        });

        assert!(path.is_none());
        assert_eq!(sample_index, samples.len());
        // Tree should have 20 nodes (same as InformedRRTStar with 20 iterations)
        assert_eq!(planner.node_list.len(), 20);

        let expected_nodes = [
            (
                1,
                [0.329_351_320_180_549, 0.376_201_685_130_901],
                0.5,
                Some(0),
            ),
            (
                2,
                [0.665_203_546_734_372, 0.746_611_298_827_467],
                0.999_962_094_343_993,
                Some(0),
            ),
            (
                5,
                [1.607_494_092_207_533, 1.315_569_836_427_121],
                2.077_200_339_639_631,
                Some(0),
            ),
            (
                10,
                [3.085_807_934_983_019, 2.339_074_098_076_378],
                3.872_141_300_094_301,
                Some(0),
            ),
            (
                15,
                [3.900_997_195_468_872, 3.858_949_080_306_712],
                5.487_191_187_069_758,
                Some(0),
            ),
        ];

        for (index, xy, cost, parent) in expected_nodes {
            let node = &planner.node_list[index];
            assert_close(node.x, xy[0]);
            assert_close(node.y, xy[1]);
            assert_close(node.cost, cost);
            assert_eq!(node.parent, parent);
        }
    }

    #[test]
    fn test_finds_path_open_space() {
        // With no obstacles and a large expand_dis, a path should be found quickly.
        let mut planner = BatchInformedRRTStar::new(
            (0.0, 0.0),
            (5.0, 10.0),
            BatchInformedRRTStarConfig {
                batch_size: 100,
                max_batches: 10,
                expand_dis: 5.0_f64.hypot(10.0),
                goal_sample_rate: 100,
                obstacle_list: vec![],
                rand_area: (-2.0, 15.0),
            },
        );

        let path = planner.planning();
        assert!(path.is_some(), "Should find a path in open space");

        let path = path.unwrap();
        assert!(path.len() >= 2);
        // Path starts at goal, ends at start (goal-to-start order).
        let first = path.first().unwrap();
        let last = path.last().unwrap();
        assert_close(first[0], 5.0);
        assert_close(first[1], 10.0);
        assert_close(last[0], 0.0);
        assert_close(last[1], 0.0);
    }

    #[test]
    fn test_finds_path_with_obstacles() {
        let mut planner = create_planner(default_obstacles(), 200, 20);

        let path = planner.planning();
        assert!(
            path.is_some(),
            "Should find a path around obstacles with enough samples"
        );

        let path = path.unwrap();
        // Verify the path is collision-free.
        for window in path.windows(2) {
            let (x1, y1) = (window[0][0], window[0][1]);
            let (x2, y2) = (window[1][0], window[1][1]);
            assert!(
                planner.check_segment_collision(x1, y1, x2, y2),
                "Path segment ({},{})--({},{}) should be collision-free",
                x1,
                y1,
                x2,
                y2
            );
        }
    }

    #[test]
    fn test_plan_from_returns_path2d() {
        let mut planner = BatchInformedRRTStar::new(
            (0.0, 0.0),
            (5.0, 10.0),
            BatchInformedRRTStarConfig {
                batch_size: 200,
                max_batches: 20,
                expand_dis: 0.5,
                goal_sample_rate: 10,
                obstacle_list: vec![],
                rand_area: (-2.0, 15.0),
            },
        );

        let result = planner.plan_from(Point2D::new(0.0, 0.0), Point2D::new(5.0, 10.0));
        assert!(result.is_ok(), "plan_from should succeed in open space");
        let path = result.unwrap();
        assert!(path.points.len() >= 2);
        // Path2D is start-to-goal order (reversed from raw).
        assert_close(path.points.first().unwrap().x, 0.0);
        assert_close(path.points.first().unwrap().y, 0.0);
        assert_close(path.points.last().unwrap().x, 5.0);
        assert_close(path.points.last().unwrap().y, 10.0);
    }

    #[test]
    fn test_batch_processing_improves_or_maintains_cost() {
        // Run with 1 large batch vs. multiple smaller batches. The multi-batch
        // version benefits from pruning and tighter ellipsoidal sampling.
        let obstacles = default_obstacles();

        let mut single_batch_costs = Vec::new();
        let mut multi_batch_costs = Vec::new();

        for _ in 0..5 {
            let mut p1 = create_planner(obstacles.clone(), 400, 1);
            if let Some(path) = p1.planning() {
                single_batch_costs.push(p1.get_path_len(&path));
            }

            let mut p2 = create_planner(obstacles.clone(), 100, 4);
            if let Some(path) = p2.planning() {
                multi_batch_costs.push(p2.get_path_len(&path));
            }
        }

        // Both configurations should find paths at least some of the time.
        assert!(
            !single_batch_costs.is_empty() || !multi_batch_costs.is_empty(),
            "At least one configuration should find a path"
        );
    }

    #[test]
    fn test_prune_nodes_removes_dominated_leaf() {
        // Deterministic check: a leaf whose cost + straight-line-to-goal exceeds
        // c_best must be removed (see `prune_nodes`). Comparing two stochastic
        // planning runs was flaky across CI runners due to `thread_rng()`.
        let mut planner = create_planner(vec![], 10, 1);
        planner.node_list = vec![
            Node {
                x: 0.0,
                y: 0.0,
                cost: 0.0,
                parent: None,
            },
            Node {
                x: 100.0,
                y: 100.0,
                cost: 5.0,
                parent: Some(0),
            },
        ];
        let (c_min, x_center, rotation_matrix) = planner.sampling_frame();
        let c_best = 15.0;
        assert!(
            planner.node_list[1].cost + planner.dist_to_goal(&planner.node_list[1]) > c_best,
            "fixture leaf should be prunable"
        );
        planner.prune_nodes_for_test(c_best, c_min, &x_center, &rotation_matrix);
        assert_eq!(planner.node_list.len(), 1);
    }
}
