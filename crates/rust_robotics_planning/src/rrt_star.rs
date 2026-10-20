#![allow(dead_code, clippy::too_many_arguments)]

//! RRT* (Rapidly-exploring Random Tree Star) path planning algorithm
//!
//! An optimized version of RRT that rewires the tree to find shorter paths.

use rand::{rngs::StdRng, Rng, SeedableRng};

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Internal node for RRT* tree
#[derive(Debug, Clone)]
pub struct Node {
    pub x: f64,
    pub y: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub cost: f64,
    pub parent: Option<usize>,
}

impl Node {
    pub fn new(x: f64, y: f64) -> Self {
        Node {
            x,
            y,
            path_x: Vec::new(),
            path_y: Vec::new(),
            cost: 0.0,
            parent: None,
        }
    }
}

/// Lightweight node state for an actual RRT* search history.
#[derive(Debug, Clone, PartialEq)]
pub struct RrtStarTraceNode {
    pub point: [f64; 2],
    pub parent: Option<usize>,
    pub cost: f64,
}

impl From<&Node> for RrtStarTraceNode {
    fn from(node: &Node) -> Self {
        Self {
            point: [node.x, node.y],
            parent: node.parent,
            cost: node.cost,
        }
    }
}

/// A node insertion, reparenting, or propagated cost update in one iteration.
#[derive(Debug, Clone, PartialEq)]
pub struct RrtStarNodeUpdate {
    pub index: usize,
    pub before: Option<RrtStarTraceNode>,
    pub after: RrtStarTraceNode,
}

/// State observed after one real planner iteration, including rejected samples.
#[derive(Debug, Clone, PartialEq)]
pub struct RrtStarSearchStep {
    pub sample: [f64; 2],
    pub nearest: usize,
    pub candidate: Option<[f64; 2]>,
    pub accepted: bool,
    pub updates: Vec<RrtStarNodeUpdate>,
    /// Current cheapest collision-free goal connection, in goal-to-start order.
    pub path: Vec<[f64; 2]>,
}

/// A deterministic run recorded as node changes, rather than final-tree reveals.
#[derive(Debug, Clone, PartialEq)]
pub struct RrtStarSearchTrace {
    pub root: RrtStarTraceNode,
    pub steps: Vec<RrtStarSearchStep>,
    pub path: Option<Vec<[f64; 2]>>,
}

impl RrtStarSearchTrace {
    /// Reconstruct the tree after this many completed iterations. Zero is the root.
    pub fn tree_at(&self, completed_iterations: usize) -> Vec<RrtStarTraceNode> {
        let mut nodes = vec![self.root.clone()];
        for step in self.steps.iter().take(completed_iterations) {
            for update in &step.updates {
                if update.index == nodes.len() {
                    nodes.push(update.after.clone());
                } else {
                    nodes[update.index] = update.after.clone();
                }
            }
        }
        nodes
    }
}

#[derive(Debug, Clone)]
pub struct RRTStar {
    pub start: Node,
    pub end: Node,
    pub min_rand: f64,
    pub max_rand: f64,
    pub expand_dis: f64,
    pub path_resolution: f64,
    pub goal_sample_rate: i32,
    pub max_iter: i32,
    pub connect_circle_dist: f64,
    pub search_until_max_iter: bool,
    pub robot_radius: f64,
    pub obstacle_list: Vec<(f64, f64, f64)>, // (x, y, radius)
    pub node_list: Vec<Node>,
}

impl RRTStar {
    pub fn new(
        start: (f64, f64),
        goal: (f64, f64),
        obstacle_list: Vec<(f64, f64, f64)>,
        rand_area: (f64, f64),
        expand_dis: f64,
        path_resolution: f64,
        goal_sample_rate: i32,
        max_iter: i32,
        connect_circle_dist: f64,
        search_until_max_iter: bool,
        robot_radius: f64,
    ) -> Self {
        RRTStar {
            start: Node::new(start.0, start.1),
            end: Node::new(goal.0, goal.1),
            min_rand: rand_area.0,
            max_rand: rand_area.1,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            connect_circle_dist,
            search_until_max_iter,
            robot_radius,
            obstacle_list,
            node_list: Vec::new(),
        }
    }

    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.planning_with_sampler(|planner| planner.get_random_node())
    }

    /// Run the same search with a seeded sampler and record each iteration.
    /// Recording does not alter parent selection, rewiring, or the stop condition.
    /// Memory stores only changed lightweight nodes and the current solution path.
    pub fn planning_with_trace(&mut self, seed: u64) -> RrtStarSearchTrace {
        let root = RrtStarTraceNode::from(&self.start);
        let mut previous = vec![root.clone()];
        let mut steps = Vec::new();
        let mut rng = StdRng::seed_from_u64(seed);
        let path = self.planning_with_observer(
            |planner| planner.get_random_node_with_rng(&mut rng),
            |planner, sample, nearest, candidate| {
                let nodes: Vec<_> = planner
                    .node_list
                    .iter()
                    .map(RrtStarTraceNode::from)
                    .collect();
                let accepted = nodes.len() > previous.len();
                let updates = nodes
                    .iter()
                    .enumerate()
                    .filter_map(|(index, node)| {
                        let before = previous.get(index);
                        (before != Some(node)).then(|| RrtStarNodeUpdate {
                            index,
                            before: before.cloned(),
                            after: node.clone(),
                        })
                    })
                    .collect();
                let path = planner
                    .search_best_goal_node()
                    .map(|i| planner.generate_final_course(i))
                    .unwrap_or_default();
                steps.push(RrtStarSearchStep {
                    sample,
                    nearest,
                    candidate,
                    accepted,
                    updates,
                    path,
                });
                previous = nodes;
            },
        );
        RrtStarSearchTrace { root, steps, path }
    }

    fn reset_search(&mut self) {
        self.node_list = vec![self.start.clone()];
    }

    fn planning_with_sampler<F>(&mut self, mut sample_node: F) -> Option<Vec<[f64; 2]>>
    where
        F: FnMut(&RRTStar) -> Node,
    {
        self.planning_with_observer(&mut sample_node, |_, _, _, _| {})
    }

    fn planning_with_observer<F, O>(
        &mut self,
        mut sample_node: F,
        mut observe: O,
    ) -> Option<Vec<[f64; 2]>>
    where
        F: FnMut(&RRTStar) -> Node,
        O: FnMut(&RRTStar, [f64; 2], usize, Option<[f64; 2]>),
    {
        self.reset_search();

        for _i in 0..self.max_iter {
            let rnd_node = sample_node(self);
            let nearest_ind = self.get_nearest_node_index(&rnd_node);
            let mut new_node = self.steer(nearest_ind, &rnd_node);
            let candidate = new_node.as_ref().map(|n| [n.x, n.y]);

            if let Some(ref mut node) = new_node {
                let near_node = &self.node_list[nearest_ind];
                node.cost = near_node.cost + self.calc_distance(near_node, node);

                if self.check_collision_free(node) {
                    let near_inds = self.find_near_nodes(node);
                    let node_with_updated_parent = self.choose_parent(node.clone(), &near_inds);

                    if let Some(updated_node) = node_with_updated_parent {
                        let new_index = self.node_list.len();
                        self.node_list.push(updated_node);
                        self.rewire(new_index, &near_inds);
                    } else {
                        self.node_list.push(node.clone());
                    }
                }
            }

            observe(self, [rnd_node.x, rnd_node.y], nearest_ind, candidate);
            if !self.search_until_max_iter && new_node.is_some() {
                if let Some(last_index) = self.search_best_goal_node() {
                    return Some(self.generate_final_course(last_index));
                }
            }
        }

        if let Some(last_index) = self.search_best_goal_node() {
            return Some(self.generate_final_course(last_index));
        }

        None
    }

    fn get_random_node(&self) -> Node {
        let mut rng = rand::rng();
        self.get_random_node_with_rng(&mut rng)
    }

    fn get_random_node_with_rng(&self, rng: &mut impl Rng) -> Node {
        if rng.random_range(0..=100) > self.goal_sample_rate {
            Node::new(
                rng.random_range(self.min_rand..=self.max_rand),
                rng.random_range(self.min_rand..=self.max_rand),
            )
        } else {
            Node::new(self.end.x, self.end.y)
        }
    }

    fn get_nearest_node_index(&self, rnd_node: &Node) -> usize {
        let mut min_dist = f64::INFINITY;
        let mut nearest_ind = 0;

        for (i, node) in self.node_list.iter().enumerate() {
            let dist = self.calc_distance(node, rnd_node);
            if dist < min_dist {
                min_dist = dist;
                nearest_ind = i;
            }
        }

        nearest_ind
    }

    fn steer(&self, from_ind: usize, to_node: &Node) -> Option<Node> {
        let from_node = &self.node_list[from_ind];
        Some(self.steer_from_node(from_node, to_node, self.expand_dis, Some(from_ind)))
    }

    fn steer_from_node(
        &self,
        from_node: &Node,
        to_node: &Node,
        extend_length: f64,
        parent: Option<usize>,
    ) -> Node {
        let mut new_node = Node::new(from_node.x, from_node.y);
        let (dist, theta) = self.calc_distance_and_angle(&new_node, to_node);
        let extend_length = extend_length.min(dist);

        new_node.path_x = vec![new_node.x];
        new_node.path_y = vec![new_node.y];

        let n_expand = (extend_length / self.path_resolution).floor() as i32;
        for _ in 0..n_expand {
            new_node.x += self.path_resolution * theta.cos();
            new_node.y += self.path_resolution * theta.sin();
            new_node.path_x.push(new_node.x);
            new_node.path_y.push(new_node.y);
        }

        let (remaining_dist, _) = self.calc_distance_and_angle(&new_node, to_node);
        if remaining_dist <= self.path_resolution {
            new_node.path_x.push(to_node.x);
            new_node.path_y.push(to_node.y);
            new_node.x = to_node.x;
            new_node.y = to_node.y;
        }

        new_node.parent = parent;
        new_node
    }

    fn check_collision_free(&self, node: &Node) -> bool {
        if node.path_x.is_empty() || node.path_y.is_empty() {
            return true;
        }

        // Every segment between the path points, not only the points.
        !self.obstacle_list.iter().any(|&(ox, oy, size)| {
            crate::rrt::polyline_within(
                &node.path_x,
                &node.path_y,
                [ox, oy],
                size + self.robot_radius,
            )
        })
    }

    fn find_near_nodes(&self, new_node: &Node) -> Vec<usize> {
        let nnode = self.node_list.len() + 1;
        let r = self.connect_circle_dist * ((nnode as f64).ln() / nnode as f64).sqrt();
        let r = r.min(self.expand_dis);

        self.node_list
            .iter()
            .enumerate()
            .filter_map(|(i, node)| {
                let dist_sq = (node.x - new_node.x).powi(2) + (node.y - new_node.y).powi(2);
                if dist_sq <= r.powi(2) {
                    Some(i)
                } else {
                    None
                }
            })
            .collect()
    }

    fn choose_parent(&self, new_node: Node, near_inds: &[usize]) -> Option<Node> {
        if near_inds.is_empty() {
            return None;
        }

        let mut costs = Vec::new();
        for &i in near_inds {
            let near_node = &self.node_list[i];
            let t_node = self.steer_from_node(near_node, &new_node, f64::INFINITY, Some(i));

            if self.check_collision_free(&t_node) {
                costs.push(self.calc_new_cost(near_node, &new_node));
            } else {
                costs.push(f64::INFINITY);
            }
        }

        let min_cost = costs.iter().fold(f64::INFINITY, |a, &b| a.min(b));

        if min_cost == f64::INFINITY {
            return None;
        }

        let min_ind = costs.iter().position(|&x| x == min_cost)?;
        let parent_ind = near_inds[min_ind];

        let mut result_node = self.steer_from_node(
            &self.node_list[parent_ind],
            &new_node,
            f64::INFINITY,
            Some(parent_ind),
        );
        result_node.cost = min_cost;

        Some(result_node)
    }

    fn search_best_goal_node(&self) -> Option<usize> {
        let dist_to_goal_list: Vec<f64> = self
            .node_list
            .iter()
            .map(|n| self.calc_dist_to_goal(n.x, n.y))
            .collect();

        let goal_inds: Vec<usize> = dist_to_goal_list
            .iter()
            .enumerate()
            .filter_map(|(i, &dist)| {
                if dist <= self.expand_dis {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();

        let safe_goal_inds: Vec<usize> = goal_inds
            .into_iter()
            .filter(|&goal_ind| {
                let t_node = self.steer_from_node(
                    &self.node_list[goal_ind],
                    &self.end,
                    f64::INFINITY,
                    Some(goal_ind),
                );
                self.check_collision_free(&t_node)
            })
            .collect();

        if safe_goal_inds.is_empty() {
            return None;
        }

        let safe_goal_costs: Vec<f64> = safe_goal_inds
            .iter()
            .map(|&i| {
                self.node_list[i].cost
                    + self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
            })
            .collect();

        let min_cost = safe_goal_costs.iter().fold(f64::INFINITY, |a, &b| a.min(b));

        safe_goal_inds
            .into_iter()
            .zip(safe_goal_costs)
            .find(|(_, cost)| *cost == min_cost)
            .map(|(i, _)| i)
    }

    fn rewire(&mut self, new_node_ind: usize, near_inds: &[usize]) {
        for &i in near_inds {
            let near_node = self.node_list[i].clone();
            let new_node = &self.node_list[new_node_ind];

            let mut edge_node =
                self.steer_from_node(new_node, &near_node, f64::INFINITY, Some(new_node_ind));
            edge_node.cost = self.calc_new_cost(new_node, &near_node);

            let no_collision = self.check_collision_free(&edge_node);
            let improved_cost = near_node.cost > edge_node.cost;

            if no_collision && improved_cost {
                self.node_list[i] = edge_node;
                self.propagate_cost_to_leaves(i);
            }
        }
    }

    fn calc_new_cost(&self, from_node: &Node, to_node: &Node) -> f64 {
        from_node.cost + self.calc_distance(from_node, to_node)
    }

    fn propagate_cost_to_leaves(&mut self, parent_ind: usize) {
        let parent_node = self.node_list[parent_ind].clone();

        for i in 0..self.node_list.len() {
            if let Some(node_parent) = self.node_list[i].parent {
                if node_parent == parent_ind {
                    self.node_list[i].cost =
                        self.calc_new_cost(&parent_node, &self.node_list[i].clone());
                    self.propagate_cost_to_leaves(i);
                }
            }
        }
    }

    fn generate_final_course(&self, goal_ind: usize) -> Vec<[f64; 2]> {
        let mut path = vec![[self.end.x, self.end.y]];
        let mut node = &self.node_list[goal_ind];

        while let Some(parent_ind) = node.parent {
            path.push([node.x, node.y]);
            node = &self.node_list[parent_ind];
        }
        path.push([node.x, node.y]);

        path
    }

    fn calc_dist_to_goal(&self, x: f64, y: f64) -> f64 {
        let dx = x - self.end.x;
        let dy = y - self.end.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn calc_distance(&self, from_node: &Node, to_node: &Node) -> f64 {
        let dx = to_node.x - from_node.x;
        let dy = to_node.y - from_node.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn calc_distance_and_angle(&self, from_node: &Node, to_node: &Node) -> (f64, f64) {
        let dx = to_node.x - from_node.x;
        let dy = to_node.y - from_node.y;
        let d = (dx * dx + dy * dy).sqrt();
        let theta = dy.atan2(dx);
        (d, theta)
    }

    /// Plan a path from the given start to goal, returning a [`Path2D`].
    ///
    /// This is a convenience wrapper around [`planning()`](Self::planning) that accepts
    /// [`Point2D`], sets the start/goal, runs the planner, and returns [`Path2D`].
    /// Requires `&mut self` because the underlying algorithm mutates internal state.
    pub fn plan_from(&mut self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.start = Node::new(start.x, start.y);
        self.end = Node::new(goal.x, goal.y);

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
                    "RRT*: Cannot find path within max iterations".to_string(),
                )
            })
    }

    /// Get the tree nodes for external inspection
    pub fn get_tree(&self) -> &[Node] {
        &self.node_list
    }

    /// Get the obstacle list
    pub fn get_obstacles(&self) -> &[(f64, f64, f64)] {
        &self.obstacle_list
    }
}

impl PathPlanner for RRTStar {
    /// Plans on a copy of this planner, so `&self` keeps its own tree; use
    /// [`RRTStar::plan_from`] to keep the tree for inspection.
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.clone().plan_from(start, goal)
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

    fn parse_xy_fixture(csv: &str) -> Vec<[f64; 2]> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let (x, y) = line
                    .split_once(',')
                    .expect("xy fixture rows must contain a comma");
                [x.parse().unwrap(), y.parse().unwrap()]
            })
            .collect()
    }

    fn create_pythonrobotics_main_planner() -> RRTStar {
        RRTStar::new(
            (0.0, 0.0),
            (6.0, 10.0),
            vec![
                (5.0, 5.0, 1.0),
                (3.0, 6.0, 2.0),
                (3.0, 8.0, 2.0),
                (3.0, 10.0, 2.0),
                (7.0, 5.0, 2.0),
                (9.0, 5.0, 2.0),
                (8.0, 10.0, 1.0),
                (6.0, 12.0, 1.0),
            ],
            (-2.0, 15.0),
            1.0,
            1.0,
            20,
            300,
            50.0,
            false,
            0.8,
        )
    }

    #[test]
    fn test_rrt_star_config() {
        let rrt = RRTStar::new(
            (0.0, 0.0),
            (6.0, 10.0),
            vec![(5.0, 5.0, 1.0)],
            (-2.0, 15.0),
            2.0,
            0.5,
            20,
            500,
            50.0,
            false,
            0.3,
        );
        assert_eq!(rrt.expand_dis, 2.0);
        assert_eq!(rrt.max_iter, 500);
    }

    #[test]
    fn test_rrt_star_upstream_no_obstacle_seeded_reference() {
        for robot_radius in [0.0, 0.8] {
            let mut rrt = RRTStar::new(
                (0.0, 0.0),
                (6.0, 10.0),
                vec![],
                (-2.0, 15.0),
                30.0,
                1.0,
                20,
                300,
                50.0,
                false,
                robot_radius,
            );
            let sample = [10.455_649_682_677_358, 11.942_970_283_541_907];
            let path = rrt
                .planning_with_sampler(|_| Node::new(sample[0], sample[1]))
                .unwrap();
            assert_eq!(rrt.node_list.len(), 2);
            assert_eq!(path, vec![[6.0, 10.0], [0.0, 0.0]]);
            assert_close(rrt.node_list[1].x, sample[0]);
            assert_close(rrt.node_list[1].y, sample[1]);
            assert_close(rrt.node_list[1].cost, 15.873_095_144_943_73);
            assert_eq!(rrt.node_list[1].parent, Some(0));
        }
    }

    #[test]
    fn test_rrt_star_upstream_seeded_main_prefix_matches_pythonrobotics_reference() {
        let mut rrt = create_pythonrobotics_main_planner();
        rrt.max_iter = 20;
        let samples = parse_xy_fixture(include_str!("testdata/rrt_star_main_seed10_samples.csv"));
        let mut sample_index = 0_usize;
        let prefix_len = 20_usize;

        let path = rrt.planning_with_sampler(|_| {
            let sample = samples
                .get(sample_index)
                .filter(|_| sample_index < prefix_len)
                .expect("python reference sample sequence exhausted");
            sample_index += 1;
            Node::new(sample[0], sample[1])
        });

        assert!(path.is_none());
        assert_eq!(sample_index, prefix_len);
        assert_eq!(rrt.node_list.len(), 14);

        let expected_nodes = [
            (
                1,
                [-0.227_015_105_864_128, 0.973_891_237_104_79],
                1.0,
                Some(0),
            ),
            (
                2,
                [0.340_848_395_898_016, 1.797_013_976_048_647],
                2.0,
                Some(1),
            ),
            (
                5,
                [2.912_922_340_312_151, 1.655_751_229_702_901],
                5.0,
                Some(4),
            ),
            (
                10,
                [5.856_411_905_724_674, 1.320_164_679_038_256],
                8.0,
                Some(9),
            ),
            (
                13,
                [8.543_112_538_843_18, 0.823_740_039_534_573],
                11.0,
                Some(12),
            ),
        ];
        for (index, xy, cost, parent) in expected_nodes {
            let node = &rrt.node_list[index];
            assert_close(node.x, xy[0]);
            assert_close(node.y, xy[1]);
            assert_close(node.cost, cost);
            assert_eq!(node.parent, parent);
        }
    }

    #[test]
    fn test_rrt_star_upstream_seeded_main_matches_pythonrobotics_reference() {
        let mut rrt = create_pythonrobotics_main_planner();
        let samples = parse_xy_fixture(include_str!("testdata/rrt_star_main_seed10_samples.csv"));
        let expected_path =
            parse_xy_fixture(include_str!("testdata/rrt_star_main_seed10_path.csv"));
        let mut sample_index = 0_usize;

        let path = rrt
            .planning_with_sampler(|_| {
                let sample = samples
                    .get(sample_index)
                    .expect("python reference sample sequence exhausted");
                sample_index += 1;
                Node::new(sample[0], sample[1])
            })
            .expect("python reference run should find a path");

        assert_eq!(sample_index, samples.len());
        assert_eq!(rrt.node_list.len(), 100);
        assert_eq!(path.len(), expected_path.len());
        for (actual, expected) in path.iter().zip(expected_path.iter()) {
            assert_close(actual[0], expected[0]);
            assert_close(actual[1], expected[1]);
        }

        let expected_nodes = [
            (
                1,
                [-0.227_015_105_864_128, 0.973_891_237_104_79],
                1.0,
                Some(0),
            ),
            (
                2,
                [0.340_848_395_898_016, 1.797_013_976_048_647],
                2.0,
                Some(1),
            ),
            (
                5,
                [2.912_922_340_312_151, 1.655_751_229_702_901],
                5.0,
                Some(4),
            ),
            (
                10,
                [5.856_411_905_724_674, 1.320_164_679_038_256],
                8.0,
                Some(9),
            ),
            (
                20,
                [12.105_226_205_468_63, 1.607_428_066_363_632],
                14.812_039_643_502_144,
                Some(19),
            ),
            (
                40,
                [13.266_098_354_827_152, 11.032_918_978_213_733],
                25.673_392_954_630_07,
                Some(39),
            ),
            (
                60,
                [8.777_150_456_317_27, 12.593_447_860_337_104],
                31.673_392_954_630_07,
                Some(53),
            ),
            (
                80,
                [10.550_895_454_349_991, 1.108_429_868_595_935],
                13.203_336_815_700_968,
                Some(17),
            ),
            (99, [6.0, 10.0], 28.741_122_081_549_424, Some(97)),
        ];

        for (index, xy, cost, parent) in expected_nodes {
            let node = &rrt.node_list[index];
            assert_close(node.x, xy[0]);
            assert_close(node.y, xy[1]);
            assert_close(node.cost, cost);
            assert_eq!(node.parent, parent);
        }
    }

    fn trace_fixture() -> RRTStar {
        RRTStar::new(
            (1.5, 1.5),
            (18.5, 18.5),
            vec![(10.0, 10.0, 2.5)],
            (0.0, 20.0),
            1.5,
            0.25,
            10,
            500,
            15.0,
            true,
            0.3,
        )
    }

    #[test]
    fn recorded_search_matches_the_same_unobserved_seeded_run() {
        let mut recorded = trace_fixture();
        let trace = recorded.planning_with_trace(7);
        let mut plain = trace_fixture();
        let mut rng = StdRng::seed_from_u64(7);
        let path = plain.planning_with_sampler(|p| p.get_random_node_with_rng(&mut rng));
        assert_eq!(trace.path, path);
        assert!(path.is_some());
        assert_eq!(
            trace.tree_at(trace.steps.len()),
            plain
                .node_list
                .iter()
                .map(RrtStarTraceNode::from)
                .collect::<Vec<_>>()
        );
        assert_eq!(
            recorded.planning_with_trace(7),
            trace,
            "a repeated run must reset all recorded state"
        );
    }

    #[test]
    fn every_recorded_prefix_has_real_parents_costs_and_feasible_improving_paths() {
        let mut planner = trace_fixture();
        let trace = planner.planning_with_trace(7);
        let mut tree = vec![trace.root.clone()];
        let mut best = f64::INFINITY;
        let mut rewires = 0;
        let mut improvements = 0;
        for step in &trace.steps {
            assert!(step.nearest < tree.len());
            let before_count = tree.len();
            for update in &step.updates {
                assert_eq!(update.before.as_ref(), tree.get(update.index));
                if let Some(before) = &update.before {
                    if before.parent != update.after.parent {
                        rewires += 1;
                    }
                }
                if update.index == tree.len() {
                    tree.push(update.after.clone());
                } else {
                    tree[update.index] = update.after.clone();
                }
            }
            assert_eq!(tree.len() - before_count, usize::from(step.accepted));
            for (i, node) in tree.iter().enumerate().skip(1) {
                let parent = node.parent.expect("non-root node has a parent");
                assert!(parent < tree.len() && parent != i);
                let p = &tree[parent];
                assert!(
                    (node.cost
                        - p.cost
                        - (node.point[0] - p.point[0]).hypot(node.point[1] - p.point[1]))
                    .abs()
                        < 1e-7
                );
                assert!(!crate::rrt::polyline_within(
                    &[p.point[0], node.point[0]],
                    &[p.point[1], node.point[1]],
                    [10.0, 10.0],
                    2.8
                ));
                let mut cursor = i;
                for depth in 0..=tree.len() {
                    match tree[cursor].parent {
                        Some(p) => cursor = p,
                        None => break,
                    }
                    assert!(depth < tree.len(), "recorded tree has a parent cycle");
                }
            }
            if !step.path.is_empty() {
                assert_eq!(step.path.first(), Some(&[18.5, 18.5]));
                assert_eq!(step.path.last(), Some(&[1.5, 1.5]));
                let length: f64 = step
                    .path
                    .windows(2)
                    .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
                    .sum();
                assert!(length <= best + 1e-7);
                if length < best - 1e-7 {
                    improvements += 1;
                }
                best = length;
            } else {
                assert!(!best.is_finite(), "a discovered path disappeared");
            }
        }
        assert!(rewires > 0 && improvements > 1);
        assert_eq!(trace.tree_at(0), vec![trace.root.clone()]);
        assert_eq!(trace.steps.last().unwrap().path, trace.path.unwrap());
    }

    #[test]
    fn trace_preserves_rejections_and_the_early_stop_condition() {
        let mut blocked = trace_fixture();
        blocked.obstacle_list = vec![(10.0, 10.0, 100.0)];
        let trace = blocked.planning_with_trace(7);
        assert_eq!(trace.steps.len(), 500);
        assert!(trace.path.is_none());
        assert!(trace
            .steps
            .iter()
            .all(|s| !s.accepted && s.updates.is_empty() && s.path.is_empty()));
        let mut early = trace_fixture();
        early.search_until_max_iter = false;
        let trace = early.planning_with_trace(7);
        assert!(trace.path.is_some() && trace.steps.len() < 500);
        assert!(trace.steps[..trace.steps.len() - 1]
            .iter()
            .all(|s| s.path.is_empty()));
    }
}
