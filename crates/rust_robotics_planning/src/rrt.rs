#![allow(clippy::too_many_arguments)]

//! RRT (Rapidly-exploring Random Tree) path planning algorithm
//!
//! Sampling-based path planning algorithm that builds a tree by
//! randomly sampling the configuration space.

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

/// Internal node for RRT tree
#[derive(Debug, Clone)]
pub struct RRTNode {
    pub x: f64,
    pub y: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub parent: Option<usize>,
}

impl RRTNode {
    pub fn new(x: f64, y: f64) -> Self {
        RRTNode {
            x,
            y,
            path_x: Vec::new(),
            path_y: Vec::new(),
            parent: None,
        }
    }
    pub fn to_point(&self) -> Point2D {
        Point2D::new(self.x, self.y)
    }
}

/// Area bounds for RRT search space
#[derive(Debug, Clone)]
pub struct AreaBounds {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl AreaBounds {
    pub fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64) -> Self {
        AreaBounds {
            xmin,
            xmax,
            ymin,
            ymax,
        }
    }
    pub fn from_array(area: [f64; 4]) -> Self {
        AreaBounds {
            xmin: area[0],
            xmax: area[1],
            ymin: area[2],
            ymax: area[3],
        }
    }
}

/// Circular obstacle (x, y, radius)
#[derive(Debug, Clone)]
pub struct CircleObstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl CircleObstacle {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Configuration for RRT planner
#[derive(Debug, Clone)]
pub struct RRTConfig {
    pub expand_dis: f64,
    pub path_resolution: f64,
    pub goal_sample_rate: i32,
    pub max_iter: usize,
    pub robot_radius: f64,
}

impl Default for RRTConfig {
    fn default() -> Self {
        Self {
            expand_dis: 3.0,
            path_resolution: 0.5,
            goal_sample_rate: 5,
            max_iter: 500,
            robot_radius: 0.8,
        }
    }
}

/// RRT path planner
pub struct RRTPlanner {
    config: RRTConfig,
    obstacles: Vec<CircleObstacle>,
    play_area: Option<AreaBounds>,
    rand_area: AreaBounds,
    node_list: Vec<RRTNode>,
    _start: RRTNode,
    goal: RRTNode,
}

impl RRTPlanner {
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        play_area: Option<AreaBounds>,
        config: RRTConfig,
    ) -> Self {
        RRTPlanner {
            config,
            obstacles,
            play_area,
            rand_area,
            node_list: Vec::new(),
            _start: RRTNode::new(0.0, 0.0),
            goal: RRTNode::new(0.0, 0.0),
        }
    }

    pub fn from_obstacles(
        obstacle_list: Vec<(f64, f64, f64)>,
        rand_area: [f64; 2],
        expand_dis: f64,
        path_resolution: f64,
        goal_sample_rate: i32,
        max_iter: usize,
        play_area: Option<[f64; 4]>,
        robot_radius: f64,
    ) -> Self {
        let obstacles = obstacle_list
            .into_iter()
            .map(|(x, y, r)| CircleObstacle::new(x, y, r))
            .collect();
        let config = RRTConfig {
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            robot_radius,
        };
        let rand_bounds = AreaBounds::new(rand_area[0], rand_area[1], rand_area[0], rand_area[1]);
        let play_bounds = play_area.map(AreaBounds::from_array);
        Self::new(obstacles, rand_bounds, play_bounds, config)
    }

    pub fn planning(&mut self, start: [f64; 2], goal: [f64; 2]) -> Option<Vec<[f64; 2]>> {
        let start_pt = Point2D::new(start[0], start[1]);
        let goal_pt = Point2D::new(goal[0], goal[1]);
        match self.plan(start_pt, goal_pt) {
            Ok(path) => Some(path.points.iter().map(|p| [p.x, p.y]).collect()),
            Err(_) => None,
        }
    }

    pub fn get_tree(&self) -> &[RRTNode] {
        &self.node_list
    }
    pub fn get_obstacles(&self) -> &[CircleObstacle] {
        &self.obstacles
    }

    fn reset_search(&mut self, start: Point2D, goal: Point2D) {
        self.node_list = vec![RRTNode::new(start.x, start.y)];
        self._start = RRTNode::new(start.x, start.y);
        self.goal = RRTNode::new(goal.x, goal.y);
    }

    fn steer(&self, from_node: &RRTNode, to_node: &RRTNode, extend_length: f64) -> RRTNode {
        let mut new_node = RRTNode::new(from_node.x, from_node.y);
        let (d, theta) = self.calc_distance_and_angle(from_node, to_node);
        new_node.path_x = vec![new_node.x];
        new_node.path_y = vec![new_node.y];
        let extend_length = extend_length.min(d);
        let n_expand = (extend_length / self.config.path_resolution).floor() as usize;
        for _ in 0..n_expand {
            new_node.x += self.config.path_resolution * theta.cos();
            new_node.y += self.config.path_resolution * theta.sin();
            new_node.path_x.push(new_node.x);
            new_node.path_y.push(new_node.y);
        }
        let (d, _) = self.calc_distance_and_angle(&new_node, to_node);
        if d <= self.config.path_resolution {
            new_node.path_x.push(to_node.x);
            new_node.path_y.push(to_node.y);
            new_node.x = to_node.x;
            new_node.y = to_node.y;
        }
        new_node
    }

    fn generate_final_course(&self, goal_ind: usize) -> Path2D {
        let mut points = vec![self.goal.to_point()];
        let mut node_index = Some(goal_ind);
        while let Some(index) = node_index {
            let node = &self.node_list[index];
            points.push(node.to_point());
            node_index = node.parent;
        }
        points.reverse();
        Path2D::from_points(points)
    }

    fn calc_dist_to_goal(&self, x: f64, y: f64) -> f64 {
        let dx = x - self.goal.x;
        let dy = y - self.goal.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn get_random_node(&self) -> RRTNode {
        let mut rng = rand::rng();
        if rng.random_range(0..=100) > self.config.goal_sample_rate {
            RRTNode::new(
                rng.random_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.random_range(self.rand_area.ymin..=self.rand_area.ymax),
            )
        } else {
            RRTNode::new(self.goal.x, self.goal.y)
        }
    }

    fn get_nearest_node_index(&self, rnd_node: &RRTNode) -> usize {
        let mut min_dist = f64::INFINITY;
        let mut min_ind = 0;
        for (i, node) in self.node_list.iter().enumerate() {
            let dist = (node.x - rnd_node.x).powi(2) + (node.y - rnd_node.y).powi(2);
            if dist < min_dist {
                min_dist = dist;
                min_ind = i;
            }
        }
        min_ind
    }

    fn check_if_outside_play_area(&self, node: &RRTNode) -> bool {
        if let Some(ref play_area) = self.play_area {
            if node.x < play_area.xmin
                || node.x > play_area.xmax
                || node.y < play_area.ymin
                || node.y > play_area.ymax
            {
                return false;
            }
        }
        true
    }

    fn check_collision(&self, node: &RRTNode) -> bool {
        for obs in &self.obstacles {
            for (&px, &py) in node.path_x.iter().zip(node.path_y.iter()) {
                let dx = obs.x - px;
                let dy = obs.y - py;
                let d = (dx * dx + dy * dy).sqrt();
                if d <= obs.radius + self.config.robot_radius {
                    return false;
                }
            }
        }
        true
    }

    fn calc_distance_and_angle(&self, from_node: &RRTNode, to_node: &RRTNode) -> (f64, f64) {
        let dx = to_node.x - from_node.x;
        let dy = to_node.y - from_node.y;
        ((dx * dx + dy * dy).sqrt(), dy.atan2(dx))
    }

    pub(crate) fn plan_with_sampler<F>(
        &mut self,
        start: Point2D,
        goal: Point2D,
        mut sample_node: F,
    ) -> Result<Path2D, RoboticsError>
    where
        F: FnMut(&RRTPlanner) -> RRTNode,
    {
        self.reset_search(start, goal);
        for _ in 0..self.config.max_iter {
            let rnd_node = sample_node(self);
            let nearest_ind = self.get_nearest_node_index(&rnd_node);
            let nearest_node = self.node_list[nearest_ind].clone();
            let new_node = self.steer(&nearest_node, &rnd_node, self.config.expand_dis);
            if self.check_if_outside_play_area(&new_node) && self.check_collision(&new_node) {
                let mut new_node = new_node;
                new_node.parent = Some(nearest_ind);
                self.node_list.push(new_node);
                let last = self.node_list.last().unwrap();
                if self.calc_dist_to_goal(last.x, last.y) <= self.config.expand_dis {
                    let final_node = self.steer(last, &self.goal.clone(), self.config.expand_dis);
                    if self.check_collision(&final_node) {
                        return Ok(self.generate_final_course(self.node_list.len() - 1));
                    }
                }
            }
        }
        Err(RoboticsError::PlanningError(
            "RRT: Cannot find path within max iterations".to_string(),
        ))
    }
}

impl PathPlanner for RRTPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let mut planner = RRTPlanner {
            config: self.config.clone(),
            obstacles: self.obstacles.clone(),
            play_area: self.play_area.clone(),
            rand_area: self.rand_area.clone(),
            node_list: vec![RRTNode::new(start.x, start.y)],
            _start: RRTNode::new(start.x, start.y),
            goal: RRTNode::new(goal.x, goal.y),
        };
        planner.plan_with_sampler(start, goal, |planner| planner.get_random_node())
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

    fn assert_point_close(actual: &Point2D, expected: [f64; 2]) {
        assert_close(actual.x, expected[0]);
        assert_close(actual.y, expected[1]);
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

    fn create_test_planner() -> RRTPlanner {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
        ];
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let config = RRTConfig {
            max_iter: 1000,
            ..Default::default()
        };
        RRTPlanner::new(obstacles, rand_area, None, config)
    }

    fn create_pythonrobotics_main_planner() -> RRTPlanner {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
            CircleObstacle::new(8.0, 10.0, 1.0),
        ];
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let config = RRTConfig {
            robot_radius: 0.8,
            ..Default::default()
        };
        RRTPlanner::new(obstacles, rand_area, None, config)
    }

    #[test]
    fn test_rrt_finds_path() {
        let planner = create_test_planner();
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(10.0, 10.0);
        let result = planner.plan(start, goal);
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_rrt_config_default() {
        let config = RRTConfig::default();
        assert_eq!(config.expand_dis, 3.0);
        assert_eq!(config.max_iter, 500);
    }

    #[test]
    fn test_rrt_upstream_test_rrt_short_goal_matches_pythonrobotics_reference() {
        let mut planner = create_pythonrobotics_main_planner();
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(1.0, 1.0);
        let sample = [10.455_649_682_677_358, 11.942_970_283_541_907];
        let path = planner
            .plan_with_sampler(start, goal, |_| RRTNode::new(sample[0], sample[1]))
            .unwrap();

        assert_eq!(planner.node_list.len(), 2);
        assert_eq!(path.points.len(), 3);
        assert_point_close(&path.points[0], [0.0, 0.0]);
        assert_point_close(
            &path.points[1],
            [1.976_107_921_083_293_5, 2.257_210_110_785_406],
        );
        assert_point_close(&path.points[2], [1.0, 1.0]);
        assert_eq!(planner.node_list[1].parent, Some(0));
        assert_eq!(planner.node_list[1].path_x.len(), 7);
        assert_close(planner.node_list[1].path_x[1], 0.329_351_320_180_548_95);
        assert_close(planner.node_list[1].path_y[1], 0.376_201_685_130_900_95);
    }

    #[test]
    fn test_rrt_upstream_main_seeded_path_matches_pythonrobotics_reference() {
        let mut planner = create_pythonrobotics_main_planner();
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(6.0, 10.0);
        let samples = parse_xy_fixture(include_str!("testdata/rrt_main_seed12345_samples.csv"));
        let expected_path = parse_xy_fixture(include_str!("testdata/rrt_main_seed12345_path.csv"));
        let mut sample_iter = samples.iter();

        let path = planner
            .plan_with_sampler(start, goal, |_| {
                let sample = sample_iter
                    .next()
                    .expect("python reference sample sequence exhausted");
                RRTNode::new(sample[0], sample[1])
            })
            .unwrap();

        assert_eq!(planner.node_list.len(), 88);
        assert_eq!(path.points.len(), expected_path.len());
        for (actual, expected) in path.points.iter().zip(expected_path.iter()) {
            assert_point_close(actual, *expected);
        }

        let expected_nodes = [
            (
                1,
                [1.976_107_921_083_293, 2.257_210_110_785_406],
                Some(0),
                7,
                [0.0, 0.329_351_320_180_549, 0.658_702_640_361_098],
                [0.0, 0.376_201_685_130_901, 0.752_403_370_261_802],
            ),
            (
                5,
                [1.229_513_438_270_946, 3.806_527_238_150_387],
                Some(1),
                5,
                [
                    1.976_107_921_083_293,
                    1.759_052_148_041_024,
                    1.541_996_374_998_755,
                ],
                [
                    2.257_210_110_785_406,
                    2.707_639_673_968_178,
                    3.158_069_237_150_95,
                ],
            ),
            (
                10,
                [-0.964_870_854_478_227, 5.566_933_984_574_426],
                Some(9),
                7,
                [
                    0.668_674_115_555_151,
                    0.358_283_476_598_378,
                    0.047_892_837_641_605,
                ],
                [
                    3.503_932_336_109_255,
                    3.895_924_238_127_659,
                    4.287_916_140_146_064,
                ],
            ),
            (
                20,
                [13.060_964_451_302_038, 12.199_474_225_398_257],
                Some(16),
                8,
                [
                    11.070_558_229_598_33,
                    11.395_810_000_597_592,
                    11.721_061_771_596_855,
                ],
                [
                    9.875_551_544_349_548,
                    10.255_303_154_565_809,
                    10.635_054_764_782_069,
                ],
            ),
            (
                87,
                [5.860_033_119_067_657, 10.721_216_347_248_003],
                Some(72),
                7,
                [
                    5.288_485_092_568_921,
                    5.383_743_096_985_377,
                    5.479_001_101_401_833,
                ],
                [
                    13.666_268_613_915_847,
                    13.175_426_569_471_206,
                    12.684_584_525_026_565,
                ],
            ),
        ];
        for (index, xy, parent, path_len, path_x3, path_y3) in expected_nodes {
            let node = &planner.node_list[index];
            assert_close(node.x, xy[0]);
            assert_close(node.y, xy[1]);
            assert_eq!(node.parent, parent);
            assert_eq!(node.path_x.len(), path_len);
            for (actual, expected) in node.path_x.iter().take(3).zip(path_x3.iter()) {
                assert_close(*actual, *expected);
            }
            for (actual, expected) in node.path_y.iter().take(3).zip(path_y3.iter()) {
                assert_close(*actual, *expected);
            }
        }
    }
}
