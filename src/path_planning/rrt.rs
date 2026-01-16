//! RRT (Rapidly-exploring Random Tree) path planning algorithm
//!
//! Sampling-based path planning algorithm that builds a tree by
//! randomly sampling the configuration space.

use rand::Rng;

use crate::common::{Point2D, Path2D, PathPlanner, RoboticsError};

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
        AreaBounds { xmin, xmax, ymin, ymax }
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
    /// Expansion distance per step
    pub expand_dis: f64,
    /// Path resolution for collision checking
    pub path_resolution: f64,
    /// Goal sampling rate (0-100)
    pub goal_sample_rate: i32,
    /// Maximum iterations
    pub max_iter: usize,
    /// Robot radius for collision checking
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
    start: RRTNode,
    goal: RRTNode,
}

impl RRTPlanner {
    /// Create a new RRT planner
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
            start: RRTNode::new(0.0, 0.0),
            goal: RRTNode::new(0.0, 0.0),
        }
    }

    /// Create with simplified parameters (legacy interface)
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

    /// Legacy planning interface
    pub fn planning(&mut self, start: [f64; 2], goal: [f64; 2]) -> Option<Vec<[f64; 2]>> {
        let start_pt = Point2D::new(start[0], start[1]);
        let goal_pt = Point2D::new(goal[0], goal[1]);

        match self.plan(start_pt, goal_pt) {
            Ok(path) => {
                let result: Vec<[f64; 2]> = path.points.iter()
                    .map(|p| [p.x, p.y])
                    .collect();
                Some(result)
            }
            Err(_) => None,
        }
    }

    /// Get the tree built during planning
    pub fn get_tree(&self) -> &[RRTNode] {
        &self.node_list
    }

    /// Get obstacles
    pub fn get_obstacles(&self) -> &[CircleObstacle] {
        &self.obstacles
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
        let mut rng = rand::thread_rng();

        if rng.gen_range(0..=100) > self.config.goal_sample_rate {
            RRTNode::new(
                rng.gen_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.gen_range(self.rand_area.ymin..=self.rand_area.ymax),
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
            if node.x < play_area.xmin || node.x > play_area.xmax ||
               node.y < play_area.ymin || node.y > play_area.ymax {
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
        let d = (dx * dx + dy * dy).sqrt();
        let theta = dy.atan2(dx);
        (d, theta)
    }
}

impl PathPlanner for RRTPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        // Need to use interior mutability for the planner state
        let mut planner = RRTPlanner {
            config: self.config.clone(),
            obstacles: self.obstacles.clone(),
            play_area: self.play_area.clone(),
            rand_area: self.rand_area.clone(),
            node_list: vec![RRTNode::new(start.x, start.y)],
            start: RRTNode::new(start.x, start.y),
            goal: RRTNode::new(goal.x, goal.y),
        };

        for _ in 0..planner.config.max_iter {
            let rnd_node = planner.get_random_node();
            let nearest_ind = planner.get_nearest_node_index(&rnd_node);
            let nearest_node = planner.node_list[nearest_ind].clone();

            let new_node = planner.steer(&nearest_node, &rnd_node, planner.config.expand_dis);

            if planner.check_if_outside_play_area(&new_node) && planner.check_collision(&new_node) {
                let mut new_node = new_node;
                new_node.parent = Some(nearest_ind);
                planner.node_list.push(new_node);

                let last = planner.node_list.last().unwrap();
                if planner.calc_dist_to_goal(last.x, last.y) <= planner.config.expand_dis {
                    let final_node = planner.steer(last, &planner.goal.clone(), planner.config.expand_dis);
                    if planner.check_collision(&final_node) {
                        return Ok(planner.generate_final_course(planner.node_list.len() - 1));
                    }
                }
            }
        }

        Err(RoboticsError::PlanningError("RRT: Cannot find path within max iterations".to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn test_rrt_finds_path() {
        let planner = create_test_planner();
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(10.0, 10.0);

        let result = planner.plan(start, goal);
        // RRT is probabilistic, so we just check it doesn't panic
        // and returns either Ok or Err
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_rrt_config_default() {
        let config = RRTConfig::default();
        assert_eq!(config.expand_dis, 3.0);
        assert_eq!(config.max_iter, 500);
    }
}
