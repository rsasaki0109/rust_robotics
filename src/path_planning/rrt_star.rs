use std::f64::consts::PI;
use rand::Rng;
use gnuplot::{Figure, Caption, Color, AxesCommon};

// RRT* planner parameters
const SHOW_ANIMATION: bool = true;

#[derive(Debug, Clone)]
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
        self.node_list = vec![self.start.clone()];
        
        for i in 0..self.max_iter {
            if i % 100 == 0 {
                println!("Iter: {}, number of nodes: {}", i, self.node_list.len());
            }
            
            let rnd_node = self.get_random_node();
            let nearest_ind = self.get_nearest_node_index(&rnd_node);
            let mut new_node = self.steer(nearest_ind, &rnd_node);
            
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
                    
                    if !self.search_until_max_iter {
                        if let Some(last_index) = self.search_best_goal_node() {
                            return Some(self.generate_final_course(last_index));
                        }
                    }
                }
            }
        }
        
        println!("Reached max iteration");
        if let Some(last_index) = self.search_best_goal_node() {
            return Some(self.generate_final_course(last_index));
        }
        
        None
    }

    fn get_random_node(&self) -> Node {
        let mut rng = rand::thread_rng();
        
        if rng.gen_range(0..100) > self.goal_sample_rate {
            Node::new(
                rng.gen_range(self.min_rand..self.max_rand),
                rng.gen_range(self.min_rand..self.max_rand),
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
        let (dist, theta) = self.calc_distance_and_angle(from_node, to_node);
        
        let extend_length = if dist > self.expand_dis {
            self.expand_dis
        } else {
            dist
        };
        
        let mut new_node = Node::new(
            from_node.x + extend_length * theta.cos(),
            from_node.y + extend_length * theta.sin(),
        );
        new_node.parent = Some(from_ind);
        
        Some(new_node)
    }

    fn check_collision_free(&self, node: &Node) -> bool {
        if let Some(parent_ind) = node.parent {
            let parent = &self.node_list[parent_ind];
            
            for &(ox, oy, size) in &self.obstacle_list {
                let dx_list = self.get_path_x(parent, node);
                let dy_list = self.get_path_y(parent, node);
                
                for (&dx, &dy) in dx_list.iter().zip(dy_list.iter()) {
                    let d = (dx - ox).powi(2) + (dy - oy).powi(2);
                    if d <= (size + self.robot_radius).powi(2) {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn get_path_x(&self, from_node: &Node, to_node: &Node) -> Vec<f64> {
        let (dist, theta) = self.calc_distance_and_angle(from_node, to_node);
        let n_expand = (dist / self.path_resolution).floor() as i32;
        
        (0..=n_expand)
            .map(|i| from_node.x + self.path_resolution * i as f64 * theta.cos())
            .collect()
    }

    fn get_path_y(&self, from_node: &Node, to_node: &Node) -> Vec<f64> {
        let (dist, theta) = self.calc_distance_and_angle(from_node, to_node);
        let n_expand = (dist / self.path_resolution).floor() as i32;
        
        (0..=n_expand)
            .map(|i| from_node.y + self.path_resolution * i as f64 * theta.sin())
            .collect()
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
            let mut t_node = Node::new(new_node.x, new_node.y);
            t_node.parent = Some(i);
            
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
        
        let mut result_node = Node::new(new_node.x, new_node.y);
        result_node.parent = Some(parent_ind);
        result_node.cost = min_cost;
        
        Some(result_node)
    }

    fn search_best_goal_node(&self) -> Option<usize> {
        let dist_to_goal_list: Vec<f64> = self.node_list
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
                let mut t_node = Node::new(self.end.x, self.end.y);
                t_node.parent = Some(goal_ind);
                self.check_collision_free(&t_node)
            })
            .collect();
        
        if safe_goal_inds.is_empty() {
            return None;
        }
        
        let safe_goal_costs: Vec<f64> = safe_goal_inds
            .iter()
            .map(|&i| {
                self.node_list[i].cost + self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
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
            
            let mut edge_node = Node::new(near_node.x, near_node.y);
            edge_node.parent = Some(new_node_ind);
            edge_node.cost = self.calc_new_cost(new_node, &near_node);
            
            let no_collision = self.check_collision_free(&edge_node);
            let improved_cost = near_node.cost > edge_node.cost;
            
            if no_collision && improved_cost {
                // Update parent references
                for node in &mut self.node_list {
                    if let Some(parent_ind) = node.parent {
                        if parent_ind == i {
                            node.parent = Some(new_node_ind);
                        }
                    }
                }
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
                    self.node_list[i].cost = self.calc_new_cost(&parent_node, &self.node_list[i].clone());
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
        
        path.reverse();
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

    pub fn visualize_path(&self, path: &[[f64; 2]]) {
        if !SHOW_ANIMATION {
            return;
        }

        let mut fg = Figure::new();
        let mut axes = fg.axes2d();
        
        // Plot obstacles
        let obs_x: Vec<f64> = self.obstacle_list.iter().map(|obs| obs.0).collect();
        let obs_y: Vec<f64> = self.obstacle_list.iter().map(|obs| obs.1).collect();
        axes.points(&obs_x, &obs_y, &[Caption("Obstacles"), Color("black")]);
        
        // Plot tree
        for node in &self.node_list {
            if let Some(parent_index) = node.parent {
                let parent = &self.node_list[parent_index];
                axes.lines(&[parent.x, node.x], &[parent.y, node.y], &[Color("blue")]);
            }
        }
        
        // Plot path
        if !path.is_empty() {
            let path_x: Vec<f64> = path.iter().map(|node| node[0]).collect();
            let path_y: Vec<f64> = path.iter().map(|node| node[1]).collect();
            axes.lines(&path_x, &path_y, &[Caption("RRT* Path"), Color("red")]);
        }
        
        // Plot start and goal
        axes.points(&[self.start.x], &[self.start.y], &[Caption("Start"), Color("green")]);
        axes.points(&[self.end.x], &[self.end.y], &[Caption("Goal"), Color("blue")]);
        
        axes.set_title("RRT* Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
        
        // Save to file
        let output_path = "img/path_planning/rrt_star_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);
        
        fg.show().unwrap();
    }
}

fn main() {
    println!("RRT* path planning start!!");

    // Obstacle list [x, y, radius]
    let obstacle_list = vec![
        (5.0, 5.0, 1.0),
        (3.0, 6.0, 2.0),
        (3.0, 8.0, 2.0),
        (3.0, 10.0, 2.0),
        (7.0, 5.0, 2.0),
        (9.0, 5.0, 2.0),
        (8.0, 10.0, 1.0),
        (6.0, 12.0, 1.0),
    ];

    let mut rrt_star = RRTStar::new(
        (0.0, 0.0),    // start
        (6.0, 10.0),   // goal
        obstacle_list,
        (-2.0, 15.0),  // rand_area
        3.0,           // expand_dis (increased)
        0.5,           // path_resolution (decreased for smoother path)
        10,            // goal_sample_rate (decreased for more exploration)
        500,           // max_iter (increased)
        30.0,          // connect_circle_dist (decreased)
        true,          // search_until_max_iter (changed to true)
        0.5,           // robot_radius (decreased)
    );

    if let Some(path) = rrt_star.planning() {
        println!("Found path with {} points!", path.len());
        rrt_star.visualize_path(&path);
    } else {
        println!("Cannot find path");
    }

    println!("RRT* path planning finish!!");
}
