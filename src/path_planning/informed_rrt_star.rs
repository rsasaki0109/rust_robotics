use std::f64::consts::PI;
use rand::Rng;
use gnuplot::{Figure, Caption, Color, AxesCommon};

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

pub struct InformedRRTStar {
    pub start: Node,
    pub goal: Node,
    pub min_rand: f64,
    pub max_rand: f64,
    pub expand_dis: f64,
    pub goal_sample_rate: i32,
    pub max_iter: usize,
    pub obstacle_list: Vec<(f64, f64, f64)>, // (x, y, radius)
    pub node_list: Vec<Node>,
}

impl InformedRRTStar {
    pub fn new(
        start: (f64, f64),
        goal: (f64, f64),
        obstacle_list: Vec<(f64, f64, f64)>,
        rand_area: (f64, f64),
        expand_dis: f64,
        goal_sample_rate: i32,
        max_iter: usize,
    ) -> Self {
        InformedRRTStar {
            start: Node::new(start.0, start.1),
            goal: Node::new(goal.0, goal.1),
            min_rand: rand_area.0,
            max_rand: rand_area.1,
            expand_dis,
            goal_sample_rate,
            max_iter,
            obstacle_list,
            node_list: Vec::new(),
        }
    }

    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.node_list = vec![self.start.clone()];
        let mut c_best = f64::INFINITY;
        let mut path = None;

        // Computing the sampling space
        let c_min = ((self.start.x - self.goal.x).powi(2) + (self.start.y - self.goal.y).powi(2)).sqrt();
        let x_center = [
            (self.start.x + self.goal.x) / 2.0,
            (self.start.y + self.goal.y) / 2.0,
        ];

        let a1 = [
            (self.goal.x - self.start.x) / c_min,
            (self.goal.y - self.start.y) / c_min,
        ];

        let e_theta = a1[1].atan2(a1[0]);

        // Rotation matrix for ellipse
        let cos_theta = e_theta.cos();
        let sin_theta = e_theta.sin();
        let rotation_matrix = [
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta],
        ];

        for i in 0..self.max_iter {
            if i % 100 == 0 {
                println!("Iter: {}, number of nodes: {}", i, self.node_list.len());
            }

            // Sample space is defined by c_best
            let rnd = self.informed_sample(c_best, c_min, &x_center, &rotation_matrix);
            let n_ind = self.get_nearest_list_index(&rnd);
            let nearest_node = &self.node_list[n_ind];

            // Steer
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

                if self.is_near_goal(&self.node_list[new_node_index]) {
                    if self.check_segment_collision(
                        self.node_list[new_node_index].x,
                        self.node_list[new_node_index].y,
                        self.goal.x,
                        self.goal.y,
                    ) {
                        let temp_path = self.get_final_course(new_node_index);
                        let temp_path_len = self.get_path_len(&temp_path);
                        if temp_path_len < c_best {
                            path = Some(temp_path);
                            c_best = temp_path_len;
                            println!("Found better path with cost: {:.2}", c_best);
                        }
                    }
                }
            }
        }

        path
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

    fn informed_sample(&self, c_max: f64, c_min: f64, x_center: &[f64; 2], rotation_matrix: &[[f64; 2]; 2]) -> [f64; 2] {
        if c_max < f64::INFINITY {
            let r = [
                c_max / 2.0,
                (c_max * c_max - c_min * c_min).sqrt() / 2.0,
            ];

            let x_ball = self.sample_unit_ball();
            
            // Transform unit ball sample
            let scaled = [r[0] * x_ball[0], r[1] * x_ball[1]];
            let rotated = [
                rotation_matrix[0][0] * scaled[0] + rotation_matrix[0][1] * scaled[1],
                rotation_matrix[1][0] * scaled[0] + rotation_matrix[1][1] * scaled[1],
            ];
            
            [rotated[0] + x_center[0], rotated[1] + x_center[1]]
        } else {
            self.sample_free_space()
        }
    }

    fn sample_unit_ball(&self) -> [f64; 2] {
        let mut rng = rand::thread_rng();
        let a: f64 = rng.gen();
        let b: f64 = rng.gen();

        let (a, b) = if b < a { (b, a) } else { (a, b) };

        let sample = (
            b * (2.0 * PI * a / b).cos(),
            b * (2.0 * PI * a / b).sin(),
        );
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

            let d = ((near_node.x - new_node.x).powi(2) + (near_node.y - new_node.y).powi(2)).sqrt();
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
        let t = (((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2).max(0.0).min(1.0);
        let projection = [v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])];
        (p[0] - projection[0]).powi(2) + (p[1] - projection[1]).powi(2)
    }

    fn check_segment_collision(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        for &(ox, oy, size) in &self.obstacle_list {
            let dd = self.distance_squared_point_to_segment([x1, y1], [x2, y2], [ox, oy]);
            if dd <= size * size {
                return false; // collision
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

    pub fn visualize_path(&self, path: &[[f64; 2]]) {
        let mut fg = Figure::new();
        let axes = fg.axes2d();

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
            axes.lines(&path_x, &path_y, &[Caption("Informed RRT* Path"), Color("red")]);
        }

        // Plot start and goal
        axes.points(&[self.start.x], &[self.start.y], &[Caption("Start"), Color("green")]);
        axes.points(&[self.goal.x], &[self.goal.y], &[Caption("Goal"), Color("blue")]);

        axes.set_title("Informed RRT* Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Save to file
        let output_path = "img/path_planning/informed_rrt_star_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);

        fg.show().unwrap();
    }
}

fn main() {
    println!("Informed RRT* path planning start!!");

    // Create obstacles
    let obstacle_list = vec![
        (5.0, 5.0, 0.5),
        (9.0, 6.0, 1.0),
        (7.0, 5.0, 1.0),
        (1.0, 5.0, 1.0),
        (3.0, 6.0, 1.0),
        (7.0, 9.0, 1.0),
    ];

    let mut rrt = InformedRRTStar::new(
        (0.0, 0.0),    // start
        (5.0, 10.0),   // goal
        obstacle_list,
        (-2.0, 15.0),  // rand_area
        0.5,           // expand_dis
        10,            // goal_sample_rate
        300,           // max_iter
    );

    if let Some(path) = rrt.planning() {
        println!("Found path with {} points!", path.len());
        rrt.visualize_path(&path);
    } else {
        println!("Cannot find path");
    }

    println!("Informed RRT* path planning finish!!");
}
