use rand::Rng;
use gnuplot::{Figure, Caption, Color, PointSymbol, AxesCommon};
use std::f64::consts::PI;

const SHOW_ANIMATION: bool = true;

#[derive(Debug, Clone)]
pub struct Node {
    pub x: f64,
    pub y: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub parent: Option<usize>,
}

impl Node {
    pub fn new(x: f64, y: f64) -> Self {
        Node {
            x,
            y,
            path_x: Vec::new(),
            path_y: Vec::new(),
            parent: None,
        }
    }
}

#[derive(Debug)]
pub struct AreaBounds {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl AreaBounds {
    pub fn new(area: [f64; 4]) -> Self {
        AreaBounds {
            xmin: area[0],
            xmax: area[1],
            ymin: area[2],
            ymax: area[3],
        }
    }
}

pub struct RRT {
    start: Node,
    end: Node,
    min_rand: f64,
    max_rand: f64,
    play_area: Option<AreaBounds>,
    expand_dis: f64,
    path_resolution: f64,
    goal_sample_rate: i32,
    max_iter: usize,
    obstacle_list: Vec<(f64, f64, f64)>, // (x, y, radius)
    node_list: Vec<Node>,
    robot_radius: f64,
}

impl RRT {
    pub fn new(
        start: [f64; 2],
        goal: [f64; 2],
        obstacle_list: Vec<(f64, f64, f64)>,
        rand_area: [f64; 2],
        expand_dis: f64,
        path_resolution: f64,
        goal_sample_rate: i32,
        max_iter: usize,
        play_area: Option<[f64; 4]>,
        robot_radius: f64,
    ) -> Self {
        let play_area = play_area.map(AreaBounds::new);
        
        RRT {
            start: Node::new(start[0], start[1]),
            end: Node::new(goal[0], goal[1]),
            min_rand: rand_area[0],
            max_rand: rand_area[1],
            play_area,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            obstacle_list,
            node_list: Vec::new(),
            robot_radius,
        }
    }

    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.node_list = vec![self.start.clone()];
        
        for i in 0..self.max_iter {
            let rnd_node = self.get_random_node();
            let nearest_ind = self.get_nearest_node_index(&rnd_node);
            let nearest_node = self.node_list[nearest_ind].clone();

            let new_node = self.steer(&nearest_node, &rnd_node, self.expand_dis);

            if self.check_if_outside_play_area(&new_node) && 
               self.check_collision(&new_node) {
                let new_index = self.node_list.len();
                let mut new_node = new_node;
                new_node.parent = Some(nearest_ind);
                self.node_list.push(new_node);

                if self.calc_dist_to_goal(self.node_list.last().unwrap().x, 
                                         self.node_list.last().unwrap().y) <= self.expand_dis {
                    let final_node = self.steer(self.node_list.last().unwrap(), &self.end, self.expand_dis);
                    if self.check_collision(&final_node) {
                        return Some(self.generate_final_course(self.node_list.len() - 1));
                    }
                }
            }

            if SHOW_ANIMATION && i % 5 == 0 {
                // Animation would go here
                println!("RRT iteration: {}", i);
            }
        }

        None // Cannot find path
    }

    fn steer(&self, from_node: &Node, to_node: &Node, extend_length: f64) -> Node {
        let mut new_node = Node::new(from_node.x, from_node.y);
        let (d, theta) = self.calc_distance_and_angle(from_node, to_node);

        new_node.path_x = vec![new_node.x];
        new_node.path_y = vec![new_node.y];

        let extend_length = if extend_length > d { d } else { extend_length };
        let n_expand = (extend_length / self.path_resolution).floor() as usize;

        for _ in 0..n_expand {
            new_node.x += self.path_resolution * theta.cos();
            new_node.y += self.path_resolution * theta.sin();
            new_node.path_x.push(new_node.x);
            new_node.path_y.push(new_node.y);
        }

        let (d, _) = self.calc_distance_and_angle(&new_node, to_node);
        if d <= self.path_resolution {
            new_node.path_x.push(to_node.x);
            new_node.path_y.push(to_node.y);
            new_node.x = to_node.x;
            new_node.y = to_node.y;
        }

        new_node
    }

    fn generate_final_course(&self, goal_ind: usize) -> Vec<[f64; 2]> {
        let mut path = vec![[self.end.x, self.end.y]];
        let mut node_index = Some(goal_ind);
        
        while let Some(index) = node_index {
            let node = &self.node_list[index];
            path.push([node.x, node.y]);
            node_index = node.parent;
        }

        path.reverse();
        path
    }

    fn calc_dist_to_goal(&self, x: f64, y: f64) -> f64 {
        let dx = x - self.end.x;
        let dy = y - self.end.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn get_random_node(&self) -> Node {
        let mut rng = rand::thread_rng();
        
        if rng.gen_range(0..=100) > self.goal_sample_rate {
            Node::new(
                rng.gen_range(self.min_rand..=self.max_rand),
                rng.gen_range(self.min_rand..=self.max_rand),
            )
        } else {
            // Goal point sampling
            Node::new(self.end.x, self.end.y)
        }
    }

    fn get_nearest_node_index(&self, rnd_node: &Node) -> usize {
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

    fn check_if_outside_play_area(&self, node: &Node) -> bool {
        if let Some(ref play_area) = self.play_area {
            if node.x < play_area.xmin || node.x > play_area.xmax ||
               node.y < play_area.ymin || node.y > play_area.ymax {
                return false; // Outside - bad
            }
        }
        true // Inside or no play area - ok
    }

    fn check_collision(&self, node: &Node) -> bool {
        for &(ox, oy, size) in &self.obstacle_list {
            for (&px, &py) in node.path_x.iter().zip(node.path_y.iter()) {
                let dx = ox - px;
                let dy = oy - py;
                let d = (dx * dx + dy * dy).sqrt();
                if d <= size + self.robot_radius {
                    return false; // Collision
                }
            }
        }
        true // Safe
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
            axes.lines(&path_x, &path_y, &[Caption("Path"), Color("red")]);
        }
        
        // Plot start and goal
        axes.points(&[self.start.x], &[self.start.y], &[Caption("Start"), Color("green")]);
        axes.points(&[self.end.x], &[self.end.y], &[Caption("Goal"), Color("blue")]);
        
        axes.set_title("RRT Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
        
        // Save to file
        let output_path = "img/path_planning/rrt_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);
        
        fg.show().unwrap();
    }
}

fn main() {
    println!("RRT path planning start!!");

    // Obstacle list [x, y, radius]
    let obstacle_list = vec![
        (5.0, 5.0, 1.0),
        (3.0, 6.0, 2.0),
        (3.0, 8.0, 2.0),
        (3.0, 10.0, 2.0),
        (7.0, 5.0, 2.0),
        (9.0, 5.0, 2.0),
        (8.0, 10.0, 1.0),
    ];

    let mut rrt = RRT::new(
        [0.0, 0.0],        // start
        [6.0, 10.0],       // goal
        obstacle_list,
        [-2.0, 15.0],      // rand_area
        3.0,               // expand_dis
        0.5,               // path_resolution
        5,                 // goal_sample_rate
        500,               // max_iter
        None,              // play_area
        0.8,               // robot_radius
    );

    if let Some(path) = rrt.planning() {
        println!("Found path with {} points!", path.len());
        rrt.visualize_path(&path);
    } else {
        println!("Cannot find path");
    }
}
