use std::collections::{BinaryHeap, HashSet};
use std::f64::INFINITY;
use std::cmp::Ordering;
use gnuplot::{Figure, Caption, Color, AxesCommon};
use nalgebra as na;

// D* Lite parameters
const SHOW_ANIMATION: bool = true;
const PAUSE_TIME: f64 = 0.1;  // アニメーションの表示間隔を少し長くする
const P_CREATE_RANDOM_OBSTACLE: f64 = 0.0;
const DEBUG: bool = true;  // デバッグログを有効にする

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Node {
    x: i32,
    y: i32,
}

impl Node {
    pub fn new(x: i32, y: i32) -> Self {
        // デバッグ情報を表示
        if DEBUG {
            println!("Creating Node with x: {}, y: {}", x, y);
        }
        Node { x, y }
    }

    fn add(&self, other: &Node) -> Node {
        Node {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    fn distance_to(&self, other: &Node) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Key {
    k1: f64,
    k2: f64,
}

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.k1.partial_cmp(&other.k1) {
            Some(Ordering::Equal) => self.k2.partial_cmp(&other.k2).unwrap_or(Ordering::Equal),
            Some(ord) => ord,
            None => Ordering::Equal,
        }
    }
}

#[derive(Debug)]
struct DStarLite {
    // Grid dimensions
    x_min_world: i32,
    y_min_world: i32,
    x_max: i32,
    y_max: i32,
    
    // Start and goal positions
    start: Node,
    goal: Node,
    
    // Algorithm state
    km: f64,
    rhs: na::DMatrix<f64>,
    g: na::DMatrix<f64>,
    
    // Priority queue
    u: BinaryHeap<(Key, Node)>,
    
    // Obstacles
    obstacles: HashSet<(i32, i32)>,
    detected_obstacles: HashSet<(i32, i32)>,
    
    // Motion model (8-connected grid)
    motions: [Node; 8],
    
    // For visualization
    detected_obstacles_for_plotting: Vec<(f64, f64)>,
    
    // State
    initialized: bool,
}

impl DStarLite {
    fn new(ox: &[i32], oy: &[i32]) -> Self {
        let x_min_world = *ox.iter().min().unwrap_or(&0);
        let y_min_world = *oy.iter().min().unwrap_or(&0);
        let x_max = ox.iter().max().unwrap_or(&0) - x_min_world + 1;
        let y_max = oy.iter().max().unwrap_or(&0) - y_min_world + 1;
        
        let obstacles: HashSet<_> = ox.iter()
            .zip(oy.iter())
            .map(|(&x, &y)| (x - x_min_world, y - y_min_world))
            .collect();
        
        let g = na::DMatrix::from_element(x_max as usize, y_max as usize, INFINITY);
        let rhs = na::DMatrix::from_element(x_max as usize, y_max as usize, INFINITY);
        
        let motions = [
            Node::new(1, 0),   // Right
            Node::new(0, 1),    // Down
            Node::new(-1, 0),   // Left
            Node::new(0, -1),   // Up
            Node::new(1, 1),    // Down-Right
            Node::new(1, -1),   // Up-Right
            Node::new(-1, 1),   // Down-Left
            Node::new(-1, -1),  // Up-Left
        ];
        
        DStarLite {
            x_min_world,
            y_min_world,
            x_max,
            y_max,
            start: Node::new(0, 0),
            goal: Node::new(0, 0),
            km: 0.0,
            rhs,
            g,
            u: BinaryHeap::new(),
            obstacles,
            detected_obstacles: HashSet::new(),
            motions,
            detected_obstacles_for_plotting: Vec::new(),
            initialized: false,
        }
    }
    
    fn is_valid(&self, node: &Node) -> bool {
        node.x >= 0 && node.x < self.x_max && node.y >= 0 && node.y < self.y_max
    }
    
    fn is_obstacle(&self, node: &Node) -> bool {
        self.obstacles.contains(&(node.x, node.y)) || 
        self.detected_obstacles.contains(&(node.x, node.y))
    }
    
    fn get_neighbors(&self, node: &Node) -> Vec<Node> {
        self.motions.iter()
            .map(|m| node.add(m))
            .filter(|n| self.is_valid(n))
            .collect()
    }
    
    fn calculate_key(&self, node: &Node) -> Key {
        let min_rhs_g = self.g[(node.x as usize, node.y as usize)].min(self.rhs[(node.x as usize, node.y as usize)]);
        let h = self.heuristic(&self.start, node);
        Key {
            k1: min_rhs_g + h + self.km,
            k2: min_rhs_g,
        }
    }
    
    fn heuristic(&self, a: &Node, b: &Node) -> f64 {
        let dx = (a.x - b.x).abs() as f64;
        let dy = (a.y - b.y).abs() as f64;
        (dx * dx + dy * dy).sqrt()
    }
    
    fn c(&self, u: &Node, v: &Node) -> f64 {
        if self.is_obstacle(v) {
            return INFINITY;
        }
        self.heuristic(u, v)
    }
    
    fn initialize(&mut self, start: Node, goal: Node) {
        self.start = Node::new(start.x - self.x_min_world, start.y - self.y_min_world);
        self.goal = Node::new(goal.x - self.x_min_world, goal.y - self.y_min_world);
        
        if !self.initialized {
            self.initialized = true;
            self.u.clear();
            self.km = 0.0;
            self.rhs = na::DMatrix::from_element(self.x_max as usize, self.y_max as usize, INFINITY);
            self.g = na::DMatrix::from_element(self.x_max as usize, self.y_max as usize, INFINITY);
            self.rhs[(self.goal.x as usize, self.goal.y as usize)] = 0.0;
            self.u.push((self.calculate_key(&self.goal), self.goal));
            self.detected_obstacles.clear();
        }
    }
    
    fn update_vertex(&mut self, u: Node) {
        if u.x != self.goal.x || u.y != self.goal.y {
            let mut min_rhs = INFINITY;
            for s in self.get_neighbors(&u) {
                let cost = self.c(&u, &s) + self.g[(s.x as usize, s.y as usize)];
                if cost < min_rhs {
                    min_rhs = cost;
                }
            }
            self.rhs[(u.x as usize, u.y as usize)] = min_rhs;
        }
        
        // Remove u from U if it's there
        self.u = std::mem::take(&mut self.u)
            .into_iter()
            .filter(|(_, node)| node.x != u.x || node.y != u.y)
            .collect();
            
        // If g(u) != rhs(u), add u to U with key(u)
        if (self.g[(u.x as usize, u.y as usize)] - self.rhs[(u.x as usize, u.y as usize)]).abs() > 1e-6 {
            self.u.push((self.calculate_key(&u), u));
        }
    }
    
    fn compute_shortest_path(&mut self) {
        if DEBUG {
            println!("Starting compute_shortest_path");
            println!("Queue size: {}", self.u.len());
        }
        while !self.u.is_empty() {
            let (k_old, u) = self.u.pop().unwrap();
            
            if k_old < self.calculate_key(&u) {
                self.u.push((self.calculate_key(&u), u));
            } else if self.g[(u.x as usize, u.y as usize)] > self.rhs[(u.x as usize, u.y as usize)] {
                self.g[(u.x as usize, u.y as usize)] = self.rhs[(u.x as usize, u.y as usize)];
                
                for s in self.get_neighbors(&u) {
                    self.update_vertex(s);
                }
            } else {
                self.g[(u.x as usize, u.y as usize)] = INFINITY;
                self.update_vertex(u);
                for s in self.get_neighbors(&u) {
                    self.update_vertex(s);
                }
            }
        }
    }
    
    fn plan(&mut self, start: Node, goal: Node) -> Option<Vec<Node>> {
        self.initialize(start, goal);
        
        // First planning
        self.compute_shortest_path();
        
        // Reconstruct path
        self.reconstruct_path()
    }
    
    fn reconstruct_path(&self) -> Option<Vec<Node>> {
        let mut path = Vec::new();
        let mut current = self.start;
        
        while current.x != self.goal.x || current.y != self.goal.y {
            path.push(Node::new(
                current.x + self.x_min_world,
                current.y + self.y_min_world,
            ));
            
            let mut min_cost = INFINITY;
            let mut next_node = current;
            
            for neighbor in self.get_neighbors(&current) {
                let cost = self.c(&current, &neighbor) + self.g[(neighbor.x as usize, neighbor.y as usize)];
                if cost < min_cost {
                    min_cost = cost;
                    next_node = neighbor;
                }
            }
            
            if next_node.x == current.x && next_node.y == current.y {
                // No path found
                return None;
            }
            
            current = next_node;
        }
        
        // Add the goal node
        path.push(Node::new(
            self.goal.x + self.x_min_world,
            self.goal.y + self.y_min_world,
        ));
        
        Some(path)
    }
    
    fn add_obstacle(&mut self, x: i32, y: i32) {
        let node = Node::new(x - self.x_min_world, y - self.y_min_world);
        if self.is_valid(&node) {
            self.detected_obstacles.insert((node.x, node.y));
            self.detected_obstacles_for_plotting.push((x as f64, y as f64));
            self.update_vertex(node);
        }
    }
}

fn main() {
    println!("D* Lite path planning");
    
    // マップサイズを小さくする
    const MAP_SIZE: i32 = 20;
    
    // 障害物の位置（境界）
    let mut ox: Vec<i32> = (0..MAP_SIZE).map(|x| x)
        .chain(std::iter::repeat(MAP_SIZE).take((MAP_SIZE + 1) as usize))
        .chain((0..=MAP_SIZE).rev())
        .chain(std::iter::repeat(0).take((MAP_SIZE + 1) as usize))
        .collect();
        
    let mut oy: Vec<i32> = std::iter::repeat(0).take((MAP_SIZE + 1) as usize)
        .chain((0..MAP_SIZE).map(|y| y))
        .chain(std::iter::repeat(MAP_SIZE).take((MAP_SIZE + 1) as usize))
        .chain((0..=MAP_SIZE).rev())
        .collect();
    
    // 中央に障害物を追加
    for i in 5..15 {
        ox.push(i);
        oy.push(10);
    }
    
    // スタートとゴールの位置（マップサイズに合わせて調整）
    let sx = 2.0;
    let sy = 2.0;
    let gx = (MAP_SIZE - 2) as f64;
    let gy = (MAP_SIZE - 2) as f64;
    
    if DEBUG {
        println!("Map size: {}x{}", MAP_SIZE, MAP_SIZE);
        println!("Start: ({}, {})", sx, sy);
        println!("Goal: ({}, {})", gx, gy);
        println!("Number of obstacles: {}", ox.len());
    }
    
    let mut d_star_lite = DStarLite::new(&ox, &oy);
    
    // 初期パスの計画
    if DEBUG {
        println!("Planning initial path...");
    }
    
    let start_time = std::time::Instant::now();
    let path = d_star_lite.plan(
        Node::new(sx as i32, sy as i32),
        Node::new(gx as i32, gy as i32),
    );
    
    if DEBUG {
        let elapsed = start_time.elapsed();
        println!("Path planning took: {:.2?}", elapsed);
    }
    
    // Visualization
    if SHOW_ANIMATION {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d();
            
            // Plot obstacles
            axes.points(
                ox.iter().map(|&x| x as f64),
                oy.iter().map(|&y| y as f64),
                &[Caption("Obstacle"), Color("black")],
            );
            
            // Plot start and goal
            axes.points(
                &[sx],
                &[sy],
                &[Caption("Start"), Color("blue")],
            );
            
            axes.points(
                &[gx],
                &[gy],
                &[Caption("Goal"), Color("red")],
            );
            
            // Plot path if found
            if let Some(path) = &path {
                let path_x: Vec<f64> = path.iter().map(|n| n.x as f64).collect();
                let path_y: Vec<f64> = path.iter().map(|n| n.y as f64).collect();
                
                axes.lines(
                    &path_x,
                    &path_y,
                    &[Caption("Path"), Color("green")],
                );
            }
            
            // Plot detected obstacles
            if !d_star_lite.detected_obstacles_for_plotting.is_empty() {
                let (detected_x, detected_y): (Vec<f64>, Vec<f64>) = 
                    d_star_lite.detected_obstacles_for_plotting.iter().cloned().unzip();
                
                axes.points(
                    detected_x,
                    detected_y,
                    &[Caption("Detected"), Color("yellow")],
                );
            }
            
            axes.set_title("D* Lite Path Planning", &[]);
            axes.set_x_label("X [m]", &[]);
            axes.set_y_label("Y [m]", &[]);
            // シンプルな軸設定
            axes.set_x_ticks(Some((gnuplot::Fix(20.0), 10)), &[], &[]);
            axes.set_y_ticks(Some((gnuplot::Fix(20.0), 10)), &[], &[]);
        }
        
        // 画像をファイルに保存
        let img_path = "d_star_lite_result.png";
        if let Err(e) = fg.save_to_png(img_path, 800, 600) {
            eprintln!("Failed to save image: {}", e);
        } else if DEBUG {
            println!("Image saved to: {}", img_path);
        }
        
        // 画像を表示
        fg.show();
    }
    
    if let Some(path) = path {
        println!("Path found with {} points", path.len());
    } else {
        println!("No path found!");
    }
}
