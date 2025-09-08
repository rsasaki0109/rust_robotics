use std::collections::{HashMap, BinaryHeap};
use std::cmp::Ordering;
use gnuplot::{Figure, Caption, Color, AxesCommon};
// Import modules directly for binary target
mod utils {
    pub mod grid_map_planner {
        include!("../utils/grid_map_planner.rs");
    }
}

// A* planner parameters
const SHOW_ANIMATION: bool = true;

#[derive(Debug, Clone)]
struct Node {
    x: i32,
    y: i32,
    cost: f64,
    parent_index: Option<usize>,
}

impl Node {
    fn new(x: i32, y: i32, cost: f64, parent_index: Option<usize>) -> Self {
        Node { x, y, cost, parent_index }
    }
}

#[derive(Debug)]
struct NodeWithPriority {
    node: Node,
    priority: f64,
    index: usize,
}

impl Eq for NodeWithPriority {}

impl PartialEq for NodeWithPriority {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Ord for NodeWithPriority {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior
        other.priority.partial_cmp(&self.priority).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for NodeWithPriority {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct AStarPlanner {
    grid_map: utils::grid_map_planner::GridMap,
    motion: Vec<(i32, i32, f64)>,
}

impl AStarPlanner {
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let grid_map = utils::grid_map_planner::GridMap::new(ox, oy, resolution, robot_radius);
        let motion = Self::get_motion_model();
        
        AStarPlanner { grid_map, motion }
    }

    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        let start_node = Node::new(
            self.grid_map.calc_xy_index(sx),
            self.grid_map.calc_xy_index(sy),
            0.0,
            None,
        );
        
        let goal_node = Node::new(
            self.grid_map.calc_xy_index(gx),
            self.grid_map.calc_xy_index(gy),
            0.0,
            None,
        );

        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashMap::new();
        let mut node_storage = Vec::new();
        
        // Add start node to storage and open set
        node_storage.push(start_node.clone());
        let start_index = node_storage.len() - 1;
        let start_grid_node = utils::grid_map_planner::Node::new(start_node.x, start_node.y, start_node.cost, start_node.parent_index);
        let start_grid_index = self.grid_map.calc_grid_index(&start_grid_node);
        
        open_set.push(NodeWithPriority {
            node: start_node.clone(),
            priority: start_node.cost + self.calc_heuristic(&start_node, &goal_node),
            index: start_index,
        });

        let mut iteration = 0;
        while let Some(current_item) = open_set.pop() {
            iteration += 1;
            if iteration % 100 == 0 {
                println!("Iteration: {}, Open set size: {}, Closed set size: {}", 
                         iteration, open_set.len(), closed_set.len());
            }
            
            let current = &current_item.node;
            let current_index = current_item.index;
            let current_grid_node = utils::grid_map_planner::Node::new(current.x, current.y, current.cost, current.parent_index);
            let current_grid_index = self.grid_map.calc_grid_index(&current_grid_node);

            // Check if we reached the goal
            if current.x == goal_node.x && current.y == goal_node.y {
                println!("Find goal after {} iterations!", iteration);
                return Some(self.calc_final_path(current_index, &node_storage));
            }

            // Move current node from open set to closed set
            closed_set.insert(current_grid_index, current_index);

            // Expand search based on motion model
            for (dx, dy, cost) in &self.motion {
                let new_x = current.x + dx;
                let new_y = current.y + dy;
                let new_cost = current.cost + cost;
                
                let new_node = Node::new(new_x, new_y, new_cost, Some(current_index));
                let new_grid_node = utils::grid_map_planner::Node::new(new_node.x, new_node.y, new_node.cost, new_node.parent_index);
                let new_grid_index = self.grid_map.calc_grid_index(&new_grid_node);

                // Skip if node is not safe
                if !self.grid_map.verify_node(&new_grid_node) {
                    continue;
                }

                // Skip if already in closed set
                if closed_set.contains_key(&new_grid_index) {
                    continue;
                }

                // Add to storage and open set
                node_storage.push(new_node.clone());
                let new_index = node_storage.len() - 1;
                
                let priority = new_node.cost + self.calc_heuristic(&new_node, &goal_node);
                open_set.push(NodeWithPriority {
                    node: new_node,
                    priority,
                    index: new_index,
                });
            }
        }

        println!("Open set is empty after {} iterations", iteration);
        None
    }

    fn calc_final_path(&self, goal_index: usize, node_storage: &[Node]) -> (Vec<f64>, Vec<f64>) {
        let mut rx = Vec::new();
        let mut ry = Vec::new();
        
        let mut current_index = Some(goal_index);
        
        while let Some(index) = current_index {
            let node = &node_storage[index];
            rx.push(self.grid_map.calc_grid_position(node.x));
            ry.push(self.grid_map.calc_grid_position(node.y));
            current_index = node.parent_index;
        }

        rx.reverse();
        ry.reverse();
        (rx, ry)
    }

    fn calc_heuristic(&self, n1: &Node, n2: &Node) -> f64 {
        let w = 1.0; // weight of heuristic
        w * ((n1.x - n2.x).pow(2) as f64 + (n1.y - n2.y).pow(2) as f64).sqrt()
    }

    fn get_motion_model() -> Vec<(i32, i32, f64)> {
        // dx, dy, cost
        vec![
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (-1, -1, 2_f64.sqrt()),
            (-1, 1, 2_f64.sqrt()),
            (1, -1, 2_f64.sqrt()),
            (1, 1, 2_f64.sqrt()),
        ]
    }
}

fn main() {
    println!("A* path planning start!!");
    
    let sx = 2.0; // start x position [m]
    let sy = 2.0; // start y position [m]
    let gx = 8.0; // goal x position [m]
    let gy = 8.0; // goal y position [m]
    let grid_size = 1.0; // grid size [m]
    let robot_radius = 0.5; // robot radius [m]
    
    println!("Setting up simple test environment...");
    
    // Set obstacle positions (simple test case)
    let mut ox = Vec::new();
    let mut oy = Vec::new();
    
    // Create boundary obstacles
    for i in 0..11 {
        ox.push(i as f64);
        oy.push(0.0);
        ox.push(i as f64);
        oy.push(10.0);
        ox.push(0.0);
        oy.push(i as f64);
        ox.push(10.0);
        oy.push(i as f64);
    }
    
    // Add a simple internal obstacle
    for i in 4..7 {
        ox.push(5.0);
        oy.push(i as f64);
    }
    
    println!("Created {} obstacles", ox.len());

    let a_star = AStarPlanner::new(&ox, &oy, grid_size, robot_radius);
    
    if let Some((rx, ry)) = a_star.planning(sx, sy, gx, gy) {
        println!("Path found with {} points", rx.len());
        
        if SHOW_ANIMATION {
            let mut fg = Figure::new();
            fg.axes2d()
                .points(&ox, &oy, &[Caption("Obstacles"), Color("black")])
                .points(&[sx], &[sy], &[Caption("Start"), Color("green")])
                .points(&[gx], &[gy], &[Caption("Goal"), Color("blue")])
                .lines(&rx, &ry, &[Caption("Path"), Color("red")])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
                .set_title("A* Path Planning", &[])
                .set_x_label("X [m]", &[])
                .set_y_label("Y [m]", &[]);
            
            // Save to file
            let output_path = "img/path_planning/a_star_result.png";
            fg.save_to_png(output_path, 800, 600).unwrap();
            println!("Plot saved to: {}", output_path);
            
            // Also show the plot
            fg.show().unwrap();
        }
    } else {
        println!("No path found!");
    }
    
    println!("A* path planning finish!!");
}
