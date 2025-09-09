// Grid map for path planning algorithms
// Based on PythonRobotics implementation

use std::f64;

#[derive(Debug, Clone)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub cost: f64,
    pub parent_index: Option<usize>,
}

impl Node {
    pub fn new(x: i32, y: i32, cost: f64, parent_index: Option<usize>) -> Self {
        Node { x, y, cost, parent_index }
    }
}

pub struct GridMap {
    pub resolution: f64,
    pub robot_radius: f64,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub x_width: i32,
    pub y_width: i32,
    pub obstacle_map: Vec<Vec<bool>>,
}

impl GridMap {
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let min_x = ox.iter().fold(f64::INFINITY, |a, &b| a.min(b)).round();
        let min_y = oy.iter().fold(f64::INFINITY, |a, &b| a.min(b)).round();
        let max_x = ox.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)).round();
        let max_y = oy.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)).round();

        println!("min_x: {}, min_y: {}, max_x: {}, max_y: {}", min_x, min_y, max_x, max_y);

        let x_width = ((max_x - min_x) / resolution).round() as i32;
        let y_width = ((max_y - min_y) / resolution).round() as i32;

        println!("x_width: {}, y_width: {}", x_width, y_width);

        // Initialize obstacle map
        let mut obstacle_map = vec![vec![false; y_width as usize]; x_width as usize];

        // Generate obstacle map
        for ix in 0..x_width {
            let x = Self::calc_grid_position_static(ix, min_x, resolution);
            for iy in 0..y_width {
                let y = Self::calc_grid_position_static(iy, min_y, resolution);
                
                for (&iox, &ioy) in ox.iter().zip(oy.iter()) {
                    let d = ((iox - x).powi(2) + (ioy - y).powi(2)).sqrt();
                    if d <= robot_radius {
                        obstacle_map[ix as usize][iy as usize] = true;
                        break;
                    }
                }
            }
        }

        GridMap {
            resolution,
            robot_radius,
            min_x,
            min_y,
            max_x,
            max_y,
            x_width,
            y_width,
            obstacle_map,
        }
    }

    pub fn calc_xy_index(&self, position: f64) -> i32 {
        ((position - self.min_x) / self.resolution).round() as i32
    }

    pub fn calc_grid_position(&self, index: i32) -> f64 {
        index as f64 * self.resolution + self.min_x
    }

    fn calc_grid_position_static(index: i32, min_position: f64, resolution: f64) -> f64 {
        index as f64 * resolution + min_position
    }

    pub fn calc_grid_index(&self, node: &Node) -> i32 {
        (node.y - self.min_y as i32) * self.x_width + (node.x - self.min_x as i32)
    }

    pub fn verify_node(&self, node: &Node) -> bool {
        let px = self.calc_grid_position(node.x);
        let py = self.calc_grid_position(node.y);

        if px < self.min_x || py < self.min_y || px >= self.max_x || py >= self.max_y {
            return false;
        }

        // Check bounds for obstacle map access
        if node.x < 0 || node.x >= self.x_width || node.y < 0 || node.y >= self.y_width {
            return false;
        }

        // Collision check
        if self.obstacle_map[node.x as usize][node.y as usize] {
            return false;
        }

        true
    }
}
