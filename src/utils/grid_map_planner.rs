// Grid map for path planning algorithms
//
// Provides occupancy grid representation and utility functions
// for grid-based path planning algorithms like A*, Dijkstra, D* Lite.

/// Node for grid-based path search
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

/// 8-connected motion model (dx, dy, cost)
pub fn get_motion_model_8() -> Vec<(i32, i32, f64)> {
    vec![
        (1, 0, 1.0),
        (0, 1, 1.0),
        (-1, 0, 1.0),
        (0, -1, 1.0),
        (-1, -1, std::f64::consts::SQRT_2),
        (-1, 1, std::f64::consts::SQRT_2),
        (1, -1, std::f64::consts::SQRT_2),
        (1, 1, std::f64::consts::SQRT_2),
    ]
}

/// 4-connected motion model (dx, dy, cost)
pub fn get_motion_model_4() -> Vec<(i32, i32, f64)> {
    vec![
        (1, 0, 1.0),
        (0, 1, 1.0),
        (-1, 0, 1.0),
        (0, -1, 1.0),
    ]
}

/// Occupancy grid map for path planning
#[derive(Debug, Clone)]
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
    /// Create a new grid map from obstacle points
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let min_x = ox.iter().fold(f64::INFINITY, |a, &b| a.min(b)).round();
        let min_y = oy.iter().fold(f64::INFINITY, |a, &b| a.min(b)).round();
        let max_x = ox.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)).round();
        let max_y = oy.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)).round();

        let x_width = ((max_x - min_x) / resolution).round() as i32;
        let y_width = ((max_y - min_y) / resolution).round() as i32;

        // Initialize obstacle map
        let mut obstacle_map = vec![vec![false; y_width as usize]; x_width as usize];

        // Generate obstacle map with robot radius inflation
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

    /// Convert world position to grid index (uses min_x as reference)
    pub fn calc_xy_index(&self, position: f64) -> i32 {
        ((position - self.min_x) / self.resolution).round() as i32
    }

    /// Convert world x position to grid index
    pub fn calc_x_index(&self, x: f64) -> i32 {
        ((x - self.min_x) / self.resolution).round() as i32
    }

    /// Convert world y position to grid index
    pub fn calc_y_index(&self, y: f64) -> i32 {
        ((y - self.min_y) / self.resolution).round() as i32
    }

    /// Convert grid index to world position
    pub fn calc_grid_position(&self, index: i32) -> f64 {
        index as f64 * self.resolution + self.min_x
    }

    /// Convert grid x index to world x position
    pub fn calc_x_position(&self, ix: i32) -> f64 {
        ix as f64 * self.resolution + self.min_x
    }

    /// Convert grid y index to world y position
    pub fn calc_y_position(&self, iy: i32) -> f64 {
        iy as f64 * self.resolution + self.min_y
    }

    fn calc_grid_position_static(index: i32, min_position: f64, resolution: f64) -> f64 {
        index as f64 * resolution + min_position
    }

    /// Calculate unique grid index for a node
    pub fn calc_grid_index(&self, node: &Node) -> i32 {
        (node.y - self.min_y as i32) * self.x_width + (node.x - self.min_x as i32)
    }

    /// Calculate unique grid index from grid coordinates
    pub fn calc_index(&self, x: i32, y: i32) -> i32 {
        (y - self.min_y as i32) * self.x_width + (x - self.min_x as i32)
    }

    /// Verify if a node is valid (within bounds and not in obstacle)
    pub fn verify_node(&self, node: &Node) -> bool {
        self.is_valid(node.x, node.y)
    }

    /// Check if grid coordinates are valid
    pub fn is_valid(&self, x: i32, y: i32) -> bool {
        let px = self.calc_x_position(x);
        let py = self.calc_y_position(y);

        if px < self.min_x || py < self.min_y || px >= self.max_x || py >= self.max_y {
            return false;
        }

        // Check bounds for obstacle map access
        if x < 0 || x >= self.x_width || y < 0 || y >= self.y_width {
            return false;
        }

        // Collision check
        !self.obstacle_map[x as usize][y as usize]
    }
}

// Alias for backward compatibility
pub type SearchNode = Node;
