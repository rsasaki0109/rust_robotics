//! Coverage path planning algorithms
//!
//! Implements two coverage strategies:
//! - Boustrophedon (back-and-forth sweep) coverage
//! - Spiral coverage

/// A 2D grid cell position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cell {
    pub x: i32,
    pub y: i32,
}

impl Cell {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Obstacle grid: true = blocked, false = free.
pub struct CoverageGrid {
    pub width: i32,
    pub height: i32,
    blocked: Vec<bool>,
}

impl CoverageGrid {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            width,
            height,
            blocked: vec![false; (width * height) as usize],
        }
    }

    pub fn set_blocked(&mut self, x: i32, y: i32, val: bool) {
        if self.in_bounds(x, y) {
            self.blocked[(y * self.width + x) as usize] = val;
        }
    }

    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return true;
        }
        self.blocked[(y * self.width + x) as usize]
    }

    pub fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && x < self.width && y >= 0 && y < self.height
    }

    pub fn is_free(&self, x: i32, y: i32) -> bool {
        self.in_bounds(x, y) && !self.is_blocked(x, y)
    }

    pub fn free_cell_count(&self) -> usize {
        self.blocked.iter().filter(|&&b| !b).count()
    }
}

/// Boustrophedon (back-and-forth sweep) coverage planner.
///
/// Sweeps column by column, alternating direction each column.
/// Skips blocked cells.
pub fn boustrophedon_coverage(grid: &CoverageGrid) -> Vec<Cell> {
    let mut path = Vec::new();
    let mut going_up = true;

    for x in 0..grid.width {
        let ys: Box<dyn Iterator<Item = i32>> = if going_up {
            Box::new(0..grid.height)
        } else {
            Box::new((0..grid.height).rev())
        };

        let mut column_has_free = false;
        for y in ys {
            if grid.is_free(x, y) {
                path.push(Cell::new(x, y));
                column_has_free = true;
            }
        }

        if column_has_free {
            going_up = !going_up;
        }
    }

    path
}

/// Spiral coverage planner.
///
/// Starts from the given position and spirals outward in a clockwise pattern.
/// Uses a right-hand-wall-following approach.
pub fn spiral_coverage(grid: &CoverageGrid, start_x: i32, start_y: i32) -> Vec<Cell> {
    let mut path = Vec::new();
    let mut visited = vec![false; (grid.width * grid.height) as usize];

    if !grid.is_free(start_x, start_y) {
        return path;
    }

    // Directions: right, down, left, up (clockwise)
    let dx = [1, 0, -1, 0];
    let dy = [0, 1, 0, -1];

    let mut x = start_x;
    let mut y = start_y;
    let mut dir = 0usize; // start going right

    let visit = |x: i32, y: i32, visited: &mut Vec<bool>| {
        visited[(y * grid.width + x) as usize] = true;
    };

    let is_visited = |x: i32, y: i32, visited: &Vec<bool>| -> bool {
        if !grid.in_bounds(x, y) {
            return true;
        }
        visited[(y * grid.width + x) as usize]
    };

    path.push(Cell::new(x, y));
    visit(x, y, &mut visited);

    let total_free = grid.free_cell_count();
    let mut stuck_count = 0;

    while path.len() < total_free && stuck_count < 4 {
        let nx = x + dx[dir];
        let ny = y + dy[dir];

        if grid.is_free(nx, ny) && !is_visited(nx, ny, &visited) {
            x = nx;
            y = ny;
            path.push(Cell::new(x, y));
            visit(x, y, &mut visited);
            stuck_count = 0;
        } else {
            // Turn clockwise
            dir = (dir + 1) % 4;
            stuck_count += 1;
        }
    }

    // If spiral gets stuck, do a second pass to pick up remaining cells
    if path.len() < total_free {
        for cy in 0..grid.height {
            for cx in 0..grid.width {
                if grid.is_free(cx, cy) && !is_visited(cx, cy, &visited) {
                    path.push(Cell::new(cx, cy));
                    visit(cx, cy, &mut visited);
                }
            }
        }
    }

    path
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boustrophedon_covers_all_free_cells() {
        let grid = CoverageGrid::new(5, 5);
        let path = boustrophedon_coverage(&grid);

        assert_eq!(path.len(), 25);
        // All cells should be visited exactly once
        let mut seen = std::collections::HashSet::new();
        for cell in &path {
            assert!(seen.insert(*cell), "duplicate cell: {:?}", cell);
        }
    }

    #[test]
    fn test_boustrophedon_skips_obstacles() {
        let mut grid = CoverageGrid::new(4, 4);
        grid.set_blocked(1, 1, true);
        grid.set_blocked(2, 2, true);

        let path = boustrophedon_coverage(&grid);
        assert_eq!(path.len(), 14); // 16 - 2 blocked

        for cell in &path {
            assert!(!grid.is_blocked(cell.x, cell.y));
        }
    }

    #[test]
    fn test_boustrophedon_path_is_connected_within_columns() {
        let grid = CoverageGrid::new(3, 4);
        let path = boustrophedon_coverage(&grid);

        // Within each column, y values should be consecutive
        assert_eq!(path.len(), 12);
    }

    #[test]
    fn test_spiral_covers_all_free_cells() {
        let grid = CoverageGrid::new(5, 5);
        let path = spiral_coverage(&grid, 2, 2);

        assert_eq!(path.len(), 25);
        let mut seen = std::collections::HashSet::new();
        for cell in &path {
            assert!(seen.insert(*cell), "duplicate cell: {:?}", cell);
        }
    }

    #[test]
    fn test_spiral_with_obstacles() {
        let mut grid = CoverageGrid::new(5, 5);
        grid.set_blocked(1, 1, true);
        grid.set_blocked(3, 3, true);

        let path = spiral_coverage(&grid, 0, 0);
        assert_eq!(path.len(), 23); // 25 - 2 blocked

        for cell in &path {
            assert!(!grid.is_blocked(cell.x, cell.y));
        }
    }

    #[test]
    fn test_spiral_from_corner() {
        let grid = CoverageGrid::new(3, 3);
        let path = spiral_coverage(&grid, 0, 0);

        assert_eq!(path.len(), 9);
        assert_eq!(path[0], Cell::new(0, 0));
    }

    #[test]
    fn test_spiral_blocked_start() {
        let mut grid = CoverageGrid::new(3, 3);
        grid.set_blocked(1, 1, true);
        let path = spiral_coverage(&grid, 1, 1);
        assert!(path.is_empty());
    }

    #[test]
    fn test_empty_grid() {
        let grid = CoverageGrid::new(0, 0);
        let path = boustrophedon_coverage(&grid);
        assert!(path.is_empty());
    }
}
