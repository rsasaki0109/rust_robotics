//! Path smoothing via line-of-sight shortcutting
//!
//! Removes redundant waypoints from a grid-based path by checking direct
//! visibility between non-adjacent points. Works with any grid planner output
//! but is especially useful for DFS, which produces long, winding paths.

use crate::grid::GridMap;
use rust_robotics_core::{Path2D, Point2D};

/// Smooth a path by removing intermediate waypoints that can be skipped
/// via direct line-of-sight.
///
/// Uses a greedy forward scan: from the current anchor point, find the
/// farthest reachable point with line-of-sight, advance the anchor there,
/// and repeat.
pub fn smooth_path(path: &Path2D, grid_map: &GridMap) -> Path2D {
    if path.len() <= 2 {
        return path.clone();
    }

    let points = &path.points;
    let mut smoothed = Vec::new();
    smoothed.push(points[0]);

    let mut anchor = 0;
    while anchor < points.len() - 1 {
        // Find the farthest point visible from anchor
        let mut farthest = anchor + 1;
        for candidate in (anchor + 2)..points.len() {
            if line_of_sight_world(grid_map, &points[anchor], &points[candidate]) {
                farthest = candidate;
            }
        }
        smoothed.push(points[farthest]);
        anchor = farthest;
    }

    Path2D::from_points(smoothed)
}

/// Check line-of-sight between two world-coordinate points on a grid.
/// Uses DDA ray marching identical to Theta*'s line_of_sight.
fn line_of_sight_world(grid_map: &GridMap, from: &Point2D, to: &Point2D) -> bool {
    let x0 = grid_map.calc_x_index(from.x);
    let y0 = grid_map.calc_y_index(from.y);
    let x1 = grid_map.calc_x_index(to.x);
    let y1 = grid_map.calc_y_index(to.y);

    if !grid_map.is_valid(x0, y0) || !grid_map.is_valid(x1, y1) {
        return false;
    }

    if x0 == x1 && y0 == y1 {
        return true;
    }

    let dx = x1 - x0;
    let dy = y1 - y0;
    let step_x = dx.signum();
    let step_y = dy.signum();
    let abs_dx = dx.abs() as f64;
    let abs_dy = dy.abs() as f64;

    let mut x = x0;
    let mut y = y0;
    let mut t_max_x = if step_x != 0 {
        0.5 / abs_dx
    } else {
        f64::INFINITY
    };
    let mut t_max_y = if step_y != 0 {
        0.5 / abs_dy
    } else {
        f64::INFINITY
    };
    let t_delta_x = if step_x != 0 {
        1.0 / abs_dx
    } else {
        f64::INFINITY
    };
    let t_delta_y = if step_y != 0 {
        1.0 / abs_dy
    } else {
        f64::INFINITY
    };

    while x != x1 || y != y1 {
        let advance_x = t_max_x <= t_max_y;
        let advance_y = t_max_y <= t_max_x;
        let next_x = if advance_x { x + step_x } else { x };
        let next_y = if advance_y { y + step_y } else { y };

        if !grid_map.is_valid_step(x, y, next_x, next_y) {
            return false;
        }

        x = next_x;
        y = next_y;

        if advance_x {
            t_max_x += t_delta_x;
        }
        if advance_y {
            t_max_y += t_delta_y;
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use rust_robotics_core::Obstacles;

    fn create_simple_obstacles() -> (Obstacles, GridMap) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        for i in 0..21 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(20.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(20.0);
            oy.push(i as f64);
        }
        for i in 5..15 {
            ox.push(10.0);
            oy.push(i as f64);
        }
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let grid_map = GridMap::try_new(&ox, &oy, 1.0, 0.5).unwrap();
        (obstacles, grid_map)
    }

    #[test]
    fn test_smooth_path_reduces_waypoints() {
        let (_obstacles, grid_map) = create_simple_obstacles();
        // Simulate a DFS-like zigzag path in an open area
        let zigzag = Path2D::from_points(vec![
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 2.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(5.0, 2.0),
            Point2D::new(6.0, 2.0),
            Point2D::new(7.0, 2.0),
            Point2D::new(8.0, 2.0),
        ]);
        let smoothed = smooth_path(&zigzag, &grid_map);
        assert!(smoothed.len() <= 2, "straight line should collapse to 2 points, got {}", smoothed.len());
        assert!((smoothed.points.first().unwrap().x - 2.0).abs() < 1e-6);
        assert!((smoothed.points.last().unwrap().x - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_smooth_path_preserves_endpoints() {
        let (_obstacles, grid_map) = create_simple_obstacles();
        let path = Path2D::from_points(vec![
            Point2D::new(2.0, 2.0),
            Point2D::new(3.0, 3.0),
            Point2D::new(4.0, 4.0),
        ]);
        let smoothed = smooth_path(&path, &grid_map);
        let first = smoothed.points.first().unwrap();
        let last = smoothed.points.last().unwrap();
        assert!((first.x - 2.0).abs() < 1e-6);
        assert!((first.y - 2.0).abs() < 1e-6);
        assert!((last.x - 4.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_smooth_path_respects_obstacles() {
        let (_obstacles, grid_map) = create_simple_obstacles();
        // Path that goes around the wall at x=10, y=5..15
        let path = Path2D::from_points(vec![
            Point2D::new(8.0, 10.0),
            Point2D::new(8.0, 16.0),
            Point2D::new(12.0, 16.0),
            Point2D::new(12.0, 10.0),
        ]);
        let smoothed = smooth_path(&path, &grid_map);
        // Should not shortcut through the wall
        assert!(
            smoothed.len() >= 3,
            "should not shortcut through wall, got {} waypoints",
            smoothed.len()
        );
    }

    #[test]
    fn test_smooth_path_with_dfs() {
        let (obstacles, _grid_map) = create_simple_obstacles();
        let dfs_planner = crate::depth_first_search::DFSPlanner::from_obstacle_points(
            &obstacles,
            crate::depth_first_search::DFSConfig::default(),
        )
        .unwrap();
        let a_star_planner = crate::a_star::AStarPlanner::from_obstacle_points(
            &obstacles,
            crate::a_star::AStarConfig::default(),
        )
        .unwrap();

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);
        let dfs_path = dfs_planner.plan(start, goal).unwrap();
        let a_star_path = a_star_planner.plan(start, goal).unwrap();
        let smoothed = smooth_path(&dfs_path, dfs_planner.grid_map());

        // Smoothed DFS should have far fewer waypoints than raw DFS
        assert!(
            smoothed.len() < dfs_path.len(),
            "smoothed ({}) should be shorter than raw DFS ({})",
            smoothed.len(),
            dfs_path.len()
        );
        // Smoothed DFS path length should be reasonable (within 20% of A*)
        let ratio = smoothed.total_length() / a_star_path.total_length();
        assert!(
            ratio < 1.2,
            "smoothed DFS length ({:.2}) should be within 20% of A* ({:.2}), ratio = {:.4}",
            smoothed.total_length(),
            a_star_path.total_length(),
            ratio
        );
    }

    #[test]
    fn test_smooth_empty_and_single_point() {
        let (_obstacles, grid_map) = create_simple_obstacles();
        let empty = Path2D::from_points(vec![]);
        assert!(smooth_path(&empty, &grid_map).is_empty());

        let single = Path2D::from_points(vec![Point2D::new(5.0, 5.0)]);
        assert_eq!(smooth_path(&single, &grid_map).len(), 1);
    }
}
