//! Integration tests for the Spiral Spanning Tree Coverage Path Planner.

#[path = "../src/spiral_spanning_tree_cpp.rs"]
mod spiral_spanning_tree_cpp;

use spiral_spanning_tree_cpp::*;
use std::collections::HashSet;

/// Helper: create an all-free grid and plan from the given start.
fn plan_all_free(height: usize, width: usize, start: MergedNode) -> CoveragePlanResult {
    let grid = OccupancyGrid::all_free(width, height);
    let planner = SpiralSpanningTreePlanner::new(grid);
    planner.plan(start)
}

#[test]
fn test_small_2x2_grid() {
    // Route has 2 entries: the node is pushed once during DFS and once during backtrace.
    let result = plan_all_free(2, 2, (0, 0));
    assert_eq!(result.route.len(), 2);
    assert!(result.edges.is_empty());
    assert!(result.path.is_empty());
}

#[test]
fn test_4x4_grid_visits_all_merged_cells() {
    let result = plan_all_free(4, 4, (0, 0));
    let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
    for r in 0..2 {
        for c in 0..2 {
            assert!(
                visited.contains(&(r, c)),
                "merged cell ({r}, {c}) was not visited"
            );
        }
    }
}

#[test]
fn test_spanning_tree_edge_count() {
    let result = plan_all_free(4, 4, (0, 0));
    assert_eq!(result.edges.len(), 3);
}

#[test]
fn test_6x6_grid_all_merged_cells_visited() {
    let result = plan_all_free(6, 6, (0, 0));
    let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
    for r in 0..3 {
        for c in 0..3 {
            assert!(
                visited.contains(&(r, c)),
                "merged cell ({r}, {c}) not visited"
            );
        }
    }
    assert_eq!(result.edges.len(), 8);
}

#[test]
fn test_path_segments_use_original_resolution() {
    let result = plan_all_free(4, 4, (0, 0));
    for seg in &result.path {
        for pt in seg {
            assert!(pt[0] >= 0 && pt[0] < 4, "row out of bounds: {}", pt[0]);
            assert!(pt[1] >= 0 && pt[1] < 4, "col out of bounds: {}", pt[1]);
        }
    }
}

#[test]
fn test_obstacle_blocks_merged_cell() {
    let mut data = vec![true; 16];
    data[0] = false;
    let grid = OccupancyGrid::new(4, 4, data);
    let planner = SpiralSpanningTreePlanner::new(grid);
    let result = planner.plan((0, 1));
    let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
    assert!(
        !visited.contains(&(0, 0)),
        "blocked merged cell (0,0) should not be visited"
    );
}

#[test]
fn test_path_not_empty_for_multi_cell() {
    let result = plan_all_free(4, 4, (0, 0));
    assert!(!result.path.is_empty());
}

#[test]
fn test_edges_connect_adjacent_nodes() {
    let result = plan_all_free(6, 6, (1, 1));
    for &(a, b) in &result.edges {
        let dist = (a.0 - b.0).unsigned_abs() + (a.1 - b.1).unsigned_abs();
        assert_eq!(dist, 1, "edge {a:?}-{b:?} is not between adjacent nodes");
    }
}

#[test]
fn test_different_start_positions() {
    for &start in &[(0, 0), (1, 0), (0, 1), (1, 1)] {
        let result = plan_all_free(4, 4, start);
        let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
        assert_eq!(
            visited.len(),
            4,
            "all 4 merged cells should be visited from start {start:?}"
        );
    }
}

#[test]
fn test_larger_grid_coverage() {
    let result = plan_all_free(10, 10, (0, 0));
    let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
    assert_eq!(visited.len(), 25);
    assert_eq!(result.edges.len(), 24);
}

#[test]
#[should_panic(expected = "width must be even")]
fn test_odd_width_panics() {
    OccupancyGrid::new(3, 4, vec![true; 12]);
}

#[test]
#[should_panic(expected = "height must be even")]
fn test_odd_height_panics() {
    OccupancyGrid::new(4, 3, vec![true; 12]);
}
