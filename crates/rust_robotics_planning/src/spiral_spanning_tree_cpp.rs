//! Spiral Spanning Tree Coverage Path Planner
//!
//! Implements the Spiral-STC algorithm for coverage path planning on a grid.
//!
//! Reference paper: "Spiral-STC: An On-Line Coverage Algorithm of Grid Environments
//! by a Mobile Robot" by Gabriely et al.
//! <https://ieeexplore.ieee.org/abstract/document/1013479>
//!
//! The algorithm works by:
//! 1. Merging the original grid into 2x2 mega-cells.
//! 2. Building a spanning tree over the free mega-cells using a recursive DFS.
//! 3. Tracing a spiral coverage path around the spanning tree edges at original resolution.

use std::collections::HashSet;

/// A node position on the merged (half-resolution) grid.
pub type MergedNode = (i32, i32);

/// A point on the original (full-resolution) grid.
pub type SubNode = [i32; 2];

/// A directed edge in the spanning tree (from, to).
pub type TreeEdge = (MergedNode, MergedNode);

/// A path segment consisting of two sub-nodes (entry, exit) in original resolution.
pub type PathSegment = [SubNode; 2];

/// Cardinal direction for movement between adjacent merged nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    North,
    South,
    East,
    West,
}

/// Result of the coverage path planning.
#[derive(Debug, Clone)]
pub struct CoveragePlanResult {
    /// Spanning tree edges on the merged grid.
    pub edges: Vec<TreeEdge>,
    /// Route of merged-grid nodes visited (including backtrace duplicates).
    pub route: Vec<MergedNode>,
    /// Coverage path segments at original grid resolution.
    pub path: Vec<PathSegment>,
}

/// Occupancy grid for the Spiral-STC planner.
///
/// The grid is stored in row-major order with `true` = free and `false` = occupied.
/// Width and height must both be even numbers so that 2x2 mega-cell merging works.
#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    width: usize,
    height: usize,
    /// Row-major occupancy: `data[row * width + col]`. `true` means free.
    data: Vec<bool>,
}

impl OccupancyGrid {
    /// Create a new occupancy grid.
    ///
    /// # Panics
    /// Panics if `width` or `height` is odd, or if `data.len() != width * height`.
    pub fn new(width: usize, height: usize, data: Vec<bool>) -> Self {
        assert!(width.is_multiple_of(2), "width must be even, got {width}");
        assert!(
            height.is_multiple_of(2),
            "height must be even, got {height}"
        );
        assert_eq!(data.len(), width * height);
        Self {
            width,
            height,
            data,
        }
    }

    /// Create an all-free grid of the given dimensions.
    ///
    /// # Panics
    /// Panics if `width` or `height` is odd.
    pub fn all_free(width: usize, height: usize) -> Self {
        Self::new(width, height, vec![true; width * height])
    }

    fn get(&self, row: usize, col: usize) -> bool {
        self.data[row * self.width + col]
    }
}

/// Spiral Spanning Tree Coverage Path Planner.
pub struct SpiralSpanningTreePlanner {
    occ: OccupancyGrid,
    merged_height: usize,
    merged_width: usize,
}

impl SpiralSpanningTreePlanner {
    /// Create a planner from an occupancy grid.
    pub fn new(occ: OccupancyGrid) -> Self {
        let merged_height = occ.height / 2;
        let merged_width = occ.width / 2;
        Self {
            occ,
            merged_height,
            merged_width,
        }
    }

    /// Plan a coverage path starting from the given merged-grid node `(row, col)`.
    ///
    /// Returns the spanning tree edges, the route on the merged grid, and the
    /// coverage path segments at original resolution.
    pub fn plan(&self, start: MergedNode) -> CoveragePlanResult {
        let mh = self.merged_height;
        let mw = self.merged_width;

        let mut visit_times = vec![vec![0u8; mw]; mh];
        visit_times[start.0 as usize][start.1 as usize] = 1;

        let mut edges = Vec::new();
        let mut route = Vec::new();

        self.build_spanning_tree(start, &mut visit_times, &mut route, &mut edges);

        // Generate coverage path from the route.
        let mut path: Vec<PathSegment> = Vec::new();
        for idx in 0..route.len().saturating_sub(1) {
            let cur = route[idx];
            let next = route[idx + 1];
            let dp = (cur.0 - next.0).unsigned_abs() + (cur.1 - next.1).unsigned_abs();

            match dp {
                0 => {
                    // Round-trip: node revisited during backtrace.
                    if idx > 0 {
                        let seg = self.round_trip_path(route[idx - 1], cur);
                        path.push(seg);
                    }
                }
                1 => {
                    path.push(self.move_segment(cur, next));
                }
                2 => {
                    // Non-adjacent route nodes: insert intermediate node from spanning tree.
                    let mid = self.intermediate_node(cur, next, &edges);
                    path.push(self.move_segment(cur, mid));
                    path.push(self.move_segment(mid, next));
                }
                _ => panic!("adjacent route node distance > 2: {dp}"),
            }
        }

        CoveragePlanResult { edges, route, path }
    }

    /// Check whether a merged-grid cell is valid (in bounds and all 4 sub-cells free).
    fn is_valid_merged(&self, i: i32, j: i32) -> bool {
        if i < 0 || j < 0 {
            return false;
        }
        let (ui, uj) = (i as usize, j as usize);
        if ui >= self.merged_height || uj >= self.merged_width {
            return false;
        }
        let r = 2 * ui;
        let c = 2 * uj;
        self.occ.get(r, c)
            && self.occ.get(r + 1, c)
            && self.occ.get(r, c + 1)
            && self.occ.get(r + 1, c + 1)
    }

    /// Recursive DFS to build spanning tree and route.
    fn build_spanning_tree(
        &self,
        current: MergedNode,
        visit_times: &mut [Vec<u8>],
        route: &mut Vec<MergedNode>,
        edges: &mut Vec<TreeEdge>,
    ) {
        // Counter-clockwise neighbor order: S, E, N, W
        const ORDER: [(i32, i32); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

        route.push(current);
        let mut found = false;

        for &(di, dj) in &ORDER {
            let ni = current.0 + di;
            let nj = current.1 + dj;
            if self.is_valid_merged(ni, nj) && visit_times[ni as usize][nj as usize] == 0 {
                let neighbor = (ni, nj);
                edges.push((current, neighbor));
                found = true;
                visit_times[ni as usize][nj as usize] = 1;
                self.build_spanning_tree(neighbor, visit_times, route, edges);
            }
        }

        // Backtrace from dead-end to first node with unvisited neighbor.
        if !found {
            let mut has_unvisited_ngb = false;
            for node in route.clone().iter().rev() {
                if visit_times[node.0 as usize][node.1 as usize] == 2 {
                    continue;
                }
                visit_times[node.0 as usize][node.1 as usize] += 1;
                route.push(*node);

                for &(di, dj) in &ORDER {
                    let ni = node.0 + di;
                    let nj = node.1 + dj;
                    if self.is_valid_merged(ni, nj) && visit_times[ni as usize][nj as usize] == 0 {
                        has_unvisited_ngb = true;
                        break;
                    }
                }
                if has_unvisited_ngb {
                    break;
                }
            }
        }
    }

    /// Determine the cardinal direction from merged node `p` to merged node `q`.
    fn direction(p: MergedNode, q: MergedNode) -> Direction {
        if p.0 == q.0 && p.1 < q.1 {
            Direction::East
        } else if p.0 == q.0 && p.1 > q.1 {
            Direction::West
        } else if p.0 < q.0 && p.1 == q.1 {
            Direction::South
        } else if p.0 > q.0 && p.1 == q.1 {
            Direction::North
        } else {
            panic!(
                "direction: only cardinal directions supported, got {:?} -> {:?}",
                p, q
            );
        }
    }

    /// Convert a merged node to one of its four sub-nodes at original resolution.
    fn sub_node(node: MergedNode, quad: &str) -> SubNode {
        let (r, c) = (node.0, node.1);
        match quad {
            "SE" => [2 * r + 1, 2 * c + 1],
            "SW" => [2 * r + 1, 2 * c],
            "NE" => [2 * r, 2 * c + 1],
            "NW" => [2 * r, 2 * c],
            _ => panic!("sub_node: invalid quadrant '{quad}'"),
        }
    }

    /// Compute the path segment when moving from merged node `p` to adjacent `q`.
    fn move_segment(&self, p: MergedNode, q: MergedNode) -> PathSegment {
        match Self::direction(p, q) {
            Direction::East => [Self::sub_node(p, "SE"), Self::sub_node(q, "SW")],
            Direction::West => [Self::sub_node(p, "NW"), Self::sub_node(q, "NE")],
            Direction::South => [Self::sub_node(p, "SW"), Self::sub_node(q, "NW")],
            Direction::North => [Self::sub_node(p, "NE"), Self::sub_node(q, "SE")],
        }
    }

    /// Compute the round-trip path segment when backtracing at a pivot node.
    fn round_trip_path(&self, last: MergedNode, pivot: MergedNode) -> PathSegment {
        match Self::direction(last, pivot) {
            Direction::East => [Self::sub_node(pivot, "SE"), Self::sub_node(pivot, "NE")],
            Direction::South => [Self::sub_node(pivot, "SW"), Self::sub_node(pivot, "SE")],
            Direction::West => [Self::sub_node(pivot, "NW"), Self::sub_node(pivot, "SW")],
            Direction::North => [Self::sub_node(pivot, "NE"), Self::sub_node(pivot, "NW")],
        }
    }

    /// Find the intermediate node between two non-adjacent route nodes
    /// by looking for a shared neighbor in the spanning tree.
    fn intermediate_node(&self, p: MergedNode, q: MergedNode, edges: &[TreeEdge]) -> MergedNode {
        let mut p_ngb = HashSet::new();
        let mut q_ngb = HashSet::new();

        for &(m, n) in edges {
            if m == p {
                p_ngb.insert(n);
            }
            if n == p {
                p_ngb.insert(m);
            }
            if m == q {
                q_ngb.insert(n);
            }
            if n == q {
                q_ngb.insert(m);
            }
        }

        let intersection: Vec<_> = p_ngb.intersection(&q_ngb).copied().collect();
        assert!(
            intersection.len() == 1,
            "expected exactly 1 intermediate node between {p:?} and {q:?}, found {}",
            intersection.len()
        );
        intersection[0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create an all-free grid and plan from the given start.
    fn plan_all_free(height: usize, width: usize, start: MergedNode) -> CoveragePlanResult {
        let grid = OccupancyGrid::all_free(width, height);
        let planner = SpiralSpanningTreePlanner::new(grid);
        planner.plan(start)
    }

    #[test]
    fn test_small_2x2_grid() {
        // 2x2 original grid -> 1x1 merged grid -> single node, no edges.
        // Route has 2 entries because the backtrace appends the node once more.
        let result = plan_all_free(2, 2, (0, 0));
        assert_eq!(result.route.len(), 2);
        assert!(result.edges.is_empty());
        assert!(result.path.is_empty());
    }

    #[test]
    fn test_4x4_grid_visits_all_merged_cells() {
        // 4x4 original -> 2x2 merged grid -> 4 merged cells.
        let result = plan_all_free(4, 4, (0, 0));

        let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
        // All 4 merged cells must be visited.
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
        // A spanning tree on n nodes has n-1 edges.
        let result = plan_all_free(4, 4, (0, 0));
        // 2x2 = 4 merged cells -> 3 edges.
        assert_eq!(result.edges.len(), 3);
    }

    #[test]
    fn test_6x6_grid_all_merged_cells_visited() {
        // 6x6 original -> 3x3 merged grid -> 9 cells.
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
        // 9 nodes -> 8 spanning tree edges.
        assert_eq!(result.edges.len(), 8);
    }

    #[test]
    fn test_path_segments_use_original_resolution() {
        let result = plan_all_free(4, 4, (0, 0));
        // All path segment coordinates should be within original grid bounds.
        for seg in &result.path {
            for pt in seg {
                assert!(pt[0] >= 0 && pt[0] < 4, "row out of bounds: {}", pt[0]);
                assert!(pt[1] >= 0 && pt[1] < 4, "col out of bounds: {}", pt[1]);
            }
        }
    }

    #[test]
    fn test_obstacle_blocks_merged_cell() {
        // 4x4 grid with one sub-cell blocked makes one merged cell invalid.
        let mut data = vec![true; 16];
        // Block cell (0,0) in original grid -> merged cell (0,0) is invalid.
        data[0] = false;
        let grid = OccupancyGrid::new(4, 4, data);
        let planner = SpiralSpanningTreePlanner::new(grid);
        // Start from merged cell (0,1) which is still free.
        let result = planner.plan((0, 1));

        let visited: HashSet<MergedNode> = result.route.iter().copied().collect();
        // Merged cell (0,0) should NOT be visited since one sub-cell is blocked.
        assert!(
            !visited.contains(&(0, 0)),
            "blocked merged cell (0,0) should not be visited"
        );
    }

    #[test]
    fn test_path_not_empty_for_multi_cell() {
        let result = plan_all_free(4, 4, (0, 0));
        assert!(
            !result.path.is_empty(),
            "path should not be empty for a multi-cell grid"
        );
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
        // The algorithm should work from any valid free merged cell.
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
        // 10x10 original -> 5x5 merged = 25 cells.
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

    #[test]
    fn test_sub_node_coordinates() {
        // Verify sub_node mapping for merged node (1, 2).
        assert_eq!(SpiralSpanningTreePlanner::sub_node((1, 2), "NW"), [2, 4]);
        assert_eq!(SpiralSpanningTreePlanner::sub_node((1, 2), "NE"), [2, 5]);
        assert_eq!(SpiralSpanningTreePlanner::sub_node((1, 2), "SW"), [3, 4]);
        assert_eq!(SpiralSpanningTreePlanner::sub_node((1, 2), "SE"), [3, 5]);
    }
}
