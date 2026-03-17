#![allow(dead_code)]

//! Dijkstra path planning algorithm
//!
//! Grid-based shortest path planning using Dijkstra's algorithm.

use itertools::iproduct;
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap};

use crate::grid_nalgebra;

/// Returns a closure that computes adjacent cell coordinates for a given cell
/// while excluding out-of-bound cells and the cell itself.
fn calculate_neighbors(nrows: usize, ncols: usize) -> impl Fn(usize, usize) -> Vec<(usize, usize)> {
    move |i, j| {
        let delta = [-1, 0, 1];
        iproduct!(delta, delta)
            .filter_map(|(di, dj)| {
                let ni = i as i32 + di;
                let nj = j as i32 + dj;
                if ni >= 0
                    && ni < nrows as i32
                    && nj >= 0
                    && nj < ncols as i32
                    && (ni, nj) != (i as i32, j as i32)
                {
                    Some((ni as usize, nj as usize))
                } else {
                    None
                }
            })
            .collect()
    }
}

/// Check if a cell or its neighbors contain an obstacle
pub fn has_collision(obstacle_map: &grid_nalgebra::Map, ni: usize, nj: usize) -> bool {
    let (height, width) = obstacle_map.shape();
    let get_neighbors = calculate_neighbors(height, width);

    let neighbors = get_neighbors(ni, nj);

    if neighbors.iter().all(|&(i, j)| obstacle_map[(i, j)] != 1) && obstacle_map[(ni, nj)] != 1 {
        return false;
    }

    true
}

/// Run Dijkstra's algorithm on a grid obstacle map.
///
/// Returns the shortest path from `start` to `goal` as a list of (row, col) coordinates,
/// or `None` if no path exists.
pub fn dijkstra_plan(
    obstacle_map: &grid_nalgebra::Map,
    start: (usize, usize),
    goal: (usize, usize),
) -> Option<Vec<(usize, usize)>> {
    let (height, width) = obstacle_map.shape();
    let get_neighbors = calculate_neighbors(height, width);

    let mut dist = BTreeMap::new();
    let mut prev: BTreeMap<(usize, usize), Option<(usize, usize)>> = BTreeMap::new();
    let mut pq = BinaryHeap::new();

    pq.push((Reverse(NotNan::new(0.0).unwrap()), start));
    dist.insert(start, 0.0);

    while let Some((distance, current)) = pq.pop() {
        if current == goal {
            // Reconstruct path
            let mut path = vec![current];
            while let Some(previous) = prev.get(path.last().unwrap()).unwrap_or(&None) {
                path.push(*previous);
                if *path.last().unwrap() == start {
                    path.reverse();
                    return Some(path);
                }
            }
            return None;
        }

        let (ci, cj) = current;
        let neighbors = get_neighbors(ci, cj);

        for (ni, nj) in neighbors {
            if !has_collision(obstacle_map, ni, nj) {
                let delta_x = ci as f64 - ni as f64;
                let delta_y = cj as f64 - nj as f64;
                let edge_distance = (delta_x * delta_x + delta_y * delta_y).sqrt();
                let alt = distance.0.into_inner() + edge_distance;

                if *dist.get(&(ni, nj)).unwrap_or(&f64::MAX) > alt {
                    dist.insert((ni, nj), alt);
                    prev.insert((ni, nj), Some((ci, cj)));
                    pq.push((Reverse(NotNan::new(alt).unwrap()), (ni, nj)));
                }
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dijkstra_simple() {
        let matrix = nalgebra::DMatrix::from_row_slice(5, 5, &[
            0, 0, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]);
        let map = grid_nalgebra::Map::new(matrix, 1).unwrap();
        let result = dijkstra_plan(&map, (0, 0), (4, 4));
        assert!(result.is_some());
        let path = result.unwrap();
        assert_eq!(*path.first().unwrap(), (0, 0));
        assert_eq!(*path.last().unwrap(), (4, 4));
    }

    #[test]
    fn test_has_collision() {
        let matrix = nalgebra::DMatrix::from_row_slice(5, 5, &[
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]);
        let map = grid_nalgebra::Map::new(matrix, 1).unwrap();
        // Cell (2,2) is an obstacle
        assert!(has_collision(&map, 2, 2));
        // Cell (0,0) is far from obstacle, no collision
        assert!(!has_collision(&map, 0, 0));
        // Cell (1,1) is adjacent to obstacle (2,2), so it has collision
        assert!(has_collision(&map, 1, 1));
    }
}
