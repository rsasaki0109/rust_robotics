// Dijkstra path planning
// author: Salah Eddine Ghamri (s.ghamri)

use gnuplot::*;
use std::thread::sleep;
use std::time::Duration;

use itertools::iproduct;
use std::collections::BTreeMap;

use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

use na::DMatrix;
extern crate nalgebra as na;
use rust_robotics::grid_map;

// The neighbors function returns adjacent cell coordinates for a given cell in a grid
// while excluding out-of-bound and the cell itself.
fn calculate_neighbors(nrows: usize, ncols: usize) -> impl Fn(usize, usize) -> Vec<(usize, usize)> {
    move |i, j| {
        let delta = [-1, 0, 1];
        let neighbors: Vec<_> = iproduct!(delta, delta)
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
            .collect();
        neighbors
    }
}

fn has_collision(obstacle_map: &grid_map::Map, ni: usize, nj: usize) -> bool {
    let (height, width) = obstacle_map.shape();
    let get_neighbors = calculate_neighbors(height, width);

    let neighbors = get_neighbors(ni, nj);

    if neighbors.iter().all(|&(i, j)| obstacle_map[(i, j)] != 1) && obstacle_map[(ni, nj)] != 1 {
        return false;
    }

    true
}

fn dijkstra_planner(obstacle_map: &grid_map::Map, start: &(usize, usize), goal: &(usize, usize)) {
    let (height, width) = obstacle_map.shape();
    let get_neighbors = calculate_neighbors(height, width);

    let mut dist = BTreeMap::new();
    let mut prev: BTreeMap<(usize, usize), Option<(usize, usize)>> = BTreeMap::new();
    let mut pq = BinaryHeap::new();

    let (mut ci, mut cj) = start;

    pq.push((Reverse(NotNan::new(0.0).unwrap()), (ci, cj)));
    dist.insert((ci, cj), 0.0 as f64);

    let mut fg = Figure::new();
    let mut map_x = vec![];
    let mut map_y = vec![];

    let mut neighbors_x = vec![];
    let mut neighbors_y = vec![];

    let mut visited_x = vec![];
    let mut visited_y = vec![];

    let mut path_x = vec![];
    let mut path_y = vec![];

    for i in 0..obstacle_map.nrows() {
        for j in 0..obstacle_map.ncols() {
            if obstacle_map[(i, j)] == 1 {
                map_x.push(j as i32);
                map_y.push(-(i as i32));
            }
        }
    }

    while let Some((distance, current)) = pq.pop() {
        if current != *goal {
            (ci, cj) = current;
            visited_x.push(cj as i32);
            visited_y.push(-(ci as i32));

            let neighbors = get_neighbors(ci, cj);

            neighbors_x.clear();
            neighbors_y.clear();

            for (ni, nj) in neighbors {
                // NOTE: in practice add a collision check with robots radius here
                if !has_collision(obstacle_map, ni, nj) {
                    neighbors_x.push(nj as i32);
                    neighbors_y.push(-(ni as i32));

                    let delta_x = (ci as f64 - ni as f64) as f64;
                    let delta_y = (cj as f64 - nj as f64) as f64;
                    let edge_distance = (delta_x * delta_x + delta_y * delta_y).sqrt();
                    let alt = distance.0.into_inner() + edge_distance;

                    if *dist.get(&(ni, nj)).unwrap_or(&f64::MAX) > alt {
                        dist.insert((ni, nj), alt);
                        prev.insert((ni, nj), Some((ci, cj)));
                        pq.push((Reverse(NotNan::new(alt).unwrap()), (ni, nj)));
                    }
                }
            }
        } else {
            let mut path = vec![current];
            while let Some(previous) = prev.get(path.last().unwrap()).unwrap_or(&None) {
                path.push(*previous);
                if *path.last().unwrap() == *start {
                    path.reverse();
                    path_x.clear();
                    path_y.clear();
                    for point in path {
                        path_x.push(point.1 as i32);
                        path_y.push(-(point.0 as i32));
                    }
                    break;
                }
            }
        }

        fg.clear_axes();
        fg.axes2d()
            .set_x_range(Fix(0.0), Fix(width as f64))
            .set_y_range(Fix(-(height as f64)), Fix(1.0))
            .points(
                map_x.iter(),
                map_y.iter(),
                &[PointSymbol('S'), Color("black"), PointSize(7.5)],
            )
            .points(
                Some(start.1 as i32),
                Some(-(start.0 as i32)),
                &[PointSymbol('O'), Color("red"), PointSize(3.0)],
            )
            .points(
                Some(goal.1),
                Some(-(goal.0 as i32)),
                &[PointSymbol('O'), Color("red"), PointSize(3.0)],
            )
            .points(
                &neighbors_x,
                &neighbors_y,
                &[PointSymbol('S'), Color("gray"), PointSize(2.0)],
            )
            .points(
                &visited_x,
                &visited_y,
                &[PointSymbol('S'), Color("green"), PointSize(2.5)],
            )
            .lines(&path_x, &path_y, &[Color("black"), LineWidth(3.0)]);

        let crate_dir = option_env!("CARGO_MANIFEST_DIR").unwrap();
        fg.show_and_keep_running().unwrap();
        sleep(Duration::from_millis(10));

        if path_x.len() > 1 {
            let _ = fg.save_to_svg(
                format!("{}/img/dijkstra_planner.svg", crate_dir).as_str(),
                800,
                600,
            );
            break;
        }
    }
}

fn main() {
    #[rustfmt::skip]
    let original_matrix = DMatrix::from_row_slice(10, 14, &[
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1 ,0 ,0 ,0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0
    ]);

    let obstacle_map = grid_map::Map::new(original_matrix, 3).unwrap();

    let start = (28, 5);
    let goal = (28, 28);

    dijkstra_planner(&obstacle_map, &start, &goal);
}
