#![allow(clippy::too_many_arguments)]

//! RRT shortcut path smoothing.
//!
//! This mirrors PythonRobotics `PathPlanning/RRT/rrt_with_pathsmoothing.py`
//! rather than the grid line-of-sight post-processing in `path_smoothing.rs`.

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

use crate::rrt::{CircleObstacle, RRTPlanner};

#[derive(Debug, Clone, Copy, PartialEq)]
struct TargetPoint {
    point: Point2D,
    anchor_index: isize,
}

/// Configuration for shortcut smoothing over an RRT-generated path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RRTPathSmoothingConfig {
    pub max_iter: usize,
    pub sample_step: f64,
}

impl Default for RRTPathSmoothingConfig {
    fn default() -> Self {
        Self {
            max_iter: 1000,
            sample_step: 0.2,
        }
    }
}

/// Shortcut-smoothed RRT planner.
pub struct RRTPathSmoothingPlanner {
    rrt: RRTPlanner,
    smoothing: RRTPathSmoothingConfig,
    robot_radius: f64,
}

impl RRTPathSmoothingPlanner {
    pub fn new(rrt: RRTPlanner, smoothing: RRTPathSmoothingConfig, robot_radius: f64) -> Self {
        Self {
            rrt,
            smoothing,
            robot_radius,
        }
    }

    pub fn from_obstacles(
        obstacle_list: Vec<(f64, f64, f64)>,
        rand_area: [f64; 2],
        expand_dis: f64,
        path_resolution: f64,
        goal_sample_rate: i32,
        max_iter: usize,
        play_area: Option<[f64; 4]>,
        robot_radius: f64,
        smoothing: RRTPathSmoothingConfig,
    ) -> Self {
        let rrt = RRTPlanner::from_obstacles(
            obstacle_list,
            rand_area,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            play_area,
            robot_radius,
        );
        Self::new(rrt, smoothing, robot_radius)
    }

    pub fn planning(&mut self, start: [f64; 2], goal: [f64; 2]) -> Option<Vec<[f64; 2]>> {
        let start_pt = Point2D::new(start[0], start[1]);
        let goal_pt = Point2D::new(goal[0], goal[1]);
        match self.plan(start_pt, goal_pt) {
            Ok(path) => Some(path.points.iter().map(|point| [point.x, point.y]).collect()),
            Err(_) => None,
        }
    }

    pub fn rrt(&self) -> &RRTPlanner {
        &self.rrt
    }
}

pub fn get_path_length(path: &Path2D) -> f64 {
    path.total_length()
}

fn get_target_point(path: &Path2D, target_length: f64) -> Option<TargetPoint> {
    if path.len() < 2 {
        return None;
    }

    let mut accumulated = 0.0;
    let mut anchor_index = 0_isize;
    let mut last_pair_length = 0.0;
    for (index, segment) in path.points.windows(2).enumerate() {
        let length = segment[0].distance(&segment[1]);
        accumulated += length;
        if accumulated >= target_length {
            anchor_index = index as isize - 1;
            last_pair_length = length;
            break;
        }
    }

    if last_pair_length <= f64::EPSILON {
        return None;
    }

    let part_ratio = (accumulated - target_length) / last_pair_length;
    let anchor = python_index(&path.points, anchor_index)?;
    let next = python_index(&path.points, anchor_index + 1)?;
    Some(TargetPoint {
        point: Point2D::new(
            anchor.x + (next.x - anchor.x) * part_ratio,
            anchor.y + (next.y - anchor.y) * part_ratio,
        ),
        anchor_index,
    })
}

fn python_index(points: &[Point2D], index: isize) -> Option<Point2D> {
    if points.is_empty() {
        return None;
    }

    let resolved = if index < 0 {
        points.len() as isize + index
    } else {
        index
    };
    points.get(resolved as usize).copied()
}

fn is_point_collision(point: Point2D, obstacle_list: &[CircleObstacle], robot_radius: f64) -> bool {
    obstacle_list.iter().any(|obstacle| {
        point.distance(&Point2D::new(obstacle.x, obstacle.y)) <= obstacle.radius + robot_radius
    })
}

pub fn line_collision_check(
    first: Point2D,
    second: Point2D,
    obstacle_list: &[CircleObstacle],
    robot_radius: f64,
    sample_step: f64,
) -> bool {
    let dx = second.x - first.x;
    let dy = second.y - first.y;
    let length = dx.hypot(dy);

    if length <= f64::EPSILON {
        return !is_point_collision(first, obstacle_list, robot_radius);
    }

    let steps = (length / sample_step) as usize + 1;
    for step in 0..=steps {
        let t = step as f64 / steps as f64;
        let point = Point2D::new(first.x + t * dx, first.y + t * dy);
        if is_point_collision(point, obstacle_list, robot_radius) {
            return false;
        }
    }

    true
}

pub fn shortcut_path_smoothing(
    path: &Path2D,
    max_iter: usize,
    obstacle_list: &[CircleObstacle],
    robot_radius: f64,
    sample_step: f64,
) -> Path2D {
    let mut rng = rand::thread_rng();
    shortcut_path_smoothing_with_sampler(
        path,
        max_iter,
        obstacle_list,
        robot_radius,
        sample_step,
        move |length| {
            let first = rng.gen_range(0.0..=length);
            let second = rng.gen_range(0.0..=length);
            (first, second)
        },
    )
}

fn shortcut_path_smoothing_with_sampler<F>(
    path: &Path2D,
    max_iter: usize,
    obstacle_list: &[CircleObstacle],
    robot_radius: f64,
    sample_step: f64,
    mut sampler: F,
) -> Path2D
where
    F: FnMut(f64) -> (f64, f64),
{
    let mut path = path.clone();
    let mut length = get_path_length(&path);

    for _ in 0..max_iter {
        let (first_pick, second_pick) = sampler(length);
        let (first_pick, second_pick) = if first_pick <= second_pick {
            (first_pick, second_pick)
        } else {
            (second_pick, first_pick)
        };

        let Some(first) = get_target_point(&path, first_pick) else {
            continue;
        };
        let Some(second) = get_target_point(&path, second_pick) else {
            continue;
        };

        if first.anchor_index <= 0 || second.anchor_index <= 0 {
            continue;
        }
        if second.anchor_index as usize + 1 > path.len() {
            continue;
        }
        if second.anchor_index == first.anchor_index {
            continue;
        }
        if !line_collision_check(
            first.point,
            second.point,
            obstacle_list,
            robot_radius,
            sample_step,
        ) {
            continue;
        }

        let mut new_points = Vec::new();
        new_points.extend_from_slice(&path.points[..=first.anchor_index as usize]);
        new_points.push(first.point);
        new_points.push(second.point);
        new_points.extend_from_slice(&path.points[second.anchor_index as usize + 1..]);
        path = Path2D::from_points(new_points);
        length = get_path_length(&path);
    }

    path
}

impl PathPlanner for RRTPathSmoothingPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let raw_path = self.rrt.plan(start, goal)?;
        Ok(shortcut_path_smoothing(
            &raw_path,
            self.smoothing.max_iter,
            self.rrt.get_obstacles(),
            self.robot_radius,
            self.smoothing.sample_step,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_point_close(actual: &Point2D, expected: [f64; 2]) {
        assert_close(actual.x, expected[0]);
        assert_close(actual.y, expected[1]);
    }

    fn parse_xy_fixture(csv: &str) -> Path2D {
        let points = csv
            .lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let (x, y) = line
                    .split_once(',')
                    .expect("xy fixture rows must contain a comma");
                Point2D::new(x.parse().unwrap(), y.parse().unwrap())
            })
            .collect();
        Path2D::from_points(points)
    }

    fn pythonrobotics_obstacles() -> Vec<CircleObstacle> {
        vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
        ]
    }

    #[test]
    fn test_target_point_matches_pythonrobotics_reference() {
        let path = parse_xy_fixture(include_str!("testdata/rrt_main_seed12345_path.csv"));
        let target = get_target_point(&path, 5.042_711_784_190_443).unwrap();
        assert_eq!(target.anchor_index, 1);
        assert_point_close(
            &target.point,
            [1.607_799_680_222_086_9, 3.021_515_605_739_562_5],
        );
    }

    #[test]
    fn test_shortcut_path_smoothing_fixed_schedule_matches_pythonrobotics_reference() {
        let path = parse_xy_fixture(include_str!("testdata/rrt_main_seed12345_path.csv"));
        let obstacles = pythonrobotics_obstacles();
        let picks = [
            (5.042_711_784_190_443, 9.593_003_338_960_78),
            (9.062_254_143_395_58, 14.126_262_302_507_822),
            (18.836_964_234_995_584, 24.764_066_566_436_444),
            (18.305_913_662_407_01, 20.679_131_888_162_48),
            (13.387_208_342_923_934, 17.057_633_477_275_18),
            (5.353_949_073_814_717, 12.050_990_819_270_86),
        ];
        let mut pick_iter = picks.iter().copied();

        let smoothed =
            shortcut_path_smoothing_with_sampler(&path, picks.len(), &obstacles, 0.3, 0.2, |_| {
                pick_iter.next().expect("pick schedule exhausted")
            });

        let expected = [
            [0.0, 0.0],
            [1.976_107_921_083_293, 2.257_210_110_785_406],
            [1.856_622_402_011_424, 2.505_163_940_383_516],
            [-0.798_753_011_969_844_9, 6.802_754_367_664_581],
            [-1.179_802_666_459_188_3, 9.161_294_659_229_442],
            [-0.313_449_454_851_734_9, 10.861_642_151_758_218],
            [-0.600_781_734_440_396_5, 11.906_895_396_420_353],
            [-0.267_061_812_685_109_03, 12.274_920_423_720_9],
            [1.817_773_784_492_888_7, 13.672_274_852_803_103],
            [2.390_944_751_282_898, 13.046_732_672_990_332],
            [5.705_791_577_974_288, 11.515_986_653_768_12],
            [5.860_033_119_067_657, 10.721_216_347_248_003],
            [6.0, 10.0],
        ];

        assert_eq!(smoothed.len(), expected.len());
        for (actual, expected) in smoothed.points.iter().zip(expected.iter()) {
            assert_point_close(actual, *expected);
        }
        assert_close(smoothed.total_length(), 22.759_016_831_433_197);
    }

    #[test]
    fn test_smoothed_path_points_remain_safe() {
        let path = parse_xy_fixture(include_str!("testdata/rrt_main_seed12345_path.csv"));
        let obstacles = pythonrobotics_obstacles();
        let picks = [
            (5.042_711_784_190_443, 9.593_003_338_960_78),
            (9.062_254_143_395_58, 14.126_262_302_507_822),
            (18.836_964_234_995_584, 24.764_066_566_436_444),
            (18.305_913_662_407_01, 20.679_131_888_162_48),
            (13.387_208_342_923_934, 17.057_633_477_275_18),
            (5.353_949_073_814_717, 12.050_990_819_270_86),
        ];
        let mut pick_iter = picks.iter().copied();
        let smoothed =
            shortcut_path_smoothing_with_sampler(&path, picks.len(), &obstacles, 0.5, 0.2, |_| {
                pick_iter.next().expect("pick schedule exhausted")
            });

        for point in &smoothed.points {
            for obstacle in &obstacles {
                let distance = point.distance(&Point2D::new(obstacle.x, obstacle.y));
                assert!(
                    distance > obstacle.radius + 0.5,
                    "point ({:.6}, {:.6}) too close to obstacle ({:.6}, {:.6})",
                    point.x,
                    point.y,
                    obstacle.x,
                    obstacle.y
                );
            }
        }
    }
}
