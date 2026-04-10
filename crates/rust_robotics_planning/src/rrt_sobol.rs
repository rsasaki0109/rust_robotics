#![allow(clippy::too_many_arguments)]

//! RRT with Sobol low-discrepancy sampling.
//!
//! This mirrors PythonRobotics `PathPlanning/RRT/rrt_with_sobol_sampler.py`
//! while reusing the existing Rust `RRTPlanner` for steering and collision
//! handling.

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

use crate::rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTNode, RRTPlanner};

const SOBOL_BITS: usize = 30;
const SOBOL_SCALE: f64 = 1.0 / ((1u64 << SOBOL_BITS) as f64);

#[derive(Debug, Clone)]
struct SobolSequence2D {
    index: usize,
    lastq: [u32; 2],
    directions: [[u32; SOBOL_BITS]; 2],
}

impl SobolSequence2D {
    fn new() -> Self {
        let mut directions = [[0_u32; SOBOL_BITS]; 2];
        for (column, direction) in directions[0].iter_mut().enumerate() {
            *direction = 1_u32 << (SOBOL_BITS - column - 1);
        }

        let mut numerators = [0_u32; SOBOL_BITS];
        numerators[0] = 1;
        for column in 1..SOBOL_BITS {
            numerators[column] = numerators[column - 1] ^ (numerators[column - 1] << 1);
        }
        for (column, numerator) in numerators.iter().copied().enumerate() {
            directions[1][column] = numerator << (SOBOL_BITS - column - 1);
        }

        Self {
            index: 0,
            lastq: [0, 0],
            directions,
        }
    }

    fn next(&mut self) -> [f64; 2] {
        let point = [
            self.lastq[0] as f64 * SOBOL_SCALE,
            self.lastq[1] as f64 * SOBOL_SCALE,
        ];

        let bit = rightmost_zero_bit_index(self.index);
        assert!(
            bit < SOBOL_BITS,
            "Sobol sequence exceeded supported bit depth of {SOBOL_BITS}"
        );

        self.lastq[0] ^= self.directions[0][bit];
        self.lastq[1] ^= self.directions[1][bit];
        self.index += 1;

        point
    }

    fn next_in_bounds(&mut self, bounds: &AreaBounds) -> [f64; 2] {
        let point = self.next();
        [
            bounds.xmin + point[0] * (bounds.xmax - bounds.xmin),
            bounds.ymin + point[1] * (bounds.ymax - bounds.ymin),
        ]
    }
}

fn rightmost_zero_bit_index(mut value: usize) -> usize {
    let mut bit = 0;
    loop {
        if value & 1 == 0 {
            return bit;
        }
        value >>= 1;
        bit += 1;
    }
}

/// RRT planner variant that uses a Sobol low-discrepancy sampler.
pub struct RRTSobolPlanner {
    config: RRTConfig,
    obstacles: Vec<CircleObstacle>,
    play_area: Option<AreaBounds>,
    rand_area: AreaBounds,
    node_list: Vec<RRTNode>,
}

impl RRTSobolPlanner {
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        play_area: Option<AreaBounds>,
        config: RRTConfig,
    ) -> Self {
        Self {
            config,
            obstacles,
            play_area,
            rand_area,
            node_list: Vec::new(),
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
    ) -> Self {
        let obstacles = obstacle_list
            .into_iter()
            .map(|(x, y, r)| CircleObstacle::new(x, y, r))
            .collect();
        let config = RRTConfig {
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            robot_radius,
        };
        let rand_bounds = AreaBounds::new(rand_area[0], rand_area[1], rand_area[0], rand_area[1]);
        let play_bounds = play_area.map(AreaBounds::from_array);
        Self::new(obstacles, rand_bounds, play_bounds, config)
    }

    pub fn planning(&mut self, start: [f64; 2], goal: [f64; 2]) -> Option<Vec<[f64; 2]>> {
        let start_pt = Point2D::new(start[0], start[1]);
        let goal_pt = Point2D::new(goal[0], goal[1]);
        match self.plan_and_store(start_pt, goal_pt) {
            Ok(path) => Some(path.points.iter().map(|point| [point.x, point.y]).collect()),
            Err(_) => None,
        }
    }

    pub fn get_tree(&self) -> &[RRTNode] {
        &self.node_list
    }

    pub fn get_obstacles(&self) -> &[CircleObstacle] {
        &self.obstacles
    }

    fn plan_and_store(&mut self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let mut planner = RRTPlanner::new(
            self.obstacles.clone(),
            self.rand_area.clone(),
            self.play_area.clone(),
            self.config.clone(),
        );
        let goal_sample_rate = self.config.goal_sample_rate;
        let goal_x = goal.x;
        let goal_y = goal.y;
        let rand_area = self.rand_area.clone();
        let mut sobol = SobolSequence2D::new();
        let mut rng = rand::thread_rng();

        let path = planner.plan_with_sampler(start, goal, move |_| {
            if rng.gen_range(0..=100) > goal_sample_rate {
                let sample = sobol.next_in_bounds(&rand_area);
                RRTNode::new(sample[0], sample[1])
            } else {
                RRTNode::new(goal_x, goal_y)
            }
        })?;

        self.node_list = planner.get_tree().to_vec();
        Ok(path)
    }
}

impl PathPlanner for RRTSobolPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let mut planner = RRTSobolPlanner::new(
            self.obstacles.clone(),
            self.rand_area.clone(),
            self.play_area.clone(),
            self.config.clone(),
        );
        planner.plan_and_store(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_point_close(actual: [f64; 2], expected: [f64; 2]) {
        assert_close(actual[0], expected[0]);
        assert_close(actual[1], expected[1]);
    }

    fn create_pythonrobotics_main_planner(goal_sample_rate: i32) -> RRTSobolPlanner {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
            CircleObstacle::new(8.0, 10.0, 1.0),
        ];
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let config = RRTConfig {
            goal_sample_rate,
            robot_radius: 0.8,
            ..Default::default()
        };
        RRTSobolPlanner::new(obstacles, rand_area, None, config)
    }

    #[test]
    fn test_sobol_sequence_first_points_match_pythonrobotics_reference() {
        let mut sampler = SobolSequence2D::new();
        let bounds = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let expected = [
            [-2.0, -2.0],
            [6.5, 6.5],
            [10.75, 2.25],
            [2.25, 10.75],
            [4.375, 4.375],
            [12.875, 12.875],
            [8.625, 0.125],
            [0.125, 8.625],
            [1.1875, 3.3125],
            [9.6875, 11.8125],
        ];

        for expected_point in expected {
            let actual = sampler.next_in_bounds(&bounds);
            assert_point_close(actual, expected_point);
        }
    }

    #[test]
    fn test_rrt_sobol_goal_sampling_disabled_matches_pythonrobotics_reference() {
        let mut planner = create_pythonrobotics_main_planner(-1);
        let path = planner.planning([0.0, 0.0], [6.0, 10.0]).unwrap();
        let expected_path = [
            [0.0, 0.0],
            [2.121_320_343_559_643, 2.121_320_343_559_642_4],
            [5.120_986_802_026_766, 2.166_054_423_938_105_4],
            [7.713_273_910_301_528, 0.656_071_792_159_690_4],
            [10.619_533_093_777_92, -0.088_009_895_868_675],
            [10.982_945_821_776_296, 2.889_897_286_890_515_3],
            [12.652_536_566_531_195, 5.382_379_331_967_025],
            [10.613_696_586_776_955, 7.583_090_929_827_073],
            [11.146_612_940_692_071, 10.535_378_205_775_984],
            [12.257_944_072_169_929, 13.321_942_957_617_964],
            [8.890_625, 13.671_875],
            [6.898_437_5, 13.273_437_5],
            [7.695_312_5, 12.476_562_5],
            [6.699_218_75, 12.144_531_25],
            [6.0, 10.0],
        ];

        assert_eq!(planner.get_tree().len(), 86);
        assert_eq!(path.len(), expected_path.len());
        for (actual, expected) in path.iter().copied().zip(expected_path.iter().copied()) {
            assert_point_close(actual, expected);
        }

        let expected_nodes = [
            (
                1,
                [-2.0, -2.0],
                Some(0),
                7,
                [0.0, -0.353_553_390_593_273_73, -0.707_106_781_186_547_5],
                [0.0, -0.353_553_390_593_273_8, -FRAC_1_SQRT_2],
            ),
            (
                2,
                [2.121_320_343_559_643, 2.121_320_343_559_642_4],
                Some(0),
                7,
                [0.0, 0.353_553_390_593_273_8, FRAC_1_SQRT_2],
                [0.0, 0.353_553_390_593_273_73, FRAC_1_SQRT_2],
            ),
            (
                6,
                [10.619_533_093_777_92, -0.088_009_895_868_675],
                Some(4),
                7,
                [
                    7.713_273_910_301_528,
                    8.197_650_440_880_928,
                    8.682_026_971_460_326,
                ],
                [
                    0.656_071_792_159_690_4,
                    0.532_058_177_488_296_1,
                    0.408_044_562_816_901_9,
                ],
            ),
            (
                11,
                [5.968_75, -0.406_25],
                Some(4),
                6,
                [
                    7.713_273_910_301_528,
                    7.286_222_224_748_394,
                    6.859_170_539_195_259,
                ],
                [
                    0.656_071_792_159_690_4,
                    0.396_020_147_546_112_5,
                    0.135_968_502_932_534_6,
                ],
            ),
        ];

        for (index, xy, parent, path_len, path_x3, path_y3) in expected_nodes {
            let node = &planner.get_tree()[index];
            assert_close(node.x, xy[0]);
            assert_close(node.y, xy[1]);
            assert_eq!(node.parent, parent);
            assert_eq!(node.path_x.len(), path_len);
            for (actual, expected) in node.path_x.iter().take(3).zip(path_x3.iter()) {
                assert_close(*actual, *expected);
            }
            for (actual, expected) in node.path_y.iter().take(3).zip(path_y3.iter()) {
                assert_close(*actual, *expected);
            }
        }
    }
}
