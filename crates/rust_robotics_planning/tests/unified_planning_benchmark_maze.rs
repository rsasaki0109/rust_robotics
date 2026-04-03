use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::bidirectional_a_star::{
    BidirectionalAStarConfig, BidirectionalAStarPlanner,
};
use rust_robotics_planning::bidirectional_bfs::{BidirectionalBFSConfig, BidirectionalBFSPlanner};
use rust_robotics_planning::breadth_first_search::{BFSConfig, BFSPlanner};
use rust_robotics_planning::depth_first_search::{DFSConfig, DFSPlanner};
use rust_robotics_planning::flow_field::{FlowFieldConfig, FlowFieldPlanner};
use rust_robotics_planning::greedy_best_first_search::{
    GreedyBestFirstConfig, GreedyBestFirstPlanner,
};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const GRID_SIZE: i32 = 50;
const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;

fn open_grid_obstacles() -> Obstacles {
    let mut obs = Obstacles::new();
    for i in 0..=GRID_SIZE {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, GRID_SIZE as f64));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(GRID_SIZE as f64, i as f64));
    }
    obs
}

fn draw_vertical_line(obstacles: &mut Obstacles, x: i32, start_y: i32, len: i32) {
    for y in start_y..start_y + len {
        obstacles.push(Point2D::new(x as f64, y as f64));
    }
}

fn draw_horizontal_line(obstacles: &mut Obstacles, start_x: i32, y: i32, len: i32) {
    for x in start_x..start_x + len {
        obstacles.push(Point2D::new(x as f64, y as f64));
    }
}

fn maze_obstacles() -> Obstacles {
    let mut obs = open_grid_obstacles();

    let vertical_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45];
    let vertical_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25];
    let vertical_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15];
    for ((x, y), len) in vertical_x
        .iter()
        .zip(vertical_y.iter())
        .zip(vertical_len.iter())
    {
        draw_vertical_line(&mut obs, *x, *y, *len);
    }

    let horizontal_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40];
    let horizontal_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45];
    let horizontal_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5];
    for ((x, y), len) in horizontal_x
        .iter()
        .zip(horizontal_y.iter())
        .zip(horizontal_len.iter())
    {
        draw_horizontal_line(&mut obs, *x, *y, *len);
    }

    obs
}

fn maze_start() -> Point2D {
    Point2D::new(5.0, 5.0)
}

fn maze_goal() -> Point2D {
    Point2D::new(35.0, 45.0)
}

macro_rules! assert_maze_path {
    ($name:literal, $planner:expr) => {{
        let path = $planner
            .plan(maze_start(), maze_goal())
            .unwrap_or_else(|err| panic!("{} failed to solve the benchmark maze: {}", $name, err));
        assert!(
            !path.is_empty(),
            "{} returned an empty path for the benchmark maze",
            $name
        );
        assert!(
            path.len() >= 2,
            "{} returned a degenerate path for the benchmark maze",
            $name
        );
    }};
}

#[test]
fn benchmark_maze_is_solvable_for_all_grid_planners() {
    let obstacles = maze_obstacles();

    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();
    let bfs = BFSPlanner::from_obstacle_points(
        &obstacles,
        BFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();
    let dfs = DFSPlanner::from_obstacle_points(
        &obstacles,
        DFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();
    let greedy = GreedyBestFirstPlanner::from_obstacle_points(
        &obstacles,
        GreedyBestFirstConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();
    let bidirectional_a_star = BidirectionalAStarPlanner::from_obstacle_points(
        &obstacles,
        BidirectionalAStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();
    let bidirectional_bfs = BidirectionalBFSPlanner::from_obstacle_points(
        &obstacles,
        BidirectionalBFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();
    let jps = JPSPlanner::from_obstacle_points(
        &obstacles,
        JPSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();
    let theta_star = ThetaStarPlanner::from_obstacle_points(
        &obstacles,
        ThetaStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();
    let flow_field = FlowFieldPlanner::from_obstacle_points(
        &obstacles,
        FlowFieldConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();

    assert_maze_path!("A*", a_star);
    assert_maze_path!("BFS", bfs);
    assert_maze_path!("DFS", dfs);
    assert_maze_path!("GreedyBestFirst", greedy);
    assert_maze_path!("BidirectionalA*", bidirectional_a_star);
    assert_maze_path!("BidirectionalBFS", bidirectional_bfs);
    assert_maze_path!("JPS", jps);
    assert_maze_path!("Theta*", theta_star);
    assert_maze_path!("FlowField", flow_field);
}
