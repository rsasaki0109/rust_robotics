use rust_robotics::path_planning::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};

fn main() {
    let obstacles = vec![
        CircleObstacle::new(5.0, 5.0, 1.0),
        CircleObstacle::new(3.0, 6.0, 2.0),
        CircleObstacle::new(7.0, 8.0, 1.5),
    ];
    let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
    let mut planner = RRTPlanner::new(obstacles, rand_area, None, RRTConfig::default());

    match planner.planning([0.0, 0.0], [10.0, 10.0]) {
        Some(path) => {
            println!("RRT path found with {} points", path.len());
        }
        None => {
            eprintln!("RRT failed to find a path");
            std::process::exit(1);
        }
    }
}
