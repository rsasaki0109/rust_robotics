use std::collections::HashSet;

use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::dijkstra::{DijkstraConfig, DijkstraPlanner};

// An offset world and a one-cell gap catch coordinate swaps and accidental
// extra obstacle inflation when comparing the two public planner APIs.
fn world(blocked: bool) -> Obstacles {
    let mut obstacles = Obstacles::new();
    for x in 0..=12 {
        for y in 0..=12 {
            if x == 0 || x == 12 || y == 0 || y == 12 || (x == 6 && (blocked || y != 4)) {
                obstacles.push(Point2D::new(f64::from(x) - 5.0, f64::from(y) + 7.0));
            }
        }
    }
    obstacles
}

#[test]
fn observation_preserves_paths_and_reports_real_unique_expansions() {
    let obstacles = world(false);
    let start = Point2D::new(-3.0, 11.0);
    let goal = Point2D::new(5.0, 11.0);
    let astar = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            robot_radius: 0.35,
            ..Default::default()
        },
    )
    .unwrap();
    let dijkstra = DijkstraPlanner::from_obstacle_points(
        &obstacles,
        DijkstraConfig {
            robot_radius: 0.35,
            ..Default::default()
        },
    )
    .unwrap();
    let (normal, stats) = astar.plan_with_stats(start, goal).unwrap();
    let mut a_cells = Vec::new();
    let observed = astar
        .plan_with_observer(start, goal, |p| a_cells.push(p))
        .unwrap();
    assert_eq!(observed.points, normal.points);
    assert_eq!(a_cells.len(), stats.expanded_nodes);

    let mut d_cells = Vec::new();
    let d_path = dijkstra
        .plan_with_observer(start, goal, |p| d_cells.push(p))
        .unwrap();
    assert_eq!(d_path.points, dijkstra.plan(start, goal).unwrap().points);
    assert!((d_path.total_length() - normal.total_length()).abs() < 1e-10);
    assert!(
        a_cells.len() < d_cells.len(),
        "zero heuristic should explore more of this field"
    );
    for cells in [&a_cells, &d_cells] {
        assert_eq!(cells.first(), Some(&start));
        assert_eq!(cells.last(), Some(&goal));
        let unique: HashSet<_> = cells.iter().map(|p| (p.x as i32, p.y as i32)).collect();
        assert_eq!(unique.len(), cells.len());
        assert!(cells.iter().all(|p| astar.grid_map().is_valid(
            astar.grid_map().calc_x_index(p.x),
            astar.grid_map().calc_y_index(p.y)
        )));
    }
    let mut again = Vec::new();
    astar
        .plan_with_observer(start, goal, |p| again.push(p))
        .unwrap();
    assert_eq!(again, a_cells, "replaying a query changed expansion order");
}

#[test]
fn failed_search_retains_exploration_but_invalid_queries_emit_nothing() {
    let obstacles = world(true);
    let astar = AStarPlanner::from_obstacle_points(&obstacles, AStarConfig::default()).unwrap();
    let dijkstra =
        DijkstraPlanner::from_obstacle_points(&obstacles, DijkstraConfig::default()).unwrap();
    let start = Point2D::new(-3.0, 11.0);
    let goal = Point2D::new(5.0, 11.0);
    let mut a_cells = Vec::new();
    let mut d_cells = Vec::new();
    assert!(astar
        .plan_with_observer(start, goal, |p| a_cells.push(p))
        .is_err());
    assert!(dijkstra
        .plan_with_observer(start, goal, |p| d_cells.push(p))
        .is_err());
    for cells in [&a_cells, &d_cells] {
        assert!(!cells.is_empty());
        assert!(cells.iter().all(|p| p.x < 1.0));
        assert!(!cells.contains(&goal));
    }
    let mut invalid = Vec::new();
    assert!(astar
        .plan_with_observer(Point2D::new(1.0, 10.0), goal, |p| invalid.push(p))
        .is_err());
    assert!(dijkstra
        .plan_with_observer(start, Point2D::new(50.0, 50.0), |p| invalid.push(p))
        .is_err());
    assert!(invalid.is_empty());

    let mut single = Vec::new();
    let path = astar
        .plan_with_observer(start, start, |p| single.push(p))
        .unwrap();
    assert_eq!(single, vec![start]);
    assert_eq!(path.points, vec![start]);
}
