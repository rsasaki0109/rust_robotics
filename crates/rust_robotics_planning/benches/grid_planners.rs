use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

fn create_obstacles() -> Obstacles {
    let mut obstacles = Obstacles::new();
    for i in 0..=20 {
        obstacles.push(Point2D::new(i as f64, 0.0));
        obstacles.push(Point2D::new(i as f64, 20.0));
        obstacles.push(Point2D::new(0.0, i as f64));
        obstacles.push(Point2D::new(20.0, i as f64));
    }
    for i in 5..15 {
        obstacles.push(Point2D::new(10.0, i as f64));
    }
    obstacles
}

fn bench_grid_planners(c: &mut Criterion) {
    let obstacles = create_obstacles();
    let start = Point2D::new(2.0, 10.0);
    let goal = Point2D::new(18.0, 10.0);

    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();

    let jps = JPSPlanner::from_obstacle_points(
        &obstacles,
        JPSConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();

    let theta_star = ThetaStarPlanner::from_obstacle_points(
        &obstacles,
        ThetaStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();

    let mut group = c.benchmark_group("grid_planners");

    group.bench_function("a_star", |b| {
        b.iter(|| a_star.plan(black_box(start), black_box(goal)).unwrap())
    });

    group.bench_function("jps", |b| {
        b.iter(|| jps.plan(black_box(start), black_box(goal)).unwrap())
    });

    group.bench_function("theta_star", |b| {
        b.iter(|| theta_star.plan(black_box(start), black_box(goal)).unwrap())
    });

    group.finish();
}

criterion_group!(benches, bench_grid_planners);
criterion_main!(benches);
