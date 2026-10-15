//! Export grid-planner paths on the observation-loss experiment's fixed maps.
//! Usage: export_clearance_paths INPUT NEW_OUTPUT RESOLUTION PLANNING_RADIUS

use rust_robotics_core::{Path2D, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 5 {
        return Err(
            "usage: export_clearance_paths INPUT NEW_OUTPUT RESOLUTION PLANNING_RADIUS".into(),
        );
    }
    let input = Path::new(&args[1]);
    let output = Path::new(&args[2]);
    let resolution: f64 = args[3].parse()?;
    let robot_radius: f64 = args[4].parse()?;
    let mut maps: BTreeMap<String, Vec<Point2D>> = BTreeMap::new();
    for line in std::fs::read_to_string(input.join("obstacles.csv"))?
        .lines()
        .skip(1)
    {
        let f: Vec<&str> = line.split(',').collect();
        maps.entry(f[0].into())
            .or_default()
            .push(Point2D::new(f[1].parse()?, f[2].parse()?));
    }
    let mut endpoints: BTreeMap<String, (Point2D, Point2D)> = BTreeMap::new();
    for line in std::fs::read_to_string(input.join("paths.csv"))?
        .lines()
        .skip(1)
    {
        let f: Vec<&str> = line.split(',').collect();
        if f[1] == "A*" {
            let p = Point2D::new(f[3].parse()?, f[4].parse()?);
            endpoints
                .entry(f[0].into())
                .and_modify(|pair| pair.1 = p)
                .or_insert((p, p));
        }
    }
    assert_eq!(maps.len(), 3);
    assert_eq!(endpoints.len(), 3);
    // The caller chooses a new directory; never overwrite an existing fixture.
    std::fs::create_dir(output)?;
    std::fs::copy(input.join("obstacles.csv"), output.join("obstacles.csv"))?;
    let mut paths = BufWriter::new(File::create(output.join("paths.csv"))?);
    let mut metrics = BufWriter::new(File::create(output.join("planning.csv"))?);
    writeln!(paths, "scenario,planner,index,x,y")?;
    writeln!(
        metrics,
        "scenario,planner,resolution,planning_radius,status,path_length,waypoints,jps_fallback"
    )?;
    for (scenario, obstacles) in maps {
        let ox: Vec<f64> = obstacles.iter().map(|p| p.x).collect();
        let oy: Vec<f64> = obstacles.iter().map(|p| p.y).collect();
        let (start, goal) = endpoints[&scenario];
        let a = AStarPlanner::try_new(
            &ox,
            &oy,
            AStarConfig {
                resolution,
                robot_radius,
                heuristic_weight: 1.0,
            },
        )?;
        let j = JPSPlanner::try_new(
            &ox,
            &oy,
            JPSConfig {
                resolution,
                robot_radius,
                heuristic_weight: 1.0,
            },
        )?;
        let t = ThetaStarPlanner::try_new(
            &ox,
            &oy,
            ThetaStarConfig {
                resolution,
                robot_radius,
                heuristic_weight: 1.0,
            },
        )?;
        let jp = j.plan_with_diagnostics(start, goal);
        let fallback = jp.as_ref().is_ok_and(|r| r.diagnostics.used_fallback);
        let plans = [
            ("A*", a.plan(start, goal), false),
            ("JPS", jp.map(|r| r.path), fallback),
            ("Theta*", t.plan(start, goal), false),
        ];
        for (planner, result, fallback) in plans {
            match result {
                Ok(path) => {
                    validate_endpoints(&path, start, goal);
                    for (index, p) in path.points.iter().enumerate() {
                        writeln!(paths, "{scenario},{planner},{index},{},{}", p.x, p.y)?;
                    }
                    writeln!(metrics, "{scenario},{planner},{resolution},{robot_radius},planned,{:.9},{},{fallback}", path.total_length(), path.len())?;
                }
                Err(error) => {
                    // Planning failures are outcomes, never replaced by nominal paths.
                    eprintln!("{scenario}/{planner}: {error}");
                    writeln!(metrics, "{scenario},{planner},{resolution},{robot_radius},planning_failed,0,0,{fallback}")?;
                }
            }
        }
    }
    paths.flush()?;
    metrics.flush()?;
    Ok(())
}

fn validate_endpoints(path: &Path2D, start: Point2D, goal: Point2D) {
    assert!(path.len() >= 2);
    assert!(path.points[0].distance(&start) < 1e-9);
    assert!(path.points.last().unwrap().distance(&goal) < 1e-9);
    assert!(path
        .points
        .iter()
        .all(|p| p.x.is_finite() && p.y.is_finite()));
    assert!(path.points.windows(2).all(|p| p[0].distance(&p[1]) > 0.0));
}
