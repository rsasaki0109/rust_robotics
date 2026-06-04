//! Rigid-body planning backend benchmark.
//!
//! Compares the deterministic lattice A* backend (the MIP-style fallback) with
//! the sampling RRT backend on identical SE(2) scenes. Both backends share the
//! same geometric feasibility machinery (pose and swept-segment separation
//! certificates), so the only thing that differs is the search strategy.
//!
//! The lattice backend runs once per scene (it is deterministic). The RRT
//! backend runs over a fixed sweep of seeds so we can report a success rate and
//! the spread of path length / sample effort without any `thread_rng`
//! nondeterminism. Results are written to CSV and a small SVG bar chart.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::planning::{
    RigidBodyConvexObstacle2D, RigidBodyMipConfig2D, RigidBodyMipPlanner2D,
    RigidBodyPlanningBackend, RigidBodyPose2D, RigidBodyRrtBackend2D, RigidBodyRrtConfig2D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/rigid-body-backend-benchmark.csv";
const SVG_OUTPUT: &str = "docs/assets/rigid-body-backend-benchmark.svg";
const RRT_SEEDS: &[u64] = &[1, 2, 3, 4, 5, 6, 7, 8];

struct Scene {
    name: &'static str,
    config: RigidBodyMipConfig2D,
    start: RigidBodyPose2D,
    goal: RigidBodyPose2D,
    require_goal_heading: bool,
}

fn open_detour_scene() -> RoboticsResult<Scene> {
    let config = RigidBodyMipConfig2D {
        obstacles: vec![RigidBodyConvexObstacle2D::aabb(3.0, 5.0, 0.0, 3.0)?],
        robot_half_length: 0.4,
        robot_half_width: 0.2,
        max_expansions: 60_000,
        ..RigidBodyMipConfig2D::new(0.0, 8.0, 0.0, 6.0)
    };
    Ok(Scene {
        name: "open-detour",
        config,
        start: RigidBodyPose2D::new(0.5, 4.5, 0.0),
        goal: RigidBodyPose2D::new(7.5, 1.5, 0.0),
        require_goal_heading: false,
    })
}

fn twin_pillars_scene() -> RoboticsResult<Scene> {
    let config = RigidBodyMipConfig2D {
        obstacles: vec![
            RigidBodyConvexObstacle2D::aabb(2.5, 3.5, 0.0, 2.5)?,
            RigidBodyConvexObstacle2D::aabb(5.0, 6.0, 3.5, 6.0)?,
        ],
        robot_half_length: 0.4,
        robot_half_width: 0.25,
        max_expansions: 80_000,
        ..RigidBodyMipConfig2D::new(0.0, 9.0, 0.0, 6.0)
    };
    Ok(Scene {
        name: "twin-pillars",
        config,
        start: RigidBodyPose2D::new(0.5, 3.0, 0.0),
        goal: RigidBodyPose2D::new(8.5, 3.0, 0.0),
        require_goal_heading: false,
    })
}

#[derive(Debug, Clone)]
struct BackendStats {
    scene: &'static str,
    backend: &'static str,
    runs: usize,
    successes: usize,
    mean_path_length: f64,
    mean_heading_change: f64,
    mean_iterations: f64,
    min_separation_margin: f64,
}

impl BackendStats {
    fn success_rate(&self) -> f64 {
        if self.runs == 0 {
            0.0
        } else {
            self.successes as f64 / self.runs as f64
        }
    }
}

fn lattice_stats(scene: &Scene) -> BackendStats {
    let planner = RigidBodyMipPlanner2D::new(scene.config.clone()).expect("valid lattice config");
    match planner.plan_path(scene.start, scene.goal, scene.require_goal_heading) {
        Ok(outcome) => BackendStats {
            scene: scene.name,
            backend: outcome.backend,
            runs: 1,
            successes: 1,
            mean_path_length: outcome.path_length,
            mean_heading_change: outcome.heading_change,
            mean_iterations: outcome.iterations as f64,
            min_separation_margin: outcome.min_separation_margin,
        },
        Err(_) => BackendStats {
            scene: scene.name,
            backend: "lattice-astar",
            runs: 1,
            successes: 0,
            mean_path_length: 0.0,
            mean_heading_change: 0.0,
            mean_iterations: 0.0,
            min_separation_margin: 0.0,
        },
    }
}

fn rrt_stats(scene: &Scene) -> BackendStats {
    let mut successes = 0usize;
    let mut total_length = 0.0;
    let mut total_heading = 0.0;
    let mut total_iterations = 0.0;
    let mut min_margin = f64::INFINITY;
    for &seed in RRT_SEEDS {
        let backend =
            RigidBodyRrtBackend2D::new(scene.config.clone(), RigidBodyRrtConfig2D::new(seed))
                .expect("valid rrt config");
        if let Ok(outcome) = backend.plan_path(scene.start, scene.goal, scene.require_goal_heading)
        {
            successes += 1;
            total_length += outcome.path_length;
            total_heading += outcome.heading_change;
            total_iterations += outcome.iterations as f64;
            min_margin = min_margin.min(outcome.min_separation_margin);
        }
    }
    let denom = successes.max(1) as f64;
    BackendStats {
        scene: scene.name,
        backend: "rrt-se2",
        runs: RRT_SEEDS.len(),
        successes,
        mean_path_length: total_length / denom,
        mean_heading_change: total_heading / denom,
        mean_iterations: total_iterations / denom,
        min_separation_margin: if successes == 0 { 0.0 } else { min_margin },
    }
}

fn collect_stats() -> RoboticsResult<Vec<BackendStats>> {
    let scenes = [open_detour_scene()?, twin_pillars_scene()?];
    let mut stats = Vec::new();
    for scene in &scenes {
        stats.push(lattice_stats(scene));
        stats.push(rrt_stats(scene));
    }
    Ok(stats)
}

fn render_csv(stats: &[BackendStats]) -> String {
    let mut csv = String::from(
        "scene,backend,runs,successes,success_rate,mean_path_length,mean_heading_change,mean_iterations,min_separation_margin\n",
    );
    for row in stats {
        let _ = writeln!(
            csv,
            "{},{},{},{},{:.3},{:.3},{:.3},{:.1},{:.3}",
            row.scene,
            row.backend,
            row.runs,
            row.successes,
            row.success_rate(),
            row.mean_path_length,
            row.mean_heading_change,
            row.mean_iterations,
            row.min_separation_margin
        );
    }
    csv
}

fn render_svg(stats: &[BackendStats]) -> String {
    let width = 720.0;
    let height = 360.0;
    let left = 70.0;
    let bottom = 300.0;
    let plot_width = 600.0;
    let max_length = stats
        .iter()
        .map(|row| row.mean_path_length)
        .fold(1.0_f64, f64::max);

    let mut svg = String::new();
    let _ = writeln!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\" viewBox=\"0 0 {width} {height}\">"
    );
    let _ = writeln!(
        svg,
        "<rect width=\"{width}\" height=\"{height}\" fill=\"#fbfbfd\"/>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Rigid-body backends: mean path length</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">lattice A* (deterministic fallback) vs RRT-SE(2) over {} seeds</text>",
        RRT_SEEDS.len()
    );

    // Axis baseline.
    let _ = writeln!(
        svg,
        "<line x1=\"{left}\" y1=\"{bottom}\" x2=\"{}\" y2=\"{bottom}\" stroke=\"#c7c7cc\" stroke-width=\"1\"/>",
        left + plot_width
    );

    let group_count = stats.len() / 2;
    let group_width = plot_width / group_count as f64;
    let bar_width = group_width * 0.3;
    for group in 0..group_count {
        let lattice = &stats[group * 2];
        let rrt = &stats[group * 2 + 1];
        let group_x = left + group as f64 * group_width;
        let center = group_x + group_width / 2.0;

        for (offset, row, color) in [
            (-bar_width * 0.6, lattice, "#0a84ff"),
            (bar_width * 0.6, rrt, "#ff9f0a"),
        ] {
            let bar_height = row.mean_path_length / max_length * (bottom - 80.0);
            let bar_x = center + offset - bar_width / 2.0;
            let bar_y = bottom - bar_height;
            let _ = writeln!(
                svg,
                "<rect x=\"{bar_x:.1}\" y=\"{bar_y:.1}\" width=\"{bar_width:.1}\" height=\"{bar_height:.1}\" fill=\"{color}\" rx=\"3\"/>"
            );
            let _ = writeln!(
                svg,
                "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\" text-anchor=\"middle\">{:.1}</text>",
                bar_x + bar_width / 2.0,
                bar_y - 5.0,
                row.mean_path_length
            );
        }
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            lattice.scene
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">RRT {}/{} solved</text>",
            bottom + 38.0,
            rrt.successes,
            rrt.runs
        );
    }

    // Legend.
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"70\" width=\"14\" height=\"14\" fill=\"#0a84ff\" rx=\"3\"/>",
        left + plot_width - 150.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"82\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">lattice A*</text>",
        left + plot_width - 130.0
    );
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"90\" width=\"14\" height=\"14\" fill=\"#ff9f0a\" rx=\"3\"/>",
        left + plot_width - 150.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"102\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">RRT-SE(2)</text>",
        left + plot_width - 130.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(stats: &[BackendStats]) {
    println!("rigid-body backend benchmark (lattice A* vs RRT-SE2)");
    for row in stats {
        println!(
            "  {:<12} {:<13} success={}/{} ({:.0}%) path_len={:.2} heading={:.2} iters={:.1} min_margin={:.3}",
            row.scene,
            row.backend,
            row.successes,
            row.runs,
            row.success_rate() * 100.0,
            row.mean_path_length,
            row.mean_heading_change,
            row.mean_iterations,
            row.min_separation_margin
        );
    }
}

fn main() -> RoboticsResult<()> {
    let stats = collect_stats()?;
    print_summary(&stats);

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&stats)).ok();
    fs::write(SVG_OUTPUT, render_svg(&stats)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lattice_solves_every_scene() {
        for row in collect_stats().unwrap() {
            if row.backend == "lattice-astar" {
                assert_eq!(row.successes, 1, "lattice failed scene {}", row.scene);
                assert!(row.mean_path_length > 0.0);
            }
        }
    }

    #[test]
    fn rrt_is_deterministic_in_aggregate() {
        let first = collect_stats().unwrap();
        let second = collect_stats().unwrap();
        for (a, b) in first.iter().zip(second.iter()) {
            assert_eq!(a.successes, b.successes);
            assert!((a.mean_path_length - b.mean_path_length).abs() < 1.0e-9);
            assert!((a.mean_iterations - b.mean_iterations).abs() < 1.0e-9);
        }
    }

    #[test]
    fn rrt_finds_paths_on_open_scene() {
        let stats = collect_stats().unwrap();
        let open_rrt = stats
            .iter()
            .find(|row| row.scene == "open-detour" && row.backend == "rrt-se2")
            .unwrap();
        assert!(
            open_rrt.successes >= RRT_SEEDS.len() / 2,
            "RRT solved only {}/{} open-detour seeds",
            open_rrt.successes,
            open_rrt.runs
        );
        assert!(open_rrt.min_separation_margin > 0.0);
    }
}
