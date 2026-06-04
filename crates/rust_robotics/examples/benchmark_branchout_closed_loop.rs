//! BranchOut closed-loop driving metrics across receding-horizon scenarios.
//!
//! Each scenario runs the multimodal BranchOut planner in a receding-horizon
//! closed loop: at every control step the planner re-plans from the current ego
//! pose, the highest-probability mode is committed with a bounded-rate tracking
//! step, the (optionally moving) traffic advances, and the closed-loop driving
//! counters are accumulated — route completion, no-collision rate, comfort, and
//! time-to-collision. Everything is deterministic. Results go to CSV and SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::planning::{
    BranchOutClosedLoopConfig2D, BranchOutClosedLoopMetrics2D, BranchOutDrivingScene2D,
    BranchOutObstacle2D, BranchOutPlanner2D, BranchOutPlannerConfig2D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/branchout-closed-loop.csv";
const SVG_OUTPUT: &str = "docs/assets/branchout-closed-loop.svg";

struct Scenario {
    name: &'static str,
    scene: BranchOutDrivingScene2D,
    obstacle_velocities: Vec<(f64, f64)>,
}

fn scenarios() -> Vec<Scenario> {
    // 1. Room to pass: the planner overtakes and completes the route.
    let overtake = Scenario {
        name: "wide-overtake",
        scene: BranchOutDrivingScene2D::wide_overtake(),
        obstacle_velocities: vec![(0.0, 0.0)],
    };
    // 2. Single blocked lane: the planner must yield behind the obstacle.
    let forced_yield = Scenario {
        name: "forced-yield",
        scene: BranchOutDrivingScene2D::forced_yield(),
        obstacle_velocities: vec![(0.0, 0.0)],
    };
    // 3. Slow lead car moving forward: the ego follows at a safe distance.
    let mut lead_scene = BranchOutDrivingScene2D::wide_overtake();
    lead_scene.obstacles = vec![BranchOutObstacle2D::new(3.0, 0.0, 0.42)];
    let moving_lead = Scenario {
        name: "moving-lead",
        scene: lead_scene,
        obstacle_velocities: vec![(0.8, 0.0)],
    };
    // 4. Obstacle approaching head-on in the ego lane: low time-to-collision.
    let mut oncoming_scene = BranchOutDrivingScene2D::wide_overtake();
    oncoming_scene.obstacles = vec![BranchOutObstacle2D::new(7.5, 0.0, 0.42)];
    let oncoming = Scenario {
        name: "oncoming",
        scene: oncoming_scene,
        obstacle_velocities: vec![(-1.4, 0.0)],
    };

    vec![overtake, forced_yield, moving_lead, oncoming]
}

#[derive(Debug, Clone)]
struct Row {
    name: &'static str,
    metrics: BranchOutClosedLoopMetrics2D,
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default())?;
    let config = BranchOutClosedLoopConfig2D::default();
    let mut rows = Vec::new();
    for scenario in scenarios() {
        let metrics =
            planner.simulate_closed_loop(&scenario.scene, &scenario.obstacle_velocities, config)?;
        rows.push(Row {
            name: scenario.name,
            metrics,
        });
    }
    Ok(rows)
}

fn format_ttc(ttc: f64) -> String {
    if ttc.is_finite() {
        format!("{ttc:.2}")
    } else {
        "inf".to_string()
    }
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,route_completion,reached_goal,no_collision_rate,collision_steps,min_clearance,mean_comfort_cost,min_time_to_collision,risky_ttc_steps\n",
    );
    for row in rows {
        let m = &row.metrics;
        let _ = writeln!(
            csv,
            "{},{:.3},{},{:.3},{},{:.3},{:.4},{},{}",
            row.name,
            m.route_completion,
            m.reached_goal,
            m.no_collision_rate,
            m.collision_steps,
            m.min_clearance,
            m.mean_comfort_cost,
            format_ttc(m.min_time_to_collision),
            m.risky_ttc_steps
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let width = 760.0;
    let height = 380.0;
    let left = 70.0;
    let bottom = 300.0;
    let plot_width = 620.0;
    let plot_height = bottom - 96.0;

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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">BranchOut closed loop: route completion by scenario</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">receding-horizon rollout; labels show no-collision rate and min time-to-collision</text>"
    );

    for ratio in [0.0, 0.5, 1.0] {
        let y = bottom - ratio * plot_height;
        let _ = writeln!(
            svg,
            "<line x1=\"{left}\" y1=\"{y:.1}\" x2=\"{:.1}\" y2=\"{y:.1}\" stroke=\"#e5e5ea\" stroke-width=\"1\"/>",
            left + plot_width
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"end\">{ratio:.1}</text>",
            left - 8.0,
            y + 4.0
        );
    }

    let group_width = plot_width / rows.len() as f64;
    let bar_width = group_width * 0.42;
    for (index, row) in rows.iter().enumerate() {
        let m = &row.metrics;
        let center = left + index as f64 * group_width + group_width / 2.0;
        let bar_height = m.route_completion * plot_height;
        let bar_x = center - bar_width / 2.0;
        let bar_y = bottom - bar_height;
        let color = if m.reached_goal { "#0a84ff" } else { "#ff9f0a" };
        let _ = writeln!(
            svg,
            "<rect x=\"{bar_x:.1}\" y=\"{bar_y:.1}\" width=\"{bar_width:.1}\" height=\"{bar_height:.1}\" fill=\"{color}\" rx=\"3\"/>"
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{:.0}%</text>",
            bar_y - 8.0,
            m.route_completion * 100.0
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            row.name
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">no-coll {:.0}% · ttc {}</text>",
            bottom + 38.0,
            m.no_collision_rate * 100.0,
            format_ttc(m.min_time_to_collision)
        );
    }

    // Legend.
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"70\" width=\"14\" height=\"14\" fill=\"#0a84ff\" rx=\"3\"/>",
        left + plot_width - 170.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"82\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">reached goal</text>",
        left + plot_width - 150.0
    );
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"90\" width=\"14\" height=\"14\" fill=\"#ff9f0a\" rx=\"3\"/>",
        left + plot_width - 170.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"102\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">yielded / incomplete</text>",
        left + plot_width - 150.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("branchout closed-loop driving metrics (receding horizon)");
    for row in rows {
        let m = &row.metrics;
        println!(
            "  {:<14} completion={:.0}% reached={} no_collision={:.0}% min_clearance={:.2} comfort={:.4} min_ttc={} risky_ttc={}",
            row.name,
            m.route_completion * 100.0,
            m.reached_goal,
            m.no_collision_rate * 100.0,
            m.min_clearance,
            m.mean_comfort_cost,
            format_ttc(m.min_time_to_collision),
            m.risky_ttc_steps
        );
    }
}

fn main() -> RoboticsResult<()> {
    let rows = collect_rows()?;
    print_summary(&rows);

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&rows)).ok();
    fs::write(SVG_OUTPUT, render_svg(&rows)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sweep_is_deterministic() {
        let first = collect_rows().unwrap();
        let second = collect_rows().unwrap();
        for (a, b) in first.iter().zip(second.iter()) {
            assert_eq!(a.metrics.executed_path, b.metrics.executed_path);
            assert_eq!(a.metrics.collision_steps, b.metrics.collision_steps);
        }
    }

    #[test]
    fn every_scenario_is_collision_free() {
        for row in collect_rows().unwrap() {
            assert_eq!(
                row.metrics.collision_steps, 0,
                "{} collided in closed loop",
                row.name
            );
            assert!(row.metrics.no_collision_rate > 0.99);
        }
    }

    #[test]
    fn overtake_reaches_goal_yield_does_not() {
        let rows = collect_rows().unwrap();
        let overtake = rows.iter().find(|row| row.name == "wide-overtake").unwrap();
        let forced_yield = rows.iter().find(|row| row.name == "forced-yield").unwrap();
        assert!(overtake.metrics.reached_goal);
        assert!(!forced_yield.metrics.reached_goal);
        assert!(overtake.metrics.route_completion > forced_yield.metrics.route_completion);
    }
}
