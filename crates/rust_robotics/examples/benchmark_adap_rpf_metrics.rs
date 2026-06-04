//! Adap-RPF metric sweep: fixed back-point vs adaptive following.
//!
//! Runs the closed-loop person-following MPPI under a sweep of occlusion and
//! moving-pedestrian scenarios, for two following strategies:
//!
//! * `fixed` — always aim for the point a desired distance directly behind the
//!   target (the naive baseline);
//! * `adaptive` — the Adap-RPF-lite target-centric sampler that sidesteps
//!   occluders and crowders.
//!
//! For each (scenario, strategy) it reports the target visibility ratio, the
//! target spacing ratio, the minimum pedestrian clearance, and collision steps,
//! using the library `PersonFollowingMetricsAccumulator2D`. Everything is
//! seeded and deterministic. Results are written to CSV and an SVG bar chart.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    MppiConfig, MppiController2D, MppiMovingObstacle2D, MppiPersonFollowingConfig2D,
    MppiPersonFollowingSampler2D, MppiState2D, PersonFollowingMetricsAccumulator2D,
    PersonFollowingMetricsConfig2D, PersonFollowingRolloutMetrics2D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/adap-rpf-metrics-sweep.csv";
const SVG_OUTPUT: &str = "docs/assets/adap-rpf-metrics-sweep.svg";
const STEPS: usize = 40;
const TARGET_SPEED: f64 = 0.30;
const SEED: u64 = 421;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scenario {
    Open,
    TrailingOccluder,
    CrossingPedestrians,
    DenseOcclusion,
}

impl Scenario {
    fn name(self) -> &'static str {
        match self {
            Scenario::Open => "open",
            Scenario::TrailingOccluder => "trailing-occluder",
            Scenario::CrossingPedestrians => "crossing-peds",
            Scenario::DenseOcclusion => "dense-occlusion",
        }
    }
}

const SCENARIOS: &[Scenario] = &[
    Scenario::Open,
    Scenario::TrailingOccluder,
    Scenario::CrossingPedestrians,
    Scenario::DenseOcclusion,
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Strategy {
    Fixed,
    Adaptive,
}

impl Strategy {
    fn name(self) -> &'static str {
        match self {
            Strategy::Fixed => "fixed",
            Strategy::Adaptive => "adaptive",
        }
    }
}

fn target_at(step: usize) -> MppiMovingObstacle2D {
    let t = step as f64 * 0.1;
    MppiMovingObstacle2D::new(TARGET_SPEED * t, 0.0, TARGET_SPEED, 0.0, 0.30)
}

fn pedestrians_at(
    scenario: Scenario,
    step: usize,
    desired_distance: f64,
) -> Vec<MppiMovingObstacle2D> {
    let target = target_at(step);
    let trailing = |y: f64| {
        // Pedestrian camped on (or just beside) the naive back-point, occluding
        // the robot-target line of sight there.
        MppiMovingObstacle2D::new(target.x - desired_distance, y, target.vx, target.vy, 0.32)
    };
    let crossers = || {
        // Pedestrians drift laterally across the following region, sweeping
        // through the robot-target line of sight without camping on the robot.
        vec![
            MppiMovingObstacle2D::new(
                target.x - 0.55,
                1.35 - 0.06 * step as f64,
                0.04,
                -0.06,
                0.26,
            ),
            MppiMovingObstacle2D::new(
                target.x - 0.85,
                -1.35 + 0.055 * step as f64,
                0.06,
                0.055,
                0.24,
            ),
        ]
    };
    match scenario {
        Scenario::Open => Vec::new(),
        Scenario::TrailingOccluder => vec![trailing(0.0)],
        Scenario::CrossingPedestrians => crossers(),
        Scenario::DenseOcclusion => {
            // Occluder sits just beside the back-point, leaving the adaptive
            // follower a clean offset pocket the fixed strategy never uses.
            let mut people = crossers();
            people.push(trailing(-0.32));
            people
        }
    }
}

fn base_config(seed: u64) -> MppiConfig {
    MppiConfig {
        horizon: 16,
        samples: 360,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.06,
        max_lambda: 22.0,
        target_effective_sample_ratio: 0.24,
        adaptive_lambda_iterations: 22,
        noise_sigma: 1.15,
        control_limit: 2.7,
        goal_weight: 1.25,
        terminal_weight: 2.8,
        velocity_weight: 0.06,
        control_weight: 0.018,
        constraint_weight: 520.0,
        constraint_discount: 0.92,
        safety_margin: 0.18,
        seed,
        ..MppiConfig::default()
    }
}

fn fixed_goal_trajectory(
    follower: MppiPersonFollowingConfig2D,
    target: MppiMovingObstacle2D,
) -> Vec<(f64, f64)> {
    (0..=follower.horizon)
        .map(|step| {
            let predicted = target.predict(step as f64 * follower.dt);
            (predicted.x - follower.desired_distance, predicted.y)
        })
        .collect()
}

#[derive(Debug, Clone)]
struct Row {
    scenario: &'static str,
    strategy: &'static str,
    metrics: PersonFollowingRolloutMetrics2D,
}

fn run(scenario: Scenario, strategy: Strategy) -> RoboticsResult<Row> {
    let follower = MppiPersonFollowingConfig2D::default();
    let mut sampler = MppiPersonFollowingSampler2D::new(follower)?;
    let mut controller = MppiController2D::new(base_config(SEED))?;
    let mut metrics = PersonFollowingMetricsAccumulator2D::new(PersonFollowingMetricsConfig2D {
        desired_distance: follower.desired_distance,
        ..PersonFollowingMetricsConfig2D::default()
    })?;
    let mut state = MppiState2D::new(-1.65, -0.08, 0.0, 0.0);

    for step in 0..STEPS {
        let target = target_at(step);
        let pedestrians = pedestrians_at(scenario, step, follower.desired_distance);
        controller.set_moving_obstacles(pedestrians.clone())?;

        let goal = match strategy {
            Strategy::Fixed => {
                controller.set_goal_trajectory(Some(fixed_goal_trajectory(follower, target)))?;
                (target.x - follower.desired_distance, target.y)
            }
            Strategy::Adaptive => {
                let candidate = sampler.select_following_point(state, target, &pedestrians)?;
                controller.set_goal_trajectory(Some(
                    sampler.goal_trajectory_for_candidate(target, candidate),
                ))?;
                (candidate.x, candidate.y)
            }
        };

        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, follower.dt);
        metrics.record_step(state, target, &pedestrians)?;
    }

    Ok(Row {
        scenario: scenario.name(),
        strategy: strategy.name(),
        metrics: metrics.finish(),
    })
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let mut rows = Vec::new();
    for &scenario in SCENARIOS {
        for strategy in [Strategy::Fixed, Strategy::Adaptive] {
            rows.push(run(scenario, strategy)?);
        }
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,strategy,visibility_ratio,spacing_ratio,mean_spacing,mean_spacing_error,min_clearance,collision_steps\n",
    );
    for row in rows {
        let m = &row.metrics;
        let _ = writeln!(
            csv,
            "{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{}",
            row.scenario,
            row.strategy,
            m.target_visibility_ratio(),
            m.target_spacing_ratio(),
            m.mean_spacing(),
            m.mean_spacing_error(),
            m.min_clearance,
            m.collision_steps
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let width = 760.0;
    let height = 360.0;
    let left = 70.0;
    let bottom = 290.0;
    let plot_width = 620.0;
    let plot_height = bottom - 90.0;

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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Adap-RPF: target visibility ratio by scenario</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">fixed back-point vs adaptive following ({STEPS}-step closed loop, seed {SEED})</text>"
    );

    // y gridlines at 0, 0.5, 1.0 visibility ratio.
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

    let group_count = SCENARIOS.len();
    let group_width = plot_width / group_count as f64;
    let bar_width = group_width * 0.28;
    for (group, &scenario) in SCENARIOS.iter().enumerate() {
        let fixed = rows
            .iter()
            .find(|row| row.scenario == scenario.name() && row.strategy == "fixed")
            .unwrap();
        let adaptive = rows
            .iter()
            .find(|row| row.scenario == scenario.name() && row.strategy == "adaptive")
            .unwrap();
        let center = left + group as f64 * group_width + group_width / 2.0;

        for (offset, row, color) in [
            (-bar_width * 0.6, fixed, "#8e8e93"),
            (bar_width * 0.6, adaptive, "#34c759"),
        ] {
            let ratio = row.metrics.target_visibility_ratio();
            let bar_height = ratio * plot_height;
            let bar_x = center + offset - bar_width / 2.0;
            let bar_y = bottom - bar_height;
            let _ = writeln!(
                svg,
                "<rect x=\"{bar_x:.1}\" y=\"{bar_y:.1}\" width=\"{bar_width:.1}\" height=\"{bar_height:.1}\" fill=\"{color}\" rx=\"3\"/>"
            );
            let _ = writeln!(
                svg,
                "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\" text-anchor=\"middle\">{:.0}%</text>",
                bar_x + bar_width / 2.0,
                bar_y - 5.0,
                ratio * 100.0
            );
        }
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            scenario.name()
        );
    }

    // Legend.
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"70\" width=\"14\" height=\"14\" fill=\"#8e8e93\" rx=\"3\"/>",
        left + plot_width - 130.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"82\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">fixed</text>",
        left + plot_width - 110.0
    );
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"90\" width=\"14\" height=\"14\" fill=\"#34c759\" rx=\"3\"/>",
        left + plot_width - 130.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"102\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">adaptive</text>",
        left + plot_width - 110.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("adap-rpf metric sweep (fixed back-point vs adaptive following)");
    for row in rows {
        let m = &row.metrics;
        println!(
            "  {:<17} {:<9} visibility={:.0}% spacing={:.0}% mean_spacing={:.2} min_clearance={:.2} collisions={}",
            row.scenario,
            row.strategy,
            m.target_visibility_ratio() * 100.0,
            m.target_spacing_ratio() * 100.0,
            m.mean_spacing(),
            m.min_clearance,
            m.collision_steps
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
            assert_eq!(a.metrics, b.metrics);
        }
    }

    #[test]
    fn open_scenario_keeps_target_visible() {
        for strategy in [Strategy::Fixed, Strategy::Adaptive] {
            let row = run(Scenario::Open, strategy).unwrap();
            assert!(
                row.metrics.target_visibility_ratio() > 0.99,
                "{} lost visibility on the open scene",
                strategy.name()
            );
        }
    }

    #[test]
    fn adaptive_beats_fixed_under_trailing_occlusion() {
        let fixed = run(Scenario::TrailingOccluder, Strategy::Fixed).unwrap();
        let adaptive = run(Scenario::TrailingOccluder, Strategy::Adaptive).unwrap();

        // The naive back-point sits on the occluder; the adaptive sampler steps
        // aside, so it should keep the target visible more often and never run
        // straight into the camped pedestrian.
        assert!(
            adaptive.metrics.target_visibility_ratio() > fixed.metrics.target_visibility_ratio()
        );
        assert!(adaptive.metrics.min_clearance > fixed.metrics.min_clearance);
    }

    #[test]
    fn adaptive_never_collides_and_holds_visibility_across_sweep() {
        for &scenario in SCENARIOS {
            let fixed = run(scenario, Strategy::Fixed).unwrap();
            let adaptive = run(scenario, Strategy::Adaptive).unwrap();
            assert_eq!(
                adaptive.metrics.collision_steps,
                0,
                "adaptive collided in {}",
                scenario.name()
            );
            assert!(
                adaptive.metrics.target_visibility_ratio()
                    >= fixed.metrics.target_visibility_ratio() - 1e-9,
                "adaptive lost visibility vs fixed in {}",
                scenario.name()
            );
        }
    }
}
