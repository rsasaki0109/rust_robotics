//! Render Adap-RPF-lite person-following MPPI as a static SVG.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::control::{
    MppiConfig, MppiController2D, MppiMovingObstacle2D, MppiPersonFollowingCandidate2D,
    MppiPersonFollowingConfig2D, MppiPersonFollowingSampler2D, MppiState2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/adap-rpf-lite-mppi.svg";
const STEPS: usize = 34;
const LEFT: f64 = 58.0;
const TOP: f64 = 86.0;
const PLOT_WIDTH: f64 = 690.0;
const PLOT_HEIGHT: f64 = 360.0;
const X_MIN: f64 = -1.85;
const X_MAX: f64 = 1.30;
const Y_MIN: f64 = -1.48;
const Y_MAX: f64 = 0.92;

#[derive(Debug, Clone)]
struct FollowingRun {
    label: &'static str,
    color: &'static str,
    states: Vec<MppiState2D>,
    candidates: Vec<MppiPersonFollowingCandidate2D>,
    final_distance: f64,
    min_clearance: f64,
    mean_occlusion: f64,
    mean_proximity: f64,
    mean_ess: f64,
}

#[derive(Debug, Clone)]
struct FollowingScene {
    target_states: Vec<MppiMovingObstacle2D>,
    blocker_states: Vec<MppiMovingObstacle2D>,
    fixed: FollowingRun,
    adaptive: FollowingRun,
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

fn target_at(step: usize) -> MppiMovingObstacle2D {
    let t = step as f64 * 0.1;
    MppiMovingObstacle2D::new(0.30 * t, 0.0, 0.30, 0.0, 0.30)
}

fn pedestrians_at(step: usize) -> Vec<MppiMovingObstacle2D> {
    let target = target_at(step);
    vec![
        MppiMovingObstacle2D::new(target.x - 0.92, 0.0, target.vx, target.vy, 0.34),
        MppiMovingObstacle2D::new(0.12 + 0.018 * step as f64, 0.72, 0.06, -0.10, 0.28),
    ]
}

fn fixed_goal_trajectory(
    config: MppiPersonFollowingConfig2D,
    target: MppiMovingObstacle2D,
) -> Vec<(f64, f64)> {
    (0..=config.horizon)
        .map(|step| {
            let predicted = target.predict(step as f64 * config.dt);
            (predicted.x - config.desired_distance, predicted.y)
        })
        .collect()
}

fn clearance_to_people(state: MppiState2D, people: &[MppiMovingObstacle2D]) -> f64 {
    people
        .iter()
        .map(|person| {
            let dx = state.x - person.x;
            let dy = state.y - person.y;
            (dx * dx + dy * dy).sqrt() - person.radius
        })
        .fold(f64::INFINITY, f64::min)
}

fn fixed_candidate(
    config: MppiPersonFollowingConfig2D,
    target: MppiMovingObstacle2D,
) -> MppiPersonFollowingCandidate2D {
    MppiPersonFollowingCandidate2D {
        x: target.x - config.desired_distance,
        y: target.y,
        offset_x: -config.desired_distance,
        offset_y: 0.0,
        distance_to_target: config.desired_distance,
        bearing: std::f64::consts::PI,
        total_cost: 0.0,
        visibility_cost: 0.0,
        proximity_cost: 0.0,
        distance_cost: 0.0,
        travel_cost: 0.0,
        stickiness_cost: 0.0,
        feasible: true,
    }
}

fn run_fixed_reference(
    follower_config: MppiPersonFollowingConfig2D,
) -> RoboticsResult<FollowingRun> {
    let mut controller = MppiController2D::new(base_config(407))?;
    let mut state = MppiState2D::new(-1.65, -0.08, 0.0, 0.0);
    let mut states = vec![state];
    let mut candidates = Vec::new();
    let mut min_clearance = f64::INFINITY;
    let mut mean_ess = 0.0;

    for step in 0..STEPS {
        let target = target_at(step);
        let pedestrians = pedestrians_at(step);
        let candidate = fixed_candidate(follower_config, target);
        controller.set_goal_trajectory(Some(fixed_goal_trajectory(follower_config, target)))?;
        controller.set_moving_obstacles(pedestrians.clone())?;
        let plan = controller.plan(state, (candidate.x, candidate.y))?;
        state = state.step(plan.first_control, follower_config.dt);
        states.push(state);
        candidates.push(candidate);
        min_clearance = min_clearance.min(clearance_to_people(state, &pedestrians));
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
    }
    let target = target_at(STEPS);
    Ok(FollowingRun {
        label: "fixed back point",
        color: "#2563eb",
        states,
        candidates,
        final_distance: ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt(),
        min_clearance,
        mean_occlusion: 0.0,
        mean_proximity: 0.0,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn run_adaptive(follower_config: MppiPersonFollowingConfig2D) -> RoboticsResult<FollowingRun> {
    let mut sampler = MppiPersonFollowingSampler2D::new(follower_config)?;
    let mut controller = MppiController2D::new(base_config(419))?;
    let mut state = MppiState2D::new(-1.65, -0.08, 0.0, 0.0);
    let mut states = vec![state];
    let mut candidates = Vec::new();
    let mut min_clearance = f64::INFINITY;
    let mut mean_occlusion = 0.0;
    let mut mean_proximity = 0.0;
    let mut mean_ess = 0.0;

    for step in 0..STEPS {
        let target = target_at(step);
        let pedestrians = pedestrians_at(step);
        let candidate = sampler.select_following_point(state, target, &pedestrians)?;
        controller.set_goal_trajectory(Some(
            sampler.goal_trajectory_for_candidate(target, candidate),
        ))?;
        controller.set_moving_obstacles(pedestrians.clone())?;
        let plan = controller.plan(state, (candidate.x, candidate.y))?;
        state = state.step(plan.first_control, follower_config.dt);
        states.push(state);
        candidates.push(candidate);
        min_clearance = min_clearance.min(clearance_to_people(state, &pedestrians));
        mean_occlusion += candidate.visibility_cost;
        mean_proximity += candidate.proximity_cost;
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
    }
    let target = target_at(STEPS);
    Ok(FollowingRun {
        label: "adaptive rpf",
        color: "#059669",
        states,
        candidates,
        final_distance: ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt(),
        min_clearance,
        mean_occlusion: mean_occlusion / STEPS as f64,
        mean_proximity: mean_proximity / STEPS as f64,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn build_scene() -> RoboticsResult<FollowingScene> {
    let follower_config = MppiPersonFollowingConfig2D::default();
    let target_states = (0..=STEPS).map(target_at).collect::<Vec<_>>();
    let blocker_states = (0..=STEPS)
        .map(|step| pedestrians_at(step)[0])
        .collect::<Vec<_>>();
    let fixed = run_fixed_reference(follower_config)?;
    let adaptive = run_adaptive(follower_config)?;
    Ok(FollowingScene {
        target_states,
        blocker_states,
        fixed,
        adaptive,
    })
}

fn sx(x: f64) -> f64 {
    LEFT + (x - X_MIN) / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn sy(y: f64) -> f64 {
    TOP + (Y_MAX - y) / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn sdx(distance: f64) -> f64 {
    distance / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn sdy(distance: f64) -> f64 {
    distance / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn trajectory_points(states: &[MppiState2D]) -> String {
    states
        .iter()
        .map(|state| format!("{:.1},{:.1}", sx(state.x), sy(state.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn obstacle_points(states: &[MppiMovingObstacle2D]) -> String {
    states
        .iter()
        .map(|state| format!("{:.1},{:.1}", sx(state.x), sy(state.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_run(svg: &mut String, run: &FollowingRun, stroke_width: f64) {
    let points = trajectory_points(&run.states);
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="0.90"/>"##,
        stroke_width + 5.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{}" stroke-width="{stroke_width:.1}" stroke-linecap="round" stroke-linejoin="round"/>"##,
        run.color
    )
    .unwrap();
    for (index, state) in run.states.iter().enumerate() {
        if index % 7 == 0 || index + 1 == run.states.len() {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="4.0" fill="{}" stroke="#ffffff" stroke-width="1.3"/>"##,
                sx(state.x),
                sy(state.y),
                run.color
            )
            .unwrap();
        }
    }
}

fn render_candidates(svg: &mut String, run: &FollowingRun) {
    for (index, candidate) in run.candidates.iter().enumerate() {
        if index % 3 == 0 || index + 1 == run.candidates.len() {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="3.5" fill="#f59e0b" stroke="#ffffff" stroke-width="1.0" opacity="{:.2}"/>"##,
                sx(candidate.x),
                sy(candidate.y),
                if run.label == "adaptive rpf" { 0.80 } else { 0.45 }
            )
            .unwrap();
        }
    }
}

fn render_people(svg: &mut String, scene: &FollowingScene) {
    let target_points = obstacle_points(&scene.target_states);
    let blocker_points = obstacle_points(&scene.blocker_states);
    writeln!(
        svg,
        r##"<polyline points="{target_points}" fill="none" stroke="#111827" stroke-width="3.2" stroke-linecap="round" stroke-dasharray="8 7"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{blocker_points}" fill="none" stroke="#ef4444" stroke-width="3.0" stroke-linecap="round" stroke-dasharray="6 6"/>"##
    )
    .unwrap();
    for (index, person) in scene.target_states.iter().enumerate() {
        if index % 8 == 0 || index + 1 == scene.target_states.len() {
            writeln!(
                svg,
                r##"<ellipse cx="{:.1}" cy="{:.1}" rx="{:.1}" ry="{:.1}" fill="#111827" opacity="0.88"/>"##,
                sx(person.x),
                sy(person.y),
                sdx(person.radius),
                sdy(person.radius)
            )
            .unwrap();
        }
    }
    for (index, person) in scene.blocker_states.iter().enumerate() {
        if index % 8 == 0 || index + 1 == scene.blocker_states.len() {
            writeln!(
                svg,
                r##"<ellipse cx="{:.1}" cy="{:.1}" rx="{:.1}" ry="{:.1}" fill="#ef4444" opacity="0.50" stroke="#991b1b" stroke-width="1.2"/>"##,
                sx(person.x),
                sy(person.y),
                sdx(person.radius),
                sdy(person.radius)
            )
            .unwrap();
        }
    }
}

fn render_stats(svg: &mut String, run: &FollowingRun, x: f64, y: f64) {
    writeln!(
        svg,
        r##"<line x1="{x:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="{}" stroke-width="7" stroke-linecap="round"/>"##,
        y - 4.0,
        x + 46.0,
        y - 4.0,
        run.color
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">{}</text>"##,
        x + 58.0,
        y,
        run.label
    )
    .unwrap();
    for (index, line) in [
        format!("target dist {:.2}", run.final_distance),
        format!("clearance   {:.2}", run.min_clearance),
        format!("visibility  {:.2}", run.mean_occlusion),
        format!("proximity   {:.2}", run.mean_proximity),
        format!("ESS ratio   {:.2}", run.mean_ess),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            y + 30.0 + index as f64 * 23.0
        )
        .unwrap();
    }
}

fn render_svg(scene: &FollowingScene) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1020" height="560" viewBox="0 0 1020 560" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(svg, r##"<title id="title">Adap-RPF-lite MPPI</title>"##).unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Adaptive person-following candidate sampling and MPPI tracking around a moving pedestrian occluder.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="1020" height="560" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Adap-RPF-lite person following</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Target-centric candidate sampling selects visible, socially safer following points before prediction-aware MPPI tracking.</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="#ffffff" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();
    render_candidates(&mut svg, &scene.fixed);
    render_candidates(&mut svg, &scene.adaptive);
    render_people(&mut svg, scene);
    render_run(&mut svg, &scene.fixed, 4.2);
    render_run(&mut svg, &scene.adaptive, 5.0);
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="none" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();

    render_stats(&mut svg, &scene.fixed, 784.0, 118.0);
    render_stats(&mut svg, &scene.adaptive, 784.0, 286.0);
    writeln!(
        svg,
        r##"<text x="58" y="494" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Black is the target, red is the predicted occluding pedestrian, amber dots are sampled/selected following points. Blue tracks a fixed back point; green tracks adaptive Adap-RPF-lite goals.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let scene = build_scene()?;
    let svg = render_svg(&scene)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} fixed_distance={:.2} adaptive_distance={:.2} fixed_clearance={:.2} adaptive_clearance={:.2}",
        scene.fixed.final_distance,
        scene.adaptive.final_distance,
        scene.fixed.min_clearance,
        scene.adaptive.min_clearance
    );
    Ok(())
}
