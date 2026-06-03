//! Render reference-free racing MPPI as a static SVG.
//!
//! The output compares waypoint-reference MPPI with a gate-progress objective
//! inspired by agile drone racing MPPI.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiGateRace2D, MppiRacingGate2D,
    MppiState2D, MppiWaypointTrack2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/mppi-racing-gate-progress.svg";
const STEPS: usize = 44;
const LEFT: f64 = 58.0;
const TOP: f64 = 88.0;
const PLOT_WIDTH: f64 = 660.0;
const PLOT_HEIGHT: f64 = 360.0;
const X_MIN: f64 = -0.16;
const X_MAX: f64 = 2.82;
const Y_MIN: f64 = -0.68;
const Y_MAX: f64 = 0.62;
const GRID_STEP: usize = 3;

#[derive(Debug, Clone)]
struct RacingRun {
    label: &'static str,
    color: &'static str,
    states: Vec<MppiState2D>,
    passed_gates: usize,
    active_gate_index: usize,
    final_track_progress: f64,
    final_gate_distance: f64,
    final_lateral_error: f64,
    min_clearance: f64,
    mean_ess: f64,
}

#[derive(Debug, Clone)]
struct RacingScene {
    track: MppiWaypointTrack2D,
    race: MppiGateRace2D,
    obstacles: Vec<MppiCircularObstacle2D>,
    waypoint: RacingRun,
    gate_progress: RacingRun,
}

fn gate(
    center: (f64, f64),
    previous: (f64, f64),
    half_width: f64,
) -> RoboticsResult<MppiRacingGate2D> {
    MppiRacingGate2D::new(
        center.0,
        center.1,
        center.0 - previous.0,
        center.1 - previous.1,
        half_width,
    )
}

fn gate_centers() -> Vec<(f64, f64)> {
    vec![
        (0.38, 0.18),
        (0.86, -0.14),
        (1.35, 0.18),
        (1.86, -0.12),
        (2.40, 0.14),
    ]
}

fn track() -> RoboticsResult<MppiWaypointTrack2D> {
    let mut waypoints = vec![(0.0, 0.0)];
    waypoints.extend(gate_centers());
    MppiWaypointTrack2D::new(waypoints)
}

fn race() -> RoboticsResult<MppiGateRace2D> {
    let mut previous = (0.0, 0.0);
    let mut gates = Vec::new();
    for center in gate_centers() {
        gates.push(gate(center, previous, 0.28)?);
        previous = center;
    }
    let mut race = MppiGateRace2D::new(gates)?;
    race.progress_weight = 6.5;
    race.lateral_weight = 0.35;
    race.pass_bonus = 9.0;
    race.miss_penalty = 42.0;
    race.terminal_gate_weight = 1.6;
    Ok(race)
}

fn obstacles() -> Vec<MppiCircularObstacle2D> {
    vec![
        MppiCircularObstacle2D::new(0.66, 0.32, 0.11),
        MppiCircularObstacle2D::new(1.12, -0.34, 0.11),
        MppiCircularObstacle2D::new(1.62, 0.34, 0.11),
        MppiCircularObstacle2D::new(2.16, -0.32, 0.11),
    ]
}

fn base_config(obstacles: Vec<MppiCircularObstacle2D>, seed: u64) -> MppiConfig {
    MppiConfig {
        horizon: 22,
        samples: 520,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.06,
        max_lambda: 28.0,
        target_effective_sample_ratio: 0.22,
        adaptive_lambda_iterations: 24,
        noise_sigma: 1.55,
        control_limit: 4.0,
        velocity_weight: 0.02,
        control_weight: 0.018,
        obstacles,
        constraint_weight: 620.0,
        constraint_discount: 0.9,
        safety_margin: 0.10,
        seed,
        ..MppiConfig::default()
    }
}

fn min_clearance(state: MppiState2D, obstacles: &[MppiCircularObstacle2D]) -> f64 {
    obstacles
        .iter()
        .map(|obstacle| {
            let dx = state.x - obstacle.x;
            let dy = state.y - obstacle.y;
            (dx * dx + dy * dy).sqrt() - obstacle.radius
        })
        .fold(f64::INFINITY, f64::min)
}

fn run_waypoint(
    track: &MppiWaypointTrack2D,
    race: &MppiGateRace2D,
    obstacles: &[MppiCircularObstacle2D],
) -> RoboticsResult<RacingRun> {
    let value_grid = track.terminal_value_grid(108, 76, -0.35, -1.05, 0.035, 5.4, 13.0)?;
    let config = MppiConfig {
        goal_weight: 0.25,
        terminal_weight: 0.65,
        terminal_value_grid: Some(value_grid),
        terminal_value_weight: 1.0,
        ..base_config(obstacles.to_vec(), 331)
    };
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut states = vec![state];
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_ess = 0.0;

    for _ in 0..STEPS {
        let projection_before = track.project(state.x, state.y)?;
        let goal = track.point_at_progress(projection_before.progress + 0.82)?;
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, dt);
        states.push(state);
        min_seen_clearance = min_seen_clearance.min(min_clearance(state, obstacles));
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
    }
    let projection = track.project(state.x, state.y)?;
    let gate_report = race.score_trajectory(&states)?;
    Ok(RacingRun {
        label: "waypoint reference",
        color: "#2563eb",
        states,
        passed_gates: gate_report.passed_gates,
        active_gate_index: gate_report.active_gate_index,
        final_track_progress: projection.progress,
        final_gate_distance: gate_report.final_gate_distance,
        final_lateral_error: gate_report.final_lateral_error,
        min_clearance: min_seen_clearance,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn run_gate_progress(
    track: &MppiWaypointTrack2D,
    race: &MppiGateRace2D,
    obstacles: &[MppiCircularObstacle2D],
) -> RoboticsResult<RacingRun> {
    let config = MppiConfig {
        goal_weight: 0.0,
        terminal_weight: 0.0,
        velocity_weight: 0.0,
        gate_race: Some(race.clone()),
        ..base_config(obstacles.to_vec(), 337)
    };
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut states = vec![state];
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_ess = 0.0;
    let mut executed_steps = 0;

    for _ in 0..STEPS {
        let plan = controller.plan(state, (0.0, 0.0))?;
        state = state.step(plan.first_control, dt);
        states.push(state);
        min_seen_clearance = min_seen_clearance.min(min_clearance(state, obstacles));
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        executed_steps += 1;
        if race.score_trajectory(&states)?.passed_gates == race.gate_count() {
            break;
        }
    }
    let projection = track.project(state.x, state.y)?;
    let gate_report = race.score_trajectory(&states)?;
    let mean_ess = if executed_steps > 0 {
        mean_ess / executed_steps as f64
    } else {
        0.0
    };
    Ok(RacingRun {
        label: "gate progress",
        color: "#dc2626",
        states,
        passed_gates: gate_report.passed_gates,
        active_gate_index: gate_report.active_gate_index,
        final_track_progress: projection.progress,
        final_gate_distance: gate_report.final_gate_distance,
        final_lateral_error: gate_report.final_lateral_error,
        min_clearance: min_seen_clearance,
        mean_ess,
    })
}

fn build_scene() -> RoboticsResult<RacingScene> {
    let track = track()?;
    let race = race()?;
    let obstacles = obstacles();
    let waypoint = run_waypoint(&track, &race, &obstacles)?;
    let gate_progress = run_gate_progress(&track, &race, &obstacles)?;
    Ok(RacingScene {
        track,
        race,
        obstacles,
        waypoint,
        gate_progress,
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

fn color_channel(value: f64) -> u8 {
    value.round().clamp(0.0, 255.0) as u8
}

fn rgb_hex(r: f64, g: f64, b: f64) -> String {
    format!(
        "#{:02x}{:02x}{:02x}",
        color_channel(r),
        color_channel(g),
        color_channel(b)
    )
}

fn interpolate_rgb(a: (f64, f64, f64), b: (f64, f64, f64), t: f64) -> String {
    let clamped = t.clamp(0.0, 1.0);
    rgb_hex(
        a.0 + (b.0 - a.0) * clamped,
        a.1 + (b.1 - a.1) * clamped,
        a.2 + (b.2 - a.2) * clamped,
    )
}

fn distance_color(value: f64, max_value: f64) -> String {
    let t = (value / max_value.max(1e-9)).clamp(0.0, 1.0);
    if t < 0.5 {
        interpolate_rgb((254.0, 242.0, 242.0), (252.0, 211.0, 77.0), t * 2.0)
    } else {
        interpolate_rgb((252.0, 211.0, 77.0), (37.0, 99.0, 235.0), (t - 0.5) * 2.0)
    }
}

fn trajectory_points(states: &[MppiState2D]) -> String {
    states
        .iter()
        .map(|state| format!("{:.1},{:.1}", sx(state.x), sy(state.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn track_points(track: &MppiWaypointTrack2D) -> String {
    track
        .waypoints
        .iter()
        .map(|&(x, y)| format!("{:.1},{:.1}", sx(x), sy(y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_first_gate_distance_field(svg: &mut String, race: &MppiGateRace2D) {
    let gate = race.gates[0];
    let max_value = gate.squared_distance(X_MAX, Y_MIN).sqrt();
    let cell_w = PLOT_WIDTH * 0.04 * GRID_STEP as f64 / (X_MAX - X_MIN);
    let cell_h = PLOT_HEIGHT * 0.04 * GRID_STEP as f64 / (Y_MAX - Y_MIN);
    for x in (0..76).step_by(GRID_STEP) {
        for y in (0..38).step_by(GRID_STEP) {
            let wx = X_MIN + x as f64 * 0.04;
            let wy = Y_MIN + y as f64 * 0.04;
            let fill = distance_color(gate.squared_distance(wx, wy).sqrt(), max_value);
            writeln!(
                svg,
                r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="{fill}" opacity="0.20"/>"##,
                sx(wx) - cell_w / 2.0,
                sy(wy) - cell_h / 2.0,
                cell_w + 0.6,
                cell_h + 0.6
            )
            .unwrap();
        }
    }
}

fn render_run(svg: &mut String, run: &RacingRun, stroke_width: f64) {
    let points = trajectory_points(&run.states);
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="0.88"/>"##,
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
        if index % 6 == 0 || index + 1 == run.states.len() {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="4.0" fill="{}" stroke="#ffffff" stroke-width="1.4"/>"##,
                sx(state.x),
                sy(state.y),
                run.color
            )
            .unwrap();
        }
    }
}

fn render_gate(svg: &mut String, gate: &MppiRacingGate2D, index: usize) {
    let tangent_x = -gate.normal_y;
    let tangent_y = gate.normal_x;
    let x0 = gate.center_x - tangent_x * gate.half_width;
    let y0 = gate.center_y - tangent_y * gate.half_width;
    let x1 = gate.center_x + tangent_x * gate.half_width;
    let y1 = gate.center_y + tangent_y * gate.half_width;
    let arrow_x = gate.center_x + gate.normal_x * 0.14;
    let arrow_y = gate.center_y + gate.normal_y * 0.14;
    writeln!(
        svg,
        r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#111827" stroke-width="8" stroke-linecap="round" opacity="0.22"/>"##,
        sx(x0),
        sy(y0),
        sx(x1),
        sy(y1)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#111827" stroke-width="3.2" stroke-linecap="round"/>"##,
        sx(x0),
        sy(y0),
        sx(x1),
        sy(y1)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#111827" stroke-width="2" stroke-linecap="round" marker-end="url(#arrow)"/>"##,
        sx(gate.center_x),
        sy(gate.center_y),
        sx(arrow_x),
        sy(arrow_y)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="11" fill="#ffffff" stroke="#111827" stroke-width="1.5"/>"##,
        sx(gate.center_x),
        sy(gate.center_y)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">G{}</text>"##,
        sx(gate.center_x),
        sy(gate.center_y),
        index + 1
    )
    .unwrap();
}

fn render_stats(svg: &mut String, run: &RacingRun, x: f64, y: f64) {
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
        format!("gates      {}/5", run.passed_gates),
        format!("next gate  {}", run.active_gate_index),
        format!("track s    {:.2}", run.final_track_progress),
        format!("gate dist  {:.2}", run.final_gate_distance),
        format!("lateral    {:.2}", run.final_lateral_error),
        format!("clearance  {:.2}", run.min_clearance),
        format!("ESS ratio  {:.2}", run.mean_ess),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            y + 30.0 + index as f64 * 21.0
        )
        .unwrap();
    }
}

fn render_svg(scene: &RacingScene) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="980" height="560" viewBox="0 0 980 560" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Reference-free racing MPPI</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">A gate course comparing waypoint-reference MPPI with gate-progress MPPI.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="980" height="560" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="5" refY="3" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L0,6 L6,3 z" fill="#111827"/></marker><clipPath id="plot-clip"><rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8"/></clipPath></defs>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Reference-free racing MPPI</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Gate progress directly rewards passing sequential gates; no time-indexed reference trajectory is used.</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="#ffffff" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(svg, r##"<g clip-path="url(#plot-clip)">"##).unwrap();
    render_first_gate_distance_field(&mut svg, &scene.race);
    writeln!(svg, "</g>").unwrap();

    let track_polyline = track_points(&scene.track);
    writeln!(
        svg,
        r##"<polyline points="{track_polyline}" fill="none" stroke="#111827" stroke-width="18" stroke-linecap="round" stroke-linejoin="round" opacity="0.08"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{track_polyline}" fill="none" stroke="#64748b" stroke-width="3.0" stroke-linecap="round" stroke-linejoin="round" stroke-dasharray="9 8" opacity="0.88"/>"##
    )
    .unwrap();

    for obstacle in &scene.obstacles {
        let ox = sx(obstacle.x);
        let oy = sy(obstacle.y);
        writeln!(
            svg,
            r##"<ellipse cx="{ox:.1}" cy="{oy:.1}" rx="{:.1}" ry="{:.1}" fill="none" stroke="#ef4444" stroke-width="2" stroke-dasharray="6 5" opacity="0.72"/>"##,
            sdx(obstacle.radius + 0.10),
            sdy(obstacle.radius + 0.10)
        )
        .unwrap();
        writeln!(
            svg,
            r##"<ellipse cx="{ox:.1}" cy="{oy:.1}" rx="{:.1}" ry="{:.1}" fill="#111827" opacity="0.85"/>"##,
            sdx(obstacle.radius),
            sdy(obstacle.radius)
        )
        .unwrap();
    }

    for (index, gate) in scene.race.gates.iter().enumerate() {
        render_gate(&mut svg, gate, index);
    }

    render_run(&mut svg, &scene.waypoint, 4.4);
    render_run(&mut svg, &scene.gate_progress, 5.2);

    let start = scene.waypoint.states[0];
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="12" fill="#111827" stroke="#ffffff" stroke-width="3"/>"##,
        sx(start.x),
        sy(start.y)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">S</text>"##,
        sx(start.x),
        sy(start.y)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="none" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();

    render_stats(&mut svg, &scene.waypoint, 754.0, 112.0);
    render_stats(&mut svg, &scene.gate_progress, 754.0, 296.0);

    writeln!(
        svg,
        r##"<text x="58" y="490" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Black bars are oriented gates; arrows show the positive crossing direction. Blue follows waypoint-track terminal values, red uses only the gate-progress race objective.</text>"##
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
        "wrote {OUTPUT} gates waypoint={}/{} gate_progress={}/{}",
        scene.waypoint.passed_gates,
        scene.race.gate_count(),
        scene.gate_progress.passed_gates,
        scene.race.gate_count()
    );
    Ok(())
}
