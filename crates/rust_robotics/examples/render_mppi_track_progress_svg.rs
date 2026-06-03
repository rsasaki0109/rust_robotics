//! Render track-progress MPPI as a static SVG.
//!
//! The output compares single-goal MPPI against a waypoint-track terminal value
//! controller on the same slalom course.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D, MppiTerminalValueGrid2D,
    MppiWaypointTrack2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/mppi-track-progress.svg";
const STEPS: usize = 20;
const LEFT: f64 = 56.0;
const TOP: f64 = 88.0;
const PLOT_WIDTH: f64 = 620.0;
const PLOT_HEIGHT: f64 = 340.0;
const X_MIN: f64 = -0.18;
const X_MAX: f64 = 2.92;
const Y_MIN: f64 = -0.92;
const Y_MAX: f64 = 0.92;
const GRID_STEP: usize = 2;

#[derive(Debug, Clone)]
struct TrackRun {
    label: &'static str,
    color: &'static str,
    states: Vec<MppiState2D>,
    final_progress: f64,
    final_lateral: f64,
    mean_lateral: f64,
    min_clearance: f64,
    mean_ess: f64,
}

#[derive(Debug, Clone)]
struct TrackScene {
    track: MppiWaypointTrack2D,
    obstacles: Vec<MppiCircularObstacle2D>,
    value_grid: MppiTerminalValueGrid2D,
    single_goal: TrackRun,
    track_progress: TrackRun,
}

fn track() -> RoboticsResult<MppiWaypointTrack2D> {
    MppiWaypointTrack2D::new(vec![
        (0.0, 0.0),
        (0.65, 0.55),
        (1.35, -0.55),
        (2.05, 0.55),
        (2.8, 0.0),
    ])
}

fn obstacles() -> Vec<MppiCircularObstacle2D> {
    vec![
        MppiCircularObstacle2D::new(0.78, 0.02, 0.24),
        MppiCircularObstacle2D::new(1.48, -0.03, 0.24),
        MppiCircularObstacle2D::new(2.18, 0.02, 0.24),
    ]
}

fn base_config(obstacles: Vec<MppiCircularObstacle2D>, seed: u64) -> MppiConfig {
    MppiConfig {
        horizon: 15,
        samples: 340,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.08,
        max_lambda: 25.0,
        target_effective_sample_ratio: 0.24,
        adaptive_lambda_iterations: 22,
        noise_sigma: 1.15,
        control_limit: 2.7,
        goal_weight: 0.35,
        terminal_weight: 0.7,
        obstacles,
        constraint_weight: 520.0,
        constraint_discount: 0.9,
        safety_margin: 0.12,
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

fn run_controller(
    label: &'static str,
    color: &'static str,
    config: MppiConfig,
    track: &MppiWaypointTrack2D,
    obstacles: &[MppiCircularObstacle2D],
    lookahead: Option<f64>,
) -> RoboticsResult<TrackRun> {
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut states = vec![state];
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_lateral = 0.0;
    let mut mean_ess = 0.0;

    for _ in 0..STEPS {
        let projection_before = track.project(state.x, state.y)?;
        let goal = if let Some(lookahead) = lookahead {
            track.point_at_progress(projection_before.progress + lookahead)?
        } else {
            track.goal()
        };
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, dt);
        states.push(state);
        let projection = track.project(state.x, state.y)?;
        mean_lateral += projection.lateral_error;
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        min_seen_clearance = min_seen_clearance.min(min_clearance(state, obstacles));
    }
    let projection = track.project(state.x, state.y)?;
    Ok(TrackRun {
        label,
        color,
        states,
        final_progress: projection.progress,
        final_lateral: projection.lateral_error,
        mean_lateral: mean_lateral / STEPS as f64,
        min_clearance: min_seen_clearance,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn build_scene() -> RoboticsResult<TrackScene> {
    let track = track()?;
    let obstacles = obstacles();
    let value_grid = track.terminal_value_grid(92, 72, -0.4, -1.4, 0.05, 7.0, 12.0)?;
    let single_goal = run_controller(
        "single goal",
        "#2563eb",
        base_config(obstacles.clone(), 223),
        &track,
        &obstacles,
        None,
    )?;
    let track_progress = run_controller(
        "track progress",
        "#059669",
        MppiConfig {
            terminal_value_grid: Some(value_grid.clone()),
            terminal_value_weight: 1.0,
            ..base_config(obstacles.clone(), 229)
        },
        &track,
        &obstacles,
        Some(0.95),
    )?;
    Ok(TrackScene {
        track,
        obstacles,
        value_grid,
        single_goal,
        track_progress,
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

fn value_color(value: f64, min_value: f64, max_value: f64) -> String {
    let span = (max_value - min_value).max(1e-9);
    let t = ((value - min_value) / span).clamp(0.0, 1.0);
    if t < 0.5 {
        interpolate_rgb((209.0, 250.0, 229.0), (253.0, 224.0, 71.0), t * 2.0)
    } else {
        interpolate_rgb((253.0, 224.0, 71.0), (239.0, 68.0, 68.0), (t - 0.5) * 2.0)
    }
}

fn value_range(grid: &MppiTerminalValueGrid2D) -> (f64, f64) {
    let mut min_value = f64::INFINITY;
    let mut max_value: f64 = 0.0;
    for x in (0..grid.width()).step_by(GRID_STEP) {
        for y in (0..grid.height()).step_by(GRID_STEP) {
            let wx = grid.origin_x + x as f64 * grid.resolution;
            let wy = grid.origin_y + y as f64 * grid.resolution;
            if !(X_MIN..=X_MAX).contains(&wx) || !(Y_MIN..=Y_MAX).contains(&wy) {
                continue;
            }
            let value = grid.values[x][y];
            min_value = min_value.min(value);
            max_value = max_value.max(value);
        }
    }
    (min_value, max_value)
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

fn render_heatmap(
    svg: &mut String,
    grid: &MppiTerminalValueGrid2D,
    min_value: f64,
    max_value: f64,
) {
    let cell_w = PLOT_WIDTH * grid.resolution * GRID_STEP as f64 / (X_MAX - X_MIN);
    let cell_h = PLOT_HEIGHT * grid.resolution * GRID_STEP as f64 / (Y_MAX - Y_MIN);
    for x in (0..grid.width()).step_by(GRID_STEP) {
        for y in (0..grid.height()).step_by(GRID_STEP) {
            let wx = grid.origin_x + x as f64 * grid.resolution;
            let wy = grid.origin_y + y as f64 * grid.resolution;
            if !(X_MIN..=X_MAX).contains(&wx) || !(Y_MIN..=Y_MAX).contains(&wy) {
                continue;
            }
            let fill = value_color(grid.values[x][y], min_value, max_value);
            writeln!(
                svg,
                r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="{fill}" opacity="0.38"/>"##,
                sx(wx) - cell_w / 2.0,
                sy(wy) - cell_h / 2.0,
                cell_w + 0.7,
                cell_h + 0.7
            )
            .unwrap();
        }
    }
}

fn render_run(svg: &mut String, run: &TrackRun, stroke_width: f64) {
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
        if index % 5 == 0 || index + 1 == run.states.len() {
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

fn render_svg(scene: &TrackScene) -> RoboticsResult<String> {
    let (min_value, max_value) = value_range(&scene.value_grid);
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="920" height="540" viewBox="0 0 920 540" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(svg, r##"<title id="title">Track-progress MPPI</title>"##).unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">A slalom course comparing single-goal MPPI with track-progress terminal values and lookahead.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="920" height="540" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="56" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">MPPI track progress</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="56" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Terminal values turn a short-horizon controller from goal chasing into course progress.</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="#ffffff" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<defs><clipPath id="plot-clip"><rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8"/></clipPath></defs>"##
    )
    .unwrap();
    writeln!(svg, r##"<g clip-path="url(#plot-clip)">"##).unwrap();
    render_heatmap(&mut svg, &scene.value_grid, min_value, max_value);
    writeln!(svg, "</g>").unwrap();

    let track_polyline = track_points(&scene.track);
    writeln!(
        svg,
        r##"<polyline points="{track_polyline}" fill="none" stroke="#111827" stroke-width="20" stroke-linecap="round" stroke-linejoin="round" opacity="0.10"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{track_polyline}" fill="none" stroke="#0f172a" stroke-width="4.2" stroke-linecap="round" stroke-linejoin="round" stroke-dasharray="10 8" opacity="0.68"/>"##
    )
    .unwrap();
    for &(x, y) in &scene.track.waypoints {
        writeln!(
            svg,
            r##"<circle cx="{:.1}" cy="{:.1}" r="6.0" fill="#0f172a" stroke="#ffffff" stroke-width="2"/>"##,
            sx(x),
            sy(y)
        )
        .unwrap();
    }

    for obstacle in &scene.obstacles {
        let ox = sx(obstacle.x);
        let oy = sy(obstacle.y);
        writeln!(
            svg,
            r##"<ellipse cx="{ox:.1}" cy="{oy:.1}" rx="{:.1}" ry="{:.1}" fill="none" stroke="#ef4444" stroke-width="2" stroke-dasharray="6 5" opacity="0.78"/>"##,
            sdx(obstacle.radius + 0.12),
            sdy(obstacle.radius + 0.12)
        )
        .unwrap();
        writeln!(
            svg,
            r##"<ellipse cx="{ox:.1}" cy="{oy:.1}" rx="{:.1}" ry="{:.1}" fill="#111827" opacity="0.88"/>"##,
            sdx(obstacle.radius),
            sdy(obstacle.radius)
        )
        .unwrap();
    }

    render_run(&mut svg, &scene.single_goal, 4.5);
    render_run(&mut svg, &scene.track_progress, 5.2);

    let start = scene.single_goal.states[0];
    let goal = scene.track.goal();
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="12" fill="#111827" stroke="#ffffff" stroke-width="3"/>"##,
        sx(start.x),
        sy(start.y)
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="12" fill="#059669" stroke="#ffffff" stroke-width="3"/>"##,
        sx(goal.0),
        sy(goal.1)
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
        r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">F</text>"##,
        sx(goal.0),
        sy(goal.1)
    )
    .unwrap();

    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="none" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();

    render_stats(&mut svg, &scene.single_goal, 710.0, 118.0);
    render_stats(&mut svg, &scene.track_progress, 710.0, 270.0);

    writeln!(
        svg,
        r##"<text x="56" y="474" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Background heatmap is the track terminal value: remaining progress plus lateral error. Dashed dark line is the desired slalom course.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn render_stats(svg: &mut String, run: &TrackRun, x: f64, y: f64) {
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
        format!("progress   {:.2}", run.final_progress),
        format!("lateral    {:.2}", run.final_lateral),
        format!("mean lat   {:.2}", run.mean_lateral),
        format!("clearance  {:.2}", run.min_clearance),
        format!("ESS ratio  {:.2}", run.mean_ess),
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

fn main() -> RoboticsResult<()> {
    let scene = build_scene()?;
    let svg = render_svg(&scene)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} progress single={:.2} track={:.2}",
        scene.single_goal.final_progress, scene.track_progress.final_progress
    );
    Ok(())
}
