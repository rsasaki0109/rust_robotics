//! Render replay-learned MPPI terminal values as a static SVG.
//!
//! The example reruns the replay value-learning setup and writes a visual docs
//! asset showing the learned value heatmap, obstacle, and final executed path.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D, MppiTerminalValueGrid2D,
    MppiTerminalValueReplayBuffer2D, MppiTerminalValueUpdateConfig2D, MppiTerminalValueUpdater2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/mppi-replay-value-grid.svg";
const EPISODES: usize = 6;
const EPISODE_STEPS: usize = 16;
const LEFT: f64 = 58.0;
const TOP: f64 = 92.0;
const PLOT_WIDTH: f64 = 580.0;
const PLOT_HEIGHT: f64 = 340.0;
const X_MIN: f64 = -0.2;
const X_MAX: f64 = 2.65;
const Y_MIN: f64 = -1.0;
const Y_MAX: f64 = 1.0;
const GRID_STEP: usize = 2;

#[derive(Debug, Clone)]
struct LearnedValueScene {
    grid: MppiTerminalValueGrid2D,
    trajectory: Vec<MppiState2D>,
    obstacle: MppiCircularObstacle2D,
    goal: (f64, f64),
    final_distance: f64,
    min_clearance: f64,
    replay_size: usize,
    replay_updates: usize,
    mean_replay_delta: f64,
    max_replay_delta: f64,
}

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn clearance(state: MppiState2D, obstacle: MppiCircularObstacle2D) -> f64 {
    let dx = state.x - obstacle.x;
    let dy = state.y - obstacle.y;
    (dx * dx + dy * dy).sqrt() - obstacle.radius
}

fn scaled_goal_grid(goal: (f64, f64)) -> RoboticsResult<MppiTerminalValueGrid2D> {
    let mut grid = MppiTerminalValueGrid2D::from_goal_distance(90, 70, -0.5, -1.75, 0.05, goal)?;
    for column in &mut grid.values {
        for value in column {
            *value *= 8.0;
        }
    }
    Ok(grid)
}

fn controller_config(
    grid: MppiTerminalValueGrid2D,
    obstacle: MppiCircularObstacle2D,
    episode: usize,
) -> MppiConfig {
    MppiConfig {
        horizon: 9,
        samples: 260,
        dt: 0.1,
        control_limit: 2.6,
        noise_sigma: 1.1,
        goal_weight: 0.45,
        terminal_weight: 0.8,
        obstacles: vec![obstacle],
        constraint_weight: 550.0,
        constraint_discount: 0.9,
        safety_margin: 0.15,
        terminal_value_weight: 1.0,
        terminal_value_grid: Some(grid),
        seed: 131 + episode as u64,
        ..MppiConfig::default()
    }
}

fn learn_scene() -> RoboticsResult<LearnedValueScene> {
    let goal = (2.4, 0.0);
    let obstacle = MppiCircularObstacle2D::new(1.0, 0.0, 0.35);
    let updater = MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D {
        learning_rate: 0.12,
        discount: 0.97,
    })?;
    let mut buffer = MppiTerminalValueReplayBuffer2D::new(32)?;
    let mut grid = scaled_goal_grid(goal)?;
    let mut trajectory = Vec::new();
    let mut final_distance = 0.0;
    let mut min_clearance = f64::INFINITY;
    let mut replay_updates = 0;
    let mut mean_delta_sum = 0.0;
    let mut max_replay_delta: f64 = 0.0;

    for episode in 0..EPISODES {
        let mut controller = MppiController2D::new(controller_config(grid, obstacle, episode))?;
        let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
        let mut episode_trajectory = vec![state];
        let mut episode_updates = 0;
        let mut episode_mean_delta_sum = 0.0;
        let mut episode_max_delta: f64 = 0.0;
        let mut episode_min_clearance = f64::INFINITY;

        for _ in 0..EPISODE_STEPS {
            let plan = controller.plan(state, goal)?;
            buffer.push(plan.best_rollout.clone())?;
            let report = buffer.update_grid(
                controller
                    .terminal_value_grid_mut()
                    .expect("terminal value grid is configured"),
                &updater,
            )?;
            episode_updates += report.updates;
            episode_mean_delta_sum += report.mean_abs_delta;
            episode_max_delta = episode_max_delta.max(report.max_abs_delta);

            state = state.step(plan.first_control, 0.1);
            episode_min_clearance = episode_min_clearance.min(clearance(state, obstacle));
            episode_trajectory.push(state);
        }

        grid = controller
            .terminal_value_grid()
            .expect("terminal value grid is configured")
            .clone();
        if episode + 1 == EPISODES {
            trajectory = episode_trajectory;
            final_distance = distance_to_goal(state, goal);
            min_clearance = episode_min_clearance;
            replay_updates = episode_updates;
            mean_delta_sum = episode_mean_delta_sum;
            max_replay_delta = episode_max_delta;
        }
    }

    Ok(LearnedValueScene {
        grid,
        trajectory,
        obstacle,
        goal,
        final_distance,
        min_clearance,
        replay_size: buffer.len(),
        replay_updates,
        mean_replay_delta: mean_delta_sum / EPISODE_STEPS as f64,
        max_replay_delta,
    })
}

fn screen_x(x: f64) -> f64 {
    LEFT + (x - X_MIN) / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn screen_y(y: f64) -> f64 {
    TOP + (Y_MAX - y) / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn screen_dx(distance: f64) -> f64 {
    distance / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn screen_dy(distance: f64) -> f64 {
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
        interpolate_rgb((16.0, 185.0, 129.0), (250.0, 204.0, 21.0), t * 2.0)
    } else {
        interpolate_rgb((250.0, 204.0, 21.0), (220.0, 38.0, 38.0), (t - 0.5) * 2.0)
    }
}

fn region_value_range(grid: &MppiTerminalValueGrid2D) -> (f64, f64) {
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
        .map(|state| format!("{:.1},{:.1}", screen_x(state.x), screen_y(state.y)))
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
                r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="{fill}" opacity="0.88"/>"##,
                screen_x(wx) - cell_w / 2.0,
                screen_y(wy) - cell_h / 2.0,
                cell_w + 0.6,
                cell_h + 0.6
            )
            .unwrap();
        }
    }
}

fn render_svg(scene: &LearnedValueScene) -> RoboticsResult<String> {
    let (min_value, max_value) = region_value_range(&scene.grid);
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="860" height="520" viewBox="0 0 860 520" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Replay-learned MPPI terminal value grid</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">A terminal value heatmap learned from replayed MPPI rollouts, with an obstacle, goal, and final executed trajectory.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="860" height="520" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">MPPI replay value learning</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Replay-backed terminal value grid with the final short-horizon rollout overlaid.</text>"##
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
    render_heatmap(&mut svg, &scene.grid, min_value, max_value);
    writeln!(svg, "</g>").unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="none" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();

    let obstacle_x = screen_x(scene.obstacle.x);
    let obstacle_y = screen_y(scene.obstacle.y);
    let obstacle_rx = screen_dx(scene.obstacle.radius);
    let obstacle_ry = screen_dy(scene.obstacle.radius);
    let safety_rx = screen_dx(scene.obstacle.radius + 0.15);
    let safety_ry = screen_dy(scene.obstacle.radius + 0.15);
    writeln!(
        svg,
        r##"<ellipse cx="{obstacle_x:.1}" cy="{obstacle_y:.1}" rx="{safety_rx:.1}" ry="{safety_ry:.1}" fill="none" stroke="#ef4444" stroke-width="2.4" stroke-dasharray="6 5" opacity="0.82"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<ellipse cx="{obstacle_x:.1}" cy="{obstacle_y:.1}" rx="{obstacle_rx:.1}" ry="{obstacle_ry:.1}" fill="#111827" opacity="0.9"/>"##
    )
    .unwrap();

    let points = trajectory_points(&scene.trajectory);
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="9" stroke-linecap="round" stroke-linejoin="round" opacity="0.88"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="5.2" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();
    for (index, state) in scene.trajectory.iter().enumerate() {
        if index % 4 == 0 || index + 1 == scene.trajectory.len() {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="4.2" fill="#2563eb" stroke="#ffffff" stroke-width="1.5"/>"##,
                screen_x(state.x),
                screen_y(state.y)
            )
            .unwrap();
        }
    }

    let start = scene.trajectory[0];
    let start_x = screen_x(start.x);
    let start_y = screen_y(start.y);
    let goal_x = screen_x(scene.goal.0);
    let goal_y = screen_y(scene.goal.1);
    writeln!(
        svg,
        r##"<circle cx="{start_x:.1}" cy="{start_y:.1}" r="12" fill="#111827" stroke="#ffffff" stroke-width="3"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{goal_x:.1}" cy="{goal_y:.1}" r="12" fill="#059669" stroke="#ffffff" stroke-width="3"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{start_x:.1}" y="{start_y:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">S</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{goal_x:.1}" y="{goal_y:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">G</text>"##
    )
    .unwrap();

    for x in [0.0, 1.0, 2.0] {
        writeln!(
            svg,
            r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#ffffff" stroke-width="1" opacity="0.35"/>"##,
            screen_x(x),
            TOP,
            screen_x(x),
            TOP + PLOT_HEIGHT
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11">{x:.0}m</text>"##,
            screen_x(x),
            TOP + PLOT_HEIGHT + 20.0
        )
        .unwrap();
    }

    let panel_x = 680.0;
    writeln!(
        svg,
        r##"<text x="{panel_x:.1}" y="112" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">Replay stats</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("buffer size   {}", scene.replay_size),
        format!("grid backups  {}", scene.replay_updates),
        format!("mean delta    {:.2}", scene.mean_replay_delta),
        format!("max delta     {:.2}", scene.max_replay_delta),
        format!("final dist    {:.2} m", scene.final_distance),
        format!("min clearance {:.2} m", scene.min_clearance),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{panel_x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            138.0 + index as f64 * 24.0
        )
        .unwrap();
    }

    writeln!(
        svg,
        r##"<text x="{panel_x:.1}" y="318" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">Value scale</text>"##
    )
    .unwrap();
    for i in 0..8 {
        let t = i as f64 / 7.0;
        let fill = value_color(
            min_value + t * (max_value - min_value),
            min_value,
            max_value,
        );
        writeln!(
            svg,
            r##"<rect x="{:.1}" y="{:.1}" width="24" height="18" rx="3" fill="{fill}"/>"##,
            panel_x + i as f64 * 25.0,
            338.0
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<text x="{panel_x:.1}" y="378" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">low value near the goal, high cost-to-go near risky or distant states</text>"##
    )
    .unwrap();

    writeln!(
        svg,
        r##"<text x="58" y="474" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Heatmap cells are the learned terminal values sampled after {EPISODES} replay-learning episodes. The dashed outline is the safety margin.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let scene = learn_scene()?;
    let svg = render_svg(&scene)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} final_distance={:.2} replay_size={}",
        scene.final_distance, scene.replay_size
    );
    Ok(())
}
