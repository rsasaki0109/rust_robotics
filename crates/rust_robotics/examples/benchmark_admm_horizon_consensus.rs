//! Receding-horizon (trajectory) formation consensus via ADMM.
//!
//! A four-agent team must hold formation around a moving goal that turns a sharp
//! L-corner, but each agent only sees a *noisy* perceived goal. Every control
//! cycle the team solves a short-horizon consensus — agreeing on a shared center
//! **trajectory** anchored at the current center — applies the first step, shifts
//! the horizon, and re-solves (a receding-horizon MPC loop). The temporal
//! smoothness penalty in the consensus couples the center across time, so the
//! executed path both averages out the per-agent noise (spatially) and stays
//! smooth (temporally). The benchmark runs the loop with no smoothing and with
//! strong smoothing and writes the executed paths plus a metrics comparison to
//! CSV/SVG. The honest trade-off it surfaces: smoothing rejects noise and cuts
//! jerk sharply, at the cost of some tracking lag at the corner.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{solve_horizon_consensus, AdmmConfig, AgentTrajectory};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/admm-horizon-consensus.csv";
const SVG_OUTPUT: &str = "docs/assets/admm-horizon-consensus.svg";

const CYCLES: usize = 34;
const HORIZON: usize = 10;
const DX: f64 = 0.18;
const CORNER: usize = 18; // goal-step index of the L-corner
const NOISE_AMP: f64 = 0.25;

/// Square formation offsets (radius ~0.6).
fn offsets() -> Vec<[f64; 2]> {
    vec![[0.6, 0.0], [0.0, 0.6], [-0.6, 0.0], [0.0, -0.6]]
}

/// True moving goal: advance in +x, then turn 90° into +y at the corner.
fn goal(step: usize) -> [f64; 2] {
    if step <= CORNER {
        [step as f64 * DX, 0.0]
    } else {
        [CORNER as f64 * DX, (step - CORNER) as f64 * DX]
    }
}

/// Deterministic per-agent perception noise (no RNG — reproducible).
fn agent_noise(agent: usize, step: usize, amp: f64) -> [f64; 2] {
    let a = agent as f64;
    let s = step as f64;
    [
        amp * (2.1 * a + 0.7 * s).sin(),
        amp * (1.3 * a + 0.9 * s).cos(),
    ]
}

fn config() -> AdmmConfig {
    AdmmConfig {
        rho: 1.0,
        max_iters: 400,
        tol: 1e-7,
    }
}

struct RunResult {
    /// Executed team-center path, one point per cycle (plus the start).
    path: Vec<[f64; 2]>,
    /// RMS acceleration (second difference) of the executed center path.
    rms_accel: f64,
    /// Mean Euclidean tracking error of the center to the true goal.
    mean_tracking: f64,
    /// Total ADMM iterations across all cycles.
    total_iters: usize,
}

/// Run the receding-horizon MPC loop for a smoothing weight and noise amplitude.
#[allow(clippy::needless_range_loop)]
fn run_mpc(smooth_weight: f64, noise_amp: f64) -> RoboticsResult<RunResult> {
    let offs = offsets();
    let mut center = goal(0);
    let mut path = vec![center];
    let mut total_iters = 0usize;

    for c in 0..CYCLES {
        // Each agent's reference over the horizon: its noisy perceived goal plus
        // its formation offset, so consensus on the center tracks the true goal.
        let agents: Vec<AgentTrajectory> = offs
            .iter()
            .enumerate()
            .map(|(i, off)| {
                let reference = (0..HORIZON)
                    .map(|t| {
                        let g = goal(c + t);
                        let nz = agent_noise(i, c + t, noise_amp);
                        [g[0] + nz[0] + off[0], g[1] + nz[1] + off[1]]
                    })
                    .collect();
                AgentTrajectory::new(reference, *off)
            })
            .collect();

        let report = solve_horizon_consensus(config(), &agents, smooth_weight, Some(center))?;
        total_iters += report.iterations;
        // Apply the first planned step.
        center = report.center[1.min(report.center.len() - 1)];
        path.push(center);
    }

    // Metrics over the executed path.
    let mut accel_sq = 0.0;
    let mut accel_n = 0usize;
    for t in 1..path.len() - 1 {
        for k in 0..2 {
            let a = path[t + 1][k] - 2.0 * path[t][k] + path[t - 1][k];
            accel_sq += a * a;
        }
        accel_n += 1;
    }
    let rms_accel = if accel_n > 0 {
        (accel_sq / accel_n as f64).sqrt()
    } else {
        0.0
    };
    let mut track = 0.0;
    for (c, p) in path.iter().enumerate() {
        let g = goal(c);
        track += ((p[0] - g[0]).powi(2) + (p[1] - g[1]).powi(2)).sqrt();
    }
    let mean_tracking = track / path.len() as f64;

    Ok(RunResult {
        path,
        rms_accel,
        mean_tracking,
        total_iters,
    })
}

fn render_csv(stiff: &RunResult, smooth: &RunResult) -> String {
    let mut csv = String::from("cycle,goal_x,goal_y,stiff_x,stiff_y,smooth_x,smooth_y\n");
    for c in 0..stiff.path.len() {
        let g = goal(c);
        let _ = writeln!(
            csv,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            c, g[0], g[1], stiff.path[c][0], stiff.path[c][1], smooth.path[c][0], smooth.path[c][1]
        );
    }
    csv
}

fn render_svg(stiff: &RunResult, smooth: &RunResult) -> String {
    let width = 920.0;
    let height = 460.0;
    let left = 40.0;
    let panel = 380.0;
    let top = 80.0;
    let bottom = 420.0;
    let world = [-0.8_f64, 4.6];
    let map = |w: [f64; 2]| {
        let sx = left + (w[0] - world[0]) / (world[1] - world[0]) * panel;
        let sy = bottom - (w[1] - world[0]) / (world[1] - world[0]) * (bottom - top);
        (sx, sy)
    };

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
        "<text x=\"{left}\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1d1d1f\">Receding-horizon formation consensus over a moving goal</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"52\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">left: executed team center past an L-corner under noisy per-agent goals; right: jerk / lag trade-off</text>"
    );

    // True goal path (grey).
    let mut gpts = String::new();
    for c in 0..stiff.path.len() {
        let (x, y) = map(goal(c));
        let _ = write!(gpts, "{x:.1},{y:.1} ");
    }
    let _ = writeln!(
        svg,
        "<polyline points=\"{}\" fill=\"none\" stroke=\"#c7c7cc\" stroke-width=\"3\"/>",
        gpts.trim()
    );

    // Executed paths.
    let draw_path = |svg: &mut String, run: &RunResult, color: &str| {
        let mut pts = String::new();
        for p in &run.path {
            let (x, y) = map(*p);
            let _ = write!(pts, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"2\"/>",
            pts.trim(),
            color
        );
    };
    draw_path(&mut svg, stiff, "#ff453a");
    draw_path(&mut svg, smooth, "#0a84ff");

    // Formation snapshots (smooth run) every 8th cycle.
    let offs = offsets();
    for c in (0..smooth.path.len()).step_by(8) {
        let ctr = smooth.path[c];
        let mut poly = String::new();
        for off in &offs {
            let (x, y) = map([ctr[0] + off[0], ctr[1] + off[1]]);
            let _ = write!(poly, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polygon points=\"{}\" fill=\"#0a84ff22\" stroke=\"#0a84ff\" stroke-width=\"0.8\"/>",
            poly.trim()
        );
    }

    // Legend.
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#c7c7cc\">true goal</text>",
        left + 8.0,
        top + 12.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#ff453a\">no smoothing</text>",
        left + 8.0,
        top + 28.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#0a84ff\">smoothed + formation</text>",
        left + 8.0,
        top + 44.0
    );

    // Right: metric comparison bars.
    let ileft = 480.0;
    let iright = width - 40.0;
    let metrics = [
        ("RMS accel (jerk)", stiff.rms_accel, smooth.rms_accel),
        (
            "mean tracking err",
            stiff.mean_tracking,
            smooth.mean_tracking,
        ),
    ];
    let bar_top = 110.0;
    let group_h = 130.0;
    for (gi, (label, sv, mv)) in metrics.iter().enumerate() {
        let y0 = bar_top + gi as f64 * group_h;
        let _ = writeln!(
            svg,
            "<text x=\"{ileft:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\">{label}</text>",
            y0 - 8.0
        );
        let scale = (iright - ileft - 120.0) / sv.max(*mv).max(1e-9);
        for (bi, (name, val, color)) in [
            ("no smoothing", *sv, "#ff453a"),
            ("smoothed", *mv, "#0a84ff"),
        ]
        .iter()
        .enumerate()
        {
            let by = y0 + 6.0 + bi as f64 * 34.0;
            let bw = val * scale;
            let _ = writeln!(
                svg,
                "<rect x=\"{ileft:.1}\" y=\"{by:.1}\" width=\"{bw:.1}\" height=\"20\" fill=\"{color}\" opacity=\"0.85\"/>"
            );
            let _ = writeln!(
                svg,
                "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\">{name}: {val:.3}</text>",
                ileft + bw + 6.0,
                by + 15.0
            );
        }
    }

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    // Main scenario: noisy per-agent perception. The SVG renders this regime.
    let stiff = run_mpc(0.0, NOISE_AMP)?;
    let smooth = run_mpc(40.0, NOISE_AMP)?;

    println!("receding-horizon formation consensus (ADMM over short trajectories)");
    println!("  noisy perception (amp={NOISE_AMP}):");
    for (name, r) in [("no-smoothing", &stiff), ("smoothed   ", &smooth)] {
        println!(
            "    {name}  rms_accel={:.4}  mean_tracking={:.4}  total_iters={}",
            r.rms_accel, r.mean_tracking, r.total_iters
        );
    }
    let jerk_drop = 100.0 * (1.0 - smooth.rms_accel / stiff.rms_accel.max(1e-9));
    println!(
        "    -> smoothing cuts jerk by {jerk_drop:.0}% AND improves tracking ({:.3} -> {:.3}): the",
        stiff.mean_tracking, smooth.mean_tracking
    );
    println!("       per-agent noise, not the corner, dominates, so consensus wins on both axes.");

    // Clean-perception reference: with near-perfect sensing the classic
    // smoothing trade-off reappears — smoothing cuts the L-corner and lags.
    let clean_stiff = run_mpc(0.0, 0.0)?;
    let clean_smooth = run_mpc(40.0, 0.0)?;
    println!("  clean perception (amp=0):");
    for (name, r) in [
        ("no-smoothing", &clean_stiff),
        ("smoothed   ", &clean_smooth),
    ] {
        println!(
            "    {name}  rms_accel={:.4}  mean_tracking={:.4}",
            r.rms_accel, r.mean_tracking
        );
    }
    println!(
        "    -> here smoothing still cuts jerk but the corner-cutting lag raises tracking ({:.3} -> {:.3}).",
        clean_stiff.mean_tracking, clean_smooth.mean_tracking
    );

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&stiff, &smooth)).ok();
    fs::write(SVG_OUTPUT, render_svg(&stiff, &smooth)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoothing_cuts_executed_jerk() {
        let stiff = run_mpc(0.0, NOISE_AMP).unwrap();
        let smooth = run_mpc(40.0, NOISE_AMP).unwrap();
        assert!(
            smooth.rms_accel < 0.6 * stiff.rms_accel,
            "smoothing should cut jerk: {} vs {}",
            smooth.rms_accel,
            stiff.rms_accel
        );
    }

    #[test]
    fn smoothing_rejects_noise_on_both_axes() {
        // In the noisy regime, consensus smoothing improves tracking too.
        let stiff = run_mpc(0.0, NOISE_AMP).unwrap();
        let smooth = run_mpc(40.0, NOISE_AMP).unwrap();
        assert!(
            smooth.mean_tracking < stiff.mean_tracking,
            "smoothing should reject noise: {} vs {}",
            smooth.mean_tracking,
            stiff.mean_tracking
        );
    }

    #[test]
    fn clean_perception_shows_corner_lag() {
        // With no noise the classic trade-off returns: smoothing cuts the corner
        // and so tracks slightly worse than the stiff run.
        let stiff = run_mpc(0.0, 0.0).unwrap();
        let smooth = run_mpc(40.0, 0.0).unwrap();
        assert!(smooth.rms_accel < stiff.rms_accel);
        assert!(
            smooth.mean_tracking > stiff.mean_tracking,
            "clean-perception corner lag expected: {} vs {}",
            smooth.mean_tracking,
            stiff.mean_tracking
        );
    }

    #[test]
    fn is_deterministic() {
        let a = run_mpc(40.0, NOISE_AMP).unwrap();
        let b = run_mpc(40.0, NOISE_AMP).unwrap();
        assert_eq!(a.path, b.path);
        assert_eq!(a.total_iters, b.total_iters);
    }
}
