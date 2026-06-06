//! Distributed formation consensus via ADMM.
//!
//! Six agents agree on a hexagon formation around a shared center while each is
//! pulled toward its own scattered preferred position. Two agents are confined to
//! a corridor (a box constraint), so the consensus center shifts to accommodate
//! them — the case where ADMM earns its keep over the closed-form average. The
//! benchmark renders the preferred positions, the final formation and center, the
//! corridor, and a primal/dual residual convergence inset to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{solve_formation_consensus, AdmmConfig, AdmmReport, AgentSpec};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/admm-formation.csv";
const SVG_OUTPUT: &str = "docs/assets/admm-formation.svg";

const N: usize = 6;
const RADIUS: f64 = 1.5;
const CORRIDOR_X: f64 = 0.3; // constrained agents cannot move right of this

fn agents() -> Vec<AgentSpec> {
    // Scattered preferred positions (fixed, deterministic).
    let preferred = [
        [2.6, 0.4],
        [1.0, 2.4],
        [-1.8, 1.8],
        [-2.6, -0.6],
        [-0.8, -2.2],
        [1.8, -2.0],
    ];
    let mut out = Vec::with_capacity(N);
    for (i, p) in preferred.iter().enumerate() {
        let ang = std::f64::consts::TAU * i as f64 / N as f64;
        let offset = [RADIUS * ang.cos(), RADIUS * ang.sin()];
        let mut spec = AgentSpec::new(*p, offset);
        // Constrain the two right-side agents to a left corridor; their formation
        // slots would otherwise sit well to the right, so the whole consensus
        // center is pulled left to keep them inside it.
        if i == 0 || i == 5 {
            spec = spec.with_box(
                [f64::NEG_INFINITY, f64::NEG_INFINITY],
                [CORRIDOR_X, f64::INFINITY],
            );
        }
        out.push(spec);
    }
    out
}

fn run() -> RoboticsResult<AdmmReport> {
    let config = AdmmConfig {
        rho: 1.0,
        max_iters: 300,
        tol: 1e-7,
    };
    solve_formation_consensus(config, &agents())
}

fn render_csv(report: &AdmmReport) -> String {
    let mut csv = String::from("iteration,primal_residual,dual_residual\n");
    for (i, (p, d)) in report
        .primal_residuals
        .iter()
        .zip(&report.dual_residuals)
        .enumerate()
    {
        let _ = writeln!(csv, "{i},{p:.6e},{d:.6e}");
    }
    let _ = writeln!(
        csv,
        "# center,{:.4},{:.4}",
        report.center[0], report.center[1]
    );
    csv
}

fn render_svg(report: &AdmmReport) -> String {
    let specs = agents();
    let width = 900.0;
    let height = 460.0;
    // Left: formation panel. Right: residual inset.
    let left = 40.0;
    let panel = 420.0;
    let top = 70.0;
    let bottom = 430.0;

    let world = [-4.0_f64, 4.0];
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
        "<text x=\"{left}\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1d1d1f\">Distributed formation consensus via ADMM</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"52\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">gray = preferred, blue = final formation, x = consensus center; two agents are confined to the shaded corridor</text>"
    );

    // Corridor (x <= CORRIDOR_X).
    let (cx0, _) = map([world[0], 0.0]);
    let (cx1, _) = map([CORRIDOR_X, 0.0]);
    let _ = writeln!(
        svg,
        "<rect x=\"{cx0:.1}\" y=\"{top:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#34c75911\" stroke=\"#34c759\" stroke-dasharray=\"4 3\"/>",
        cx1 - cx0,
        bottom - top
    );

    // Formation polygon (final positions in agent order).
    let mut poly = String::new();
    for p in &report.positions {
        let (sx, sy) = map(*p);
        let _ = write!(poly, "{sx:.1},{sy:.1} ");
    }
    let _ = writeln!(
        svg,
        "<polygon points=\"{}\" fill=\"#0a84ff14\" stroke=\"#0a84ff\" stroke-width=\"1.5\"/>",
        poly.trim()
    );

    // Preferred (gray) and final (blue) markers, with pull lines.
    for (p, spec) in report.positions.iter().zip(&specs) {
        let (px, py) = map(spec.preferred);
        let (fx, fy) = map(*p);
        let _ = writeln!(
            svg,
            "<line x1=\"{px:.1}\" y1=\"{py:.1}\" x2=\"{fx:.1}\" y2=\"{fy:.1}\" stroke=\"#c7c7cc\" stroke-width=\"1\"/>"
        );
        let _ = writeln!(
            svg,
            "<circle cx=\"{px:.1}\" cy=\"{py:.1}\" r=\"4\" fill=\"#8e8e93\"/>"
        );
        let _ = writeln!(
            svg,
            "<circle cx=\"{fx:.1}\" cy=\"{fy:.1}\" r=\"5\" fill=\"#0a84ff\"/>"
        );
    }
    let (zx, zy) = map(report.center);
    let _ = writeln!(
        svg,
        "<text x=\"{zx:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#ff453a\" text-anchor=\"middle\">x</text>",
        zy + 5.0
    );

    // Residual inset (log10 scale).
    let ileft = 520.0;
    let iright = width - 30.0;
    let itop = 90.0;
    let ibottom = 360.0;
    let _ = writeln!(
        svg,
        "<rect x=\"{ileft}\" y=\"{itop}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#ffffff\" stroke=\"#e5e5ea\"/>",
        iright - ileft,
        ibottom - itop
    );
    let _ = writeln!(
        svg,
        "<text x=\"{ileft}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">residual convergence (log10)</text>",
        itop - 6.0
    );
    let all: Vec<f64> = report
        .primal_residuals
        .iter()
        .chain(&report.dual_residuals)
        .copied()
        .filter(|v| *v > 0.0)
        .collect();
    let lmax = all
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        .log10();
    let lmin = all.iter().cloned().fold(f64::INFINITY, f64::min).log10();
    let span = (lmax - lmin).max(1e-9);
    let nit = report.primal_residuals.len().max(1);
    let plot = |svg: &mut String, data: &[f64], color: &str| {
        let mut pts = String::new();
        for (i, &v) in data.iter().enumerate() {
            let x = ileft + (i as f64 / (nit - 1).max(1) as f64) * (iright - ileft);
            let ly = if v > 0.0 { v.log10() } else { lmin };
            let y = ibottom - (ly - lmin) / span * (ibottom - itop);
            let _ = write!(pts, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"1.8\"/>",
            pts.trim()
        );
    };
    plot(&mut svg, &report.primal_residuals, "#0a84ff");
    plot(&mut svg, &report.dual_residuals, "#ff9f0a");
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#0a84ff\">primal</text>",
        ileft + 10.0,
        itop + 18.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#ff9f0a\">dual</text>",
        ileft + 10.0,
        itop + 34.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">iteration ({} to converge)</text>",
        (ileft + iright) / 2.0,
        ibottom + 20.0,
        report.iterations
    );

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let report = run()?;
    println!("distributed formation consensus via ADMM");
    println!(
        "  converged in {} iterations; center = ({:.3}, {:.3})",
        report.iterations, report.center[0], report.center[1]
    );
    println!(
        "  final primal residual {:.2e}, dual residual {:.2e}",
        report.primal_residuals.last().unwrap(),
        report.dual_residuals.last().unwrap()
    );

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&report)).ok();
    fs::write(SVG_OUTPUT, render_svg(&report)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converges() {
        let report = run().unwrap();
        assert!(*report.primal_residuals.last().unwrap() < 1e-5);
        assert!(*report.dual_residuals.last().unwrap() < 1e-5);
    }

    #[test]
    fn constrained_agents_stay_in_corridor() {
        let report = run().unwrap();
        for (i, p) in report.positions.iter().enumerate() {
            if i == 0 || i == 5 {
                assert!(
                    p[0] <= CORRIDOR_X + 1e-9,
                    "agent {i} at {p:?} left corridor"
                );
            }
        }
    }

    #[test]
    fn is_deterministic() {
        let a = run().unwrap();
        let b = run().unwrap();
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.center, b.center);
    }
}
