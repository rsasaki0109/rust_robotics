//! Decentralized formation consensus over a communication graph.
//!
//! The same eight agents reach the same formation consensus three ways, talking
//! only to graph neighbors via decentralized ADMM: over a line graph, a ring, and
//! a complete graph. All converge to the same center, but the *rate* is set by
//! the graph connectivity — the better-connected graph converges in far fewer
//! iterations. The benchmark renders the ring formation and a disagreement-vs-
//! iteration convergence comparison to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{solve_graph_consensus, AdmmConfig, AgentSpec, GraphConsensusReport};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/admm-graph-consensus.csv";
const SVG_OUTPUT: &str = "docs/assets/admm-graph-consensus.svg";

const N: usize = 8;

fn agents() -> Vec<AgentSpec> {
    (0..N)
        .map(|i| {
            // Scattered preferred positions on a wide circle; small formation ring.
            let pa = std::f64::consts::TAU * (i as f64 * 1.7) / N as f64;
            let preferred = [3.0 * pa.cos(), 3.0 * (pa * 0.8).sin()];
            let po = std::f64::consts::TAU * i as f64 / N as f64;
            let offset = [1.0 * po.cos(), 1.0 * po.sin()];
            AgentSpec::new(preferred, offset)
        })
        .collect()
}

fn line_edges() -> Vec<(usize, usize)> {
    (0..N - 1).map(|i| (i, i + 1)).collect()
}
fn ring_edges() -> Vec<(usize, usize)> {
    (0..N).map(|i| (i, (i + 1) % N)).collect()
}
fn complete_edges() -> Vec<(usize, usize)> {
    let mut e = Vec::new();
    for i in 0..N {
        for j in (i + 1)..N {
            e.push((i, j));
        }
    }
    e
}

struct Topo {
    name: &'static str,
    color: &'static str,
    report: GraphConsensusReport,
}

fn config() -> AdmmConfig {
    AdmmConfig {
        rho: 1.0,
        max_iters: 5000,
        tol: 1e-6,
    }
}

fn run() -> RoboticsResult<Vec<Topo>> {
    let a = agents();
    Ok(vec![
        Topo {
            name: "line",
            color: "#ff453a",
            report: solve_graph_consensus(config(), &a, &line_edges())?,
        },
        Topo {
            name: "ring",
            color: "#ff9f0a",
            report: solve_graph_consensus(config(), &a, &ring_edges())?,
        },
        Topo {
            name: "complete",
            color: "#0a84ff",
            report: solve_graph_consensus(config(), &a, &complete_edges())?,
        },
    ])
}

fn render_csv(topos: &[Topo]) -> String {
    let mut csv = String::from("topology,iterations,center_x,center_y,final_disagreement\n");
    for t in topos {
        let r = &t.report;
        let _ = writeln!(
            csv,
            "{},{},{:.4},{:.4},{:.3e}",
            t.name,
            r.iterations,
            r.center[0],
            r.center[1],
            r.disagreement.last().copied().unwrap_or(0.0)
        );
    }
    csv
}

fn render_svg(topos: &[Topo]) -> String {
    let specs = agents();
    let width = 920.0;
    let height = 440.0;

    // Left: ring formation panel.
    let left = 40.0;
    let panel = 360.0;
    let top = 70.0;
    let bottom = 410.0;
    let world = [-3.6_f64, 3.6];
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
        "<text x=\"{left}\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1d1d1f\">Decentralized formation consensus over a communication graph</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"52\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">left: ring graph and final formation; right: disagreement vs iteration — connectivity sets the rate</text>"
    );

    // Ring edges between final formation positions.
    let ring = &topos.iter().find(|t| t.name == "ring").unwrap().report;
    for (i, j) in ring_edges() {
        let (x1, y1) = map(ring.positions[i]);
        let (x2, y2) = map(ring.positions[j]);
        let _ = writeln!(
            svg,
            "<line x1=\"{x1:.1}\" y1=\"{y1:.1}\" x2=\"{x2:.1}\" y2=\"{y2:.1}\" stroke=\"#d0d0d5\" stroke-width=\"1\"/>"
        );
    }
    for (p, spec) in ring.positions.iter().zip(&specs) {
        let (px, py) = map(spec.preferred);
        let (fx, fy) = map(*p);
        let _ = writeln!(
            svg,
            "<circle cx=\"{px:.1}\" cy=\"{py:.1}\" r=\"3\" fill=\"#c7c7cc\"/>"
        );
        let _ = writeln!(
            svg,
            "<circle cx=\"{fx:.1}\" cy=\"{fy:.1}\" r=\"5\" fill=\"#ff9f0a\"/>"
        );
    }
    let (zx, zy) = map(ring.center);
    let _ = writeln!(
        svg,
        "<text x=\"{zx:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#ff453a\" text-anchor=\"middle\">x</text>",
        zy + 5.0
    );

    // Right: disagreement convergence (log10).
    let ileft = 470.0;
    let iright = width - 30.0;
    let itop = 80.0;
    let ibottom = 380.0;
    let _ = writeln!(
        svg,
        "<rect x=\"{ileft}\" y=\"{itop}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#ffffff\" stroke=\"#e5e5ea\"/>",
        iright - ileft,
        ibottom - itop
    );
    let max_iter = topos
        .iter()
        .map(|t| t.report.disagreement.len())
        .max()
        .unwrap_or(1)
        .max(2);
    let mut lmax = f64::NEG_INFINITY;
    let mut lmin = f64::INFINITY;
    for t in topos {
        for &v in &t.report.disagreement {
            if v > 0.0 {
                lmax = lmax.max(v.log10());
                lmin = lmin.min(v.log10());
            }
        }
    }
    let span = (lmax - lmin).max(1e-9);
    for (k, t) in topos.iter().enumerate() {
        let mut pts = String::new();
        for (i, &v) in t.report.disagreement.iter().enumerate() {
            let x = ileft + (i as f64 / (max_iter - 1) as f64) * (iright - ileft);
            let ly = if v > 0.0 { v.log10() } else { lmin };
            let y = ibottom - (ly - lmin) / span * (ibottom - itop);
            let _ = write!(pts, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"1.8\"/>",
            pts.trim(),
            t.color
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"{}\">{} ({} iters)</text>",
            ileft + 12.0,
            itop + 18.0 + k as f64 * 16.0,
            t.color,
            t.name,
            t.report.iterations
        );
    }
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">iteration -> (disagreement, log10)</text>",
        (ileft + iright) / 2.0,
        ibottom + 22.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let topos = run()?;
    println!("decentralized graph consensus (ADMM over a communication graph)");
    for t in &topos {
        let r = &t.report;
        println!(
            "  {:<9} iters={:<5} center=({:.3},{:.3}) final_disagreement={:.2e}",
            t.name,
            r.iterations,
            r.center[0],
            r.center[1],
            r.disagreement.last().copied().unwrap_or(0.0)
        );
    }

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&topos)).ok();
    fs::write(SVG_OUTPUT, render_svg(&topos)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_topologies_reach_the_same_center() {
        let topos = run().unwrap();
        let c0 = topos[0].report.center;
        for t in &topos {
            assert!((t.report.center[0] - c0[0]).abs() < 1e-2);
            assert!((t.report.center[1] - c0[1]).abs() < 1e-2);
            assert!(*t.report.disagreement.last().unwrap() < 1e-5);
        }
    }

    #[test]
    fn full_connectivity_converges_fastest() {
        // The complete graph (every agent talks to every other) converges in far
        // fewer iterations than the sparse line/ring graphs. (For a small graph
        // the line and ring are close and can swap, so only the strong ordering
        // against the complete graph is asserted.)
        let topos = run().unwrap();
        let iters = |name| {
            topos
                .iter()
                .find(|t| t.name == name)
                .unwrap()
                .report
                .iterations
        };
        assert!(iters("complete") < iters("ring"));
        assert!(iters("complete") < iters("line"));
    }

    #[test]
    fn is_deterministic() {
        let a = run().unwrap();
        let b = run().unwrap();
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.report.positions, y.report.positions);
        }
    }
}
