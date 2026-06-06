//! Two-contact (two-pusher) quasi-static pushing.
//!
//! A scripted demonstration (no controller) of what a *second* simultaneous
//! contact buys, solved contact-implicitly with the ellipsoidal limit surface:
//!
//! - `single` — one off-center contact pushing forward: the slider both
//!   translates and rotates, so its path curves.
//! - `two-point` — two contacts on the same face at +/-h pushing forward: the
//!   second contact cancels the off-center torque, so the slider tracks straight.
//! - `couple` — a back-face contact high and a front-face contact low push in
//!   opposite directions, forming a couple: the slider spins almost in place.
//!
//! Each scenario is rendered as a strip of slider outlines sampled along the
//! motion, with the CoM path, to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{PusherCommand, PusherSliderParams, SliderState};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/pusher-slider-two-contact.csv";
const SVG_OUTPUT: &str = "docs/assets/pusher-slider-two-contact.svg";

const DT: f64 = 0.1;
const VN: f64 = 0.05;

struct Scenario {
    name: &'static str,
    steps: usize,
    trajectory: Vec<[f64; 3]>,
}

fn run_single(params: PusherSliderParams, steps: usize) -> Vec<[f64; 3]> {
    let h = 0.6 * params.half_extent;
    let cmd = PusherCommand::on_face(0, h, VN, 0.0);
    let mut state = SliderState::new(0.0, 0.0, 0.0);
    let mut traj = vec![state.pose];
    for _ in 0..steps {
        let (next, _) = params.step(state, cmd, DT);
        state = next;
        traj.push(state.pose);
    }
    traj
}

fn run_two(
    params: PusherSliderParams,
    c1: PusherCommand,
    c2: PusherCommand,
    steps: usize,
) -> Vec<[f64; 3]> {
    let mut state = SliderState::new(0.0, 0.0, 0.0);
    let mut traj = vec![state.pose];
    for _ in 0..steps {
        let (next, _) = params.two_contact_step(state, c1, c2, DT);
        state = next;
        traj.push(state.pose);
    }
    traj
}

fn scenarios() -> Vec<Scenario> {
    let params = PusherSliderParams::default();
    let h = 0.6 * params.half_extent;
    let hc = 0.7 * params.half_extent;
    vec![
        Scenario {
            name: "single (curves)",
            steps: 40,
            trajectory: run_single(params, 40),
        },
        Scenario {
            name: "two-point (straight)",
            steps: 40,
            trajectory: run_two(
                params,
                PusherCommand::on_face(0, h, VN, 0.0),
                PusherCommand::on_face(0, -h, VN, 0.0),
                40,
            ),
        },
        Scenario {
            name: "couple (spins in place)",
            steps: 34,
            trajectory: run_two(
                params,
                PusherCommand::on_face(0, hc, VN, 0.0),
                PusherCommand::on_face(2, -hc, VN, 0.0),
                34,
            ),
        },
    ]
}

fn render_csv(scs: &[Scenario]) -> String {
    let mut csv =
        String::from("scenario,steps,final_x,final_y,final_theta_deg,total_rotation_deg\n");
    for s in scs {
        let f = s.trajectory.last().unwrap();
        let _ = writeln!(
            csv,
            "{},{},{:.4},{:.4},{:.2},{:.2}",
            s.name,
            s.steps,
            f[0],
            f[1],
            f[2].to_degrees(),
            (f[2] - s.trajectory[0][2]).to_degrees()
        );
    }
    csv
}

fn box_corners(pose: [f64; 3], b: f64) -> [[f64; 2]; 4] {
    let (s, c) = pose[2].sin_cos();
    let local = [[-b, -b], [b, -b], [b, b], [-b, b]];
    let mut out = [[0.0; 2]; 4];
    for (o, l) in out.iter_mut().zip(local.iter()) {
        o[0] = pose[0] + c * l[0] - s * l[1];
        o[1] = pose[1] + s * l[0] + c * l[1];
    }
    out
}

fn render_svg(scs: &[Scenario]) -> String {
    let b = PusherSliderParams::default().half_extent;
    let panel_w = 300.0;
    let panel_h = 300.0;
    let width = panel_w * scs.len() as f64;
    let height = panel_h + 40.0;

    // Shared world bounds.
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for s in scs {
        for p in &s.trajectory {
            min_x = min_x.min(p[0]);
            max_x = max_x.max(p[0]);
            min_y = min_y.min(p[1]);
            max_y = max_y.max(p[1]);
        }
    }
    min_x -= 1.6 * b;
    max_x += 1.6 * b;
    min_y -= 1.6 * b;
    max_y += 1.6 * b;
    let span = (max_x - min_x).max(max_y - min_y).max(1e-6);

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
        "<text x=\"12\" y=\"24\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#1d1d1f\">Two-contact pushing: a second pusher straightens the path or spins the slider in place</text>"
    );

    for (i, s) in scs.iter().enumerate() {
        let px = i as f64 * panel_w;
        let py = 40.0;
        let map = |w: [f64; 2]| {
            let sx = px + 20.0 + ((w[0] - min_x) / span) * (panel_w - 40.0);
            let sy = py + (panel_h - 20.0) - ((w[1] - min_y) / span) * (panel_h - 40.0);
            (sx, sy)
        };

        // CoM path.
        let mut path = String::new();
        for p in &s.trajectory {
            let (sx, sy) = map([p[0], p[1]]);
            let _ = write!(path, "{sx:.1},{sy:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"#c7c7cc\" stroke-width=\"1.5\"/>",
            path.trim()
        );

        // Box outlines sampled along the motion.
        let n = s.trajectory.len();
        let samples = 6usize;
        for k in 0..=samples {
            let idx = (k * (n - 1)) / samples;
            let frac = k as f64 / samples as f64;
            let shade = 200 - (frac * 150.0) as i32;
            let stroke = format!("rgb({shade},{shade},{})", 235 - (frac * 40.0) as i32);
            let fill = if k == samples { "#0a84ff22" } else { "none" };
            let corners = box_corners(s.trajectory[idx], b);
            let mut poly = String::new();
            for c in &corners {
                let (sx, sy) = map(*c);
                let _ = write!(poly, "{sx:.1},{sy:.1} ");
            }
            let _ = writeln!(
                svg,
                "<polygon points=\"{}\" fill=\"{fill}\" stroke=\"{stroke}\" stroke-width=\"1.2\"/>",
                poly.trim()
            );
        }

        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            px + panel_w / 2.0,
            py + panel_h - 2.0,
            s.name
        );
    }

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let scs = scenarios();
    println!("two-contact pushing (contact-implicit quasi-static)");
    for s in &scs {
        let f = s.trajectory.last().unwrap();
        println!(
            "  {:<24} final=({:.3},{:.3}) theta={:.1}deg rotation={:.1}deg",
            s.name,
            f[0],
            f[1],
            f[2].to_degrees(),
            (f[2] - s.trajectory[0][2]).to_degrees()
        );
    }

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&scs)).ok();
    fs::write(SVG_OUTPUT, render_svg(&scs)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_point_is_straighter_than_single() {
        let scs = scenarios();
        let single = scs.iter().find(|s| s.name.starts_with("single")).unwrap();
        let two = scs
            .iter()
            .find(|s| s.name.starts_with("two-point"))
            .unwrap();
        let single_rot = (single.trajectory.last().unwrap()[2]).abs();
        let two_rot = (two.trajectory.last().unwrap()[2]).abs();
        assert!(
            two_rot < 0.1 * single_rot.max(1e-6),
            "two-point rotation {two_rot} should be far below single {single_rot}"
        );
    }

    #[test]
    fn couple_spins_with_little_translation() {
        let scs = scenarios();
        let couple = scs.iter().find(|s| s.name.starts_with("couple")).unwrap();
        let f = couple.trajectory.last().unwrap();
        let translation = (f[0] * f[0] + f[1] * f[1]).sqrt();
        assert!(f[2].abs() > 0.3, "should spin: {} rad", f[2]);
        assert!(
            translation < PusherSliderParams::default().half_extent,
            "should stay near origin: {translation}"
        );
    }
}
