//! Render the checked-in EuRoC VIO replay as a dependency-free SVG.

use std::fmt::Write;
use std::path::{Path, PathBuf};

use nalgebra::Vector3;
use rust_robotics::slam::dataset::EurocDataset;
use rust_robotics::slam::vio_pipeline::{
    euroc_vio_input, pose_error, run_vio_pipeline, VioPipelineConfig,
};

const OUTPUT: &str = "docs/assets/euroc-vio-pipeline.svg";

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../rust_robotics_slam/tests/fixtures/euroc_mini")
}

fn path_points(values: &[f64], x0: f64, y0: f64) -> String {
    let maximum = values
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .max(1.0e-9);
    values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            let fraction = index as f64 / (values.len() - 1).max(1) as f64;
            format!(
                "{:.1},{:.1}",
                x0 + fraction * 520.0,
                y0 - value / maximum * 170.0
            )
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dataset = EurocDataset::load(fixture_path())?;
    let tracks = dataset.load_feature_tracks()?;
    let mut input = euroc_vio_input(&dataset, &tracks, Vector3::new(0.0, 0.0, -9.81))?;
    for landmark in &mut input.landmarks {
        *landmark += Vector3::new(0.02, -0.01, 0.04);
    }
    let result = run_vio_pipeline(&input, &VioPipelineConfig::default())?;
    let truth_x = dataset
        .ground_truth
        .iter()
        .map(|state| state.world_from_body[(0, 3)])
        .collect::<Vec<_>>();
    let imu_x = result
        .imu_states
        .iter()
        .map(|state| state.position.x)
        .collect::<Vec<_>>();
    let optimized_x = result
        .trajectory
        .iter()
        .map(|pose| pose[(0, 3)])
        .collect::<Vec<_>>();
    let expected_terminal =
        dataset.ground_truth.last().unwrap().world_from_body * dataset.camera.body_from_sensor;
    let terminal_error = pose_error(result.trajectory.last().unwrap(), &expected_terminal);

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1160" height="600" viewBox="0 0 1160 600" role="img" aria-labelledby="title desc"><title id="title">EuRoC visual-inertial pipeline</title><desc id="desc">IMU preintegration, bundle adjustment, and pose graph fusion with a trajectory plot.</desc><rect width="1160" height="600" fill="#f8fafc"/>"##
    )?;
    writeln!(
        svg,
        r##"<text x="54" y="48" font-family="sans-serif" font-size="28" font-weight="700" fill="#0f172a">EuRoC → metric visual-inertial trajectory</text><text x="54" y="76" font-family="sans-serif" font-size="14" fill="#475569">Official MAV CSV layout · pre-extracted feature sidecar · pure Rust optimization</text>"##
    )?;
    for (index, (title, detail, color)) in [
        ("EuRoC loader", "IMU · cam0 · T_BS", "#2563eb"),
        ("Preintegration", "SO(3) · bias · gravity", "#7c3aed"),
        ("Bundle adjustment", "camera + landmarks · Schur", "#ea580c"),
        ("SE(3) pose graph", "IMU edges + visual closure", "#059669"),
    ]
    .into_iter()
    .enumerate()
    {
        let x = 54 + index * 270;
        writeln!(
            svg,
            r##"<rect x="{x}" y="108" width="220" height="92" rx="14" fill="#fff" stroke="{color}" stroke-width="2"/><text x="{}" y="143" font-family="sans-serif" font-size="17" font-weight="700" fill="{color}">{title}</text><text x="{}" y="171" font-family="sans-serif" font-size="12" fill="#475569">{detail}</text>"##,
            x + 18,
            x + 18
        )?;
        if index < 3 {
            writeln!(
                svg,
                r##"<path d="M {} 154 H {}" stroke="#94a3b8" stroke-width="3" marker-end="url(#arrow)"/>"##,
                x + 222,
                x + 266
            )?;
        }
    }
    writeln!(
        svg,
        r##"<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#94a3b8"/></marker></defs>"##
    )?;
    writeln!(
        svg,
        r##"<rect x="54" y="236" width="690" height="300" rx="16" fill="#fff" stroke="#cbd5e1"/><text x="78" y="271" font-family="sans-serif" font-size="18" font-weight="700" fill="#0f172a">Metric x-position over camera time</text><line x1="100" y1="490" x2="680" y2="490" stroke="#94a3b8"/><line x1="100" y1="300" x2="100" y2="490" stroke="#94a3b8"/>"##
    )?;
    for (values, color, width) in [
        (&truth_x, "#0f172a", 7),
        (&imu_x, "#7c3aed", 4),
        (&optimized_x, "#059669", 3),
    ] {
        writeln!(
            svg,
            r##"<polyline points="{}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round" stroke-linejoin="round"/>"##,
            path_points(values, 100.0, 490.0)
        )?;
    }
    writeln!(
        svg,
        r##"<text x="100" y="518" font-family="sans-serif" font-size="12" fill="#64748b">1.000 s</text><text x="625" y="518" font-family="sans-serif" font-size="12" fill="#64748b">1.020 s</text><circle cx="785" cy="286" r="6" fill="#0f172a"/><text x="800" y="291" font-family="sans-serif" font-size="14" fill="#334155">ground truth</text><circle cx="785" cy="318" r="6" fill="#7c3aed"/><text x="800" y="323" font-family="sans-serif" font-size="14" fill="#334155">IMU prediction</text><circle cx="785" cy="350" r="6" fill="#059669"/><text x="800" y="355" font-family="sans-serif" font-size="14" fill="#334155">fused trajectory</text>"##
    )?;
    writeln!(
        svg,
        r##"<rect x="770" y="390" width="330" height="146" rx="14" fill="#ecfdf5" stroke="#6ee7b7"/><text x="792" y="423" font-family="sans-serif" font-size="16" font-weight="700" fill="#065f46">Replay diagnostics</text><text x="792" y="452" font-family="monospace" font-size="13" fill="#334155">keyframes              {}</text><text x="792" y="476" font-family="monospace" font-size="13" fill="#334155">feature observations   {}</text><text x="792" y="500" font-family="monospace" font-size="13" fill="#334155">terminal SE(3) error   {:.3e}</text>"##,
        result.trajectory.len(),
        input.observations.len(),
        terminal_error
    )?;
    writeln!(
        svg,
        r##"<text x="54" y="576" font-family="sans-serif" font-size="12" fill="#64748b">Generated by cargo run --example render_euroc_vio_svg; no Python, GUI, or system renderer required.</text></svg>"##
    )?;
    if let Some(parent) = Path::new(OUTPUT).parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(OUTPUT, svg)?;
    println!("wrote {OUTPUT}");
    Ok(())
}
