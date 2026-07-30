//! Generate a VIO feature sidecar directly from EuRoC cam0 PNG images.
//!
//! ```text
//! cargo run --release -p rust_robotics --example generate_euroc_feature_tracks \
//!   --no-default-features --features slam -- /datasets/EuRoC/MH_01_easy
//! ```

use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use nalgebra::Vector3;
use rust_robotics::slam::dataset::EurocDataset;
use rust_robotics::slam::imu_preintegration::ImuNoise;
use rust_robotics::slam::vio_pipeline::euroc_imu_camera_trajectory;
use rust_robotics::slam::visual_frontend::{
    triangulate_tracks, FeatureFrontendConfig, FeatureTracker, GrayImage, TriangulationConfig,
};

type CliResult<T> = Result<T, Box<dyn std::error::Error>>;

#[derive(Debug)]
struct Arguments {
    sequence: PathBuf,
    output: Option<PathBuf>,
    overwrite: bool,
    max_features: usize,
}

#[derive(Debug)]
struct GenerationSummary {
    frames: usize,
    accepted_matches: usize,
    persistent_tracks: usize,
    triangulated_landmarks: usize,
    observations: usize,
    output: PathBuf,
}

fn parse_arguments() -> CliResult<Arguments> {
    let mut values = std::env::args_os().skip(1);
    let sequence = values.next().map(PathBuf::from).ok_or(
        "usage: generate_euroc_feature_tracks SEQUENCE [--output DIR] [--force] \
         [--max-features N]",
    )?;
    let mut output = None;
    let mut overwrite = false;
    let mut max_features = FeatureFrontendConfig::default().max_features;
    while let Some(argument) = values.next() {
        match argument.to_str() {
            Some("--output") => {
                output = Some(PathBuf::from(
                    values.next().ok_or("--output requires a directory")?,
                ));
            }
            Some("--force") => overwrite = true,
            Some("--max-features") => {
                max_features = values
                    .next()
                    .and_then(|value| value.to_str().and_then(|value| value.parse().ok()))
                    .ok_or("--max-features requires a positive integer")?;
                if max_features == 0 {
                    return Err("--max-features must be positive".into());
                }
            }
            _ => {
                return Err(format!("unrecognized argument: {}", argument.to_string_lossy()).into())
            }
        }
    }
    Ok(Arguments {
        sequence,
        output,
        overwrite,
        max_features,
    })
}

fn decode_png(path: &Path) -> CliResult<GrayImage> {
    let mut decoder = png::Decoder::new(BufReader::new(File::open(path)?));
    decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
    let mut reader = decoder.read_info()?;
    let mut buffer = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut buffer)?;
    let bytes = &buffer[..info.buffer_size()];
    let pixels = match info.color_type {
        png::ColorType::Grayscale => bytes.to_vec(),
        png::ColorType::GrayscaleAlpha => bytes.chunks_exact(2).map(|pixel| pixel[0]).collect(),
        png::ColorType::Rgb => bytes
            .chunks_exact(3)
            .map(|pixel| rgb_to_gray(pixel[0], pixel[1], pixel[2]))
            .collect(),
        png::ColorType::Rgba => bytes
            .chunks_exact(4)
            .map(|pixel| rgb_to_gray(pixel[0], pixel[1], pixel[2]))
            .collect(),
        png::ColorType::Indexed => {
            return Err("indexed PNG remained after decoder expansion".into());
        }
    };
    Ok(GrayImage::new(
        info.width as usize,
        info.height as usize,
        pixels,
    )?)
}

fn rgb_to_gray(red: u8, green: u8, blue: u8) -> u8 {
    ((77 * red as u16 + 150 * green as u16 + 29 * blue as u16) >> 8) as u8
}

fn generate(arguments: &Arguments) -> CliResult<GenerationSummary> {
    let dataset = EurocDataset::load(&arguments.sequence)?;
    let output = arguments
        .output
        .clone()
        .unwrap_or_else(|| dataset.root.join("rust_robotics"));
    let config = FeatureFrontendConfig {
        max_features: arguments.max_features,
        ..FeatureFrontendConfig::default()
    };
    let mut tracker = FeatureTracker::new(config)?;
    for (index, frame) in dataset.camera_frames.iter().enumerate() {
        tracker.process(decode_png(&frame.image_path)?)?;
        if (index + 1) % 100 == 0 || index + 1 == dataset.camera_frames.len() {
            eprintln!(
                "tracked {}/{} frames",
                index + 1,
                dataset.camera_frames.len()
            );
        }
    }
    let accepted_matches = tracker.accepted_matches();
    let image_tracks = tracker.finish();
    let camera_trajectory =
        euroc_imu_camera_trajectory(&dataset, Vector3::new(0.0, 0.0, -9.81), ImuNoise::default())?;
    let timestamps = dataset
        .camera_frames
        .iter()
        .map(|frame| frame.timestamp_ns)
        .collect::<Vec<_>>();
    let tracks = triangulate_tracks(
        &image_tracks,
        &timestamps,
        &camera_trajectory,
        dataset.camera.intrinsics,
        TriangulationConfig::default(),
    )?;
    dataset.write_feature_tracks(&tracks, &output, arguments.overwrite)?;
    Ok(GenerationSummary {
        frames: dataset.camera_frames.len(),
        accepted_matches,
        persistent_tracks: image_tracks.len(),
        triangulated_landmarks: tracks.landmarks.len(),
        observations: tracks.observations.len(),
        output,
    })
}

fn main() -> CliResult<()> {
    let arguments = parse_arguments()?;
    let summary = generate(&arguments)?;
    println!("EuRoC visual frontend: {}", arguments.sequence.display());
    println!("  frames={}", summary.frames);
    println!("  accepted_matches={}", summary.accepted_matches);
    println!("  persistent_tracks={}", summary.persistent_tracks);
    println!(
        "  triangulated_landmarks={}",
        summary.triangulated_landmarks
    );
    println!("  observations={}", summary.observations);
    println!("  output={}", summary.output.display());
    println!("  status=ok");
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::io::BufWriter;

    use super::*;

    fn write_png(path: &Path, shift: usize) {
        let width = 96;
        let height = 72;
        let mut pixels = vec![20; width * height];
        for &(x, y) in &[(22, 18), (46, 20), (74, 22), (28, 48), (62, 50)] {
            let x = x - shift;
            for row in y - 3..=y + 3 {
                for column in x - 3..=x + 3 {
                    pixels[row * width + column] = if (row + column) % 2 == 0 { 240 } else { 80 };
                }
            }
        }
        let mut encoder = png::Encoder::new(
            BufWriter::new(File::create(path).unwrap()),
            width as u32,
            height as u32,
        );
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Eight);
        encoder
            .write_header()
            .unwrap()
            .write_image_data(&pixels)
            .unwrap();
    }

    #[test]
    fn png_cli_pipeline_writes_a_reloadable_sidecar() {
        let root = std::env::temp_dir().join(format!(
            "rust_robotics_euroc_frontend_{}",
            std::process::id()
        ));
        if root.exists() {
            fs::remove_dir_all(&root).unwrap();
        }
        let mav0 = root.join("mav0");
        let camera = mav0.join("cam0");
        fs::create_dir_all(camera.join("data")).unwrap();
        fs::create_dir_all(mav0.join("imu0")).unwrap();
        fs::create_dir_all(mav0.join("state_groundtruth_estimate0")).unwrap();
        fs::write(
            camera.join("data.csv"),
            "#timestamp,filename\n1000000000,0.png\n1100000000,1.png\n1200000000,2.png\n",
        )
        .unwrap();
        fs::write(
            camera.join("sensor.yaml"),
            "sensor_type: camera\nT_BS:\n  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\nresolution: [96, 72]\nintrinsics: [80, 80, 48, 36]\n",
        )
        .unwrap();
        fs::write(
            mav0.join("imu0/data.csv"),
            "#timestamp,wx,wy,wz,ax,ay,az\n1000000000,0,0,0,0,0,9.81\n1050000000,0,0,0,0,0,9.81\n1100000000,0,0,0,0,0,9.81\n1150000000,0,0,0,0,0,9.81\n1200000000,0,0,0,0,0,9.81\n",
        )
        .unwrap();
        fs::write(
            mav0.join("state_groundtruth_estimate0/data.csv"),
            "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,bw_x,bw_y,bw_z,ba_x,ba_y,ba_z\n1000000000,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0\n",
        )
        .unwrap();
        write_png(&camera.join("data/0.png"), 0);
        write_png(&camera.join("data/1.png"), 2);
        write_png(&camera.join("data/2.png"), 4);

        let output = root.join("generated");
        let summary = generate(&Arguments {
            sequence: root.clone(),
            output: Some(output.clone()),
            overwrite: false,
            max_features: 80,
        })
        .unwrap();
        assert_eq!(summary.frames, 3);
        assert!(summary.triangulated_landmarks >= 3);
        let mut dataset = EurocDataset::load(&root).unwrap();
        dataset.root = mav0;
        fs::create_dir_all(dataset.root.join("rust_robotics")).unwrap();
        fs::copy(
            output.join("landmarks.csv"),
            dataset.root.join("rust_robotics/landmarks.csv"),
        )
        .unwrap();
        fs::copy(
            output.join("observations.csv"),
            dataset.root.join("rust_robotics/observations.csv"),
        )
        .unwrap();
        let reloaded = dataset.load_feature_tracks().unwrap();
        assert_eq!(reloaded.landmarks.len(), summary.triangulated_landmarks);
        fs::remove_dir_all(root).unwrap();
    }
}
