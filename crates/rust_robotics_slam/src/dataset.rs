//! Loaders for the EuRoC MAV and KITTI odometry dataset layouts.

use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Matrix4, Quaternion, UnitQuaternion, Vector2, Vector3};

/// Errors returned while reading robotics datasets.
#[derive(Debug)]
pub enum DatasetError {
    Io(io::Error),
    InvalidData(String),
}

impl fmt::Display for DatasetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => error.fmt(formatter),
            Self::InvalidData(message) => write!(formatter, "invalid dataset: {message}"),
        }
    }
}

impl std::error::Error for DatasetError {}

impl From<io::Error> for DatasetError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

pub type DatasetResult<T> = Result<T, DatasetError>;

/// Timestamped EuRoC IMU measurement in SI units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub timestamp_ns: i64,
    pub angular_velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

/// Timestamp and on-disk image path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraFrame {
    pub timestamp_ns: i64,
    pub image_path: PathBuf,
}

/// EuRoC ground-truth state in the world frame.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundTruthState {
    pub timestamp_ns: i64,
    pub world_from_body: Matrix4<f64>,
    pub velocity: Vector3<f64>,
    pub gyroscope_bias: Vector3<f64>,
    pub accelerometer_bias: Vector3<f64>,
}

/// Minimal pinhole metadata from a EuRoC `sensor.yaml`.
#[derive(Debug, Clone, PartialEq)]
pub struct EurocCameraCalibration {
    pub intrinsics: [f64; 4],
    pub resolution: [usize; 2],
    /// EuRoC `T_BS`: body-from-camera transform.
    pub body_from_sensor: Matrix4<f64>,
}

/// Loaded EuRoC MAV sequence.
#[derive(Debug, Clone)]
pub struct EurocDataset {
    pub root: PathBuf,
    pub imu: Vec<ImuSample>,
    pub camera_frames: Vec<CameraFrame>,
    pub ground_truth: Vec<GroundTruthState>,
    pub camera: EurocCameraCalibration,
}

/// One initialized landmark in the optional RustRobotics feature sidecar.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackedLandmark {
    pub id: usize,
    pub position: Vector3<f64>,
}

/// One timestamped feature observation in the optional sidecar.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureTrackObservation {
    pub timestamp_ns: i64,
    pub landmark_id: usize,
    pub pixel: Vector2<f64>,
}

/// Pre-extracted visual tracks consumed by the offline VIO pipeline.
#[derive(Debug, Clone, PartialEq)]
pub struct EurocFeatureTracks {
    pub landmarks: Vec<TrackedLandmark>,
    pub observations: Vec<FeatureTrackObservation>,
}

impl EurocDataset {
    /// Loads one extracted EuRoC sequence. `root` may point either at the
    /// sequence directory or directly at its `mav0` directory.
    pub fn load(root: impl AsRef<Path>) -> DatasetResult<Self> {
        let root = root.as_ref();
        let mav0 = if root.join("mav0").is_dir() {
            root.join("mav0")
        } else {
            root.to_path_buf()
        };
        let imu = parse_euroc_imu(&mav0.join("imu0/data.csv"))?;
        let camera_frames =
            parse_image_index(&mav0.join("cam0/data.csv"), &mav0.join("cam0/data"))?;
        let ground_truth_path = mav0.join("state_groundtruth_estimate0/data.csv");
        let ground_truth = if ground_truth_path.exists() {
            parse_euroc_ground_truth(&ground_truth_path)?
        } else {
            Vec::new()
        };
        let camera = parse_euroc_camera_yaml(&mav0.join("cam0/sensor.yaml"))?;
        ensure_strict_timestamps(imu.iter().map(|sample| sample.timestamp_ns), "EuRoC IMU")?;
        ensure_strict_timestamps(
            camera_frames.iter().map(|frame| frame.timestamp_ns),
            "EuRoC camera",
        )?;
        Ok(Self {
            root: mav0,
            imu,
            camera_frames,
            ground_truth,
            camera,
        })
    }

    /// Returns IMU samples in `(start, end]`.
    pub fn imu_between(&self, start_ns: i64, end_ns: i64) -> &[ImuSample] {
        let start = self
            .imu
            .partition_point(|sample| sample.timestamp_ns <= start_ns);
        let end = self
            .imu
            .partition_point(|sample| sample.timestamp_ns <= end_ns);
        &self.imu[start..end]
    }

    /// Loads optional pre-extracted tracks from
    /// `mav0/rust_robotics/{landmarks,observations}.csv`.
    pub fn load_feature_tracks(&self) -> DatasetResult<EurocFeatureTracks> {
        let directory = self.root.join("rust_robotics");
        let landmarks = data_lines(&directory.join("landmarks.csv"))?
            .iter()
            .map(|line| {
                let fields = parse_fields::<f64>(line, 4, "feature landmark row")?;
                Ok(TrackedLandmark {
                    id: fields[0] as usize,
                    position: Vector3::new(fields[1], fields[2], fields[3]),
                })
            })
            .collect::<DatasetResult<Vec<_>>>()?;
        let observations = data_lines(&directory.join("observations.csv"))?
            .iter()
            .map(|line| {
                let fields = parse_fields::<f64>(line, 4, "feature observation row")?;
                Ok(FeatureTrackObservation {
                    timestamp_ns: fields[0] as i64,
                    landmark_id: fields[1] as usize,
                    pixel: Vector2::new(fields[2], fields[3]),
                })
            })
            .collect::<DatasetResult<Vec<_>>>()?;
        if landmarks
            .iter()
            .enumerate()
            .any(|(index, landmark)| landmark.id != index)
        {
            return Err(DatasetError::InvalidData(
                "feature landmark IDs must be contiguous and zero-based".into(),
            ));
        }
        Ok(EurocFeatureTracks {
            landmarks,
            observations,
        })
    }
}

/// KITTI camera projection matrices.
#[derive(Debug, Clone, PartialEq)]
pub struct KittiCalibration {
    pub projections: [Option<nalgebra::SMatrix<f64, 3, 4>>; 4],
    pub transform_velodyne_to_camera: Option<Matrix4<f64>>,
}

/// One KITTI odometry frame.
#[derive(Debug, Clone, PartialEq)]
pub struct KittiFrame {
    pub index: usize,
    pub timestamp_seconds: f64,
    pub image_paths: [PathBuf; 4],
    pub velodyne_path: PathBuf,
    pub ground_truth_pose: Option<Matrix4<f64>>,
}

/// Loaded KITTI odometry sequence metadata.
#[derive(Debug, Clone)]
pub struct KittiOdometryDataset {
    pub sequence: String,
    pub calibration: KittiCalibration,
    pub frames: Vec<KittiFrame>,
}

impl KittiOdometryDataset {
    /// Loads a KITTI odometry sequence such as `"00"`.
    pub fn load(root: impl AsRef<Path>, sequence: &str) -> DatasetResult<Self> {
        let root = root.as_ref();
        let directory = root.join("sequences").join(sequence);
        let times = numeric_lines(&directory.join("times.txt"))?;
        if times.windows(2).any(|window| window[1] <= window[0]) {
            return Err(DatasetError::InvalidData(
                "KITTI timestamps must be strictly increasing".into(),
            ));
        }
        let calibration = parse_kitti_calibration(&directory.join("calib.txt"))?;
        let pose_path = root.join("poses").join(format!("{sequence}.txt"));
        let poses = if pose_path.exists() {
            parse_kitti_poses(&pose_path)?
        } else {
            Vec::new()
        };
        if !poses.is_empty() && poses.len() != times.len() {
            return Err(DatasetError::InvalidData(format!(
                "KITTI pose count {} does not match timestamp count {}",
                poses.len(),
                times.len()
            )));
        }
        let frames = times
            .into_iter()
            .enumerate()
            .map(|(index, timestamp_seconds)| {
                let stem = format!("{index:06}");
                KittiFrame {
                    index,
                    timestamp_seconds,
                    image_paths: std::array::from_fn(|camera| {
                        directory
                            .join(format!("image_{camera}"))
                            .join(format!("{stem}.png"))
                    }),
                    velodyne_path: directory.join("velodyne").join(format!("{stem}.bin")),
                    ground_truth_pose: poses.get(index).copied(),
                }
            })
            .collect();
        Ok(Self {
            sequence: sequence.into(),
            calibration,
            frames,
        })
    }

    /// Reads `(x, y, z, reflectance)` float32 tuples from a Velodyne scan.
    pub fn read_velodyne(path: impl AsRef<Path>) -> DatasetResult<Vec<[f32; 4]>> {
        let bytes = fs::read(path)?;
        if bytes.len() % 16 != 0 {
            return Err(DatasetError::InvalidData(
                "KITTI Velodyne data length is not divisible by 16".into(),
            ));
        }
        Ok(bytes
            .chunks_exact(16)
            .map(|chunk| {
                std::array::from_fn(|index| {
                    let offset = index * 4;
                    f32::from_le_bytes(chunk[offset..offset + 4].try_into().expect("four bytes"))
                })
            })
            .collect())
    }
}

fn data_lines(path: &Path) -> DatasetResult<Vec<String>> {
    Ok(fs::read_to_string(path)?
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .map(str::to_owned)
        .collect())
}

fn parse_fields<T>(line: &str, expected: usize, context: &str) -> DatasetResult<Vec<T>>
where
    T: std::str::FromStr,
{
    let fields = line
        .split(',')
        .map(str::trim)
        .map(|value| {
            value.parse::<T>().map_err(|_| {
                DatasetError::InvalidData(format!("{context} contains invalid value `{value}`"))
            })
        })
        .collect::<DatasetResult<Vec<_>>>()?;
    if fields.len() != expected {
        return Err(DatasetError::InvalidData(format!(
            "{context} expected {expected} columns, found {}",
            fields.len()
        )));
    }
    Ok(fields)
}

fn parse_euroc_imu(path: &Path) -> DatasetResult<Vec<ImuSample>> {
    data_lines(path)?
        .iter()
        .map(|line| {
            let fields = parse_fields::<f64>(line, 7, "EuRoC IMU row")?;
            Ok(ImuSample {
                timestamp_ns: fields[0] as i64,
                angular_velocity: Vector3::new(fields[1], fields[2], fields[3]),
                acceleration: Vector3::new(fields[4], fields[5], fields[6]),
            })
        })
        .collect()
}

fn parse_image_index(path: &Path, data_directory: &Path) -> DatasetResult<Vec<CameraFrame>> {
    data_lines(path)?
        .iter()
        .map(|line| {
            let mut fields = line.split(',').map(str::trim);
            let timestamp = fields
                .next()
                .and_then(|value| value.parse::<i64>().ok())
                .ok_or_else(|| {
                    DatasetError::InvalidData("invalid EuRoC camera timestamp".into())
                })?;
            let filename = fields
                .next()
                .ok_or_else(|| DatasetError::InvalidData("missing EuRoC image filename".into()))?;
            if fields.next().is_some() {
                return Err(DatasetError::InvalidData(
                    "EuRoC camera row must have two columns".into(),
                ));
            }
            Ok(CameraFrame {
                timestamp_ns: timestamp,
                image_path: data_directory.join(filename),
            })
        })
        .collect()
}

fn parse_euroc_ground_truth(path: &Path) -> DatasetResult<Vec<GroundTruthState>> {
    data_lines(path)?
        .iter()
        .map(|line| {
            let fields = parse_fields::<f64>(line, 17, "EuRoC ground-truth row")?;
            let quaternion = UnitQuaternion::new_normalize(Quaternion::new(
                fields[4], fields[5], fields[6], fields[7],
            ));
            let mut transform = Matrix4::identity();
            transform
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(quaternion.to_rotation_matrix().matrix());
            transform[(0, 3)] = fields[1];
            transform[(1, 3)] = fields[2];
            transform[(2, 3)] = fields[3];
            Ok(GroundTruthState {
                timestamp_ns: fields[0] as i64,
                world_from_body: transform,
                velocity: Vector3::new(fields[8], fields[9], fields[10]),
                gyroscope_bias: Vector3::new(fields[11], fields[12], fields[13]),
                accelerometer_bias: Vector3::new(fields[14], fields[15], fields[16]),
            })
        })
        .collect()
}

fn bracket_values<T>(line: &str, key: &str) -> DatasetResult<Vec<T>>
where
    T: std::str::FromStr,
{
    let values = line
        .strip_prefix(key)
        .and_then(|value| value.trim().strip_prefix('['))
        .and_then(|value| value.strip_suffix(']'))
        .ok_or_else(|| DatasetError::InvalidData(format!("missing `{key}` list")))?;
    values
        .split(',')
        .map(str::trim)
        .map(|value| {
            value
                .parse()
                .map_err(|_| DatasetError::InvalidData(format!("invalid `{key}` value `{value}`")))
        })
        .collect()
}

fn parse_euroc_camera_yaml(path: &Path) -> DatasetResult<EurocCameraCalibration> {
    let contents = fs::read_to_string(path)?;
    let intrinsics_line = contents
        .lines()
        .map(str::trim)
        .find(|line| line.starts_with("intrinsics:"))
        .ok_or_else(|| DatasetError::InvalidData("missing EuRoC intrinsics".into()))?;
    let resolution_line = contents
        .lines()
        .map(str::trim)
        .find(|line| line.starts_with("resolution:"))
        .ok_or_else(|| DatasetError::InvalidData("missing EuRoC resolution".into()))?;
    let intrinsics = bracket_values::<f64>(intrinsics_line, "intrinsics:")?;
    let resolution = bracket_values::<usize>(resolution_line, "resolution:")?;
    let mut in_body_from_sensor = false;
    let mut body_from_sensor = None;
    for line in contents.lines().map(str::trim) {
        if line == "T_BS:" {
            in_body_from_sensor = true;
        } else if in_body_from_sensor && line.starts_with("data:") {
            let values = bracket_values::<f64>(line, "data:")?;
            if values.len() != 16 {
                return Err(DatasetError::InvalidData(
                    "EuRoC T_BS must contain 16 values".into(),
                ));
            }
            body_from_sensor = Some(Matrix4::from_row_slice(&values));
            break;
        }
    }
    Ok(EurocCameraCalibration {
        intrinsics: intrinsics.try_into().map_err(|_| {
            DatasetError::InvalidData("EuRoC intrinsics must contain four values".into())
        })?,
        resolution: resolution.try_into().map_err(|_| {
            DatasetError::InvalidData("EuRoC resolution must contain two values".into())
        })?,
        body_from_sensor: body_from_sensor
            .ok_or_else(|| DatasetError::InvalidData("missing EuRoC camera T_BS".into()))?,
    })
}

fn numeric_lines(path: &Path) -> DatasetResult<Vec<f64>> {
    data_lines(path)?
        .iter()
        .map(|line| {
            line.parse()
                .map_err(|_| DatasetError::InvalidData(format!("invalid numeric row `{line}`")))
        })
        .collect()
}

fn whitespace_numbers(line: &str, context: &str) -> DatasetResult<Vec<f64>> {
    line.split_whitespace()
        .map(|value| {
            value.parse().map_err(|_| {
                DatasetError::InvalidData(format!("{context} contains invalid value `{value}`"))
            })
        })
        .collect()
}

fn matrix_3x4(values: &[f64], context: &str) -> DatasetResult<nalgebra::SMatrix<f64, 3, 4>> {
    if values.len() != 12 {
        return Err(DatasetError::InvalidData(format!(
            "{context} needs 12 values"
        )));
    }
    Ok(nalgebra::SMatrix::<f64, 3, 4>::from_row_slice(values))
}

fn parse_kitti_calibration(path: &Path) -> DatasetResult<KittiCalibration> {
    let mut projections = [None, None, None, None];
    let mut transform_velodyne_to_camera = None;
    for line in data_lines(path)? {
        let (key, values) = line.split_once(':').ok_or_else(|| {
            DatasetError::InvalidData(format!("invalid KITTI calibration row `{line}`"))
        })?;
        let values = whitespace_numbers(values, "KITTI calibration")?;
        if let Some(camera) = key
            .strip_prefix('P')
            .and_then(|value| value.parse::<usize>().ok())
        {
            if camera < projections.len() {
                projections[camera] = Some(matrix_3x4(&values, key)?);
            }
        } else if matches!(key, "Tr" | "Tr_velo_to_cam") {
            let matrix = matrix_3x4(&values, key)?;
            let mut transform = Matrix4::identity();
            transform.fixed_view_mut::<3, 4>(0, 0).copy_from(&matrix);
            transform_velodyne_to_camera = Some(transform);
        }
    }
    if projections.iter().all(Option::is_none) {
        return Err(DatasetError::InvalidData(
            "KITTI calibration contains no projection matrices".into(),
        ));
    }
    Ok(KittiCalibration {
        projections,
        transform_velodyne_to_camera,
    })
}

fn parse_kitti_poses(path: &Path) -> DatasetResult<Vec<Matrix4<f64>>> {
    data_lines(path)?
        .iter()
        .map(|line| {
            let values = whitespace_numbers(line, "KITTI pose")?;
            let matrix = matrix_3x4(&values, "KITTI pose")?;
            let mut transform = Matrix4::identity();
            transform.fixed_view_mut::<3, 4>(0, 0).copy_from(&matrix);
            let rotation = transform.fixed_view::<3, 3>(0, 0);
            if (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1.0e-4 {
                return Err(DatasetError::InvalidData(
                    "KITTI pose rotation is not orthonormal".into(),
                ));
            }
            Ok(transform)
        })
        .collect()
}

fn ensure_strict_timestamps(
    timestamps: impl IntoIterator<Item = i64>,
    context: &str,
) -> DatasetResult<()> {
    let mut previous = None;
    for timestamp in timestamps {
        if previous.is_some_and(|value| timestamp <= value) {
            return Err(DatasetError::InvalidData(format!(
                "{context} timestamps must be strictly increasing"
            )));
        }
        previous = Some(timestamp);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name)
    }

    #[test]
    fn loads_euroc_layout_and_slices_imu() {
        let dataset = EurocDataset::load(fixture("euroc_mini")).unwrap();
        assert_eq!(dataset.imu.len(), 5);
        assert_eq!(dataset.camera_frames.len(), 3);
        assert_eq!(dataset.ground_truth.len(), 3);
        assert_eq!(dataset.camera.resolution, [752, 480]);
        assert_eq!(dataset.camera.body_from_sensor, Matrix4::identity());
        assert_eq!(dataset.imu_between(1_000_000_000, 1_010_000_000).len(), 2);
        assert!(dataset.camera_frames[0]
            .image_path
            .ends_with("1000000000.png"));
        let tracks = dataset.load_feature_tracks().unwrap();
        assert_eq!(tracks.landmarks.len(), 4);
        assert_eq!(tracks.observations.len(), 12);
    }

    #[test]
    fn loads_kitti_layout_and_velodyne_points() {
        let dataset = KittiOdometryDataset::load(fixture("kitti_mini"), "00").unwrap();
        assert_eq!(dataset.frames.len(), 3);
        assert_eq!(dataset.frames[2].ground_truth_pose.unwrap()[(0, 3)], 2.0);
        let scan_path = std::env::temp_dir().join(format!(
            "rust_robotics_kitti_scan_{}.bin",
            std::process::id()
        ));
        let bytes = [1.0_f32, 2.0, 3.0, 0.5, -1.0, 0.0, 4.0, 0.25]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        fs::write(&scan_path, bytes).unwrap();
        let points = KittiOdometryDataset::read_velodyne(&scan_path).unwrap();
        fs::remove_file(scan_path).unwrap();
        assert_eq!(points, vec![[1.0, 2.0, 3.0, 0.5], [-1.0, 0.0, 4.0, 0.25]]);
    }
}
