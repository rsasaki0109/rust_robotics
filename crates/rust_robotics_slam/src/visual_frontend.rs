//! Decoder-independent sparse visual frontend for EuRoC-style sequences.
//!
//! Corners are distributed with non-maximum suppression, tracked with
//! pyramidal Lucas-Kanade optical flow, checked in both directions, and
//! triangulated from an IMU-predicted camera trajectory.

use nalgebra::{Matrix3, Matrix4, Vector2, Vector3};
use rust_robotics_optimization::{OptimizationError, OptimizationResult};

use crate::dataset::{EurocFeatureTracks, FeatureTrackObservation, TrackedLandmark};

/// Owned eight-bit grayscale image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayImage {
    width: usize,
    height: usize,
    pixels: Vec<u8>,
}

impl GrayImage {
    pub fn new(width: usize, height: usize, pixels: Vec<u8>) -> OptimizationResult<Self> {
        if width < 8 || height < 8 || pixels.len() != width * height {
            return Err(OptimizationError::InvalidParameter(
                "gray image dimensions or pixel count are invalid".into(),
            ));
        }
        Ok(Self {
            width,
            height,
            pixels,
        })
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    fn sample(&self, point: Vector2<f64>) -> Option<f64> {
        if point.x < 0.0
            || point.y < 0.0
            || point.x >= (self.width - 1) as f64
            || point.y >= (self.height - 1) as f64
        {
            return None;
        }
        let x = point.x.floor() as usize;
        let y = point.y.floor() as usize;
        let dx = point.x - x as f64;
        let dy = point.y - y as f64;
        let at = |column: usize, row: usize| self.pixels[row * self.width + column] as f64;
        Some(
            at(x, y) * (1.0 - dx) * (1.0 - dy)
                + at(x + 1, y) * dx * (1.0 - dy)
                + at(x, y + 1) * (1.0 - dx) * dy
                + at(x + 1, y + 1) * dx * dy,
        )
    }

    fn half_size(&self) -> Self {
        let width = self.width / 2;
        let height = self.height / 2;
        let mut pixels = vec![0; width * height];
        for y in 0..height {
            for x in 0..width {
                let source = 2 * y * self.width + 2 * x;
                let sum = self.pixels[source] as u16
                    + self.pixels[source + 1] as u16
                    + self.pixels[source + self.width] as u16
                    + self.pixels[source + self.width + 1] as u16;
                pixels[y * width + x] = (sum / 4) as u8;
            }
        }
        Self {
            width,
            height,
            pixels,
        }
    }
}

/// Sparse tracking parameters.
#[derive(Debug, Clone, Copy)]
pub struct FeatureFrontendConfig {
    pub max_features: usize,
    pub quality_level: f64,
    pub min_distance: f64,
    pub pyramid_levels: usize,
    pub window_radius: usize,
    pub max_iterations: usize,
    pub convergence_epsilon: f64,
    pub forward_backward_tolerance: f64,
    pub max_patch_rms: f64,
    pub min_track_length: usize,
}

impl Default for FeatureFrontendConfig {
    fn default() -> Self {
        Self {
            max_features: 240,
            quality_level: 0.01,
            min_distance: 12.0,
            pyramid_levels: 3,
            window_radius: 3,
            max_iterations: 12,
            convergence_epsilon: 0.02,
            forward_backward_tolerance: 1.0,
            max_patch_rms: 30.0,
            min_track_length: 3,
        }
    }
}

/// One 2D observation retained by the frontend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImageFeatureObservation {
    pub frame: usize,
    pub pixel: Vector2<f64>,
}

/// A feature identity followed across consecutive frames.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageFeatureTrack {
    pub id: usize,
    pub observations: Vec<ImageFeatureObservation>,
}

/// Stateful streaming tracker. Only the previous image pyramid is retained.
#[derive(Debug)]
pub struct FeatureTracker {
    config: FeatureFrontendConfig,
    previous: Option<Vec<GrayImage>>,
    tracks: Vec<ImageFeatureTrack>,
    active: Vec<usize>,
    frame_count: usize,
    accepted_matches: usize,
}

impl FeatureTracker {
    pub fn new(config: FeatureFrontendConfig) -> OptimizationResult<Self> {
        validate_config(&config)?;
        Ok(Self {
            config,
            previous: None,
            tracks: Vec::new(),
            active: Vec::new(),
            frame_count: 0,
            accepted_matches: 0,
        })
    }

    /// Adds one image in timestamp order.
    pub fn process(&mut self, image: GrayImage) -> OptimizationResult<()> {
        let pyramid = image_pyramid(image, self.config.pyramid_levels);
        if let Some(previous) = &self.previous {
            let mut next_active = Vec::new();
            for &track_index in &self.active {
                let previous_pixel = self.tracks[track_index]
                    .observations
                    .last()
                    .expect("active track has an observation")
                    .pixel;
                let Some(current_pixel) =
                    track_pyramid(previous, &pyramid, previous_pixel, &self.config)
                else {
                    continue;
                };
                let Some(backward_pixel) =
                    track_pyramid(&pyramid, previous, current_pixel, &self.config)
                else {
                    continue;
                };
                if (backward_pixel - previous_pixel).norm() > self.config.forward_backward_tolerance
                {
                    continue;
                }
                self.tracks[track_index]
                    .observations
                    .push(ImageFeatureObservation {
                        frame: self.frame_count,
                        pixel: current_pixel,
                    });
                next_active.push(track_index);
                self.accepted_matches += 1;
            }
            self.active = next_active;
        }

        let occupied = self
            .active
            .iter()
            .map(|&index| {
                self.tracks[index]
                    .observations
                    .last()
                    .expect("active track has an observation")
                    .pixel
            })
            .collect::<Vec<_>>();
        let capacity = self.config.max_features.saturating_sub(self.active.len());
        let new_points = detect_corners(&pyramid[0], &occupied, capacity, &self.config);
        for pixel in new_points {
            let index = self.tracks.len();
            self.tracks.push(ImageFeatureTrack {
                id: index,
                observations: vec![ImageFeatureObservation {
                    frame: self.frame_count,
                    pixel,
                }],
            });
            self.active.push(index);
        }
        self.previous = Some(pyramid);
        self.frame_count += 1;
        Ok(())
    }

    pub fn frame_count(&self) -> usize {
        self.frame_count
    }

    pub fn accepted_matches(&self) -> usize {
        self.accepted_matches
    }

    pub fn finish(self) -> Vec<ImageFeatureTrack> {
        self.tracks
            .into_iter()
            .filter(|track| track.observations.len() >= self.config.min_track_length)
            .collect()
    }
}

/// Landmark initialization parameters.
#[derive(Debug, Clone, Copy)]
pub struct TriangulationConfig {
    pub min_parallax_radians: f64,
    pub min_depth: f64,
    pub max_reprojection_error: f64,
}

impl Default for TriangulationConfig {
    fn default() -> Self {
        Self {
            min_parallax_radians: 0.005,
            min_depth: 0.1,
            max_reprojection_error: 8.0,
        }
    }
}

/// Triangulates 2D tracks and converts them to the VIO sidecar representation.
pub fn triangulate_tracks(
    tracks: &[ImageFeatureTrack],
    timestamps_ns: &[i64],
    world_from_camera: &[Matrix4<f64>],
    intrinsics: [f64; 4],
    config: TriangulationConfig,
) -> OptimizationResult<EurocFeatureTracks> {
    if timestamps_ns.len() != world_from_camera.len() || timestamps_ns.is_empty() {
        return Err(OptimizationError::InvalidParameter(
            "timestamps and camera trajectory must have the same nonzero length".into(),
        ));
    }
    if config.min_parallax_radians <= 0.0
        || config.min_depth <= 0.0
        || config.max_reprojection_error <= 0.0
    {
        return Err(OptimizationError::InvalidParameter(
            "triangulation thresholds must be positive".into(),
        ));
    }

    let mut landmarks = Vec::new();
    let mut observations = Vec::new();
    for track in tracks {
        if track
            .observations
            .iter()
            .any(|observation| observation.frame >= world_from_camera.len())
        {
            return Err(OptimizationError::InvalidParameter(
                "feature observation frame is out of range".into(),
            ));
        }
        let Some(position) = triangulate_one(track, world_from_camera, intrinsics, config)? else {
            continue;
        };
        let id = landmarks.len();
        landmarks.push(TrackedLandmark { id, position });
        observations.extend(
            track
                .observations
                .iter()
                .map(|observation| FeatureTrackObservation {
                    timestamp_ns: timestamps_ns[observation.frame],
                    landmark_id: id,
                    pixel: observation.pixel,
                }),
        );
    }
    if landmarks.is_empty() {
        return Err(OptimizationError::InvalidParameter(
            "no feature track passed triangulation".into(),
        ));
    }
    Ok(EurocFeatureTracks {
        landmarks,
        observations,
    })
}

fn validate_config(config: &FeatureFrontendConfig) -> OptimizationResult<()> {
    if config.max_features == 0
        || !(0.0..=1.0).contains(&config.quality_level)
        || config.quality_level == 0.0
        || config.min_distance < 1.0
        || config.pyramid_levels == 0
        || config.window_radius < 2
        || config.max_iterations == 0
        || config.convergence_epsilon <= 0.0
        || config.forward_backward_tolerance <= 0.0
        || config.max_patch_rms <= 0.0
        || config.min_track_length < 2
    {
        return Err(OptimizationError::InvalidParameter(
            "invalid visual frontend configuration".into(),
        ));
    }
    Ok(())
}

fn image_pyramid(image: GrayImage, levels: usize) -> Vec<GrayImage> {
    let mut pyramid = vec![image];
    while pyramid.len() < levels {
        let previous = pyramid.last().expect("base image exists");
        if previous.width < 16 || previous.height < 16 {
            break;
        }
        pyramid.push(previous.half_size());
    }
    pyramid
}

fn detect_corners(
    image: &GrayImage,
    occupied: &[Vector2<f64>],
    capacity: usize,
    config: &FeatureFrontendConfig,
) -> Vec<Vector2<f64>> {
    if capacity == 0 {
        return Vec::new();
    }
    let margin = config.window_radius + 2;
    let mut scores = vec![0.0; image.width * image.height];
    let mut maximum: f64 = 0.0;
    for y in margin..image.height - margin {
        for x in margin..image.width - margin {
            let mut xx = 0.0;
            let mut xy = 0.0;
            let mut yy = 0.0;
            for row in y - 1..=y + 1 {
                for column in x - 1..=x + 1 {
                    let gx = image.pixels[row * image.width + column + 1] as f64
                        - image.pixels[row * image.width + column - 1] as f64;
                    let gy = image.pixels[(row + 1) * image.width + column] as f64
                        - image.pixels[(row - 1) * image.width + column] as f64;
                    xx += gx * gx;
                    xy += gx * gy;
                    yy += gy * gy;
                }
            }
            let trace = xx + yy;
            let score = 0.5 * (trace - ((xx - yy).powi(2) + 4.0 * xy * xy).sqrt());
            maximum = maximum.max(score);
            scores[y * image.width + x] = score;
        }
    }
    let threshold = maximum * config.quality_level;
    let mut candidates = Vec::new();
    for y in margin + 1..image.height - margin - 1 {
        for x in margin + 1..image.width - margin - 1 {
            let score = scores[y * image.width + x];
            if score < threshold || score <= 0.0 {
                continue;
            }
            let is_local_maximum = (y - 1..=y + 1).all(|row| {
                (x - 1..=x + 1).all(|column| {
                    (row == y && column == x) || scores[row * image.width + column] <= score
                })
            });
            if is_local_maximum {
                candidates.push((score, Vector2::new(x as f64, y as f64)));
            }
        }
    }
    candidates.sort_by(|left, right| right.0.total_cmp(&left.0));

    let minimum_squared = config.min_distance * config.min_distance;
    let mut selected = occupied.to_vec();
    let existing = selected.len();
    for (_, point) in candidates {
        if selected
            .iter()
            .all(|other| (point - other).norm_squared() >= minimum_squared)
        {
            selected.push(point);
            if selected.len() - existing == capacity {
                break;
            }
        }
    }
    selected.into_iter().skip(existing).collect()
}

fn track_pyramid(
    source: &[GrayImage],
    destination: &[GrayImage],
    source_point: Vector2<f64>,
    config: &FeatureFrontendConfig,
) -> Option<Vector2<f64>> {
    let levels = source.len().min(destination.len());
    let mut destination_point = Vector2::zeros();
    for level in (0..levels).rev() {
        let scale = (1_usize << level) as f64;
        let point = source_point / scale;
        if level == levels - 1 {
            destination_point = point;
        } else {
            destination_point *= 2.0;
        }
        destination_point = track_level(
            &source[level],
            &destination[level],
            point,
            destination_point,
            config,
        )?;
    }
    Some(destination_point)
}

fn track_level(
    source: &GrayImage,
    destination: &GrayImage,
    source_point: Vector2<f64>,
    mut destination_point: Vector2<f64>,
    config: &FeatureFrontendConfig,
) -> Option<Vector2<f64>> {
    let radius = config.window_radius as isize;
    let mut final_squared_error = 0.0;
    let mut count = 0;
    for _ in 0..config.max_iterations {
        let mut hessian = Matrix3::<f64>::zeros();
        let mut gradient = Vector2::<f64>::zeros();
        final_squared_error = 0.0;
        count = 0;
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                let offset = Vector2::new(dx as f64, dy as f64);
                let reference = source.sample(source_point + offset)?;
                let target_point = destination_point + offset;
                let target = destination.sample(target_point)?;
                let gx = 0.5
                    * (destination.sample(target_point + Vector2::new(1.0, 0.0))?
                        - destination.sample(target_point - Vector2::new(1.0, 0.0))?);
                let gy = 0.5
                    * (destination.sample(target_point + Vector2::new(0.0, 1.0))?
                        - destination.sample(target_point - Vector2::new(0.0, 1.0))?);
                let residual = target - reference;
                hessian[(0, 0)] += gx * gx;
                hessian[(0, 1)] += gx * gy;
                hessian[(1, 0)] += gx * gy;
                hessian[(1, 1)] += gy * gy;
                gradient.x += gx * residual;
                gradient.y += gy * residual;
                final_squared_error += residual * residual;
                count += 1;
            }
        }
        let determinant = hessian[(0, 0)] * hessian[(1, 1)] - hessian[(0, 1)].powi(2);
        if determinant < 1.0e-6 {
            return None;
        }
        let delta = Vector2::<f64>::new(
            (-hessian[(1, 1)] * gradient.x + hessian[(0, 1)] * gradient.y) / determinant,
            (hessian[(1, 0)] * gradient.x - hessian[(0, 0)] * gradient.y) / determinant,
        );
        if !delta.iter().all(|value| value.is_finite()) || delta.norm() > 3.0 {
            return None;
        }
        destination_point += delta;
        if delta.norm() < config.convergence_epsilon {
            break;
        }
    }
    let rms = (final_squared_error / count as f64).sqrt();
    (rms <= config.max_patch_rms).then_some(destination_point)
}

fn triangulate_one(
    track: &ImageFeatureTrack,
    poses: &[Matrix4<f64>],
    intrinsics: [f64; 4],
    config: TriangulationConfig,
) -> OptimizationResult<Option<Vector3<f64>>> {
    if track.observations.len() < 2 {
        return Ok(None);
    }
    let [fx, fy, cx, cy] = intrinsics;
    if fx <= 0.0 || fy <= 0.0 {
        return Err(OptimizationError::InvalidParameter(
            "camera focal lengths must be positive".into(),
        ));
    }
    let mut rays = Vec::with_capacity(track.observations.len());
    for observation in &track.observations {
        let pose = &poses[observation.frame];
        let origin = pose.fixed_view::<3, 1>(0, 3).into_owned();
        let camera_ray = Vector3::new(
            (observation.pixel.x - cx) / fx,
            (observation.pixel.y - cy) / fy,
            1.0,
        )
        .normalize();
        let direction = pose.fixed_view::<3, 3>(0, 0) * camera_ray;
        rays.push((origin, direction));
    }
    let maximum_parallax = rays
        .iter()
        .enumerate()
        .flat_map(|(index, left)| rays[index + 1..].iter().map(move |right| (left, right)))
        .map(|(left, right)| left.1.dot(&right.1).clamp(-1.0, 1.0).acos())
        .fold(0.0_f64, f64::max);
    if maximum_parallax < config.min_parallax_radians {
        return Ok(None);
    }

    let mut normal = Matrix3::zeros();
    let mut right_hand_side = Vector3::zeros();
    for (origin, direction) in &rays {
        let projection = Matrix3::identity() - direction * direction.transpose();
        normal += projection;
        right_hand_side += projection * origin;
    }
    let Some(position) = normal.lu().solve(&right_hand_side) else {
        return Ok(None);
    };
    for (observation, pose) in track.observations.iter().zip(
        track
            .observations
            .iter()
            .map(|observation| &poses[observation.frame]),
    ) {
        let camera_from_world_rotation = pose.fixed_view::<3, 3>(0, 0).transpose();
        let camera_point =
            camera_from_world_rotation * (position - pose.fixed_view::<3, 1>(0, 3).into_owned());
        if camera_point.z <= config.min_depth {
            return Ok(None);
        }
        let projected = Vector2::new(
            fx * camera_point.x / camera_point.z + cx,
            fy * camera_point.y / camera_point.z + cy,
        );
        if (projected - observation.pixel).norm() > config.max_reprojection_error {
            return Ok(None);
        }
    }
    Ok(Some(position))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_image(shift_x: usize) -> GrayImage {
        let width = 96;
        let height = 72;
        let mut pixels = vec![20; width * height];
        for &(x, y) in &[(18, 18), (42, 20), (70, 22), (24, 48), (58, 50)] {
            let x = x + shift_x;
            for row in y - 3..=y + 3 {
                for column in x - 3..=x + 3 {
                    pixels[row * width + column] = if (row + column) % 2 == 0 { 240 } else { 80 };
                }
            }
        }
        GrayImage::new(width, height, pixels).unwrap()
    }

    #[test]
    fn pyramidal_tracker_follows_translation() {
        let config = FeatureFrontendConfig {
            min_distance: 8.0,
            min_track_length: 3,
            ..FeatureFrontendConfig::default()
        };
        let mut tracker = FeatureTracker::new(config).unwrap();
        tracker.process(synthetic_image(0)).unwrap();
        tracker.process(synthetic_image(2)).unwrap();
        tracker.process(synthetic_image(4)).unwrap();
        assert!(tracker.accepted_matches() >= 6);
        let tracks = tracker.finish();
        assert!(tracks.len() >= 3);
        for track in tracks {
            let displacement =
                track.observations.last().unwrap().pixel - track.observations[0].pixel;
            assert!((displacement.x - 4.0).abs() < 0.25);
            assert!(displacement.y.abs() < 0.25);
        }
    }

    #[test]
    fn triangulation_recovers_known_point() {
        let intrinsics = [200.0, 200.0, 48.0, 36.0];
        let point = Vector3::new(0.2, -0.1, 4.0);
        let mut poses = vec![Matrix4::identity(); 3];
        poses[1][(0, 3)] = 0.2;
        poses[2][(0, 3)] = 0.4;
        let observations = poses
            .iter()
            .enumerate()
            .map(|(frame, pose)| {
                let camera_point = point - pose.fixed_view::<3, 1>(0, 3).into_owned();
                ImageFeatureObservation {
                    frame,
                    pixel: Vector2::new(
                        intrinsics[0] * camera_point.x / camera_point.z + intrinsics[2],
                        intrinsics[1] * camera_point.y / camera_point.z + intrinsics[3],
                    ),
                }
            })
            .collect();
        let result = triangulate_tracks(
            &[ImageFeatureTrack {
                id: 7,
                observations,
            }],
            &[1, 2, 3],
            &poses,
            intrinsics,
            TriangulationConfig::default(),
        )
        .unwrap();
        assert_eq!(result.landmarks[0].id, 0);
        assert!((result.landmarks[0].position - point).norm() < 1.0e-10);
        assert_eq!(result.observations.len(), 3);
    }
}
