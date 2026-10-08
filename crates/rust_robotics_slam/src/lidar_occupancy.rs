//! Log-odds occupancy grid built from LiDAR scans at known poses, e.g. the
//! node scans of [`crate::lidar_graph_slam::LidarGraphSlam`] at their
//! optimized poses.
//!
//! Every beam marks the cells it passes through as free and its end cell as
//! occupied (Bresenham ray tracing). The grid answers occupancy queries for
//! planning and provides a distance-to-nearest-obstacle field for scan
//! likelihoods ([`OccupancyGrid::distance_field`]).

use nalgebra::Vector2;
use rust_robotics_core::Pose2D;

use crate::scan_to_map::{beam_angle, transform_scan_to_world};

const MISS: u8 = 1;
const HIT: u8 = 2;

/// The cells one scan observed; a hit overrides a miss.
struct ScanUpdate {
    width: usize,
    marks: Vec<u8>,
    touched: Vec<usize>,
}

impl ScanUpdate {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            marks: vec![0; width * height],
            touched: Vec::new(),
        }
    }

    fn mark(&mut self, (x, y): (usize, usize), mark: u8) {
        let index = y * self.width + x;
        if self.marks[index] == 0 {
            self.touched.push(index);
        }
        self.marks[index] = self.marks[index].max(mark);
    }
}

/// Log-odds increments and thresholds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OccupancyConfig {
    /// Cell edge \[m\].
    pub resolution: f64,
    /// Log-odds added to the end cell of a beam.
    pub hit: f32,
    /// Log-odds added to cells a beam passes through (negative).
    pub miss: f32,
    /// Log-odds clamp, so a cell can still change its mind.
    pub clamp: f32,
    /// Cells above this log-odds are occupied.
    pub occupied_threshold: f32,
    /// Cells below this log-odds are free.
    pub free_threshold: f32,
}

impl Default for OccupancyConfig {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            hit: 0.9,
            miss: -0.4,
            clamp: 5.0,
            occupied_threshold: 0.6,
            free_threshold: -0.3,
        }
    }
}

/// State of one cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellState {
    Unknown,
    Free,
    Occupied,
}

/// A fixed-size log-odds occupancy grid over an axis-aligned world rectangle.
#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    config: OccupancyConfig,
    origin: Vector2<f64>,
    width: usize,
    height: usize,
    log_odds: Vec<f32>,
}

impl OccupancyGrid {
    /// Creates an all-unknown grid covering `[min, max]` (world, meters).
    pub fn new(min: Vector2<f64>, max: Vector2<f64>, config: OccupancyConfig) -> Self {
        let resolution = config.resolution.max(1.0e-3);
        let width = (((max.x - min.x) / resolution).ceil() as usize).max(1);
        let height = (((max.y - min.y) / resolution).ceil() as usize).max(1);
        Self {
            config: OccupancyConfig {
                resolution,
                ..config
            },
            origin: min,
            width,
            height,
            log_odds: vec![0.0; width * height],
        }
    }

    /// Builds a grid from body-frame scans taken at `poses`, sized to fit
    /// every pose and scan point plus `margin` meters.
    pub fn from_scans<'a>(
        scans: impl IntoIterator<Item = (Pose2D, &'a [Vector2<f64>])> + Clone,
        margin: f64,
        config: OccupancyConfig,
    ) -> Self {
        let mut min = Vector2::new(f64::INFINITY, f64::INFINITY);
        let mut max = Vector2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for (pose, scan) in scans.clone() {
            let origin = Vector2::new(pose.x, pose.y);
            min = min.inf(&origin);
            max = max.sup(&origin);
            for point in transform_scan_to_world(scan, pose) {
                min = min.inf(&point);
                max = max.sup(&point);
            }
        }
        if !min.x.is_finite() {
            min = Vector2::zeros();
            max = Vector2::zeros();
        }
        let margin = Vector2::new(margin, margin);
        let mut grid = Self::new(min - margin, max + margin, config);
        for (pose, scan) in scans {
            grid.insert_scan(pose, scan);
        }
        grid
    }

    /// Cell edge \[m\].
    pub fn resolution(&self) -> f64 {
        self.config.resolution
    }

    /// `(width, height)` in cells.
    pub fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// World position of the grid corner (cell `(0, 0)`'s lower-left corner).
    pub fn origin(&self) -> Vector2<f64> {
        self.origin
    }

    /// Cell containing `point`, if inside the grid.
    pub fn cell_of(&self, point: Vector2<f64>) -> Option<(usize, usize)> {
        let x = ((point.x - self.origin.x) / self.config.resolution).floor();
        let y = ((point.y - self.origin.y) / self.config.resolution).floor();
        (x >= 0.0 && y >= 0.0 && (x as usize) < self.width && (y as usize) < self.height)
            .then_some((x as usize, y as usize))
    }

    /// World center of cell `(x, y)`.
    pub fn cell_center(&self, x: usize, y: usize) -> Vector2<f64> {
        self.origin
            + Vector2::new(
                (x as f64 + 0.5) * self.config.resolution,
                (y as f64 + 0.5) * self.config.resolution,
            )
    }

    /// State of cell `(x, y)`.
    pub fn state(&self, x: usize, y: usize) -> CellState {
        let value = self.log_odds[y * self.width + x];
        if value > self.config.occupied_threshold {
            CellState::Occupied
        } else if value < self.config.free_threshold {
            CellState::Free
        } else {
            CellState::Unknown
        }
    }

    /// State of the cell containing `point` (`Unknown` outside the grid).
    pub fn state_at(&self, point: Vector2<f64>) -> CellState {
        self.cell_of(point)
            .map_or(CellState::Unknown, |(x, y)| self.state(x, y))
    }

    /// World centers of every occupied cell.
    pub fn occupied_points(&self) -> Vec<Vector2<f64>> {
        (0..self.height)
            .flat_map(|y| (0..self.width).map(move |x| (x, y)))
            .filter(|&(x, y)| self.state(x, y) == CellState::Occupied)
            .map(|(x, y)| self.cell_center(x, y))
            .collect()
    }

    /// Integrates one body-frame scan taken at `pose`. Each cell is updated
    /// at most once per scan, and a hit wins over a miss, so a wall cell is
    /// not erased by neighboring beams that graze it.
    pub fn insert_scan(&mut self, pose: Pose2D, scan_body: &[Vector2<f64>]) {
        let Some(start) = self.cell_of(Vector2::new(pose.x, pose.y)) else {
            return;
        };
        let mut update = ScanUpdate::new(self.width, self.height);
        for point in transform_scan_to_world(scan_body, pose) {
            let Some(end) = self.cell_of(point) else {
                continue;
            };
            self.walk(&mut update, start, end);
            update.mark(end, HIT);
        }
        self.apply(update);
    }

    /// Integrates a full 360° range scan taken at `pose` (beam order of
    /// [`crate::scan_to_map::ray_cast_ranges`]). Beams without a return
    /// (non-finite or beyond `max_range`) still clear the cells out to
    /// `max_range`, so open space becomes known free instead of unknown, and
    /// free-only sub-rays fill the angular gaps between beams so far cells are
    /// not left as unknown speckles (which would look like frontiers).
    pub fn insert_ranges(&mut self, pose: Pose2D, ranges: &[f64], max_range: f64) {
        let Some(start) = self.cell_of(Vector2::new(pose.x, pose.y)) else {
            return;
        };
        let count = ranges.len();
        let spacing = std::f64::consts::TAU / count.max(1) as f64;
        // Half-cell ray spacing at max range: 8-connected lines one cell
        // apart can leave moiré gaps between them.
        let sub_rays =
            ((2.0 * max_range * spacing / self.config.resolution).ceil() as usize).max(1);
        let mut update = ScanUpdate::new(self.width, self.height);
        for (beam, &range) in ranges.iter().enumerate() {
            let hit = range.is_finite() && range <= max_range;
            let reach = if hit { range } else { max_range };
            let center = pose.yaw + beam_angle(beam, count);
            for sub in 0..sub_rays {
                // Sub-rays span the half-open sector around the beam.
                let angle = center + spacing * ((sub as f64 + 0.5) / sub_rays as f64 - 0.5);
                let main = sub == sub_rays / 2;
                // Off-center sub-rays stop a cell short of a wall they may
                // not actually reach, and never mark it occupied.
                let length = if hit && !main {
                    (reach - self.config.resolution).max(0.0)
                } else {
                    reach
                };
                let Some(end) = self.last_cell_along(pose, angle, length) else {
                    continue;
                };
                let ends_inside = self.cell_of(Vector2::new(
                    pose.x + length * angle.cos(),
                    pose.y + length * angle.sin(),
                )) == Some(end);
                self.walk(&mut update, start, end);
                update.mark(
                    end,
                    if hit && main && ends_inside {
                        HIT
                    } else {
                        MISS
                    },
                );
            }
        }
        self.apply(update);
    }

    /// The last grid cell along a ray of `length` from `pose` at `angle`.
    fn last_cell_along(&self, pose: Pose2D, angle: f64, length: f64) -> Option<(usize, usize)> {
        let end = Vector2::new(pose.x + length * angle.cos(), pose.y + length * angle.sin());
        if let Some(cell) = self.cell_of(end) {
            return Some(cell);
        }
        let steps = (length / self.config.resolution).ceil() as usize;
        (0..=steps).rev().find_map(|step| {
            let t = length * step as f64 / steps.max(1) as f64;
            self.cell_of(Vector2::new(
                pose.x + t * angle.cos(),
                pose.y + t * angle.sin(),
            ))
        })
    }

    /// Applies one scan's marks: +hit on hit cells, +miss on the others.
    fn apply(&mut self, update: ScanUpdate) {
        let clamp = self.config.clamp;
        for index in update.touched {
            let delta = if update.marks[index] == HIT {
                self.config.hit
            } else {
                self.config.miss
            };
            let cell = &mut self.log_odds[index];
            *cell = (*cell + delta).clamp(-clamp, clamp);
        }
    }

    /// Marks every cell from `start` up to (not including) `end` as free.
    fn walk(&self, update: &mut ScanUpdate, start: (usize, usize), end: (usize, usize)) {
        let (mut x, mut y) = (start.0 as i64, start.1 as i64);
        let (x1, y1) = (end.0 as i64, end.1 as i64);
        let (dx, dy) = ((x1 - x).abs(), -(y1 - y).abs());
        let (sx, sy) = (if x < x1 { 1 } else { -1 }, if y < y1 { 1 } else { -1 });
        let mut error = dx + dy;
        while (x, y) != (x1, y1) {
            update.mark((x as usize, y as usize), MISS);
            let doubled = 2 * error;
            if doubled >= dy {
                error += dy;
                x += sx;
            }
            if doubled <= dx {
                error += dx;
                y += sy;
            }
        }
    }

    /// Distance \[m\] from every cell center to the nearest occupied cell
    /// center (two-pass chamfer approximation, exact along axes and
    /// diagonals), capped at `max_distance`. Row-major, `width × height`.
    pub fn distance_field(&self, max_distance: f64) -> Vec<f32> {
        let resolution = self.config.resolution as f32;
        let cap = max_distance as f32;
        let mut field: Vec<f32> = (0..self.width * self.height)
            .map(|index| {
                let (x, y) = (index % self.width, index / self.width);
                if self.state(x, y) == CellState::Occupied {
                    0.0
                } else {
                    cap
                }
            })
            .collect();
        let straight = resolution;
        let diagonal = resolution * std::f32::consts::SQRT_2;
        let (w, h) = (self.width as i64, self.height as i64);
        let relax = |field: &mut [f32], x: i64, y: i64, offsets: &[(i64, i64, f32)]| {
            let index = (y * w + x) as usize;
            for &(dx, dy, cost) in offsets {
                let (nx, ny) = (x + dx, y + dy);
                if nx >= 0 && ny >= 0 && nx < w && ny < h {
                    let candidate = field[(ny * w + nx) as usize] + cost;
                    if candidate < field[index] {
                        field[index] = candidate;
                    }
                }
            }
        };
        let forward = [
            (-1, 0, straight),
            (0, -1, straight),
            (-1, -1, diagonal),
            (1, -1, diagonal),
        ];
        let backward = [
            (1, 0, straight),
            (0, 1, straight),
            (1, 1, diagonal),
            (-1, 1, diagonal),
        ];
        for y in 0..h {
            for x in 0..w {
                relax(&mut field, x, y, &forward);
            }
        }
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                relax(&mut field, x, y, &backward);
            }
        }
        for value in &mut field {
            *value = value.min(cap);
        }
        field
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scan_to_map::{ranges_to_points, ray_cast_ranges, LineSegment};

    fn room() -> Vec<LineSegment> {
        let c = [
            Vector2::new(-3.0, -2.0),
            Vector2::new(3.0, -2.0),
            Vector2::new(3.0, 2.0),
            Vector2::new(-3.0, 2.0),
        ];
        (0..4)
            .map(|i| LineSegment::new(c[i], c[(i + 1) % 4]))
            .collect()
    }

    fn room_grid() -> OccupancyGrid {
        let pose = Pose2D::new(0.0, 0.0, 0.3);
        let scan = ranges_to_points(&ray_cast_ranges(pose, &room(), 360, 10.0));
        OccupancyGrid::from_scans([(pose, scan.as_slice())], 0.5, OccupancyConfig::default())
    }

    #[test]
    fn walls_are_occupied_inside_is_free_outside_is_unknown() {
        let grid = room_grid();
        assert_eq!(grid.state_at(Vector2::new(3.0, 0.5)), CellState::Occupied);
        assert_eq!(grid.state_at(Vector2::new(-1.0, 2.0)), CellState::Occupied);
        assert_eq!(grid.state_at(Vector2::new(1.0, 0.5)), CellState::Free);
        assert_eq!(grid.state_at(Vector2::new(0.0, 0.0)), CellState::Free);
        assert_eq!(grid.state_at(Vector2::new(3.4, 0.0)), CellState::Unknown);
        assert_eq!(grid.state_at(Vector2::new(50.0, 0.0)), CellState::Unknown);
    }

    #[test]
    fn occupied_points_lie_on_the_walls() {
        let grid = room_grid();
        let points = grid.occupied_points();
        assert!(points.len() > 100);
        for point in points {
            let to_wall = (3.0 - point.x.abs()).min(2.0 - point.y.abs()).abs();
            assert!(to_wall < 0.15, "{point:?}");
        }
    }

    #[test]
    fn beams_without_a_return_clear_open_space() {
        // Only the wall at x = 3 is in range; every other beam has no return.
        let wall = [LineSegment::new(
            Vector2::new(3.0, -1.0),
            Vector2::new(3.0, 1.0),
        )];
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let ranges = ray_cast_ranges(pose, &wall, 360, 5.0);
        let mut grid = OccupancyGrid::new(
            Vector2::new(-6.0, -6.0),
            Vector2::new(6.0, 6.0),
            OccupancyConfig::default(),
        );
        grid.insert_ranges(pose, &ranges, 5.0);
        // The wall lies on a cell boundary; its hits land on either side.
        let wall_cells = [Vector2::new(2.95, 0.35), Vector2::new(3.05, 0.35)];
        assert!(wall_cells
            .iter()
            .any(|point| grid.state_at(*point) == CellState::Occupied));
        assert_eq!(grid.state_at(Vector2::new(-4.0, 0.0)), CellState::Free);
        assert_eq!(grid.state_at(Vector2::new(0.0, 4.5)), CellState::Free);
        assert_eq!(grid.state_at(Vector2::new(-5.5, 0.0)), CellState::Unknown);
        assert_eq!(grid.state_at(Vector2::new(4.0, 0.0)), CellState::Unknown);
    }

    #[test]
    fn distance_field_grows_away_from_walls() {
        let grid = room_grid();
        let field = grid.distance_field(2.0);
        let (w, _) = grid.size();
        let at = |p: Vector2<f64>| {
            let (x, y) = grid.cell_of(p).unwrap();
            field[y * w + x]
        };
        assert_eq!(at(Vector2::new(3.0, 0.5)), 0.0);
        let one_meter = at(Vector2::new(2.0, 0.0));
        assert!((one_meter - 1.0).abs() < 0.15, "{one_meter}");
        assert!(at(Vector2::new(0.0, 0.0)) > 1.8);
    }
}
