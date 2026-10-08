//! Flags LiDAR points on moving objects (people walking by) so they stay out
//! of scan matching and the map.
//!
//! The filter keeps its own short-term log-odds grid in the SLAM frame. A
//! point that lands in a cell earlier beams have repeatedly seen as free can
//! only be something that moved there: it is flagged dynamic. Walls are hit
//! by every scan and never become free, so they always pass. Something that
//! stops and stays long enough is re-learned as static, which is the right
//! answer for a parked object.

use nalgebra::Vector2;
use rust_robotics_core::Pose2D;

use crate::lidar_occupancy::{OccupancyConfig, OccupancyGrid};
use crate::scan_to_map::transform_scan_to_world;

/// Tuning parameters for [`DynamicPointFilter`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynamicFilterConfig {
    /// Grid cell edge \[m\].
    pub resolution: f64,
    /// A point is dynamic when its cell's log-odds is below this (several
    /// more misses than hits).
    pub free_log_odds: f32,
    /// Also check the neighbors within this many cells, so a point that is
    /// a few centimeters off its cell still finds a static wall it belongs
    /// to.
    pub neighborhood: usize,
}

impl Default for DynamicFilterConfig {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            free_log_odds: -1.0,
            neighborhood: 1,
        }
    }
}

/// Short-term free-space memory that separates moving from static points.
#[derive(Debug, Clone)]
pub struct DynamicPointFilter {
    config: DynamicFilterConfig,
    grid: OccupancyGrid,
}

impl DynamicPointFilter {
    /// A filter covering the world rectangle `[min, max]`.
    pub fn new(min: Vector2<f64>, max: Vector2<f64>, config: DynamicFilterConfig) -> Self {
        let grid = OccupancyGrid::new(
            min,
            max,
            OccupancyConfig {
                resolution: config.resolution,
                // Weak hits: a cell seen free needs many consecutive hits
                // before it counts as static again, so a person standing in
                // it for a moment stays dynamic while a parked object is
                // re-learned. The clamp bounds how long that takes.
                hit: 0.3,
                clamp: 3.0,
                ..OccupancyConfig::default()
            },
        );
        Self { config, grid }
    }

    /// Whether the world point lies in space recently seen as free, with no
    /// static evidence nearby.
    pub fn is_dynamic(&self, point: Vector2<f64>) -> bool {
        let Some((cx, cy)) = self.grid.cell_of(point) else {
            return false;
        };
        let (width, height) = self.grid.size();
        let reach = self.config.neighborhood as i64;
        let mut free = false;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let (x, y) = (cx as i64 + dx, cy as i64 + dy);
                if x < 0 || y < 0 || x as usize >= width || y as usize >= height {
                    continue;
                }
                let log_odds = self.grid.log_odds(x as usize, y as usize);
                if log_odds > 0.0 {
                    // A static surface is right here.
                    return false;
                }
                if dx == 0 && dy == 0 {
                    free = log_odds < self.config.free_log_odds;
                }
            }
        }
        free
    }

    /// Splits a body-frame scan taken at `pose` into static and dynamic
    /// points (both body frame, in scan order), judged against the scans
    /// seen so far.
    pub fn split(
        &self,
        pose: Pose2D,
        scan_body: &[Vector2<f64>],
    ) -> (Vec<Vector2<f64>>, Vec<Vector2<f64>>) {
        let world = transform_scan_to_world(scan_body, pose);
        let mut kept = Vec::with_capacity(scan_body.len());
        let mut dynamic = Vec::new();
        for (body, world) in scan_body.iter().zip(world) {
            if self.is_dynamic(world) {
                dynamic.push(*body);
            } else {
                kept.push(*body);
            }
        }
        (kept, dynamic)
    }

    /// Remembers the free space and hits of a full scan at `pose`.
    pub fn observe(&mut self, pose: Pose2D, scan_body: &[Vector2<f64>]) {
        self.grid.insert_scan(pose, scan_body);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scan_to_map::{ranges_to_points, ray_cast_ranges, LineSegment};

    fn room() -> Vec<LineSegment> {
        let v = Vector2::new;
        vec![
            LineSegment::new(v(-5.0, -3.0), v(5.0, -3.0)),
            LineSegment::new(v(5.0, -3.0), v(5.0, 3.0)),
            LineSegment::new(v(5.0, 3.0), v(-5.0, 3.0)),
            LineSegment::new(v(-5.0, 3.0), v(-5.0, -3.0)),
        ]
    }

    fn person(x: f64, y: f64) -> Vec<LineSegment> {
        let v = Vector2::new;
        let h = 0.2;
        vec![
            LineSegment::new(v(x - h, y - h), v(x + h, y - h)),
            LineSegment::new(v(x + h, y - h), v(x + h, y + h)),
            LineSegment::new(v(x + h, y + h), v(x - h, y + h)),
            LineSegment::new(v(x - h, y + h), v(x - h, y - h)),
        ]
    }

    fn scan(pose: Pose2D, walls: &[LineSegment]) -> Vec<Vector2<f64>> {
        ranges_to_points(&ray_cast_ranges(pose, walls, 360, 12.0))
    }

    #[test]
    fn a_person_walking_into_seen_free_space_is_flagged_walls_are_not() {
        let mut filter = DynamicPointFilter::new(
            Vector2::new(-6.0, -4.0),
            Vector2::new(6.0, 4.0),
            DynamicFilterConfig::default(),
        );
        let pose = Pose2D::new(-3.0, 0.0, 0.0);
        // The empty room, seen a few times.
        for _ in 0..4 {
            filter.observe(pose, &scan(pose, &room()));
        }
        // Someone walks in.
        let mut walls = room();
        walls.extend(person(1.0, 0.5));
        let (kept, dynamic) = filter.split(pose, &scan(pose, &walls));
        assert!(!dynamic.is_empty(), "the person was not flagged");
        for point in transform_scan_to_world(&dynamic, pose) {
            assert!(
                (point - Vector2::new(1.0, 0.5)).norm() < 0.4,
                "flagged a non-person point {point:?}"
            );
        }
        // Every wall point is kept.
        let walls_seen = transform_scan_to_world(&kept, pose)
            .into_iter()
            .filter(|p| p.x.abs() > 4.8 || p.y.abs() > 2.8)
            .count();
        let walls_total = scan(pose, &walls)
            .into_iter()
            .map(|p| p + Vector2::new(pose.x, pose.y))
            .filter(|p| p.x.abs() > 4.8 || p.y.abs() > 2.8)
            .count();
        assert_eq!(walls_seen, walls_total);
    }

    #[test]
    fn something_that_stays_becomes_static() {
        let mut filter = DynamicPointFilter::new(
            Vector2::new(-6.0, -4.0),
            Vector2::new(6.0, 4.0),
            DynamicFilterConfig::default(),
        );
        let pose = Pose2D::new(-3.0, 0.0, 0.0);
        for _ in 0..4 {
            filter.observe(pose, &scan(pose, &room()));
        }
        let mut walls = room();
        walls.extend(person(1.0, 0.5));
        let parked = scan(pose, &walls);
        for _ in 0..10 {
            filter.observe(pose, &parked);
        }
        let (_, dynamic) = filter.split(pose, &parked);
        assert!(dynamic.is_empty(), "{} points still dynamic", dynamic.len());
    }
}
