#![allow(dead_code, clippy::too_many_arguments)]

//! Potential Field path planning algorithm
//!
//! Uses attractive and repulsive potential fields to guide
//! the robot from start to goal while avoiding obstacles.

use std::collections::VecDeque;

use rust_robotics_core::{Path2D, Point2D, RoboticsError, RoboticsResult};

// Parameters
const KP: f64 = 5.0;
const ETA: f64 = 100.0;
const AREA_WIDTH: f64 = 30.0;
const OSCILLATIONS_DETECTION_LENGTH: usize = 3;

pub struct PotentialFieldPlanner {
    pub resolution: f64,
    pub robot_radius: f64,
}

impl PotentialFieldPlanner {
    pub fn new(resolution: f64, robot_radius: f64) -> Self {
        PotentialFieldPlanner {
            resolution,
            robot_radius,
        }
    }

    pub fn planning(
        &self,
        sx: f64,
        sy: f64,
        gx: f64,
        gy: f64,
        ox: &[f64],
        oy: &[f64],
    ) -> Option<(Vec<f64>, Vec<f64>)> {
        let (pmap, minx, miny) = self.calc_potential_field(gx, gy, ox, oy, sx, sy);

        let mut d = ((sx - gx).powi(2) + (sy - gy).powi(2)).sqrt();
        let mut ix = ((sx - minx) / self.resolution).round() as i32;
        let mut iy = ((sy - miny) / self.resolution).round() as i32;

        let mut rx = vec![sx];
        let mut ry = vec![sy];
        let motion = self.get_motion_model();
        let mut previous_ids = VecDeque::new();

        while d >= self.resolution {
            let mut minp = f64::INFINITY;
            let mut minix = -1;
            let mut miniy = -1;

            for motion_step in &motion {
                let inx = ix + motion_step[0];
                let iny = iy + motion_step[1];

                let p = if inx >= pmap.len() as i32
                    || iny >= pmap[0].len() as i32
                    || inx < 0
                    || iny < 0
                {
                    f64::INFINITY
                } else {
                    pmap[inx as usize][iny as usize]
                };

                if minp > p {
                    minp = p;
                    minix = inx;
                    miniy = iny;
                }
            }

            ix = minix;
            iy = miniy;
            let xp = ix as f64 * self.resolution + minx;
            let yp = iy as f64 * self.resolution + miny;
            d = ((gx - xp).powi(2) + (gy - yp).powi(2)).sqrt();
            rx.push(xp);
            ry.push(yp);

            if self.oscillations_detection(&mut previous_ids, ix, iy) {
                break;
            }
        }

        Some((rx, ry))
    }

    fn calc_potential_field(
        &self,
        gx: f64,
        gy: f64,
        ox: &[f64],
        oy: &[f64],
        sx: f64,
        sy: f64,
    ) -> (Vec<Vec<f64>>, f64, f64) {
        let minx = [ox.iter().fold(f64::INFINITY, |a, &b| a.min(b)), sx, gx]
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b))
            - AREA_WIDTH / 2.0;
        let miny = [oy.iter().fold(f64::INFINITY, |a, &b| a.min(b)), sy, gy]
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b))
            - AREA_WIDTH / 2.0;
        let maxx = [ox.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)), sx, gx]
            .iter()
            .fold(f64::NEG_INFINITY, |a, &b| a.max(b))
            + AREA_WIDTH / 2.0;
        let maxy = [oy.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)), sy, gy]
            .iter()
            .fold(f64::NEG_INFINITY, |a, &b| a.max(b))
            + AREA_WIDTH / 2.0;

        let xw = ((maxx - minx) / self.resolution).round() as usize;
        let yw = ((maxy - miny) / self.resolution).round() as usize;

        let mut pmap = vec![vec![0.0; yw]; xw];

        for (ix, column) in pmap.iter_mut().enumerate().take(xw) {
            let x = ix as f64 * self.resolution + minx;
            for (iy, value) in column.iter_mut().enumerate().take(yw) {
                let y = iy as f64 * self.resolution + miny;
                let ug = self.calc_attractive_potential(x, y, gx, gy);
                let uo = self.calc_repulsive_potential(x, y, ox, oy);
                let uf = ug + uo;
                *value = uf;
            }
        }

        (pmap, minx, miny)
    }

    fn calc_attractive_potential(&self, x: f64, y: f64, gx: f64, gy: f64) -> f64 {
        0.5 * KP * ((x - gx).powi(2) + (y - gy).powi(2)).sqrt()
    }

    fn calc_repulsive_potential(&self, x: f64, y: f64, ox: &[f64], oy: &[f64]) -> f64 {
        let mut dmin = f64::INFINITY;
        let mut minid = 0;

        for (i, (&ox_i, &oy_i)) in ox.iter().zip(oy.iter()).enumerate() {
            let d = ((x - ox_i).powi(2) + (y - oy_i).powi(2)).sqrt();
            if dmin >= d {
                dmin = d;
                minid = i;
            }
        }

        let dq = ((x - ox[minid]).powi(2) + (y - oy[minid]).powi(2)).sqrt();

        if dq <= self.robot_radius {
            let dq = if dq <= 0.1 { 0.1 } else { dq };
            0.5 * ETA * (1.0 / dq - 1.0 / self.robot_radius).powi(2)
        } else {
            0.0
        }
    }

    fn get_motion_model(&self) -> Vec<[i32; 2]> {
        vec![
            [1, 0],
            [0, 1],
            [-1, 0],
            [0, -1],
            [-1, -1],
            [-1, 1],
            [1, -1],
            [1, 1],
        ]
    }

    /// Plan a path from start to goal while avoiding obstacles, returning a [`Path2D`].
    ///
    /// This is a convenience wrapper around [`planning()`](Self::planning) that accepts
    /// [`Point2D`] and returns [`Path2D`].
    pub fn plan_with_obstacles(
        &self,
        start: Point2D,
        goal: Point2D,
        ox: &[f64],
        oy: &[f64],
    ) -> RoboticsResult<Path2D> {
        self.planning(start.x, start.y, goal.x, goal.y, ox, oy)
            .map(|(rx, ry)| Path2D::from_xy(&rx, &ry))
            .ok_or_else(|| {
                RoboticsError::PlanningError(
                    "PotentialField: oscillation detected, no path found".to_string(),
                )
            })
    }

    fn oscillations_detection(
        &self,
        previous_ids: &mut VecDeque<(i32, i32)>,
        ix: i32,
        iy: i32,
    ) -> bool {
        previous_ids.push_back((ix, iy));

        if previous_ids.len() > OSCILLATIONS_DETECTION_LENGTH {
            previous_ids.pop_front();
        }

        let mut seen = std::collections::HashSet::new();
        for &index in previous_ids.iter() {
            if seen.contains(&index) {
                return true;
            }
            seen.insert(index);
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_potential_field_creation() {
        let planner = PotentialFieldPlanner::new(0.5, 5.0);
        assert_eq!(planner.resolution, 0.5);
        assert_eq!(planner.robot_radius, 5.0);
    }
}
