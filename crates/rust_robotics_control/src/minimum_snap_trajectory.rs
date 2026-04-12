//! Minimum-snap 3D trajectory generation for drone waypoint tracking.
//!
//! This module builds piecewise seventh-order polynomial segments with
//! position, velocity, acceleration, and jerk boundary constraints. The
//! generated trajectory can be sampled into [`DesiredState`] values and fed
//! into the existing quadrotor PD tracker.

use nalgebra::{SMatrix, SVector, Vector3};

use crate::drone_3d_trajectory::{
    simulate_desired_states, DesiredState, SimulationConfig, SimulationRecord,
};

type Matrix8 = SMatrix<f64, 8, 8>;
type Vector8 = SVector<f64, 8>;

/// Coefficients of a seventh-order polynomial:
/// `c[0] + c[1] * t + ... + c[7] * t^7`.
#[derive(Debug, Clone)]
pub struct MinimumSnapCoeffs {
    pub c: [f64; 8],
}

impl MinimumSnapCoeffs {
    fn evaluate_derivative(&self, t: f64, order: usize) -> f64 {
        self.c
            .iter()
            .enumerate()
            .skip(order)
            .map(|(power, coeff)| {
                coeff * derivative_factor(power, order) * t.powi((power - order) as i32)
            })
            .sum()
    }

    pub fn position(&self, t: f64) -> f64 {
        self.evaluate_derivative(t, 0)
    }

    pub fn velocity(&self, t: f64) -> f64 {
        self.evaluate_derivative(t, 1)
    }

    pub fn acceleration(&self, t: f64) -> f64 {
        self.evaluate_derivative(t, 2)
    }

    pub fn jerk(&self, t: f64) -> f64 {
        self.evaluate_derivative(t, 3)
    }

    pub fn snap(&self, t: f64) -> f64 {
        self.evaluate_derivative(t, 4)
    }
}

fn derivative_factor(power: usize, order: usize) -> f64 {
    ((power - order + 1)..=power).fold(1.0, |acc, value| acc * value as f64)
}

#[derive(Debug, Clone)]
struct AxisBoundary {
    start_pos: f64,
    end_pos: f64,
    start_vel: f64,
    end_vel: f64,
    start_acc: f64,
    end_acc: f64,
    start_jerk: f64,
    end_jerk: f64,
}

/// Full boundary conditions for one 3D minimum-snap endpoint.
#[derive(Debug, Clone)]
pub struct MinimumSnapBoundary {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub jerk: Vector3<f64>,
}

impl MinimumSnapBoundary {
    pub fn at_rest(position: Vector3<f64>) -> Self {
        Self {
            position,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            jerk: Vector3::zeros(),
        }
    }
}

/// A piecewise minimum-snap segment for x/y/z.
#[derive(Debug, Clone)]
pub struct MinimumSnapSegment {
    pub x: MinimumSnapCoeffs,
    pub y: MinimumSnapCoeffs,
    pub z: MinimumSnapCoeffs,
    pub duration: f64,
}

impl MinimumSnapSegment {
    pub fn desired_state(&self, t: f64) -> DesiredState {
        let t = t.clamp(0.0, self.duration);
        DesiredState {
            position: Vector3::new(self.x.position(t), self.y.position(t), self.z.position(t)),
            velocity: Vector3::new(self.x.velocity(t), self.y.velocity(t), self.z.velocity(t)),
            acceleration: Vector3::new(
                self.x.acceleration(t),
                self.y.acceleration(t),
                self.z.acceleration(t),
            ),
            yaw: 0.0,
        }
    }

    pub fn jerk(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, self.duration);
        Vector3::new(self.x.jerk(t), self.y.jerk(t), self.z.jerk(t))
    }

    pub fn snap(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, self.duration);
        Vector3::new(self.x.snap(t), self.y.snap(t), self.z.snap(t))
    }
}

fn solve_minimum_snap_axis(boundary: &AxisBoundary, duration: f64) -> MinimumSnapCoeffs {
    let t = duration;
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t3 * t;
    let t5 = t4 * t;
    let t6 = t5 * t;
    let t7 = t6 * t;

    #[rustfmt::skip]
    let a = Matrix8::from_row_slice(&[
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, t,   t2,  t3,  t4,  t5,  t6,  t7,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 2.0 * t, 3.0 * t2, 4.0 * t3, 5.0 * t4, 6.0 * t5, 7.0 * t6,
        0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 2.0, 6.0 * t, 12.0 * t2, 20.0 * t3, 30.0 * t4, 42.0 * t5,
        0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * t2, 120.0 * t3, 210.0 * t4,
    ]);

    let rhs = Vector8::from_row_slice(&[
        boundary.start_pos,
        boundary.end_pos,
        boundary.start_vel,
        boundary.end_vel,
        boundary.start_acc,
        boundary.end_acc,
        boundary.start_jerk,
        boundary.end_jerk,
    ]);

    let coeffs = a
        .lu()
        .solve(&rhs)
        .expect("Minimum-snap matrix should be invertible");

    MinimumSnapCoeffs {
        c: [
            coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7],
        ],
    }
}

pub fn generate_minimum_snap_segment(
    start: &Vector3<f64>,
    end: &Vector3<f64>,
    duration: f64,
) -> MinimumSnapSegment {
    generate_minimum_snap_segment_full(
        &MinimumSnapBoundary::at_rest(*start),
        &MinimumSnapBoundary::at_rest(*end),
        duration,
    )
}

pub fn generate_minimum_snap_segment_full(
    start: &MinimumSnapBoundary,
    end: &MinimumSnapBoundary,
    duration: f64,
) -> MinimumSnapSegment {
    let solve = |i: usize| {
        solve_minimum_snap_axis(
            &AxisBoundary {
                start_pos: start.position[i],
                end_pos: end.position[i],
                start_vel: start.velocity[i],
                end_vel: end.velocity[i],
                start_acc: start.acceleration[i],
                end_acc: end.acceleration[i],
                start_jerk: start.jerk[i],
                end_jerk: end.jerk[i],
            },
            duration,
        )
    };

    MinimumSnapSegment {
        x: solve(0),
        y: solve(1),
        z: solve(2),
        duration,
    }
}

pub fn generate_waypoint_minimum_snap_trajectory(
    waypoints: &[Vector3<f64>],
    segment_duration: f64,
) -> Vec<MinimumSnapSegment> {
    let segment_durations = vec![segment_duration; waypoints.len()];
    generate_waypoint_minimum_snap_trajectory_with_durations(waypoints, &segment_durations)
}

pub fn generate_waypoint_minimum_snap_trajectory_with_durations(
    waypoints: &[Vector3<f64>],
    segment_durations: &[f64],
) -> Vec<MinimumSnapSegment> {
    let n = waypoints.len();
    assert!(n >= 2, "Need at least 2 waypoints");
    assert_eq!(
        n,
        segment_durations.len(),
        "Need one segment duration per waypoint-to-waypoint segment"
    );

    (0..n)
        .map(|i| {
            generate_minimum_snap_segment(
                &waypoints[i],
                &waypoints[(i + 1) % n],
                segment_durations[i],
            )
        })
        .collect()
}

pub fn sample_minimum_snap_trajectory(
    segments: &[MinimumSnapSegment],
    dt: f64,
) -> Vec<DesiredState> {
    assert!(dt > 0.0, "dt must be positive");

    let mut samples = Vec::new();
    for segment in segments {
        let mut t = 0.0;
        while t <= segment.duration {
            samples.push(segment.desired_state(t));
            t += dt;
        }
    }
    samples
}

pub fn simulate_minimum_snap_following(
    segments: &[MinimumSnapSegment],
    config: &SimulationConfig,
    start_position: Vector3<f64>,
) -> SimulationRecord {
    let desired_states = sample_minimum_snap_trajectory(segments, config.dt);
    simulate_desired_states(start_position, &desired_states, config)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    #[test]
    fn test_minimum_snap_boundary_conditions() {
        let start = Vector3::new(0.0, 0.0, 1.0);
        let end = Vector3::new(5.0, -2.0, 3.0);
        let seg = generate_minimum_snap_segment(&start, &end, 4.0);

        assert!((seg.x.position(0.0) - start.x).abs() < EPSILON);
        assert!((seg.y.position(4.0) - end.y).abs() < EPSILON);
        assert!(seg.x.velocity(0.0).abs() < EPSILON);
        assert!(seg.x.velocity(4.0).abs() < EPSILON);
        assert!(seg.x.acceleration(0.0).abs() < EPSILON);
        assert!(seg.x.acceleration(4.0).abs() < EPSILON);
        assert!(seg.x.jerk(0.0).abs() < EPSILON);
        assert!(seg.x.jerk(4.0).abs() < EPSILON);
    }

    #[test]
    fn test_waypoint_minimum_snap_generation_with_durations() {
        let waypoints = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 1.0, 2.0),
            Vector3::new(2.0, 3.0, 1.0),
        ];
        let segments =
            generate_waypoint_minimum_snap_trajectory_with_durations(&waypoints, &[2.0, 3.0, 4.0]);

        assert_eq!(segments.len(), 3);
        assert!((segments[0].x.position(2.0) - 4.0).abs() < EPSILON);
        assert!((segments[1].y.position(3.0) - 3.0).abs() < EPSILON);
        assert!((segments[2].z.position(4.0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_sample_minimum_snap_trajectory() {
        let waypoints = vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0)];
        let segments = generate_waypoint_minimum_snap_trajectory(&waypoints, 1.0);
        let samples = sample_minimum_snap_trajectory(&segments, 0.5);

        assert_eq!(samples.len(), 6);
        assert!((samples[0].position.x - 0.0).abs() < EPSILON);
        assert!((samples[2].position.x - 2.0).abs() < 1e-6);
    }
}
