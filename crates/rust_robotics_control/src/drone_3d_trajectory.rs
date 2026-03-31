//! 3D drone trajectory following with quadrotor dynamics.
//!
//! Implements a quadrotor dynamics model and a PD trajectory tracking controller
//! that follows quintic polynomial trajectories through 3D waypoints.
//!
//! Based on PythonRobotics `AerialNavigation/drone_3d_trajectory_following`.
//!
//! # Overview
//!
//! 1. **Quintic trajectory generation**: Given start/end positions (with velocity and
//!    acceleration boundary conditions), solve for 6 polynomial coefficients per axis
//!    so that position, velocity, and acceleration are continuous at segment boundaries.
//!
//! 2. **PD controller**: Computes thrust and torques from the tracking error between
//!    desired and actual states.
//!
//! 3. **Quadrotor dynamics**: Euler-integrated rigid body with roll/pitch/yaw, driven
//!    by thrust and body torques.

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

/// Gravity acceleration \[m/s^2\].
const GRAVITY: f64 = 9.81;

// ---------------------------------------------------------------------------
// Quintic trajectory generation
// ---------------------------------------------------------------------------

/// Coefficients of a quintic polynomial for one axis: `c0*t^5 + c1*t^4 + ... + c5`.
#[derive(Debug, Clone)]
pub struct QuinticCoeffs {
    /// Coefficients `[c0, c1, c2, c3, c4, c5]` (highest power first).
    pub c: [f64; 6],
}

impl QuinticCoeffs {
    /// Evaluate position at time `t`.
    pub fn position(&self, t: f64) -> f64 {
        let c = &self.c;
        c[0] * t.powi(5) + c[1] * t.powi(4) + c[2] * t.powi(3) + c[3] * t.powi(2) + c[4] * t + c[5]
    }

    /// Evaluate velocity at time `t`.
    pub fn velocity(&self, t: f64) -> f64 {
        let c = &self.c;
        5.0 * c[0] * t.powi(4)
            + 4.0 * c[1] * t.powi(3)
            + 3.0 * c[2] * t.powi(2)
            + 2.0 * c[3] * t
            + c[4]
    }

    /// Evaluate acceleration at time `t`.
    pub fn acceleration(&self, t: f64) -> f64 {
        let c = &self.c;
        20.0 * c[0] * t.powi(3) + 12.0 * c[1] * t.powi(2) + 6.0 * c[2] * t + 2.0 * c[3]
    }
}

/// A 3D quintic trajectory segment (one per axis).
#[derive(Debug, Clone)]
pub struct TrajectorySegment {
    pub x: QuinticCoeffs,
    pub y: QuinticCoeffs,
    pub z: QuinticCoeffs,
}

/// Boundary conditions for one axis of a trajectory segment.
struct AxisBoundary {
    start_pos: f64,
    end_pos: f64,
    start_vel: f64,
    end_vel: f64,
    start_acc: f64,
    end_acc: f64,
}

/// Solve the quintic polynomial coefficients for one axis.
///
/// The 6x6 system enforces position, velocity, and acceleration at `t=0` and `t=T`.
fn solve_quintic_axis(b: &AxisBoundary, duration: f64) -> QuinticCoeffs {
    let big_t = duration;
    let t2 = big_t * big_t;
    let t3 = t2 * big_t;
    let t4 = t3 * big_t;
    let t5 = t4 * big_t;

    // A matrix (row-major):
    //   pos(0)=start:  [0,  0,  0,  0, 0, 1]
    //   pos(T)=end:    [T^5, T^4, T^3, T^2, T, 1]
    //   vel(0)=sv:     [0,  0,  0,  0, 1, 0]
    //   vel(T)=ev:     [5T^4, 4T^3, 3T^2, 2T, 1, 0]
    //   acc(0)=sa:     [0,  0,  0,  2, 0, 0]
    //   acc(T)=ea:     [20T^3, 12T^2, 6T, 2, 0, 0]
    #[rustfmt::skip]
    let a = Matrix6::new(
        0.0,     0.0,    0.0,   0.0,      0.0, 1.0,
        t5,      t4,     t3,    t2,        big_t, 1.0,
        0.0,     0.0,    0.0,   0.0,      1.0, 0.0,
        5.0*t4,  4.0*t3, 3.0*t2, 2.0*big_t, 1.0, 0.0,
        0.0,     0.0,    0.0,   2.0,      0.0, 0.0,
        20.0*t3, 12.0*t2, 6.0*big_t, 2.0, 0.0, 0.0,
    );

    let rhs = Vector6::new(
        b.start_pos,
        b.end_pos,
        b.start_vel,
        b.end_vel,
        b.start_acc,
        b.end_acc,
    );

    let decomp = a.lu();
    let coeffs = decomp
        .solve(&rhs)
        .expect("Quintic matrix should be invertible");

    QuinticCoeffs {
        c: [
            coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5],
        ],
    }
}

/// Generate a quintic trajectory segment between two 3D waypoints.
///
/// Boundary velocities and accelerations default to zero (rest-to-rest).
pub fn generate_trajectory_segment(
    start: &Vector3<f64>,
    end: &Vector3<f64>,
    duration: f64,
) -> TrajectorySegment {
    generate_trajectory_segment_full(
        start,
        end,
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::zeros(),
        duration,
    )
}

/// Generate a quintic trajectory segment with full boundary conditions.
pub fn generate_trajectory_segment_full(
    start_pos: &Vector3<f64>,
    end_pos: &Vector3<f64>,
    start_vel: &Vector3<f64>,
    end_vel: &Vector3<f64>,
    start_acc: &Vector3<f64>,
    end_acc: &Vector3<f64>,
    duration: f64,
) -> TrajectorySegment {
    let solve = |i: usize| {
        solve_quintic_axis(
            &AxisBoundary {
                start_pos: start_pos[i],
                end_pos: end_pos[i],
                start_vel: start_vel[i],
                end_vel: end_vel[i],
                start_acc: start_acc[i],
                end_acc: end_acc[i],
            },
            duration,
        )
    };
    TrajectorySegment {
        x: solve(0),
        y: solve(1),
        z: solve(2),
    }
}

/// Generate trajectory segments for a closed loop through the given waypoints.
///
/// Each segment uses rest-to-rest boundary conditions and the given duration.
pub fn generate_waypoint_trajectory(
    waypoints: &[Vector3<f64>],
    segment_duration: f64,
) -> Vec<TrajectorySegment> {
    let n = waypoints.len();
    assert!(n >= 2, "Need at least 2 waypoints");
    (0..n)
        .map(|i| {
            generate_trajectory_segment(&waypoints[i], &waypoints[(i + 1) % n], segment_duration)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Quadrotor dynamics and controller
// ---------------------------------------------------------------------------

/// Physical parameters of the quadrotor.
#[derive(Debug, Clone)]
pub struct QuadrotorParams {
    /// Mass \[kg\].
    pub mass: f64,
    /// Moment of inertia about the x-axis \[kg m^2\].
    pub ixx: f64,
    /// Moment of inertia about the y-axis \[kg m^2\].
    pub iyy: f64,
    /// Moment of inertia about the z-axis \[kg m^2\].
    pub izz: f64,
    /// Gravity acceleration \[m/s^2\].
    pub gravity: f64,
}

impl Default for QuadrotorParams {
    fn default() -> Self {
        Self {
            mass: 0.2,
            ixx: 1.0,
            iyy: 1.0,
            izz: 1.0,
            gravity: GRAVITY,
        }
    }
}

/// PD controller gains.
#[derive(Debug, Clone)]
pub struct ControllerGains {
    /// Proportional gains for position \[x, y, z\].
    pub kp_pos: Vector3<f64>,
    /// Derivative gains for position \[x, y, z\].
    pub kd_pos: Vector3<f64>,
    /// Proportional gains for attitude \[roll, pitch, yaw\].
    pub kp_att: Vector3<f64>,
}

impl Default for ControllerGains {
    fn default() -> Self {
        Self {
            kp_pos: Vector3::new(1.0, 1.0, 1.0),
            kd_pos: Vector3::new(10.0, 10.0, 1.0),
            kp_att: Vector3::new(25.0, 25.0, 25.0),
        }
    }
}

/// Full state of the quadrotor.
#[derive(Debug, Clone)]
pub struct QuadrotorState {
    /// Position \[x, y, z\] in meters.
    pub position: Vector3<f64>,
    /// Velocity \[vx, vy, vz\] in m/s.
    pub velocity: Vector3<f64>,
    /// Euler angles \[roll, pitch, yaw\] in radians.
    pub orientation: Vector3<f64>,
    /// Angular velocity \[roll_dot, pitch_dot, yaw_dot\] in rad/s.
    pub angular_velocity: Vector3<f64>,
}

impl QuadrotorState {
    /// Create a new state at the given position, at rest.
    pub fn new(position: Vector3<f64>) -> Self {
        Self {
            position,
            velocity: Vector3::zeros(),
            orientation: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
        }
    }
}

/// Desired trajectory values at a single time instant.
#[derive(Debug, Clone)]
pub struct DesiredState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub yaw: f64,
}

impl DesiredState {
    /// Evaluate a trajectory segment at time `t`.
    pub fn from_segment(seg: &TrajectorySegment, t: f64) -> Self {
        Self {
            position: Vector3::new(seg.x.position(t), seg.y.position(t), seg.z.position(t)),
            velocity: Vector3::new(seg.x.velocity(t), seg.y.velocity(t), seg.z.velocity(t)),
            acceleration: Vector3::new(
                seg.x.acceleration(t),
                seg.y.acceleration(t),
                seg.z.acceleration(t),
            ),
            yaw: 0.0,
        }
    }
}

/// Control outputs: total thrust and body torques.
#[derive(Debug, Clone)]
pub struct ControlOutput {
    /// Total thrust \[N\].
    pub thrust: f64,
    /// Torque about x-axis (roll) \[N m\].
    pub roll_torque: f64,
    /// Torque about y-axis (pitch) \[N m\].
    pub pitch_torque: f64,
    /// Torque about z-axis (yaw) \[N m\].
    pub yaw_torque: f64,
}

/// Compute the ZYX rotation matrix from Euler angles (roll, pitch, yaw).
pub fn rotation_matrix(roll: f64, pitch: f64, yaw: f64) -> Matrix3<f64> {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();

    Matrix3::new(
        cy * cp,
        -sy * cr + cy * sp * sr,
        sy * sr + cy * sp * cr,
        sy * cp,
        cy * cr + sy * sp * sr,
        -cy * sr + sy * sp * cr,
        -sp,
        cp * sr,
        cp * cr,
    )
}

/// Compute the PD control output for the quadrotor.
///
/// This follows the same controller structure as the Python reference:
/// - Thrust compensates gravity and tracks desired z with PD.
/// - Roll/pitch torques are proportional corrections derived from desired
///   lateral accelerations.
/// - Yaw torque is proportional to yaw error.
pub fn compute_control(
    state: &QuadrotorState,
    desired: &DesiredState,
    params: &QuadrotorParams,
    gains: &ControllerGains,
) -> ControlOutput {
    let g = params.gravity;
    let m = params.mass;

    // Thrust (along body z-axis)
    let thrust = m
        * (g + desired.acceleration.z
            + gains.kp_pos.z * (desired.position.z - state.position.z)
            + gains.kd_pos.z * (desired.velocity.z - state.velocity.z));

    let des_yaw = desired.yaw;
    let (sy, cy) = des_yaw.sin_cos();

    // Desired roll and pitch from lateral accelerations
    let des_roll = (desired.acceleration.x * sy - desired.acceleration.y * cy) / g;
    let des_pitch = (desired.acceleration.x * cy - desired.acceleration.y * sy) / g;

    let roll_torque = gains.kp_att.x * (des_roll - state.orientation.x);
    let pitch_torque = gains.kp_att.y * (des_pitch - state.orientation.y);
    let yaw_torque = gains.kp_att.z * (des_yaw - state.orientation.z);

    ControlOutput {
        thrust,
        roll_torque,
        pitch_torque,
        yaw_torque,
    }
}

/// Advance the quadrotor state by one time step using Euler integration.
pub fn step_dynamics(
    state: &mut QuadrotorState,
    control: &ControlOutput,
    params: &QuadrotorParams,
    dt: f64,
) {
    let m = params.mass;
    let g = params.gravity;

    // Angular acceleration -> angular velocity -> orientation
    state.angular_velocity.x += control.roll_torque * dt / params.ixx;
    state.angular_velocity.y += control.pitch_torque * dt / params.iyy;
    state.angular_velocity.z += control.yaw_torque * dt / params.izz;

    state.orientation += state.angular_velocity * dt;

    // Linear acceleration in world frame via rotation matrix
    let r = rotation_matrix(
        state.orientation.x,
        state.orientation.y,
        state.orientation.z,
    );
    let thrust_body = Vector3::new(0.0, 0.0, control.thrust);
    let gravity_world = Vector3::new(0.0, 0.0, m * g);
    let acc = (r * thrust_body - gravity_world) / m;

    state.velocity += acc * dt;
    state.position += state.velocity * dt;
}

/// Record of the quadrotor state at each simulation step.
#[derive(Debug, Clone)]
pub struct SimulationRecord {
    /// Position history.
    pub positions: Vec<Vector3<f64>>,
    /// Orientation (Euler angles) history.
    pub orientations: Vec<Vector3<f64>>,
    /// Control output history.
    pub controls: Vec<ControlOutput>,
}

/// Configuration for the trajectory-following simulation.
#[derive(Debug, Clone)]
pub struct SimulationConfig {
    /// Time step \[s\].
    pub dt: f64,
    /// Duration per trajectory segment \[s\].
    pub segment_duration: f64,
    /// Number of full loops through the waypoints.
    pub n_loops: usize,
    /// Quadrotor physical parameters.
    pub params: QuadrotorParams,
    /// Controller gains.
    pub gains: ControllerGains,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            dt: 0.1,
            segment_duration: 5.0,
            n_loops: 2,
            params: QuadrotorParams::default(),
            gains: ControllerGains::default(),
        }
    }
}

/// Run the full trajectory-following simulation.
///
/// The quadrotor starts at the first waypoint and follows a closed loop
/// through all waypoints for `config.n_loops` complete passes.
///
/// Returns a [`SimulationRecord`] containing the full state history.
pub fn simulate_trajectory_following(
    waypoints: &[Vector3<f64>],
    config: &SimulationConfig,
) -> SimulationRecord {
    let segments = generate_waypoint_trajectory(waypoints, config.segment_duration);
    let n_segments = segments.len();

    let mut state = QuadrotorState::new(waypoints[0]);
    let mut record = SimulationRecord {
        positions: vec![state.position],
        orientations: vec![state.orientation],
        controls: Vec::new(),
    };

    let total_segments = n_segments * config.n_loops;

    for seg_idx in 0..total_segments {
        let segment = &segments[seg_idx % n_segments];
        let mut t = 0.0;
        while t <= config.segment_duration {
            let desired = DesiredState::from_segment(segment, t);
            let control = compute_control(&state, &desired, &config.params, &config.gains);
            step_dynamics(&mut state, &control, &config.params, config.dt);

            record.positions.push(state.position);
            record.orientations.push(state.orientation);
            record.controls.push(control);

            t += config.dt;
        }
    }

    record
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // -----------------------------------------------------------------------
    // Quintic trajectory tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_quintic_boundary_conditions() {
        let start = Vector3::new(0.0, 0.0, 5.0);
        let end = Vector3::new(10.0, 0.0, 5.0);
        let duration = 5.0;
        let seg = generate_trajectory_segment(&start, &end, duration);

        // Position at t=0
        assert!((seg.x.position(0.0) - 0.0).abs() < EPSILON);
        assert!((seg.y.position(0.0) - 0.0).abs() < EPSILON);
        assert!((seg.z.position(0.0) - 5.0).abs() < EPSILON);

        // Position at t=T
        assert!((seg.x.position(duration) - 10.0).abs() < EPSILON);
        assert!((seg.y.position(duration) - 0.0).abs() < EPSILON);
        assert!((seg.z.position(duration) - 5.0).abs() < EPSILON);

        // Velocity at endpoints should be zero (rest-to-rest)
        assert!(seg.x.velocity(0.0).abs() < EPSILON);
        assert!(seg.x.velocity(duration).abs() < EPSILON);

        // Acceleration at endpoints should be zero
        assert!(seg.x.acceleration(0.0).abs() < EPSILON);
        assert!(seg.x.acceleration(duration).abs() < EPSILON);
    }

    #[test]
    fn test_quintic_constant_z() {
        // When start.z == end.z, z should remain constant.
        let seg = generate_trajectory_segment(
            &Vector3::new(0.0, 0.0, 5.0),
            &Vector3::new(10.0, 10.0, 5.0),
            5.0,
        );
        for i in 0..=50 {
            let t = i as f64 * 0.1;
            assert!(
                (seg.z.position(t) - 5.0).abs() < EPSILON,
                "z({}) = {} != 5.0",
                t,
                seg.z.position(t)
            );
        }
    }

    #[test]
    fn test_quintic_with_initial_velocity() {
        let start_pos = Vector3::new(0.0, 0.0, 0.0);
        let end_pos = Vector3::new(10.0, 0.0, 0.0);
        let start_vel = Vector3::new(2.0, 0.0, 0.0);
        let end_vel = Vector3::new(0.0, 0.0, 0.0);
        let duration = 5.0;

        let seg = generate_trajectory_segment_full(
            &start_pos,
            &end_pos,
            &start_vel,
            &end_vel,
            &Vector3::zeros(),
            &Vector3::zeros(),
            duration,
        );

        assert!((seg.x.velocity(0.0) - 2.0).abs() < EPSILON);
        assert!(seg.x.velocity(duration).abs() < EPSILON);
    }

    #[test]
    fn test_waypoint_trajectory_generation() {
        let waypoints = vec![
            Vector3::new(-5.0, -5.0, 5.0),
            Vector3::new(5.0, -5.0, 5.0),
            Vector3::new(5.0, 5.0, 5.0),
            Vector3::new(-5.0, 5.0, 5.0),
        ];
        let segments = generate_waypoint_trajectory(&waypoints, 5.0);
        assert_eq!(segments.len(), 4);

        // Each segment should start at its waypoint and end at the next
        for (i, seg) in segments.iter().enumerate() {
            let next = (i + 1) % waypoints.len();
            assert!(
                (seg.x.position(0.0) - waypoints[i].x).abs() < EPSILON,
                "Segment {} start x mismatch",
                i
            );
            assert!(
                (seg.x.position(5.0) - waypoints[next].x).abs() < EPSILON,
                "Segment {} end x mismatch",
                i
            );
        }
    }

    // -----------------------------------------------------------------------
    // Rotation matrix test
    // -----------------------------------------------------------------------

    #[test]
    fn test_rotation_identity() {
        let r = rotation_matrix(0.0, 0.0, 0.0);
        let identity = Matrix3::identity();
        assert!((r - identity).norm() < EPSILON);
    }

    #[test]
    fn test_rotation_orthogonal() {
        let r = rotation_matrix(0.3, 0.5, 1.0);
        let should_be_identity = r * r.transpose();
        assert!(
            (should_be_identity - Matrix3::identity()).norm() < 1e-12,
            "Rotation matrix not orthogonal"
        );
    }

    // -----------------------------------------------------------------------
    // Controller test
    // -----------------------------------------------------------------------

    #[test]
    fn test_hover_control() {
        // Quadrotor at desired position with zero error => thrust = m*g, no torques.
        let params = QuadrotorParams::default();
        let gains = ControllerGains::default();
        let pos = Vector3::new(0.0, 0.0, 5.0);
        let state = QuadrotorState::new(pos);
        let desired = DesiredState {
            position: pos,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            yaw: 0.0,
        };

        let ctrl = compute_control(&state, &desired, &params, &gains);
        let expected_thrust = params.mass * params.gravity;
        assert!(
            (ctrl.thrust - expected_thrust).abs() < EPSILON,
            "Hover thrust should be m*g, got {}",
            ctrl.thrust
        );
        assert!(ctrl.roll_torque.abs() < EPSILON);
        assert!(ctrl.pitch_torque.abs() < EPSILON);
        assert!(ctrl.yaw_torque.abs() < EPSILON);
    }

    #[test]
    fn test_control_z_error_increases_thrust() {
        let params = QuadrotorParams::default();
        let gains = ControllerGains::default();
        let state = QuadrotorState::new(Vector3::new(0.0, 0.0, 4.0));
        let desired = DesiredState {
            position: Vector3::new(0.0, 0.0, 5.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            yaw: 0.0,
        };

        let ctrl = compute_control(&state, &desired, &params, &gains);
        let hover_thrust = params.mass * params.gravity;
        assert!(
            ctrl.thrust > hover_thrust,
            "Below desired z => thrust should exceed hover"
        );
    }

    // -----------------------------------------------------------------------
    // Dynamics step test
    // -----------------------------------------------------------------------

    #[test]
    fn test_dynamics_freefall() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::new(Vector3::new(0.0, 0.0, 10.0));
        let zero_control = ControlOutput {
            thrust: 0.0,
            roll_torque: 0.0,
            pitch_torque: 0.0,
            yaw_torque: 0.0,
        };

        let dt = 0.01;
        for _ in 0..100 {
            step_dynamics(&mut state, &zero_control, &params, dt);
        }

        // After 1s of freefall: z ~ 10 - 0.5*g*1^2 ~ 5.095
        let expected_z = 10.0 - 0.5 * GRAVITY * 1.0;
        assert!(
            (state.position.z - expected_z).abs() < 0.5,
            "Freefall z: {}, expected ~{}",
            state.position.z,
            expected_z
        );
        // Velocity should be ~ -g*t = -9.81
        assert!(
            (state.velocity.z - (-GRAVITY)).abs() < 0.2,
            "Freefall vz: {}, expected ~{}",
            state.velocity.z,
            -GRAVITY
        );
        // x, y should stay zero
        assert!(state.position.x.abs() < EPSILON);
        assert!(state.position.y.abs() < EPSILON);
    }

    #[test]
    fn test_dynamics_hover() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::new(Vector3::new(0.0, 0.0, 5.0));
        let hover_control = ControlOutput {
            thrust: params.mass * params.gravity,
            roll_torque: 0.0,
            pitch_torque: 0.0,
            yaw_torque: 0.0,
        };

        let dt = 0.01;
        for _ in 0..1000 {
            step_dynamics(&mut state, &hover_control, &params, dt);
        }

        // After 10s of perfect hover, position should be unchanged.
        assert!(
            (state.position.z - 5.0).abs() < 0.01,
            "Hover z drifted to {}",
            state.position.z
        );
        assert!(state.velocity.z.abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Full simulation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_square_trajectory() {
        let waypoints = vec![
            Vector3::new(-5.0, -5.0, 5.0),
            Vector3::new(5.0, -5.0, 5.0),
            Vector3::new(5.0, 5.0, 5.0),
            Vector3::new(-5.0, 5.0, 5.0),
        ];

        let config = SimulationConfig {
            n_loops: 2,
            ..Default::default()
        };

        let record = simulate_trajectory_following(&waypoints, &config);

        // Should have recorded steps
        assert!(record.positions.len() > 100);
        assert_eq!(record.positions.len(), record.orientations.len());
        // Controls are one fewer than states (initial state has no control)
        assert_eq!(record.controls.len(), record.positions.len() - 1);

        // Altitude should stay roughly around 5.0 (constant z trajectory)
        let z_values: Vec<f64> = record.positions.iter().map(|p| p.z).collect();
        let z_min = z_values.iter().cloned().fold(f64::INFINITY, f64::min);
        let z_max = z_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            z_min > 2.0 && z_max < 8.0,
            "Altitude should stay near 5.0, got range [{}, {}]",
            z_min,
            z_max
        );
    }

    #[test]
    fn test_simulate_ascending_trajectory() {
        let waypoints = vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 10.0)];

        let config = SimulationConfig {
            n_loops: 1,
            segment_duration: 5.0,
            ..Default::default()
        };

        let record = simulate_trajectory_following(&waypoints, &config);

        // The drone should ascend: final z should be above start
        let final_z = record.positions.last().unwrap().z;
        assert!(
            final_z > 0.0,
            "Drone should have ascended, final z = {}",
            final_z
        );
    }

    #[test]
    fn test_simulation_record_consistency() {
        let waypoints = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(5.0, 0.0, 5.0),
            Vector3::new(5.0, 5.0, 5.0),
        ];

        let config = SimulationConfig {
            dt: 0.05,
            segment_duration: 3.0,
            n_loops: 1,
            ..Default::default()
        };

        let record = simulate_trajectory_following(&waypoints, &config);

        // First recorded position should be the starting waypoint
        let start = &record.positions[0];
        assert!((start.x - 0.0).abs() < EPSILON);
        assert!((start.y - 0.0).abs() < EPSILON);
        assert!((start.z - 5.0).abs() < EPSILON);

        // All positions should be finite
        for (i, p) in record.positions.iter().enumerate() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "Non-finite position at step {}: {:?}",
                i,
                p
            );
        }
    }
}
