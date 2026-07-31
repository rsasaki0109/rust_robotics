//! Bias-aware IMU preintegration with covariance propagation.
//!
//! Error states use `[rotation, position, velocity]`; biases use
//! `[accelerometer, gyroscope]`.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, SMatrix, Vector3};
use rust_robotics_core::{skew, so3_exp, so3_left_jacobian, so3_left_jacobian_inverse, so3_log};
use rust_robotics_optimization::{
    solve, Factor, FactorEvaluation, OptimizationError, OptimizationResult, Problem, SolverConfig,
    SolverSummary, Variable, VariableId,
};

pub type Matrix9 = SMatrix<f64, 9, 9>;
pub type Matrix9x6 = SMatrix<f64, 9, 6>;
pub type Matrix6 = SMatrix<f64, 6, 6>;

/// One accelerometer/gyroscope sample expressed in an IMU sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuMeasurement {
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

/// Rigid mounting transform from an IMU sensor frame to the navigation body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuExtrinsics {
    pub rotation_body_from_sensor: Matrix3<f64>,
    pub translation_body_from_sensor: Vector3<f64>,
}

impl ImuExtrinsics {
    pub fn identity() -> Self {
        Self {
            rotation_body_from_sensor: Matrix3::identity(),
            translation_body_from_sensor: Vector3::zeros(),
        }
    }

    /// Extracts the rotation and translation from a homogeneous body-from-sensor transform.
    pub fn from_homogeneous(body_from_sensor: &Matrix4<f64>) -> OptimizationResult<Self> {
        if !body_from_sensor.iter().all(|value| value.is_finite()) {
            return Err(OptimizationError::InvalidParameter(
                "IMU extrinsic transform must be finite".into(),
            ));
        }
        let bottom_row = body_from_sensor.row(3);
        if (bottom_row[0].abs() > 1.0e-12)
            || (bottom_row[1].abs() > 1.0e-12)
            || (bottom_row[2].abs() > 1.0e-12)
            || ((bottom_row[3] - 1.0).abs() > 1.0e-12)
        {
            return Err(OptimizationError::InvalidParameter(
                "IMU extrinsic transform must be homogeneous".into(),
            ));
        }
        let rotation = body_from_sensor.fixed_view::<3, 3>(0, 0).into_owned();
        let orthogonality_error = rotation.transpose() * rotation - Matrix3::identity();
        if orthogonality_error.norm() > 1.0e-8 || (rotation.determinant() - 1.0).abs() > 1.0e-8 {
            return Err(OptimizationError::InvalidParameter(
                "IMU extrinsic rotation must be in SO(3)".into(),
            ));
        }
        Ok(Self {
            rotation_body_from_sensor: rotation,
            translation_body_from_sensor: body_from_sensor.fixed_view::<3, 1>(0, 3).into_owned(),
        })
    }

    /// Transforms a sensor-frame IMU sample to the body frame.
    ///
    /// The acceleration includes centripetal and tangential lever-arm terms,
    /// matching MathematicalRobotics `transformIMU`.
    pub fn transform(
        &self,
        measurement: ImuMeasurement,
        angular_acceleration_sensor: Vector3<f64>,
    ) -> ImuMeasurement {
        let angular_velocity = self.rotation_body_from_sensor * measurement.angular_velocity;
        let angular_acceleration = self.rotation_body_from_sensor * angular_acceleration_sensor;
        let translation = self.translation_body_from_sensor;
        let acceleration = self.rotation_body_from_sensor * measurement.acceleration
            - skew(&angular_velocity) * skew(&angular_velocity) * translation
            + skew(&translation) * angular_acceleration;
        ImuMeasurement {
            acceleration,
            angular_velocity,
        }
    }
}

/// Accelerometer and gyroscope biases.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuBias {
    pub accelerometer: Vector3<f64>,
    pub gyroscope: Vector3<f64>,
}

impl ImuBias {
    pub fn zero() -> Self {
        Self {
            accelerometer: Vector3::zeros(),
            gyroscope: Vector3::zeros(),
        }
    }

    fn vector(&self) -> DVector<f64> {
        DVector::from_vec(vec![
            self.accelerometer.x,
            self.accelerometer.y,
            self.accelerometer.z,
            self.gyroscope.x,
            self.gyroscope.y,
            self.gyroscope.z,
        ])
    }
}

/// Navigation state in a world frame.
#[derive(Debug, Clone, PartialEq)]
pub struct NavState {
    pub rotation: Matrix3<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

impl NavState {
    pub fn identity() -> Self {
        Self {
            rotation: Matrix3::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }
}

/// White measurement noise used during covariance propagation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuNoise {
    pub accelerometer_sigma: f64,
    pub gyroscope_sigma: f64,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self {
            accelerometer_sigma: 0.1,
            gyroscope_sigma: 0.01,
        }
    }
}

/// Preintegrated measurements between two navigation states.
#[derive(Debug, Clone)]
pub struct PreintegratedImuMeasurement {
    pub delta_rotation: Matrix3<f64>,
    pub delta_position: Vector3<f64>,
    pub delta_velocity: Vector3<f64>,
    pub delta_time: f64,
    pub covariance: Matrix9,
    pub bias_jacobian: Matrix9x6,
    pub linearization_bias: ImuBias,
    noise: ImuNoise,
}

impl PreintegratedImuMeasurement {
    pub fn new(linearization_bias: ImuBias, noise: ImuNoise) -> Self {
        Self {
            delta_rotation: Matrix3::identity(),
            delta_position: Vector3::zeros(),
            delta_velocity: Vector3::zeros(),
            delta_time: 0.0,
            covariance: Matrix9::zeros(),
            bias_jacobian: Matrix9x6::zeros(),
            linearization_bias,
            noise,
        }
    }

    /// Integrates one body-frame specific-force and angular-rate sample.
    pub fn integrate(
        &mut self,
        acceleration: Vector3<f64>,
        angular_velocity: Vector3<f64>,
        dt: f64,
    ) -> OptimizationResult<()> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(OptimizationError::InvalidParameter(
                "IMU sample dt must be finite and positive".into(),
            ));
        }
        let acceleration = acceleration - self.linearization_bias.accelerometer;
        let angular_velocity = angular_velocity - self.linearization_bias.gyroscope;
        let rotation = self.delta_rotation;
        let rotated_acceleration = rotation * acceleration;
        let half_dt_squared = 0.5 * dt * dt;

        self.delta_position += self.delta_velocity * dt + rotated_acceleration * half_dt_squared;
        self.delta_velocity += rotated_acceleration * dt;
        self.delta_rotation *= so3_exp(&(angular_velocity * dt));
        self.delta_time += dt;

        let mut transition = Matrix9::identity();
        transition
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&so3_exp(&(-angular_velocity * dt)));
        transition
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-rotation * skew(&acceleration) * half_dt_squared));
        transition
            .fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(Matrix3::identity() * dt));
        transition
            .fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-rotation * skew(&acceleration) * dt));

        let mut noise_jacobian = Matrix9x6::zeros();
        noise_jacobian
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Matrix3::identity() * dt));
        noise_jacobian
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(rotation * half_dt_squared));
        noise_jacobian
            .fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(rotation * dt));

        let mut measurement_covariance = Matrix6::zeros();
        measurement_covariance
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::identity() * self.noise.accelerometer_sigma.powi(2)));
        measurement_covariance
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::identity() * self.noise.gyroscope_sigma.powi(2)));

        self.covariance = transition * self.covariance * transition.transpose()
            + noise_jacobian * measurement_covariance * noise_jacobian.transpose();
        self.bias_jacobian = transition * self.bias_jacobian - noise_jacobian;
        Ok(())
    }

    /// Transforms and integrates one sensor-frame IMU sample.
    pub fn integrate_sensor_measurement(
        &mut self,
        measurement: ImuMeasurement,
        angular_acceleration_sensor: Vector3<f64>,
        extrinsics: &ImuExtrinsics,
        dt: f64,
    ) -> OptimizationResult<()> {
        let body_measurement = extrinsics.transform(measurement, angular_acceleration_sensor);
        self.integrate(
            body_measurement.acceleration,
            body_measurement.angular_velocity,
            dt,
        )
    }

    /// Predicts the next navigation state, applying first-order bias correction.
    pub fn predict(&self, state: &NavState, bias: &ImuBias, gravity: Vector3<f64>) -> NavState {
        let (corrected_rotation, corrected_position, corrected_velocity) =
            self.corrected_delta(bias);
        let dt = self.delta_time;

        NavState {
            rotation: state.rotation * corrected_rotation,
            position: state.position
                + state.velocity * dt
                + gravity * (0.5 * dt * dt)
                + state.rotation * corrected_position,
            velocity: state.velocity + gravity * dt + state.rotation * corrected_velocity,
        }
    }

    fn corrected_delta(&self, bias: &ImuBias) -> (Matrix3<f64>, Vector3<f64>, Vector3<f64>) {
        let bias_delta = bias.vector() - self.linearization_bias.vector();
        let correction = self.bias_jacobian * bias_delta;
        (
            self.delta_rotation * so3_exp(&Vector3::from(correction.fixed_rows::<3>(0))),
            self.delta_position + Vector3::from(correction.fixed_rows::<3>(3)),
            self.delta_velocity + Vector3::from(correction.fixed_rows::<3>(6)),
        )
    }
}

/// Relative rotation, position and velocity measurement between navigation states.
#[derive(Debug, Clone, PartialEq)]
pub struct NavStateDelta {
    pub rotation: Matrix3<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

impl NavStateDelta {
    pub fn identity() -> Self {
        Self {
            rotation: Matrix3::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }
}

/// Creates a 9-DOF manifold variable for a navigation state.
pub fn nav_state_variable(state: &NavState) -> Variable {
    Variable::with_retraction(encode_nav_state(state), 9, retract_nav_state)
        .expect("navigation state dimensions are valid")
}

/// Creates a Euclidean six-dimensional bias variable.
pub fn imu_bias_variable(bias: &ImuBias) -> Variable {
    Variable::euclidean(bias.vector())
}

/// Prior factor for one six-dimensional accelerometer/gyroscope bias.
pub struct BiasPriorFactor {
    variables: [VariableId; 1],
    measurement: ImuBias,
    information: Matrix6,
}

impl BiasPriorFactor {
    pub fn new(variable: VariableId, measurement: ImuBias, information: Matrix6) -> Self {
        Self {
            variables: [variable],
            measurement,
            information,
        }
    }
}

impl Factor for BiasPriorFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        expect_dimensions(values, &[6], "bias prior")?;
        Ok(FactorEvaluation {
            residual: values[0] - self.measurement.vector(),
            information: fixed_to_dynamic(&self.information),
            jacobians: vec![DMatrix::identity(6, 6)],
        })
    }
}

/// Random-walk factor constraining two consecutive IMU biases.
pub struct BiasBetweenFactor {
    variables: [VariableId; 2],
    information: Matrix6,
}

impl BiasBetweenFactor {
    pub fn new(bias_i: VariableId, bias_j: VariableId, information: Matrix6) -> Self {
        Self {
            variables: [bias_i, bias_j],
            information,
        }
    }
}

impl Factor for BiasBetweenFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        expect_dimensions(values, &[6, 6], "bias between")?;
        Ok(FactorEvaluation {
            residual: values[0] - values[1],
            information: fixed_to_dynamic(&self.information),
            jacobians: vec![DMatrix::identity(6, 6), -DMatrix::identity(6, 6)],
        })
    }
}

/// Prior factor for a full navigation state.
pub struct NavStatePriorFactor {
    variables: [VariableId; 1],
    measurement: NavState,
    information: Matrix9,
}

impl NavStatePriorFactor {
    pub fn new(variable: VariableId, measurement: NavState, information: Matrix9) -> Self {
        Self {
            variables: [variable],
            measurement,
            information,
        }
    }

    fn residual(&self, value: &DVector<f64>) -> nalgebra::SVector<f64, 9> {
        let state = decode_nav_state(value);
        nav_state_local(&state, &self.measurement)
    }
}

impl Factor for NavStatePriorFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        expect_dimensions(values, &[9], "navigation-state prior")?;
        let state = decode_nav_state(values[0]);
        let residual = self.residual(values[0]);
        let rotation_residual = Vector3::from(residual.fixed_rows::<3>(0));
        let local_position = Vector3::from(residual.fixed_rows::<3>(3));
        let local_velocity = Vector3::from(residual.fixed_rows::<3>(6));
        let inverse_rotation = state.rotation.transpose();
        let mut jacobian = DMatrix::zeros(9, 9);
        jacobian
            .view_mut((0, 0), (3, 3))
            .copy_from(&(-so3_left_jacobian_inverse(&rotation_residual)));
        jacobian
            .view_mut((3, 0), (3, 3))
            .copy_from(&skew(&local_position));
        jacobian
            .view_mut((3, 3), (3, 3))
            .copy_from(&(-inverse_rotation));
        jacobian
            .view_mut((6, 0), (3, 3))
            .copy_from(&skew(&local_velocity));
        jacobian
            .view_mut((6, 6), (3, 3))
            .copy_from(&(-inverse_rotation));
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice(residual.as_slice()),
            information: fixed_to_dynamic(&self.information),
            jacobians: vec![jacobian],
        })
    }
}

/// Constrains velocity to the finite difference between consecutive positions.
pub struct PositionVelocityFactor {
    variables: [VariableId; 2],
    delta_time: f64,
    information: Matrix3<f64>,
}

impl PositionVelocityFactor {
    pub fn new(
        state_i: VariableId,
        state_j: VariableId,
        delta_time: f64,
        information: Matrix3<f64>,
    ) -> OptimizationResult<Self> {
        if !delta_time.is_finite() || delta_time <= 0.0 {
            return Err(OptimizationError::InvalidParameter(
                "position/velocity factor dt must be finite and positive".into(),
            ));
        }
        Ok(Self {
            variables: [state_i, state_j],
            delta_time,
            information,
        })
    }
}

impl Factor for PositionVelocityFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        expect_dimensions(values, &[9, 9], "position/velocity")?;
        let state_i = decode_nav_state(values[0]);
        let state_j = decode_nav_state(values[1]);
        let inverse_dt = self.delta_time.recip();
        let residual = state_i.velocity - (state_j.position - state_i.position) * inverse_dt;
        let mut jacobian_i = DMatrix::zeros(3, 9);
        jacobian_i
            .view_mut((0, 3), (3, 3))
            .copy_from(&(Matrix3::identity() * inverse_dt));
        jacobian_i
            .view_mut((0, 6), (3, 3))
            .copy_from(&Matrix3::identity());
        let mut jacobian_j = DMatrix::zeros(3, 9);
        jacobian_j
            .view_mut((0, 3), (3, 3))
            .copy_from(&(-Matrix3::identity() * inverse_dt));
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice(residual.as_slice()),
            information: fixed_to_dynamic(&self.information),
            jacobians: vec![jacobian_i, jacobian_j],
        })
    }
}

/// Relative navigation-state factor, equivalent to MathematicalRobotics `NavitransEdge`.
pub struct NavStateBetweenFactor {
    variables: [VariableId; 2],
    measurement: NavStateDelta,
    information: Matrix9,
}

impl NavStateBetweenFactor {
    pub fn new(
        state_i: VariableId,
        state_j: VariableId,
        measurement: NavStateDelta,
        information: Matrix9,
    ) -> Self {
        Self {
            variables: [state_i, state_j],
            measurement,
            information,
        }
    }

    fn residual(
        &self,
        value_i: &DVector<f64>,
        value_j: &DVector<f64>,
    ) -> nalgebra::SVector<f64, 9> {
        let state_i = decode_nav_state(value_i);
        let state_j = decode_nav_state(value_j);
        let predicted = nav_state_delta(&state_i, &state_j);
        let rotation_error = so3_log(&(self.measurement.rotation.transpose() * predicted.rotation));
        nalgebra::SVector::<f64, 9>::from_iterator(
            rotation_error
                .iter()
                .chain((predicted.position - self.measurement.position).iter())
                .chain((predicted.velocity - self.measurement.velocity).iter())
                .copied(),
        )
    }
}

impl Factor for NavStateBetweenFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        expect_dimensions(values, &[9, 9], "navigation-state between")?;
        let state_i = decode_nav_state(values[0]);
        let state_j = decode_nav_state(values[1]);
        let residual = self.residual(values[0], values[1]);
        let local_position = state_i.rotation.transpose() * (state_j.position - state_i.position);
        let local_velocity = state_i.rotation.transpose() * (state_j.velocity - state_i.velocity);
        let rotation_residual = Vector3::from(residual.fixed_rows::<3>(0));
        let inverse_rotation_i = state_i.rotation.transpose();
        let mut jacobian_i = DMatrix::zeros(9, 9);
        jacobian_i.view_mut((0, 0), (3, 3)).copy_from(
            &(-so3_left_jacobian_inverse(&rotation_residual)
                * self.measurement.rotation.transpose()),
        );
        jacobian_i
            .view_mut((3, 0), (3, 3))
            .copy_from(&skew(&local_position));
        jacobian_i
            .view_mut((3, 3), (3, 3))
            .copy_from(&(-inverse_rotation_i));
        jacobian_i
            .view_mut((6, 0), (3, 3))
            .copy_from(&skew(&local_velocity));
        jacobian_i
            .view_mut((6, 6), (3, 3))
            .copy_from(&(-inverse_rotation_i));

        let mut jacobian_j = DMatrix::zeros(9, 9);
        jacobian_j
            .view_mut((0, 0), (3, 3))
            .copy_from(&so3_left_jacobian_inverse(&(-rotation_residual)));
        jacobian_j
            .view_mut((3, 3), (3, 3))
            .copy_from(&inverse_rotation_i);
        jacobian_j
            .view_mut((6, 6), (3, 3))
            .copy_from(&inverse_rotation_i);
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice(residual.as_slice()),
            information: fixed_to_dynamic(&self.information),
            jacobians: vec![jacobian_i, jacobian_j],
        })
    }
}

/// IMU factor linking `(state_i, state_j, bias_i)`.
pub struct ImuFactor {
    variables: [VariableId; 3],
    measurement: PreintegratedImuMeasurement,
    gravity: Vector3<f64>,
}

impl ImuFactor {
    pub fn new(
        state_i: VariableId,
        state_j: VariableId,
        bias_i: VariableId,
        measurement: PreintegratedImuMeasurement,
        gravity: Vector3<f64>,
    ) -> Self {
        Self {
            variables: [state_i, state_j, bias_i],
            measurement,
            gravity,
        }
    }
}

impl Factor for ImuFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        if values[0].len() != 9 || values[1].len() != 9 || values[2].len() != 6 {
            return Err(OptimizationError::InvalidFactor(
                "IMU factor expects state(9), state(9), bias(6)".into(),
            ));
        }
        let residual = self.residual(values);
        let jacobians = self.analytic_jacobians(values, &residual);
        let regularized = self.measurement.covariance + Matrix9::identity() * 1.0e-12;
        let information = regularized
            .try_inverse()
            .ok_or(OptimizationError::LinearSolveFailed)?;
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice(residual.as_slice()),
            information: DMatrix::from_column_slice(9, 9, information.as_slice()),
            jacobians,
        })
    }
}

impl ImuFactor {
    fn residual(&self, values: &[&DVector<f64>]) -> nalgebra::SVector<f64, 9> {
        let state_i = decode_nav_state(values[0]);
        let state_j = decode_nav_state(values[1]);
        let bias = decode_bias(values[2]);
        let (delta_rotation, delta_position, delta_velocity) =
            self.measurement.corrected_delta(&bias);
        let dt = self.measurement.delta_time;
        let rotation_error = so3_log(
            &(delta_rotation.transpose() * state_i.rotation.transpose() * state_j.rotation),
        );
        let position_error = state_i.rotation.transpose()
            * (state_j.position
                - state_i.position
                - state_i.velocity * dt
                - self.gravity * (0.5 * dt * dt))
            - delta_position;
        let velocity_error = state_i.rotation.transpose()
            * (state_j.velocity - state_i.velocity - self.gravity * dt)
            - delta_velocity;
        nalgebra::SVector::<f64, 9>::from_iterator(
            rotation_error
                .iter()
                .chain(position_error.iter())
                .chain(velocity_error.iter())
                .copied(),
        )
    }

    fn analytic_jacobians(
        &self,
        values: &[&DVector<f64>],
        residual: &nalgebra::SVector<f64, 9>,
    ) -> Vec<DMatrix<f64>> {
        let state_i = decode_nav_state(values[0]);
        let state_j = decode_nav_state(values[1]);
        let bias = decode_bias(values[2]);
        let inverse_rotation_i = state_i.rotation.transpose();
        let dt = self.measurement.delta_time;
        let position_delta = state_j.position
            - state_i.position
            - state_i.velocity * dt
            - self.gravity * (0.5 * dt * dt);
        let velocity_delta = state_j.velocity - state_i.velocity - self.gravity * dt;
        let local_position = inverse_rotation_i * position_delta;
        let local_velocity = inverse_rotation_i * velocity_delta;
        let rotation_residual = Vector3::from(residual.fixed_rows::<3>(0));
        let left_inverse = so3_left_jacobian_inverse(&rotation_residual);
        let right_inverse = so3_left_jacobian_inverse(&(-rotation_residual));
        let (corrected_rotation, _, _) = self.measurement.corrected_delta(&bias);

        let mut state_i_jacobian = DMatrix::zeros(9, 9);
        state_i_jacobian
            .view_mut((0, 0), (3, 3))
            .copy_from(&(-left_inverse * corrected_rotation.transpose()));
        state_i_jacobian
            .view_mut((3, 0), (3, 3))
            .copy_from(&skew(&local_position));
        state_i_jacobian
            .view_mut((3, 3), (3, 3))
            .copy_from(&(-inverse_rotation_i));
        state_i_jacobian
            .view_mut((3, 6), (3, 3))
            .copy_from(&(-inverse_rotation_i * dt));
        state_i_jacobian
            .view_mut((6, 0), (3, 3))
            .copy_from(&skew(&local_velocity));
        state_i_jacobian
            .view_mut((6, 6), (3, 3))
            .copy_from(&(-inverse_rotation_i));

        let mut state_j_jacobian = DMatrix::zeros(9, 9);
        state_j_jacobian
            .view_mut((0, 0), (3, 3))
            .copy_from(&right_inverse);
        state_j_jacobian
            .view_mut((3, 3), (3, 3))
            .copy_from(&inverse_rotation_i);
        state_j_jacobian
            .view_mut((6, 6), (3, 3))
            .copy_from(&inverse_rotation_i);

        let bias_delta = bias.vector() - self.measurement.linearization_bias.vector();
        let correction = self.measurement.bias_jacobian * bias_delta;
        let rotation_correction = Vector3::from(correction.fixed_rows::<3>(0));
        let correction_right_jacobian = so3_left_jacobian(&(-rotation_correction));
        let mut bias_jacobian = DMatrix::zeros(9, 6);
        bias_jacobian.view_mut((0, 0), (3, 6)).copy_from(
            &(-left_inverse
                * correction_right_jacobian
                * self.measurement.bias_jacobian.fixed_rows::<3>(0)),
        );
        bias_jacobian
            .view_mut((3, 0), (3, 6))
            .copy_from(&(-self.measurement.bias_jacobian.fixed_rows::<3>(3)));
        bias_jacobian
            .view_mut((6, 0), (3, 6))
            .copy_from(&(-self.measurement.bias_jacobian.fixed_rows::<3>(6)));

        vec![state_i_jacobian, state_j_jacobian, bias_jacobian]
    }

    #[cfg(test)]
    fn numerical_jacobian(&self, values: &[&DVector<f64>], variable: usize) -> DMatrix<f64> {
        let dimension = if variable == 2 { 6 } else { 9 };
        let epsilon = 1.0e-6;
        let mut jacobian = DMatrix::zeros(9, dimension);
        for column in 0..dimension {
            let mut increment = DVector::zeros(dimension);
            increment[column] = epsilon;
            let plus_value = if variable == 2 {
                values[variable] + &increment
            } else {
                retract_nav_state(values[variable], &increment).expect("dimensions are valid")
            };
            let minus_value = if variable == 2 {
                values[variable] - &increment
            } else {
                retract_nav_state(values[variable], &(-&increment)).expect("dimensions are valid")
            };
            let mut plus_values = values.to_vec();
            let mut minus_values = values.to_vec();
            plus_values[variable] = &plus_value;
            minus_values[variable] = &minus_value;
            let derivative =
                (self.residual(&plus_values) - self.residual(&minus_values)) / (2.0 * epsilon);
            jacobian.set_column(column, &derivative);
        }
        jacobian
    }
}

/// Configuration for joint navigation-state and IMU-bias refinement.
#[derive(Debug, Clone)]
pub struct ImuTrajectoryConfig {
    pub solver: SolverConfig,
    pub nav_prior_information: Matrix9,
    /// Optional information applied to every supplied initial navigation state.
    ///
    /// This lets visual pose estimates constrain the inertial trajectory. Zero
    /// rows/columns can leave velocity or another component unconstrained.
    pub nav_measurement_information: Option<Matrix9>,
    pub bias_prior_information: Matrix6,
    pub bias_between_information: Matrix6,
}

impl Default for ImuTrajectoryConfig {
    fn default() -> Self {
        Self {
            solver: SolverConfig::default(),
            nav_prior_information: Matrix9::identity() * 1.0e8,
            nav_measurement_information: None,
            bias_prior_information: Matrix6::identity() * 1.0e4,
            bias_between_information: Matrix6::identity() * 1.0e3,
        }
    }
}

/// Refined navigation states, time-varying biases and solver diagnostics.
#[derive(Debug, Clone)]
pub struct ImuTrajectoryResult {
    pub states: Vec<NavState>,
    pub biases: Vec<ImuBias>,
    pub summary: SolverSummary,
}

/// Jointly refines a navigation-state sequence and one bias per keyframe.
///
/// The first state and bias receive priors, each preintegrated interval adds an
/// [`ImuFactor`], and consecutive biases receive a random-walk constraint.
pub fn optimize_imu_trajectory(
    initial_states: &[NavState],
    initial_bias: ImuBias,
    measurements: &[PreintegratedImuMeasurement],
    gravity: Vector3<f64>,
    config: &ImuTrajectoryConfig,
) -> OptimizationResult<ImuTrajectoryResult> {
    if initial_states.len() < 2 || measurements.len() + 1 != initial_states.len() {
        return Err(OptimizationError::InvalidParameter(
            "IMU trajectory requires N states and N-1 measurements".into(),
        ));
    }
    let mut problem = Problem::new();
    let state_ids = initial_states
        .iter()
        .map(|state| problem.add_variable(nav_state_variable(state)))
        .collect::<Vec<_>>();
    let bias_ids = initial_states
        .iter()
        .map(|_| problem.add_variable(imu_bias_variable(&initial_bias)))
        .collect::<Vec<_>>();

    problem.add_factor(NavStatePriorFactor::new(
        state_ids[0],
        initial_states[0].clone(),
        config.nav_prior_information,
    ))?;
    problem.add_factor(BiasPriorFactor::new(
        bias_ids[0],
        initial_bias,
        config.bias_prior_information,
    ))?;
    if let Some(information) = config.nav_measurement_information {
        for (id, measurement) in state_ids.iter().zip(initial_states) {
            problem.add_factor(NavStatePriorFactor::new(
                *id,
                measurement.clone(),
                information,
            ))?;
        }
    }
    for (index, measurement) in measurements.iter().enumerate() {
        problem.add_factor(ImuFactor::new(
            state_ids[index],
            state_ids[index + 1],
            bias_ids[index],
            measurement.clone(),
            gravity,
        ))?;
        problem.add_factor(BiasBetweenFactor::new(
            bias_ids[index],
            bias_ids[index + 1],
            config.bias_between_information,
        ))?;
    }

    let summary = solve(&mut problem, &config.solver)?;
    let states = state_ids
        .iter()
        .map(|id| {
            problem
                .variable(*id)
                .map(|variable| decode_nav_state(variable.value()))
        })
        .collect::<OptimizationResult<Vec<_>>>()?;
    let biases = bias_ids
        .iter()
        .map(|id| {
            problem
                .variable(*id)
                .map(|variable| decode_bias(variable.value()))
        })
        .collect::<OptimizationResult<Vec<_>>>()?;
    Ok(ImuTrajectoryResult {
        states,
        biases,
        summary,
    })
}

fn nav_state_delta(from: &NavState, to: &NavState) -> NavStateDelta {
    let inverse_rotation = from.rotation.transpose();
    NavStateDelta {
        rotation: inverse_rotation * to.rotation,
        position: inverse_rotation * (to.position - from.position),
        velocity: inverse_rotation * (to.velocity - from.velocity),
    }
}

fn nav_state_local(from: &NavState, to: &NavState) -> nalgebra::SVector<f64, 9> {
    let delta = nav_state_delta(from, to);
    let rotation = so3_log(&delta.rotation);
    nalgebra::SVector::<f64, 9>::from_iterator(
        rotation
            .iter()
            .chain(delta.position.iter())
            .chain(delta.velocity.iter())
            .copied(),
    )
}

fn expect_dimensions(
    values: &[&DVector<f64>],
    dimensions: &[usize],
    factor_name: &str,
) -> OptimizationResult<()> {
    if values.len() != dimensions.len()
        || values
            .iter()
            .zip(dimensions)
            .any(|(value, dimension)| value.len() != *dimension)
    {
        return Err(OptimizationError::InvalidFactor(format!(
            "{factor_name} factor received invalid variable dimensions"
        )));
    }
    Ok(())
}

fn fixed_to_dynamic<const N: usize>(matrix: &SMatrix<f64, N, N>) -> DMatrix<f64> {
    DMatrix::from_column_slice(N, N, matrix.as_slice())
}

fn encode_nav_state(state: &NavState) -> DVector<f64> {
    let rotation = so3_log(&state.rotation);
    DVector::from_vec(vec![
        rotation.x,
        rotation.y,
        rotation.z,
        state.position.x,
        state.position.y,
        state.position.z,
        state.velocity.x,
        state.velocity.y,
        state.velocity.z,
    ])
}

fn decode_nav_state(value: &DVector<f64>) -> NavState {
    NavState {
        rotation: so3_exp(&Vector3::new(value[0], value[1], value[2])),
        position: Vector3::new(value[3], value[4], value[5]),
        velocity: Vector3::new(value[6], value[7], value[8]),
    }
}

fn decode_bias(value: &DVector<f64>) -> ImuBias {
    ImuBias {
        accelerometer: Vector3::new(value[0], value[1], value[2]),
        gyroscope: Vector3::new(value[3], value[4], value[5]),
    }
}

fn retract_nav_state(
    value: &DVector<f64>,
    increment: &DVector<f64>,
) -> OptimizationResult<DVector<f64>> {
    if value.len() != 9 || increment.len() != 9 {
        return Err(OptimizationError::InvalidParameter(
            "navigation state and increment must have dimension 9".into(),
        ));
    }
    let state = decode_nav_state(value);
    let updated = NavState {
        rotation: state.rotation * so3_exp(&Vector3::new(increment[0], increment[1], increment[2])),
        position: state.position + Vector3::new(increment[3], increment[4], increment[5]),
        velocity: state.velocity + Vector3::new(increment[6], increment[7], increment[8]),
    };
    Ok(encode_nav_state(&updated))
}

#[cfg(test)]
mod tests {
    use rust_robotics_optimization::{Factor, Problem};

    use super::*;

    fn numerical_factor_jacobian(
        factor: &dyn Factor,
        values: &[DVector<f64>],
        variable: usize,
        nav_state: bool,
    ) -> DMatrix<f64> {
        let residual_dimension = {
            let refs = values.iter().collect::<Vec<_>>();
            factor.evaluate(&refs).unwrap().residual.len()
        };
        let dimension = values[variable].len();
        let epsilon = 1.0e-6;
        let mut jacobian = DMatrix::zeros(residual_dimension, dimension);
        for column in 0..dimension {
            let mut increment = DVector::zeros(dimension);
            increment[column] = epsilon;
            let mut plus = values.to_vec();
            let mut minus = values.to_vec();
            plus[variable] = if nav_state {
                retract_nav_state(&values[variable], &increment).unwrap()
            } else {
                &values[variable] + &increment
            };
            minus[variable] = if nav_state {
                retract_nav_state(&values[variable], &(-&increment)).unwrap()
            } else {
                &values[variable] - &increment
            };
            let plus_refs = plus.iter().collect::<Vec<_>>();
            let minus_refs = minus.iter().collect::<Vec<_>>();
            let derivative = (factor.evaluate(&plus_refs).unwrap().residual
                - factor.evaluate(&minus_refs).unwrap().residual)
                / (2.0 * epsilon);
            jacobian.set_column(column, &derivative);
        }
        jacobian
    }

    #[test]
    fn imu_extrinsics_include_rotation_and_lever_arm_terms() {
        let extrinsics = ImuExtrinsics {
            rotation_body_from_sensor: so3_exp(&Vector3::new(0.1, -0.2, 0.3)),
            translation_body_from_sensor: Vector3::new(0.4, -0.2, 0.1),
        };
        let measurement = ImuMeasurement {
            acceleration: Vector3::new(0.3, -0.1, 9.7),
            angular_velocity: Vector3::new(0.2, -0.3, 0.4),
        };
        let angular_acceleration = Vector3::new(0.05, 0.02, -0.04);
        let transformed = extrinsics.transform(measurement, angular_acceleration);
        let rotation = extrinsics.rotation_body_from_sensor;
        let omega = rotation * measurement.angular_velocity;
        let alpha = rotation * angular_acceleration;
        let expected_acceleration = rotation * measurement.acceleration
            - skew(&omega) * skew(&omega) * extrinsics.translation_body_from_sensor
            + skew(&extrinsics.translation_body_from_sensor) * alpha;
        assert!((transformed.acceleration - expected_acceleration).norm() < 1.0e-12);
        assert!((transformed.angular_velocity - omega).norm() < 1.0e-12);

        let identity = ImuExtrinsics::identity().transform(measurement, angular_acceleration);
        assert_eq!(identity, measurement);
    }

    #[test]
    fn specialized_factor_jacobians_match_finite_differences() {
        let state_i = NavState {
            rotation: so3_exp(&Vector3::new(0.2, -0.1, 0.3)),
            position: Vector3::new(1.0, -2.0, 0.5),
            velocity: Vector3::new(0.4, 0.1, -0.2),
        };
        let state_j = NavState {
            rotation: so3_exp(&Vector3::new(-0.1, 0.3, 0.2)),
            position: Vector3::new(1.3, -1.8, 0.7),
            velocity: Vector3::new(0.2, 0.5, -0.1),
        };
        let measurement = NavState {
            rotation: so3_exp(&Vector3::new(0.25, -0.05, 0.28)),
            position: Vector3::new(0.9, -1.9, 0.55),
            velocity: Vector3::new(0.35, 0.2, -0.15),
        };
        let value_i = encode_nav_state(&state_i);
        let value_j = encode_nav_state(&state_j);

        let prior = NavStatePriorFactor::new(VariableId(0), measurement, Matrix9::identity());
        let prior_values = vec![value_i.clone()];
        let prior_refs = prior_values.iter().collect::<Vec<_>>();
        let prior_evaluation = prior.evaluate(&prior_refs).unwrap();
        let prior_numerical = numerical_factor_jacobian(&prior, &prior_values, 0, true);
        assert!(
            (&prior_evaluation.jacobians[0] - prior_numerical).norm() < 1.0e-5,
            "navigation prior analytic Jacobian disagrees with finite differences"
        );

        let between_measurement = NavStateDelta {
            rotation: so3_exp(&Vector3::new(-0.2, 0.1, 0.15)),
            position: Vector3::new(0.2, 0.1, -0.05),
            velocity: Vector3::new(-0.1, 0.25, 0.08),
        };
        let between = NavStateBetweenFactor::new(
            VariableId(0),
            VariableId(1),
            between_measurement,
            Matrix9::identity(),
        );
        let between_values = vec![value_i.clone(), value_j.clone()];
        let between_refs = between_values.iter().collect::<Vec<_>>();
        let between_evaluation = between.evaluate(&between_refs).unwrap();
        for variable in 0..2 {
            let numerical = numerical_factor_jacobian(&between, &between_values, variable, true);
            assert!(
                (&between_evaluation.jacobians[variable] - numerical).norm() < 1.0e-5,
                "navigation between analytic Jacobian {variable} disagrees with finite differences"
            );
        }

        let position_velocity =
            PositionVelocityFactor::new(VariableId(0), VariableId(1), 0.2, Matrix3::identity())
                .unwrap();
        let pv_evaluation = position_velocity.evaluate(&between_refs).unwrap();
        for variable in 0..2 {
            let numerical =
                numerical_factor_jacobian(&position_velocity, &between_values, variable, true);
            assert!(
                (&pv_evaluation.jacobians[variable] - numerical).norm() < 1.0e-7,
                "position/velocity analytic Jacobian {variable} disagrees with finite differences"
            );
        }
    }

    #[test]
    fn bias_factors_match_mathematical_robotics_residuals() {
        let bias_i = ImuBias {
            accelerometer: Vector3::new(0.1, -0.2, 0.3),
            gyroscope: Vector3::new(-0.01, 0.02, 0.03),
        };
        let bias_j = ImuBias {
            accelerometer: Vector3::new(0.08, -0.18, 0.25),
            gyroscope: Vector3::new(-0.02, 0.01, 0.04),
        };
        let prior = BiasPriorFactor::new(VariableId(0), bias_j, Matrix6::identity());
        let prior_value = bias_i.vector();
        let prior_evaluation = prior.evaluate(&[&prior_value]).unwrap();
        assert!((prior_evaluation.residual - (bias_i.vector() - bias_j.vector())).norm() < 1.0e-12);

        let between = BiasBetweenFactor::new(VariableId(0), VariableId(1), Matrix6::identity());
        let value_i = bias_i.vector();
        let value_j = bias_j.vector();
        let evaluation = between.evaluate(&[&value_i, &value_j]).unwrap();
        assert!((evaluation.residual - (bias_i.vector() - bias_j.vector())).norm() < 1.0e-12);
        let values = vec![value_i, value_j];
        for variable in 0..2 {
            let numerical = numerical_factor_jacobian(&between, &values, variable, false);
            assert!(
                (&evaluation.jacobians[variable] - numerical).norm() < 1.0e-9,
                "bias-between analytic Jacobian {variable} disagrees with finite differences"
            );
        }
    }

    #[test]
    fn stationary_imu_prediction_cancels_gravity() {
        let mut measurement =
            PreintegratedImuMeasurement::new(ImuBias::zero(), ImuNoise::default());
        for _ in 0..100 {
            measurement
                .integrate(Vector3::new(0.0, 0.0, 9.81), Vector3::zeros(), 0.01)
                .unwrap();
        }
        let predicted = measurement.predict(
            &NavState::identity(),
            &ImuBias::zero(),
            Vector3::new(0.0, 0.0, -9.81),
        );
        assert!(predicted.position.norm() < 1.0e-9);
        assert!(predicted.velocity.norm() < 1.0e-9);
        assert!(measurement.covariance.trace() > 0.0);
    }

    #[test]
    fn imu_factor_has_zero_residual_for_prediction() {
        let mut measurement =
            PreintegratedImuMeasurement::new(ImuBias::zero(), ImuNoise::default());
        measurement
            .integrate(
                Vector3::new(0.2, -0.1, 9.81),
                Vector3::new(0.01, 0.02, -0.01),
                0.1,
            )
            .unwrap();
        let state_i = NavState {
            rotation: so3_exp(&Vector3::new(0.2, -0.1, 0.3)),
            position: Vector3::new(1.0, -2.0, 0.5),
            velocity: Vector3::new(0.4, 0.1, -0.2),
        };
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let state_j = measurement.predict(&state_i, &ImuBias::zero(), gravity);
        let mut problem = Problem::new();
        let id_i = problem.add_variable(nav_state_variable(&state_i));
        let id_j = problem.add_variable(nav_state_variable(&state_j));
        let bias_id = problem.add_variable(imu_bias_variable(&ImuBias::zero()));
        let factor = ImuFactor::new(id_i, id_j, bias_id, measurement, gravity);
        let values = [
            problem.variable(id_i).unwrap().value(),
            problem.variable(id_j).unwrap().value(),
            problem.variable(bias_id).unwrap().value(),
        ];
        let evaluation = factor.evaluate(&values).unwrap();
        assert!(evaluation.residual.norm() < 1.0e-9);
        assert_eq!(evaluation.jacobians[0].shape(), (9, 9));
        assert_eq!(evaluation.jacobians[2].shape(), (9, 6));
        for variable in 0..3 {
            let numerical = factor.numerical_jacobian(&values, variable);
            assert!(
                (&evaluation.jacobians[variable] - numerical).norm() < 1.0e-5,
                "analytic IMU Jacobian {variable} disagrees with finite differences"
            );
        }
    }

    #[test]
    fn trajectory_optimizer_preserves_consistent_prediction() {
        let bias = ImuBias {
            accelerometer: Vector3::new(0.01, -0.02, 0.03),
            gyroscope: Vector3::new(0.001, -0.002, 0.003),
        };
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let initial = NavState::identity();
        let mut measurement = PreintegratedImuMeasurement::new(bias, ImuNoise::default());
        measurement
            .integrate(
                Vector3::new(0.1, -0.2, 9.9),
                Vector3::new(0.02, 0.01, -0.03),
                0.1,
            )
            .unwrap();
        let predicted = measurement.predict(&initial, &bias, gravity);
        let result = optimize_imu_trajectory(
            &[initial.clone(), predicted.clone()],
            bias,
            &[measurement],
            gravity,
            &ImuTrajectoryConfig::default(),
        )
        .unwrap();
        assert!(nav_state_local(&result.states[0], &initial).norm() < 1.0e-9);
        assert!(nav_state_local(&result.states[1], &predicted).norm() < 1.0e-9);
        assert!((result.biases[0].vector() - bias.vector()).norm() < 1.0e-9);
        assert!((result.biases[1].vector() - bias.vector()).norm() < 1.0e-9);
    }

    #[test]
    fn visual_state_priors_make_constant_accelerometer_bias_observable() {
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let true_bias = ImuBias {
            accelerometer: Vector3::new(0.2, -0.1, 0.05),
            gyroscope: Vector3::zeros(),
        };
        let linearization_bias = ImuBias::zero();
        let initial = NavState::identity();
        let mut states = vec![initial.clone()];
        let mut measurements = Vec::new();
        for _ in 0..4 {
            let mut measurement =
                PreintegratedImuMeasurement::new(linearization_bias, ImuNoise::default());
            for _ in 0..10 {
                measurement
                    .integrate(
                        Vector3::new(0.0, 0.0, 9.81) + true_bias.accelerometer,
                        Vector3::zeros(),
                        0.01,
                    )
                    .unwrap();
            }
            states.push(measurement.predict(states.last().unwrap(), &true_bias, gravity));
            measurements.push(measurement);
        }
        let mut visual_information = Matrix9::zeros();
        visual_information
            .fixed_view_mut::<6, 6>(0, 0)
            .copy_from(&(SMatrix::<f64, 6, 6>::identity() * 1.0e8));
        let result = optimize_imu_trajectory(
            &states,
            linearization_bias,
            &measurements,
            gravity,
            &ImuTrajectoryConfig {
                nav_measurement_information: Some(visual_information),
                bias_prior_information: Matrix6::identity() * 1.0e-6,
                bias_between_information: Matrix6::identity() * 1.0e6,
                ..ImuTrajectoryConfig::default()
            },
        )
        .unwrap();
        assert!(
            (result.biases[0].accelerometer - true_bias.accelerometer).norm() < 1.0e-4,
            "estimated bias {:?}",
            result.biases[0].accelerometer
        );
    }
}
