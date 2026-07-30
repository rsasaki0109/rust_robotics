//! Bias-aware IMU preintegration with covariance propagation.
//!
//! Error states use `[rotation, position, velocity]`; biases use
//! `[accelerometer, gyroscope]`.

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, Vector3};
use rust_robotics_core::{skew, so3_exp, so3_log};
use rust_robotics_optimization::{
    Factor, FactorEvaluation, OptimizationError, OptimizationResult, Variable, VariableId,
};

pub type Matrix9 = SMatrix<f64, 9, 9>;
pub type Matrix9x6 = SMatrix<f64, 9, 6>;
pub type Matrix6 = SMatrix<f64, 6, 6>;

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

/// Creates a 9-DOF manifold variable for a navigation state.
pub fn nav_state_variable(state: &NavState) -> Variable {
    Variable::with_retraction(encode_nav_state(state), 9, retract_nav_state)
        .expect("navigation state dimensions are valid")
}

/// Creates a Euclidean six-dimensional bias variable.
pub fn imu_bias_variable(bias: &ImuBias) -> Variable {
    Variable::euclidean(bias.vector())
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
        let jacobians = (0..3)
            .map(|variable| self.numerical_jacobian(values, variable))
            .collect::<Vec<_>>();
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
    }
}
