use core::ops::AddAssign;

use nalgebra::DVector;

use crate::sparse::BlockSparseHessian;
use crate::{OptimizationError, OptimizationResult, Problem};

/// Nonlinear least-squares algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverMethod {
    GaussNewton,
    LevenbergMarquardt,
}

/// Linear solver used for each nonlinear iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LinearSolver {
    /// Materialize the block Hessian as a dense matrix and use LU.
    Dense,
    /// Keep the Hessian block-sparse and solve it with preconditioned
    /// conjugate gradients.
    BlockSparsePcg {
        max_iterations: usize,
        tolerance: f64,
    },
    /// Eliminate independent trailing variable blocks, then solve the reduced
    /// retained system densely. This is the classic bundle-adjustment Schur
    /// complement when cameras precede landmarks.
    SchurComplement { retained_dimension: usize },
}

/// Solver limits and convergence tolerances.
#[derive(Debug, Clone, Copy)]
pub struct SolverConfig {
    pub method: SolverMethod,
    pub max_iterations: usize,
    pub gradient_tolerance: f64,
    pub step_tolerance: f64,
    pub cost_tolerance: f64,
    pub initial_damping: f64,
    pub linear_solver: LinearSolver,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            method: SolverMethod::LevenbergMarquardt,
            max_iterations: 50,
            gradient_tolerance: 1.0e-10,
            step_tolerance: 1.0e-10,
            cost_tolerance: 1.0e-12,
            initial_damping: 1.0e-3,
            linear_solver: LinearSolver::Dense,
        }
    }
}

/// Reason the nonlinear solve stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminationReason {
    GradientConverged,
    StepConverged,
    CostConverged,
    MaxIterations,
}

/// Diagnostics returned by [`solve`].
#[derive(Debug, Clone)]
pub struct SolverSummary {
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub termination: TerminationReason,
    pub linear_iterations: usize,
    pub hessian_blocks: usize,
    pub hessian_scalar_entries: usize,
}

/// Solves a factor graph in place.
pub fn solve(problem: &mut Problem, config: &SolverConfig) -> OptimizationResult<SolverSummary> {
    validate_config(config)?;
    let layout = Layout::new(problem);
    if layout.dimension == 0 {
        let cost = evaluate_cost(problem)?;
        return Ok(SolverSummary {
            initial_cost: cost,
            final_cost: cost,
            iterations: 0,
            accepted_steps: 0,
            termination: TerminationReason::GradientConverged,
            linear_iterations: 0,
            hessian_blocks: 0,
            hessian_scalar_entries: 0,
        });
    }

    let initial_cost = evaluate_cost(problem)?;
    let mut current_cost = initial_cost;
    let mut damping = config.initial_damping;
    let mut accepted_steps = 0;
    let mut total_linear_iterations = 0;
    let mut hessian_blocks = 0;
    let mut hessian_scalar_entries = 0;

    for iteration in 0..config.max_iterations {
        let mut linearization = linearize(problem, &layout)?;
        hessian_blocks = linearization.hessian.block_count();
        hessian_scalar_entries = linearization.hessian.scalar_entries();
        if linearization.gradient.amax() <= config.gradient_tolerance {
            return Ok(summary(
                initial_cost,
                current_cost,
                iteration,
                accepted_steps,
                TerminationReason::GradientConverged,
                total_linear_iterations,
                hessian_blocks,
                hessian_scalar_entries,
            ));
        }

        if config.method == SolverMethod::LevenbergMarquardt {
            linearization.hessian.add_levenberg_damping(damping);
        }

        let right_hand_side = -linearization.gradient;
        let (increment, linear_iterations) = solve_linear_system(
            &linearization.hessian,
            &right_hand_side,
            config.linear_solver,
        )?;
        total_linear_iterations += linear_iterations;
        if !increment.iter().all(|value| value.is_finite()) {
            return Err(OptimizationError::NonFiniteValue);
        }
        if increment.norm() <= config.step_tolerance {
            return Ok(summary(
                initial_cost,
                current_cost,
                iteration + 1,
                accepted_steps,
                TerminationReason::StepConverged,
                total_linear_iterations,
                hessian_blocks,
                hessian_scalar_entries,
            ));
        }

        let snapshot = snapshot_values(problem);
        apply_increment(problem, &layout, &increment)?;
        let trial_cost = evaluate_cost(problem)?;
        let accepted = config.method == SolverMethod::GaussNewton || trial_cost < current_cost;

        if accepted {
            accepted_steps += 1;
            let cost_change = (current_cost - trial_cost).abs();
            current_cost = trial_cost;
            damping = (damping * 0.3).max(1.0e-15);
            if cost_change <= config.cost_tolerance {
                return Ok(summary(
                    initial_cost,
                    current_cost,
                    iteration + 1,
                    accepted_steps,
                    TerminationReason::CostConverged,
                    total_linear_iterations,
                    hessian_blocks,
                    hessian_scalar_entries,
                ));
            }
        } else {
            restore_values(problem, snapshot);
            damping = (damping * 10.0).min(1.0e15);
        }
    }

    Ok(summary(
        initial_cost,
        current_cost,
        config.max_iterations,
        accepted_steps,
        TerminationReason::MaxIterations,
        total_linear_iterations,
        hessian_blocks,
        hessian_scalar_entries,
    ))
}

struct Layout {
    offsets: Vec<Option<usize>>,
    dimension: usize,
}

impl Layout {
    fn new(problem: &Problem) -> Self {
        let mut offsets = Vec::with_capacity(problem.variables().len());
        let mut dimension = 0;
        for variable in problem.variables() {
            if variable.is_fixed() {
                offsets.push(None);
            } else {
                offsets.push(Some(dimension));
                dimension += variable.tangent_dimension();
            }
        }
        Self { offsets, dimension }
    }
}

struct Linearization {
    hessian: BlockSparseHessian,
    gradient: DVector<f64>,
}

fn linearize(problem: &Problem, layout: &Layout) -> OptimizationResult<Linearization> {
    let mut hessian = BlockSparseHessian::new(layout.dimension);
    let mut gradient = DVector::zeros(layout.dimension);

    for factor in problem.factors() {
        let ids = factor.variable_ids();
        let values = ids
            .iter()
            .map(|id| problem.variable(*id).map(|variable| variable.value()))
            .collect::<OptimizationResult<Vec<_>>>()?;
        let evaluation = factor.evaluate(&values)?;
        validate_factor(problem, ids, &evaluation)?;
        let weighted_residual = &evaluation.information * &evaluation.residual;
        let squared_error = evaluation.residual.dot(&weighted_residual);
        let weight = factor
            .robust_kernel()
            .evaluate(squared_error)
            .first_derivative;

        for (local_i, id_i) in ids.iter().enumerate() {
            let Some(offset_i) = layout.offsets[id_i.0] else {
                continue;
            };
            let jacobian_i = &evaluation.jacobians[local_i];
            let dimension_i = jacobian_i.ncols();
            let gradient_i = weight * jacobian_i.transpose() * &weighted_residual;
            gradient
                .rows_mut(offset_i, dimension_i)
                .add_assign(&gradient_i);

            for (local_j, id_j) in ids.iter().enumerate() {
                let Some(offset_j) = layout.offsets[id_j.0] else {
                    continue;
                };
                let jacobian_j = &evaluation.jacobians[local_j];
                let block = weight * jacobian_i.transpose() * &evaluation.information * jacobian_j;
                if offset_i <= offset_j {
                    hessian.add_block(offset_i, offset_j, block);
                }
            }
        }
    }
    Ok(Linearization { hessian, gradient })
}

fn evaluate_cost(problem: &Problem) -> OptimizationResult<f64> {
    let mut cost = 0.0;
    for factor in problem.factors() {
        let values = factor
            .variable_ids()
            .iter()
            .map(|id| problem.variable(*id).map(|variable| variable.value()))
            .collect::<OptimizationResult<Vec<_>>>()?;
        let evaluation = factor.evaluate(&values)?;
        validate_factor(problem, factor.variable_ids(), &evaluation)?;
        let squared_error = evaluation
            .residual
            .dot(&(&evaluation.information * &evaluation.residual));
        cost += 0.5 * factor.robust_kernel().evaluate(squared_error).value;
    }
    if cost.is_finite() {
        Ok(cost)
    } else {
        Err(OptimizationError::NonFiniteValue)
    }
}

fn validate_factor(
    problem: &Problem,
    ids: &[crate::VariableId],
    evaluation: &crate::FactorEvaluation,
) -> OptimizationResult<()> {
    let residual_dimension = evaluation.residual.len();
    if residual_dimension == 0
        || evaluation.information.nrows() != residual_dimension
        || evaluation.information.ncols() != residual_dimension
        || evaluation.jacobians.len() != ids.len()
    {
        return Err(OptimizationError::InvalidFactor(
            "residual, information, or Jacobian block count is inconsistent".into(),
        ));
    }
    for (id, jacobian) in ids.iter().zip(&evaluation.jacobians) {
        let variable = problem.variable(*id)?;
        if jacobian.nrows() != residual_dimension
            || jacobian.ncols() != variable.tangent_dimension()
        {
            return Err(OptimizationError::InvalidFactor(format!(
                "Jacobian for variable {} has shape {}x{}, expected {}x{}",
                id.0,
                jacobian.nrows(),
                jacobian.ncols(),
                residual_dimension,
                variable.tangent_dimension()
            )));
        }
    }
    Ok(())
}

fn apply_increment(
    problem: &mut Problem,
    layout: &Layout,
    increment: &DVector<f64>,
) -> OptimizationResult<()> {
    for (index, variable) in problem.variables_mut().iter_mut().enumerate() {
        if let Some(offset) = layout.offsets[index] {
            let local = increment
                .rows(offset, variable.tangent_dimension())
                .into_owned();
            variable.apply(&local)?;
        }
    }
    Ok(())
}

fn snapshot_values(problem: &Problem) -> Vec<DVector<f64>> {
    problem
        .variables()
        .iter()
        .map(|variable| variable.value().clone())
        .collect()
}

fn restore_values(problem: &mut Problem, values: Vec<DVector<f64>>) {
    for (variable, value) in problem.variables_mut().iter_mut().zip(values) {
        variable.restore(value);
    }
}

fn validate_config(config: &SolverConfig) -> OptimizationResult<()> {
    if config.max_iterations == 0
        || config.gradient_tolerance < 0.0
        || config.step_tolerance < 0.0
        || config.cost_tolerance < 0.0
        || config.initial_damping <= 0.0
        || matches!(
            config.linear_solver,
            LinearSolver::BlockSparsePcg {
                max_iterations: 0,
                ..
            }
        )
        || matches!(
            config.linear_solver,
            LinearSolver::BlockSparsePcg { tolerance, .. } if tolerance <= 0.0
        )
    {
        return Err(OptimizationError::InvalidParameter(
            "solver limits and tolerances must be positive".into(),
        ));
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn summary(
    initial_cost: f64,
    final_cost: f64,
    iterations: usize,
    accepted_steps: usize,
    termination: TerminationReason,
    linear_iterations: usize,
    hessian_blocks: usize,
    hessian_scalar_entries: usize,
) -> SolverSummary {
    SolverSummary {
        initial_cost,
        final_cost,
        iterations,
        accepted_steps,
        termination,
        linear_iterations,
        hessian_blocks,
        hessian_scalar_entries,
    }
}

fn solve_linear_system(
    hessian: &BlockSparseHessian,
    right_hand_side: &DVector<f64>,
    solver: LinearSolver,
) -> OptimizationResult<(DVector<f64>, usize)> {
    match solver {
        LinearSolver::Dense => hessian
            .to_dense()
            .lu()
            .solve(right_hand_side)
            .map(|solution| (solution, 1))
            .ok_or(OptimizationError::LinearSolveFailed),
        LinearSolver::BlockSparsePcg {
            max_iterations,
            tolerance,
        } => hessian.solve_pcg(right_hand_side, max_iterations, tolerance),
        LinearSolver::SchurComplement { retained_dimension } => hessian
            .solve_schur(right_hand_side, retained_dimension)
            .map(|solution| (solution, 1)),
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, DVector};

    use super::*;
    use crate::{Factor, FactorEvaluation, Variable, VariableId};

    struct RosenbrockFactor {
        variables: [VariableId; 1],
    }

    impl Factor for RosenbrockFactor {
        fn variable_ids(&self) -> &[VariableId] {
            &self.variables
        }

        fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
            let x = values[0][0];
            let y = values[0][1];
            Ok(FactorEvaluation::unit_weighted(
                DVector::from_vec(vec![10.0 * (y - x * x), 1.0 - x]),
                vec![DMatrix::from_row_slice(2, 2, &[-20.0 * x, 10.0, -1.0, 0.0])],
            ))
        }
    }

    #[test]
    fn levenberg_marquardt_solves_rosenbrock_problem() {
        let mut problem = Problem::new();
        let variable =
            problem.add_variable(Variable::euclidean(DVector::from_vec(vec![-1.2, 1.0])));
        problem
            .add_factor(RosenbrockFactor {
                variables: [variable],
            })
            .unwrap();

        let summary = solve(&mut problem, &SolverConfig::default()).unwrap();
        let solution = problem.variable(variable).unwrap().value();
        assert!((solution[0] - 1.0).abs() < 1.0e-7);
        assert!((solution[1] - 1.0).abs() < 1.0e-7);
        assert!(summary.final_cost < summary.initial_cost);
    }

    #[test]
    fn block_sparse_solver_solves_rosenbrock_problem() {
        let mut problem = Problem::new();
        let variable =
            problem.add_variable(Variable::euclidean(DVector::from_vec(vec![-1.2, 1.0])));
        problem
            .add_factor(RosenbrockFactor {
                variables: [variable],
            })
            .unwrap();
        let config = SolverConfig {
            linear_solver: LinearSolver::BlockSparsePcg {
                max_iterations: 100,
                tolerance: 1.0e-12,
            },
            ..SolverConfig::default()
        };
        let summary = solve(&mut problem, &config).unwrap();
        let solution = problem.variable(variable).unwrap().value();
        assert!((solution[0] - 1.0).abs() < 1.0e-7);
        assert!((solution[1] - 1.0).abs() < 1.0e-7);
        assert!(summary.hessian_scalar_entries <= 4);
        assert!(summary.linear_iterations > 0);
    }
}
