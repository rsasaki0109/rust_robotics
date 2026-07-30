#![forbid(unsafe_code)]
//! Nonlinear least-squares and factor-graph optimization.

mod error;
mod graph;
mod loss;
mod solver;

pub use error::{OptimizationError, OptimizationResult};
pub use graph::{Factor, FactorEvaluation, Problem, Variable, VariableId};
pub use loss::{LossEvaluation, RobustKernel};
pub use solver::{solve, SolverConfig, SolverMethod, SolverSummary, TerminationReason};
