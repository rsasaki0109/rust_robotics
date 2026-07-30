use nalgebra::{DMatrix, DVector};

use crate::{OptimizationError, OptimizationResult, RobustKernel};

/// Stable index of a variable in a [`Problem`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VariableId(pub usize);

/// Retraction used to apply a tangent increment to a variable.
pub type Retraction =
    fn(value: &DVector<f64>, increment: &DVector<f64>) -> OptimizationResult<DVector<f64>>;

/// An optimizable value and its local update rule.
pub struct Variable {
    value: DVector<f64>,
    tangent_dimension: usize,
    fixed: bool,
    retraction: Retraction,
}

impl Variable {
    /// Creates a Euclidean variable whose update is vector addition.
    pub fn euclidean(value: DVector<f64>) -> Self {
        let tangent_dimension = value.len();
        Self {
            value,
            tangent_dimension,
            fixed: false,
            retraction: euclidean_retraction,
        }
    }

    /// Creates a variable with a custom manifold retraction.
    pub fn with_retraction(
        value: DVector<f64>,
        tangent_dimension: usize,
        retraction: Retraction,
    ) -> OptimizationResult<Self> {
        if value.is_empty() || tangent_dimension == 0 {
            return Err(OptimizationError::InvalidParameter(
                "variable dimensions must be non-zero".into(),
            ));
        }
        Ok(Self {
            value,
            tangent_dimension,
            fixed: false,
            retraction,
        })
    }

    pub fn value(&self) -> &DVector<f64> {
        &self.value
    }

    pub fn tangent_dimension(&self) -> usize {
        self.tangent_dimension
    }

    pub fn is_fixed(&self) -> bool {
        self.fixed
    }

    pub fn set_fixed(&mut self, fixed: bool) {
        self.fixed = fixed;
    }

    pub(crate) fn apply(&mut self, increment: &DVector<f64>) -> OptimizationResult<()> {
        if increment.len() != self.tangent_dimension {
            return Err(OptimizationError::InvalidParameter(format!(
                "increment dimension {}, expected {}",
                increment.len(),
                self.tangent_dimension
            )));
        }
        let updated = (self.retraction)(&self.value, increment)?;
        if updated.len() != self.value.len() || !updated.iter().all(|value| value.is_finite()) {
            return Err(OptimizationError::NonFiniteValue);
        }
        self.value = updated;
        Ok(())
    }

    pub(crate) fn restore(&mut self, value: DVector<f64>) {
        self.value = value;
    }
}

/// Residual, information matrix, and one Jacobian block per linked variable.
pub struct FactorEvaluation {
    pub residual: DVector<f64>,
    pub information: DMatrix<f64>,
    pub jacobians: Vec<DMatrix<f64>>,
}

impl FactorEvaluation {
    pub fn unit_weighted(residual: DVector<f64>, jacobians: Vec<DMatrix<f64>>) -> Self {
        let information = DMatrix::identity(residual.len(), residual.len());
        Self {
            residual,
            information,
            jacobians,
        }
    }
}

/// A residual block linking one or more problem variables.
pub trait Factor {
    fn variable_ids(&self) -> &[VariableId];
    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation>;

    fn robust_kernel(&self) -> RobustKernel {
        RobustKernel::L2
    }
}

/// Collection of variables and residual factors.
#[derive(Default)]
pub struct Problem {
    variables: Vec<Variable>,
    factors: Vec<Box<dyn Factor>>,
}

impl Problem {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_variable(&mut self, variable: Variable) -> VariableId {
        let id = VariableId(self.variables.len());
        self.variables.push(variable);
        id
    }

    pub fn variable(&self, id: VariableId) -> OptimizationResult<&Variable> {
        self.variables.get(id.0).ok_or_else(|| {
            OptimizationError::InvalidParameter(format!("unknown variable {}", id.0))
        })
    }

    pub fn variable_mut(&mut self, id: VariableId) -> OptimizationResult<&mut Variable> {
        self.variables.get_mut(id.0).ok_or_else(|| {
            OptimizationError::InvalidParameter(format!("unknown variable {}", id.0))
        })
    }

    pub fn add_factor<F: Factor + 'static>(&mut self, factor: F) -> OptimizationResult<()> {
        if factor.variable_ids().is_empty() {
            return Err(OptimizationError::InvalidFactor(
                "a factor must link at least one variable".into(),
            ));
        }
        for id in factor.variable_ids() {
            self.variable(*id)?;
        }
        self.factors.push(Box::new(factor));
        Ok(())
    }

    pub(crate) fn variables(&self) -> &[Variable] {
        &self.variables
    }

    pub(crate) fn variables_mut(&mut self) -> &mut [Variable] {
        &mut self.variables
    }

    pub(crate) fn factors(&self) -> &[Box<dyn Factor>] {
        &self.factors
    }
}

fn euclidean_retraction(
    value: &DVector<f64>,
    increment: &DVector<f64>,
) -> OptimizationResult<DVector<f64>> {
    if value.len() != increment.len() {
        return Err(OptimizationError::InvalidParameter(
            "Euclidean value and increment dimensions differ".into(),
        ));
    }
    Ok(value + increment)
}
