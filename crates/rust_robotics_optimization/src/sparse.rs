use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector};

use crate::{OptimizationError, OptimizationResult};

/// Upper-triangular block-sparse symmetric matrix.
pub(crate) struct BlockSparseHessian {
    dimension: usize,
    blocks: BTreeMap<(usize, usize), DMatrix<f64>>,
}

impl BlockSparseHessian {
    pub(crate) fn new(dimension: usize) -> Self {
        Self {
            dimension,
            blocks: BTreeMap::new(),
        }
    }

    pub(crate) fn add_block(
        &mut self,
        row_offset: usize,
        column_offset: usize,
        block: DMatrix<f64>,
    ) {
        debug_assert!(row_offset <= column_offset);
        self.blocks
            .entry((row_offset, column_offset))
            .and_modify(|current| *current += &block)
            .or_insert(block);
    }

    pub(crate) fn add_levenberg_damping(&mut self, damping: f64) {
        for ((row_offset, column_offset), block) in &mut self.blocks {
            if row_offset == column_offset {
                for index in 0..block.nrows().min(block.ncols()) {
                    block[(index, index)] += damping * block[(index, index)].abs().max(1.0);
                }
            }
        }
    }

    pub(crate) fn block_count(&self) -> usize {
        self.blocks.len()
    }

    pub(crate) fn scalar_entries(&self) -> usize {
        self.blocks.values().map(DMatrix::len).sum()
    }

    pub(crate) fn to_dense(&self) -> DMatrix<f64> {
        let mut dense = DMatrix::zeros(self.dimension, self.dimension);
        for (&(row_offset, column_offset), block) in &self.blocks {
            dense
                .view_mut((row_offset, column_offset), (block.nrows(), block.ncols()))
                .copy_from(block);
            if row_offset != column_offset {
                dense
                    .view_mut((column_offset, row_offset), (block.ncols(), block.nrows()))
                    .copy_from(&block.transpose());
            }
        }
        dense
    }

    fn multiply(&self, vector: &DVector<f64>) -> DVector<f64> {
        let mut result = DVector::zeros(self.dimension);
        for (&(row_offset, column_offset), block) in &self.blocks {
            let column = vector.rows(column_offset, block.ncols());
            result
                .rows_mut(row_offset, block.nrows())
                .add_assign(block * column);
            if row_offset != column_offset {
                let row = vector.rows(row_offset, block.nrows());
                result
                    .rows_mut(column_offset, block.ncols())
                    .add_assign(block.transpose() * row);
            }
        }
        result
    }

    fn inverse_diagonal(&self) -> DVector<f64> {
        let mut diagonal = DVector::from_element(self.dimension, 1.0);
        for (&(row_offset, column_offset), block) in &self.blocks {
            if row_offset == column_offset {
                for index in 0..block.nrows().min(block.ncols()) {
                    diagonal[row_offset + index] = 1.0 / block[(index, index)].abs().max(1.0e-12);
                }
            }
        }
        diagonal
    }

    pub(crate) fn solve_pcg(
        &self,
        right_hand_side: &DVector<f64>,
        max_iterations: usize,
        tolerance: f64,
    ) -> OptimizationResult<(DVector<f64>, usize)> {
        let mut solution = DVector::zeros(self.dimension);
        let mut residual = right_hand_side.clone();
        let inverse_diagonal = self.inverse_diagonal();
        let mut preconditioned = inverse_diagonal.component_mul(&residual);
        let mut direction = preconditioned.clone();
        let mut residual_dot_preconditioned = residual.dot(&preconditioned);
        let threshold = tolerance * (1.0 + right_hand_side.norm());

        if residual.norm() <= threshold {
            return Ok((solution, 0));
        }

        for iteration in 0..max_iterations {
            let matrix_direction = self.multiply(&direction);
            let denominator = direction.dot(&matrix_direction);
            // The system can be well-conditioned but very small near nonlinear
            // convergence, so an absolute epsilon is not a valid breakdown
            // threshold here. PCG only requires positive curvature.
            if !denominator.is_finite() || denominator <= 0.0 {
                return Err(OptimizationError::LinearSolveFailed);
            }
            let alpha = residual_dot_preconditioned / denominator;
            solution += alpha * &direction;
            residual -= alpha * matrix_direction;
            if residual.norm() <= threshold {
                return Ok((solution, iteration + 1));
            }
            preconditioned = inverse_diagonal.component_mul(&residual);
            let next_dot = residual.dot(&preconditioned);
            if !next_dot.is_finite() {
                return Err(OptimizationError::NonFiniteValue);
            }
            let beta = next_dot / residual_dot_preconditioned;
            direction = &preconditioned + beta * direction;
            residual_dot_preconditioned = next_dot;
        }
        Err(OptimizationError::LinearSolveFailed)
    }

    pub(crate) fn solve_schur(
        &self,
        right_hand_side: &DVector<f64>,
        retained_dimension: usize,
    ) -> OptimizationResult<DVector<f64>> {
        if retained_dimension > self.dimension || right_hand_side.len() != self.dimension {
            return Err(OptimizationError::InvalidParameter(
                "Schur partition is outside the linear system".into(),
            ));
        }

        let mut reduced = DMatrix::zeros(retained_dimension, retained_dimension);
        for (&(row, column), block) in &self.blocks {
            if column < retained_dimension {
                reduced
                    .view_mut((row, column), (block.nrows(), block.ncols()))
                    .copy_from(block);
                if row != column {
                    reduced
                        .view_mut((column, row), (block.ncols(), block.nrows()))
                        .copy_from(&block.transpose());
                }
            } else if row >= retained_dimension && row != column {
                return Err(OptimizationError::InvalidParameter(
                    "Schur-eliminated variables must not have off-diagonal coupling".into(),
                ));
            }
        }

        let mut reduced_rhs = right_hand_side.rows(0, retained_dimension).into_owned();
        struct EliminatedBlock {
            offset: usize,
            inverse: DMatrix<f64>,
            cross: DMatrix<f64>,
        }
        let mut eliminated = Vec::new();
        for (&(offset, column), diagonal) in &self.blocks {
            if offset != column || offset < retained_dimension {
                continue;
            }
            let inverse = diagonal
                .clone()
                .try_inverse()
                .ok_or(OptimizationError::LinearSolveFailed)?;
            let mut cross = DMatrix::zeros(retained_dimension, diagonal.nrows());
            for (&(row, block_column), block) in &self.blocks {
                if row < retained_dimension && block_column == offset {
                    cross
                        .view_mut((row, 0), (block.nrows(), block.ncols()))
                        .copy_from(block);
                }
            }
            let rhs = right_hand_side.rows(offset, diagonal.nrows()).into_owned();
            reduced -= &cross * &inverse * cross.transpose();
            reduced_rhs -= &cross * &inverse * rhs;
            eliminated.push(EliminatedBlock {
                offset,
                inverse,
                cross,
            });
        }

        let retained = if retained_dimension == 0 {
            DVector::zeros(0)
        } else {
            reduced
                .lu()
                .solve(&reduced_rhs)
                .ok_or(OptimizationError::LinearSolveFailed)?
        };
        let mut solution = DVector::zeros(self.dimension);
        solution
            .rows_mut(0, retained_dimension)
            .copy_from(&retained);
        for block in eliminated {
            let dimension = block.inverse.nrows();
            let rhs = right_hand_side.rows(block.offset, dimension).into_owned();
            let value = &block.inverse * (rhs - block.cross.transpose() * &retained);
            solution.rows_mut(block.offset, dimension).copy_from(&value);
        }
        Ok(solution)
    }
}

use core::ops::AddAssign;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_sparse_pcg_matches_dense_solution() {
        let mut matrix = BlockSparseHessian::new(4);
        matrix.add_block(0, 0, DMatrix::from_diagonal_element(2, 2, 4.0));
        matrix.add_block(0, 2, DMatrix::from_element(2, 2, 0.2));
        matrix.add_block(2, 2, DMatrix::from_diagonal_element(2, 2, 3.0));
        let rhs = DVector::from_vec(vec![1.0, 2.0, -1.0, 0.5]);
        let dense_solution = matrix.to_dense().lu().solve(&rhs).unwrap();
        let (sparse_solution, _) = matrix.solve_pcg(&rhs, 50, 1.0e-12).unwrap();
        assert!((dense_solution - sparse_solution).norm() < 1.0e-9);
    }

    #[test]
    fn schur_complement_matches_dense_solution() {
        let mut matrix = BlockSparseHessian::new(8);
        matrix.add_block(0, 0, DMatrix::from_diagonal_element(2, 2, 5.0));
        matrix.add_block(0, 2, DMatrix::from_element(2, 3, 0.2));
        matrix.add_block(0, 5, DMatrix::from_element(2, 3, -0.1));
        matrix.add_block(2, 2, DMatrix::from_diagonal_element(3, 3, 4.0));
        matrix.add_block(5, 5, DMatrix::from_diagonal_element(3, 3, 3.0));
        let rhs = DVector::from_iterator(8, (0..8).map(|index| index as f64 - 2.0));
        let dense = matrix.to_dense().lu().solve(&rhs).unwrap();
        let schur = matrix.solve_schur(&rhs, 2).unwrap();
        assert!((dense - schur).norm() < 1.0e-10);
    }
}
