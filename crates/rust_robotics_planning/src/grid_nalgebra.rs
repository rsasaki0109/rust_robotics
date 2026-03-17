// grid map definition
// author:Salah Eddine Ghamri (s.ghamri)

use std::ops::Deref;

pub struct Map {
    grid: nalgebra::DMatrix<i32>,
}

impl Map {
    pub fn new(original_matrix: nalgebra::DMatrix<i32>, scale: usize) -> Result<Self, &'static str> {
        if scale < 1 {
            return Err("scale must be >= 1");
        }
        let grid = original_matrix.kronecker(&nalgebra::DMatrix::<i32>::repeat(scale, scale, 1));
        Ok(Self { grid })
    }
}

impl Deref for Map {
    type Target = nalgebra::DMatrix<i32>;

    fn deref(&self) -> &Self::Target {
        &self.grid
    }
}
