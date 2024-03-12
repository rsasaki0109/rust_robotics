// grid map definition
// author:Salah Eddine Ghamri (s.ghamri)

use std::ops::Deref;
extern crate nalgebra as na;

pub struct Map {
    grid: na::DMatrix<i32>,
}

impl Map {
    pub fn new(original_matrix: na::DMatrix<i32>, scale: usize) -> Result<Self, &'static str> {
        if scale < 1 {
            return Err("scale must be >= 1");
        }
        let grid = original_matrix.kronecker(&na::DMatrix::<i32>::repeat(scale, scale, 1));
        Ok(Self { grid })
    }
}

impl Deref for Map {
    type Target = na::DMatrix<i32>;

    fn deref(&self) -> &Self::Target {
        &self.grid
    }
}
