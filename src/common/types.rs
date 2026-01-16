//! Common types used throughout rust_robotics

use nalgebra::{Vector2, Vector3, Vector4, Matrix2, Matrix4};

/// 2D point representation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn origin() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    pub fn distance(&self, other: &Point2D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn to_vector(&self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
}

impl From<(f64, f64)> for Point2D {
    fn from(tuple: (f64, f64)) -> Self {
        Self { x: tuple.0, y: tuple.1 }
    }
}

impl From<Vector2<f64>> for Point2D {
    fn from(v: Vector2<f64>) -> Self {
        Self { x: v[0], y: v[1] }
    }
}

/// 3D point representation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn origin() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn distance(&self, other: &Point3D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2)).sqrt()
    }

    pub fn to_vector(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

/// 2D pose (position + orientation)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub fn origin() -> Self {
        Self { x: 0.0, y: 0.0, yaw: 0.0 }
    }

    pub fn position(&self) -> Point2D {
        Point2D::new(self.x, self.y)
    }

    pub fn to_vector(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.yaw)
    }

    /// Normalize yaw to [-pi, pi]
    pub fn normalize_yaw(&mut self) {
        while self.yaw > std::f64::consts::PI {
            self.yaw -= 2.0 * std::f64::consts::PI;
        }
        while self.yaw < -std::f64::consts::PI {
            self.yaw += 2.0 * std::f64::consts::PI;
        }
    }
}

impl From<Vector3<f64>> for Pose2D {
    fn from(v: Vector3<f64>) -> Self {
        Self { x: v[0], y: v[1], yaw: v[2] }
    }
}

/// Robot state with pose and velocity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
}

impl State2D {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64) -> Self {
        Self { x, y, yaw, v }
    }

    pub fn origin() -> Self {
        Self { x: 0.0, y: 0.0, yaw: 0.0, v: 0.0 }
    }

    pub fn pose(&self) -> Pose2D {
        Pose2D::new(self.x, self.y, self.yaw)
    }

    pub fn position(&self) -> Point2D {
        Point2D::new(self.x, self.y)
    }

    pub fn to_vector(&self) -> Vector4<f64> {
        Vector4::new(self.x, self.y, self.yaw, self.v)
    }
}

impl From<Vector4<f64>> for State2D {
    fn from(v: Vector4<f64>) -> Self {
        Self { x: v[0], y: v[1], yaw: v[2], v: v[3] }
    }
}

/// Control input for differential drive robot
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlInput {
    pub v: f64,      // linear velocity
    pub omega: f64,  // angular velocity
}

impl ControlInput {
    pub fn new(v: f64, omega: f64) -> Self {
        Self { v, omega }
    }

    pub fn zero() -> Self {
        Self { v: 0.0, omega: 0.0 }
    }

    pub fn to_vector(&self) -> Vector2<f64> {
        Vector2::new(self.v, self.omega)
    }
}

impl From<Vector2<f64>> for ControlInput {
    fn from(v: Vector2<f64>) -> Self {
        Self { v: v[0], omega: v[1] }
    }
}

/// Path represented as a sequence of 2D points
#[derive(Debug, Clone)]
pub struct Path2D {
    pub points: Vec<Point2D>,
}

impl Path2D {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn from_points(points: Vec<Point2D>) -> Self {
        Self { points }
    }

    pub fn from_xy(x: &[f64], y: &[f64]) -> Self {
        assert_eq!(x.len(), y.len());
        let points = x.iter().zip(y.iter())
            .map(|(&x, &y)| Point2D::new(x, y))
            .collect();
        Self { points }
    }

    pub fn push(&mut self, point: Point2D) {
        self.points.push(point);
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn x_coords(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.x).collect()
    }

    pub fn y_coords(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.y).collect()
    }

    pub fn total_length(&self) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }
        self.points.windows(2)
            .map(|w| w[0].distance(&w[1]))
            .sum()
    }
}

impl Default for Path2D {
    fn default() -> Self {
        Self::new()
    }
}

/// Grid node for graph-based planners
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridNode {
    pub x: i32,
    pub y: i32,
}

impl GridNode {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Obstacle representation
#[derive(Debug, Clone)]
pub struct Obstacles {
    pub points: Vec<Point2D>,
}

impl Obstacles {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn from_points(points: Vec<Point2D>) -> Self {
        Self { points }
    }

    pub fn from_xy(x: &[f64], y: &[f64]) -> Self {
        assert_eq!(x.len(), y.len());
        let points = x.iter().zip(y.iter())
            .map(|(&x, &y)| Point2D::new(x, y))
            .collect();
        Self { points }
    }

    pub fn push(&mut self, point: Point2D) {
        self.points.push(point);
    }

    pub fn x_coords(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.x).collect()
    }

    pub fn y_coords(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.y).collect()
    }
}

impl Default for Obstacles {
    fn default() -> Self {
        Self::new()
    }
}

/// Covariance matrix wrapper for 2D state estimation
#[derive(Debug, Clone)]
pub struct Covariance2D(pub Matrix2<f64>);

impl Covariance2D {
    pub fn identity() -> Self {
        Self(Matrix2::identity())
    }

    pub fn from_diagonal(diag: Vector2<f64>) -> Self {
        Self(Matrix2::from_diagonal(&diag))
    }
}

/// Covariance matrix wrapper for 4D state estimation
#[derive(Debug, Clone)]
pub struct Covariance4D(pub Matrix4<f64>);

impl Covariance4D {
    pub fn identity() -> Self {
        Self(Matrix4::identity())
    }

    pub fn from_diagonal(diag: Vector4<f64>) -> Self {
        Self(Matrix4::from_diagonal(&diag))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point2d_distance() {
        let p1 = Point2D::new(0.0, 0.0);
        let p2 = Point2D::new(3.0, 4.0);
        assert!((p1.distance(&p2) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_normalize_yaw() {
        let mut pose = Pose2D::new(0.0, 0.0, 4.0);
        pose.normalize_yaw();
        assert!(pose.yaw >= -std::f64::consts::PI && pose.yaw <= std::f64::consts::PI);
    }

    #[test]
    fn test_path2d_total_length() {
        let path = Path2D::from_xy(&[0.0, 1.0, 1.0], &[0.0, 0.0, 1.0]);
        assert!((path.total_length() - 2.0).abs() < 1e-10);
    }
}
