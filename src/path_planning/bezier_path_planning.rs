// Path planning with Bezier curve.
//
// Author: Atsushi Sakai(@Atsushi_twi)
//         Rust implementation

use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

// Binomial coefficient calculation (replacement for scipy.special.comb)
fn binomial_coefficient(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    if k == 0 || k == n {
        return 1.0;
    }
    
    let k = if k > n - k { n - k } else { k }; // Take advantage of symmetry
    
    let mut result = 1.0;
    for i in 0..k {
        result *= (n - i) as f64;
        result /= (i + 1) as f64;
    }
    result
}

// Bernstein polynomial
fn bernstein_poly(n: usize, i: usize, t: f64) -> f64 {
    binomial_coefficient(n, i) * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
}

// Return one point on the bezier curve
fn bezier(t: f64, control_points: &[(f64, f64)]) -> (f64, f64) {
    let n = control_points.len() - 1;
    let mut x = 0.0;
    let mut y = 0.0;
    
    for (i, &(px, py)) in control_points.iter().enumerate() {
        let basis = bernstein_poly(n, i, t);
        x += basis * px;
        y += basis * py;
    }
    
    (x, y)
}

// Compute bezier path (trajectory) given control points
fn calc_bezier_path(control_points: &[(f64, f64)], n_points: usize) -> Vec<(f64, f64)> {
    let mut trajectory = Vec::new();
    
    for i in 0..n_points {
        let t = i as f64 / (n_points - 1) as f64;
        trajectory.push(bezier(t, control_points));
    }
    
    trajectory
}

// Compute control points and path given start and end position
fn calc_4points_bezier_path(sx: f64, sy: f64, syaw: f64, ex: f64, ey: f64, eyaw: f64, offset: f64) 
    -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    let dist = ((sx - ex).powi(2) + (sy - ey).powi(2)).sqrt() / offset;
    
    let control_points = vec![
        (sx, sy),
        (sx + dist * syaw.cos(), sy + dist * syaw.sin()),
        (ex - dist * eyaw.cos(), ey - dist * eyaw.sin()),
        (ex, ey),
    ];
    
    let path = calc_bezier_path(&control_points, 100);
    
    (path, control_points)
}

// Compute control points of the successive derivatives of a given bezier curve
fn bezier_derivatives_control_points(control_points: &[(f64, f64)], n_derivatives: usize) 
    -> Vec<Vec<(f64, f64)>> {
    let mut derivatives = vec![control_points.to_vec()];
    
    for i in 0..n_derivatives {
        let current = &derivatives[i];
        let n = current.len();
        
        if n <= 1 {
            break;
        }
        
        let mut next_derivative = Vec::new();
        for j in 0..n-1 {
            let dx = (n - 1) as f64 * (current[j + 1].0 - current[j].0);
            let dy = (n - 1) as f64 * (current[j + 1].1 - current[j].1);
            next_derivative.push((dx, dy));
        }
        derivatives.push(next_derivative);
    }
    
    derivatives
}

// Calculate curvature at parameter t
fn calc_curvature(control_points: &[(f64, f64)], t: f64) -> f64 {
    let derivatives = bezier_derivatives_control_points(control_points, 2);
    
    if derivatives.len() < 3 {
        return 0.0;
    }
    
    let first_deriv = bezier(t, &derivatives[1]);
    let second_deriv = bezier(t, &derivatives[2]);
    
    let dx = first_deriv.0;
    let dy = first_deriv.1;
    let ddx = second_deriv.0;
    let ddy = second_deriv.1;
    
    let numerator = dx * ddy - dy * ddx;
    let denominator = (dx * dx + dy * dy).powf(1.5);
    
    if denominator.abs() < 1e-10 {
        0.0
    } else {
        numerator / denominator
    }
}

pub struct BezierPathPlanner {
    pub path: Vec<(f64, f64)>,
    pub control_points: Vec<(f64, f64)>,
    pub curvature: Vec<f64>,
}

impl BezierPathPlanner {
    pub fn new() -> Self {
        BezierPathPlanner {
            path: Vec::new(),
            control_points: Vec::new(),
            curvature: Vec::new(),
        }
    }
    
    pub fn planning(&mut self, sx: f64, sy: f64, syaw: f64, ex: f64, ey: f64, eyaw: f64, offset: f64) -> bool {
        let (path, control_points) = calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset);
        
        // Calculate curvature along the path
        let mut curvature = Vec::new();
        for i in 0..path.len() {
            let t = i as f64 / (path.len() - 1) as f64;
            curvature.push(calc_curvature(&control_points, t));
        }
        
        self.path = path;
        self.control_points = control_points;
        self.curvature = curvature;
        
        println!("Bezier path generated with {} points", self.path.len());
        true
    }
    
    pub fn planning_with_control_points(&mut self, control_points: Vec<(f64, f64)>, n_points: usize) -> bool {
        if control_points.len() < 2 {
            println!("Need at least 2 control points");
            return false;
        }
        
        let path = calc_bezier_path(&control_points, n_points);
        
        // Calculate curvature along the path
        let mut curvature = Vec::new();
        for i in 0..path.len() {
            let t = i as f64 / (path.len() - 1) as f64;
            curvature.push(calc_curvature(&control_points, t));
        }
        
        self.path = path;
        self.control_points = control_points;
        self.curvature = curvature;
        
        println!("Bezier path generated with {} points from {} control points", 
                 self.path.len(), self.control_points.len());
        true
    }
    
    pub fn visualize(&self) {
        if self.path.is_empty() {
            println!("No path to visualize");
            return;
        }
        
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        
        // Plot path
        let path_x: Vec<f64> = self.path.iter().map(|p| p.0).collect();
        let path_y: Vec<f64> = self.path.iter().map(|p| p.1).collect();
        axes.lines(&path_x, &path_y, &[Caption("Bezier Path"), Color("blue")]);
        
        // Plot control points
        let control_x: Vec<f64> = self.control_points.iter().map(|p| p.0).collect();
        let control_y: Vec<f64> = self.control_points.iter().map(|p| p.1).collect();
        axes.points(&control_x, &control_y, &[Caption("Control Points"), Color("red")]);
        axes.lines(&control_x, &control_y, &[Caption("Control Polygon"), Color("red")]);
        
        axes.set_title("Bezier Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
        
        let output_path = "img/path_planning/bezier_path_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);
        
        fg.show().unwrap();
    }
    
    pub fn visualize_curvature(&self) {
        if self.curvature.is_empty() {
            return;
        }
        
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        
        let s: Vec<f64> = (0..self.curvature.len()).map(|i| i as f64).collect();
        axes.lines(&s, &self.curvature, &[Caption("Curvature"), Color("red")]);
        
        axes.set_title("Bezier Path Curvature Profile", &[])
            .set_x_label("Point Index", &[])
            .set_y_label("Curvature [1/m]", &[]);
        
        fg.save_to_png("img/path_planning/bezier_curvature_profile.png", 800, 600).unwrap();
        println!("Curvature profile saved to img/path_planning/bezier_curvature_profile.png");
    }
}

fn main() {
    println!("Bezier path planner start!!");
    
    // Example 1: 4-point Bezier path from start/end poses
    let start_x = 10.0;
    let start_y = 1.0;
    let start_yaw = (180.0_f64).to_radians();
    
    let end_x = -0.0;
    let end_y = -3.0;
    let end_yaw = (45.0_f64).to_radians();
    let offset = 3.0;
    
    let mut planner = BezierPathPlanner::new();
    
    if planner.planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset) {
        println!("4-point Bezier path generated successfully!");
        planner.visualize();
        planner.visualize_curvature();
    }
    
    // Example 2: Bezier path with custom control points
    let control_points = vec![
        (-1.0, 0.0),
        (3.0, -3.0),
        (4.0, 1.0),
        (2.0, 1.0),
        (1.0, 3.0),
    ];
    
    let mut planner2 = BezierPathPlanner::new();
    
    if planner2.planning_with_control_points(control_points, 100) {
        println!("Custom control points Bezier path generated successfully!");
        
        // Save as separate file
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        
        let path_x: Vec<f64> = planner2.path.iter().map(|p| p.0).collect();
        let path_y: Vec<f64> = planner2.path.iter().map(|p| p.1).collect();
        axes.lines(&path_x, &path_y, &[Caption("Bezier Path"), Color("blue")]);
        
        let control_x: Vec<f64> = planner2.control_points.iter().map(|p| p.0).collect();
        let control_y: Vec<f64> = planner2.control_points.iter().map(|p| p.1).collect();
        axes.points(&control_x, &control_y, &[Caption("Control Points"), Color("red")]);
        axes.lines(&control_x, &control_y, &[Caption("Control Polygon"), Color("red")]);
        
        axes.set_title("Bezier Path with Custom Control Points", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
        
        fg.save_to_png("img/path_planning/bezier_custom_result.png", 800, 600).unwrap();
        println!("Custom control points plot saved to: img/path_planning/bezier_custom_result.png");
        
        fg.show().unwrap();
    }
    
    println!("Bezier path planner finish!!");
}
