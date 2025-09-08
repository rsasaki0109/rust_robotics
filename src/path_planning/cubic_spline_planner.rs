// https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/CubicSpline
// https://github.com/onlytailei/CppRobotics/blob/master/include/cubic_spline.h
// Cubic spline planner
//
// Author: Atsushi Sakai(@Atsushi_twi)
//         TAI Lei
//         Ryohei Sasaki(@rsasaki0109)

extern crate nalgebra as na;
use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

#[derive(Debug, Clone)] 
struct Spline {
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
    x: Vec<f64>,
    y: Vec<f64>,
}

impl Spline {
    fn new(x: &Vec<f64>, y: &Vec<f64>)-> Spline{
        let nx = x.len();
        let mut b: Vec<f64> = Vec::with_capacity(nx);
        let mut d: Vec<f64> = Vec::with_capacity(nx);
        let mut h: Vec<f64> = Vec::with_capacity(nx-1);
        for i in 0..nx-1 {
            h.push(x[i+1] - x[i]);
        }
        let a = y.clone();
        let a_mat = Spline::__calc_a(&h);
        let b_mat = Spline::__calc_b(&h, &a);
        
        let a_mat_inv = a_mat.try_inverse().unwrap();
        
        let c_na = a_mat_inv * b_mat;
        let mut c: Vec<f64> = Vec::with_capacity(c_na.len());
        for i in 0..c_na.len(){
            c.push(c_na[i]);
        }
        for i in 0..nx-1 {
            d.push((c[i+1] - c[i]) / (3. * h[i]));
            let tb = (a[i + 1] - a[i]) / h[i] - h[i] * 
                (c[i + 1] + 2.0 * c[i]) / 3.0;
            b.push(tb);
        }

        Spline {
          a: a, b: b,
          c: c, d: d,
          x: x.to_vec(), y: y.to_vec(),
        }
    }
    
      
    fn calc(self, t: f64)-> f64 {
        let i = self.clone().__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        let result = self.a[i] + self.b[i] * dx + self.c[i] * dx.powi(2) + self.d[i] * dx.powi(3);
        result
    }

    fn calcd(self, t: f64)-> f64 {
        let i = self.clone().__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        let b = self.b[i];
        let c = self.c[i];
        let d = self.d[i];
        let result = b + 2. * c * dx +  3. * d * dx.powi(2);
        result
    }

    fn calcdd(self, t: f64)-> f64 {
        let i = self.clone().__search_index(t);
        let x = self.x[i];
        let dx = t - x;
        let result = 2. * self.c[i] +  6. * self.d[i] * dx;
        result
    }

    fn __search_index(self, t: f64)-> usize {
        let nx = self.x.len();
        self.bisect(t, 0, nx)
    }

    fn __calc_a(h: &Vec<f64>)-> na::DMatrix<f64> {
        let nx = h.len()+1;
        let mut a = na::DMatrix::from_diagonal_element(nx, nx, 0.0) ;
        a[(0, 0)] = 1.;
        for i in 0..nx-1 {
            if i != nx-2 {
                a[(i+1, i+1)] = 2.0 * (h[i] + h[i + 1]);
            }
            a[(i+1, i)] = h[i];
            a[(i, i+1)] = h[i];
        }
        a[(0, 1)] = 0.;
        a[(nx-1, nx-2)] = 0.;
        a[(nx-1, nx-1)] = 1.;
        a
    }

    fn __calc_b(h: &Vec<f64>, a: &Vec<f64>)-> na::DVector<f64> {
        let nx = h.len() +1;
        let mut b = na::DVector::zeros(nx);
        for i in 0..nx-2 {
            b[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) /
            h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
        }
        b
    }
    
    fn bisect(self, t: f64, s: usize, e: usize) -> usize
    {
        let mid = (s + e) / 2;
        if t == self.x[mid] || e - s <= 1 {
            return mid
        } else if t > self.x[mid] {
            return self.bisect(t, mid, e)
        } else {
            return self.bisect(t, s, mid)
        }
    }
}


#[derive(Debug, Clone)] 
pub struct Spline2D {
    pub s: Vec<f64>,
    sx: Spline,
    sy: Spline,
}

impl Spline2D {

    pub fn new(x: Vec<f64>, y: Vec<f64>)-> Spline2D {
        let s = Spline2D::__calc_s(&x, &y);
        let sx = Spline::new(&s, &x);
        let sy = Spline::new(&s, &y);

        Spline2D {
          s: s,
          sx: sx, sy: sy,
        }
    }

    fn __calc_s(x: &Vec<f64>, y: &Vec<f64>) -> Vec<f64>
    {
        let nx = x.len();
        let mut dx: Vec<f64> = Vec::with_capacity(nx-1);
        let mut dy: Vec<f64> = Vec::with_capacity(nx-1);
        
        
        for i in 0..nx-1 {
            dx.push(x[i+1] - x[i]);
            dy.push(y[i+1] - y[i]);
        }
        let mut ds: Vec<f64> = Vec::with_capacity(nx-1);
        for i in 0..nx-1 {
            let dsi = (dx[i].powi(2) + dy[i].powi(2)).sqrt();
            ds.push(dsi);
        }
        let mut s: Vec<f64> = Vec::with_capacity(nx);
        s.push(0.);
        for i in 0..nx-1 {
            s.push(s[i] + ds[i]);
        }
        s
    }

    pub fn calc_position(self, is: f64) -> (f64, f64)
    {
        let x = self.sx.calc(is);
        let y = self.sy.calc(is);
        (x, y)
    }

    fn calc_curvature(self, is: f64) -> f64
    {
        let dx = self.sx.clone().calcd(is);
        let ddx = self.sx.calcdd(is);
        let dy = self.sy.clone().calcd(is);
        let ddy = self.sy.calcdd(is);
        let k = (ddy * dx - ddx * dy) / ((dx.powi(2) + dy.powi(2)).powf(3. / 2.));
        k
    }

    fn calc_yaw(self, is: f64) -> f64
    {
        let dx = self.sx.calcd(is); 
        let dy = self.sy.calcd(is);
        let yaw = dy.atan2(dx); 
        yaw
    }
    
}

pub fn calc_spline_course(x: Vec<f64>, y: Vec<f64>, ds: f64) -> 
(Vec<(f64,f64)>, Vec<f64>, Vec<f64>, Vec<f64>)
{
    let sp = Spline2D::new(x, y);
    let s_end = sp.s[sp.s.len()-1]; 
    let n = (s_end / ds) as usize;
    let mut r: Vec<(f64,f64)> = Vec::with_capacity(n);
    let mut ryaw: Vec<f64> = Vec::with_capacity(n);
    let mut rk: Vec<f64> = Vec::with_capacity(n);
    let mut s: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n-1 {
        let is = (i as f64/ (n -1)as f64)  * s_end;
        let pair = sp.clone().calc_position(is);
        r.push(pair);
        ryaw.push(sp.clone().calc_yaw(is));
        rk.push(sp.clone().calc_curvature(is));
        s.push(is);
    }
    (r, ryaw, rk, s)
}

pub struct CubicSplinePlanner {
    pub path: Vec<(f64, f64)>,
    pub yaw: Vec<f64>,
    pub curvature: Vec<f64>,
    pub s: Vec<f64>,
}

impl CubicSplinePlanner {
    pub fn new() -> Self {
        CubicSplinePlanner {
            path: Vec::new(),
            yaw: Vec::new(),
            curvature: Vec::new(),
            s: Vec::new(),
        }
    }

    pub fn planning(&mut self, waypoints_x: Vec<f64>, waypoints_y: Vec<f64>, ds: f64) -> bool {
        if waypoints_x.len() != waypoints_y.len() || waypoints_x.len() < 2 {
            println!("Invalid waypoints: need at least 2 points and x,y must have same length");
            return false;
        }

        let (path, yaw, curvature, s) = calc_spline_course(waypoints_x, waypoints_y, ds);
        
        self.path = path;
        self.yaw = yaw;
        self.curvature = curvature;
        self.s = s;

        println!("Cubic spline path generated with {} points", self.path.len());
        true
    }

    pub fn visualize_path(&self, waypoints_x: &[f64], waypoints_y: &[f64]) {
        if self.path.is_empty() {
            println!("No path to visualize");
            return;
        }

        let mut fg = Figure::new();
        let axes = fg.axes2d();

        // Plot waypoints
        axes.points(waypoints_x, waypoints_y, &[Caption("Waypoints"), Color("red")]);

        // Plot spline path
        let path_x: Vec<f64> = self.path.iter().map(|p| p.0).collect();
        let path_y: Vec<f64> = self.path.iter().map(|p| p.1).collect();
        axes.lines(&path_x, &path_y, &[Caption("Cubic Spline Path"), Color("blue")]);

        axes.set_title("Cubic Spline Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Save to file
        let output_path = "img/path_planning/cubic_spline_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);

        fg.show().unwrap();
    }

    pub fn visualize_profiles(&self) {
        if self.s.is_empty() {
            return;
        }

        // Yaw profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        let yaw_deg: Vec<f64> = self.yaw.iter().map(|y| y * 180.0 / PI).collect();
        axes.lines(&self.s, &yaw_deg, &[Caption("Yaw"), Color("red")]);
        axes.set_title("Yaw Profile", &[])
            .set_x_label("Distance [m]", &[])
            .set_y_label("Yaw [deg]", &[]);
        fg.save_to_png("img/path_planning/cubic_spline_yaw_profile.png", 800, 600).unwrap();

        // Curvature profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        axes.lines(&self.s, &self.curvature, &[Caption("Curvature"), Color("red")]);
        axes.set_title("Curvature Profile", &[])
            .set_x_label("Distance [m]", &[])
            .set_y_label("Curvature [1/m]", &[]);
        fg.save_to_png("img/path_planning/cubic_spline_curvature_profile.png", 800, 600).unwrap();

        println!("Profile plots saved to img/path_planning/");
    }
}

fn main() {
    println!("Cubic Spline Path Planning start!!");

    // Define waypoints
    let waypoints_x = vec![0.0, 10.0, 20.5, 30.0, 40.5, 50.0];
    let waypoints_y = vec![0.0, -6.0, 5.0, 6.5, 0.0, -4.0];
    let ds = 0.1; // distance step [m]

    let mut planner = CubicSplinePlanner::new();

    if planner.planning(waypoints_x.clone(), waypoints_y.clone(), ds) {
        println!("Path generated successfully!");
        planner.visualize_path(&waypoints_x, &waypoints_y);
        planner.visualize_profiles();
    } else {
        println!("Failed to generate path");
    }

    println!("Cubic Spline Path Planning finish!!");
}
