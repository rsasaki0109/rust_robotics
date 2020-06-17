// https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/CubicSpline
// https://github.com/onlytailei/CppRobotics/blob/master/include/cubic_spline.h
// Cubic spline planner
//
// Author: Atsushi Sakai(@Atsushi_twi)
//         TAI Lei
//         Ryohei Sasaki(@rsasaki0109)

extern crate nalgebra as na;

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
