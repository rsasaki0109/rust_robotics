// Extended kalman filter (EKF) localization sample
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)

extern crate nalgebra;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::{PointMarker, PointStyle};

use rand::distributions::{Normal, Distribution};

fn motion_model(x: nalgebra::Vector4<f64>, _u: nalgebra::Vector2<f64>, dt: f64)
-> nalgebra::Vector4<f64>
{
    let _yaw = x[2];
    let _f = nalgebra::Matrix4::new(
        1., 0., 0., 0.,
        0., 1., 0., 0.,
        0., 0., 1., 0.,
        0., 0., 0., 1.);
    let _b = nalgebra::Matrix4x2::new(
        dt * (_yaw).cos(), 0.,
        dt * (_yaw).sin(), 0.,
        0., dt,
        1., 0.);
    _f * x + _b * _u
}

fn jacob_f(x: nalgebra::Vector4<f64>, _u: nalgebra::Vector2<f64>, dt: f64) -> nalgebra::Matrix4<f64>
{
    let _yaw = x[2];
    let _v = _u[0];
    let _jf = nalgebra::Matrix4::new(
        1., 0., -dt * _v * (_yaw).sin(), dt * (_yaw).cos(),
        0., 1., dt * _v * (_yaw).cos(), dt * (_yaw).sin(),
        0., 0., 1., 0.,
        0., 0., 0., 1.);
    _jf
}


fn observation_model(x: nalgebra::Vector4<f64>) -> nalgebra::Vector2<f64>
{
    let _h = nalgebra::Matrix2x4::new(
        1., 0., 0., 0.,
        0., 1., 0., 0.);
    _h * x
}

fn jacob_h() -> nalgebra::Matrix2x4<f64>
{
    let _jh = nalgebra::Matrix2x4::new(
        1., 0., 0., 0.,
        0., 1., 0., 0.);
    _jh
}


fn ekf_estimation(
    x_est: nalgebra::Vector4<f64>,
    p_est: nalgebra::Matrix4<f64>,
    z: nalgebra::Vector2<f64>,
    u: nalgebra::Vector2<f64>,
    q: nalgebra::Matrix4<f64>,
    r: nalgebra::Matrix2<f64>,
    dt: f64
) -> (nalgebra::Vector4<f64>, nalgebra::Matrix4<f64>)
{
    let x_pred = motion_model(x_est, u, dt);
    let j_f = jacob_f(x_pred, u, dt);
    let p_pred = j_f * p_est * j_f.transpose() + q;

    let j_h = jacob_h();
    let z_pred = observation_model(x_pred);
    let y = z - z_pred;
    let s = j_h * p_pred * j_h.transpose() + r; 
    let k = p_pred * j_h.transpose() * s.try_inverse().unwrap();
    let new_x_est = x_pred + k * y;
    let new_p_est = (nalgebra::Matrix4::identity() - k * j_h) * p_pred;
    
    (new_x_est , new_p_est)
}

fn main() {
    let sim_time = 50.0;
    let dt = 0.1;
    let mut time = 0.;

    let mut q = nalgebra::Matrix4::<f64>::identity();
    q[(0, 0)] = (0.1_f64).powi(2i32);
    q[(1, 1)] = (1.0/180.0 * std::f64::consts::PI).powi(2i32);
    q[(2, 2)] = (0.1_f64).powi(2i32);
    q[(3, 3)] = (0.1_f64).powi(2i32);
    let r = nalgebra::Matrix2::<f64>::identity();

    let q_sim = nalgebra::Matrix2::new(
        1., 0.,
        0., (30.0/180.0 * std::f64::consts::PI).powi(2i32));
    let r_sim = nalgebra::Matrix2::<f64>::identity();

    
    let u = nalgebra::Vector2::new(1.0, 0.1);
    let mut ud = nalgebra::Vector2::new(0., 0.);
    let mut x_dr = nalgebra::Vector4::new(0., 0. , 0., 0.);
    let mut x_true = nalgebra::Vector4::new(0., 0. , 0., 0.);

    let mut x_est = nalgebra::Vector4::new(0., 0. , 0., 0.);
    let mut p_est = nalgebra::Matrix4::<f64>::identity();

    let mut z = nalgebra::Vector2::new(0., 0.);

    let normal = Normal::new(0., 1.); // mean 0., standard deviation 1.

    let mut hz = vec![(0., 0.)];
    let mut htrue  = vec![(0., 0.)];
    let mut hdr  = vec![(0., 0.)];
    let mut hest  = vec![(0., 0.)];
    

    while time < sim_time {
        time += dt;
        ud[0] = u[0] + normal.sample(&mut rand::thread_rng()) * q_sim[(0, 0)];
        ud[1] = u[1] + normal.sample(&mut rand::thread_rng()) * q_sim[(1, 1)];

        x_true = motion_model(x_true, u, dt);
        x_dr = motion_model(x_dr, ud, dt);

        z[0] = x_true[0] + normal.sample(&mut rand::thread_rng()) * r_sim[(0, 0)];
        z[1] = x_true[1] + normal.sample(&mut rand::thread_rng()) * r_sim[(1, 1)];

        let pair = ekf_estimation(x_est, p_est, z, ud, q, r, dt);
        x_est = pair.0;
        p_est = pair.1;

        hz.push((z[0], z[1]));
        htrue.push((x_true[0], x_true[1]));
        hdr.push((x_dr[0], x_dr[1]));
        hest.push((x_est[0], x_est[1]));
        
    }
    

    let s0: Plot = Plot::new(hz).point_style(
        PointStyle::new() 
            .colour("#DD3355")
            .size(3.),
    ); 
    let s1: Plot = Plot::new(htrue).point_style(
        PointStyle::new()
            .colour("#0000ff")
            .size(3.),
    ); 
    let s2: Plot = Plot::new(hdr).point_style(
        PointStyle::new() 
            .colour("#FFFF00")
            .size(3.),
    ); 
    let s3: Plot = Plot::new(hest).point_style(
        PointStyle::new() 
            .colour("#35C788")
            .size(3.),
    ); 

    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .add(s2)
        .add(s3)
        .x_range(-15., 15.)
        .y_range(-5., 25.)
        .x_label("x")
        .y_label("y");

    Page::single(&v).save("./img/ekf.svg").unwrap();
}
