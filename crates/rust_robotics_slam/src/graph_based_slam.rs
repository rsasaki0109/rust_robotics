#![allow(dead_code, clippy::too_many_arguments)]

// Graph-based SLAM
// author: Atsushi Sakai (@Atsushi_twi)
//         Rust port
//
// Ref:
// [A Tutorial on Graph-Based SLAM]
// http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;

// Simulation parameters
const DT: f64 = 2.0; // time step [s]
const MAX_RANGE: f64 = 30.0; // maximum observation range [m]
const STATE_SIZE: usize = 3; // State size [x, y, yaw]

// Noise parameters
const Q_SIM: [[f64; 2]; 2] = [[0.2, 0.0], [0.0, 0.00030462]]; // 0.00030462 ≈ deg2rad(1.0)^2
const R_SIM: [[f64; 2]; 2] = [[0.1, 0.0], [0.0, 0.030462]]; // 0.030462 ≈ deg2rad(10.0)^2

// Covariance parameters of Graph Based SLAM
const C_SIGMA1: f64 = 0.1;
const C_SIGMA2: f64 = 0.1;
const C_SIGMA3: f64 = 0.017453; // deg2rad(1.0)

const MAX_ITR: usize = 20; // Maximum iteration

/// Observation data structure
#[derive(Clone)]
pub struct Observation {
    pub d: f64,     // distance
    pub angle: f64, // angle relative to robot
    pub phi: f64,   // absolute angle
    pub id: usize,  // landmark id
}

/// Edge represents a constraint between two poses
#[derive(Clone)]
struct Edge {
    e: Vector3<f64>,     // error vector
    omega: Matrix3<f64>, // information matrix
    d1: f64,
    d2: f64,
    yaw1: f64,
    yaw2: f64,
    angle1: f64,
    angle2: f64,
    id1: usize,
    id2: usize,
}

impl Edge {
    fn new() -> Self {
        Edge {
            e: Vector3::zeros(),
            omega: Matrix3::zeros(),
            d1: 0.0,
            d2: 0.0,
            yaw1: 0.0,
            yaw2: 0.0,
            angle1: 0.0,
            angle2: 0.0,
            id1: 0,
            id2: 0,
        }
    }
}

/// Normalize angle to [-pi, pi]
fn pi_2_pi(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Calculate observation noise covariance
fn cal_observation_sigma() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        C_SIGMA1 * C_SIGMA1,
        C_SIGMA2 * C_SIGMA2,
        C_SIGMA3 * C_SIGMA3,
    ))
}

/// Calculate 3D rotation matrix around z-axis
fn calc_3d_rotational_matrix(angle: f64) -> Matrix3<f64> {
    let c = angle.cos();
    let s = angle.sin();
    Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

/// Calculate edge between two poses that observed the same landmark
fn calc_edge(
    x1: f64,
    y1: f64,
    yaw1: f64,
    x2: f64,
    y2: f64,
    yaw2: f64,
    d1: f64,
    angle1: f64,
    d2: f64,
    angle2: f64,
    t1: usize,
    t2: usize,
) -> Edge {
    let mut edge = Edge::new();

    let tangle1 = pi_2_pi(yaw1 + angle1);
    let tangle2 = pi_2_pi(yaw2 + angle2);

    let tmp1 = d1 * tangle1.cos();
    let tmp2 = d2 * tangle2.cos();
    let tmp3 = d1 * tangle1.sin();
    let tmp4 = d2 * tangle2.sin();

    // Error vector
    edge.e[0] = x2 - x1 - tmp1 + tmp2;
    edge.e[1] = y2 - y1 - tmp3 + tmp4;
    edge.e[2] = 0.0;

    // Information matrix
    let rt1 = calc_3d_rotational_matrix(tangle1);
    let rt2 = calc_3d_rotational_matrix(tangle2);

    let sig1 = cal_observation_sigma();
    let sig2 = cal_observation_sigma();

    let cov = rt1 * sig1 * rt1.transpose() + rt2 * sig2 * rt2.transpose();
    edge.omega = cov.try_inverse().unwrap_or(Matrix3::identity());

    edge.d1 = d1;
    edge.d2 = d2;
    edge.yaw1 = yaw1;
    edge.yaw2 = yaw2;
    edge.angle1 = angle1;
    edge.angle2 = angle2;
    edge.id1 = t1;
    edge.id2 = t2;

    edge
}

/// Calculate all edges from pose and observation lists
fn calc_edges(x_list: &DMatrix<f64>, z_list: &[Option<Vec<Observation>>]) -> (Vec<Edge>, f64) {
    let mut edges = Vec::new();
    let mut cost = 0.0;

    let nt = z_list.len();

    // Generate all pairs of time indices
    for t1 in 0..nt {
        for t2 in (t1 + 1)..nt {
            let x1 = x_list[(0, t1)];
            let y1 = x_list[(1, t1)];
            let yaw1 = x_list[(2, t1)];
            let x2 = x_list[(0, t2)];
            let y2 = x_list[(1, t2)];
            let yaw2 = x_list[(2, t2)];

            // Check if both have observations
            if let (Some(z1), Some(z2)) = (&z_list[t1], &z_list[t2]) {
                // Find matching landmarks
                for obs1 in z1 {
                    for obs2 in z2 {
                        if obs1.id == obs2.id {
                            let edge = calc_edge(
                                x1, y1, yaw1, x2, y2, yaw2, obs1.d, obs1.angle, obs2.d, obs2.angle,
                                t1, t2,
                            );

                            // Calculate cost: e^T * omega * e
                            let e_t_omega = edge.e.transpose() * edge.omega;
                            cost += (e_t_omega * edge.e)[(0, 0)];

                            edges.push(edge);
                        }
                    }
                }
            }
        }
    }

    (edges, cost)
}

/// Calculate Jacobians for an edge
fn calc_jacobian(edge: &Edge) -> (Matrix3<f64>, Matrix3<f64>) {
    let t1 = edge.yaw1 + edge.angle1;
    let a = Matrix3::new(
        -1.0,
        0.0,
        edge.d1 * t1.sin(),
        0.0,
        -1.0,
        -edge.d1 * t1.cos(),
        0.0,
        0.0,
        0.0,
    );

    let t2 = edge.yaw2 + edge.angle2;
    let b = Matrix3::new(
        1.0,
        0.0,
        -edge.d2 * t2.sin(),
        0.0,
        1.0,
        edge.d2 * t2.cos(),
        0.0,
        0.0,
        0.0,
    );

    (a, b)
}

/// Fill H matrix and b vector for an edge
fn fill_h_and_b(h: &mut DMatrix<f64>, b: &mut DVector<f64>, edge: &Edge) {
    let (a, b_jac) = calc_jacobian(edge);

    let id1 = edge.id1 * STATE_SIZE;
    let id2 = edge.id2 * STATE_SIZE;

    // H[id1:id1+3, id1:id1+3] += A^T * omega * A
    let at_omega = a.transpose() * edge.omega;
    let at_omega_a = at_omega * a;
    let at_omega_b = at_omega * b_jac;

    let bt_omega = b_jac.transpose() * edge.omega;
    let bt_omega_a = bt_omega * a;
    let bt_omega_b = bt_omega * b_jac;

    for i in 0..STATE_SIZE {
        for j in 0..STATE_SIZE {
            h[(id1 + i, id1 + j)] += at_omega_a[(i, j)];
            h[(id1 + i, id2 + j)] += at_omega_b[(i, j)];
            h[(id2 + i, id1 + j)] += bt_omega_a[(i, j)];
            h[(id2 + i, id2 + j)] += bt_omega_b[(i, j)];
        }
    }

    // b[id1:id1+3] += A^T * omega * e
    let at_omega_e = at_omega * edge.e;
    let bt_omega_e = bt_omega * edge.e;

    for i in 0..STATE_SIZE {
        b[id1 + i] += at_omega_e[i];
        b[id2 + i] += bt_omega_e[i];
    }
}

/// Graph-based SLAM optimization
pub fn graph_based_slam(
    x_init: &DMatrix<f64>,
    hz: &[Option<Vec<Observation>>],
) -> DMatrix<f64> {
    let mut x_opt = x_init.clone();
    let nt = x_opt.ncols();
    let n = nt * STATE_SIZE;

    for _itr in 0..MAX_ITR {
        let (edges, _cost) = calc_edges(&x_opt, hz);

        let mut h = DMatrix::<f64>::zeros(n, n);
        let mut b = DVector::<f64>::zeros(n);

        for edge in &edges {
            fill_h_and_b(&mut h, &mut b, edge);
        }

        // Fix origin by adding identity to first pose block
        for i in 0..STATE_SIZE {
            h[(i, i)] += 1.0;
        }

        // Solve H * dx = -b
        let h_inv = match h.clone().try_inverse() {
            Some(inv) => inv,
            None => {
                // Simple fallback - add regularization
                let mut h_reg = h.clone();
                for i in 0..n {
                    h_reg[(i, i)] += 0.001;
                }
                h_reg.try_inverse().unwrap_or(DMatrix::identity(n, n))
            }
        };

        let dx = -h_inv * &b;

        // Update poses
        for i in 0..nt {
            for j in 0..STATE_SIZE {
                x_opt[(j, i)] += dx[i * STATE_SIZE + j];
            }
        }

        let diff = dx.dot(&dx);

        if diff < 1.0e-5 {
            break;
        }
    }

    x_opt
}

/// Motion model for robot
pub fn motion_model(x: &Vector3<f64>, u: &[f64; 2]) -> Vector3<f64> {
    let yaw = x[2];
    Vector3::new(
        x[0] + u[0] * DT * yaw.cos(),
        x[1] + u[0] * DT * yaw.sin(),
        x[2] + u[1] * DT,
    )
}

/// Calculate control input
pub fn calc_input() -> [f64; 2] {
    let v = 1.0; // [m/s]
    let yaw_rate = 0.1; // [rad/s]
    [v, yaw_rate]
}

/// Simulate observation and motion
pub fn observation(
    x_true: &Vector3<f64>,
    xd: &Vector3<f64>,
    u: &[f64; 2],
    rfid: &[(f64, f64)],
) -> (
    Vector3<f64>,
    Option<Vec<Observation>>,
    Vector3<f64>,
    [f64; 2],
) {
    let normal = Normal::new(0.0, 1.0).unwrap();
    let mut rng = rand::thread_rng();

    // True state update
    let x_true_new = motion_model(x_true, u);

    // Observations
    let mut z = Vec::new();
    for (i, (lx, ly)) in rfid.iter().enumerate() {
        let dx = lx - x_true_new[0];
        let dy = ly - x_true_new[1];
        let d = (dx * dx + dy * dy).sqrt();
        let angle = pi_2_pi(dy.atan2(dx) - x_true_new[2]);
        let phi = pi_2_pi(dy.atan2(dx));

        if d <= MAX_RANGE {
            // Add noise
            let dn = d + normal.sample(&mut rng) * Q_SIM[0][0].sqrt();
            let angle_noise = normal.sample(&mut rng) * Q_SIM[1][1].sqrt();
            let angle_noisy = angle + angle_noise;
            let phi_noisy = phi + angle_noise;

            z.push(Observation {
                d: dn,
                angle: angle_noisy,
                phi: phi_noisy,
                id: i,
            });
        }
    }

    // Dead reckoning with noise
    let ud = [
        u[0] + normal.sample(&mut rng) * R_SIM[0][0].sqrt(),
        u[1] + normal.sample(&mut rng) * R_SIM[1][1].sqrt(),
    ];
    let xd_new = motion_model(xd, &ud);

    let z_opt = if z.is_empty() { None } else { Some(z) };

    (x_true_new, z_opt, xd_new, ud)
}
