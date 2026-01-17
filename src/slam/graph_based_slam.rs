// Graph-based SLAM
// author: Atsushi Sakai (@Atsushi_twi)
//         Rust port
//
// Ref:
// [A Tutorial on Graph-Based SLAM]
// http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

use nalgebra::{Matrix3, Vector3, DMatrix, DVector};
use rand_distr::{Distribution, Normal};
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};
use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;

// Simulation parameters
const DT: f64 = 2.0; // time step [s]
const SIM_TIME: f64 = 100.0; // simulation time [s]
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

const SHOW_GRAPH_D_TIME: f64 = 20.0; // [s]
const SHOW_ANIMATION: bool = false;

/// Observation data structure
#[derive(Clone)]
struct Observation {
    d: f64,      // distance
    angle: f64,  // angle relative to robot
    phi: f64,    // absolute angle
    id: usize,   // landmark id
}

/// Edge represents a constraint between two poses
#[derive(Clone)]
struct Edge {
    e: Vector3<f64>,      // error vector
    omega: Matrix3<f64>,  // information matrix
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
    Matrix3::new(
        c, -s, 0.0,
        s,  c, 0.0,
        0.0, 0.0, 1.0,
    )
}

/// Calculate edge between two poses that observed the same landmark
fn calc_edge(
    x1: f64, y1: f64, yaw1: f64,
    x2: f64, y2: f64, yaw2: f64,
    d1: f64, angle1: f64,
    d2: f64, angle2: f64,
    t1: usize, t2: usize,
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
                                x1, y1, yaw1,
                                x2, y2, yaw2,
                                obs1.d, obs1.angle,
                                obs2.d, obs2.angle,
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

    println!("cost: {:.4}, n_edge: {}", cost, edges.len());
    (edges, cost)
}

/// Calculate Jacobians for an edge
fn calc_jacobian(edge: &Edge) -> (Matrix3<f64>, Matrix3<f64>) {
    let t1 = edge.yaw1 + edge.angle1;
    let a = Matrix3::new(
        -1.0, 0.0, edge.d1 * t1.sin(),
        0.0, -1.0, -edge.d1 * t1.cos(),
        0.0, 0.0, 0.0,
    );

    let t2 = edge.yaw2 + edge.angle2;
    let b = Matrix3::new(
        1.0, 0.0, -edge.d2 * t2.sin(),
        0.0, 1.0, edge.d2 * t2.cos(),
        0.0, 0.0, 0.0,
    );

    (a, b)
}

/// Fill H matrix and b vector for an edge
fn fill_h_and_b(
    h: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    edge: &Edge,
) {
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
fn graph_based_slam(
    x_init: &DMatrix<f64>,
    hz: &[Option<Vec<Observation>>],
) -> DMatrix<f64> {
    println!("start graph based slam");

    let mut x_opt = x_init.clone();
    let nt = x_opt.ncols();
    let n = nt * STATE_SIZE;

    for itr in 0..MAX_ITR {
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
                println!("Warning: H matrix is singular, using pseudo-inverse");
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
        println!("iteration: {}, diff: {:.6}", itr + 1, diff);

        if diff < 1.0e-5 {
            break;
        }
    }

    x_opt
}

/// Motion model for robot
fn motion_model(x: &Vector3<f64>, u: &[f64; 2]) -> Vector3<f64> {
    let yaw = x[2];
    Vector3::new(
        x[0] + u[0] * DT * yaw.cos(),
        x[1] + u[0] * DT * yaw.sin(),
        x[2] + u[1] * DT,
    )
}

/// Calculate control input
fn calc_input() -> [f64; 2] {
    let v = 1.0;        // [m/s]
    let yaw_rate = 0.1; // [rad/s]
    [v, yaw_rate]
}

/// Simulate observation and motion
fn observation(
    x_true: &Vector3<f64>,
    xd: &Vector3<f64>,
    u: &[f64; 2],
    rfid: &[(f64, f64)],
) -> (Vector3<f64>, Option<Vec<Observation>>, Vector3<f64>, [f64; 2]) {
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

/// Save SVG plot directly without gnuplot dependency
fn save_svg(
    path: &str,
    true_x: &[f64], true_y: &[f64],
    dr_x: &[f64], dr_y: &[f64],
    opt_x: &[f64], opt_y: &[f64],
    lm_x: &[f64], lm_y: &[f64],
) {
    let width = 640;
    let height = 480;
    let margin = 60.0;

    // Calculate bounds
    let all_x: Vec<f64> = true_x.iter()
        .chain(dr_x.iter())
        .chain(opt_x.iter())
        .chain(lm_x.iter())
        .copied()
        .collect();
    let all_y: Vec<f64> = true_y.iter()
        .chain(dr_y.iter())
        .chain(opt_y.iter())
        .chain(lm_y.iter())
        .copied()
        .collect();

    let x_min = all_x.iter().cloned().fold(f64::INFINITY, f64::min);
    let x_max = all_x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let y_min = all_y.iter().cloned().fold(f64::INFINITY, f64::min);
    let y_max = all_y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let x_range = (x_max - x_min).max(1.0);
    let y_range = (y_max - y_min).max(1.0);
    let range = x_range.max(y_range) * 1.1;

    let x_center = (x_min + x_max) / 2.0;
    let y_center = (y_min + y_max) / 2.0;

    let plot_width = width as f64 - 2.0 * margin;
    let plot_height = height as f64 - 2.0 * margin;
    let scale = plot_width.min(plot_height) / range;

    let transform_x = |x: f64| -> f64 {
        margin + plot_width / 2.0 + (x - x_center) * scale
    };
    let transform_y = |y: f64| -> f64 {
        margin + plot_height / 2.0 - (y - y_center) * scale
    };

    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">
<rect width="100%" height="100%" fill="white"/>
<text x="{}" y="30" text-anchor="middle" font-size="16" font-family="sans-serif">Graph-based SLAM</text>
"#,
        width, height, width, height,
        width / 2
    ));

    // Draw grid
    svg.push_str("<g stroke=\"#dddddd\" stroke-width=\"0.5\">");
    for i in 0..=10 {
        let x = margin + (i as f64 / 10.0) * plot_width;
        let y = margin + (i as f64 / 10.0) * plot_height;
        svg.push_str(&format!(
            r#"<line x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
            x, margin, x, margin + plot_height
        ));
        svg.push_str(&format!(
            r#"<line x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
            margin, y, margin + plot_width, y
        ));
    }
    svg.push_str("</g>\n");

    // Draw dead reckoning trajectory (black)
    if dr_x.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#333\" stroke-width=\"1.5\" points=\"");
        for (x, y) in dr_x.iter().zip(dr_y.iter()) {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(*x), transform_y(*y)));
        }
        svg.push_str("\"/>\n");
    }

    // Draw true trajectory (blue)
    if true_x.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"blue\" stroke-width=\"2\" points=\"");
        for (x, y) in true_x.iter().zip(true_y.iter()) {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(*x), transform_y(*y)));
        }
        svg.push_str("\"/>\n");
    }

    // Draw optimized trajectory (red)
    if opt_x.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"red\" stroke-width=\"2\" points=\"");
        for (x, y) in opt_x.iter().zip(opt_y.iter()) {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(*x), transform_y(*y)));
        }
        svg.push_str("\"/>\n");
    }

    // Draw landmarks (black stars)
    for (x, y) in lm_x.iter().zip(lm_y.iter()) {
        let cx = transform_x(*x);
        let cy = transform_y(*y);
        // Draw a star shape
        svg.push_str(&format!(
            r#"<polygon fill="black" points="{},{} {},{} {},{} {},{} {},{} {},{} {},{} {},{} {},{} {},{}"/>"#,
            cx, cy - 8.0,
            cx + 2.5, cy - 2.5,
            cx + 8.0, cy - 2.5,
            cx + 4.0, cy + 2.0,
            cx + 5.5, cy + 8.0,
            cx, cy + 4.5,
            cx - 5.5, cy + 8.0,
            cx - 4.0, cy + 2.0,
            cx - 8.0, cy - 2.5,
            cx - 2.5, cy - 2.5
        ));
    }

    // Legend
    let legend_x = width as f64 - 150.0;
    let legend_y = 50.0;
    svg.push_str(&format!(
        "<rect x=\"{}\" y=\"{}\" width=\"140\" height=\"80\" fill=\"white\" stroke=\"#ccc\"/>",
        legend_x, legend_y
    ));
    svg.push_str(&format!(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"blue\" stroke-width=\"2\"/>",
        legend_x + 10.0, legend_y + 20.0, legend_x + 40.0, legend_y + 20.0
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" font-size=\"12\" font-family=\"sans-serif\">True</text>",
        legend_x + 50.0, legend_y + 24.0
    ));
    svg.push_str(&format!(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#333\" stroke-width=\"1.5\"/>",
        legend_x + 10.0, legend_y + 40.0, legend_x + 40.0, legend_y + 40.0
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" font-size=\"12\" font-family=\"sans-serif\">Dead Reckoning</text>",
        legend_x + 50.0, legend_y + 44.0
    ));
    svg.push_str(&format!(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"red\" stroke-width=\"2\"/>",
        legend_x + 10.0, legend_y + 60.0, legend_x + 40.0, legend_y + 60.0
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" font-size=\"12\" font-family=\"sans-serif\">Graph SLAM</text>",
        legend_x + 50.0, legend_y + 64.0
    ));

    svg.push_str("</svg>\n");

    let mut file = File::create(path).expect("Failed to create SVG file");
    file.write_all(svg.as_bytes()).expect("Failed to write SVG");
}

fn main() {
    println!("Graph-based SLAM start!");

    let mut time = 0.0;

    // RFID/Landmark positions [x, y]
    let rfid: Vec<(f64, f64)> = vec![
        (10.0, -2.0),
        (15.0, 10.0),
        (3.0, 15.0),
        (-5.0, 20.0),
        (-5.0, 5.0),
    ];

    // State vectors [x, y, yaw]
    let mut x_true = Vector3::new(0.0, 0.0, 0.0);
    let mut x_dr = Vector3::new(0.0, 0.0, 0.0); // Dead reckoning

    // History
    let mut h_x_true: Vec<Vector3<f64>> = vec![x_true];
    let mut h_x_dr: Vec<Vector3<f64>> = vec![x_dr];
    let mut hz: Vec<Option<Vec<Observation>>> = Vec::new();

    let mut d_time = 0.0;
    let mut fig = Figure::new();

    // Initial observation
    let u = calc_input();
    let (x_true_new, z, x_dr_new, _ud) = observation(&x_true, &x_dr, &u, &rfid);
    x_true = x_true_new;
    x_dr = x_dr_new;
    hz.push(z);

    while time <= SIM_TIME {
        time += DT;
        d_time += DT;

        let u = calc_input();
        let (x_true_new, z, x_dr_new, _ud) = observation(&x_true, &x_dr, &u, &rfid);

        x_true = x_true_new;
        x_dr = x_dr_new;

        h_x_true.push(x_true);
        h_x_dr.push(x_dr);
        hz.push(z);

        // Run optimization periodically
        if d_time >= SHOW_GRAPH_D_TIME {
            // Convert history to matrix format
            let nt = h_x_dr.len();
            let mut h_x_dr_mat = DMatrix::<f64>::zeros(3, nt);
            for (i, pose) in h_x_dr.iter().enumerate() {
                h_x_dr_mat[(0, i)] = pose[0];
                h_x_dr_mat[(1, i)] = pose[1];
                h_x_dr_mat[(2, i)] = pose[2];
            }

            let x_opt = graph_based_slam(&h_x_dr_mat, &hz);
            d_time = 0.0;

            if SHOW_ANIMATION {
                fig.clear_axes();

                // True trajectory
                let true_x: Vec<f64> = h_x_true.iter().map(|p| p[0]).collect();
                let true_y: Vec<f64> = h_x_true.iter().map(|p| p[1]).collect();

                // Dead reckoning trajectory
                let dr_x: Vec<f64> = h_x_dr.iter().map(|p| p[0]).collect();
                let dr_y: Vec<f64> = h_x_dr.iter().map(|p| p[1]).collect();

                // Optimized trajectory
                let opt_x: Vec<f64> = (0..x_opt.ncols()).map(|i| x_opt[(0, i)]).collect();
                let opt_y: Vec<f64> = (0..x_opt.ncols()).map(|i| x_opt[(1, i)]).collect();

                // Landmarks
                let lm_x: Vec<f64> = rfid.iter().map(|p| p.0).collect();
                let lm_y: Vec<f64> = rfid.iter().map(|p| p.1).collect();

                fig.axes2d()
                    .set_title(&format!("Graph-based SLAM (Time: {:.1}s)", time), &[])
                    .set_x_label("x [m]", &[])
                    .set_y_label("y [m]", &[])
                    .set_aspect_ratio(gnuplot::Fix(1.0))
                    .points(&lm_x, &lm_y, &[Caption("Landmarks"), Color("black"), PointSymbol('*'), PointSize(2.0)])
                    .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
                    .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("black")])
                    .lines(&opt_x, &opt_y, &[Caption("Graph SLAM"), Color("red")]);

                fig.show_and_keep_running().unwrap();
            }
        }
    }

    println!("Done!");

    // Final optimization
    let nt = h_x_dr.len();
    let mut h_x_dr_mat = DMatrix::<f64>::zeros(3, nt);
    for (i, pose) in h_x_dr.iter().enumerate() {
        h_x_dr_mat[(0, i)] = pose[0];
        h_x_dr_mat[(1, i)] = pose[1];
        h_x_dr_mat[(2, i)] = pose[2];
    }

    let x_opt = graph_based_slam(&h_x_dr_mat, &hz);

    // Save final plot
    fig.clear_axes();

    let true_x: Vec<f64> = h_x_true.iter().map(|p| p[0]).collect();
    let true_y: Vec<f64> = h_x_true.iter().map(|p| p[1]).collect();
    let dr_x: Vec<f64> = h_x_dr.iter().map(|p| p[0]).collect();
    let dr_y: Vec<f64> = h_x_dr.iter().map(|p| p[1]).collect();
    let opt_x: Vec<f64> = (0..x_opt.ncols()).map(|i| x_opt[(0, i)]).collect();
    let opt_y: Vec<f64> = (0..x_opt.ncols()).map(|i| x_opt[(1, i)]).collect();
    let lm_x: Vec<f64> = rfid.iter().map(|p| p.0).collect();
    let lm_y: Vec<f64> = rfid.iter().map(|p| p.1).collect();

    fig.axes2d()
        .set_title("Graph-based SLAM", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(gnuplot::Fix(1.0))
        .points(&lm_x, &lm_y, &[Caption("Landmarks"), Color("black"), PointSymbol('*'), PointSize(2.0)])
        .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
        .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("black")])
        .lines(&opt_x, &opt_y, &[Caption("Graph SLAM"), Color("red")]);

    // Save SVG directly without gnuplot dependency
    let svg_path = "./img/slam/graph_based_slam.svg";
    save_svg(svg_path, &true_x, &true_y, &dr_x, &dr_y, &opt_x, &opt_y, &lm_x, &lm_y);
    println!("Plot saved to {}", svg_path);

    // Print final error
    let true_final = h_x_true.last().unwrap();
    let dr_final = h_x_dr.last().unwrap();
    let opt_final_x = x_opt[(0, x_opt.ncols() - 1)];
    let opt_final_y = x_opt[(1, x_opt.ncols() - 1)];

    let dr_error = ((dr_final[0] - true_final[0]).powi(2) + (dr_final[1] - true_final[1]).powi(2)).sqrt();
    let opt_error = ((opt_final_x - true_final[0]).powi(2) + (opt_final_y - true_final[1]).powi(2)).sqrt();

    println!("\nFinal position errors:");
    println!("  Dead Reckoning error: {:.4} m", dr_error);
    println!("  Graph SLAM error: {:.4} m", opt_error);
}
