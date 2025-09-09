// Reeds-Shepp Path Planner
//
// Author: Atsushi Sakai(@Atsushi_twi)
//         Videh Patel(@videh25) : Added the missing RS paths
//         Rust implementation

use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct Path {
    pub lengths: Vec<f64>,
    pub ctypes: Vec<char>,
    pub l: f64,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub yaw: Vec<f64>,
    pub directions: Vec<i32>,
}

impl Path {
    pub fn new() -> Self {
        Path {
            lengths: Vec::new(),
            ctypes: Vec::new(),
            l: 0.0,
            x: Vec::new(),
            y: Vec::new(),
            yaw: Vec::new(),
            directions: Vec::new(),
        }
    }
}

fn pi_2_pi(x: f64) -> f64 {
    let mut result = x;
    while result > PI {
        result -= 2.0 * PI;
    }
    while result < -PI {
        result += 2.0 * PI;
    }
    result
}

fn mod2pi(x: f64) -> f64 {
    let v = x % (2.0 * PI);
    if v < -PI {
        v + 2.0 * PI
    } else if v > PI {
        v - 2.0 * PI
    } else {
        v
    }
}

fn polar(x: f64, y: f64) -> (f64, f64) {
    let r = (x * x + y * y).sqrt();
    let theta = y.atan2(x);
    (r, theta)
}

fn left_straight_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let (u, t) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if 0.0 <= t && t <= PI {
        let v = mod2pi(phi - t);
        if 0.0 <= v && v <= PI {
            return (true, vec![t, u, v], vec!['L', 'S', 'L']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_straight_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let (u1, t1) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u1_sq = u1 * u1;
    if u1_sq >= 4.0 {
        let u = (u1_sq - 4.0).sqrt();
        let theta = (2.0_f64).atan2(u);
        let t = mod2pi(t1 + theta);
        let v = mod2pi(t - phi);
        
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, u, v], vec!['L', 'S', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right_x_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = mod2pi(a + theta + PI / 2.0);
        let u = mod2pi(PI - 2.0 * a);
        let v = mod2pi(phi - t - u);
        return (true, vec![t, -u, v], vec!['L', 'R', 'L']);
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = mod2pi(a + theta + PI / 2.0);
        let u = mod2pi(PI - 2.0 * a);
        let v = mod2pi(-phi + t + u);
        return (true, vec![t, -u, -v], vec!['L', 'R', 'L']);
    }
    (false, Vec::new(), Vec::new())
}

fn left_right_x_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 <= 4.0 {
        let u = (1.0 - u1 * u1 * 0.125).acos();
        let a = (2.0 * u.sin() / u1).asin();
        let t = mod2pi(-a + theta + PI / 2.0);
        let v = mod2pi(t - u - phi);
        return (true, vec![t, u, -v], vec!['L', 'R', 'L']);
    }
    (false, Vec::new(), Vec::new())
}

fn left_right_x_left_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 <= 2.0 {
        let a = ((u1 + 2.0) * 0.25).acos();
        let t = mod2pi(theta + a + PI / 2.0);
        let u = mod2pi(a);
        let v = mod2pi(phi - t + 2.0 * u);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return (true, vec![t, u, -u, -v], vec!['L', 'R', 'L', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right_left_x_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    let u2 = (20.0 - u1 * u1) / 16.0;
    
    if 0.0 <= u2 && u2 <= 1.0 {
        let u = u2.acos();
        let a = (2.0 * u.sin() / u1).asin();
        let t = mod2pi(theta + a + PI / 2.0);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, -u, -u, v], vec!['L', 'R', 'L', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right90_straight_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 >= 2.0 {
        let u = (u1 * u1 - 4.0).sqrt() - 2.0;
        let a = (2.0_f64).atan2((u1 * u1 - 4.0).sqrt());
        let t = mod2pi(theta + a + PI / 2.0);
        let v = mod2pi(t - phi + PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, -PI / 2.0, -u, -v], vec!['L', 'R', 'S', 'L']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_straight_right90_x_left(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 >= 2.0 {
        let u = (u1 * u1 - 4.0).sqrt() - 2.0;
        let a = ((u1 * u1 - 4.0).sqrt()).atan2(2.0);
        let t = mod2pi(theta - a + PI / 2.0);
        let v = mod2pi(t - phi - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, u, PI / 2.0, -v], vec!['L', 'S', 'R', 'L']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right90_straight_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 >= 2.0 {
        let t = mod2pi(theta + PI / 2.0);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, -PI / 2.0, -u, -v], vec!['L', 'R', 'S', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_straight_left90_x_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 >= 2.0 {
        let t = mod2pi(theta);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, u, PI / 2.0, -v], vec!['L', 'S', 'L', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn left_x_right90_straight_left90_x_right(x: f64, y: f64, phi: f64) -> (bool, Vec<f64>, Vec<char>) {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    
    if u1 >= 4.0 {
        let u = (u1 * u1 - 4.0).sqrt() - 4.0;
        let a = (2.0_f64).atan2((u1 * u1 - 4.0).sqrt());
        let t = mod2pi(theta + a + PI / 2.0);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return (true, vec![t, -PI / 2.0, -u, -PI / 2.0, v], vec!['L', 'R', 'S', 'L', 'R']);
        }
    }
    (false, Vec::new(), Vec::new())
}

fn timeflip(travel_distances: Vec<f64>) -> Vec<f64> {
    travel_distances.iter().map(|x| -x).collect()
}

fn reflect(steering_directions: Vec<char>) -> Vec<char> {
    steering_directions.iter().map(|&dirn| {
        match dirn {
            'L' => 'R',
            'R' => 'L',
            _ => 'S',
        }
    }).collect()
}

fn set_path(paths: &mut Vec<Path>, lengths: Vec<f64>, ctypes: Vec<char>, step_size: f64) {
    let mut path = Path::new();
    path.ctypes = ctypes;
    path.lengths = lengths;
    path.l = path.lengths.iter().map(|x| x.abs()).sum();
    
    // Check if same path exists
    for existing_path in paths.iter() {
        let type_is_same = existing_path.ctypes == path.ctypes;
        let length_is_close = (existing_path.l - path.l).abs() <= step_size;
        if type_is_same && length_is_close {
            return; // Same path found, do not insert
        }
    }
    
    // Check path is long enough
    if path.l <= step_size {
        return; // Too short, do not insert
    }
    
    paths.push(path);
}

fn generate_path(q0: [f64; 3], q1: [f64; 3], max_curvature: f64, step_size: f64) -> Vec<Path> {
    let dx = q1[0] - q0[0];
    let dy = q1[1] - q0[1];
    let dth = q1[2] - q0[2];
    let c = q0[2].cos();
    let s = q0[2].sin();
    let x = (c * dx + s * dy) * max_curvature;
    let y = (-s * dx + c * dy) * max_curvature;
    let step_size = step_size * max_curvature;
    
    let mut paths = Vec::new();
    
    let path_functions: Vec<fn(f64, f64, f64) -> (bool, Vec<f64>, Vec<char>)> = vec![
        left_straight_left, left_straight_right,
        left_x_right_x_left, left_x_right_left, left_right_x_left,
        left_right_x_left_right, left_x_right_left_x_right,
        left_x_right90_straight_left, left_x_right90_straight_right,
        left_straight_right90_x_left, left_straight_left90_x_right,
        left_x_right90_straight_left90_x_right,
    ];
    
    for path_func in path_functions {
        // Original
        let (flag, travel_distances, steering_dirns) = path_func(x, y, dth);
        if flag {
            let min_dist = 0.1 * travel_distances.iter().map(|d| d.abs()).sum::<f64>();
            let valid = travel_distances.iter().all(|&d| d.abs() >= min_dist || d.abs() >= step_size);
            if valid {
                set_path(&mut paths, travel_distances, steering_dirns, step_size);
            }
        }
        
        // Timeflip
        let (flag, travel_distances, steering_dirns) = path_func(-x, y, -dth);
        if flag {
            let min_dist = 0.1 * travel_distances.iter().map(|d| d.abs()).sum::<f64>();
            let valid = travel_distances.iter().all(|&d| d.abs() >= min_dist || d.abs() >= step_size);
            if valid {
                let travel_distances = timeflip(travel_distances);
                set_path(&mut paths, travel_distances, steering_dirns, step_size);
            }
        }
        
        // Reflect
        let (flag, travel_distances, steering_dirns) = path_func(x, -y, -dth);
        if flag {
            let min_dist = 0.1 * travel_distances.iter().map(|d| d.abs()).sum::<f64>();
            let valid = travel_distances.iter().all(|&d| d.abs() >= min_dist || d.abs() >= step_size);
            if valid {
                let steering_dirns = reflect(steering_dirns);
                set_path(&mut paths, travel_distances, steering_dirns, step_size);
            }
        }
        
        // Timeflip + Reflect
        let (flag, travel_distances, steering_dirns) = path_func(-x, -y, dth);
        if flag {
            let min_dist = 0.1 * travel_distances.iter().map(|d| d.abs()).sum::<f64>();
            let valid = travel_distances.iter().all(|&d| d.abs() >= min_dist || d.abs() >= step_size);
            if valid {
                let travel_distances = timeflip(travel_distances);
                let steering_dirns = reflect(steering_dirns);
                set_path(&mut paths, travel_distances, steering_dirns, step_size);
            }
        }
    }
    
    paths
}

fn calc_interpolate_dists_list(lengths: &[f64], step_size: f64) -> Vec<Vec<f64>> {
    let mut interpolate_dists_list = Vec::new();
    
    for &length in lengths {
        let d_dist = if length >= 0.0 { step_size } else { -step_size };
        let mut interp_dists = Vec::new();
        let mut current = 0.0;
        
        while (length >= 0.0 && current < length) || (length < 0.0 && current > length) {
            interp_dists.push(current);
            current += d_dist;
        }
        interp_dists.push(length);
        interpolate_dists_list.push(interp_dists);
    }
    
    interpolate_dists_list
}

fn interpolate(dist: f64, length: f64, mode: char, max_curvature: f64, 
               origin_x: f64, origin_y: f64, origin_yaw: f64) -> (f64, f64, f64, i32) {
    if mode == 'S' {
        let x = origin_x + dist / max_curvature * origin_yaw.cos();
        let y = origin_y + dist / max_curvature * origin_yaw.sin();
        let yaw = origin_yaw;
        (x, y, yaw, if length > 0.0 { 1 } else { -1 })
    } else {
        let ldx = dist.sin() / max_curvature;
        let ldy;
        let yaw;
        
        if mode == 'L' {
            ldy = (1.0 - dist.cos()) / max_curvature;
            yaw = origin_yaw + dist;
        } else { // 'R'
            ldy = (1.0 - dist.cos()) / -max_curvature;
            yaw = origin_yaw - dist;
        }
        
        let gdx = (-origin_yaw).cos() * ldx + (-origin_yaw).sin() * ldy;
        let gdy = -(-origin_yaw).sin() * ldx + (-origin_yaw).cos() * ldy;
        let x = origin_x + gdx;
        let y = origin_y + gdy;
        
        (x, y, yaw, if length > 0.0 { 1 } else { -1 })
    }
}

fn generate_local_course(lengths: &[f64], modes: &[char], max_curvature: f64, step_size: f64) 
    -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<i32>) {
    let interpolate_dists_list = calc_interpolate_dists_list(lengths, step_size * max_curvature);
    
    let mut origin_x = 0.0;
    let mut origin_y = 0.0;
    let mut origin_yaw = 0.0;
    
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    let mut yaws = Vec::new();
    let mut directions = Vec::new();
    
    for ((interp_dists, &mode), &length) in interpolate_dists_list.iter().zip(modes).zip(lengths) {
        for &dist in interp_dists {
            let (x, y, yaw, direction) = interpolate(dist, length, mode, max_curvature, 
                                                   origin_x, origin_y, origin_yaw);
            xs.push(x);
            ys.push(y);
            yaws.push(yaw);
            directions.push(direction);
        }
        if !xs.is_empty() {
            origin_x = xs[xs.len() - 1];
            origin_y = ys[ys.len() - 1];
            origin_yaw = yaws[yaws.len() - 1];
        }
    }
    
    (xs, ys, yaws, directions)
}

fn calc_paths(sx: f64, sy: f64, syaw: f64, gx: f64, gy: f64, gyaw: f64, 
              maxc: f64, step_size: f64) -> Vec<Path> {
    let q0 = [sx, sy, syaw];
    let q1 = [gx, gy, gyaw];
    
    let mut paths = generate_path(q0, q1, maxc, step_size);
    
    for path in &mut paths {
        let (xs, ys, yaws, directions) = generate_local_course(&path.lengths, &path.ctypes, maxc, step_size);
        
        // Convert to global coordinate
        path.x = xs.iter().zip(ys.iter()).map(|(&ix, &iy)| {
            (-q0[2]).cos() * ix + (-q0[2]).sin() * iy + q0[0]
        }).collect();
        
        path.y = xs.iter().zip(ys.iter()).map(|(&ix, &iy)| {
            -(-q0[2]).sin() * ix + (-q0[2]).cos() * iy + q0[1]
        }).collect();
        
        path.yaw = yaws.iter().map(|&yaw| pi_2_pi(yaw + q0[2])).collect();
        path.directions = directions;
        path.lengths = path.lengths.iter().map(|&length| length / maxc).collect();
        path.l = path.l / maxc;
    }
    
    paths
}

pub fn reeds_shepp_path_planning(sx: f64, sy: f64, syaw: f64, gx: f64, gy: f64, gyaw: f64, 
                                maxc: f64, step_size: f64) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<char>, Vec<f64>)> {
    let paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);
    
    if paths.is_empty() {
        return None;
    }
    
    // Find minimum cost path
    let best_path = paths.iter().min_by(|a, b| a.l.partial_cmp(&b.l).unwrap())?;
    
    Some((best_path.x.clone(), best_path.y.clone(), best_path.yaw.clone(), 
          best_path.ctypes.clone(), best_path.lengths.clone()))
}

pub struct ReedsSheppPlanner {
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub path_yaw: Vec<f64>,
    pub modes: Vec<char>,
    pub lengths: Vec<f64>,
}

impl ReedsSheppPlanner {
    pub fn new() -> Self {
        ReedsSheppPlanner {
            path_x: Vec::new(),
            path_y: Vec::new(),
            path_yaw: Vec::new(),
            modes: Vec::new(),
            lengths: Vec::new(),
        }
    }
    
    pub fn planning(&mut self, sx: f64, sy: f64, syaw: f64, gx: f64, gy: f64, gyaw: f64, 
                   max_curvature: f64, step_size: f64) -> bool {
        if let Some((x, y, yaw, modes, lengths)) = reeds_shepp_path_planning(
            sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size) {
            self.path_x = x;
            self.path_y = y;
            self.path_yaw = yaw;
            self.modes = modes;
            self.lengths = lengths;
            println!("Reeds-Shepp path generated with {} points", self.path_x.len());
            true
        } else {
            println!("Failed to generate Reeds-Shepp path");
            false
        }
    }
    
    pub fn visualize(&self, sx: f64, sy: f64, syaw: f64, gx: f64, gy: f64, gyaw: f64) {
        if self.path_x.is_empty() {
            println!("No path to visualize");
            return;
        }
        
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        
        // Plot path
        axes.lines(&self.path_x, &self.path_y, &[Caption("Reeds-Shepp Path"), Color("blue")]);
        
        // Plot start and goal arrows
        let arrow_length = 0.5;
        let start_arrow_x = vec![sx, sx + arrow_length * syaw.cos()];
        let start_arrow_y = vec![sy, sy + arrow_length * syaw.sin()];
        let goal_arrow_x = vec![gx, gx + arrow_length * gyaw.cos()];
        let goal_arrow_y = vec![gy, gy + arrow_length * gyaw.sin()];
        
        axes.lines(&start_arrow_x, &start_arrow_y, &[Caption("Start"), Color("red")]);
        axes.lines(&goal_arrow_x, &goal_arrow_y, &[Caption("Goal"), Color("green")]);
        
        axes.set_title("Reeds-Shepp Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));
        
        let output_path = "img/path_planning/reeds_shepp_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);
        
        fg.show().unwrap();
    }
}

fn main() {
    println!("Reeds Shepp path planner start!!");
    
    let start_x = -1.0;
    let start_y = -4.0;
    let start_yaw = (-20.0_f64).to_radians();
    
    let end_x = 5.0;
    let end_y = 5.0;
    let end_yaw = (25.0_f64).to_radians();
    
    let curvature = 0.1;
    let step_size = 0.05;
    
    let mut planner = ReedsSheppPlanner::new();
    
    if planner.planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size) {
        println!("Path modes: {:?}", planner.modes);
        println!("Path lengths: {:?}", planner.lengths);
        planner.visualize(start_x, start_y, start_yaw, end_x, end_y, end_yaw);
    } else {
        println!("Failed to generate path");
    }
    
    println!("Reeds Shepp path planner finish!!");
}
