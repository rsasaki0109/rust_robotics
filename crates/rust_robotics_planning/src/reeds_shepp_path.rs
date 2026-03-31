#![allow(
    dead_code,
    clippy::needless_borrows_for_generic_args,
    clippy::new_without_default,
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::assign_op_pattern
)]

//! Reeds-Shepp Path Planner
//!
//! Computes optimal paths for a car-like robot that can move both
//! forward and backward, using combinations of straight and arc segments.

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
    if (0.0..=PI).contains(&t) {
        let v = mod2pi(phi - t);
        if (0.0..=PI).contains(&v) {
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

    if (0.0..=1.0).contains(&u2) {
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
            return (
                true,
                vec![t, -PI / 2.0, -u, -PI / 2.0, v],
                vec!['L', 'R', 'S', 'L', 'R'],
            );
        }
    }
    (false, Vec::new(), Vec::new())
}

fn timeflip(travel_distances: Vec<f64>) -> Vec<f64> {
    travel_distances.iter().map(|x| -x).collect()
}

fn reflect(steering_directions: Vec<char>) -> Vec<char> {
    steering_directions
        .iter()
        .map(|&dirn| match dirn {
            'L' => 'R',
            'R' => 'L',
            _ => 'S',
        })
        .collect()
}

fn set_path(paths: &mut Vec<Path>, lengths: Vec<f64>, ctypes: Vec<char>, step_size: f64) {
    let mut path = Path::new();
    path.ctypes = ctypes;
    path.lengths = lengths;
    path.l = path.lengths.iter().map(|x| x.abs()).sum();

    for existing_path in paths.iter() {
        let type_is_same = existing_path.ctypes == path.ctypes;
        let length_is_close = (existing_path.l - path.l) <= step_size;
        if type_is_same && length_is_close {
            return;
        }
    }

    if path.l <= step_size {
        return;
    }

    paths.push(path);
}

fn has_too_small_segment(travel_distances: &[f64], step_size: f64) -> bool {
    let min_dist = 0.1 * travel_distances.iter().map(|d| d.abs()).sum::<f64>();
    travel_distances
        .iter()
        .any(|&distance| min_dist < distance.abs() && distance.abs() < step_size)
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
        left_straight_left,
        left_straight_right,
        left_x_right_x_left,
        left_x_right_left,
        left_right_x_left,
        left_right_x_left_right,
        left_x_right_left_x_right,
        left_x_right90_straight_left,
        left_x_right90_straight_right,
        left_straight_right90_x_left,
        left_straight_left90_x_right,
        left_x_right90_straight_left90_x_right,
    ];

    for path_func in path_functions {
        // Original
        let (flag, travel_distances, steering_dirns) = path_func(x, y, dth);
        if flag {
            if has_too_small_segment(&travel_distances, step_size) {
                return Vec::new();
            }
            set_path(&mut paths, travel_distances, steering_dirns, step_size);
        }

        // Timeflip
        let (flag, travel_distances, steering_dirns) = path_func(-x, y, -dth);
        if flag {
            if has_too_small_segment(&travel_distances, step_size) {
                return Vec::new();
            }
            let travel_distances = timeflip(travel_distances);
            set_path(&mut paths, travel_distances, steering_dirns, step_size);
        }

        // Reflect
        let (flag, travel_distances, steering_dirns) = path_func(x, -y, -dth);
        if flag {
            if has_too_small_segment(&travel_distances, step_size) {
                return Vec::new();
            }
            let steering_dirns = reflect(steering_dirns);
            set_path(&mut paths, travel_distances, steering_dirns, step_size);
        }

        // Timeflip + Reflect
        let (flag, travel_distances, steering_dirns) = path_func(-x, -y, dth);
        if flag {
            if has_too_small_segment(&travel_distances, step_size) {
                return Vec::new();
            }
            let travel_distances = timeflip(travel_distances);
            let steering_dirns = reflect(steering_dirns);
            set_path(&mut paths, travel_distances, steering_dirns, step_size);
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

fn interpolate(
    dist: f64,
    length: f64,
    mode: char,
    max_curvature: f64,
    origin_x: f64,
    origin_y: f64,
    origin_yaw: f64,
) -> (f64, f64, f64, i32) {
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
        } else {
            // 'R'
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

fn generate_local_course(
    lengths: &[f64],
    modes: &[char],
    max_curvature: f64,
    step_size: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<i32>) {
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
            let (x, y, yaw, direction) = interpolate(
                dist,
                length,
                mode,
                max_curvature,
                origin_x,
                origin_y,
                origin_yaw,
            );
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

fn calc_paths(
    sx: f64,
    sy: f64,
    syaw: f64,
    gx: f64,
    gy: f64,
    gyaw: f64,
    maxc: f64,
    step_size: f64,
) -> Vec<Path> {
    let q0 = [sx, sy, syaw];
    let q1 = [gx, gy, gyaw];

    let mut paths = generate_path(q0, q1, maxc, step_size);

    for path in &mut paths {
        let (xs, ys, yaws, directions) =
            generate_local_course(&path.lengths, &path.ctypes, maxc, step_size);

        path.x = xs
            .iter()
            .zip(ys.iter())
            .map(|(&ix, &iy)| (-q0[2]).cos() * ix + (-q0[2]).sin() * iy + q0[0])
            .collect();

        path.y = xs
            .iter()
            .zip(ys.iter())
            .map(|(&ix, &iy)| -(-q0[2]).sin() * ix + (-q0[2]).cos() * iy + q0[1])
            .collect();

        path.yaw = yaws.iter().map(|&yaw| pi_2_pi(yaw + q0[2])).collect();
        path.directions = directions;
        path.lengths = path.lengths.iter().map(|&length| length / maxc).collect();
        path.l = path.l / maxc;
    }

    paths
}

pub fn reeds_shepp_path_planning(
    sx: f64,
    sy: f64,
    syaw: f64,
    gx: f64,
    gy: f64,
    gyaw: f64,
    maxc: f64,
    step_size: f64,
) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<char>, Vec<f64>)> {
    let paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);

    if paths.is_empty() {
        return None;
    }

    let best_path = paths.iter().min_by(|a, b| a.l.partial_cmp(&b.l).unwrap())?;

    Some((
        best_path.x.clone(),
        best_path.y.clone(),
        best_path.yaw.clone(),
        best_path.ctypes.clone(),
        best_path.lengths.clone(),
    ))
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

    pub fn planning(
        &mut self,
        sx: f64,
        sy: f64,
        syaw: f64,
        gx: f64,
        gy: f64,
        gyaw: f64,
        max_curvature: f64,
        step_size: f64,
    ) -> bool {
        if let Some((x, y, yaw, modes, lengths)) =
            reeds_shepp_path_planning(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size)
        {
            self.path_x = x;
            self.path_y = y;
            self.path_yaw = yaw;
            self.modes = modes;
            self.lengths = lengths;
            true
        } else {
            false
        }
    }
}

impl Default for ReedsSheppPlanner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::excessive_precision)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn path_fingerprint(path: &Path) -> (f64, f64, f64, f64, f64, f64) {
        let sum_x: f64 = path.x.iter().sum();
        let sum_y: f64 = path.y.iter().sum();
        let sum_yaw: f64 = path.yaw.iter().sum();
        let weighted_sum_x: f64 = path
            .x
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        let weighted_sum_y: f64 = path
            .y
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        let weighted_sum_yaw: f64 = path
            .yaw
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();

        (
            sum_x,
            sum_y,
            sum_yaw,
            weighted_sum_x,
            weighted_sum_y,
            weighted_sum_yaw,
        )
    }

    fn assert_fingerprint_close(
        actual: (f64, f64, f64, f64, f64, f64),
        expected: (f64, f64, f64, f64, f64, f64),
    ) {
        assert!(approx_eq(actual.0, expected.0, 1e-9));
        assert!(approx_eq(actual.1, expected.1, 1e-9));
        assert!(approx_eq(actual.2, expected.2, 1e-9));
        assert!(approx_eq(actual.3, expected.3, 1e-6));
        assert!(approx_eq(actual.4, expected.4, 1e-6));
        assert!(approx_eq(actual.5, expected.5, 1e-6));
    }

    #[test]
    fn test_reeds_shepp_planning() {
        let mut planner = ReedsSheppPlanner::new();
        let result = planner.planning(
            -1.0,
            -4.0,
            (-20.0_f64).to_radians(),
            5.0,
            5.0,
            (25.0_f64).to_radians(),
            0.1,
            0.05,
        );
        assert!(result);
        assert!(!planner.path_x.is_empty());
        assert!(!planner.modes.is_empty());
    }

    #[test]
    fn test_reeds_shepp_path_planning_fn() {
        let result = reeds_shepp_path_planning(0.0, 0.0, 0.0, 5.0, 5.0, 0.5, 0.2, 0.1);
        assert!(result.is_some());
    }

    #[test]
    fn test_calc_paths_matches_upstream_main_example_candidates() {
        let paths = calc_paths(
            -1.0,
            -4.0,
            (-20.0_f64).to_radians(),
            5.0,
            5.0,
            25.0_f64.to_radians(),
            0.1,
            0.05,
        );

        assert_eq!(paths.len(), 5);

        let best_path = paths
            .iter()
            .min_by(|a, b| a.l.partial_cmp(&b.l).unwrap())
            .unwrap();
        assert_eq!(best_path.ctypes, vec!['R', 'L', 'R']);
        assert!(approx_eq(best_path.l, 19.647_318_205_087_014, 1e-12));
        assert!(approx_eq(
            best_path.lengths[0],
            -5.228_525_224_523_852,
            1e-12
        ));
        assert!(approx_eq(
            best_path.lengths[1],
            8.522_124_695_006_896,
            1e-12
        ));
        assert!(approx_eq(
            best_path.lengths[2],
            5.896_668_285_556_266,
            1e-12
        ));
        assert_eq!(best_path.x.len(), 397);
        assert_eq!(best_path.directions.len(), 397);
        assert_eq!(best_path.directions.iter().sum::<i32>(), 185);
        assert_eq!(
            best_path
                .directions
                .iter()
                .enumerate()
                .map(|(index, direction)| (index + 1) as i32 * direction)
                .sum::<i32>(),
            67_661
        );
    }

    #[test]
    fn test_reeds_shepp_path_planning_matches_upstream_main_example() {
        let (x, y, yaw, modes, lengths) = reeds_shepp_path_planning(
            -1.0,
            -4.0,
            (-20.0_f64).to_radians(),
            5.0,
            5.0,
            25.0_f64.to_radians(),
            0.1,
            0.05,
        )
        .unwrap();

        assert_eq!(modes, vec!['R', 'L', 'R']);
        assert!(approx_eq(lengths[0], -5.228_525_224_523_852, 1e-12));
        assert!(approx_eq(lengths[1], 8.522_124_695_006_896, 1e-12));
        assert!(approx_eq(lengths[2], 5.896_668_285_556_266, 1e-12));
        assert_eq!(x.len(), 397);
        assert_eq!(y.len(), 397);
        assert_eq!(yaw.len(), 397);

        let expected_samples = [
            (0usize, -1.0, -4.0, -0.349_065_850_398_865_95),
            (
                50,
                -3.431_162_528_468_660_4,
                -3.445_956_303_130_415_2,
                -0.099_065_850_398_865_95,
            ),
            (
                100,
                -5.923_818_705_704_953,
                -3.510_615_718_197_37,
                0.150_934_149_601_134_05,
            ),
            (
                150,
                -4.041_585_620_182_658,
                -2.932_812_755_977_549,
                0.393_786_672_053_519_47,
            ),
            (396, 5.000_000_000_000_005, 5.0, 0.436_332_312_998_582_33),
        ];

        for (index, expected_x, expected_y, expected_yaw) in expected_samples {
            assert!(approx_eq(x[index], expected_x, 1e-12));
            assert!(approx_eq(y[index], expected_y, 1e-12));
            assert!(approx_eq(yaw[index], expected_yaw, 1e-12));
        }

        let sum_x: f64 = x.iter().sum();
        let sum_y: f64 = y.iter().sum();
        let sum_yaw: f64 = yaw.iter().sum();
        let weighted_sum_x: f64 = x
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        let weighted_sum_y: f64 = y
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        let weighted_sum_yaw: f64 = yaw
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();

        assert!(approx_eq(sum_x, -474.632_647_731_712_4, 1e-9));
        assert!(approx_eq(sum_y, -278.042_684_871_068_6, 1e-9));
        assert!(approx_eq(sum_yaw, 181.229_623_459_273_85, 1e-9));
        assert!(approx_eq(weighted_sum_x, 24_450.581_151_074_42, 1e-6));
        assert!(approx_eq(weighted_sum_y, 72_297.919_198_954_09, 1e-6));
        assert!(approx_eq(weighted_sum_yaw, 50_733.291_448_638_01, 1e-6));
    }

    #[test]
    fn test_calc_paths_matches_upstream_representative_mode_coverage() {
        struct Scenario {
            goal: (f64, f64, f64),
            path_count: usize,
            mode: &'static [char],
            lengths: &'static [f64],
            total: f64,
            sample_count: usize,
            direction_sum: i32,
            direction_weighted: i32,
            fingerprint: (f64, f64, f64, f64, f64, f64),
        }

        let start = (0.0, 0.0, -PI / 2.0);
        let scenarios = [
            Scenario {
                goal: (2.0, -6.0, -PI / 4.0),
                path_count: 9,
                mode: &['L', 'S', 'L'],
                lengths: &[
                    1.069_877_996_599_901_2,
                    2.521_981_303_104_907_6,
                    2.857_112_820_387_34,
                ],
                total: 6.448_972_120_092_149,
                sample_count: 69,
                direction_sum: 69,
                direction_weighted: 2_415,
                fingerprint: (
                    47.658_101_248_550_764,
                    -214.691_563_475_246_82,
                    -86.182_939_224_211_54,
                    2_416.980_927_612_332_4,
                    -9_962.039_734_007_922,
                    -2_763.536_289_485_696,
                ),
            },
            Scenario {
                goal: (4.0, -6.0, -PI / 4.0),
                path_count: 8,
                mode: &['L', 'S', 'R'],
                lengths: &[
                    3.999_542_478_656_482,
                    3.390_792_633_449_932_8,
                    0.072_551_661_669_240_65,
                ],
                total: 7.462_886_773_775_654,
                sample_count: 78,
                direction_sum: 78,
                direction_weighted: 3_081,
                fingerprint: (
                    125.110_316_958_156_83,
                    -256.540_636_968_477_1,
                    -76.540_100_984_546_51,
                    7_190.184_713_371_897_5,
                    -13_176.153_323_150_214,
                    -2_605.762_179_928_655_3,
                ),
            },
            Scenario {
                goal: (-4.0, -6.0, -3.0 * PI / 4.0),
                path_count: 8,
                mode: &['R', 'S', 'L'],
                lengths: &[
                    3.999_542_478_656_482_4,
                    3.390_792_633_449_932_8,
                    0.072_551_661_669_241_21,
                ],
                total: 7.462_886_773_775_656,
                sample_count: 78,
                direction_sum: 78,
                direction_weighted: 3_081,
                fingerprint: (
                    -125.110_316_958_156_82,
                    -256.540_636_968_477_14,
                    -168.504_125_995_457_34,
                    -7_190.184_713_371_896,
                    -13_176.153_323_150_216,
                    -7_073.484_785_781_497_5,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, PI),
                path_count: 9,
                mode: &['R', 'S', 'R'],
                lengths: &[
                    3.926_990_816_987_239,
                    1.414_213_562_373_095_6,
                    3.926_990_816_987_246,
                ],
                total: 9.268_195_196_347_58,
                sample_count: 98,
                direction_sum: 98,
                direction_weighted: 4_851,
                fingerprint: (
                    -224.230_783_041_927_37,
                    -368.245_146_496_736_9,
                    -225.193_346_359_169_7,
                    -16_127.734_907_593_596,
                    -23_204.350_340_917_714,
                    -11_989.089_837_846_506,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, -3.0 * PI / 4.0),
                path_count: 6,
                mode: &['L', 'R', 'L'],
                lengths: &[
                    -0.068_966_848_925_894_33,
                    6.503_746_832_829_835,
                    2.645_722_864_768_488,
                ],
                total: 9.218_436_546_524_217,
                sample_count: 97,
                direction_sum: 93,
                direction_weighted: 4_747,
                fingerprint: (
                    -230.574_821_137_002_54,
                    -331.875_810_770_970_8,
                    -226.763_994_255_941_5,
                    -16_521.117_485_464_518,
                    -20_916.307_531_974_84,
                    -11_997.697_736_223_088,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, -PI / 4.0),
                path_count: 6,
                mode: &['R', 'L', 'R'],
                lengths: &[
                    5.000_868_664_582_23,
                    5.561_988_484_973_19,
                    -3.365_870_996_596_280_8,
                ],
                total: 13.928_728_146_151_7,
                sample_count: 144,
                direction_sum: 74,
                direction_weighted: 1_550,
                fingerprint: (
                    -443.806_209_833_689_9,
                    -744.765_650_985_955,
                    -261.971_336_928_568_35,
                    -43_201.590_996_218_62,
                    -67_652.561_148_743_36,
                    -16_979.120_856_723_654,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, -PI / 2.0),
                path_count: 6,
                mode: &['L', 'R', 'L', 'R'],
                lengths: &[
                    -0.823_349_563_605_059_2,
                    5.119_726_880_494_763,
                    5.119_726_880_494_763,
                    -0.823_349_563_605_059_2,
                ],
                total: 11.886_152_888_199_643,
                sample_count: 126,
                direction_sum: 86,
                direction_weighted: 5_461,
                fingerprint: (
                    -380.403_475_990_598_96,
                    -380.444_909_002_660_5,
                    -271.291_151_985_038_8,
                    -34_379.127_386_055_37,
                    -34_617.891_225_324_86,
                    -17_198.177_166_350_25,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, PI / 4.0),
                path_count: 4,
                mode: &['L', 'R', 'S', 'L'],
                lengths: &[
                    4.566_911_224_275_087,
                    -7.853_981_633_974_483,
                    -0.833_066_927_667_685,
                    -0.639_920_407_287_846_2,
                ],
                total: 13.893_880_193_205_101,
                sample_count: 145,
                direction_sum: -51,
                direction_weighted: -8_329,
                fingerprint: (
                    -203.973_341_223_846_2,
                    -476.554_271_600_198_1,
                    -25.723_479_016_967_794,
                    -28_387.645_373_178_693,
                    -41_452.300_874_567_176,
                    2_865.645_752_604_772,
                ),
            },
            Scenario {
                goal: (-4.0, -6.0, PI),
                path_count: 6,
                mode: &['L', 'S', 'R', 'L'],
                lengths: &[
                    0.473_590_382_777_882,
                    0.099_504_938_362_080_52,
                    7.853_981_633_974_483,
                    -0.473_590_382_777_882,
                ],
                total: 8.900_667_337_892_326,
                sample_count: 94,
                direction_sum: 82,
                direction_weighted: 3_367,
                fingerprint: (
                    -145.451_641_721_792_1,
                    -351.919_906_224_341_96,
                    -211.935_238_269_393_84,
                    -10_708.096_596_694_699,
                    -21_581.461_832_452_35,
                    -11_398.570_917_831_472,
                ),
            },
            Scenario {
                goal: (4.0, -6.0, 0.0),
                path_count: 6,
                mode: &['R', 'S', 'L', 'R'],
                lengths: &[
                    0.473_590_382_777_883_1,
                    0.099_504_938_362_076_08,
                    7.853_981_633_974_483,
                    -0.473_590_382_777_883_1,
                ],
                total: 8.900_667_337_892_324,
                sample_count: 94,
                direction_sum: 82,
                direction_weighted: 3_367,
                fingerprint: (
                    145.451_641_721_792_13,
                    -351.919_906_224_341_6,
                    -83.374_471_168_046_72,
                    10_708.096_596_694_699,
                    -21_581.461_832_452_34,
                    -2_628.640_280_446_955,
                ),
            },
            Scenario {
                goal: (-6.0, -6.0, 3.0 * PI / 4.0),
                path_count: 4,
                mode: &['R', 'S', 'R', 'L'],
                lengths: &[
                    2.219_874_422_095_635_2,
                    0.559_236_463_071_481_9,
                    7.853_981_633_974_483,
                    -1.707_116_394_891_605_7,
                ],
                total: 12.340_208_914_033_205,
                sample_count: 130,
                direction_sum: 92,
                direction_weighted: 3_917,
                fingerprint: (
                    -435.220_953_689_669,
                    -527.340_252_871_888_3,
                    -89.312_402_029_516_8,
                    -40_676.492_201_712_46,
                    -42_014.939_961_467_01,
                    2_377.125_258_488_301,
                ),
            },
            Scenario {
                goal: (-6.0, -2.0, -PI / 4.0),
                path_count: 5,
                mode: &['R', 'L', 'R', 'L'],
                lengths: &[
                    1.605_151_609_076_620_5,
                    4.327_807_515_918_15,
                    -4.327_807_515_918_15,
                    -3.123_472_605_772_438,
                ],
                total: 13.384_239_246_685_356,
                sample_count: 141,
                direction_sum: -15,
                direction_weighted: -5_979,
                fingerprint: (
                    -234.424_840_465_174_4,
                    -469.790_322_454_025_9,
                    -138.906_286_644_584_9,
                    -26_289.793_852_112_638,
                    -35_567.135_089_521_57,
                    -6_993.944_431_851_961,
                ),
            },
        ];

        for scenario in scenarios {
            let paths = calc_paths(
                start.0,
                start.1,
                start.2,
                scenario.goal.0,
                scenario.goal.1,
                scenario.goal.2,
                0.2,
                0.1,
            );
            assert_eq!(paths.len(), scenario.path_count);

            let best_path = paths
                .iter()
                .min_by(|a, b| a.l.partial_cmp(&b.l).unwrap())
                .unwrap();

            assert_eq!(best_path.ctypes, scenario.mode);
            assert_eq!(best_path.lengths.len(), scenario.lengths.len());
            for (actual, expected) in best_path.lengths.iter().zip(scenario.lengths.iter()) {
                assert!(approx_eq(*actual, *expected, 1e-12));
            }
            assert!(approx_eq(best_path.l, scenario.total, 1e-12));
            assert_eq!(best_path.x.len(), scenario.sample_count);
            assert_eq!(best_path.y.len(), scenario.sample_count);
            assert_eq!(best_path.yaw.len(), scenario.sample_count);
            assert_eq!(
                best_path.directions.iter().sum::<i32>(),
                scenario.direction_sum
            );
            assert_eq!(
                best_path
                    .directions
                    .iter()
                    .enumerate()
                    .map(|(index, direction)| (index + 1) as i32 * direction)
                    .sum::<i32>(),
                scenario.direction_weighted
            );
            assert!(approx_eq(
                *best_path.x.last().unwrap(),
                scenario.goal.0,
                1e-12
            ));
            assert!(approx_eq(
                *best_path.y.last().unwrap(),
                scenario.goal.1,
                1e-12
            ));
            assert!(approx_eq(
                pi_2_pi(*best_path.yaw.last().unwrap() - scenario.goal.2),
                0.0,
                1e-12
            ));
            assert_fingerprint_close(path_fingerprint(best_path), scenario.fingerprint);
        }
    }
}
