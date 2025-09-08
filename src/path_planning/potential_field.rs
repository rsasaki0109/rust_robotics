use std::collections::VecDeque;
use gnuplot::{Figure, Caption, Color, AxesCommon};

// Parameters
const KP: f64 = 5.0;  // attractive potential gain
const ETA: f64 = 100.0;  // repulsive potential gain
const AREA_WIDTH: f64 = 30.0;  // potential area width [m]
const OSCILLATIONS_DETECTION_LENGTH: usize = 3;  // number of previous positions used to check oscillations
const SHOW_ANIMATION: bool = true;

pub struct PotentialFieldPlanner {
    pub resolution: f64,
    pub robot_radius: f64,
}

impl PotentialFieldPlanner {
    pub fn new(resolution: f64, robot_radius: f64) -> Self {
        PotentialFieldPlanner {
            resolution,
            robot_radius,
        }
    }

    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64, ox: &[f64], oy: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        // Calculate potential field
        let (pmap, minx, miny) = self.calc_potential_field(gx, gy, ox, oy, sx, sy);

        // Search path
        let mut d = ((sx - gx).powi(2) + (sy - gy).powi(2)).sqrt();
        let mut ix = ((sx - minx) / self.resolution).round() as i32;
        let mut iy = ((sy - miny) / self.resolution).round() as i32;
        let gix = ((gx - minx) / self.resolution).round() as i32;
        let giy = ((gy - miny) / self.resolution).round() as i32;

        println!("Start: ({}, {}), Goal: ({}, {})", ix, iy, gix, giy);

        let mut rx = vec![sx];
        let mut ry = vec![sy];
        let motion = self.get_motion_model();
        let mut previous_ids = VecDeque::new();

        while d >= self.resolution {
            let mut minp = f64::INFINITY;
            let mut minix = -1;
            let mut miniy = -1;

            for motion_step in &motion {
                let inx = ix + motion_step[0];
                let iny = iy + motion_step[1];
                
                let p = if inx >= pmap.len() as i32 || iny >= pmap[0].len() as i32 || inx < 0 || iny < 0 {
                    f64::INFINITY  // outside area
                } else {
                    pmap[inx as usize][iny as usize]
                };

                if minp > p {
                    minp = p;
                    minix = inx;
                    miniy = iny;
                }
            }

            ix = minix;
            iy = miniy;
            let xp = ix as f64 * self.resolution + minx;
            let yp = iy as f64 * self.resolution + miny;
            d = ((gx - xp).powi(2) + (gy - yp).powi(2)).sqrt();
            rx.push(xp);
            ry.push(yp);

            if self.oscillations_detection(&mut previous_ids, ix, iy) {
                println!("Oscillation detected at ({},{})!", ix, iy);
                break;
            }
        }

        println!("Goal reached!");
        Some((rx, ry))
    }

    fn calc_potential_field(&self, gx: f64, gy: f64, ox: &[f64], oy: &[f64], sx: f64, sy: f64) -> (Vec<Vec<f64>>, f64, f64) {
        let minx = [ox.iter().fold(f64::INFINITY, |a, &b| a.min(b)), sx, gx].iter().fold(f64::INFINITY, |a, &b| a.min(b)) - AREA_WIDTH / 2.0;
        let miny = [oy.iter().fold(f64::INFINITY, |a, &b| a.min(b)), sy, gy].iter().fold(f64::INFINITY, |a, &b| a.min(b)) - AREA_WIDTH / 2.0;
        let maxx = [ox.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)), sx, gx].iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)) + AREA_WIDTH / 2.0;
        let maxy = [oy.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)), sy, gy].iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)) + AREA_WIDTH / 2.0;
        
        let xw = ((maxx - minx) / self.resolution).round() as usize;
        let yw = ((maxy - miny) / self.resolution).round() as usize;

        // Calculate each potential
        let mut pmap = vec![vec![0.0; yw]; xw];

        for ix in 0..xw {
            let x = ix as f64 * self.resolution + minx;
            for iy in 0..yw {
                let y = iy as f64 * self.resolution + miny;
                let ug = self.calc_attractive_potential(x, y, gx, gy);
                let uo = self.calc_repulsive_potential(x, y, ox, oy);
                let uf = ug + uo;
                pmap[ix][iy] = uf;
            }
        }

        (pmap, minx, miny)
    }

    fn calc_attractive_potential(&self, x: f64, y: f64, gx: f64, gy: f64) -> f64 {
        0.5 * KP * ((x - gx).powi(2) + (y - gy).powi(2)).sqrt()
    }

    fn calc_repulsive_potential(&self, x: f64, y: f64, ox: &[f64], oy: &[f64]) -> f64 {
        // Search nearest obstacle
        let mut dmin = f64::INFINITY;
        let mut minid = 0;

        for (i, (&ox_i, &oy_i)) in ox.iter().zip(oy.iter()).enumerate() {
            let d = ((x - ox_i).powi(2) + (y - oy_i).powi(2)).sqrt();
            if dmin >= d {
                dmin = d;
                minid = i;
            }
        }

        // Calculate repulsive potential
        let dq = ((x - ox[minid]).powi(2) + (y - oy[minid]).powi(2)).sqrt();

        if dq <= self.robot_radius {
            let dq = if dq <= 0.1 { 0.1 } else { dq };
            0.5 * ETA * (1.0 / dq - 1.0 / self.robot_radius).powi(2)
        } else {
            0.0
        }
    }

    fn get_motion_model(&self) -> Vec<[i32; 2]> {
        vec![
            [1, 0],
            [0, 1],
            [-1, 0],
            [0, -1],
            [-1, -1],
            [-1, 1],
            [1, -1],
            [1, 1],
        ]
    }

    fn oscillations_detection(&self, previous_ids: &mut VecDeque<(i32, i32)>, ix: i32, iy: i32) -> bool {
        previous_ids.push_back((ix, iy));

        if previous_ids.len() > OSCILLATIONS_DETECTION_LENGTH {
            previous_ids.pop_front();
        }

        // Check if contains any duplicates
        let mut seen = std::collections::HashSet::new();
        for &index in previous_ids.iter() {
            if seen.contains(&index) {
                return true;
            }
            seen.insert(index);
        }
        false
    }

    pub fn visualize_path(&self, rx: &[f64], ry: &[f64], sx: f64, sy: f64, gx: f64, gy: f64, ox: &[f64], oy: &[f64]) {
        if !SHOW_ANIMATION {
            return;
        }

        let mut fg = Figure::new();
        let mut axes = fg.axes2d();

        // Plot obstacles
        axes.points(ox, oy, &[Caption("Obstacles"), Color("black")]);

        // Plot path
        axes.lines(rx, ry, &[Caption("Potential Field Path"), Color("red")]);

        // Plot start and goal
        axes.points(&[sx], &[sy], &[Caption("Start"), Color("green")]);
        axes.points(&[gx], &[gy], &[Caption("Goal"), Color("blue")]);

        axes.set_title("Potential Field Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Save to file
        let output_path = "img/path_planning/potential_field_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);

        fg.show().unwrap();
    }
}

fn main() {
    println!("Potential Field path planning start!!");

    let sx = 0.0;  // start x position [m]
    let sy = 10.0; // start y position [m]
    let gx = 30.0; // goal x position [m]
    let gy = 30.0; // goal y position [m]
    let grid_size = 0.5; // potential grid size [m]
    let robot_radius = 5.0; // robot radius [m]

    let ox = vec![15.0, 5.0, 20.0, 25.0]; // obstacle x position list [m]
    let oy = vec![25.0, 15.0, 26.0, 25.0]; // obstacle y position list [m]

    let planner = PotentialFieldPlanner::new(grid_size, robot_radius);

    if let Some((rx, ry)) = planner.planning(sx, sy, gx, gy, &ox, &oy) {
        println!("Found path with {} points!", rx.len());
        planner.visualize_path(&rx, &ry, sx, sy, gx, gy, &ox, &oy);
    } else {
        println!("Cannot find path");
    }

    println!("Potential Field path planning finish!!");
}
