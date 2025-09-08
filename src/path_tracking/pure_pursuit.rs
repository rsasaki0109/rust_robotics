// Path tracking simulation with pure pursuit steering and PID speed control.
// author: Atsushi Sakai (@Atsushi_twi)
//         Guillaume Jacquenot (@Gjacquenot)
//         Ryohei Sasaki (@rsasaki0109)
use gnuplot::{Figure, Caption, Color, AxesCommon};

#[derive(Debug, Copy, Clone)] // Turn Move Semantics into Copy Semantics
struct State {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
    rear_x: f64,
    rear_y: f64,
    wb: f64,
}

impl State {
    fn new(x:(f64,f64,f64,f64), wb: f64)-> State{
        State {
          x: x.0, y: x.1,
          yaw: x.2, v: x.3,
          rear_x: x.0 - ((wb / 2.) * (x.2).cos()),
          rear_y: x.1 - ((wb / 2.) * (x.2).sin()),
          wb: wb
        }
      }
    fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * (self.yaw).cos() * dt;
        self.y += self.v * (self.yaw).sin() * dt;
        self.yaw += self.v / self.wb * (delta).tan() * dt;
        self.v += a * dt;
        self.rear_x = self.x - ((self.wb / 2.) * (self.yaw).cos());
        self.rear_y = self.y - ((self.wb / 2.) * (self.yaw).sin());
      }
    fn calc_distance(&self, point_x: f64, point_y: f64) -> f64{
        let dx = self.rear_x - point_x;
        let dy = self.rear_y - point_y;
        (dx.powi(2) + dy.powi(2) ).sqrt()
      }   
}

fn proportional_control(target: f64, current: f64, kp: f64) -> f64
{
    kp * (target - current)
}


struct TargetCourse {
  cx: Vec<f64>,
  cy: Vec<f64>,
  old_nearest_point_index: i32,
  k: f64,
  lfc: f64
}

impl TargetCourse {
  fn new(c:(Vec<f64>,Vec<f64>),k: f64, lfc: f64)-> TargetCourse{
    TargetCourse {
      cx: c.0, cy: c.1,
      old_nearest_point_index: -1,
      k: k,
      lfc: lfc
    }
  }
  fn search_target_index(&mut self, state: State) -> (i32, f64){
    let mut ind = 0;
    if self.old_nearest_point_index == -1 {
      let mut d_min = std::f64::MAX;
      let mut ind_min = 0;
      for i in 0..self.cx.len()-1 { // The type of i is usize
        let dx = state.rear_x - self.cx[i];
        let dy = state.rear_y - self.cy[i];
        let d = (dx.powi(2) + dy.powi(2)).sqrt();
        if d < d_min {
          d_min = d;
          ind_min = i;
        }
      } 
      self.old_nearest_point_index = ind_min as i32;
      ind = ind_min;
    } else {
      ind = self.old_nearest_point_index as usize;
      let mut distance_this_index = state.calc_distance(self.cx[ind],
        self.cy[ind]);
      loop {
        if ind + 1 >= self.cx.len() {
          break;
        }
        let distance_next_index = state.calc_distance(self.cx[ind + 1],
          self.cy[ind + 1]);
        if distance_this_index < distance_next_index {
          break
        }
        ind += 1;
        distance_this_index = distance_next_index;
      }
      self.old_nearest_point_index = ind as i32;
    }
    let lf = self.k * state.v + self.lfc;
    while ind < self.cx.len() && lf > state.calc_distance(self.cx[ind], self.cy[ind]) {
      if ind + 1 >= self.cx.len() {
        break
      }
      ind += 1;
    }
    (ind as i32, lf)
  }
}

fn pure_pursuit_steer_control(state: State, trajectory: &mut TargetCourse, pind: i32)
-> (f64, i32)
{
    let pair = trajectory.search_target_index(state);
    let mut ind = pair.0;
    let lf = pair.1;
    if pind > ind {
      ind = pind;
    }
    
    let tx: f64;
    let ty: f64;
    let traj_len = trajectory.cx.len();
    if ind < traj_len as i32 {
      tx = trajectory.cx[ind as usize];
      ty = trajectory.cy[ind as usize];
    } else {
      tx = trajectory.cx[traj_len - 1];
      ty = trajectory.cy[traj_len - 1];
      ind = traj_len as i32 - 1
    }

    let alpha = (ty - state.rear_y).atan2(tx - state.rear_x) - state.yaw;
    let delta = (2.0 * state.wb * alpha.sin() / lf).atan2(1.0);
    (delta, ind) 
}

pub struct PurePursuitController {
    pub reference_path: Vec<(f64, f64)>,
    pub trajectory: Vec<(f64, f64)>,
    pub k: f64,
    pub lfc: f64,
    pub wb: f64,
}

impl PurePursuitController {
    pub fn new(k: f64, lfc: f64, wb: f64) -> Self {
        PurePursuitController {
            reference_path: Vec::new(),
            trajectory: Vec::new(),
            k,
            lfc,
            wb,
        }
    }

    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64) -> bool {
        if waypoints.len() < 2 {
            println!("Need at least 2 waypoints");
            return false;
        }

        // Generate reference path
        self.reference_path = waypoints;
        
        // Simulate pure pursuit tracking
        let kp = 1.0;  // speed proportional gain
        let dt = 0.1;  // [s] time tick
        let t_max = 100.0;  // max simulation time

        // initial state
        let init_x = (self.reference_path[0].0, self.reference_path[0].1 - 3.0, 0.0, 0.0);
        let mut state = State::new(init_x, self.wb);
        self.trajectory = vec![(state.x, state.y)];
        let mut time = 0.0;
        
        let cx: Vec<f64> = self.reference_path.iter().map(|p| p.0).collect();
        let cy: Vec<f64> = self.reference_path.iter().map(|p| p.1).collect();
        let mut target_course = TargetCourse::new((cx, cy), self.k, self.lfc);
        let pair = target_course.search_target_index(state);
        let mut target_ind = pair.0;

        while t_max > time {
            let ai = proportional_control(target_speed, state.v, kp);
            let tmp = pure_pursuit_steer_control(state, &mut target_course, target_ind);
            let di = tmp.0;
            target_ind = tmp.1;
            state.update(ai, di, dt);
            time += dt;

            self.trajectory.push((state.x, state.y));
            
            // Check if reached goal
            let goal = self.reference_path.last().unwrap();
            let dx = state.x - goal.0;
            let dy = state.y - goal.1;
            let distance_to_goal = (dx * dx + dy * dy).sqrt();
            if distance_to_goal < 2.0 {
                println!("Goal reached!");
                break;
            }
        }
        
        true
    }

    pub fn visualize(&self, filename: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Pure Pursuit Path Tracking", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

            // Plot reference path
            let ref_x: Vec<f64> = self.reference_path.iter().map(|p| p.0).collect();
            let ref_y: Vec<f64> = self.reference_path.iter().map(|p| p.1).collect();
            axes.lines(&ref_x, &ref_y, &[Caption("Reference Path"), Color("blue")]);

            // Plot trajectory
            let traj_x: Vec<f64> = self.trajectory.iter().map(|p| p.0).collect();
            let traj_y: Vec<f64> = self.trajectory.iter().map(|p| p.1).collect();
            axes.lines(&traj_x, &traj_y, &[Caption("Pure Pursuit Trajectory"), Color("red")]);

            // Plot start and goal
            if !self.reference_path.is_empty() {
                let start = &self.reference_path[0];
                axes.points(&[start.0], &[start.1], &[Caption("Start"), Color("green")]);
                let goal = self.reference_path.last().unwrap();
                axes.points(&[goal.0], &[goal.1], &[Caption("Goal"), Color("red")]);
            }
        }

        let output_path = format!("img/path_tracking/{}", filename);
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("Pure pursuit path tracking visualization saved to: {}", output_path);
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/path_tracking").unwrap();

    let mut controller = PurePursuitController::new(0.1, 2.0, 2.9);
    
    // Generate sinusoidal reference path
    let mut waypoints = Vec::new();
    let dx = 0.5;
    let length = (50.0 / dx) as usize;
    for i in 0..length {
        let x = dx * i as f64;
        let y = (i as f64 / 2.0) * (i as f64 / 5.0).sin();
        waypoints.push((x, y));
    }
    
    let target_speed = 10.0 / 3.6; // 10 km/h to m/s
    
    println!("Starting Pure Pursuit path tracking...");
    
    if controller.planning(waypoints, target_speed) {
        controller.visualize("pure_pursuit.png");
        println!("Pure Pursuit Path Tracking complete!");
    } else {
        println!("Planning failed!");
    }
}