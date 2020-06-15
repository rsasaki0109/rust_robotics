// Path tracking simulation with pure pursuit steering and PID speed control.
// author: Atsushi Sakai (@Atsushi_twi)
//         Guillaume Jacquenot (@Gjacquenot)
//         Ryohei Sasaki (@rsasaki0109)
use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::LineStyle;

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
  fn search_target_index(&mut self, _state: State) -> (i32, f64){
    let mut ind = 0;
    if self.old_nearest_point_index == -1 {
      let mut d_min = std::f64::MAX;
      let mut ind_min = 0;
      for i in 0..self.cx.len()-1 { // The type of i is usize
        let d = (self.cx[i].powi(2) + self.cy[i].powi(2) ).sqrt();
        if d < d_min {
          d_min = d;
          ind_min = i;
        }
      } 
      self.old_nearest_point_index = ind_min as i32;
      ind = ind_min;
    } else {
      ind = self.old_nearest_point_index as usize;
      let mut distance_this_index = _state.calc_distance(self.cx[ind],
        self.cy[ind]);
      loop {
        let distance_next_index = _state.calc_distance(self.cx[ind + 1],
          self.cy[ind + 1]);
        if distance_this_index < distance_next_index {
          break
        }
        if (ind + 1) < self.cx.len() {
          ind = ind + 1
        } else {
          ind = ind
        }
        distance_this_index = distance_next_index;
      }
      self.old_nearest_point_index = ind as i32;
    }
    let lf = self.k * _state.v + self.lfc;
    while lf > _state.calc_distance(self.cx[ind], self.cy[ind]) {
      if ind + 1 > self.cx.len() {
        break
      }
      ind += 1;
    }
    (ind as i32, lf)
  }    
}

fn pure_pursuit_steer_control(_state: State, _trajectory: &mut TargetCourse, pind: i32)
-> (f64, i32)
{
    let pair = _trajectory.search_target_index(_state);
    let mut ind = pair.0;
    let lf = pair.1;
    if pind > ind {
      ind = pind;
    }
    
    let tx: f64;
    let ty: f64;
    let traj_len = _trajectory.cx.len();
    if ind < traj_len as i32 {
      tx = _trajectory.cx[ind as usize];
      ty = _trajectory.cy[ind as usize];
    } else {
      tx = _trajectory.cx[traj_len - 1];
      ty = _trajectory.cy[traj_len - 1];
      ind = traj_len as i32 - 1
    }

    let alpha = (ty - _state.rear_y).atan2(tx - _state.rear_x) - _state.yaw;
    let delta = (2.0 * _state.wb * alpha.sin() / lf).atan2(1.0);
    (delta, ind) 
}

fn main() {
    // Parameters
    let _k = 0.1;  // look forward gain
    let _lfc = 2.0; // [m] look-ahead distance
    let _kp = 1.0;  // speed proportional gain
    let _dt = 0.1;  // [s] time tick
    let wb = 2.9;  // [m] wheel base of vehicle

    let _target_speed = 10.0 / 3.6; // [m/s]
    let _t_max = 100.0;  // max simulation time

    // target course
    let mut cx = Vec::<f64>::new();
    let mut cy = Vec::<f64>::new();
    let mut c = vec![(0., 0.)];
    let dx = 0.5;
    let _length = (50. /dx) as usize;
    for i in 0.._length - 1 {
      cx.push(0.5 * i as f64);
      cy.push((i as f64)/2. * (i as f64/5.).sin());
      c.push((cx[i],cy[i]));
    }

    // initial state
    let init_x = (0., -3., 0., 0.); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, wb);
    let mut states = vec![(state.x, state.y)];
    let mut _time = 0.;
    let mut target_course = TargetCourse::new((cx, cy), _k, _lfc);
    let pair = target_course.search_target_index(state);
    let target_ind = pair.0;

    while _t_max > _time {
      let ai = proportional_control(_target_speed, state.v, _kp);
      let tmp = pure_pursuit_steer_control(state, &mut target_course, target_ind);
      // &mut is a variable reference
      let di = tmp.0;
      state.update(ai, di, _dt);
      _time += _dt;

      states.push((state.x, state.y));
    }

    let s0: Plot = Plot::new(c).line_style(
      LineStyle::new() 
          .colour("#000000"),
    );

    let s1: Plot = Plot::new(states).line_style(
      LineStyle::new() 
          .colour("#35C788"),
    );

    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .x_range( 0., 50.)
        .y_range(-50., 50.)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/pure_pursuit.svg").unwrap();
}