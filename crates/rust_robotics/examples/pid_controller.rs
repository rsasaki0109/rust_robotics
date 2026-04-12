//! PID step response visualization example.
//!
//! Generates four subplots (P-only, PI, PD, PID) and saves:
//! `img/control/pid_step_response.png`.

use gnuplot::{AxesCommon, Caption, Color, Figure};
use rust_robotics::control::pid_controller::{PIDConfig, PIDController};
use rust_robotics::core::Controller;

const DT: f64 = 0.02;
const SIM_TIME: f64 = 10.0;
const SETPOINT: f64 = 1.0;
const PLANT_TAU: f64 = 1.0;

fn simulate_step_response(mut controller: PIDController) -> (Vec<f64>, Vec<f64>) {
    let steps = (SIM_TIME / DT) as usize;
    let mut t = Vec::with_capacity(steps + 1);
    let mut y = Vec::with_capacity(steps + 1);

    let mut state = 0.0;
    for k in 0..=steps {
        let time = k as f64 * DT;
        let control = controller.compute(&state, &SETPOINT);

        // First-order plant: y_dot = (-y + u) / tau
        let dy = (-state + control) / PLANT_TAU;
        state += DT * dy;

        t.push(time);
        y.push(state);
    }

    (t, y)
}

fn main() {
    std::fs::create_dir_all("img/control").expect("failed to create img/control directory");

    let p_only = PIDController::new(PIDConfig {
        kp: 2.0,
        ki: 0.0,
        kd: 0.0,
        dt: DT,
        max_integral: 10.0,
        max_output: 10.0,
    });
    let pi = PIDController::new(PIDConfig {
        kp: 1.2,
        ki: 0.8,
        kd: 0.0,
        dt: DT,
        max_integral: 10.0,
        max_output: 10.0,
    });
    let pd = PIDController::new(PIDConfig {
        kp: 1.6,
        ki: 0.0,
        kd: 0.25,
        dt: DT,
        max_integral: 10.0,
        max_output: 10.0,
    });
    let pid = PIDController::new(PIDConfig {
        kp: 1.2,
        ki: 0.8,
        kd: 0.2,
        dt: DT,
        max_integral: 10.0,
        max_output: 10.0,
    });

    let (t_p, y_p) = simulate_step_response(p_only);
    let (t_pi, y_pi) = simulate_step_response(pi);
    let (t_pd, y_pd) = simulate_step_response(pd);
    let (t_pid, y_pid) = simulate_step_response(pid);
    let target = vec![SETPOINT; t_p.len()];

    let mut fg = Figure::new();
    fg.set_multiplot_layout(2, 2);

    {
        fg.axes2d()
            .set_title("P-only", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Output", &[])
            .lines(&t_p, &y_p, &[Caption("P"), Color("blue")])
            .lines(&t_p, &target, &[Caption("Reference"), Color("black")]);

        fg.axes2d()
            .set_title("PI", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Output", &[])
            .lines(&t_pi, &y_pi, &[Caption("PI"), Color("red")])
            .lines(&t_pi, &target, &[Caption("Reference"), Color("black")]);

        fg.axes2d()
            .set_title("PD", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Output", &[])
            .lines(&t_pd, &y_pd, &[Caption("PD"), Color("green")])
            .lines(&t_pd, &target, &[Caption("Reference"), Color("black")]);

        fg.axes2d()
            .set_title("PID", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Output", &[])
            .lines(&t_pid, &y_pid, &[Caption("PID"), Color("purple")])
            .lines(&t_pid, &target, &[Caption("Reference"), Color("black")]);
    }

    let output_path = "img/control/pid_step_response.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show()
        .expect("failed to render gnuplot figure for PID step response");
    println!("Saved {}", output_path);
}
