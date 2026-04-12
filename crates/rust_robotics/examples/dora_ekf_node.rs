//! EKF localization node for dora-rs integration.
//!
//! When the `dora` feature is enabled, this example runs as a dora node:
//! - input `control`: JSON `{"v": <f64>, "omega": <f64>, "dt": <f64>}`
//! - input `measurement`: JSON `{"x": <f64>, "y": <f64>}`
//! - output `state_estimate`: JSON state estimate from EKF
//!
//! Without the `dora` feature, it runs in a stdin/stdout fallback mode:
//! - `control <v> <omega> <dt>`
//! - `measurement <x> <y>`
//! - emits `state_estimate <x> <y> <yaw> <v>`

#[cfg(feature = "dora")]
mod dora_runtime {
    use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
    use rust_robotics::core::StateEstimator;
    use rust_robotics::localization::ekf::{EKFControl, EKFLocalizer, EKFMeasurement};
    use serde::{Deserialize, Serialize};
    use std::error::Error;

    type NodeResult<T> = Result<T, Box<dyn Error>>;

    #[derive(Debug, Deserialize)]
    struct ControlPayload {
        v: f64,
        omega: f64,
        dt: f64,
    }

    #[derive(Debug, Deserialize)]
    struct MeasurementPayload {
        x: f64,
        y: f64,
    }

    #[derive(Debug, Serialize)]
    struct StateEstimatePayload {
        x: f64,
        y: f64,
        yaw: f64,
        v: f64,
        covariance_trace: f64,
    }

    pub fn run() -> NodeResult<()> {
        let output_id = DataId::from("state_estimate".to_owned());
        let (mut node, mut events) = DoraNode::init_from_env()?;
        let mut ekf = EKFLocalizer::with_defaults();
        let mut control = EKFControl::new(0.0, 0.0);
        let mut dt = 0.1;
        let mut estimates_sent = 0usize;

        while let Some(event) = events.recv() {
            match event {
                Event::Input { id, data, metadata } if id.as_str() == "control" => {
                    let payload: &str = TryFrom::try_from(&data)?;
                    if payload.is_empty() {
                        return Err("received empty control payload".into());
                    }
                    let parsed: ControlPayload = serde_json::from_str(payload)?;
                    if !parsed.dt.is_finite() || parsed.dt <= 0.0 {
                        return Err(format!("invalid dt in control payload: {}", parsed.dt).into());
                    }
                    control = EKFControl::new(parsed.v, parsed.omega);
                    dt = parsed.dt;
                }
                Event::Input { id, data, metadata } if id.as_str() == "measurement" => {
                    let payload: &str = TryFrom::try_from(&data)?;
                    if payload.is_empty() {
                        return Err("received empty measurement payload".into());
                    }
                    let measurement: MeasurementPayload = serde_json::from_str(payload)?;
                    let z = EKFMeasurement::new(measurement.x, measurement.y);
                    ekf.predict(&control, dt);
                    ekf.update(&z);

                    let state = ekf.state_2d();
                    let covariance = ekf.get_covariance_matrix();
                    let covariance_trace = covariance[(0, 0)]
                        + covariance[(1, 1)]
                        + covariance[(2, 2)]
                        + covariance[(3, 3)];

                    let payload = StateEstimatePayload {
                        x: state.x,
                        y: state.y,
                        yaw: state.yaw,
                        v: state.v,
                        covariance_trace,
                    };
                    let encoded = serde_json::to_string(&payload)?;
                    node.send_output(output_id.clone(), metadata.parameters, encoded.into_arrow())?;
                    estimates_sent += 1;
                }
                Event::Input { id, .. } => eprintln!("ignoring unexpected input `{id}`"),
                Event::InputClosed { id } => {
                    if id.as_str() == "measurement" && estimates_sent == 0 {
                        return Err(
                            "measurement input closed before any state estimate was published"
                                .into(),
                        );
                    }
                    if id.as_str() == "measurement" || id.as_str() == "control" {
                        break;
                    }
                }
                Event::Stop(_) => break,
                Event::Reload { .. } | Event::Error(_) => {}
                _ => {}
            }
        }

        if estimates_sent == 0 {
            return Err("node exited before publishing a state estimate".into());
        }

        Ok(())
    }
}

#[cfg(not(feature = "dora"))]
mod stdio_fallback {
    use rust_robotics::core::StateEstimator;
    use rust_robotics::localization::ekf::{EKFControl, EKFLocalizer, EKFMeasurement};
    use std::error::Error;
    use std::io::{self, BufRead};

    type NodeResult<T> = Result<T, Box<dyn Error>>;

    pub fn run() -> NodeResult<()> {
        let stdin = io::stdin();
        let mut ekf = EKFLocalizer::with_defaults();
        let mut control = EKFControl::new(0.0, 0.0);
        let mut dt = 0.1;

        for line in stdin.lock().lines() {
            let line = line?;
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }

            let mut parts = trimmed.split_whitespace();
            let command = parts.next().ok_or("missing command")?;
            match command {
                "control" => {
                    let v: f64 = parts.next().ok_or("missing control v")?.parse()?;
                    let omega: f64 = parts.next().ok_or("missing control omega")?.parse()?;
                    let next_dt: f64 = parts.next().ok_or("missing control dt")?.parse()?;
                    if !next_dt.is_finite() || next_dt <= 0.0 {
                        return Err(format!("invalid dt: {}", next_dt).into());
                    }
                    control = EKFControl::new(v, omega);
                    dt = next_dt;
                }
                "measurement" => {
                    let x: f64 = parts.next().ok_or("missing measurement x")?.parse()?;
                    let y: f64 = parts.next().ok_or("missing measurement y")?.parse()?;
                    let measurement = EKFMeasurement::new(x, y);
                    ekf.predict(&control, dt);
                    ekf.update(&measurement);
                    let state = ekf.state_2d();
                    println!(
                        "state_estimate {:.6} {:.6} {:.6} {:.6}",
                        state.x, state.y, state.yaw, state.v
                    );
                }
                _ => return Err(format!("unknown command: {}", command).into()),
            }
        }

        Ok(())
    }
}

#[cfg(feature = "dora")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    dora_runtime::run()
}

#[cfg(not(feature = "dora"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    stdio_fallback::run()
}
