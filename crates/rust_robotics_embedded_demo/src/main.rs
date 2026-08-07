//! RustRobotics embedded reference demo (bare-metal, no_std).
//!
//! Runs a deterministic closed-loop demo on a Cortex-M4F (STM32F405, emulated
//! by QEMU `netduino2`): a Pure Pursuit path tracker and a PID speed controller
//! steer a simulated vehicle along a straight path, while an EKF localizer
//! fuses noisy position measurements into a state estimate. This proves the
//! "no_std Kalman on a $5 MCU" pitch: the same `rust_robotics_localization`
//! and `rust_robotics_control` crates used by the host examples build and run
//! on a microcontroller with zero `std` (all f64 math via libm).
//!
//! Results are printed over semihosting; a final PASS/FAIL line plus the exit
//! code is what CI greps for.

#![no_std]
#![no_main]

use core::panic::PanicInfo;

use cortex_m_rt::entry;
use cortex_m_semihosting::debug::{exit, EXIT_FAILURE, EXIT_SUCCESS};
use cortex_m_semihosting::hprintln;
use embedded_alloc::Heap;
use num_traits::Float;

use rust_robotics_control::pure_pursuit::VehicleState;
use rust_robotics_control::{PIDController, PurePursuitConfig, PurePursuitController};
use rust_robotics_core::{Path2D, Point2D, StateEstimator};
use rust_robotics_localization::{EKFControl, EKFLocalizer, EKFMeasurement};

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: [u8; 64 * 1024] = [0u8; 64 * 1024];

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    exit(EXIT_FAILURE);
    #[allow(clippy::empty_loop)]
    loop {}
}

/// Minimal deterministic LCG so the demo needs no entropy source on the MCU.
struct Lcg(u64);

impl Lcg {
    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }

    /// Uniform float in `[0, 1)`.
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / 9007199254740992.0)
    }

    /// Standard-normal-ish noise via Box-Muller on two uniforms.
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-9);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (core::f64::consts::TAU * u2).cos()
    }
}

#[entry]
fn main() -> ! {
    // SAFETY: bare-metal, single-threaded; called once before any allocation.
    unsafe {
        HEAP.init(
            core::ptr::addr_of!(HEAP_MEM) as usize,
            core::mem::size_of::<[u8; 64 * 1024]>(),
        )
    }

    let dt = 0.1_f64;
    let target_speed = 1.0_f64;
    let noise_std = 0.08_f64;

    // Straight path along +x, sampled every 0.5 m.
    let path = Path2D::from_points(
        (0..=16)
            .map(|i| Point2D::new(i as f64 * 0.5, 0.0))
            .collect(),
    );

    let mut pursuer = PurePursuitController::new(PurePursuitConfig {
        look_ahead_gain: 0.5,
        look_ahead_distance: 1.0,
        wheelbase: 0.5,
        kp: 1.0,
        goal_threshold: 0.2,
    });
    pursuer.set_path(path);

    let mut speed_pid = PIDController::with_gains(2.0, 0.2, 0.0, dt);

    let mut veh = VehicleState::new(0.0, 0.0, 0.0, 0.0, 0.5);

    let mut ekf = EKFLocalizer::with_defaults();

    let mut rng = Lcg(0x5EED_1234_DEAD_BEEF);

    let mut final_error = 0.0_f64;

    hprintln!("rust_robotics embedded demo: EKF + PID/Pure Pursuit on Cortex-M");
    for step in 0..400 {
        let steer = pursuer.compute_steering(&veh);
        let accel = speed_pid.step(target_speed - veh.v);

        // True (noisy-free) plant.
        veh.update(accel, steer, dt);

        // Noisy position measurement.
        let mx = veh.x + noise_std * rng.next_gaussian();
        let my = veh.y + noise_std * rng.next_gaussian();

        // EKF: predict with the same control the plant received.
        let yaw_rate = veh.v / veh.wheelbase * steer.tan();
        let control = EKFControl::new(veh.v, yaw_rate);
        ekf.predict(&control, dt);
        let measurement = EKFMeasurement::new(mx, my);
        ekf.update(&measurement);

        let state = ekf.get_state();
        let ex = state[0] - veh.x;
        let ey = state[1] - veh.y;
        final_error = (ex * ex + ey * ey).sqrt();

        if step % 100 == 0 {
            hprintln!(
                "step={:3} true=({:+.2},{:+.2}) est=({:+.2},{:+.2}) err={:.3}",
                step,
                veh.x,
                veh.y,
                state[0],
                state[1],
                final_error
            );
        }
    }

    hprintln!("final EKF position error = {:.4} m", final_error);
    let ok = final_error < 0.5;
    hprintln!("embedded demo {}", if ok { "PASS" } else { "FAIL" });
    exit(if ok { EXIT_SUCCESS } else { EXIT_FAILURE });
    #[allow(clippy::empty_loop)]
    loop {}
}
