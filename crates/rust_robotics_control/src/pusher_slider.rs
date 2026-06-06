//! Quasi-static planar pushing (pusher-slider) with contact modes and MPPI.
//!
//! A pure-Rust reproduction slice for contact-rich planar manipulation in the
//! spirit of "Push Anything": a point pusher that may contact any of the
//! slider's four faces shoves a rigid square slider across a table to a goal
//! pose. The slider obeys the classic quasi-static ellipsoidal limit-surface
//! model (Goyal/Howe/Mason; Lynch; Hogan-Rodriguez): motion is determined by the
//! contact, not by inertia, and the contact can *stick* or *slide* depending on
//! whether the required tangential force stays inside the pusher friction cone.
//!
//! - [`PusherSliderParams`] holds the slider half-extent, the limit-surface
//!   characteristic length, and the pusher friction coefficient.
//! - [`PusherCommand`] is a body-frame pusher motion on a chosen face: the face
//!   index, a contact offset along it, a normal push speed, and a tangential
//!   slip speed.
//! - [`PusherSliderParams::step`] advances the slider one quasi-static step and
//!   reports the realized [`ContactMode`].
//! - [`PusherSliderMppiController`] runs MPPI per face and executes the command
//!   from the lowest-cost face, so it can switch faces to reach goals (such as a
//!   pure rotation) that a single face cannot; [`simulate_push`] runs the closed
//!   loop and returns a [`PushReport`].
//!
//! The model is exact for the sticking and sliding regimes of a single point
//! contact on each face; simultaneous multi-contact pushing is left as an
//! extension.

use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// Planar pose `[x, y, theta]` of the slider in the world frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SliderState {
    pub pose: [f64; 3],
}

impl SliderState {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            pose: [x, y, theta],
        }
    }

    pub fn x(self) -> f64 {
        self.pose[0]
    }
    pub fn y(self) -> f64 {
        self.pose[1]
    }
    pub fn theta(self) -> f64 {
        self.pose[2]
    }
}

/// The realized contact regime for a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactMode {
    /// No (or negative) normal force: the pusher is not pushing.
    Separated,
    /// Contact sticks: the contact point moves with the pusher.
    Stick,
    /// Contact slides along the face toward +tangent (friction cone saturated).
    SlideUp,
    /// Contact slides along the face toward -tangent.
    SlideDown,
}

/// Body-frame pusher command on one of the slider's four faces.
///
/// `face` selects the contact face (`0` = back `-x`, `1` = `+y`, `2` = front
/// `+x`, `3` = `-y`). The pusher pushes along the inward normal of that face;
/// `contact` is the offset along the face, `push_speed` is the normal speed
/// (clamped to be non-negative), and `tangent_speed` is the commanded tangential
/// slip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PusherCommand {
    pub face: usize,
    pub contact: f64,
    pub push_speed: f64,
    pub tangent_speed: f64,
}

impl PusherCommand {
    /// A command on the back face (`face = 0`).
    pub fn new(contact: f64, push_speed: f64, tangent_speed: f64) -> Self {
        Self {
            face: 0,
            contact,
            push_speed,
            tangent_speed,
        }
    }

    /// A command on an explicit face (wrapped into `0..4`).
    pub fn on_face(face: usize, contact: f64, push_speed: f64, tangent_speed: f64) -> Self {
        Self {
            face: face % 4,
            contact,
            push_speed,
            tangent_speed,
        }
    }
}

/// Quasi-static pusher-slider parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PusherSliderParams {
    /// Half side length of the square slider \[m\].
    pub half_extent: f64,
    /// Limit-surface characteristic length `c` \[m\] (couples force to torque).
    pub char_len: f64,
    /// Coulomb friction coefficient between pusher and slider.
    pub pusher_friction: f64,
}

impl Default for PusherSliderParams {
    fn default() -> Self {
        // c ~ 0.6 * half-extent is the usual uniform-pressure square estimate.
        Self {
            half_extent: 0.05,
            char_len: 0.03,
            pusher_friction: 0.3,
        }
    }
}

impl PusherSliderParams {
    pub fn new(half_extent: f64, char_len: f64, pusher_friction: f64) -> RoboticsResult<Self> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(half_extent) || !positive(char_len) {
            return Err(RoboticsError::InvalidParameter(
                "pusher-slider half_extent and char_len must be finite and positive".to_string(),
            ));
        }
        if !pusher_friction.is_finite() || pusher_friction < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "pusher-slider pusher_friction must be finite and non-negative".to_string(),
            ));
        }
        Ok(Self {
            half_extent,
            char_len,
            pusher_friction,
        })
    }

    /// Body-frame contact point `p`, inward normal `d`, and tangent `t` for a
    /// face index (`0..4`) and offset along the face.
    fn contact_frame(self, face: usize, contact: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
        let b = self.half_extent;
        let s = contact.clamp(-b, b);
        match face % 4 {
            0 => ([-b, s], [1.0, 0.0], [0.0, 1.0]),
            1 => ([s, b], [0.0, -1.0], [1.0, 0.0]),
            2 => ([b, s], [-1.0, 0.0], [0.0, -1.0]),
            _ => ([s, -b], [0.0, 1.0], [-1.0, 0.0]),
        }
    }

    /// Body-frame slider twist `[vx, vy, omega]` and contact mode for a command.
    ///
    /// The limit-surface solve is identical for every face; only the contact
    /// frame `(p, d, t)` rotates, so the friction cone is evaluated along the
    /// face's own normal/tangent rather than the body axes.
    fn twist(self, command: PusherCommand) -> ([f64; 3], ContactMode) {
        let c2 = self.char_len * self.char_len;
        let (p, d, t) = self.contact_frame(command.face, command.contact);
        let [px, py] = p;
        let vn = command.push_speed.max(0.0);
        let vt = command.tangent_speed;

        if vn <= 1e-12 {
            return ([0.0; 3], ContactMode::Separated);
        }

        // Pusher velocity in the body frame and the limit-surface solve for the
        // contact force whose motion matches it (same M for every face):
        //   [wx, wy]^T = (1/c^2) [[c^2+py^2, -px*py], [-px*py, c^2+px^2]] [fx, fy]^T.
        let wx = vn * d[0] + vt * t[0];
        let wy = vn * d[1] + vt * t[1];
        let m11 = (c2 + py * py) / c2;
        let m12 = -(px * py) / c2;
        let m22 = (c2 + px * px) / c2;
        let det = m11 * m22 - m12 * m12;
        let (fx, fy) = if det.abs() > 1e-15 {
            ((m22 * wx - m12 * wy) / det, (-m12 * wx + m11 * wy) / det)
        } else {
            (wx, wy)
        };

        // Resolve the force into the face's normal/tangent for the friction cone.
        let fn_ = fx * d[0] + fy * d[1];
        let ft = fx * t[0] + fy * t[1];
        if fn_ <= 0.0 {
            // Would require pulling on the object: no contact motion.
            return ([0.0; 3], ContactMode::Separated);
        }

        let mu = self.pusher_friction;
        if ft.abs() <= mu * fn_ + 1e-12 {
            // Stick: the slider twist is the limit-surface image of the wrench.
            let omega = (px * fy - py * fx) / c2;
            ([fx, fy, omega], ContactMode::Stick)
        } else {
            // Slide: the tangential force saturates on the friction-cone edge and
            // the contact slides along the face. Scale the cone-edge wrench so the
            // normal contact-point speed still matches the commanded push.
            let sign = if ft > 0.0 { 1.0 } else { -1.0 };
            let fe = [d[0] + sign * mu * t[0], d[1] + sign * mu * t[1]];
            let omega1 = (px * fe[1] - py * fe[0]) / c2;
            // Contact-point velocity per unit scale, projected on the normal d.
            let cv = [fe[0] - omega1 * py, fe[1] + omega1 * px];
            let proj = cv[0] * d[0] + cv[1] * d[1];
            let k = if proj.abs() > 1e-12 { vn / proj } else { vn };
            let k = k.max(0.0);
            let mode = if sign > 0.0 {
                ContactMode::SlideUp
            } else {
                ContactMode::SlideDown
            };
            ([k * fe[0], k * fe[1], k * omega1], mode)
        }
    }

    /// Advance the slider one quasi-static step, returning the new state and the
    /// realized contact mode.
    pub fn step(
        self,
        state: SliderState,
        command: PusherCommand,
        dt: f64,
    ) -> (SliderState, ContactMode) {
        let ([vx_b, vy_b, omega], mode) = self.twist(command);
        if mode == ContactMode::Separated {
            return (state, mode);
        }
        // Rotate the body-frame CoM velocity into the world and integrate.
        let theta = state.theta();
        let (s, co) = theta.sin_cos();
        let vx_w = co * vx_b - s * vy_b;
        let vy_w = s * vx_b + co * vy_b;
        let next = SliderState::new(
            state.x() + vx_w * dt,
            state.y() + vy_w * dt,
            theta + omega * dt,
        );
        (next, mode)
    }

    /// World-frame contact point for a command (useful for rendering).
    pub fn contact_point(self, state: SliderState, command: PusherCommand) -> [f64; 2] {
        let ([px, py], _, _) = self.contact_frame(command.face, command.contact);
        let (s, co) = state.theta().sin_cos();
        [state.x() + co * px - s * py, state.y() + s * px + co * py]
    }
}

/// Configuration for the sampling-based pushing controller.
#[derive(Debug, Clone, PartialEq)]
pub struct PusherMppiConfig {
    pub horizon: usize,
    pub samples: usize,
    pub dt: f64,
    /// Std-dev of the contact-offset perturbation \[m\].
    pub contact_sigma: f64,
    /// Std-dev of the tangential-slip perturbation \[m/s\].
    pub tangent_sigma: f64,
    /// Std-dev of the normal-push perturbation \[m/s\].
    pub push_sigma: f64,
    /// Initial (nominal) normal push speed \[m/s\].
    pub push_speed: f64,
    /// Maximum normal push speed \[m/s\] (the controller may slow toward 0).
    pub max_push_speed: f64,
    pub position_weight: f64,
    pub heading_weight: f64,
    pub lambda: f64,
    pub seed: u64,
}

impl Default for PusherMppiConfig {
    fn default() -> Self {
        Self {
            horizon: 18,
            samples: 400,
            dt: 0.1,
            contact_sigma: 0.03,
            tangent_sigma: 0.05,
            push_sigma: 0.04,
            push_speed: 0.06,
            max_push_speed: 0.12,
            // Pose weights are large because positions are O(0.1 m): squared
            // errors are tiny, so the softmax temperature needs cost values of
            // order 1-100 to discriminate rollouts (and let the pusher brake).
            position_weight: 250.0,
            heading_weight: 6.0,
            lambda: 1.0,
            seed: 7,
        }
    }
}

fn validate_config(config: &PusherMppiConfig) -> RoboticsResult<()> {
    if config.horizon == 0 || config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI horizon and samples must be positive".to_string(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI dt must be finite and positive".to_string(),
        ));
    }
    if !config.contact_sigma.is_finite()
        || config.contact_sigma <= 0.0
        || !config.tangent_sigma.is_finite()
        || config.tangent_sigma <= 0.0
        || !config.push_sigma.is_finite()
        || config.push_sigma <= 0.0
    {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI sigmas must be finite and positive".to_string(),
        ));
    }
    if !config.max_push_speed.is_finite() || config.max_push_speed <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI max_push_speed must be finite and positive".to_string(),
        ));
    }
    if !config.lambda.is_finite() || config.lambda <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI lambda must be finite and positive".to_string(),
        ));
    }
    if config.position_weight < 0.0 || config.heading_weight < 0.0 || config.push_speed < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "pusher MPPI weights and push_speed must be non-negative".to_string(),
        ));
    }
    Ok(())
}

/// Result of one `plan` call.
#[derive(Debug, Clone, PartialEq)]
pub struct PusherMppiPlan {
    pub command: PusherCommand,
    pub best_cost: f64,
}

/// The three per-control Gaussian perturbations used while sampling.
struct PusherNoise {
    contact: Normal<f64>,
    tangent: Normal<f64>,
    push: Normal<f64>,
}

/// Deterministic, seeded sampling controller for planar pushing.
///
/// The controller is face-aware: it keeps a warm-started nominal plan for each of
/// the slider's four faces, runs MPPI within each face, and executes the command
/// from whichever face yields the lowest-cost rollout. Switching faces is what
/// makes rotation-without-lateral-drift goals reachable (e.g. push the back face
/// to spin one way, the front face to spin back and cancel the translation).
#[derive(Debug, Clone)]
pub struct PusherSliderMppiController {
    config: PusherMppiConfig,
    params: PusherSliderParams,
    /// One warm-started nominal sequence per face (`0..4`).
    nominals: Vec<Vec<PusherCommand>>,
    rng: StdRng,
}

impl PusherSliderMppiController {
    pub fn new(config: PusherMppiConfig, params: PusherSliderParams) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nominals = (0..4)
            .map(|face| {
                vec![PusherCommand::on_face(face, 0.0, config.push_speed, 0.0); config.horizon]
            })
            .collect();
        let rng = StdRng::seed_from_u64(config.seed);
        Ok(Self {
            config,
            params,
            nominals,
            rng,
        })
    }

    pub fn config(&self) -> &PusherMppiConfig {
        &self.config
    }

    fn rollout_cost(
        &self,
        start: SliderState,
        goal: SliderState,
        controls: &[PusherCommand],
    ) -> f64 {
        let mut state = start;
        let mut cost = 0.0;
        for (i, &command) in controls.iter().enumerate() {
            let (next, _) = self.params.step(state, command, self.config.dt);
            state = next;
            // Weight later steps more heavily (terminal emphasis).
            let w = (i + 1) as f64 / controls.len() as f64;
            cost += w * self.pose_cost(state, goal);
        }
        cost
    }

    fn pose_cost(&self, state: SliderState, goal: SliderState) -> f64 {
        let dx = state.x() - goal.x();
        let dy = state.y() - goal.y();
        let dtheta = wrap_angle(state.theta() - goal.theta());
        self.config.position_weight * (dx * dx + dy * dy)
            + self.config.heading_weight * dtheta * dtheta
    }

    /// Run MPPI for a single face, returning its warm-start-shifted nominal and
    /// the best (minimum) rollout cost seen for that face.
    fn plan_face(
        &mut self,
        face: usize,
        start: SliderState,
        goal: SliderState,
        per_face_samples: usize,
        noise: &PusherNoise,
    ) -> (Vec<PusherCommand>, f64) {
        let contact_noise = &noise.contact;
        let tangent_noise = &noise.tangent;
        let push_noise = &noise.push;
        let b = self.params.half_extent;
        let max_push = self.config.max_push_speed;
        let horizon = self.config.horizon;
        let base_seq = self.nominals[face].clone();

        let mut costs = Vec::with_capacity(per_face_samples);
        let mut sequences = Vec::with_capacity(per_face_samples);
        let mut best_cost = f64::INFINITY;

        for _ in 0..per_face_samples {
            let mut controls = Vec::with_capacity(horizon);
            for &bcmd in &base_seq {
                let contact = (bcmd.contact + contact_noise.sample(&mut self.rng)).clamp(-b, b);
                let tangent = bcmd.tangent_speed + tangent_noise.sample(&mut self.rng);
                let push =
                    (bcmd.push_speed + push_noise.sample(&mut self.rng)).clamp(0.0, max_push);
                controls.push(PusherCommand::on_face(face, contact, push, tangent));
            }
            let cost = self.rollout_cost(start, goal, &controls);
            best_cost = best_cost.min(cost);
            costs.push(cost);
            sequences.push(controls);
        }

        let min_cost = costs.iter().copied().fold(f64::INFINITY, f64::min);
        let mut weight_sum = 0.0;
        let mut weights = Vec::with_capacity(costs.len());
        for &cost in &costs {
            let w = (-(cost - min_cost) / self.config.lambda).exp();
            weight_sum += w;
            weights.push(w);
        }

        let updated = if weight_sum <= 0.0 || !weight_sum.is_finite() {
            let best_index = costs
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);
            sequences[best_index].clone()
        } else {
            let mut acc_seq = vec![PusherCommand::on_face(face, 0.0, 0.0, 0.0); horizon];
            for (w, controls) in weights.iter().zip(&sequences) {
                let n = w / weight_sum;
                for (acc, command) in acc_seq.iter_mut().zip(controls) {
                    acc.contact += n * command.contact;
                    acc.push_speed += n * command.push_speed;
                    acc.tangent_speed += n * command.tangent_speed;
                }
            }
            for command in &mut acc_seq {
                command.contact = command.contact.clamp(-b, b);
                command.push_speed = command.push_speed.clamp(0.0, max_push);
            }
            acc_seq
        };

        (updated, best_cost)
    }

    /// Plan the next pusher command toward `goal`, choosing the best contact face.
    pub fn plan(
        &mut self,
        start: SliderState,
        goal: SliderState,
    ) -> RoboticsResult<PusherMppiPlan> {
        let make = |sigma: f64, what: &str| {
            Normal::new(0.0, sigma).map_err(|_| {
                RoboticsError::InvalidParameter(format!("invalid pusher {what} distribution"))
            })
        };
        let noise = PusherNoise {
            contact: make(self.config.contact_sigma, "contact")?,
            tangent: make(self.config.tangent_sigma, "tangent")?,
            push: make(self.config.push_sigma, "push")?,
        };
        let per_face = (self.config.samples / 4).max(1);

        let mut best: Option<(PusherCommand, f64)> = None;
        for face in 0..4 {
            let (updated, cost) = self.plan_face(face, start, goal, per_face, &noise);
            let first = updated[0];
            // Warm-start: shift this face's nominal by one step.
            let mut shifted = updated[1..].to_vec();
            shifted.push(*updated.last().unwrap());
            self.nominals[face] = shifted;

            if best.map(|(_, c)| cost < c).unwrap_or(true) {
                best = Some((first, cost));
            }
        }

        let (command, best_cost) = best.unwrap();
        Ok(PusherMppiPlan { command, best_cost })
    }
}

/// Metrics for a closed-loop push.
#[derive(Debug, Clone, PartialEq)]
pub struct PushReport {
    pub steps: usize,
    pub final_pose: [f64; 3],
    pub position_error: f64,
    pub heading_error: f64,
    pub stick_fraction: f64,
    pub slide_fraction: f64,
    pub path: Vec<[f64; 3]>,
    pub modes: Vec<ContactMode>,
}

/// Drive the pushing controller toward a goal pose and report the result.
pub fn simulate_push(
    config: PusherMppiConfig,
    params: PusherSliderParams,
    start: SliderState,
    goal: SliderState,
    max_steps: usize,
) -> RoboticsResult<PushReport> {
    let dt = config.dt;
    let mut controller = PusherSliderMppiController::new(config, params)?;
    let mut state = start;
    let mut path = vec![state.pose];
    let mut modes = Vec::new();
    let mut stick = 0usize;
    let mut slide = 0usize;
    let mut executed = 0usize;

    for _ in 0..max_steps {
        let plan = controller.plan(state, goal)?;
        let (next, mode) = params.step(state, plan.command, dt);
        match mode {
            ContactMode::Stick => stick += 1,
            ContactMode::SlideUp | ContactMode::SlideDown => slide += 1,
            ContactMode::Separated => {}
        }
        state = next;
        path.push(state.pose);
        modes.push(mode);
        executed += 1;

        let dx = state.x() - goal.x();
        let dy = state.y() - goal.y();
        if (dx * dx + dy * dy).sqrt() < 0.2 * params.half_extent
            && wrap_angle(state.theta() - goal.theta()).abs() < 0.05
        {
            break;
        }
    }

    let dx = state.x() - goal.x();
    let dy = state.y() - goal.y();
    let denom = executed.max(1) as f64;
    Ok(PushReport {
        steps: executed,
        final_pose: state.pose,
        position_error: (dx * dx + dy * dy).sqrt(),
        heading_error: wrap_angle(state.theta() - goal.theta()).abs(),
        stick_fraction: stick as f64 / denom,
        slide_fraction: slide as f64 / denom,
        path,
        modes,
    })
}

fn wrap_angle(a: f64) -> f64 {
    let mut x = a;
    while x > std::f64::consts::PI {
        x -= std::f64::consts::TAU;
    }
    while x < -std::f64::consts::PI {
        x += std::f64::consts::TAU;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_centered_push_translates_without_rotation() {
        let params = PusherSliderParams::default();
        let mut state = SliderState::new(0.0, 0.0, 0.0);
        for _ in 0..20 {
            let (next, mode) = params.step(state, PusherCommand::new(0.0, 0.05, 0.0), 0.05);
            assert_eq!(mode, ContactMode::Stick);
            state = next;
        }
        assert!(state.x() > 0.0, "should move +x: {}", state.x());
        assert!(state.y().abs() < 1e-9, "no lateral drift: {}", state.y());
        assert!(state.theta().abs() < 1e-9, "no rotation: {}", state.theta());
    }

    #[test]
    fn offcenter_push_induces_rotation() {
        let params = PusherSliderParams::default();
        let mut state = SliderState::new(0.0, 0.0, 0.0);
        let cmd = PusherCommand::new(0.7 * params.half_extent, 0.05, 0.0);
        for _ in 0..20 {
            let (next, _) = params.step(state, cmd, 0.05);
            state = next;
        }
        assert!(
            state.theta().abs() > 1e-3,
            "should rotate: {}",
            state.theta()
        );
    }

    #[test]
    fn large_tangent_slip_slides() {
        let params = PusherSliderParams::default();
        let state = SliderState::new(0.0, 0.0, 0.0);
        let (_, mode) = params.step(state, PusherCommand::new(0.0, 0.05, 1.0), 0.05);
        assert!(matches!(
            mode,
            ContactMode::SlideUp | ContactMode::SlideDown
        ));
    }

    #[test]
    fn no_push_keeps_state() {
        let params = PusherSliderParams::default();
        let state = SliderState::new(0.3, -0.2, 0.5);
        let (next, mode) = params.step(state, PusherCommand::new(0.0, 0.0, 0.2), 0.05);
        assert_eq!(mode, ContactMode::Separated);
        assert_eq!(next, state);
    }

    #[test]
    fn rejects_invalid_params() {
        assert!(PusherSliderParams::new(0.0, 0.03, 0.3).is_err());
        assert!(PusherSliderParams::new(0.05, -0.01, 0.3).is_err());
        assert!(PusherSliderParams::new(0.05, 0.03, -0.1).is_err());
    }

    #[test]
    fn controller_pushes_toward_goal() {
        let params = PusherSliderParams::default();
        let start = SliderState::new(0.0, 0.0, 0.0);
        // On-axis reach: a single back-face pusher drives the slider forward.
        let goal = SliderState::new(0.3, 0.0, 0.0);
        let report = simulate_push(PusherMppiConfig::default(), params, start, goal, 150).unwrap();
        let start_err = ((start.x() - goal.x()).powi(2) + (start.y() - goal.y()).powi(2)).sqrt();
        assert!(
            report.position_error < 0.3 * start_err,
            "should approach goal: {} vs start {}",
            report.position_error,
            start_err
        );
    }

    #[test]
    fn simulation_is_deterministic() {
        let params = PusherSliderParams::default();
        let start = SliderState::new(0.0, 0.0, 0.0);
        let goal = SliderState::new(0.3, 0.0, 0.0);
        let run = || simulate_push(PusherMppiConfig::default(), params, start, goal, 80).unwrap();
        let a = run();
        let b = run();
        assert_eq!(a.path, b.path);
        assert_eq!(a.position_error, b.position_error);
    }

    #[test]
    fn pushing_on_a_chosen_face_reverses_translation() {
        // The front face (face 2) pushes the slider in -x, the back face in +x.
        let params = PusherSliderParams::default();
        let state = SliderState::new(0.0, 0.0, 0.0);
        let (back, _) = params.step(state, PusherCommand::on_face(0, 0.0, 0.05, 0.0), 0.1);
        let (front, _) = params.step(state, PusherCommand::on_face(2, 0.0, 0.05, 0.0), 0.1);
        assert!(back.x() > 0.0, "back face pushes +x: {}", back.x());
        assert!(front.x() < 0.0, "front face pushes -x: {}", front.x());
    }

    #[test]
    fn multiface_rotates_without_net_translation() {
        // A pure rotation goal (theta only) is unreachable by a single back-face
        // pusher but reachable once the controller may switch faces.
        let params = PusherSliderParams::default();
        let start = SliderState::new(0.0, 0.0, 0.0);
        let goal = SliderState::new(0.0, 0.0, 0.5);
        let report = simulate_push(PusherMppiConfig::default(), params, start, goal, 300).unwrap();
        assert!(
            report.heading_error < 0.1,
            "should reach the heading: err {}",
            report.heading_error
        );
        assert!(
            report.position_error < 2.0 * params.half_extent,
            "should keep position near origin: err {}",
            report.position_error
        );
    }
}
