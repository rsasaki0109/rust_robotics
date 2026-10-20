//! Deterministic, equally budgeted joint-space and Cartesian tracking runs.

mod view;

use super::{wrap, ElbowBranch, TwoJointArm};
use std::collections::BTreeMap;
use std::f64::consts::PI;

const DT: f64 = 0.01;
const LAST: usize = 600;
const GAIN: f64 = 4.0;
const SPEED_LIMIT: f64 = 2.0;

#[derive(Clone, Debug, PartialEq)]
struct Config {
    start: [f64; 2],
    target: [f64; 2],
    branch: ElbowBranch,
    damping: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            start: [-0.35, 0.4],
            target: [1.2, 1.1],
            branch: ElbowBranch::Positive,
            damping: 0.2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Sample {
    angles: [f64; 2],
    tip: [f64; 2],
    error: f64,
    rates: [f64; 2],
    velocity: [f64; 2],
    requested: [f64; 2],
    determinant: f64,
    limited: bool,
}

fn model(angles: [f64; 2]) -> TwoJointArm {
    let mut arm = TwoJointArm::new();
    arm.theta1 = angles[0];
    arm.theta2 = angles[1];
    arm
}

fn generate(config: &Config) -> Vec<[Sample; 2]> {
    let goal = TwoJointArm::inverse_kinematics(config.target[0], config.target[1], config.branch);
    let mut poses = [config.start; 2];
    let mut frames = Vec::with_capacity(LAST + 1);
    for frame in 0..=LAST {
        let pair = std::array::from_fn(|controller| {
            let arm = model(poses[controller]);
            let p = arm.forward_kinematics();
            let tip = [p.0, p.1];
            let error = [config.target[0] - p.0, config.target[1] - p.1];
            let j = arm.jacobian();
            let (mut rates, requested) = if let Some((a, b)) = goal.filter(|_| frame < LAST) {
                if controller == 0 {
                    let q = [GAIN * wrap(a - arm.theta1), GAIN * wrap(b - arm.theta2)];
                    (
                        q,
                        [
                            j[0][0] * q[0] + j[0][1] * q[1],
                            j[1][0] * q[0] + j[1][1] * q[1],
                        ],
                    )
                } else {
                    let v = [GAIN * error[0], GAIN * error[1]];
                    (
                        arm.damped_joint_velocity(v, config.damping)
                            .expect("validated finite tracking inputs"),
                        v,
                    )
                }
            } else {
                ([0.0; 2], [0.0; 2])
            };
            // Both controllers get the same Euclidean joint-rate budget.
            let norm = rates[0].hypot(rates[1]);
            let limited = norm > SPEED_LIMIT;
            if limited {
                for rate in &mut rates {
                    *rate *= SPEED_LIMIT / norm;
                }
            }
            Sample {
                angles: poses[controller],
                tip,
                error: error[0].hypot(error[1]),
                rates,
                velocity: [
                    j[0][0] * rates[0] + j[0][1] * rates[1],
                    j[1][0] * rates[0] + j[1][1] * rates[1],
                ],
                requested,
                determinant: j[0][0] * j[1][1] - j[0][1] * j[1][0],
                limited,
            }
        });
        frames.push(pair);
        for (pose, sample) in poses.iter_mut().zip(pair) {
            for (angle, rate) in pose.iter_mut().zip(sample.rates) {
                if rate != 0.0 {
                    *angle = wrap(*angle + DT * rate);
                }
            }
        }
    }
    frames
}

pub(super) struct TrackingComparison {
    config: Config,
    frames: Vec<[Sample; 2]>,
    frame: usize,
    playing: bool,
    last_time: Option<f64>,
    fraction: f64,
    rect: Option<egui::Rect>,
    graph: Option<egui::Rect>,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: usize,
    last_action: &'static str,
    revision: usize,
    generation: usize,
    painted_frame: Option<usize>,
    painted_generation: Option<usize>,
    published_generation: Option<usize>,
    errors_json: String,
}

impl Default for TrackingComparison {
    fn default() -> Self {
        let config = Config::default();
        let frames = generate(&config);
        let mut result = Self {
            config,
            frames,
            frame: 0,
            playing: false,
            last_time: None,
            fraction: 0.0,
            rect: None,
            graph: None,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            action_count: 0,
            last_action: "none",
            revision: 0,
            generation: 0,
            painted_frame: None,
            painted_generation: None,
            published_generation: None,
            errors_json: String::new(),
        };
        result.cache_errors();
        result
    }
}

impl TrackingComparison {
    fn cache_errors(&mut self) {
        self.errors_json = format!(
            "[{}]",
            (0..2)
                .map(|i| format!(
                    "[{}]",
                    self.frames
                        .iter()
                        .map(|p| p[i].error.to_string())
                        .collect::<Vec<_>>()
                        .join(",")
                ))
                .collect::<Vec<_>>()
                .join(",")
        );
    }

    fn rebuild(&mut self, config: Config) {
        self.frames = generate(&config);
        self.config = config;
        self.generation += 1;
        self.cache_errors();
        self.seek(0);
    }

    pub(super) fn capture_pose(&mut self, start: [f64; 2], target: [f64; 2], branch: ElbowBranch) {
        self.rebuild(Config {
            start,
            target,
            branch,
            damping: self.config.damping,
        });
    }

    fn preset(&mut self, name: &str) {
        let mut config = Config {
            damping: self.config.damping,
            ..Config::default()
        };
        match name {
            "straight" => {
                config.start = [0.0, 0.0];
                config.target = [1.2, 0.0];
            }
            "near" => {
                config.start = [0.0, 0.02];
                config.target = [1.2, 0.6];
            }
            "outside" => config.target = [2.3, 0.7],
            _ => {}
        }
        self.rebuild(config);
    }

    fn seek(&mut self, frame: usize) {
        self.frame = frame.min(LAST);
        self.playing = false;
        self.last_time = None;
        self.fraction = 0.0;
    }

    pub(super) fn tick(&mut self, now: f64, active: bool) {
        let previous = self.last_time.replace(now);
        if !active || !self.playing {
            self.last_time = None;
            return;
        }
        if let Some(previous) = previous {
            let dt = now - previous;
            if (0.0..1.0).contains(&dt) {
                self.fraction += dt / DT;
                let whole = (self.fraction + 1e-9).floor() as usize;
                self.fraction = (self.fraction - whole as f64).max(0.0);
                self.frame = (self.frame + whole).min(LAST);
            }
        }
        if self.frame == LAST {
            self.playing = false;
        }
    }

    pub(super) fn playing(&self) -> bool {
        self.playing
    }

    pub(super) fn apply_share_query(&mut self, query: &str) -> bool {
        if crate::share::value(query, "tab") != Some("arm")
            || crate::share::value(query, "arm_view") != Some("compare")
        {
            return false;
        }
        self.preset(crate::share::value(query, "arm_cmp_case").unwrap_or("reach"));
        let mut c = self.config.clone();
        // URL omissions use stable defaults, independent of previous UI edits.
        c.damping = Config::default().damping;
        let bounded = |key, lo, hi| {
            crate::share::value(query, key)
                .and_then(|s| s.parse::<f64>().ok())
                .filter(|v| v.is_finite() && (lo..=hi).contains(v))
        };
        for (key, value, lo, hi) in [
            ("arm_cmp_a", &mut c.start[0], -PI, PI),
            ("arm_cmp_x", &mut c.target[0], -2.5, 2.5),
        ] {
            if let Some(v) = bounded(key, lo, hi) {
                *value = v;
            }
        }
        if let Some(v) = bounded("arm_cmp_b", -PI, PI) {
            c.start[1] = v;
        }
        if let Some(v) = bounded("arm_cmp_y", -2.5, 2.5) {
            c.target[1] = v;
        }
        if let Some(v) = bounded("arm_cmp_damping", 0.02, 1.0) {
            c.damping = v;
        }
        c.branch = if crate::share::value(query, "arm_cmp_branch") == Some("negative") {
            ElbowBranch::Negative
        } else {
            ElbowBranch::Positive
        };
        self.rebuild(c);
        self.seek(crate::share::bounded_usize(query, "arm_cmp_frame", LAST).unwrap_or(0));
        self.playing =
            self.frame < LAST && crate::share::boolean(query, "arm_cmp_playing").unwrap_or(false);
        true
    }

    pub(super) fn share_query(&self) -> String {
        let c = &self.config;
        format!("tab=arm&arm_view=compare&arm_cmp_case=custom&arm_cmp_a={}&arm_cmp_b={}&arm_cmp_x={}&arm_cmp_y={}&arm_cmp_branch={}&arm_cmp_damping={}&arm_cmp_frame={}&arm_cmp_playing=0",c.start[0],c.start[1],c.target[0],c.target[1],if c.branch==ElbowBranch::Positive {"positive"} else {"negative"},c.damping,self.frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_share_initial_conditions_and_apply_the_recorded_speed_limited_commands() {
        let d = TrackingComparison::default();
        assert_eq!(d.frames.len(), LAST + 1);
        for (frame, pair) in d.frames.iter().enumerate() {
            for (i, s) in pair.iter().enumerate() {
                assert!(s.rates[0].hypot(s.rates[1]) <= SPEED_LIMIT + 1e-12);
                assert!(
                    (s.error
                        - (s.tip[0] - d.config.target[0]).hypot(s.tip[1] - d.config.target[1]))
                    .abs()
                        < 1e-12
                );
                let j = model(s.angles).jacobian();
                for (velocity, row) in s.velocity.iter().zip(j) {
                    assert!((velocity - row[0] * s.rates[0] - row[1] * s.rates[1]).abs() < 1e-12);
                }
                if frame < LAST {
                    for joint in 0..2 {
                        assert!(
                            (wrap(d.frames[frame + 1][i].angles[joint] - s.angles[joint])
                                - DT * s.rates[joint])
                                .abs()
                                < 1e-12
                        );
                    }
                } else {
                    assert_eq!(s.rates, [0.0, 0.0]);
                }
            }
        }
        assert_eq!(d.frames[0][0].angles, d.frames[0][1].angles);
        for s in d.frames[LAST] {
            assert!(s.error < 0.01);
        }
    }

    #[test]
    fn straight_pose_exposes_radial_stall_and_unreachable_targets_hold_both_arms() {
        let mut d = TrackingComparison::default();
        d.preset("straight");
        assert!(d.frames[LAST][0].error < 0.01);
        assert!(d.frames[0][0].rates[0].hypot(d.frames[0][0].rates[1]) > 1.9);
        for pair in &d.frames {
            assert_eq!(pair[1].angles, [0.0, 0.0]);
            assert!((pair[1].error - 0.8).abs() < 1e-12);
        }
        d.preset("outside");
        for pair in &d.frames {
            for s in pair {
                assert_eq!(s.angles, d.config.start);
                assert_eq!(s.rates, [0.0, 0.0]);
            }
        }
        d.preset("near");
        assert_ne!(d.frames[0][1].tip, d.frames[LAST][1].tip);
    }

    #[test]
    fn damping_changes_only_cartesian_run_and_branch_changes_only_joint_run() {
        let mut d = TrackingComparison::default();
        let initial = d.frames.clone();
        let mut c = d.config.clone();
        c.damping = 0.6;
        d.rebuild(c);
        assert!(d.frames.iter().zip(&initial).all(|(a, b)| a[0] == b[0]));
        assert_ne!(d.frames[80][1].angles, initial[80][1].angles);
        let initial = d.frames.clone();
        let mut c = d.config.clone();
        c.branch = ElbowBranch::Negative;
        d.rebuild(c);
        assert!(d.frames.iter().zip(&initial).all(|(a, b)| a[1] == b[1]));
        assert_ne!(d.frames[80][0].angles, initial[80][0].angles);
    }

    #[test]
    fn share_reconstructs_precise_runs_at_the_same_paused_frame() {
        let mut a = TrackingComparison::default();
        assert!(a.apply_share_query("tab=arm&arm_view=compare&arm_cmp_a=-1.234567890123&arm_cmp_b=2.345678901234&arm_cmp_x=-0.876543210987&arm_cmp_y=1.123456789012&arm_cmp_branch=negative&arm_cmp_damping=0.345678901234&arm_cmp_frame=237&arm_cmp_playing=1"));
        let mut b = TrackingComparison::default();
        assert!(b.apply_share_query(&a.share_query()));
        assert_eq!(a.config, b.config);
        assert_eq!(a.frames, b.frames);
        assert_eq!(a.frame, b.frame);
        assert!(!b.playing);
        let before = b.share_query();
        assert!(!b.apply_share_query("tab=grid&arm_view=compare"));
        assert_eq!(b.share_query(), before);
        b.apply_share_query("tab=arm&arm_view=compare&arm_cmp_a=NaN&arm_cmp_b=99&arm_cmp_x=inf&arm_cmp_damping=0&arm_cmp_frame=99999");
        assert_eq!(b.config, Config::default());
        assert_eq!(b.frame, 0);
    }

    #[test]
    fn playback_uses_elapsed_time_freezes_and_discards_background_gaps() {
        let mut d = TrackingComparison {
            playing: true,
            ..TrackingComparison::default()
        };
        d.tick(0.0, true);
        d.tick(0.4, true);
        assert_eq!(d.frame, 40);
        d.seek(123);
        d.tick(1.0, true);
        d.tick(1.4, true);
        assert_eq!(d.frame, 123);
        d.playing = true;
        d.tick(2.0, true);
        d.tick(200.0, true);
        assert_eq!(d.frame, 123);
        d.tick(200.4, true);
        assert_eq!(d.frame, 163);
        d.tick(201.0, false);
        d.tick(202.0, true);
        assert_eq!(d.frame, 163);
        d.seek(LAST - 1);
        d.playing = true;
        d.tick(203.0, true);
        d.tick(203.4, true);
        assert_eq!(d.frame, LAST);
        assert!(!d.playing);
    }
}
