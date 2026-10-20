//! Interactive geometry and joint tracking for the library's two-link arm.

mod view;

use rust_robotics_control::two_joint_arm_control::{ElbowBranch, TwoJointArm};
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

pub struct ArmDemo {
    arm: TwoJointArm,
    branch: ElbowBranch,
    joints_mode: bool,
    playing: bool,
    show_other: bool,
    last_time: Option<f64>,
    elapsed: f64,
    steps: usize,
    trail: Vec<[f64; 2]>,
    rect: Option<egui::Rect>,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: usize,
    last_action: &'static str,
    revision: usize,
}

impl Default for ArmDemo {
    fn default() -> Self {
        let mut demo = Self {
            arm: TwoJointArm::new(),
            branch: ElbowBranch::Positive,
            joints_mode: false,
            playing: false,
            show_other: true,
            last_time: None,
            elapsed: 0.0,
            steps: 0,
            trail: Vec::new(),
            rect: None,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            action_count: 0,
            last_action: "none",
            revision: 0,
        };
        demo.preset("reach");
        demo
    }
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

impl ArmDemo {
    fn target(&self) -> [f64; 2] {
        [self.arm.target_x, self.arm.target_y]
    }

    fn tip(&self) -> [f64; 2] {
        let (x, y) = self.arm.forward_kinematics();
        [x, y]
    }

    fn solution(&self, branch: ElbowBranch) -> Option<[f64; 2]> {
        TwoJointArm::inverse_kinematics(self.arm.target_x, self.arm.target_y, branch)
            .map(|(a, b)| [a, b])
    }

    fn other_branch(&self) -> ElbowBranch {
        if self.branch == ElbowBranch::Positive {
            ElbowBranch::Negative
        } else {
            ElbowBranch::Positive
        }
    }

    fn error(&self) -> f64 {
        let [x, y] = self.tip();
        (x - self.arm.target_x).hypot(y - self.arm.target_y)
    }

    fn determinant(&self) -> f64 {
        let [a, b] = TwoJointArm::LINK_LENGTHS;
        a * b * self.arm.theta2.sin()
    }

    fn clear_motion(&mut self) {
        self.playing = false;
        self.last_time = None;
        self.elapsed = 0.0;
        self.steps = 0;
        self.trail = vec![self.tip()];
    }

    fn preset(&mut self, name: &str) {
        self.arm.theta1 = -0.35;
        self.arm.theta2 = 0.4;
        let target = match name {
            "stretch" => [2.0, 0.0],
            "outside" => [2.3, 0.7],
            "fold" => [0.0, 0.0],
            _ => [1.2, 1.1],
        };
        self.arm.set_target(target[0], target[1]);
        self.branch = ElbowBranch::Positive;
        self.joints_mode = false;
        self.clear_motion();
    }

    fn set_target(&mut self, target: [f64; 2]) {
        self.arm.set_target(target[0], target[1]);
        self.joints_mode = false;
        if self.solution(self.branch).is_none() {
            self.playing = false;
        }
    }

    fn snap(&mut self) {
        if let Some([a, b]) = self.solution(self.branch) {
            self.arm.theta1 = wrap(a);
            self.arm.theta2 = wrap(b);
            self.joints_mode = false;
            self.clear_motion();
        }
    }

    /// Exact integration of a gain-4 proportional joint-angle controller.
    /// This is kinematic tracking, without inertia or collision checking.
    fn advance(&mut self, dt: f64) {
        if self.joints_mode || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        let Some([a, b]) = self.solution(self.branch) else {
            return;
        };
        let alpha = 1.0 - (-4.0 * dt).exp();
        self.arm.theta1 = wrap(self.arm.theta1 + alpha * wrap(a - self.arm.theta1));
        self.arm.theta2 = wrap(self.arm.theta2 + alpha * wrap(b - self.arm.theta2));
        self.elapsed += dt;
        self.steps += 1;
        let tip = self.tip();
        if self
            .trail
            .last()
            .map_or(true, |p| (p[0] - tip[0]).hypot(p[1] - tip[1]) > 1e-5)
        {
            self.trail.push(tip);
            if self.trail.len() > 240 {
                self.trail.remove(0);
            }
        }
    }

    fn tick(&mut self, now: f64, active: bool) {
        let previous = self.last_time.replace(now);
        if !active || !self.playing {
            self.last_time = None;
            return;
        }
        if let Some(previous) = previous {
            let dt = now - previous;
            // A background tab must not jump when the user returns.
            if (0.0..1.0).contains(&dt) {
                self.advance(dt);
            }
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        self.tick(ctx.input(|i| i.time), active);
        if active && self.playing {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("arm") {
            return;
        }
        let bounded = |key, lo, hi| {
            crate::share::value(query, key)
                .and_then(|s| s.parse::<f64>().ok())
                .filter(|v| v.is_finite() && (lo..=hi).contains(v))
        };
        if let Some(v) = bounded("arm_x", -2.5, 2.5) {
            self.arm.target_x = v;
        }
        if let Some(v) = bounded("arm_y", -2.5, 2.5) {
            self.arm.target_y = v;
        }
        if let Some(v) = bounded("arm_a", -PI, PI) {
            self.arm.theta1 = v;
        }
        if let Some(v) = bounded("arm_b", -PI, PI) {
            self.arm.theta2 = v;
        }
        self.branch = if crate::share::value(query, "arm_branch") == Some("negative") {
            ElbowBranch::Negative
        } else {
            ElbowBranch::Positive
        };
        self.joints_mode = crate::share::value(query, "arm_mode") == Some("fk");
        self.show_other = crate::share::boolean(query, "arm_other").unwrap_or(true);
        self.clear_motion();
        self.playing = !self.joints_mode
            && self.solution(self.branch).is_some()
            && crate::share::boolean(query, "arm_playing").unwrap_or(false);
    }

    pub fn share_query(&self) -> String {
        // Open at the exact current pose; a running controller would move it.
        format!("tab=arm&arm_x={}&arm_y={}&arm_a={}&arm_b={}&arm_branch={}&arm_mode={}&arm_other={}&arm_playing=0",
            self.arm.target_x, self.arm.target_y, self.arm.theta1, self.arm.theta2,
            if self.branch == ElbowBranch::Positive { "positive" } else { "negative" },
            if self.joints_mode { "fk" } else { "ik" }, u8::from(self.show_other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_branches_reach_the_goal_and_outside_goals_hold_pose() {
        let mut d = ArmDemo::default();
        for branch in [ElbowBranch::Positive, ElbowBranch::Negative] {
            d.branch = branch;
            d.snap();
            assert!(d.error() < 1e-12);
            assert_eq!(
                d.arm.theta2.is_sign_positive(),
                branch == ElbowBranch::Positive
            );
        }
        let before = d.tip();
        d.set_target([2.3, 0.7]);
        assert!(d.solution(d.branch).is_none());
        assert!(d.solution(d.other_branch()).is_none());
        d.snap();
        d.advance(0.4);
        assert_eq!(d.tip(), before);
        assert_eq!(d.steps, 0);
        for name in ["stretch", "fold"] {
            d.preset(name);
            d.snap();
            assert!(d.error() < 1e-12 && d.determinant().abs() < 1e-12);
        }
    }

    #[test]
    fn elapsed_time_tracking_is_stable_and_pauses_discard_background_gaps() {
        let mut a = ArmDemo::default();
        let mut b = ArmDemo::default();
        a.advance(0.4);
        for _ in 0..40 {
            b.advance(0.01);
        }
        assert!((a.arm.theta1 - b.arm.theta1).abs() < 1e-12);
        assert!((a.arm.theta2 - b.arm.theta2).abs() < 1e-12);
        assert!(a.error() < ArmDemo::default().error());
        let before = a.tip();
        a.tick(1.0, true);
        a.tick(1.4, true);
        assert_eq!(a.tip(), before);
        a.playing = true;
        a.tick(2.0, true);
        a.tick(200.0, true);
        assert_eq!(a.tip(), before);
        a.tick(200.4, true);
        assert_ne!(a.tip(), before);
        a.joints_mode = true;
        let before = a.tip();
        a.advance(0.4);
        assert_eq!(a.tip(), before);
    }

    #[test]
    fn sharing_preserves_precise_pose_target_branch_and_mode() {
        let mut a = ArmDemo::default();
        a.apply_share_query("tab=arm&arm_x=-1.23456789012345&arm_y=0.87654321098765&arm_a=1.23456789012345&arm_b=-2.34567890123456&arm_branch=negative&arm_mode=fk&arm_other=0");
        let mut b = ArmDemo::default();
        b.apply_share_query(&a.share_query());
        assert_eq!(a.share_query(), b.share_query());
        assert_eq!(a.tip(), b.tip());
        assert_eq!(a.target(), b.target());
        assert!(b.joints_mode && !b.show_other && !b.playing);
        assert_eq!(b.trail, vec![b.tip()]);
        let ctx = egui::Context::default();
        let before = b.share_query();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| b.controls(ctx, ui));
        });
        assert_eq!(
            b.share_query(),
            before,
            "display rounding must not edit precise angles"
        );
    }

    #[test]
    fn malformed_queries_and_other_tabs_cannot_install_bad_geometry() {
        let mut d = ArmDemo::default();
        let before = d.share_query();
        d.apply_share_query("tab=grid&arm_a=1&arm_y=2");
        assert_eq!(d.share_query(), before);
        d.apply_share_query(
            "tab=arm&arm_x=NaN&arm_y=99&arm_a=inf&arm_b=-4&arm_branch=bad&arm_mode=bad",
        );
        assert_eq!(d.share_query(), before);
        d.apply_share_query("tab=arm&arm_x=2.5&arm_y=2.5&arm_playing=1");
        assert!(!d.playing && d.solution(d.branch).is_none());
    }
}
