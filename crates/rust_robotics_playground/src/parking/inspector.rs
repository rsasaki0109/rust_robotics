//! Inspect the planner's actual samples and gear boundaries. Playback indexes
//! recorded poses; it never interpolates an unchecked pose or replans a seek.

mod view;

use super::{wrap, ParkingDemo, Plan, Pose, LOT_H, LOT_W, PARALLEL_GOAL, PARALLEL_START};
use std::collections::BTreeMap;

const REPLAY_RATE: f64 = 10.0;

pub(super) struct Inspector {
    pub(super) active: bool,
    cursor: usize,
    playing: bool,
    ghosts: bool,
    circles: bool,
    clock: Option<f64>,
    pending: f64,
    event: Option<bool>,
    generation: usize,
    plan_json: String,
    changes: Vec<usize>,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<egui::Rect>,
    graph: Option<egui::Rect>,
    painted: Option<(usize, usize, bool, bool)>,
    witnesses: String,
    ghost_indices: Vec<usize>,
}

impl Default for Inspector {
    fn default() -> Self {
        Self {
            active: false,
            cursor: 0,
            playing: false,
            ghosts: true,
            circles: false,
            clock: None,
            pending: 0.0,
            event: None,
            generation: 0,
            plan_json: "null".into(),
            changes: Vec::new(),
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map: None,
            graph: None,
            painted: None,
            witnesses: "[]".into(),
            ghost_indices: Vec::new(),
        }
    }
}

impl Inspector {
    pub(super) fn record(&mut self, plan: Option<&Plan>) {
        self.generation += 1;
        self.clock = None;
        self.pending = 0.0;
        self.playing = false;
        self.changes.clear();
        self.plan_json = "null".into();
        if let Some(p) = plan {
            self.changes = p
                .directions
                .windows(2)
                .enumerate()
                .filter_map(|(i, w)| (w[0] != w[1]).then_some(i + 1))
                .collect();
            if let Some(at_switch) = self.event.take() {
                let index = self
                    .changes
                    .iter()
                    .copied()
                    .find(|&i| p.distance[i] > 1e-9)
                    .or_else(|| self.changes.first().copied())
                    .unwrap_or(0);
                self.cursor = index.saturating_sub(usize::from(!at_switch));
            }
            self.cursor = self.cursor.min(p.poses.len() - 1);
            self.plan_json = format!(
                "{{\"poses\":{:?},\"directions\":{:?},\"distance\":{:?},\"changes\":{:?}}}",
                p.poses, p.directions, p.distance, self.changes
            );
        } else {
            self.cursor = 0;
            self.event = None;
        }
    }

    fn widget(&mut self, name: &'static str, r: &egui::Response) {
        self.actions.insert(name, r.rect.center().into());
        self.hits.insert(name, r.hovered());
    }

    fn ack(&mut self, name: &'static str, ctx: &egui::Context) {
        self.count += 1;
        self.last_action = name;
        ctx.request_repaint();
    }

    pub(super) fn button(&mut self, name: &'static str, r: &egui::Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.ack(name, &r.ctx);
            true
        } else {
            false
        }
    }

    fn seek(&mut self, index: usize, last: usize) {
        self.cursor = index.min(last);
        self.playing = false;
        self.clock = None;
        self.pending = 0.0;
    }

    fn advance(&mut self, now: f64, last: usize) {
        let previous = self.clock.replace(now);
        if !self.playing {
            return;
        }
        if let Some(previous) = previous {
            self.pending += (now - previous).clamp(0.0, 1.0) * REPLAY_RATE;
            let steps = self.pending.floor() as usize;
            self.pending -= steps as f64;
            self.cursor = (self.cursor + steps).min(last);
            if self.cursor == last {
                self.playing = false;
                self.pending = 0.0;
            }
        }
    }
}

fn precise_pose(query: &str, name: &str) -> Option<Pose> {
    let v: Vec<f64> = crate::share::value(query, name)?
        .split(',')
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    (v.len() == 3
        && v.iter().all(|x| x.is_finite())
        && (0.0..=LOT_W).contains(&v[0])
        && (0.0..=LOT_H).contains(&v[1]))
    .then(|| [v[0], v[1], wrap(v[2])])
}

impl ParkingDemo {
    pub(super) fn enter_inspector(&mut self) {
        self.ensure_planned();
        self.inspector.active = true;
        self.inspector.seek(0, self.inspector_last());
        self.inspector.record(self.plan.as_ref());
    }

    pub(super) fn apply_inspector_query(&mut self, query: &str) {
        self.inspector.active = true;
        self.car = PARALLEL_START;
        self.goal = PARALLEL_GOAL;
        self.plan = None;
        match crate::share::value(query, "parking_case") {
            Some("back") => {
                self.car = [4.0, 9.0, 0.0];
                self.goal = [19.85, 18.4, -std::f64::consts::FRAC_PI_2];
            }
            Some("turn") => {
                self.car = [15.0, 7.0, 0.0];
                self.goal = [15.0, 7.0, -std::f64::consts::PI];
            }
            Some("blocked") => self.goal = [2.5, 1.4, 0.0],
            _ => {}
        }
        if let Some(p) = precise_pose(query, "parking_start") {
            self.car = p;
        }
        if let Some(p) = precise_pose(query, "parking_goal") {
            self.goal = p;
        }
        self.inspector.cursor = crate::share::value(query, "parking_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.inspector.event = match crate::share::value(query, "parking_event") {
            Some("before") => Some(false),
            Some("switch") => Some(true),
            _ => None,
        };
        self.inspector.ghosts = crate::share::boolean(query, "parking_sweep").unwrap_or(true);
        self.inspector.circles = crate::share::boolean(query, "parking_circles").unwrap_or(false);
        self.inspector.playing = false;
        self.inspector.clock = None;
        self.dragging = None;
        self.failure = None;
        self.driven = 0.0;
        self.last_time = None;
        self.needs_plan = true;
    }

    pub(super) fn inspector_query(&self) -> String {
        format!("tab=parking&parking_view=inspect&parking_start={},{},{}&parking_goal={},{},{}&parking_at={}&parking_sweep={}&parking_circles={}", self.car[0],self.car[1],self.car[2],self.goal[0],self.goal[1],self.goal[2],self.inspector.cursor,u8::from(self.inspector.ghosts),u8::from(self.inspector.circles))
    }

    fn inspector_last(&self) -> usize {
        self.plan.as_ref().map_or(0, |p| p.poses.len() - 1)
    }

    pub(super) fn inspected_pose(&self) -> Pose {
        self.plan.as_ref().map_or(self.car, |p| {
            p.poses[self.inspector.cursor.min(p.poses.len() - 1)]
        })
    }

    fn inspector_goal(&mut self, goal: Pose) {
        // Every edit starts from the configured rear-axle pose. Seeking only
        // changes the view and never changes the next planning problem.
        self.plan = None;
        self.goal = [goal[0], goal[1], wrap(goal[2])];
        self.inspector.cursor = 0;
        self.plan_now();
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        if active && self.inspector.active {
            self.ensure_planned();
            self.inspector
                .advance(ctx.input(|i| i.time), self.inspector_last());
            if self.inspector.playing {
                ctx.request_repaint();
            }
        } else {
            self.inspector.clock = None;
            self.inspector.pending = 0.0;
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.inspector.revision += 1;
        let active = active && self.inspector.active;
        if !active {
            self.inspector.painted = None;
        }
        self.publish_inspector(ctx, active);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inspected(query: &str) -> ParkingDemo {
        let mut d = ParkingDemo::default();
        d.apply_share_query(&format!("tab=parking&parking_view=inspect&{query}"));
        d.ensure_planned();
        d
    }

    #[test]
    fn inspector_uses_exact_planner_poses_and_real_gear_boundaries() {
        let mut original = ParkingDemo::default();
        original.ensure_planned();
        let mut d = inspected("parking_event=before");
        let p = d.plan.as_ref().unwrap();
        assert_eq!(p.poses, original.plan.as_ref().unwrap().poses);
        assert_eq!(p.directions, original.plan.as_ref().unwrap().directions);
        let changes: Vec<_> = p
            .directions
            .windows(2)
            .enumerate()
            .filter(|(_, w)| w[0] != w[1])
            .map(|(i, _)| i + 1)
            .collect();
        assert_eq!(d.inspector.changes, changes);
        let switch = changes
            .iter()
            .copied()
            .find(|&i| p.distance[i] > 1e-9)
            .unwrap();
        assert_eq!(d.inspector.cursor, switch - 1);
        let direction = p.directions[switch - 1];
        let poses = p.poses.clone();
        let generation = d.inspector.generation;
        for (i, pose) in poses.iter().enumerate() {
            d.inspector.seek(i, poses.len() - 1);
            assert_eq!(d.current_pose(), *pose);
        }
        d.inspector.seek(switch, poses.len() - 1);
        assert_ne!(d.plan.as_ref().unwrap().directions[switch], direction);
        d.ensure_planned();
        assert_eq!(d.plan.as_ref().unwrap().poses, poses);
        assert_eq!(
            d.inspector.generation, generation,
            "seeking must not solve another path"
        );
    }

    #[test]
    fn playback_counts_real_samples_and_pause_and_tab_switch_do_not_advance() {
        let mut d = inspected("");
        let last = d.inspector_last();
        d.inspector.playing = true;
        d.inspector.advance(10.0, last);
        d.inspector.advance(10.5, last);
        assert_eq!(
            d.inspector.cursor, 5,
            "one slow frame advances every elapsed sample"
        );
        assert_eq!(
            d.current_pose(),
            d.plan.as_ref().unwrap().poses[d.inspector.cursor]
        );
        d.inspector.seek(2, last);
        d.inspector.advance(30.0, last);
        assert_eq!(d.inspector.cursor, 2);
        d.inspector.playing = true;
        d.inspector.clock = None;
        d.inspector.advance(100.0, last);
        assert_eq!(
            d.inspector.cursor, 2,
            "time outside the active tab is not replayed"
        );
        d.inspector.seek(last - 1, last);
        d.inspector.playing = true;
        d.inspector.advance(110.0, last);
        d.inspector.advance(111.0, last);
        assert_eq!(d.inspector.cursor, last);
        assert!(!d.inspector.playing);
    }

    #[test]
    fn exact_shares_restore_the_same_problem_samples_cursor_and_overlays() {
        let d=inspected("parking_start=3.123456789,8.234567891,0.123456789&parking_goal=20.123456789,8.987654321,0.234567891&parking_at=13&parking_sweep=0&parking_circles=1");
        assert!(d.plan.is_some(), "{:?}", d.failure);
        let q = d.share_query();
        let mut restored = ParkingDemo::default();
        restored.apply_share_query(&q);
        restored.ensure_planned();
        assert_eq!(restored.share_query(), q);
        assert_eq!(restored.car, d.car);
        assert_eq!(restored.goal, d.goal);
        assert_eq!(restored.inspector.plan_json, d.inspector.plan_json);
        assert_eq!(restored.inspected_pose(), d.inspected_pose());
        assert!(!restored.inspector.playing);
        assert!(!restored.inspector.ghosts);
        assert!(restored.inspector.circles);
        restored.apply_share_query("tab=parking&car=3,8,0");
        assert!(
            !restored.inspector.active,
            "legacy parking links retain live mode"
        );
    }

    #[test]
    fn goal_edits_keep_the_configured_start_and_surface_actual_planning_failure() {
        let mut d = inspected("");
        let start = d.car;
        d.inspector.seek(d.inspector_last() / 2, d.inspector_last());
        d.inspector_goal([25.0, 9.0, 0.0]);
        assert_eq!(d.car, start);
        assert_eq!(d.plan.as_ref().unwrap().poses[0], start);
        assert_eq!(d.inspector.cursor, 0);
        assert!(!d.inspector.playing);
        d.inspector_goal([2.5, 1.4, 0.0]);
        assert_eq!(d.car, start);
        assert!(d.plan.is_none());
        assert!(d.failure.as_ref().unwrap().contains("collides"));
        assert_eq!(d.inspector.plan_json, "null");
        assert!(d.inspector.changes.is_empty());
        let mut restored = ParkingDemo::default();
        restored.apply_share_query(&d.share_query());
        restored.ensure_planned();
        assert_eq!(restored.failure, d.failure);
        assert!(restored.plan.is_none());
    }
}
