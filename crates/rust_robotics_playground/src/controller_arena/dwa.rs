//! Inspect the actual DWA sample lattice, costs, fallback and applied motion.
#[cfg(test)]
mod tests;
mod view;

use egui::{Context, Rect, Response};
use rust_robotics_core::Point2D;
use rust_robotics_planning::dwa::{motion, DWAConfig, DWAPlanTrace, DWAPlanner, DWAState};
use std::collections::BTreeMap;

type Point = [f64; 2];
const DT: f64 = 0.2;
const LIMIT: usize = 120;
const WORLD: Point = [12., 8.];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Case {
    Open,
    Wall,
    Crossing,
    Trapped,
}
impl Case {
    const ALL: [Self; 4] = [Self::Open, Self::Wall, Self::Crossing, Self::Trapped];
    fn slug(self) -> &'static str {
        match self {
            Self::Open => "open",
            Self::Wall => "wall",
            Self::Crossing => "crossing",
            Self::Trapped => "trapped",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Open => "Open floor",
            Self::Wall => "Wall",
            Self::Crossing => "Crossing point",
            Self::Trapped => "Blocked start",
        }
    }
    fn probe(self) -> Point {
        match self {
            Self::Open => [6., 6.],
            Self::Wall => [5., 4.1],
            Self::Crossing => [5., 6.5],
            Self::Trapped => [2., 4.],
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Policy {
    Balanced,
    Clearance,
    Speed,
}
impl Policy {
    const ALL: [Self; 3] = [Self::Balanced, Self::Clearance, Self::Speed];
    fn slug(self) -> &'static str {
        match self {
            Self::Balanced => "balanced",
            Self::Clearance => "clearance",
            Self::Speed => "speed",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Balanced => "Balanced",
            Self::Clearance => "More clearance",
            Self::Speed => "More speed",
        }
    }
    fn weights(self) -> [f64; 3] {
        match self {
            Self::Balanced => [1.5, 1., 1.],
            Self::Clearance => [1.5, 1., 3.],
            Self::Speed => [1.5, 3., 0.3],
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Edit {
    Inspect,
    Goal,
    Obstacle,
}
impl Edit {
    const ALL: [Self; 3] = [Self::Inspect, Self::Goal, Self::Obstacle];
    fn slug(self) -> &'static str {
        match self {
            Self::Inspect => "inspect_arc",
            Self::Goal => "edit_goal",
            Self::Obstacle => "edit_obstacle",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Inspect => "Inspect",
            Self::Goal => "Move goal",
            Self::Obstacle => "Move gold point",
        }
    }
}
fn point(s: &DWAState) -> Point {
    [s[0], s[1]]
}
fn array(s: &DWAState) -> [f64; 5] {
    [s[0], s[1], s[2], s[3], s[4]]
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn parse_point(q: &str, key: &str) -> Option<Point> {
    let (a, b) = crate::share::value(q, key)?.split_once(',')?;
    let p = [a.parse::<f64>().ok()?, b.parse::<f64>().ok()?];
    (p.iter().all(|v| v.is_finite()) && (0.5..=11.5).contains(&p[0]) && (0.5..=7.5).contains(&p[1]))
        .then_some(p)
}

pub(super) struct DwaLab {
    pub(super) active: bool,
    case: Case,
    policy: Policy,
    horizon: f64,
    radius: f64,
    goal: Point,
    probe: Point,
    states: Vec<DWAState>,
    controls: Vec<[f64; 2]>,
    trace: Option<DWAPlanTrace>,
    cursor: usize,
    selected: usize,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    generation: usize,
    edit: Edit,
    cloud: bool,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    scrolling: bool,
    scroll_revision: usize,
    map: Option<Rect>,
    velocity: Option<Rect>,
    bars: Option<Rect>,
    timeline: Option<Rect>,
    painted: Option<[usize; 4]>,
    witnesses: String,
}
impl Default for DwaLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Wall,
            policy: Policy::Balanced,
            horizon: 2.4,
            radius: 0.45,
            goal: [10., 4.],
            probe: Case::Wall.probe(),
            states: vec![],
            controls: vec![],
            trace: None,
            cursor: 0,
            selected: 0,
            playing: false,
            clock: None,
            pending: 0.,
            generation: 0,
            edit: Edit::Inspect,
            cloud: true,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            scrolling: false,
            scroll_revision: 0,
            map: None,
            velocity: None,
            bars: None,
            timeline: None,
            painted: None,
            witnesses: "[]".into(),
        }
    }
}
impl DwaLab {
    fn config(&self) -> DWAConfig {
        let [a, b, c] = self.policy.weights();
        DWAConfig {
            max_speed: 1.2,
            min_speed: 0.,
            max_yaw_rate: 1.2,
            max_accel: 1.,
            max_delta_yaw_rate: 2.,
            v_resolution: 0.1,
            yaw_rate_resolution: 0.1,
            dt: DT,
            predict_time: self.horizon,
            to_goal_cost_gain: a,
            speed_cost_gain: b,
            obstacle_cost_gain: c,
            robot_radius: self.radius,
            goal_threshold: 0.4,
            ..DWAConfig::default()
        }
    }
    fn obstacles(&self, at: usize) -> Vec<Point> {
        // A set of point obstacles, exactly as consumed by the library. Borders
        // and the wall are sampled points too, not continuous collision shapes.
        let mut p = Vec::new();
        for i in 0..=40 {
            let x = i as f64 * 0.3;
            p.push([x, 0.]);
            p.push([x, 8.]);
        }
        for i in 1..27 {
            let y = i as f64 * 0.3;
            p.push([0., y]);
            p.push([12., y]);
        }
        if self.case == Case::Wall {
            for i in 0..18 {
                p.push([6., 0.6 + i as f64 * 0.3]);
            }
        }
        if self.case == Case::Crossing {
            p.push([6., 4. + 2. * (at as f64 * DT * 1.1).sin()]);
        }
        p.push(self.probe);
        p
    }
    fn planner(&self, state: DWAState, at: usize) -> DWAPlanner {
        let mut p = DWAPlanner::new(self.config());
        p.set_state(state);
        p.set_goal(Point2D::new(self.goal[0], self.goal[1]));
        p.set_obstacles(
            self.obstacles(at)
                .iter()
                .map(|q| Point2D::new(q[0], q[1]))
                .collect(),
        );
        p
    }
    fn record(&mut self) {
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        self.cursor = 0;
        let mut state = DWAState::new(2., 4., 0., 0.6, 0.);
        self.states = vec![state];
        self.controls.clear();
        for at in 0..LIMIT {
            let mut p = self.planner(state, at);
            if p.is_goal_reached() {
                break;
            }
            let u = p.plan_step();
            self.controls.push([u[0], u[1]]);
            state = motion(state, u, DT);
            self.states.push(state);
        }
        self.generation += 1;
        self.seek(0);
    }
    fn last(&self) -> usize {
        self.states.len().saturating_sub(1)
    }
    fn seek(&mut self, at: usize) {
        self.cursor = at.min(self.last());
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        if let Some(s) = self.states.get(self.cursor) {
            self.trace = Some(
                self.planner(*s, self.cursor)
                    .try_plan_step_with_trace()
                    .unwrap(),
            );
            self.selected = self.trace().selected.unwrap_or(0);
        }
    }
    fn trace(&self) -> &DWAPlanTrace {
        self.trace.as_ref().unwrap()
    }
    fn state(&self) -> &DWAState {
        &self.states[self.cursor]
    }
    fn reached(&self) -> bool {
        distance(point(self.state()), self.goal) <= 0.4
    }
    fn status(&self) -> &'static str {
        if self.reached() {
            "arrived"
        } else if self.cursor == LIMIT {
            "limit"
        } else if self.trace().selected.is_none() {
            "blocked"
        } else {
            "planning"
        }
    }
    fn preset(&mut self, case: Case) {
        self.case = case;
        self.goal = [10., 4.];
        self.probe = case.probe();
        self.record();
    }
    fn advance(&mut self, now: f64) {
        let before = self.clock.replace(now);
        if !self.playing {
            self.pending = 0.;
            return;
        }
        let Some(before) = before else {
            return;
        };
        let dt = now - before;
        if !(0.0..1.).contains(&dt) {
            return;
        }
        self.pending += dt;
        let n = ((self.pending + 1e-9) / DT).floor() as usize;
        if n > 0 {
            let pending = (self.pending - n as f64 * DT).max(0.);
            let at = (self.cursor + n).min(self.last());
            self.seek(at);
            self.playing = at < self.last();
            self.pending = if self.playing { pending } else { 0. };
            self.clock = Some(now);
        }
    }
    pub(super) fn enter(&mut self) {
        self.active = true;
        self.record();
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "tab") == Some("arena")
            && crate::share::value(q, "arena_view") == Some("dwa");
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|s| Some(s.slug()) == crate::share::value(q, "dwa_case"))
            .unwrap_or(Case::Wall);
        self.policy = Policy::ALL
            .into_iter()
            .find(|s| Some(s.slug()) == crate::share::value(q, "dwa_policy"))
            .unwrap_or(Policy::Balanced);
        self.horizon = crate::share::value(q, "dwa_horizon")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| [1.6, 2.4, 3.2].contains(v))
            .unwrap_or(2.4);
        self.radius = crate::share::value(q, "dwa_radius")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| [0.25, 0.45, 0.7].contains(v))
            .unwrap_or(0.45);
        self.goal = parse_point(q, "dwa_goal").unwrap_or([10., 4.]);
        self.probe = parse_point(q, "dwa_probe").unwrap_or(self.case.probe());
        self.cloud = crate::share::boolean(q, "dwa_cloud").unwrap_or(true);
        self.edit = Edit::ALL
            .into_iter()
            .find(|s| Some(s.slug()) == crate::share::value(q, "dwa_edit"))
            .unwrap_or(Edit::Inspect);
        self.record();
        self.seek(crate::share::bounded_usize(q, "dwa_at", LIMIT).unwrap_or(0));
        self.selected = crate::share::bounded_usize(q, "dwa_selected", 1000)
            .unwrap_or(self.selected)
            .min(self.trace().candidates.len().saturating_sub(1));
    }
    pub(super) fn share_query(&self) -> String {
        format!("tab=arena&arena_view=dwa&dwa_case={}&dwa_policy={}&dwa_horizon={}&dwa_radius={}&dwa_goal={},{}&dwa_probe={},{}&dwa_at={}&dwa_selected={}&dwa_cloud={}&dwa_edit={}",self.case.slug(),self.policy.slug(),self.horizon,self.radius,self.goal[0],self.goal[1],self.probe[0],self.probe[1],self.cursor,self.selected,u8::from(self.cloud),self.edit.slug())
    }
    pub(super) fn begin_frame(&mut self, ctx: &Context, active: bool) {
        self.actions.clear();
        self.hits.clear();
        self.painted = None;
        let raw = ctx.input(|i| i.raw_scroll_delta != egui::Vec2::ZERO);
        if raw {
            self.scroll_revision += 1;
        }
        self.scrolling = raw || ctx.input(|i| i.smooth_scroll_delta != egui::Vec2::ZERO);
        if self.active && active {
            self.advance(ctx.input(|i| i.time));
        } else {
            self.clock = None;
            self.pending = 0.;
        }
    }
    fn widget(&mut self, name: &'static str, r: &Response) {
        self.actions.insert(name, r.rect.center().into());
        self.hits.insert(name, r.hovered());
    }
    fn ack(&mut self, name: &'static str, ctx: &Context) {
        self.count += 1;
        self.last_action = name;
        ctx.request_repaint();
    }
    pub(super) fn button(&mut self, name: &'static str, r: &Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.ack(name, &r.ctx);
            true
        } else {
            false
        }
    }
    pub(super) fn finish(&mut self, ctx: &Context, active: bool) {
        self.revision += 1;
        self.publish(ctx, active && self.active);
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn publish(&self, _ctx: &Context, _active: bool) {}
    #[cfg(target_arch = "wasm32")]
    fn publish(&self, ctx: &Context, active: bool) {
        let Some(c) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |k: &str, v: String| {
            let _ = c.set_attribute(&format!("data-dwa-{k}"), &v);
        };
        set("active", u8::from(active).to_string());
        set(
            "actions",
            format!(
                "{{{}}}",
                self.actions
                    .iter()
                    .map(|(k, p)| format!("\"{k}\":{p:?}"))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set(
            "hits",
            format!(
                "{{{}}}",
                self.hits
                    .iter()
                    .map(|(k, p)| format!("\"{k}\":{p}"))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set("action-count", self.count.to_string());
        set("last-action", self.last_action.into());
        set("ui-revision", self.revision.to_string());
        set("scrolling", u8::from(self.scrolling).to_string());
        set("scroll-revision", self.scroll_revision.to_string());
        set(
            "pointer",
            ctx.input(|i| i.pointer.hover_pos())
                .map_or("null".into(), |p| format!("{:?}", [p.x, p.y])),
        );
        if !active || self.trace.is_none() {
            return;
        }
        let t = self.trace();
        let optional = |v: Option<f64>| {
            v.filter(|v| v.is_finite())
                .map_or("null".into(), |v| v.to_string())
        };
        let candidates=t.candidates.iter().map(|s|format!("{{\"control\":{:?},\"states\":{:?},\"goal\":{},\"speed\":{},\"obstacle\":{},\"total\":{}}}",
            [s.trajectory.control[0],s.trajectory.control[1]],s.trajectory.states.iter().map(array).collect::<Vec<_>>(),s.goal_cost,s.speed_cost,optional(s.obstacle_cost),optional(s.obstacle_cost.map(|_|s.trajectory.cost)))).collect::<Vec<_>>().join(",");
        set("candidates", format!("[{candidates}]"));
        set(
            "window",
            format!(
                "{:?}",
                [
                    t.dynamic_window[0],
                    t.dynamic_window[1],
                    t.dynamic_window[2],
                    t.dynamic_window[3]
                ]
            ),
        );
        set(
            "chosen",
            t.selected.map_or("null".into(), |i| i.to_string()),
        );
        set(
            "applied",
            format!("{:?}", [t.applied_control[0], t.applied_control[1]]),
        );
        set("override", u8::from(t.stuck_override).to_string());
        set("state", format!("{:?}", array(self.state())));
        set(
            "path",
            format!("{:?}", self.states.iter().map(array).collect::<Vec<_>>()),
        );
        set("controls", format!("{:?}", self.controls));
        set("obstacles", format!("{:?}", self.obstacles(self.cursor)));
        set("goal", format!("{:?}", self.goal));
        set("probe", format!("{:?}", self.probe));
        set("weights", format!("{:?}", self.policy.weights()));
        for (k, v) in [
            ("cursor", self.cursor),
            ("last", self.last()),
            ("selected", self.selected),
            ("generation", self.generation),
        ] {
            set(k, v.to_string());
        }
        set("case", self.case.slug().into());
        set("policy", self.policy.slug().into());
        set("horizon", self.horizon.to_string());
        set("radius", self.radius.to_string());
        set("dt", DT.to_string());
        set("limit", LIMIT.to_string());
        set("status", self.status().into());
        set("playing", u8::from(self.playing).to_string());
        set("cloud", u8::from(self.cloud).to_string());
        set("edit", self.edit.slug().into());
        set("share", self.share_query());
        for (k, r) in [
            ("map", self.map),
            ("velocity", self.velocity),
            ("bars", self.bars),
            ("timeline", self.timeline),
        ] {
            set(
                k,
                r.map_or("null".into(), |r| {
                    format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
                }),
            );
        }
        set(
            "painted",
            self.painted.map_or("null".into(), |v| format!("{v:?}")),
        );
        set("witnesses", self.witnesses.clone());
    }
}
