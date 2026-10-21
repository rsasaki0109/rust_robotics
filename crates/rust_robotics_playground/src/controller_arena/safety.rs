//! Recorded CBF decisions from the library rollout, with honest constraint checks.
mod view;
use egui::{Context, Rect, Response};
use rust_robotics_control::cbf_safety_filter::{
    simulate_cbf_navigation_with_trace, CbfConvexObstacle2D, CbfNavConfig, CbfNavReport,
    CbfNavStep, CbfSafetyFilter,
};
use std::collections::BTreeMap;

type Point = [f64; 2];
const DT: f64 = 0.05;
const RADIUS: f64 = 0.2;
const GAIN: f64 = 1.5;
const SPEED: f64 = 1.4;
const TOLERANCE: f64 = 0.15;
const STEPS: usize = 400;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Corner,
    HeadOn,
    Gap,
    Trapped,
}
impl Case {
    const ALL: [Self; 4] = [Self::Corner, Self::HeadOn, Self::Gap, Self::Trapped];
    fn slug(self) -> &'static str {
        match self {
            Self::Corner => "corner",
            Self::HeadOn => "headon",
            Self::Gap => "gap",
            Self::Trapped => "trapped",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Corner => "Corner",
            Self::HeadOn => "Head-on",
            Self::Gap => "Gap",
            Self::Trapped => "Trapped start",
        }
    }
    fn start(self) -> Point {
        if self == Self::Trapped {
            [0., 0.]
        } else {
            [-4., 0.]
        }
    }
    fn boxes(self) -> Vec<[f64; 4]> {
        match self {
            Self::Corner => vec![[-0.8, 0.8, 0.05, 1.65]],
            Self::HeadOn => vec![[-0.8, 0.8, -0.8, 0.8]],
            Self::Gap => vec![[-0.6, 0.6, 0.5, 3.], [-0.6, 0.6, -3., -0.5]],
            Self::Trapped => vec![[-1., 1., 0.1, 2.], [-1., 1., -2., -0.1]],
        }
    }
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
struct Run {
    report: CbfNavReport,
    trace: Vec<CbfNavStep>,
    clearances: Vec<f64>,
}
impl Run {
    fn position(&self, at: usize) -> Point {
        self.report.path[at.min(self.report.steps)]
    }
    fn decision(&self, at: usize) -> Option<&CbfNavStep> {
        self.trace.get(at)
    }
}
#[derive(Clone, Copy)]
struct Constraint {
    distance: f64,
    gradient: Point,
    bound: f64,
    nominal_slack: f64,
    applied_slack: f64,
}
struct Decision {
    position: Point,
    nominal: Point,
    applied: Point,
    constraints: Vec<Constraint>,
    feasible: Vec<Point>,
    span: f64,
    has_step: bool,
}

pub(super) struct SafetyLab {
    pub(super) active: bool,
    case: Case,
    start: Point,
    goal: Point,
    boxes: Vec<[f64; 4]>,
    alpha: f64,
    margin: f64,
    filter: CbfSafetyFilter,
    runs: Vec<Run>,
    cursor: usize,
    selected: usize,
    future: bool,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    generation: usize,
    edit_obstacle: bool,
    drag: Option<Point>,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<Rect>,
    velocity: Option<Rect>,
    graph: Option<Rect>,
    painted: Option<(usize, usize, usize, bool)>,
    witnesses: String,
}
impl Default for SafetyLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Corner,
            start: Case::Corner.start(),
            goal: [4., 0.],
            boxes: Case::Corner.boxes(),
            alpha: 2.,
            margin: 0.1,
            filter: CbfSafetyFilter::new(vec![], 2., RADIUS).unwrap(),
            runs: vec![],
            cursor: 0,
            selected: 0,
            future: true,
            playing: false,
            clock: None,
            pending: 0.,
            generation: 0,
            edit_obstacle: false,
            drag: None,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map: None,
            velocity: None,
            graph: None,
            painted: None,
            witnesses: "[]".into(),
        }
    }
}
impl SafetyLab {
    fn last(&self) -> usize {
        self.runs.iter().map(|r| r.report.steps).max().unwrap_or(0)
    }
    fn first(&self) -> usize {
        self.runs[1]
            .trace
            .iter()
            .position(|s| s.intervened)
            .unwrap_or(0)
    }
    fn preset(&mut self, case: Case) {
        self.case = case;
        self.start = case.start();
        self.goal = [4., 0.];
        self.boxes = case.boxes();
        self.selected = 0;
        self.cursor = 0;
        self.record();
    }
    fn record(&mut self) {
        let obstacles = self
            .boxes
            .iter()
            .map(|b| CbfConvexObstacle2D::aabb(b[0], b[1], b[2], b[3]).unwrap())
            .collect();
        self.filter = CbfSafetyFilter::new(obstacles, self.alpha, RADIUS)
            .unwrap()
            .with_margin(self.margin)
            .unwrap();
        let config = CbfNavConfig {
            dt: DT,
            max_steps: STEPS,
            goal_gain: GAIN,
            max_speed: SPEED,
            goal_tolerance: TOLERANCE,
        };
        self.runs = [false, true]
            .into_iter()
            .map(|on| {
                let (report, trace) = simulate_cbf_navigation_with_trace(
                    &self.filter,
                    self.start,
                    self.goal,
                    &config,
                    on,
                )
                .unwrap();
                let clearances = report
                    .path
                    .iter()
                    .map(|&p| self.filter.min_barrier(p))
                    .collect();
                Run {
                    report,
                    trace,
                    clearances,
                }
            })
            .collect();
        self.selected = self.selected.min(self.boxes.len() - 1);
        self.generation += 1;
        self.seek(self.cursor);
    }
    pub(super) fn enter(&mut self) {
        self.active = true;
        self.record();
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "tab") == Some("arena")
            && crate::share::value(q, "arena_view") == Some("safety");
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        self.drag = None;
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|c| Some(c.slug()) == crate::share::value(q, "safety_case"))
            .unwrap_or(Case::Corner);
        self.start = self.case.start();
        self.goal = crate::share::value(q, "safety_goal")
            .and_then(parse_point)
            .unwrap_or([4., 0.]);
        self.boxes = crate::share::value(q, "safety_boxes")
            .and_then(|v| {
                let boxes: Vec<[f64; 4]> = v
                    .split(';')
                    .map(|s| {
                        let p: Vec<f64> = s
                            .split(',')
                            .map(str::parse)
                            .collect::<Result<_, _>>()
                            .ok()?;
                        (p.len() == 4
                            && p.iter().all(|v| v.is_finite() && v.abs() <= 6.)
                            && p[2].abs() <= 4.
                            && p[3].abs() <= 4.
                            && p[1] - p[0] >= 0.1
                            && p[3] - p[2] >= 0.1)
                            .then(|| [p[0], p[1], p[2], p[3]])
                    })
                    .collect::<Option<_>>()?;
                (boxes.len() == self.case.boxes().len()).then_some(boxes)
            })
            .unwrap_or_else(|| self.case.boxes());
        self.alpha = crate::share::value(q, "safety_alpha")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| v.is_finite() && (0.5..=5.).contains(v))
            .unwrap_or(2.);
        self.margin = crate::share::value(q, "safety_margin")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| v.is_finite() && (0.0..=0.45).contains(v))
            .unwrap_or(0.1);
        self.future = crate::share::boolean(q, "safety_future").unwrap_or(true);
        self.selected = crate::share::value(q, "safety_obstacle")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.edit_obstacle = crate::share::value(q, "safety_edit") == Some("obstacle");
        self.cursor = crate::share::value(q, "safety_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.record();
        if crate::share::value(q, "safety_at").is_none()
            && crate::share::value(q, "safety_event") == Some("first")
        {
            self.seek(self.first());
        }
    }
    pub(super) fn share_query(&self) -> String {
        let boxes = self
            .boxes
            .iter()
            .map(|p| format!("{},{},{},{}", p[0], p[1], p[2], p[3]))
            .collect::<Vec<_>>()
            .join(";");
        format!("tab=arena&arena_view=safety&safety_case={}&safety_goal={},{}&safety_boxes={boxes}&safety_alpha={}&safety_margin={}&safety_at={}&safety_obstacle={}&safety_future={}&safety_edit={}",self.case.slug(),self.goal[0],self.goal[1],self.alpha,self.margin,self.cursor,self.selected,u8::from(self.future),if self.edit_obstacle{"obstacle"}else{"goal"})
    }
    fn seek(&mut self, at: usize) {
        self.cursor = at.min(self.last());
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
    }
    fn edit(&mut self, p: Point) {
        if self.edit_obstacle {
            let b = &mut self.boxes[self.selected];
            let half = [(b[1] - b[0]) / 2., (b[3] - b[2]) / 2.];
            let p = [
                p[0].clamp(-6. + half[0], 6. - half[0]),
                p[1].clamp(-4. + half[1], 4. - half[1]),
            ];
            *b = [
                p[0] - half[0],
                p[0] + half[0],
                p[1] - half[1],
                p[1] + half[1],
            ];
        } else {
            self.goal = p;
        }
        self.cursor = 0;
        self.record();
    }
    fn advance(&mut self, now: f64) {
        let dt = self.clock.replace(now).map_or(0., |t| now - t);
        if !self.playing || !(0.0..1.0).contains(&dt) {
            self.pending = 0.;
            return;
        }
        self.pending += dt / DT;
        let n = (self.pending + 1e-9).floor() as usize;
        self.pending = (self.pending - n as f64).max(0.);
        self.cursor = (self.cursor + n).min(self.last());
        if self.cursor == self.last() {
            self.playing = false;
        }
    }
    pub(super) fn begin_frame(&mut self, ctx: &Context, active: bool) {
        self.actions.clear();
        self.hits.clear();
        self.painted = None;
        if active && self.active {
            self.advance(ctx.input(|i| i.time));
        } else {
            self.clock = None;
            self.pending = 0.;
            self.drag = None;
        }
    }
    fn widget(&mut self, key: &'static str, r: &Response) {
        self.actions.insert(key, r.rect.center().into());
        self.hits.insert(key, r.hovered());
    }
    fn ack(&mut self, key: &'static str, ctx: &Context) {
        self.count += 1;
        self.last_action = key;
        ctx.request_repaint();
    }
    pub(super) fn button(&mut self, key: &'static str, r: &Response) -> bool {
        self.widget(key, r);
        if r.clicked() {
            self.ack(key, &r.ctx);
            true
        } else {
            false
        }
    }
    fn decision(&self) -> Decision {
        let run = &self.runs[1];
        let position = run.position(self.cursor);
        let step = run.decision(self.cursor);
        let nominal = step.map_or([0., 0.], |s| s.nominal);
        let applied = step.map_or([0., 0.], |s| s.control);
        let constraints = self
            .filter
            .obstacles
            .iter()
            .map(|o| {
                let (distance, gradient) = o.barrier(position);
                let bound = -self.alpha * (distance - RADIUS - self.margin);
                Constraint {
                    distance,
                    gradient,
                    bound,
                    nominal_slack: dot(gradient, nominal) - bound,
                    applied_slack: dot(gradient, applied) - bound,
                }
            })
            .collect::<Vec<_>>();
        let span = nominal
            .into_iter()
            .chain(applied)
            .map(f64::abs)
            .fold(1.6, f64::max)
            * 1.2;
        let feasible = constraints.iter().fold(
            vec![[-span, -span], [span, -span], [span, span], [-span, span]],
            |poly, c| clip(&poly, c.gradient, c.bound),
        );
        Decision {
            position,
            nominal,
            applied,
            constraints,
            feasible,
            span,
            has_step: step.is_some(),
        }
    }
    fn curves(&self) -> Vec<Vec<f64>> {
        self.runs
            .iter()
            .map(|r| {
                (0..=self.last())
                    .map(|i| r.clearances[i.min(r.report.steps)])
                    .collect()
            })
            .collect()
    }
    pub(super) fn finish(&mut self, ctx: &Context, active: bool) {
        self.revision += 1;
        self.publish(ctx, active && self.active);
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn publish(&self, _ctx: &Context, _active: bool) {}
    #[cfg(target_arch = "wasm32")]
    fn publish(&self, ctx: &Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let actions = self
            .actions
            .iter()
            .map(|(k, v)| format!("\"{k}\":{v:?}"))
            .collect::<Vec<_>>()
            .join(",");
        let hits = self
            .hits
            .iter()
            .map(|(k, v)| format!("\"{k}\":{v}"))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = ctx.input(|i| i.pointer.hover_pos()).map(|p| [p.x, p.y]);
        let press = ctx.input(|i| i.pointer.press_origin()).map(|p| [p.x, p.y]);
        for (k, v) in [
            ("active", u8::from(active).to_string()),
            ("actions", format!("{{{actions}}}")),
            ("hits", format!("{{{hits}}}")),
            ("action-count", self.count.to_string()),
            ("last-action", self.last_action.into()),
            ("ui-revision", self.revision.to_string()),
            (
                "pointer",
                pointer.map_or("null".into(), |p| format!("{p:?}")),
            ),
            ("press", press.map_or("null".into(), |p| format!("{p:?}"))),
            (
                "down",
                u8::from(ctx.input(|i| i.pointer.primary_down())).to_string(),
            ),
            (
                "drag",
                self.drag.map_or("null".into(), |p| format!("{p:?}")),
            ),
        ] {
            let _ = canvas.set_attribute(&format!("data-safety-{k}"), &v);
        }
        if !active || self.runs.is_empty() {
            return;
        }
        let d = self.decision();
        let encode_rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        let constraints=d.constraints.iter().map(|c|format!("{{\"distance\":{},\"gradient\":{:?},\"bound\":{},\"nominal_slack\":{},\"applied_slack\":{}}}",c.distance,c.gradient,c.bound,c.nominal_slack,c.applied_slack)).collect::<Vec<_>>().join(",");
        let runs=self.runs.iter().map(|r| {
            let step=r.decision(self.cursor).map_or("null".into(),|s|format!("{{\"position\":{:?},\"nominal\":{:?},\"control\":{:?},\"next\":{:?},\"intervened\":{}}}",s.position,s.nominal,s.control,s.next,s.intervened));
            let observed_min=r.clearances[..=self.cursor.min(r.report.steps)].iter().copied().fold(f64::INFINITY,f64::min);
            let observed_interventions=r.trace.iter().take(self.cursor.min(r.report.steps)).filter(|s|s.intervened).count();
            format!("{{\"path\":{:?},\"clearances\":{:?},\"steps\":{},\"reached\":{},\"collided\":{},\"interventions\":{},\"minimum\":{},\"position\":{:?},\"decision\":{step},\"observed_min\":{observed_min},\"observed_interventions\":{observed_interventions}}}",r.report.path,r.clearances,r.report.steps,r.report.reached_goal,r.report.collided,r.report.interventions,r.report.min_clearance,r.position(self.cursor))
        }).collect::<Vec<_>>().join(",");
        for (k, v) in [
            ("case", self.case.slug().into()),
            ("start", format!("{:?}", self.start)),
            ("goal", format!("{:?}", self.goal)),
            ("boxes", format!("{:?}", self.boxes)),
            ("alpha", self.alpha.to_string()),
            ("margin", self.margin.to_string()),
            ("radius", RADIUS.to_string()),
            ("dt", DT.to_string()),
            ("limit", STEPS.to_string()),
            ("gain", GAIN.to_string()),
            ("speed", SPEED.to_string()),
            ("tolerance", TOLERANCE.to_string()),
            ("runs", format!("[{runs}]")),
            ("cursor", self.cursor.to_string()),
            ("last", self.last().to_string()),
            ("first", self.first().to_string()),
            ("selected", self.selected.to_string()),
            ("generation", self.generation.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("future", u8::from(self.future).to_string()),
            (
                "edit",
                if self.edit_obstacle {
                    "obstacle"
                } else {
                    "goal"
                }
                .into(),
            ),
            ("constraints", format!("[{constraints}]")),
            ("feasible", format!("{:?}", d.feasible)),
            ("span", d.span.to_string()),
            ("has-step", u8::from(d.has_step).to_string()),
            ("map", encode_rect(self.map)),
            ("velocity", encode_rect(self.velocity)),
            ("graph", encode_rect(self.graph)),
            ("curves", format!("{:?}", self.curves())),
            ("witnesses", self.witnesses.clone()),
            (
                "painted",
                self.painted
                    .map_or("null".into(), |(g, c, s, f)| format!("[{g},{c},{s},{f}]")),
            ),
            ("share", self.share_query()),
        ] {
            let _ = canvas.set_attribute(&format!("data-safety-{k}"), &v);
        }
        let _ = canvas.set_attribute(
            "aria-label",
            &format!(
                "Safety filter lab, {} frame {} of {}, {}",
                self.case.label(),
                self.cursor,
                self.last(),
                if self.playing { "playing" } else { "paused" }
            ),
        );
    }
}
fn parse_point(v: &str) -> Option<Point> {
    let p: Vec<f64> = v
        .split(',')
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    (p.len() == 2 && p.iter().all(|v| v.is_finite()) && p[0].abs() <= 6. && p[1].abs() <= 4.)
        .then(|| [p[0], p[1]])
}
/// Clip a bounded display polygon by a real velocity half-plane, not a speed limit.
fn clip(poly: &[Point], g: Point, b: f64) -> Vec<Point> {
    if poly.is_empty() {
        return vec![];
    }
    let mut out = vec![];
    let mut previous = *poly.last().unwrap();
    let mut ps = dot(g, previous) - b;
    for &current in poly {
        let cs = dot(g, current) - b;
        if (ps >= 0.) != (cs >= 0.) {
            let t = ps / (ps - cs);
            out.push([
                previous[0] + t * (current[0] - previous[0]),
                previous[1] + t * (current[1] - previous[1]),
            ]);
        }
        if cs >= 0. {
            out.push(current);
        }
        previous = current;
        ps = cs;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lab(q: &str) -> SafetyLab {
        let mut s = SafetyLab::default();
        s.apply_query(&format!("tab=arena&arena_view=safety&{q}"));
        s
    }
    fn independent_distance(p: Point, b: [f64; 4]) -> f64 {
        if p[0] >= b[0] && p[0] <= b[1] && p[1] >= b[2] && p[1] <= b[3] {
            -[p[0] - b[0], b[1] - p[0], p[1] - b[2], b[3] - p[1]]
                .into_iter()
                .fold(f64::INFINITY, f64::min)
        } else {
            distance(p, [p[0].clamp(b[0], b[1]), p[1].clamp(b[2], b[3])])
        }
    }
    fn oracle(nominal: Point, constraints: &[Constraint]) -> Option<Point> {
        let mut candidates = vec![nominal];
        for c in constraints {
            let t = c.bound - dot(c.gradient, nominal);
            candidates.push([
                nominal[0] + t * c.gradient[0],
                nominal[1] + t * c.gradient[1],
            ]);
        }
        for (i, a) in constraints.iter().enumerate() {
            for b in constraints.iter().skip(i + 1) {
                let det = a.gradient[0] * b.gradient[1] - a.gradient[1] * b.gradient[0];
                if det.abs() > 1e-10 {
                    candidates.push([
                        (a.bound * b.gradient[1] - a.gradient[1] * b.bound) / det,
                        (a.gradient[0] * b.bound - a.bound * b.gradient[0]) / det,
                    ]);
                }
            }
        }
        candidates
            .into_iter()
            .filter(|&u| {
                constraints
                    .iter()
                    .all(|c| dot(c.gradient, u) >= c.bound - 1e-7)
            })
            .min_by(|&a, &b| distance(a, nominal).total_cmp(&distance(b, nominal)))
    }
    #[test]
    fn corner_avoidance_and_headon_deadlock_share_the_actual_goal_controller() {
        let s = lab("");
        let raw = &s.runs[0];
        let filtered = &s.runs[1];
        assert!(raw.report.collided && raw.report.reached_goal);
        assert!(!filtered.report.collided && filtered.report.reached_goal);
        assert!(filtered.report.min_clearance >= 0.1 - 1e-8);
        let first = s.first();
        assert!(first > 0);
        assert_eq!(raw.report.path[..=first], filtered.report.path[..=first]);
        assert!(filtered.trace[first].intervened);
        assert_ne!(raw.report.path[first + 1], filtered.report.path[first + 1]);
        let head = lab("safety_case=headon");
        let stopped = &head.runs[1];
        assert!(!stopped.report.collided && !stopped.report.reached_goal);
        assert_eq!(stopped.report.steps, STEPS);
        assert!(distance(stopped.trace.last().unwrap().control, [0., 0.]) < 1e-5);
        assert!(distance(*stopped.report.path.last().unwrap(), head.goal) > 4.);
    }
    #[test]
    fn every_recorded_decision_has_true_clearance_and_minimal_feasible_correction() {
        for case in Case::ALL {
            for alpha in [0.8, 2., 5.] {
                let mut s = lab(&format!("safety_case={}&safety_alpha={alpha}", case.slug()));
                for (i, run) in s.runs.iter().enumerate() {
                    assert_eq!(run.trace.len(), run.report.steps);
                    for (k, step) in run.trace.iter().enumerate() {
                        assert_eq!(step.position, run.report.path[k]);
                        assert_eq!(step.next, run.report.path[k + 1]);
                        for a in 0..2 {
                            assert!(
                                (step.position[a] + step.control[a] * DT - step.next[a]).abs()
                                    < 1e-12
                            );
                        }
                        if i == 0 {
                            assert_eq!(step.nominal, step.control);
                            assert!(!step.intervened);
                        }
                    }
                    for (k, &p) in run.report.path.iter().enumerate() {
                        let expected = s
                            .boxes
                            .iter()
                            .map(|&b| independent_distance(p, b) - RADIUS)
                            .fold(f64::INFINITY, f64::min);
                        assert!((run.clearances[k] - expected).abs() < 1e-10);
                    }
                }
                for at in 0..s.runs[1].report.steps {
                    s.cursor = at;
                    let d = s.decision();
                    for (i, c) in d.constraints.iter().enumerate() {
                        assert!(
                            (c.distance - independent_distance(d.position, s.boxes[i])).abs()
                                < 1e-10
                        );
                        assert!((dot(c.gradient, c.gradient) - 1.).abs() < 1e-10);
                        if c.distance > 1e-10 {
                            let b = s.boxes[i];
                            let q = [
                                d.position[0].clamp(b[0], b[1]),
                                d.position[1].clamp(b[2], b[3]),
                            ];
                            for (k, qk) in q.into_iter().enumerate() {
                                assert!(
                                    (c.gradient[k] - (d.position[k] - qk) / c.distance).abs()
                                        < 1e-10
                                );
                            }
                        }
                    }
                    match oracle(d.nominal, &d.constraints) {
                        Some(best) => {
                            assert!(
                                d.constraints.iter().all(|c| c.applied_slack >= -1e-7),
                                "{case:?} alpha{alpha} at{at}"
                            );
                            assert!(
                                (distance(best, d.nominal) - distance(d.applied, d.nominal)).abs()
                                    < 1e-7,
                                "{case:?} alpha{alpha} at{at}"
                            );
                        }
                        None => assert!(d.constraints.iter().any(|c| c.applied_slack < -1e-7)),
                    }
                    for v in d.feasible {
                        assert!(d
                            .constraints
                            .iter()
                            .all(|c| dot(c.gradient, v) >= c.bound - 1e-7));
                    }
                }
            }
        }
    }
    #[test]
    fn gap_margin_and_overlapping_start_expose_the_limits_of_a_reactive_filter() {
        let gap = lab("safety_case=gap");
        assert!(gap.runs[1].report.reached_goal && !gap.runs[1].report.collided);
        let wide = lab("safety_case=gap&safety_margin=0.4");
        assert!(!wide.runs[1].report.reached_goal && !wide.runs[1].report.collided);
        assert_eq!(
            wide.runs[0].report.path, gap.runs[0].report.path,
            "filter settings changed the raw comparison"
        );
        let trapped = lab("safety_case=trapped");
        let d = trapped.decision();
        assert!(trapped.runs[1].clearances[0] < 0.);
        assert!(trapped.runs[1].report.collided);
        assert!(d.feasible.is_empty() && oracle(d.nominal, &d.constraints).is_none());
        assert!(d.constraints.iter().any(|c| c.applied_slack < -0.1));
        assert!(dot(d.constraints[0].gradient, d.constraints[1].gradient) < -0.999999);
        assert!(d.constraints[0].bound + d.constraints[1].bound > 0.);
    }
    #[test]
    fn precise_shares_restore_actual_paths_decisions_and_editing_state_paused() {
        let mut s=lab("safety_case=gap&safety_alpha=1.234567891&safety_margin=0.123456789&safety_goal=4.123456789,0.234567891&safety_at=47&safety_obstacle=1&safety_future=0&safety_edit=obstacle");
        s.playing = true;
        let q = s.share_query();
        let restored = lab(&q);
        assert_eq!(restored.share_query(), q);
        assert!(!restored.playing);
        for (a, b) in s.runs.iter().zip(&restored.runs) {
            assert_eq!(a.report, b.report);
            assert_eq!(a.trace, b.trace);
            assert_eq!(a.clearances, b.clearances);
        }
        let goal = s.goal;
        let start = s.start;
        let b = s.boxes[s.selected];
        let generation = s.generation;
        s.edit([1.123456789, -0.345678912]);
        assert_eq!(s.goal, goal);
        assert_eq!(s.start, start);
        assert_eq!(s.generation, generation + 1);
        assert_eq!(s.cursor, 0);
        assert!(!s.playing);
        let after = s.boxes[s.selected];
        assert!((after[1] - after[0] - (b[1] - b[0])).abs() < 1e-12);
        assert!((after[3] - after[2] - (b[3] - b[2])).abs() < 1e-12);
        let bad=lab("safety_goal=NaN,0&safety_alpha=NaN&safety_margin=9&safety_boxes=0,0,0,0&safety_at=99999&safety_obstacle=99999");
        assert_eq!(bad.goal, [4., 0.]);
        assert_eq!(bad.alpha, 2.);
        assert_eq!(bad.margin, 0.1);
        assert_eq!(bad.boxes, Case::Corner.boxes());
        assert_eq!(bad.cursor, bad.last());
        assert_eq!(bad.selected, 0);
        s.apply_query("tab=arena&arena_frame=5&arena_playing=1");
        assert!(!s.active);
    }
    #[test]
    fn replay_uses_physical_time_and_holds_finished_runs_without_recomputing() {
        for (n, dt) in [(200, 0.01), (50, 0.04), (5, 0.4)] {
            let mut s = lab("");
            let generation = s.generation;
            let path = s.runs[1].report.path.clone();
            s.playing = true;
            s.advance(10.);
            for i in 1..=n {
                s.advance(10. + i as f64 * dt);
            }
            assert_eq!(s.cursor, 40);
            assert_eq!(s.generation, generation);
            assert_eq!(s.runs[1].report.path, path);
            s.seek(7);
            s.advance(300.);
            assert_eq!(s.cursor, 7);
            assert!(!s.playing);
            s.seek(s.last() - 1);
            s.playing = true;
            s.advance(400.);
            s.advance(400.8);
            assert_eq!(s.cursor, s.last());
            assert!(!s.playing);
            for run in &s.runs {
                assert_eq!(run.position(s.cursor), *run.report.path.last().unwrap());
                assert!(run.decision(s.cursor).is_none());
            }
        }
    }
}
