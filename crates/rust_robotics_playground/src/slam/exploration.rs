//! Occlusion-aware frontier navigation using actual library decisions.
mod view;
use egui::{Context, Rect, Response};
use rust_robotics_planning::frontier_navigator::{
    simulate_frontier_navigation_with_trace, FrontierCandidate, FrontierNavConfig,
    FrontierNavPhase, FrontierNavReport, FrontierNavStep, FrontierNavWorld, Knowledge,
};
use std::collections::BTreeMap;
type Cell = (i32, i32);
const WIDTH: usize = 24;
const HEIGHT: usize = 16;
const LIMIT: usize = 64;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Corner,
    Doors,
    Sealed,
    Hall,
}
impl Case {
    const ALL: [Self; 4] = [Self::Corner, Self::Doors, Self::Sealed, Self::Hall];
    fn slug(self) -> &'static str {
        match self {
            Self::Corner => "corner",
            Self::Doors => "doors",
            Self::Sealed => "sealed",
            Self::Hall => "hall",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Corner => "Around a wall",
            Self::Doors => "Two doors",
            Self::Sealed => "Sealed room",
            Self::Hall => "Pillar hall",
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Strategy {
    Balanced,
    Goal,
    Curious,
}
impl Strategy {
    const ALL: [Self; 3] = [Self::Balanced, Self::Goal, Self::Curious];
    fn slug(self) -> &'static str {
        match self {
            Self::Balanced => "balanced",
            Self::Goal => "goal_strategy",
            Self::Curious => "curious",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Balanced => "Balanced",
            Self::Goal => "Goal focused",
            Self::Curious => "More discovery",
        }
    }
    fn weights(self) -> [f64; 4] {
        match self {
            Self::Balanced => [1., 0.35, 1.5, 0.15],
            Self::Goal => [2., 0.35, 1.5, 0.],
            Self::Curious => [0.3, 0.35, 1.5, 5.],
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Edit {
    Inspect,
    Walls,
    Start,
    Goal,
}
impl Edit {
    const ALL: [Self; 4] = [Self::Inspect, Self::Walls, Self::Start, Self::Goal];
    fn slug(self) -> &'static str {
        match self {
            Self::Inspect => "inspect",
            Self::Walls => "walls",
            Self::Start => "edit_start",
            Self::Goal => "edit_goal",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Inspect => "Inspect frontier",
            Self::Walls => "Draw walls",
            Self::Start => "Move start",
            Self::Goal => "Move goal",
        }
    }
}
#[cfg(target_arch = "wasm32")]
fn phase_name(p: FrontierNavPhase) -> &'static str {
    match p {
        FrontierNavPhase::Frontier => "frontier",
        FrontierNavPhase::Goal => "goal",
        FrontierNavPhase::Arrived => "arrived",
        FrontierNavPhase::NoFrontiers => "no_frontiers",
        FrontierNavPhase::NoReachableFrontier => "unreachable",
        FrontierNavPhase::NoProgress => "no_progress",
        FrontierNavPhase::IterationLimit => "iteration_limit",
    }
}
fn point(p: Cell) -> [i32; 2] {
    [p.0, p.1]
}
fn code(k: Knowledge) -> u8 {
    match k {
        Knowledge::Unknown => 0,
        Knowledge::Free => 1,
        Knowledge::Occupied => 2,
    }
}
fn flat(known: &[Vec<Knowledge>]) -> Vec<u8> {
    (0..WIDTH * HEIGHT)
        .map(|i| code(known[i % WIDTH][i / WIDTH]))
        .collect()
}
fn distance(a: Cell, b: Cell) -> f64 {
    f64::from((a.0 - b.0).pow(2) + (a.1 - b.1).pow(2)).sqrt()
}
fn inside(c: Cell) -> bool {
    (1..WIDTH as i32 - 1).contains(&c.0) && (1..HEIGHT as i32 - 1).contains(&c.1)
}
fn line(a: Cell, b: Cell) -> Vec<Cell> {
    let (mut x, mut y) = a;
    let (dx, dy) = ((b.0 - x).abs(), -(b.1 - y).abs());
    let (sx, sy) = ((b.0 - x).signum(), (b.1 - y).signum());
    let mut error = dx + dy;
    let mut cells = vec![a];
    while (x, y) != b {
        let twice = 2 * error;
        if twice >= dy {
            error += dy;
            x += sx;
        }
        if twice <= dx {
            error += dx;
            y += sy;
        }
        cells.push((x, y));
    }
    cells
}
fn world(case: Case) -> FrontierNavWorld {
    let (start, goal) = if case == Case::Corner {
        ((1, 8), (22, 8))
    } else {
        ((2, 8), (21, 8))
    };
    let mut w =
        FrontierNavWorld::new(WIDTH as i32, HEIGHT as i32, start, goal).expect("fixed dimensions");
    w.fill_rect(0, WIDTH as i32 - 1, 0, 0);
    w.fill_rect(0, WIDTH as i32 - 1, HEIGHT as i32 - 1, HEIGHT as i32 - 1);
    w.fill_rect(0, 0, 0, HEIGHT as i32 - 1);
    w.fill_rect(WIDTH as i32 - 1, WIDTH as i32 - 1, 0, HEIGHT as i32 - 1);
    match case {
        Case::Corner => w.fill_rect(12, 13, 1, 12),
        Case::Doors => {
            w.fill_rect(12, 12, 1, 14);
            w.occupied[12][4] = false;
            w.occupied[12][11] = false;
        }
        Case::Sealed => w.fill_rect(12, 12, 0, 15),
        Case::Hall => {
            w.fill_rect(8, 8, 4, 7);
            w.fill_rect(13, 13, 9, 11);
            w.fill_rect(18, 18, 3, 5);
        }
    }
    w
}
pub(super) struct ExploreLab {
    pub(super) active: bool,
    case: Case,
    strategy: Strategy,
    range: f64,
    budget: usize,
    world: FrontierNavWorld,
    report: Option<FrontierNavReport>,
    steps: Vec<FrontierNavStep>,
    coverage: Vec<usize>,
    lengths: Vec<f64>,
    cursor: usize,
    selected: usize,
    truth: bool,
    route: bool,
    edit: Edit,
    note: &'static str,
    generation: usize,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<Rect>,
    graph: Option<Rect>,
    candidates: Option<Rect>,
    components: Option<Rect>,
    drag: Option<(Cell, Cell)>,
    painted: Option<(usize, usize, usize, bool, bool, Edit)>,
    witnesses: String,
}
impl Default for ExploreLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Corner,
            strategy: Strategy::Balanced,
            range: 5.,
            budget: 3,
            world: world(Case::Corner),
            report: None,
            steps: Vec::new(),
            coverage: Vec::new(),
            lengths: Vec::new(),
            cursor: 0,
            selected: 0,
            truth: false,
            route: true,
            edit: Edit::Inspect,
            note: "",
            generation: 0,
            playing: false,
            clock: None,
            pending: 0.,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map: None,
            graph: None,
            candidates: None,
            components: None,
            drag: None,
            painted: None,
            witnesses: "[]".into(),
        }
    }
}
impl ExploreLab {
    fn config(&self) -> FrontierNavConfig {
        let [goal, cost, los, open] = self.strategy.weights();
        FrontierNavConfig {
            sensor_range: self.range,
            step_budget: self.budget,
            max_iterations: LIMIT,
            goal_weight: goal,
            cost_weight: cost,
            line_of_sight_weight: los,
            openness_weight: open,
        }
    }
    fn last(&self) -> usize {
        self.report.as_ref().map_or(0, |r| r.path.len() - 1)
    }
    fn decision_at(&self, at: usize) -> usize {
        self.steps
            .iter()
            .rposition(|s| s.path_index <= at)
            .unwrap_or(0)
    }
    fn decision(&self) -> usize {
        self.decision_at(self.cursor)
    }
    fn step(&self) -> &FrontierNavStep {
        &self.steps[self.decision()]
    }
    fn robot(&self) -> Cell {
        self.report.as_ref().expect("recorded").path[self.cursor]
    }
    fn plan_end(&self) -> usize {
        self.steps
            .get(self.decision() + 1)
            .map_or(self.cursor, |s| s.path_index)
    }
    fn candidate(&self) -> Option<&FrontierCandidate> {
        self.step().candidates.get(self.selected)
    }
    fn terms(&self) -> Option<[f64; 4]> {
        let c = self.candidate()?;
        let cost = c.travel_cost?;
        let [a, b, d, e] = self.strategy.weights();
        Some([
            a * c.goal_gain,
            -b * cost,
            d * if c.line_of_sight { 1. } else { 0. },
            e * c.normalized_openness,
        ])
    }
    fn truth_visible(&self) -> bool {
        self.truth || self.edit == Edit::Walls
    }
    fn first_wall(&self) -> usize {
        self.steps
            .iter()
            .find(|s| {
                s.phase == FrontierNavPhase::Frontier
                    && (1..WIDTH - 1)
                        .any(|x| (1..HEIGHT - 1).any(|y| s.known[x][y] == Knowledge::Occupied))
            })
            .map_or(0, |s| s.path_index)
    }
    fn record(&mut self) {
        let (report, steps) = simulate_frontier_navigation_with_trace(&self.world, &self.config())
            .expect("bounded valid world and config");
        self.report = Some(report);
        self.steps = steps;
        self.coverage = (0..=self.last())
            .map(|at| {
                self.steps[self.decision_at(at)]
                    .known
                    .iter()
                    .flatten()
                    .filter(|k| **k != Knowledge::Unknown)
                    .count()
            })
            .collect();
        self.lengths = vec![0.];
        for pair in self.report.as_ref().unwrap().path.windows(2) {
            self.lengths
                .push(self.lengths.last().unwrap() + distance(pair[0], pair[1]));
        }
        self.generation += 1;
        self.note = "";
        self.seek(0);
    }
    fn preset(&mut self, case: Case) {
        self.case = case;
        self.world = world(case);
        self.record();
    }
    pub(super) fn enter(&mut self) {
        self.active = true;
        if self.report.is_none() {
            self.record();
        }
        self.seek(self.cursor);
    }
    fn seek(&mut self, at: usize) {
        self.cursor = at.min(self.last());
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        self.selected = self.step().selected.unwrap_or(0);
    }
    fn next_decision(&self) -> usize {
        self.steps
            .iter()
            .find(|s| s.path_index > self.cursor)
            .map_or(self.last(), |s| s.path_index)
    }
    fn set_cell(&mut self, c: Cell) {
        let c = (
            c.0.clamp(0, WIDTH as i32 - 1),
            c.1.clamp(0, HEIGHT as i32 - 1),
        );
        if self.edit == Edit::Inspect {
            if let Some((index, _)) =
                self.step()
                    .candidates
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        distance(a.representative, c).total_cmp(&distance(b.representative, c))
                    })
            {
                self.selected = index;
            }
            self.playing = false;
            self.clock = None;
            self.pending = 0.;
            return;
        }
        if self.edit == Edit::Walls {
            self.paint(c, c);
            return;
        }
        if !inside(c) || self.world.occupied[c.0 as usize][c.1 as usize] {
            self.note = "Choose a free cell inside the border.";
            return;
        }
        match self.edit {
            Edit::Start => self.world.start = c,
            Edit::Goal => self.world.goal = c,
            _ => {}
        }
        self.record();
    }
    fn paint(&mut self, a: Cell, b: Cell) {
        let occupied = !self.world.occupied[a.0 as usize][a.1 as usize];
        for c in line(a, b) {
            if inside(c) && c != self.world.start && c != self.world.goal {
                self.world.occupied[c.0 as usize][c.1 as usize] = occupied;
            }
        }
        self.record();
    }
    fn advance(&mut self, now: f64) {
        let elapsed = self.clock.replace(now).map_or(0., |t| now - t);
        if !self.playing || !(0.0..1.0).contains(&elapsed) {
            self.pending = 0.;
            return;
        }
        self.pending += elapsed;
        let mut moved = false;
        while self.pending + 1e-9 >= 0.25 && self.cursor < self.last() {
            self.pending -= 0.25;
            self.cursor += 1;
            moved = true;
        }
        if moved {
            self.selected = self.step().selected.unwrap_or(0);
        }
        if self.cursor == self.last() {
            self.playing = false;
        }
    }
    fn map_bits(&self) -> String {
        (0..WIDTH * HEIGHT)
            .map(|i| {
                if self.world.occupied[i % WIDTH][i / WIDTH] {
                    '1'
                } else {
                    '0'
                }
            })
            .collect()
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "slam_view") == Some("explore");
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        self.drag = None;
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|c| Some(c.slug()) == crate::share::value(q, "explore_case"))
            .unwrap_or(Case::Corner);
        self.world = world(self.case);
        self.strategy = Strategy::ALL
            .into_iter()
            .find(|s| Some(s.slug()) == crate::share::value(q, "explore_strategy"))
            .unwrap_or(Strategy::Balanced);
        self.range = crate::share::value(q, "explore_range")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|v| [3., 5., 7.].contains(v))
            .unwrap_or(5.);
        self.budget = crate::share::value(q, "explore_budget")
            .and_then(|v| v.parse().ok())
            .filter(|v| [1, 3].contains(v))
            .unwrap_or(3);
        let endpoint = |key, fallback| {
            crate::share::value(q, key)
                .and_then(|v| {
                    let p: Vec<i32> = v
                        .split(',')
                        .map(str::parse)
                        .collect::<Result<_, _>>()
                        .ok()?;
                    (p.len() == 2 && inside((p[0], p[1]))).then(|| (p[0], p[1]))
                })
                .unwrap_or(fallback)
        };
        self.world.start = endpoint("explore_start", self.world.start);
        self.world.goal = endpoint("explore_goal", self.world.goal);
        if let Some(bits) = crate::share::value(q, "explore_map")
            .filter(|s| s.len() == WIDTH * HEIGHT && s.bytes().all(|b| b == b'0' || b == b'1'))
        {
            for (i, b) in bits.bytes().enumerate() {
                self.world.occupied[i % WIDTH][i / WIDTH] =
                    !inside(((i % WIDTH) as i32, (i / WIDTH) as i32)) || b == b'1';
            }
        }
        for c in [self.world.start, self.world.goal] {
            self.world.occupied[c.0 as usize][c.1 as usize] = false;
        }
        self.truth = crate::share::boolean(q, "explore_truth").unwrap_or(false);
        self.route = crate::share::boolean(q, "explore_route").unwrap_or(true);
        self.edit = Edit::ALL
            .into_iter()
            .find(|e| Some(e.slug()) == crate::share::value(q, "explore_edit"))
            .unwrap_or(Edit::Inspect);
        self.record();
        let at = crate::share::value(q, "explore_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(if crate::share::value(q, "explore_event") == Some("wall") {
                self.first_wall()
            } else {
                0
            });
        self.seek(at);
        self.selected = crate::share::value(q, "explore_candidate")
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(self.selected)
            .min(self.step().candidates.len().saturating_sub(1));
    }
    pub(super) fn share_query(&self) -> String {
        format!("tab=slam&slam_view=explore&explore_case={}&explore_strategy={}&explore_range={}&explore_budget={}&explore_start={},{}&explore_goal={},{}&explore_map={}&explore_at={}&explore_candidate={}&explore_truth={}&explore_route={}&explore_edit={}",
            self.case.slug(),self.strategy.slug(),self.range,self.budget,self.world.start.0,self.world.start.1,
            self.world.goal.0,self.world.goal.1,self.map_bits(),self.cursor,self.selected,u8::from(self.truth),u8::from(self.route),self.edit.slug())
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
        let set = |key: &str, value: String| {
            let _ = c.set_attribute(&format!("data-explore-{key}"), &value);
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
                    .map(|(k, v)| format!("\"{k}\":{v}"))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set("action-count", self.count.to_string());
        set("last-action", self.last_action.into());
        set("ui-revision", self.revision.to_string());
        for (name, p) in [
            ("pointer", ctx.input(|i| i.pointer.hover_pos())),
            ("press", ctx.input(|i| i.pointer.press_origin())),
        ] {
            set(
                name,
                p.map_or("null".into(), |p| format!("{:?}", [p.x, p.y])),
            );
        }
        set(
            "down",
            u8::from(ctx.input(|i| i.pointer.primary_down())).to_string(),
        );
        set(
            "drag",
            self.drag.map_or("null".into(), |(a, b)| {
                format!("[{:?},{:?}]", point(a), point(b))
            }),
        );
        if !active || self.report.is_none() {
            return;
        }
        let report = self.report.as_ref().unwrap();
        let step = self.step();
        let optional = |v: Option<f64>| v.map_or("null".into(), |v| v.to_string());
        let rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        let steps=self.steps.iter().map(|s| {
            let candidates=s.candidates.iter().map(|a|format!("{{\"rep\":{:?},\"members\":{:?},\"openness\":{},\"normalized\":{},\"gain\":{},\"cost\":{},\"los\":{},\"score\":{}}}",
                point(a.representative),a.members.iter().copied().map(point).collect::<Vec<_>>(),a.openness,a.normalized_openness,a.goal_gain,optional(a.travel_cost),a.line_of_sight,optional(a.score))).collect::<Vec<_>>().join(",");
            format!("{{\"robot\":{:?},\"at\":{},\"known\":{:?},\"candidates\":[{candidates}],\"chosen\":{},\"phase\":\"{}\"}}",
                point(s.robot),s.path_index,flat(&s.known),s.selected.map_or("null".into(),|v|v.to_string()),phase_name(s.phase))
        }).collect::<Vec<_>>().join(",");
        for (key, value) in [
            ("case", self.case.slug().into()),
            ("strategy", self.strategy.slug().into()),
            ("weights", format!("{:?}", self.strategy.weights())),
            ("range", self.range.to_string()),
            ("budget", self.budget.to_string()),
            ("limit", LIMIT.to_string()),
            ("width", WIDTH.to_string()),
            ("height", HEIGHT.to_string()),
            ("start", format!("{:?}", point(self.world.start))),
            ("goal", format!("{:?}", point(self.world.goal))),
            ("world", self.map_bits()),
            (
                "path",
                format!(
                    "{:?}",
                    report.path.iter().copied().map(point).collect::<Vec<_>>()
                ),
            ),
            ("steps", format!("[{steps}]")),
            ("cursor", self.cursor.to_string()),
            ("last", self.last().to_string()),
            ("selected", self.selected.to_string()),
            ("decision", self.decision().to_string()),
            ("robot", format!("{:?}", point(self.robot()))),
            ("known", format!("{:?}", flat(&step.known))),
            ("phase", phase_name(step.phase).into()),
            ("coverage", format!("{:?}", self.coverage)),
            ("lengths", format!("{:?}", self.lengths)),
            ("length", report.path_length.to_string()),
            ("reached", u8::from(report.reached_goal).to_string()),
            ("iterations", report.iterations.to_string()),
            ("first-wall", self.first_wall().to_string()),
            ("generation", self.generation.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("truth", u8::from(self.truth).to_string()),
            ("route", u8::from(self.route).to_string()),
            ("edit", self.edit.slug().into()),
            ("map", rect(self.map)),
            ("graph", rect(self.graph)),
            ("candidates", rect(self.candidates)),
            ("components", rect(self.components)),
            ("witnesses", self.witnesses.clone()),
            (
                "painted",
                self.painted.map_or("null".into(), |(g, t, s, w, r, e)| {
                    format!("[{g},{t},{s},{w},{r},\"{}\"]", e.slug())
                }),
            ),
            ("share", self.share_query()),
        ] {
            set(key, value);
        }
        let _ = c.set_attribute(
            "aria-label",
            &format!(
                "Frontier exploration lab, {}, move {} of {}, {}, {}",
                self.case.label(),
                self.cursor,
                self.last(),
                phase_name(step.phase),
                if self.playing { "playing" } else { "paused" }
            ),
        );
    }
}
#[cfg(test)]
mod tests;
