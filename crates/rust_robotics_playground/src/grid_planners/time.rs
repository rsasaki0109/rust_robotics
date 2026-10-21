//! Inspect actual SIPP paths and half-open cell reservations on one tick clock.
mod view;

use egui::{Context, Rect, Response};
use rust_robotics_planning::sipp::{DynamicObstacle, Interval, SippConfig, SippPlanner};
use std::collections::BTreeMap;

type Cell = [usize; 2];
const WIDTH: usize = 13;
const HEIGHT: usize = 7;
const HORIZON: usize = 24;
const START: Cell = [1, 3];
const GOAL: Cell = [11, 3];
const GATE: Cell = [6, 3];
const TICK_SECONDS: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Case {
    Brief,
    Long,
    Corridor,
    Closed,
}
impl Case {
    const ALL: [Self; 4] = [Self::Brief, Self::Long, Self::Corridor, Self::Closed];
    fn slug(self) -> &'static str {
        match self {
            Self::Brief => "brief",
            Self::Long => "long",
            Self::Corridor => "corridor",
            Self::Closed => "closed",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Brief => "Brief closure",
            Self::Long => "Long closure",
            Self::Corridor => "No detour",
            Self::Closed => "Beyond horizon",
        }
    }
    fn duration(self) -> usize {
        match self {
            Self::Long => 8,
            Self::Closed => 20,
            _ => 3,
        }
    }
    fn map(self) -> Vec<Vec<bool>> {
        let mut map = vec![vec![false; HEIGHT]; WIDTH];
        for (x, col) in map.iter_mut().enumerate() {
            for (y, blocked) in col.iter_mut().enumerate() {
                *blocked = x == 0
                    || x == WIDTH - 1
                    || y == 0
                    || y == HEIGHT - 1
                    || (x == GATE[0]
                        && y != GATE[1]
                        && (y != 1 || matches!(self, Self::Corridor | Self::Closed)));
            }
        }
        map
    }
}

#[derive(Debug, Clone)]
struct Run {
    path: Vec<[usize; 3]>,
    positions: Vec<Cell>,
    arrival: Option<usize>,
    moves: usize,
    waits: usize,
    conflicts: Vec<usize>,
    error: Option<String>,
}
impl Run {
    fn position(&self, t: usize) -> Option<Cell> {
        self.positions.get(t).copied()
    }
    fn observed(&self, t: usize) -> (usize, usize) {
        let end = t.min(self.arrival.unwrap_or(0));
        let moves = self.path.iter().skip(1).take_while(|p| p[2] <= end).count();
        (moves, end - moves)
    }
    fn waiting(&self, t: usize) -> bool {
        self.arrival.is_some_and(|a| t < a) && self.position(t) == self.position(t + 1)
    }
}

#[derive(Debug, Clone)]
pub(super) struct TimeLab {
    pub(super) active: bool,
    case: Case,
    map: Vec<Vec<bool>>,
    close: usize,
    duration: usize,
    runs: Vec<Run>,
    intervals: Vec<Vec<[usize; 2]>>,
    cursor: usize,
    selected: Cell,
    future: bool,
    edit_walls: bool,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    generation: usize,
    drag: Option<Cell>,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map_rect: Option<Rect>,
    band: Option<Rect>,
    graph: Option<Rect>,
    painted: Option<(usize, usize, Cell, bool)>,
    witnesses: String,
}
impl Default for TimeLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Brief,
            map: Case::Brief.map(),
            close: 5,
            duration: 3,
            runs: vec![],
            intervals: vec![],
            cursor: 0,
            selected: GATE,
            future: true,
            edit_walls: false,
            playing: false,
            clock: None,
            pending: 0.,
            generation: 0,
            drag: None,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map_rect: None,
            band: None,
            graph: None,
            painted: None,
            witnesses: String::new(),
        }
    }
}
impl TimeLab {
    pub(super) fn activate(&mut self) {
        self.active = true;
        if self.runs.is_empty() {
            self.record();
        } else {
            self.seek(self.cursor);
        }
    }

    fn reserved(&self, p: Cell, t: usize) -> bool {
        self.map[p[0]][p[1]]
            || t > HORIZON
            || (p == GATE && (self.close..self.close + self.duration).contains(&t))
    }
    fn planner(&self, aware: bool) -> SippPlanner {
        let mut dynamic_obstacles = vec![];
        if aware {
            dynamic_obstacles.push(DynamicObstacle {
                x: GATE[0] as i32,
                y: GATE[1] as i32,
                interval: Interval::new(self.close as u64, (self.close + self.duration) as u64),
            });
        }
        // A finite experiment: no returned arrival may exceed the visible horizon.
        for x in 0..WIDTH {
            for y in 0..HEIGHT {
                dynamic_obstacles.push(DynamicObstacle {
                    x: x as i32,
                    y: y as i32,
                    interval: Interval::new((HORIZON + 1) as u64, u64::MAX),
                });
            }
        }
        SippPlanner::new(SippConfig {
            width: WIDTH as i32,
            height: HEIGHT as i32,
            obstacle_map: self.map.clone(),
            dynamic_obstacles,
            allow_diagonal: false,
        })
        .expect("bounded lab grid")
    }
    fn run(&self, planner: &SippPlanner) -> Run {
        match planner.plan(
            START[0] as i32,
            START[1] as i32,
            GOAL[0] as i32,
            GOAL[1] as i32,
        ) {
            Ok(path) => {
                let path: Vec<_> = path
                    .iter()
                    .map(|p| [p.x as usize, p.y as usize, p.t as usize])
                    .collect();
                let arrival = path.last().expect("SIPP includes goal")[2];
                let mut positions = Vec::with_capacity(HORIZON + 1);
                let mut i = 0;
                for t in 0..=HORIZON {
                    while i + 1 < path.len() && path[i + 1][2] <= t {
                        i += 1;
                    }
                    positions.push([path[i][0], path[i][1]]);
                }
                let conflicts = positions
                    .iter()
                    .enumerate()
                    .filter_map(|(t, &p)| self.reserved(p, t).then_some(t))
                    .collect();
                let moves = path.len() - 1;
                Run {
                    path,
                    positions,
                    arrival: Some(arrival),
                    moves,
                    waits: arrival - moves,
                    conflicts,
                    error: None,
                }
            }
            Err(e) => Run {
                path: vec![],
                positions: vec![],
                arrival: None,
                moves: 0,
                waits: 0,
                conflicts: vec![],
                error: Some(e.to_string()),
            },
        }
    }
    fn record(&mut self) {
        let raw = self.planner(false);
        let aware = self.planner(true);
        self.runs = vec![self.run(&raw), self.run(&aware)];
        self.intervals = (0..HEIGHT)
            .flat_map(|y| (0..WIDTH).map(move |x| (x, y)))
            .map(|(x, y)| {
                aware
                    .get_safe_intervals(x as i32, y as i32)
                    .iter()
                    .map(|i| [i.start as usize, i.end.min((HORIZON + 1) as u64) as usize])
                    .collect()
            })
            .collect();
        self.generation += 1;
        self.seek(self.cursor);
    }
    fn preset(&mut self, case: Case) {
        self.case = case;
        self.map = case.map();
        self.close = 5;
        self.duration = case.duration();
        self.selected = GATE;
        self.cursor = 0;
        self.record();
    }
    fn first(&self) -> usize {
        (0..=HORIZON)
            .find(|&t| self.runs[0].position(t) != self.runs[1].position(t))
            .unwrap_or(0)
    }
    fn seek(&mut self, t: usize) {
        self.cursor = t.min(HORIZON);
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
    }
    fn edit(&mut self, from: Cell, to: Cell) {
        self.selected = to;
        if self.edit_walls {
            let blocked = !self.map[from[0]][from[1]];
            let mut changed = false;
            for (x, y) in super::cells_between((from[0], from[1]), (to[0], to[1])) {
                let p = [x, y];
                if x > 0
                    && x < WIDTH - 1
                    && y > 0
                    && y < HEIGHT - 1
                    && ![START, GOAL, GATE].contains(&p)
                {
                    changed |= self.map[x][y] != blocked;
                    self.map[x][y] = blocked;
                }
            }
            if changed {
                self.cursor = 0;
                self.record();
            }
        }
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "grid_view") == Some("time");
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|c| crate::share::value(q, "time_case") == Some(c.slug()))
            .unwrap_or(Case::Brief);
        self.map = crate::share::value(q, "time_map")
            .and_then(Self::decode_map)
            .unwrap_or_else(|| self.case.map());
        self.close = crate::share::value(q, "time_close")
            .and_then(|v| v.parse().ok())
            .filter(|v| *v <= 12)
            .unwrap_or(5);
        self.duration = crate::share::value(q, "time_duration")
            .and_then(|v| v.parse().ok())
            .filter(|v| (1..=25).contains(v))
            .unwrap_or(self.case.duration());
        self.selected = crate::share::value(q, "time_cell")
            .and_then(|v| {
                let (x, y) = v.split_once(',')?;
                let p = [x.parse().ok()?, y.parse().ok()?];
                (p[0] < WIDTH && p[1] < HEIGHT).then_some(p)
            })
            .unwrap_or(GATE);
        self.cursor = crate::share::value(q, "time_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.future = crate::share::boolean(q, "time_future").unwrap_or(true);
        self.edit_walls = crate::share::value(q, "time_edit") == Some("walls");
        self.record();
        if crate::share::value(q, "time_at").is_none()
            && crate::share::value(q, "time_event") == Some("first")
        {
            self.seek(self.first());
        }
    }
    fn encode_map(&self) -> String {
        (0..WIDTH * HEIGHT)
            .map(|i| {
                if self.map[i % WIDTH][i / WIDTH] {
                    '1'
                } else {
                    '0'
                }
            })
            .collect()
    }
    fn decode_map(v: &str) -> Option<Vec<Vec<bool>>> {
        if v.len() != WIDTH * HEIGHT || v.bytes().any(|b| b != b'0' && b != b'1') {
            return None;
        }
        let mut map = vec![vec![false; HEIGHT]; WIDTH];
        for (i, b) in v.bytes().enumerate() {
            map[i % WIDTH][i / WIDTH] = b == b'1';
        }
        if [START, GOAL, GATE].into_iter().any(|p| map[p[0]][p[1]]) {
            return None;
        }
        for (x, col) in map.iter().enumerate() {
            for (y, &b) in col.iter().enumerate() {
                if (x == 0 || x == WIDTH - 1 || y == 0 || y == HEIGHT - 1) && !b {
                    return None;
                }
            }
        }
        Some(map)
    }
    pub(super) fn share_query(&self) -> String {
        format!("tab=grid&grid_view=time&time_case={}&time_close={}&time_duration={}&time_map={}&time_at={}&time_cell={},{}&time_future={}&time_edit={}",
            self.case.slug(),self.close,self.duration,self.encode_map(),self.cursor,self.selected[0],self.selected[1],u8::from(self.future),if self.edit_walls {"walls"} else {"select"})
    }
    fn advance(&mut self, now: f64) {
        let dt = self.clock.replace(now).map_or(0., |p| now - p);
        if !self.playing || !(0.0..1.0).contains(&dt) {
            self.pending = 0.;
            return;
        }
        self.pending += dt / TICK_SECONDS;
        let n = (self.pending + 1e-9).floor() as usize;
        self.pending = (self.pending - n as f64).max(0.);
        self.cursor = (self.cursor + n).min(HORIZON);
        if self.cursor == HORIZON {
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
        let pointer = ctx.input(|i| {
            i.pointer
                .latest_pos()
                .map(|p| format!("{:?}", [p.x, p.y]))
                .unwrap_or("null".into())
        });
        let press = ctx.input(|i| {
            i.pointer
                .press_origin()
                .map(|p| format!("{:?}", [p.x, p.y]))
                .unwrap_or("null".into())
        });
        for (k, v) in [
            ("active", u8::from(active).to_string()),
            ("actions", format!("{{{actions}}}")),
            ("hits", format!("{{{hits}}}")),
            ("action-count", self.count.to_string()),
            ("last-action", self.last_action.into()),
            ("ui-revision", self.revision.to_string()),
            ("pointer", pointer),
            ("press", press),
            (
                "down",
                u8::from(ctx.input(|i| i.pointer.primary_down())).to_string(),
            ),
            (
                "drag",
                self.drag.map_or("null".into(), |p| format!("{p:?}")),
            ),
        ] {
            let _ = canvas.set_attribute(&format!("data-time-{k}"), &v);
        }
        if !active || self.runs.is_empty() {
            return;
        }
        let rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        let runs=self.runs.iter().map(|r|{
            let arrival=r.arrival.map_or("null".into(),|n|n.to_string());
            let position=r.position(self.cursor).map_or("null".into(),|p|format!("{p:?}"));
            let error=r.error.as_ref().map_or("null".into(),|e|format!("{e:?}"));
            let (observed_moves,observed_waits)=r.observed(self.cursor);
            format!("{{\"path\":{:?},\"positions\":{:?},\"arrival\":{arrival},\"moves\":{},\"waits\":{},\"conflicts\":{:?},\"position\":{position},\"waiting\":{},\"observed_moves\":{observed_moves},\"observed_waits\":{observed_waits},\"error\":{error}}}",r.path,r.positions,r.moves,r.waits,r.conflicts,r.waiting(self.cursor))
        }).collect::<Vec<_>>().join(",");
        let walls: Vec<_> = (0..HEIGHT)
            .flat_map(|y| (0..WIDTH).map(move |x| [x, y]))
            .filter(|p| self.map[p[0]][p[1]])
            .collect();
        for (k, v) in [
            ("case", self.case.slug().into()),
            ("width", WIDTH.to_string()),
            ("height", HEIGHT.to_string()),
            ("horizon", HORIZON.to_string()),
            ("tick-seconds", TICK_SECONDS.to_string()),
            ("start", format!("{START:?}")),
            ("goal", format!("{GOAL:?}")),
            ("gate", format!("{GATE:?}")),
            ("close", self.close.to_string()),
            ("duration", self.duration.to_string()),
            ("walls", format!("{walls:?}")),
            ("intervals", format!("{:?}", self.intervals)),
            ("runs", format!("[{runs}]")),
            ("cursor", self.cursor.to_string()),
            ("first", self.first().to_string()),
            ("selected", format!("{:?}", self.selected)),
            ("future", u8::from(self.future).to_string()),
            (
                "edit",
                if self.edit_walls { "walls" } else { "select" }.into(),
            ),
            ("playing", u8::from(self.playing).to_string()),
            ("generation", self.generation.to_string()),
            ("map", rect(self.map_rect)),
            ("band", rect(self.band)),
            ("graph", rect(self.graph)),
            (
                "painted",
                self.painted
                    .map_or("null".into(), |(g, t, p, f)| format!("[{g},{t},{p:?},{f}]")),
            ),
            ("witnesses", format!("[{}]", self.witnesses)),
            ("share", self.share_query()),
        ] {
            let _ = canvas.set_attribute(&format!("data-time-{k}"), &v);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashSet, VecDeque};
    fn lab(q: &str) -> TimeLab {
        let mut s = TimeLab::default();
        s.apply_query(&format!("grid_view=time&{q}"));
        s
    }
    fn oracle(s: &TimeLab, aware: bool) -> Option<usize> {
        let mut seen = HashSet::from([(START, 0)]);
        let mut queue = VecDeque::from([(START, 0)]);
        while let Some((p, t)) = queue.pop_front() {
            if p == GOAL {
                return Some(t);
            }
            if t == HORIZON {
                continue;
            }
            for (dx, dy) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (x, y) = (p[0] as i32 + dx, p[1] as i32 + dy);
                if x < 0 || y < 0 || x >= WIDTH as i32 || y >= HEIGHT as i32 {
                    continue;
                }
                let n = [x as usize, y as usize];
                let nt = t + 1;
                let blocked = s.map[n[0]][n[1]]
                    || (aware && n == GATE && nt >= s.close && nt < s.close + s.duration);
                if !blocked && seen.insert((n, nt)) {
                    queue.push_back((n, nt));
                }
            }
        }
        None
    }
    #[test]
    fn short_closure_waits_long_closure_detours_and_horizon_can_prevent_arrival() {
        let brief = lab("");
        let b = &brief.runs[1];
        assert_eq!(brief.runs[0].arrival, Some(10));
        assert_eq!(brief.runs[0].conflicts, vec![5]);
        assert_eq!((b.arrival, b.moves, b.waits), (Some(13), 10, 3));
        assert!(b.conflicts.is_empty());
        assert!(b.waiting(brief.first()));
        assert_eq!(brief.first(), 5);
        let long = lab("time_case=long");
        assert_eq!(
            (long.runs[1].arrival, long.runs[1].moves, long.runs[1].waits),
            (Some(14), 14, 0)
        );
        assert!(long.runs[1].path.iter().any(|p| p[1] == 1));
        let corridor = lab("time_case=corridor");
        assert_eq!(corridor.runs[1].arrival, Some(13));
        let closed = lab("time_case=closed");
        assert_eq!(closed.runs[1].arrival, None);
        assert!(closed.runs[1].path.is_empty() && closed.runs[1].positions.is_empty());
        assert!(closed.runs[1].error.is_some());
    }
    #[test]
    fn compressed_sipp_matches_independent_full_spacetime_search_and_every_implicit_wait() {
        for case in Case::ALL {
            for close in [0, 3, 5, 9, 12] {
                for duration in [1, 3, 8, 20, 25] {
                    let s = lab(&format!(
                        "time_case={}&time_close={close}&time_duration={duration}",
                        case.slug()
                    ));
                    for (i, r) in s.runs.iter().enumerate() {
                        assert_eq!(r.arrival, oracle(&s, i == 1));
                        if let Some(arrival) = r.arrival {
                            assert_eq!(r.path.first(), Some(&[START[0], START[1], 0]));
                            assert_eq!(r.path.last(), Some(&[GOAL[0], GOAL[1], arrival]));
                            for w in r.path.windows(2) {
                                assert_eq!(
                                    w[0][0].abs_diff(w[1][0]) + w[0][1].abs_diff(w[1][1]),
                                    1
                                );
                                assert!(w[1][2] > w[0][2]);
                                for t in w[0][2]..w[1][2] {
                                    assert_eq!(r.positions[t], [w[0][0], w[0][1]]);
                                }
                            }
                            assert_eq!(r.positions.len(), HORIZON + 1);
                            assert_eq!(r.moves + r.waits, arrival);
                            for t in 0..=HORIZON {
                                assert!(!s.map[r.positions[t][0]][r.positions[t][1]]);
                                if i == 1 {
                                    assert!(!s.reserved(r.positions[t], t));
                                }
                                let (moves, waits) = r.observed(t);
                                assert_eq!(moves + waits, t.min(arrival));
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn displayed_safe_intervals_are_exact_half_open_complements_including_static_walls() {
        let s = lab("");
        assert_eq!(
            s.intervals[GATE[1] * WIDTH + GATE[0]],
            vec![[0, 5], [8, 25]]
        );
        assert!(!s.reserved(GATE, 4));
        assert!(s.reserved(GATE, 5));
        assert!(s.reserved(GATE, 7));
        assert!(!s.reserved(GATE, 8));
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                for t in 0..=HORIZON {
                    let intervals = &s.intervals[y * WIDTH + x];
                    assert_eq!(
                        intervals.iter().any(|p| p[0] <= t && t < p[1]),
                        !s.reserved([x, y], t)
                    );
                }
            }
        }
    }
    #[test]
    fn map_edits_and_exact_paused_shares_preserve_inputs_paths_and_intervals() {
        let mut s = lab("time_at=7&time_cell=4,3&time_future=0&time_edit=walls");
        s.edit([6, 1], [6, 1]);
        assert!(s.map[6][1]);
        assert_eq!(s.runs[1].waits, 3);
        s.edit(START, START);
        assert!(!s.map[START[0]][START[1]]);
        s.edit_walls = false;
        s.edit([4, 3], [4, 3]);
        s.seek(6);
        s.active = false;
        let generation = s.generation;
        s.activate();
        assert_eq!(s.generation, generation);
        assert_eq!(s.cursor, 6);
        assert!(s.map[6][1]);
        s.playing = true;
        let restored = lab(&s.share_query());
        assert_eq!(restored.map, s.map);
        assert_eq!(restored.intervals, s.intervals);
        assert_eq!(restored.runs[1].path, s.runs[1].path);
        assert_eq!(restored.runs[0].positions, s.runs[0].positions);
        assert_eq!(restored.cursor, 6);
        assert_eq!(restored.selected, [4, 3]);
        assert!(!restored.playing && !restored.future && !restored.edit_walls);
        let invalid=lab("time_case=unknown&time_map=1&time_close=999&time_duration=0&time_at=999&time_cell=99,2");
        assert_eq!(invalid.case, Case::Brief);
        assert_eq!(invalid.map, Case::Brief.map());
        assert_eq!(
            (
                invalid.close,
                invalid.duration,
                invalid.cursor,
                invalid.selected
            ),
            (5, 3, HORIZON, GATE)
        );
        let mut legacy = TimeLab::default();
        legacy.apply_query("tab=grid&planner=astar");
        assert!(!legacy.active && legacy.runs.is_empty());
    }
    #[test]
    fn playback_uses_physical_ticks_at_slow_and_fast_paints_without_replanning() {
        for dt in [0.01, 0.04, 0.4] {
            let mut s = lab("");
            s.playing = true;
            let generation = s.generation;
            s.advance(0.);
            for i in 1..=(4. / dt) as usize {
                s.advance(i as f64 * dt);
            }
            assert_eq!(s.cursor, 8);
            assert_eq!(s.generation, generation);
            s.seek(4);
            assert!(!s.playing);
            s.advance(20.);
            assert_eq!(s.cursor, 4);
            s.playing = true;
            s.advance(100.);
            assert_eq!(s.cursor, 4);
            s.seek(HORIZON);
            assert_eq!(s.runs[1].position(HORIZON), Some(GOAL));
            assert!(!s.runs[1].waiting(HORIZON));
        }
    }
}
