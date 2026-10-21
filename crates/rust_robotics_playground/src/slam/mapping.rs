//! Replay the library's actual sparse-return occupancy updates at known poses.
mod view;

use egui::{Context, Rect, Response};
use nalgebra::Vector2;
use rust_robotics_core::Pose2D;
use rust_robotics_slam::lidar_occupancy::{
    CellState, OccupancyCellUpdate, OccupancyConfig, OccupancyEvidence, OccupancyGrid,
};
use std::collections::BTreeMap;

type Point = [f64; 2];
const WIDTH: usize = 24;
const HEIGHT: usize = 16;
const LAST: usize = 20;
const RESOLUTION: f64 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Repeat,
    Clear,
    Absent,
    Conflict,
}
impl Case {
    const ALL: [Self; 4] = [Self::Repeat, Self::Clear, Self::Absent, Self::Conflict];
    fn slug(self) -> &'static str {
        match self {
            Self::Repeat => "repeat",
            Self::Clear => "clear",
            Self::Absent => "absent",
            Self::Conflict => "conflict",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Repeat => "Repeat returns",
            Self::Clear => "Farther returns",
            Self::Absent => "No observations",
            Self::Conflict => "Hit wins",
        }
    }
    fn switches(self) -> bool {
        matches!(self, Self::Clear | Self::Absent)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Edit {
    Inspect,
    Sensor,
    Return,
}
impl Edit {
    const ALL: [Self; 3] = [Self::Inspect, Self::Sensor, Self::Return];
    fn slug(self) -> &'static str {
        match self {
            Self::Inspect => "inspect",
            Self::Sensor => "sensor",
            Self::Return => "return",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Inspect => "Inspect cell",
            Self::Sensor => "Move sensor",
            Self::Return => "Move returns",
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
struct Frame {
    points: Vec<Point>,
    updates: Vec<OccupancyCellUpdate>,
    values: Vec<f32>,
    states: Vec<u8>,
    seen: Vec<usize>,
}
fn probability(value: f32) -> f64 {
    1. / (1. + (-f64::from(value)).exp())
}
fn cell(point: Point) -> [usize; 2] {
    [
        (point[0] / RESOLUTION).floor() as usize,
        (point[1] / RESOLUTION).floor() as usize,
    ]
}
fn bounded(point: Point, edit: Edit) -> Point {
    match edit {
        Edit::Sensor => [point[0].clamp(0.75, 3.25), point[1].clamp(0.75, 7.25)],
        Edit::Return => [point[0].clamp(4.25, 8.25), point[1].clamp(2.25, 5.75)],
        Edit::Inspect => [point[0].clamp(0., 11.999999), point[1].clamp(0., 7.999999)],
    }
}
pub(super) struct MappingLab {
    pub(super) active: bool,
    case: Case,
    sensor: Point,
    target: Point,
    hit: f32,
    miss: f32,
    duplicates: bool,
    future: bool,
    rays: bool,
    edit: Edit,
    selected: [usize; 2],
    frames: Vec<Frame>,
    cursor: usize,
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
    drag: Option<Point>,
    painted: Option<(usize, usize, [usize; 2], bool, bool)>,
    witnesses: String,
}
impl Default for MappingLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Clear,
            sensor: [2.25, 4.25],
            target: [6.25, 4.25],
            hit: 0.9,
            miss: -0.4,
            duplicates: false,
            future: true,
            rays: true,
            edit: Edit::Inspect,
            selected: [12, 8],
            frames: Vec::new(),
            cursor: 0,
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
            drag: None,
            painted: None,
            witnesses: "[]".into(),
        }
    }
}
impl MappingLab {
    fn frame(&self) -> &Frame {
        &self.frames[self.cursor]
    }
    fn index(&self) -> usize {
        self.selected[1] * WIDTH + self.selected[0]
    }
    fn packet(&self, batch: usize) -> Vec<Point> {
        if batch == 0 || (self.case == Case::Absent && batch > 4) {
            return Vec::new();
        }
        let [x, y] = self.target;
        let mut points = if self.case == Case::Conflict {
            vec![[x, y], [x + 3., y + 0.2]]
        } else {
            let shift = if self.case == Case::Clear && batch > 4 {
                3.
            } else {
                0.
            };
            vec![[x + shift, y - 1.], [x + shift, y], [x + shift, y + 1.]]
        };
        if self.duplicates {
            points.extend(points.clone());
        }
        points
    }
    fn record(&mut self) {
        let mut grid = OccupancyGrid::new(
            Vector2::zeros(),
            Vector2::new(12., 8.),
            OccupancyConfig {
                resolution: RESOLUTION,
                hit: self.hit,
                miss: self.miss,
                ..OccupancyConfig::default()
            },
        );
        let mut seen = vec![0; WIDTH * HEIGHT];
        self.frames = (0..=LAST)
            .map(|batch| {
                let points = self.packet(batch);
                let scan: Vec<_> = points
                    .iter()
                    .map(|p| Vector2::new(p[0] - self.sensor[0], p[1] - self.sensor[1]))
                    .collect();
                let updates = grid
                    .insert_scan_with_trace(Pose2D::new(self.sensor[0], self.sensor[1], 0.), &scan);
                for u in &updates {
                    seen[u.y * WIDTH + u.x] += 1;
                }
                Frame {
                    points,
                    updates,
                    values: (0..WIDTH * HEIGHT)
                        .map(|i| grid.log_odds(i % WIDTH, i / WIDTH))
                        .collect(),
                    states: (0..WIDTH * HEIGHT)
                        .map(|i| match grid.state(i % WIDTH, i / WIDTH) {
                            CellState::Unknown => 0,
                            CellState::Free => 1,
                            CellState::Occupied => 2,
                        })
                        .collect(),
                    seen: seen.clone(),
                }
            })
            .collect();
        self.generation += 1;
        self.seek(0);
    }
    pub(super) fn enter(&mut self) {
        self.active = true;
        if self.frames.is_empty() {
            self.record();
        }
        self.seek(self.cursor);
    }
    fn seek(&mut self, at: usize) {
        self.cursor = at.min(LAST);
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
    }
    fn set_point(&mut self, point: Point) {
        let p = bounded(point, self.edit);
        match self.edit {
            Edit::Inspect => {
                self.selected = cell(p);
                self.seek(self.cursor);
            }
            Edit::Sensor => {
                self.sensor = p;
                self.record();
            }
            Edit::Return => {
                self.target = p;
                self.selected = cell(p);
                self.record();
            }
        }
    }
    fn cleared(&self) -> Option<usize> {
        let mut occupied = false;
        self.frames.iter().position(|f| {
            let state = f.states[self.index()];
            occupied |= state == 2;
            occupied && state == 1
        })
    }
    fn stats(&self) -> [usize; 4] {
        let f = self.frame();
        let mut counts = [0; 4]; // unseen, observed but uncertain, free, occupied
        for i in 0..WIDTH * HEIGHT {
            counts[if f.seen[i] == 0 {
                0
            } else {
                match f.states[i] {
                    1 => 2,
                    2 => 3,
                    _ => 1,
                }
            }] += 1;
        }
        counts
    }
    fn advance(&mut self, now: f64) {
        let elapsed = self.clock.replace(now).map_or(0., |t| now - t);
        if !self.playing || !(0.0..1.0).contains(&elapsed) {
            self.pending = 0.;
            return;
        }
        self.pending += elapsed;
        while self.pending + 1e-9 >= 0.5 && self.cursor < LAST {
            self.pending -= 0.5;
            self.cursor += 1;
        }
        if self.cursor == LAST {
            self.playing = false;
        }
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "slam_view") == Some("mapping");
        self.clock = None;
        self.pending = 0.;
        self.playing = false;
        self.drag = None;
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|c| Some(c.slug()) == crate::share::value(q, "mapping_case"))
            .unwrap_or(Case::Clear);
        let point = |key, fallback: Point, edit| {
            crate::share::value(q, key)
                .and_then(|v| {
                    let p: Vec<f64> = v
                        .split(',')
                        .map(str::parse)
                        .collect::<Result<_, _>>()
                        .ok()?;
                    (p.len() == 2 && p.iter().all(|v| v.is_finite()))
                        .then(|| bounded([p[0], p[1]], edit))
                })
                .unwrap_or(fallback)
        };
        self.sensor = point("mapping_sensor", [2.25, 4.25], Edit::Sensor);
        self.target = point("mapping_return", [6.25, 4.25], Edit::Return);
        let gain = |key, options: &[f32], fallback| {
            crate::share::value(q, key)
                .and_then(|v| v.parse::<f32>().ok())
                .filter(|v| options.contains(v))
                .unwrap_or(fallback)
        };
        self.hit = gain("mapping_hit", &[0.4, 0.9, 1.4], 0.9);
        self.miss = gain("mapping_miss", &[-0.2, -0.4, -0.8], -0.4);
        self.duplicates = crate::share::boolean(q, "mapping_duplicates").unwrap_or(false);
        self.future = crate::share::boolean(q, "mapping_future").unwrap_or(true);
        self.rays = crate::share::boolean(q, "mapping_rays").unwrap_or(true);
        self.edit = Edit::ALL
            .into_iter()
            .find(|e| Some(e.slug()) == crate::share::value(q, "mapping_edit"))
            .unwrap_or(Edit::Inspect);
        self.selected = crate::share::value(q, "mapping_cell")
            .and_then(|v| {
                let p: Vec<usize> = v
                    .split(',')
                    .map(str::parse)
                    .collect::<Result<_, _>>()
                    .ok()?;
                (p.len() == 2 && p[0] < WIDTH && p[1] < HEIGHT).then(|| [p[0], p[1]])
            })
            .unwrap_or_else(|| cell(self.target));
        self.record();
        let at = crate::share::value(q, "mapping_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(
                if crate::share::value(q, "mapping_event") == Some("change") && self.case.switches()
                {
                    5
                } else {
                    0
                },
            );
        self.seek(at);
    }
    pub(super) fn share_query(&self) -> String {
        format!("tab=slam&slam_view=mapping&mapping_case={}&mapping_sensor={},{}&mapping_return={},{}&mapping_hit={}&mapping_miss={}&mapping_at={}&mapping_cell={},{}&mapping_future={}&mapping_rays={}&mapping_duplicates={}&mapping_edit={}",
            self.case.slug(),self.sensor[0],self.sensor[1],self.target[0],self.target[1],
            self.hit,self.miss,self.cursor,self.selected[0],self.selected[1],
            u8::from(self.future),u8::from(self.rays),u8::from(self.duplicates),self.edit.slug())
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
            let _ = c.set_attribute(&format!("data-mapping-{key}"), &value);
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
            self.drag.map_or("null".into(), |p| format!("{p:?}")),
        );
        if !active || self.frames.is_empty() {
            return;
        }
        let f = self.frame();
        let rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        let update = |u: &OccupancyCellUpdate| {
            format!(
                "[{},{},{},{},{}]",
                u.x,
                u.y,
                u.evidence == OccupancyEvidence::Hit,
                u.before,
                u.after
            )
        };
        for (k, v) in [
            ("case", self.case.slug().into()),
            ("sensor", format!("{:?}", self.sensor)),
            ("return", format!("{:?}", self.target)),
            ("hit", self.hit.to_string()),
            ("miss", self.miss.to_string()),
            ("clamp", "5".into()),
            ("thresholds", "[-0.3,0.6]".into()),
            ("width", WIDTH.to_string()),
            ("height", HEIGHT.to_string()),
            ("resolution", RESOLUTION.to_string()),
            ("cursor", self.cursor.to_string()),
            ("last", LAST.to_string()),
            ("selected", format!("{:?}", self.selected)),
            ("generation", self.generation.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("future", u8::from(self.future).to_string()),
            ("rays", u8::from(self.rays).to_string()),
            ("duplicates", u8::from(self.duplicates).to_string()),
            ("edit", self.edit.slug().into()),
            ("values", format!("{:?}", f.values)),
            ("states", format!("{:?}", f.states)),
            ("seen", format!("{:?}", f.seen)),
            ("stats", format!("{:?}", self.stats())),
            (
                "probability",
                probability(f.values[self.index()]).to_string(),
            ),
            (
                "history",
                format!(
                    "{:?}",
                    self.frames
                        .iter()
                        .map(|f| f.values[self.index()])
                        .collect::<Vec<_>>()
                ),
            ),
            (
                "packets",
                format!(
                    "{:?}",
                    self.frames.iter().map(|f| &f.points).collect::<Vec<_>>()
                ),
            ),
            (
                "updates",
                format!(
                    "[{}]",
                    self.frames
                        .iter()
                        .map(|f| format!(
                            "[{}]",
                            f.updates.iter().map(update).collect::<Vec<_>>().join(",")
                        ))
                        .collect::<Vec<_>>()
                        .join(",")
                ),
            ),
            (
                "current-update",
                f.updates
                    .iter()
                    .find(|u| [u.x, u.y] == self.selected)
                    .map_or("null".into(), update),
            ),
            (
                "cleared",
                self.cleared().map_or("null".into(), |v| v.to_string()),
            ),
            ("map", rect(self.map)),
            ("graph", rect(self.graph)),
            ("witnesses", self.witnesses.clone()),
            (
                "painted",
                self.painted.map_or("null".into(), |(g, t, c, f, r)| {
                    format!("[{g},{t},{c:?},{f},{r}]")
                }),
            ),
            ("share", self.share_query()),
        ] {
            set(k, v);
        }
        let _ = c.set_attribute(
            "aria-label",
            &format!(
                "Occupancy evidence lab, {}, batch {} of {}, cell {:?}, {}",
                self.case.label(),
                self.cursor,
                LAST,
                self.selected,
                if self.playing { "playing" } else { "paused" }
            ),
        );
    }
}

#[cfg(test)]
mod tests;
