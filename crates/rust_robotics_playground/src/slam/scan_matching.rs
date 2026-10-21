//! Recorded point-to-point ICP, with actual nearest-neighbor associations.
//! The source scan is immutable; initial-pose edits solve a fresh alignment.
mod view;

use egui::{Context, Rect, Response};
use nalgebra::DMatrix;
use rust_robotics_slam::icp_matching::icp_matching_with_trace;
use std::collections::BTreeMap;

type Point = [f64; 2];
type Pose = [f64; 3];
const TRUTH: Pose = [0.75, -0.5, 0.2];
const SPAN: f64 = 14.;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Case {
    Corner,
    Poor,
    Partial,
    Bays,
}

fn apply(pose: Pose, p: Point) -> Point {
    let (s, c) = pose[2].sin_cos();
    [c * p[0] - s * p[1] + pose[0], s * p[0] + c * p[1] + pose[1]]
}
fn inverse(pose: Pose, p: Point) -> Point {
    let (s, c) = pose[2].sin_cos();
    let x = p[0] - pose[0];
    let y = p[1] - pose[1];
    [c * x + s * y, -s * x + c * y]
}
fn matrix(points: &[Point]) -> DMatrix<f64> {
    DMatrix::from_fn(2, points.len(), |i, j| points[j][i])
}
fn clouds(case: Case) -> (Vec<Point>, Vec<Point>) {
    if case == Case::Bays {
        let mut bay = Vec::new();
        for i in 0..=8 {
            bay.push([-0.6 + i as f64 * 0.15, -1.1]);
        }
        for i in 1..=8 {
            bay.push([-0.6, -1.1 + i as f64 * 0.15]);
        }
        for i in 1..=5 {
            bay.push([-0.6 + i as f64 * 0.15, 0.1]);
        }
        let target = [-3., 0., 3.]
            .into_iter()
            .flat_map(|x| bay.iter().map(move |p| [p[0] + x, p[1]]))
            .collect();
        return (bay.iter().map(|&p| inverse(TRUTH, p)).collect(), target);
    }
    let mut target = Vec::new();
    for i in 0..=20 {
        target.push([-4. + i as f64 * 0.35, -2.5]);
    }
    for i in 1..=17 {
        target.push([-4., -2.5 + i as f64 * 0.35]);
    }
    for i in 0..=10 {
        target.push([-1.8 + i as f64 * 0.35, 2.8]);
    }
    for i in 0..10 {
        let a = i as f64 * std::f64::consts::TAU / 10.;
        target.push([2. + 0.55 * a.cos(), 0.6 + 0.35 * a.sin()]);
    }
    let source = target
        .iter()
        .enumerate()
        .filter(|(i, _)| case != Case::Partial || !(38..49).contains(i))
        .map(|(_, &p)| inverse(TRUTH, p))
        .collect();
    (source, target)
}
impl ScanLab {
    fn last(&self) -> usize {
        self.frames.len().saturating_sub(1)
    }
    fn frame(&self) -> &Frame {
        &self.frames[self.cursor]
    }
    fn preset(&mut self, case: Case) {
        self.case = case;
        self.initial = case.initial();
        self.cursor = 0;
        self.selected = 0;
        self.record();
    }
    fn record(&mut self) {
        (self.source, self.target) = clouds(self.case);
        let input: Vec<_> = self
            .source
            .iter()
            .map(|&p| apply(self.initial, p))
            .collect();
        let (result, trace) = icp_matching_with_trace(&matrix(&self.target), &matrix(&input));
        let (s, c) = self.initial[2].sin_cos();
        let h = DMatrix::from_row_slice(
            3,
            3,
            &[c, -s, self.initial[0], s, c, self.initial[1], 0., 0., 1.],
        );
        self.frames = trace
            .into_iter()
            .map(|f| {
                let total = &f.transform * &h;
                Frame {
                    points: (0..f.points.ncols())
                        .map(|j| [f.points[(0, j)], f.points[(1, j)]])
                        .collect(),
                    pairs: f.correspondences,
                    pose: [
                        total[(0, 2)],
                        total[(1, 2)],
                        total[(1, 0)].atan2(total[(0, 0)]),
                    ],
                    mean: f.mean_error,
                }
            })
            .collect();
        self.converged = result.converged;
        self.assessments = result.iterations;
        self.selected = self.selected.min(self.source.len() - 1);
        self.generation += 1;
        self.seek(self.cursor);
    }
    pub(super) fn enter(&mut self) {
        self.active = true;
        self.record();
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "algorithm") == Some("icp")
            && crate::share::value(q, "icp_view") == Some("inspect");
        self.clock = None;
        self.pending = 0.;
        self.playing = false;
        self.drag = None;
        if !self.active {
            return;
        }
        self.case = Case::ALL
            .into_iter()
            .find(|c| Some(c.slug()) == crate::share::value(q, "icp_case"))
            .unwrap_or(Case::Corner);
        self.initial = crate::share::value(q, "icp_initial")
            .and_then(|v| {
                let p: Vec<f64> = v
                    .split(',')
                    .map(str::parse)
                    .collect::<Result<_, _>>()
                    .ok()?;
                (p.len() == 3
                    && p.iter().all(|v| v.is_finite())
                    && p[0].abs() <= 7.
                    && p[1].abs() <= 7.
                    && p[2].abs() <= std::f64::consts::PI)
                    .then(|| [p[0], p[1], p[2]])
            })
            .unwrap_or_else(|| self.case.initial());
        self.cursor = crate::share::value(q, "icp_at")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.selected = crate::share::value(q, "icp_point")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        self.pairs = crate::share::boolean(q, "icp_pairs").unwrap_or(true);
        self.ghost = crate::share::boolean(q, "icp_ghost").unwrap_or(true);
        self.record();
    }
    pub(super) fn share_query(&self) -> String {
        format!("tab=slam&algorithm=icp&icp_view=inspect&icp_case={}&icp_initial={},{},{}&icp_at={}&icp_point={}&icp_pairs={}&icp_ghost={}",self.case.slug(),self.initial[0],self.initial[1],self.initial[2],self.cursor,self.selected,u8::from(self.pairs),u8::from(self.ghost))
    }
    fn seek(&mut self, at: usize) {
        self.cursor = at.min(self.last());
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
    }
    fn edit(&mut self, pose: Pose) {
        self.initial = pose;
        self.cursor = 0;
        self.record();
    }
    fn advance(&mut self, now: f64) {
        let elapsed = self.clock.replace(now).map_or(0., |t| now - t);
        if !self.playing || !(0.0..1.0).contains(&elapsed) {
            self.pending = 0.;
            return;
        }
        self.pending += elapsed * 2.;
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
        let actions = self
            .actions
            .iter()
            .map(|(k, p)| format!("\"{k}\":{p:?}"))
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
            let _ = c.set_attribute(&format!("data-icp-{k}"), &v);
        }
        if !active || self.frames.is_empty() {
            return;
        }
        let f = self.frame();
        let rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        for (k, v) in [
            ("case", self.case.slug().into()),
            ("initial", format!("{:?}", self.initial)),
            ("truth", format!("{TRUTH:?}")),
            ("source", format!("{:?}", self.source)),
            ("target", format!("{:?}", self.target)),
            ("points", format!("{:?}", f.points)),
            ("pairs", format!("{:?}", f.pairs)),
            ("pose", format!("{:?}", f.pose)),
            ("mean", f.mean.to_string()),
            ("cursor", self.cursor.to_string()),
            ("last", self.last().to_string()),
            ("selected", self.selected.to_string()),
            ("generation", self.generation.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("show-pairs", u8::from(self.pairs).to_string()),
            ("ghost", u8::from(self.ghost).to_string()),
            ("converged", u8::from(self.converged).to_string()),
            ("assessments", self.assessments.to_string()),
            (
                "errors",
                format!(
                    "{:?}",
                    self.frames.iter().map(|f| f.mean).collect::<Vec<_>>()
                ),
            ),
            ("map", rect(self.map)),
            ("graph", rect(self.charts[0])),
            ("residuals", rect(self.charts[1])),
            ("witnesses", self.witnesses.clone()),
            ("bars", self.bars.clone()),
            (
                "painted",
                self.painted.map_or("null".into(), |(g, f, p, c, h)| {
                    format!("[{g},{f},{p},{c},{h}]")
                }),
            ),
            ("share", self.share_query()),
        ] {
            let _ = c.set_attribute(&format!("data-icp-{k}"), &v);
        }
        let _ = c.set_attribute(
            "aria-label",
            &format!(
                "Scan matching lab, {} iteration {} of {}, point {}, {}",
                self.case.label(),
                self.cursor,
                self.last(),
                self.selected,
                if self.playing { "playing" } else { "paused" }
            ),
        );
    }
}

impl Case {
    const ALL: [Self; 4] = [Self::Corner, Self::Poor, Self::Partial, Self::Bays];
    fn slug(self) -> &'static str {
        match self {
            Self::Corner => "corner",
            Self::Poor => "poor",
            Self::Partial => "partial",
            Self::Bays => "bays",
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Corner => "Corner",
            Self::Poor => "Poor guess",
            Self::Partial => "Partial scan",
            Self::Bays => "Repeated bays",
        }
    }
    fn initial(self) -> Pose {
        match self {
            Self::Poor => [0., 0., 0.],
            Self::Bays => [3.75, -0.5, 0.2],
            _ => [0.6, -0.4, 0.25],
        }
    }
}

struct Frame {
    points: Vec<Point>,
    pairs: Vec<usize>,
    pose: Pose,
    mean: f64,
}
pub(super) struct ScanLab {
    pub(super) active: bool,
    case: Case,
    initial: Pose,
    source: Vec<Point>,
    target: Vec<Point>,
    frames: Vec<Frame>,
    cursor: usize,
    selected: usize,
    pairs: bool,
    ghost: bool,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    converged: bool,
    assessments: usize,
    generation: usize,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<Rect>,
    charts: [Option<Rect>; 2],
    drag: Option<Pose>,
    painted: Option<(usize, usize, usize, bool, bool)>,
    witnesses: String,
    bars: String,
}
impl Default for ScanLab {
    fn default() -> Self {
        Self {
            active: false,
            case: Case::Corner,
            initial: Case::Corner.initial(),
            source: Vec::new(),
            target: Vec::new(),
            frames: Vec::new(),
            cursor: 0,
            selected: 0,
            pairs: true,
            ghost: true,
            playing: false,
            clock: None,
            pending: 0.,
            converged: false,
            assessments: 0,
            generation: 0,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map: None,
            charts: [None, None],
            drag: None,
            painted: None,
            witnesses: "[]".into(),
            bars: "[]".into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lab(q: &str) -> ScanLab {
        let mut s = ScanLab::default();
        s.apply_query(&format!("algorithm=icp&icp_view=inspect&{q}"));
        s
    }
    fn distance(a: Point, b: Point) -> f64 {
        (a[0] - b[0]).hypot(a[1] - b[1])
    }

    #[test]
    fn corner_recovers_the_known_alignment_from_a_different_initial_pose() {
        let s = lab("");
        let first = &s.frames[0];
        let last = s.frames.last().unwrap();
        assert!(s.frames.len() > 4);
        assert_eq!(
            first.points,
            s.source
                .iter()
                .map(|&p| apply(s.initial, p))
                .collect::<Vec<_>>()
        );
        assert!(last.mean < first.mean * 0.1);
        assert!(
            distance([last.pose[0], last.pose[1]], [TRUTH[0], TRUTH[1]]) < 0.04,
            "{:?}",
            last.pose
        );
        assert!((last.pose[2] - TRUTH[2]).abs() < 0.02);
        // Retain the failed origin guess: ICP is local even on this corner.
        let poor = lab("icp_case=poor");
        let stopped = poor.frames.last().unwrap();
        assert_eq!(poor.source, s.source);
        assert_eq!(poor.target, s.target);
        assert_eq!(poor.initial, [0., 0., 0.]);
        assert!(!poor.converged);
        assert!(stopped.mean > 0.2);
        assert!(stopped.mean < poor.frames[0].mean);
        assert!(distance([stopped.pose[0], stopped.pose[1]], [TRUTH[0], TRUTH[1]]) > 0.3);
    }

    #[test]
    fn every_frame_is_a_rigid_transform_with_the_actual_nearest_neighbors() {
        for case in Case::ALL {
            let mut s = ScanLab::default();
            s.preset(case);
            let mut previous = f64::INFINITY;
            for f in &s.frames {
                let mut total = 0.;
                for (i, &p) in f.points.iter().enumerate() {
                    assert!(distance(p, apply(f.pose, s.source[i])) < 1e-10);
                    let q = s.target[f.pairs[i]];
                    let d = distance(p, q);
                    let min = s
                        .target
                        .iter()
                        .map(|&q| distance(p, q))
                        .fold(f64::INFINITY, f64::min);
                    assert!((d - min).abs() < 1e-10);
                    total += d;
                }
                assert!((total / s.source.len() as f64 - f.mean).abs() < 1e-10);
                assert!(f.mean <= previous + 0.0001 / s.source.len() as f64);
                previous = f.mean;
            }
            for &p in &s.source {
                assert!(s
                    .target
                    .iter()
                    .any(|&q| distance(apply(TRUTH, p), q) < 1e-10));
            }
            if case == Case::Partial {
                assert!(s.source.len() < s.target.len());
            }
        }
        // Independently solve the first fixed-association rigid fit in 2D.
        let s = lab("");
        let f = &s.frames[0];
        let n = f.points.len() as f64;
        let src = [0, 1].map(|k| f.points.iter().map(|p| p[k]).sum::<f64>() / n);
        let dst = [0, 1].map(|k| f.pairs.iter().map(|&j| s.target[j][k]).sum::<f64>() / n);
        let (mut cross, mut dot) = (0., 0.);
        for (p, &j) in f.points.iter().zip(&f.pairs) {
            let q = s.target[j];
            let a = [p[0] - src[0], p[1] - src[1]];
            let b = [q[0] - dst[0], q[1] - dst[1]];
            cross += a[0] * b[1] - a[1] * b[0];
            dot += a[0] * b[0] + a[1] * b[1];
        }
        let yaw = cross.atan2(dot);
        let rotated = apply([0., 0., yaw], src);
        let delta = [dst[0] - rotated[0], dst[1] - rotated[1], yaw];
        for (p, &next) in f.points.iter().zip(&s.frames[1].points) {
            assert!(distance(apply(delta, *p), next) < 1e-10);
        }
    }

    #[test]
    fn repeated_bays_can_converge_with_zero_residual_at_the_wrong_known_position() {
        let s = lab("icp_case=bays");
        let f = s.frames.last().unwrap();
        assert!(s.converged);
        assert!(f.mean < 1e-10);
        assert!((distance([f.pose[0], f.pose[1]], [TRUTH[0], TRUTH[1]]) - 3.).abs() < 1e-10);
        assert!((f.pose[2] - TRUTH[2]).abs() < 1e-10);
        assert!(f.pairs.iter().all(|&j| j >= s.source.len() * 2));
    }

    #[test]
    fn precise_share_restores_the_same_scan_trace_cursor_point_and_overlays_paused() {
        let mut s=lab("icp_case=partial&icp_initial=1.123456789,-0.234567891,0.312345678&icp_at=5&icp_point=12&icp_pairs=0&icp_ghost=0");
        s.playing = true;
        let q = s.share_query();
        let b = lab(&q);
        assert_eq!(b.share_query(), q);
        assert!(!b.playing);
        assert_eq!(b.source, s.source);
        assert_eq!(b.target, s.target);
        assert_eq!(b.frames.len(), s.frames.len());
        for (a, b) in s.frames.iter().zip(&b.frames) {
            assert_eq!(a.points, b.points);
            assert_eq!(a.pairs, b.pairs);
            assert_eq!(a.pose, b.pose);
            assert_eq!(a.mean, b.mean);
        }
        let source = s.source.clone();
        let target = s.target.clone();
        let generation = s.generation;
        s.edit([0.5, -0.2, 0.12]);
        assert_eq!(s.source, source);
        assert_eq!(s.target, target);
        assert_eq!(s.cursor, 0);
        assert_eq!(s.generation, generation + 1);
        assert!(!s.playing);
        let bad = lab("icp_initial=NaN,0,0&icp_point=999&icp_at=999");
        assert_eq!(bad.initial, Case::Corner.initial());
        assert_eq!(bad.cursor, bad.last());
        assert_eq!(bad.selected, bad.source.len() - 1);
        s.apply_query("algorithm=icp&frame=8&playing=1");
        assert!(!s.active, "legacy replay links retain their mode");
    }

    #[test]
    fn replay_and_seeking_only_read_recorded_alignments_at_any_paint_rate() {
        for (n, dt) in [(200, 0.01), (50, 0.04), (5, 0.4)] {
            let mut s = lab("");
            let generation = s.generation;
            let first = s.frames[0].points.clone();
            s.playing = true;
            s.advance(10.);
            for i in 1..=n {
                s.advance(10. + i as f64 * dt);
            }
            assert_eq!(s.cursor, 4.min(s.last()));
            assert_eq!(s.generation, generation);
            assert_eq!(s.frames[0].points, first);
            s.seek(1);
            s.advance(100.);
            assert_eq!(s.cursor, 1);
            assert!(!s.playing);
            s.seek(s.last() - 1);
            s.playing = true;
            s.advance(200.);
            s.advance(200.8);
            assert_eq!(s.cursor, s.last());
            assert!(!s.playing);
        }
    }
}
