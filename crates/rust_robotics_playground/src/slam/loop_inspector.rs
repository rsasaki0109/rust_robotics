//! Prefix-correct views of the recorded LiDAR graph SLAM runs.

mod view;

use super::{CorridorLoopRun, Pose2D, SlamDemo, SlamKind, WRONG_LOOP_TOLERANCE};
use std::collections::BTreeMap;

const RATE: f64 = 40.0;

#[derive(Clone, Debug)]
struct Event {
    frame: usize,
    from: usize,
    to: usize,
    relative: Pose2D,
    wrong: bool,
    optimized: bool,
    inlier: f64,
    residual: f64,
}

struct Trace {
    errors: [Vec<f64>; 2],
    events: Vec<Event>,
    errors_json: String,
    events_json: String,
}

fn distance(a: Pose2D, b: Pose2D) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn pose_json(p: Pose2D) -> String {
    format!("[{},{},{}]", p.x, p.y, p.yaw)
}

#[cfg(target_arch = "wasm32")]
fn poses_json(poses: &[Pose2D]) -> String {
    format!(
        "[{}]",
        poses
            .iter()
            .map(|p| pose_json(*p))
            .collect::<Vec<_>>()
            .join(",")
    )
}

impl Trace {
    fn new(run: &CorridorLoopRun) -> Self {
        let errors: [Vec<f64>; 2] = std::array::from_fn(|i| {
            run.frames
                .iter()
                .map(|f| distance(f.truth, if i == 0 { f.front_end } else { f.estimate }))
                .collect()
        });
        let events: Vec<Event> = run
            .frames
            .iter()
            .enumerate()
            .filter_map(|(frame, f)| {
                f.loop_closure.map(|c| Event {
                    frame,
                    from: c.from,
                    to: c.to,
                    relative: c.relative,
                    wrong: run.is_wrong_loop(&c, WRONG_LOOP_TOLERANCE),
                    optimized: f.optimized,
                    inlier: c.inlier_ratio,
                    residual: c.mean_residual,
                })
            })
            .collect();
        let errors_json = format!(
            "[{}]",
            errors
                .iter()
                .map(|e| format!(
                    "[{}]",
                    e.iter().map(f64::to_string).collect::<Vec<_>>().join(",")
                ))
                .collect::<Vec<_>>()
                .join(",")
        );
        let events_json = format!("[{}]", events.iter().map(|e| format!("{{\"frame\":{},\"from\":{},\"to\":{},\"relative\":{},\"wrong\":{},\"optimized\":{},\"inlier\":{},\"residual\":{}}}",e.frame,e.from,e.to,pose_json(e.relative),e.wrong,e.optimized,e.inlier,e.residual)).collect::<Vec<_>>().join(","));
        Self {
            errors,
            events,
            errors_json,
            events_json,
        }
    }

    fn jump(&self, frame: usize, forward: bool) -> Option<usize> {
        if forward {
            self.events
                .iter()
                .find(|e| e.frame > frame)
                .map(|e| e.frame)
        } else {
            self.events
                .iter()
                .rev()
                .find(|e| e.frame < frame)
                .map(|e| e.frame)
        }
    }
}

#[derive(Default)]
pub(super) struct Inspector {
    pub(super) compare: bool,
    pub(super) vectors: bool,
    traces: [Option<Trace>; 3],
    initial_event: Option<bool>,
    last_time: Option<f64>,
    fraction: f64,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: usize,
    last_action: &'static str,
    revision: usize,
    graph: Option<egui::Rect>,
    maps: Vec<egui::Rect>,
    painted: Option<(usize, usize, bool)>,
    painted_witnesses: [String; 2],
    published_scenario: Option<usize>,
}

impl Inspector {
    fn seek(&mut self, frame: &mut usize, playing: &mut bool, value: usize, last: usize) {
        *frame = value.min(last);
        *playing = false;
        self.last_time = None;
        self.fraction = 0.0;
    }

    fn tick(&mut self, frame: &mut usize, playing: &mut bool, now: f64, last: usize, active: bool) {
        let previous = self.last_time.replace(now);
        if !active || !*playing {
            self.last_time = None;
            return;
        }
        if let Some(previous) = previous {
            let dt = now - previous;
            if (0.0..1.0).contains(&dt) {
                self.fraction += dt * RATE;
                let whole = (self.fraction + 1e-9).floor() as usize;
                self.fraction = (self.fraction - whole as f64).max(0.0);
                *frame = (*frame + whole).min(last);
            }
        }
        if *frame == last {
            *playing = false;
        }
    }

    pub(super) fn button(&mut self, name: &'static str, r: &egui::Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.acknowledge(name, &r.ctx);
            true
        } else {
            false
        }
    }

    fn widget(&mut self, name: &'static str, r: &egui::Response) {
        self.actions.insert(name, r.rect.center().into());
        self.hits.insert(name, r.hovered());
    }

    fn acknowledge(&mut self, name: &'static str, ctx: &egui::Context) {
        self.action_count += 1;
        self.last_action = name;
        ctx.request_repaint();
    }
}

impl SlamDemo {
    pub(super) fn apply_loop_inspector_query(&mut self, query: &str) {
        self.loop_inspector.compare = self.kind == SlamKind::LoopClosure
            && crate::share::value(query, "loop_view") == Some("compare");
        self.loop_inspector.vectors = crate::share::boolean(query, "loop_vectors").unwrap_or(false);
        self.loop_inspector.initial_event = match crate::share::value(query, "loop_event") {
            Some("first") => Some(true),
            Some("before") => Some(false),
            _ => None,
        };
        self.loop_inspector.last_time = None;
        self.loop_inspector.fraction = 0.0;
    }

    fn ensure_loop_inspector(&mut self) {
        let i = self.loop_scenario.index();
        if self.loop_runs[i].is_none() {
            self.loop_runs[i] = Some(self.loop_scenario.run());
        }
        let run = self.loop_runs[i].as_ref().expect("recorded run");
        let trace = self.loop_inspector.traces[i].get_or_insert_with(|| Trace::new(run));
        self.frame_idx = self.frame_idx.min(run.frames.len() - 1);
        if let Some(at_event) = self.loop_inspector.initial_event.take() {
            let first = trace.events.first().map_or(0, |e| e.frame);
            self.loop_inspector.seek(
                &mut self.frame_idx,
                &mut self.playing,
                if at_event {
                    first
                } else {
                    first.saturating_sub(1)
                },
                run.frames.len() - 1,
            );
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        self.explore_lab.begin_frame(ctx, active);
        self.mapping_lab
            .begin_frame(ctx, active && !self.explore_lab.active);
        let active = active && !self.mapping_lab.active && !self.explore_lab.active;
        self.scan_lab
            .begin_frame(ctx, active && self.kind == SlamKind::Icp);
        let compare = self.kind == SlamKind::LoopClosure && self.loop_inspector.compare;
        if let Some(run) = &self.loop_runs[self.loop_scenario.index()] {
            self.loop_inspector.tick(
                &mut self.frame_idx,
                &mut self.playing,
                ctx.input(|i| i.time),
                run.frames.len() - 1,
                active && compare,
            );
        } else {
            self.loop_inspector.last_time = None;
        }
        if active && compare && self.playing {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    fn loop_seek(&mut self, frame: usize) {
        let last = self.loop_runs[self.loop_scenario.index()]
            .as_ref()
            .expect("recorded run")
            .frames
            .len()
            - 1;
        self.loop_inspector
            .seek(&mut self.frame_idx, &mut self.playing, frame, last);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slam::LoopScenario;

    #[test]
    fn clock_follows_elapsed_time_and_discards_pauses_and_background_gaps() {
        for hz in [60.0, 2.5] {
            let mut s = Inspector::default();
            let (mut f, mut playing) = (0, true);
            s.tick(&mut f, &mut playing, 0.0, 1000, true);
            for i in 1..=(2.0 * hz) as usize {
                s.tick(&mut f, &mut playing, i as f64 / hz, 1000, true);
            }
            assert_eq!(f, 80);
            playing = false;
            s.tick(&mut f, &mut playing, 3.0, 1000, true);
            playing = true;
            s.tick(&mut f, &mut playing, 100.0, 1000, true);
            s.tick(&mut f, &mut playing, 200.0, 1000, true);
            assert_eq!(f, 80);
            s.tick(&mut f, &mut playing, 200.5, 1000, true);
            assert_eq!(f, 100);
            s.tick(&mut f, &mut playing, 201.0, 1000, false);
            s.tick(&mut f, &mut playing, 300.0, 1000, true);
            assert_eq!(f, 100);
            s.seek(&mut f, &mut playing, 5000, 101);
            assert_eq!(f, 101);
            assert!(!playing);
        }
    }

    #[test]
    fn comparison_links_are_lazy_exact_paused_and_backward_compatible() {
        let mut demo = SlamDemo::default();
        demo.apply_share_query("tab=slam&algorithm=loop&loop_view=compare&scenario=aliased_off&frame=321&playing=1&loop_vectors=1");
        assert!(demo.loop_inspector.compare && demo.loop_inspector.vectors);
        assert!(demo.loop_runs.iter().all(Option::is_none));
        let q = demo.share_query();
        assert!(q.contains("playing=0"));
        let mut restored = SlamDemo::default();
        restored.apply_share_query(&q);
        assert_eq!(restored.share_query(), q);
        assert!(!restored.playing);
        assert_eq!(restored.frame_idx, 321);
        restored.apply_share_query(
            "tab=slam&algorithm=loop&frontend_map=1&scenario=corridor&frame=9&playing=1",
        );
        assert!(!restored.loop_inspector.compare);
        assert!(restored.show_front_end_map && restored.playing);
        restored.apply_share_query(
            "tab=slam&algorithm=loop&loop_view=bad&loop_vectors=NaN&loop_event=bad",
        );
        assert!(!restored.loop_inspector.compare && !restored.loop_inspector.vectors);
        assert_eq!(restored.loop_inspector.initial_event, None);
    }

    #[test]
    fn recorded_events_and_maps_use_only_the_selected_prefix() {
        let run = LoopScenario::Corridor.run();
        let trace = Trace::new(&run);
        assert_eq!(trace.errors[0].len(), run.frames.len());
        let first = trace.events.first().expect("corridor revisit").frame;
        assert_eq!(trace.jump(0, true), Some(first));
        assert_eq!(trace.jump(first, false), None);
        assert!(trace.events.iter().all(|e| !e.wrong));
        for (i, f) in run.frames.iter().enumerate() {
            assert_eq!(trace.errors[0][i], distance(f.truth, f.front_end));
            assert_eq!(trace.errors[1][i], distance(f.truth, f.estimate));
            let prefix: Vec<_> = trace.events.iter().filter(|e| e.frame <= i).collect();
            assert_eq!(
                prefix.len(),
                run.loop_closures
                    .iter()
                    .filter(|e| e.to < f.node_poses.len())
                    .count()
            );
            assert!(prefix
                .iter()
                .all(|e| e.from < f.node_poses.len() && e.to < f.node_poses.len()));
            if i < first {
                assert!(prefix.is_empty());
            }
        }
        let f = &run.frames[first];
        assert!(f.optimized);
        assert!(distance(f.estimate, f.truth) < distance(f.front_end, f.truth));
        assert!(f
            .node_poses
            .iter()
            .zip(&run.node_front_end)
            .any(|(a, b)| distance(*a, *b) > 0.1));
        eprintln!(
            "corridor updates={}, first={}, events={}, error before={:.3}, graph={:.3}",
            run.frames.len(),
            first,
            trace.events.len(),
            trace.errors[0][first],
            trace.errors[1][first]
        );
    }

    #[test]
    fn ambiguity_check_changes_graph_matches_but_preserves_the_observations() {
        let off = LoopScenario::AliasedNoCheck.run();
        let on = LoopScenario::AliasedWithCheck.run();
        assert_eq!(off.frames.len(), on.frames.len());
        assert_eq!(off.node_truth, on.node_truth);
        assert_eq!(off.node_front_end, on.node_front_end);
        for (a, b) in off.frames.iter().zip(&on.frames) {
            assert_eq!(a.truth, b.truth);
            assert_eq!(a.front_end, b.front_end);
            assert_eq!(a.scan, b.scan);
        }
        let (a, b) = (Trace::new(&off), Trace::new(&on));
        assert_eq!(a.errors[0], b.errors[0]);
        assert_ne!(a.errors[1], b.errors[1]);
        assert!(a.events.iter().any(|e| e.wrong));
        assert!(b.events.iter().all(|e| !e.wrong));
        eprintln!(
            "repeated corridor: off={} events ({} false), on={} events ({} false)",
            a.events.len(),
            a.events.iter().filter(|e| e.wrong).count(),
            b.events.len(),
            b.events.iter().filter(|e| e.wrong).count()
        );
    }

    #[test]
    fn two_maps_fit_a_phone_and_use_the_same_selected_update() {
        let mut demo = SlamDemo::default();
        demo.apply_share_query(
            "tab=slam&algorithm=loop&loop_view=compare&loop_event=first&playing=0&loop_vectors=1",
        );
        let ctx = egui::Context::default();
        for width in [320.0, 1280.0] {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 900.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| demo.loop_scene(ctx, ui));
                });
            });
            let maps = &demo.loop_inspector.maps;
            assert_eq!(maps.len(), 2);
            assert!(maps.iter().all(|r| r.left() >= 0.0 && r.right() <= width));
            assert_eq!(maps[0].width(), maps[1].width());
            assert_eq!(maps[0].height(), maps[1].height());
            assert_eq!(demo.loop_inspector.painted, Some((demo.frame_idx, 0, true)));
            assert!(!demo.playing);
            assert!(demo.loop_runs[0].as_ref().unwrap().frames[demo.frame_idx]
                .loop_closure
                .is_some());
            // Returning at the same update must still wait for a fresh map
            // paint, rather than acknowledging a previous comparison view.
            demo.finish(&ctx, false);
            assert_eq!(demo.loop_inspector.painted, None);
        }
    }
}
