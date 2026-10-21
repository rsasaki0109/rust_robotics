#[cfg(target_arch = "wasm32")]
use super::poses_json;
use super::{distance, pose_json, SlamDemo, Trace};
use crate::slam::{LoopScenario, SlamKind};
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};
use rust_robotics_core::Pose2D;
use rust_robotics_slam::lidar_loop_scenario::CorridorLoopRun;

const FRONT: Color32 = Color32::from_rgb(255, 157, 91);
const GRAPH: Color32 = Color32::from_rgb(76, 216, 203);
const LOOP: Color32 = Color32::from_rgb(224, 120, 246);
const GOLD: Color32 = Color32::from_rgb(255, 212, 101);

fn screen(rect: Rect, p: [f64; 2]) -> Pos2 {
    egui::pos2(
        rect.left() + ((p[0] + 16.0) / 32.0) as f32 * rect.width(),
        rect.bottom() - ((p[1] + 11.0) / 22.0) as f32 * rect.height(),
    )
}

fn transform(p: [f64; 2], pose: Pose2D) -> [f64; 2] {
    [
        pose.x + pose.yaw.cos() * p[0] - pose.yaw.sin() * p[1],
        pose.y + pose.yaw.sin() * p[0] + pose.yaw.cos() * p[1],
    ]
}

fn map(
    ui: &egui::Ui,
    rect: Rect,
    run: &CorridorLoopRun,
    trace: &Trace,
    frame: usize,
    graph: bool,
    vectors: bool,
) -> String {
    let f = &run.frames[frame];
    let count = f.node_poses.len();
    let poses = if graph {
        &f.node_poses[..]
    } else {
        &run.node_front_end[..count]
    };
    let color = if graph { GRAPH } else { FRONT };
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
    for wall in &run.walls {
        painter.line_segment(
            [
                screen(rect, [wall.start.x, wall.start.y]),
                screen(rect, [wall.end.x, wall.end.y]),
            ],
            Stroke::new(1.0_f32, Color32::from_gray(85)),
        );
    }
    let stride = (count * run.node_scans[0].len()).div_ceil(4500).max(1);
    let selected = [0, count / 2, count - 1];
    let mut witnesses: [String; 3] = std::array::from_fn(|_| String::new());
    for (node, (pose, scan)) in poses.iter().zip(&run.node_scans).enumerate() {
        for (beam, p) in scan.iter().step_by(stride).enumerate() {
            let world = transform([p.x, p.y], *pose);
            painter.circle_filled(screen(rect, world), 1.1, color.gamma_multiply(0.5));
            // Capture the coordinates actually passed to the painter.
            if beam == 0 && selected.contains(&node) {
                let entry = format!(
                    "{{\"node\":{node},\"local\":[{},{}],\"pose\":{},\"world\":[{},{}]}}",
                    p.x,
                    p.y,
                    pose_json(*pose),
                    world[0],
                    world[1]
                );
                for (slot, index) in selected.iter().enumerate() {
                    if *index == node {
                        witnesses[slot] = entry.clone();
                    }
                }
            }
        }
    }
    let path: Vec<_> = poses.iter().map(|p| screen(rect, [p.x, p.y])).collect();
    if path.len() > 1 {
        painter.add(Shape::line(path, Stroke::new(1.6_f32, color)));
    }
    if graph {
        for e in trace.events.iter().filter(|e| e.frame <= frame) {
            let (a, b) = (poses[e.from], poses[e.to]);
            painter.line_segment(
                [screen(rect, [a.x, a.y]), screen(rect, [b.x, b.y])],
                Stroke::new(
                    if e.frame == frame { 3.5_f32 } else { 1.5_f32 },
                    if e.wrong { GOLD } else { LOOP },
                ),
            );
        }
        if vectors {
            for (a, b) in run.node_front_end[..count]
                .iter()
                .zip(poses)
                .step_by(count.div_ceil(32).max(1))
            {
                let (from, to) = (screen(rect, [a.x, a.y]), screen(rect, [b.x, b.y]));
                if from.distance(to) > 1.0 {
                    painter.arrow(from, to - from, Stroke::new(1.0_f32, GOLD));
                }
            }
        }
    }
    let pose = if graph { f.estimate } else { f.front_end };
    for p in f.scan.iter().step_by(2) {
        painter.circle_filled(
            screen(rect, transform([p.x, p.y], pose)),
            1.1,
            Color32::from_rgb(250, 100, 110),
        );
    }
    let at = screen(rect, [pose.x, pose.y]);
    painter.circle_filled(at, 4.0, color);
    painter.arrow(
        at,
        egui::vec2(pose.yaw.cos() as f32 * 10.0, -pose.yaw.sin() as f32 * 10.0),
        Stroke::new(2.0_f32, color),
    );
    painter.circle_stroke(
        screen(rect, [f.truth.x, f.truth.y]),
        5.0,
        Stroke::new(1.0_f32, Color32::WHITE),
    );
    format!("[{}]", witnesses.join(","))
}

impl SlamDemo {
    pub(in crate::slam) fn loop_controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.ensure_loop_inspector();
        crate::ui_kit::section(ui, "Loop closure lab");
        crate::ui_kit::hint(
            ui,
            "The same scans, placed at front-end and pose-graph poses at one recorded update.",
        );
        ui.horizontal_wrapped(|ui| {
            for (scenario, title) in [
                (LoopScenario::Corridor, "Corridor"),
                (LoopScenario::AliasedNoCheck, "Repeated: check off"),
                (LoopScenario::AliasedWithCheck, "Repeated: check on"),
            ] {
                let r = ui.selectable_label(self.loop_scenario == scenario, title);
                if self.loop_inspector.button(scenario.slug(), &r) {
                    self.loop_scenario = scenario;
                    self.ensure_loop_inspector();
                    self.loop_seek(0);
                }
            }
        });
        crate::ui_kit::section(ui, "Inspect a recorded event");
        let index = self.loop_scenario.index();
        let trace = self.loop_inspector.traces[index].as_ref().expect("trace");
        let first = trace.events.first().map(|e| e.frame);
        let previous = trace.jump(self.frame_idx, false);
        let next = trace.jump(self.frame_idx, true);
        let first_false = trace.events.iter().find(|e| e.wrong).map(|e| e.frame);
        ui.horizontal_wrapped(|ui| {
            for (key, title, frame) in [
                ("before", "Before first", first.map(|f| f.saturating_sub(1))),
                ("first", "First loop", first),
                ("previous", "Previous loop", previous),
                ("next", "Next loop", next),
                ("false", "First false loop", first_false),
            ] {
                let r = ui.add_enabled(frame.is_some(), egui::Button::new(title));
                if self.loop_inspector.button(key, &r) {
                    self.loop_seek(frame.expect("enabled jump"));
                }
            }
        });
        let last = self.loop_runs[index].as_ref().expect("run").frames.len() - 1;
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.loop_inspector.button("play", &r) {
                if self.frame_idx == last {
                    self.loop_seek(0);
                }
                self.playing = !self.playing;
                self.loop_inspector.last_time = None;
            }
            for (key, title, frame) in [
                ("start", "Start", 0),
                ("back", "Back", self.frame_idx.saturating_sub(1)),
                ("step", "+1 update", (self.frame_idx + 1).min(last)),
                ("result", "Result", last),
            ] {
                let r = ui.button(title);
                if self.loop_inspector.button(key, &r) {
                    self.loop_seek(frame);
                }
            }
        });
        let mut frame = self.frame_idx;
        let r = ui.add(egui::Slider::new(&mut frame, 0..=last).text("update"));
        self.loop_inspector.widget("cursor", &r);
        if r.changed() {
            self.loop_seek(frame);
            ctx.request_repaint();
        }
        if r.clicked() || r.drag_stopped() {
            self.loop_inspector.acknowledge("cursor", ctx);
        }
        ui.label(format!(
            "Update {} / {last} · {}",
            self.frame_idx,
            if self.playing { "Playing" } else { "Paused" }
        ));
        let r = ui.checkbox(&mut self.loop_inspector.vectors, "Node correction arrows");
        self.loop_inspector.button("vectors", &r);
        let run = self.loop_runs[index].as_ref().expect("run");
        let trace = self.loop_inspector.traces[index].as_ref().expect("trace");
        let f = &run.frames[self.frame_idx];
        let accepted = trace
            .events
            .iter()
            .filter(|e| e.frame <= self.frame_idx)
            .count();
        let wrong = trace
            .events
            .iter()
            .filter(|e| e.frame <= self.frame_idx && e.wrong)
            .count();
        ui.label(format!(
            "{} map nodes · {accepted} accepted loops",
            f.node_poses.len()
        ));
        if wrong > 0 {
            ui.colored_label(GOLD, format!("{wrong} false loops · simulation truth"));
        }
        for (i, color, title) in [(0, FRONT, "Front end"), (1, GRAPH, "Pose graph")] {
            ui.colored_label(
                color,
                format!("{title} error: {:.3} m", trace.errors[i][self.frame_idx]),
            );
        }
        let max = f
            .node_poses
            .iter()
            .zip(&run.node_front_end)
            .map(|(a, b)| distance(*a, *b))
            .fold(0.0_f64, f64::max);
        ui.label(format!("Largest cumulative node shift: {max:.2} m"));
        if let Some(e) = trace.events.iter().find(|e| e.frame == self.frame_idx) {
            ui.label(format!("Matched node {} to {}", e.to, e.from));
            ui.label(format!(
                "Inliers {:.0}% · residual {:.3} m",
                e.inlier * 100.0,
                e.residual
            ));
            ui.label(if e.optimized {
                "Pose graph re-optimized at this update"
            } else {
                "Accepted edge; no re-optimization at this update"
            });
            if e.wrong {
                ui.colored_label(GOLD, "False match, judged from simulation truth");
            }
        } else if let Some(next) = next {
            ui.label(format!("Next recorded loop: update {next}"));
        }
        crate::ui_kit::legend(
            ui,
            &[
                (FRONT, "front end"),
                (GRAPH, "pose graph"),
                (LOOP, "loop edge"),
                (GOLD, "false edge / node shift"),
                (Color32::WHITE, "true robot"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"loop_compare_help","Both maps use the same node scans, only up to the selected update, on identical world scales. The front end stays independent of loop corrections. The right map uses the pose graph recorded at this update, without later optimization. Gold arrows show cumulative changes from front-end node poses. A loop edge matches a revisited place; it can move many older nodes when the graph is re-optimized. Repeated pillars can create a geometrically plausible but false match. The ambiguity check helps reject these. False-loop labels use simulation ground truth and are not information available to a real robot. The graph shows both full recorded position-error histories; event markers are accepted loops. Tap it to inspect one shared update. Playback is 40 recorded updates per second, not a physical time model; gaps of at least one second are discarded. Share opens this exact update paused.");
        let r = ui.button("Single map");
        if self.loop_inspector.button("single", &r) {
            self.loop_inspector.compare = false;
            self.playing = false;
        }
    }

    pub(in crate::slam) fn loop_scene(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        self.ensure_loop_inspector();
        ui.heading("Loop closure lab");
        let width = ui.available_width();
        let side_by_side = width >= 680.0;
        let map_width = if side_by_side {
            (width - 12.0) / 2.0
        } else {
            width
        };
        let map_height = map_width * 22.0 / 32.0;
        let panel_height = map_height + 27.0;
        let total = if side_by_side {
            panel_height
        } else {
            panel_height * 2.0 + 10.0
        };
        let (area, _) = ui.allocate_exact_size(egui::vec2(width, total), Sense::hover());
        self.loop_inspector.maps = (0..2)
            .map(|i| {
                let offset = if side_by_side {
                    egui::vec2(i as f32 * (map_width + 12.0), 27.0)
                } else {
                    egui::vec2(0.0, i as f32 * (panel_height + 10.0) + 27.0)
                };
                Rect::from_min_size(area.min + offset, egui::vec2(map_width, map_height))
            })
            .collect();
        // The graph's input selects the update before either map is painted.
        self.loop_error_graph(ui);
        let index = self.loop_scenario.index();
        let run = self.loop_runs[index].as_ref().expect("run");
        let trace = self.loop_inspector.traces[index].as_ref().expect("trace");
        for (i, rect) in self.loop_inspector.maps.iter().enumerate() {
            let color = if i == 0 { FRONT } else { GRAPH };
            let label = if i == 0 { "Scan-to-map" } else { "Pose graph" };
            ui.painter().text(
                rect.left_top() - egui::vec2(0.0, 22.0),
                egui::Align2::LEFT_TOP,
                format!("{label} · {:.3} m", trace.errors[i][self.frame_idx]),
                egui::FontId::proportional(15.0),
                color,
            );
            self.loop_inspector.painted_witnesses[i] = map(
                ui,
                *rect,
                run,
                trace,
                self.frame_idx,
                i == 1,
                self.loop_inspector.vectors,
            );
        }
        self.loop_inspector.painted = Some((self.frame_idx, index, self.loop_inspector.vectors));
        let events = trace
            .events
            .iter()
            .filter(|e| e.frame <= self.frame_idx)
            .count();
        ui.label(format!(
            "Update {} · {events} accepted loops · same scans and world scale",
            self.frame_idx
        ));
        crate::ui_kit::hint(
            ui,
            "Full recorded run · tap the error graph or jump between loop events",
        );
    }

    fn loop_error_graph(&mut self, ui: &mut egui::Ui) {
        let (rect, r) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 118.0),
            Sense::click_and_drag(),
        );
        self.loop_inspector.widget("graph", &r);
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(32.0, 14.0),
            rect.max - egui::vec2(10.0, 24.0),
        );
        self.loop_inspector.graph = Some(plot);
        let index = self.loop_scenario.index();
        let last = self.loop_runs[index].as_ref().expect("run").frames.len() - 1;
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                self.loop_seek(
                    (((p.x - plot.left()) / plot.width()).clamp(0.0, 1.0) * last as f32).round()
                        as usize,
                );
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.loop_inspector.acknowledge("graph", &r.ctx);
        }
        let trace = self.loop_inspector.traces[index].as_ref().expect("trace");
        let max = trace
            .errors
            .iter()
            .flatten()
            .copied()
            .fold(0.05_f64, f64::max)
            * 1.08;
        let point = |frame: usize, value: f64| {
            egui::pos2(
                plot.left() + plot.width() * frame as f32 / last.max(1) as f32,
                plot.bottom() - plot.height() * (value / max) as f32,
            )
        };
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        for e in &trace.events {
            let x = point(e.frame, 0.0).x;
            painter.line_segment(
                [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
                Stroke::new(
                    0.6_f32,
                    if e.wrong {
                        GOLD.gamma_multiply(0.4)
                    } else {
                        LOOP.gamma_multiply(0.4)
                    },
                ),
            );
        }
        for (i, color) in [(0, FRONT), (1, GRAPH)] {
            painter.add(Shape::line(
                trace.errors[i]
                    .iter()
                    .enumerate()
                    .map(|(f, v)| point(f, *v))
                    .collect(),
                Stroke::new(1.8_f32, color),
            ));
            painter.circle_filled(
                point(self.frame_idx, trace.errors[i][self.frame_idx]),
                3.5,
                color,
            );
        }
        let x = point(self.frame_idx, 0.0).x;
        painter.line_segment(
            [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
            Stroke::new(1.0_f32, Color32::WHITE),
        );
        painter.text(
            rect.left_top() + egui::vec2(4.0, 1.0),
            egui::Align2::LEFT_TOP,
            "Position error / m",
            egui::FontId::proportional(11.0),
            Color32::LIGHT_GRAY,
        );
        for f in [0, last / 2, last] {
            painter.text(
                egui::pos2(point(f, 0.0).x, plot.bottom() + 4.0),
                egui::Align2::CENTER_TOP,
                format!("{f}"),
                egui::FontId::proportional(11.0),
                Color32::GRAY,
            );
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.mapping_lab.finish(ctx, active);
        let active = active && !self.mapping_lab.active;
        self.scan_lab
            .finish(ctx, active && self.kind == SlamKind::Icp);
        self.loop_inspector.revision += 1;
        let active = active && self.kind == SlamKind::LoopClosure && self.loop_inspector.compare;
        if !active {
            self.loop_inspector.painted = None;
        }
        self.publish_loop(ctx, active);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_loop(&mut self, _ctx: &egui::Context, _active: bool) {
        let _ = self.loop_inspector.published_scenario;
        let _ = &self.loop_inspector.painted_witnesses;
        if let Some(t) = &self.loop_inspector.traces[self.loop_scenario.index()] {
            let _ = (&t.errors_json, &t.events_json);
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn publish_loop(&mut self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |key: &str, value: &str| {
            let _ = canvas.set_attribute(&format!("data-loop-{key}"), value);
        };
        let state = &mut self.loop_inspector;
        set("active", if active { "1" } else { "0" });
        set(
            "actions",
            &format!(
                "{{{}}}",
                state
                    .actions
                    .iter()
                    .map(|(k, p)| format!("\"{k}\":[{},{}]", p[0], p[1]))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set(
            "hits",
            &format!(
                "{{{}}}",
                state
                    .hits
                    .iter()
                    .map(|(k, v)| format!("\"{k}\":{v}"))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set("action-count", &state.action_count.to_string());
        set("last-action", state.last_action);
        set("ui-revision", &state.revision.to_string());
        set(
            "pointer",
            &ctx.input(|i| i.pointer.hover_pos())
                .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y)),
        );
        if !active {
            return;
        }
        let index = self.loop_scenario.index();
        let Some(run) = &self.loop_runs[index] else {
            return;
        };
        let trace = state.traces[index].as_ref().expect("trace");
        if state.published_scenario != Some(index) {
            set("errors", &trace.errors_json);
            set("events", &trace.events_json);
            state.published_scenario = Some(index);
        }
        let f = &run.frames[self.frame_idx];
        let count = f.node_poses.len();
        let rect_json =
            |r: Rect| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height());
        let shifts = f
            .node_poses
            .iter()
            .zip(&run.node_front_end)
            .map(|(a, b)| distance(*a, *b))
            .collect::<Vec<_>>();
        let last = run.frames.len() - 1;
        let accepted = trace
            .events
            .iter()
            .filter(|e| e.frame <= self.frame_idx)
            .count();
        let wrong = trace
            .events
            .iter()
            .filter(|e| e.frame <= self.frame_idx && e.wrong)
            .count();
        let painted = state.painted;
        for (key, value) in [
            ("frame", self.frame_idx.to_string()),
            ("last", last.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("scenario", self.loop_scenario.slug().into()),
            ("scenario-index", index.to_string()),
            ("vectors", u8::from(state.vectors).to_string()),
            ("truth", pose_json(f.truth)),
            ("front", pose_json(f.front_end)),
            ("estimate", pose_json(f.estimate)),
            ("front-nodes", poses_json(&run.node_front_end[..count])),
            ("nodes", poses_json(&f.node_poses)),
            ("node-truth", poses_json(&run.node_truth[..count])),
            (
                "shifts",
                format!(
                    "[{}]",
                    shifts
                        .iter()
                        .map(f64::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                ),
            ),
            (
                "witnesses",
                format!(
                    "[{},{}]",
                    state.painted_witnesses[0], state.painted_witnesses[1]
                ),
            ),
            ("accepted", accepted.to_string()),
            ("wrong", wrong.to_string()),
            ("optimized", u8::from(f.optimized).to_string()),
            (
                "painted-frame",
                painted.map_or_else(|| "null".into(), |p| p.0.to_string()),
            ),
            (
                "painted-scenario",
                painted.map_or_else(|| "null".into(), |p| p.1.to_string()),
            ),
            (
                "painted-vectors",
                painted.map_or_else(|| "null".into(), |p| u8::from(p.2).to_string()),
            ),
            (
                "maps",
                format!(
                    "[{}]",
                    state
                        .maps
                        .iter()
                        .map(|r| rect_json(*r))
                        .collect::<Vec<_>>()
                        .join(",")
                ),
            ),
            (
                "graph",
                state.graph.map_or_else(|| "null".into(), rect_json),
            ),
        ] {
            set(key, &value);
        }
        set("share", &self.share_query());
    }
}
