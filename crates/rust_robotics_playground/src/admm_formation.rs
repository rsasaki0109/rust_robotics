//! Interactive ADMM horizon-consensus formation demo.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use rust_robotics_control::{solve_horizon_consensus, AdmmConfig, AgentTrajectory};

const CYCLES: usize = 34;
const HORIZON: usize = 10;
const DX: f64 = 0.18;
const CORNER: usize = 18;
const WORLD_MIN: f64 = -1.0;
const WORLD_MAX: f64 = 4.8;

#[derive(Clone)]
struct AdmmFrame {
    center: [f64; 2],
    agents: Vec<[f64; 2]>,
}

struct AdmmRun {
    color: Color32,
    frames: Vec<AdmmFrame>,
    rms_accel: f64,
    mean_tracking: f64,
}

pub struct AdmmFormationDemo {
    noise_amp: f32,
    show_stiff: bool,
    show_smooth: bool,
    frame_idx: usize,
    playing: bool,
    stiff_run: AdmmRun,
    smooth_run: AdmmRun,
    dirty: bool,
    /// Input time of the last replay step \[s\].
    last_advance: f64,
}

fn offsets() -> Vec<[f64; 2]> {
    vec![[0.6, 0.0], [0.0, 0.6], [-0.6, 0.0], [0.0, -0.6]]
}

fn goal(step: usize) -> [f64; 2] {
    if step <= CORNER {
        [step as f64 * DX, 0.0]
    } else {
        [CORNER as f64 * DX, (step - CORNER) as f64 * DX]
    }
}

fn agent_noise(agent: usize, step: usize, amp: f64) -> [f64; 2] {
    let a = agent as f64;
    let s = step as f64;
    [
        amp * (2.1 * a + 0.7 * s).sin(),
        amp * (1.3 * a + 0.9 * s).cos(),
    ]
}

fn admm_config() -> AdmmConfig {
    AdmmConfig {
        rho: 1.0,
        max_iters: 400,
        tol: 1e-7,
    }
}

fn run_mpc(smooth_weight: f64, noise_amp: f64) -> AdmmRun {
    let offs = offsets();
    let mut center = goal(0);
    let mut path = vec![center];
    let mut frames = Vec::with_capacity(CYCLES + 1);

    frames.push(AdmmFrame {
        center,
        agents: offs
            .iter()
            .map(|o| [center[0] + o[0], center[1] + o[1]])
            .collect(),
    });

    for c in 0..CYCLES {
        let agents: Vec<AgentTrajectory> = offs
            .iter()
            .enumerate()
            .map(|(i, off)| {
                let reference = (0..HORIZON)
                    .map(|t| {
                        let g = goal(c + t);
                        let nz = agent_noise(i, c + t, noise_amp);
                        [g[0] + nz[0] + off[0], g[1] + nz[1] + off[1]]
                    })
                    .collect();
                AgentTrajectory::new(reference, *off)
            })
            .collect();

        let report = solve_horizon_consensus(admm_config(), &agents, smooth_weight, Some(center))
            .expect("horizon consensus");
        center = report.center[1.min(report.center.len() - 1)];
        path.push(center);

        let agent_positions: Vec<[f64; 2]> = report
            .trajectories
            .iter()
            .map(|tr| tr[1.min(tr.len() - 1)])
            .collect();

        frames.push(AdmmFrame {
            center,
            agents: agent_positions,
        });
    }

    let mut accel_sq = 0.0;
    let mut accel_n = 0usize;
    for t in 1..path.len() - 1 {
        let dx = path[t + 1][0] - 2.0 * path[t][0] + path[t - 1][0];
        let dy = path[t + 1][1] - 2.0 * path[t][1] + path[t - 1][1];
        accel_sq += dx * dx + dy * dy;
        accel_n += 1;
    }
    let rms_accel = if accel_n > 0 {
        (accel_sq / accel_n as f64).sqrt()
    } else {
        0.0
    };
    let mean_tracking: f64 = path
        .iter()
        .enumerate()
        .map(|(c, p)| {
            let g = goal(c);
            ((p[0] - g[0]).powi(2) + (p[1] - g[1]).powi(2)).sqrt()
        })
        .sum::<f64>()
        / path.len() as f64;

    AdmmRun {
        color: if smooth_weight > 0.0 {
            Color32::from_rgb(255, 140, 80)
        } else {
            Color32::from_rgb(100, 180, 255)
        },
        frames,
        rms_accel,
        mean_tracking,
    }
}

fn build_runs(noise_amp: f64) -> (AdmmRun, AdmmRun) {
    (run_mpc(0.0, noise_amp), run_mpc(8.0, noise_amp))
}

impl Default for AdmmFormationDemo {
    fn default() -> Self {
        let (stiff_run, smooth_run) = build_runs(0.25);
        Self {
            noise_amp: 0.25,
            show_stiff: true,
            show_smooth: true,
            frame_idx: 0,
            playing: true,
            stiff_run,
            smooth_run,
            dirty: false,
            last_advance: 0.0,
        }
    }
}

impl AdmmFormationDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if let Some(noise) = crate::share::bounded_f32(query, "noise", 0.0, 0.5) {
            self.noise_amp = noise;
            self.dirty = true;
        }
        self.rebuild_if_needed();
        if let Some(show_stiff) = crate::share::boolean(query, "stiff") {
            self.show_stiff = show_stiff;
        }
        if let Some(show_smooth) = crate::share::boolean(query, "smooth") {
            self.show_smooth = show_smooth;
        }
        if let Some(frame) = crate::share::bounded_usize(query, "frame", CYCLES) {
            self.frame_idx = frame;
        }
        if let Some(playing) = crate::share::boolean(query, "playing") {
            self.playing = playing;
        }
    }

    pub fn share_query(&self) -> String {
        format!(
            "tab=admm&noise={:.2}&stiff={}&smooth={}&frame={}&playing={}",
            self.noise_amp,
            u8::from(self.show_stiff),
            u8::from(self.show_smooth),
            self.frame_idx,
            u8::from(self.playing)
        )
    }

    fn rebuild_if_needed(&mut self) {
        if self.dirty {
            let (stiff, smooth) = build_runs(f64::from(self.noise_amp));
            self.stiff_run = stiff;
            self.smooth_run = smooth;
            self.frame_idx = 0;
            self.dirty = false;
        }
    }

    fn world_rect(ui: &egui::Ui) -> (Rect, f32) {
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 72.0);
        (rect, rect.width())
    }

    fn world_to_screen(&self, rect: Rect, side: f32, x: f64, y: f64) -> Pos2 {
        let u = ((x - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
        let v = 1.0 - ((y - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
        rect.min + Vec2::new(u * side, v * side)
    }

    fn draw_run_trail(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        side: f32,
        run: &AdmmRun,
        upto: usize,
    ) {
        if upto >= 1 {
            let pts: Vec<Pos2> = run.frames[..=upto.min(run.frames.len() - 1)]
                .iter()
                .map(|f| self.world_to_screen(rect, side, f.center[0], f.center[1]))
                .collect();
            painter.add(egui::Shape::line(
                pts,
                Stroke::new(
                    2.0_f32,
                    Color32::from_rgba_unmultiplied(
                        run.color.r(),
                        run.color.g(),
                        run.color.b(),
                        120,
                    ),
                ),
            ));
        }
    }

    fn draw_frame_agents(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        side: f32,
        frame: &AdmmFrame,
        color: Color32,
    ) {
        let center = self.world_to_screen(rect, side, frame.center[0], frame.center[1]);
        painter.circle_stroke(center, 8.0, Stroke::new(1.5_f32, color));
        for agent in &frame.agents {
            let c = self.world_to_screen(rect, side, agent[0], agent[1]);
            painter.circle_filled(c, 5.0, color);
            painter.line_segment(
                [center, c],
                Stroke::new(
                    1.0_f32,
                    Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 80),
                ),
            );
        }
    }

    fn draw_scene(&self, ui: &mut egui::Ui, rect: Rect, side: f32) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));

        let goal_pts: Vec<Pos2> = (0..=CYCLES)
            .map(|c| {
                let g = goal(c);
                self.world_to_screen(rect, side, g[0], g[1])
            })
            .collect();
        painter.add(egui::Shape::line(
            goal_pts,
            Stroke::new(2.5_f32, Color32::from_rgb(180, 180, 190)),
        ));

        let idx = self
            .frame_idx
            .min(self.stiff_run.frames.len().saturating_sub(1));
        if self.show_stiff {
            self.draw_run_trail(&painter, rect, side, &self.stiff_run, idx);
        }
        if self.show_smooth {
            self.draw_run_trail(&painter, rect, side, &self.smooth_run, idx);
        }

        if self.show_stiff {
            self.draw_frame_agents(
                &painter,
                rect,
                side,
                &self.stiff_run.frames[idx],
                self.stiff_run.color,
            );
        }
        if self.show_smooth {
            self.draw_frame_agents(
                &painter,
                rect,
                side,
                &self.smooth_run.frames[idx],
                self.smooth_run.color,
            );
        }

        let g = goal(idx);
        let gc = self.world_to_screen(rect, side, g[0], g[1]);
        painter.circle_filled(gc, 6.0, Color32::from_rgb(220, 220, 230));
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Goal noise per agent");
        if ui
            .add(egui::Slider::new(&mut self.noise_amp, 0.0..=0.5).suffix(" m"))
            .changed()
        {
            self.dirty = true;
        }
        crate::ui_kit::section(ui, "Show");
        ui.checkbox(&mut self.show_stiff, "Stiff (λ = 0)");
        ui.checkbox(&mut self.show_smooth, "Smoothed (λ = 8)");
        crate::ui_kit::legend(
            ui,
            &[
                (self.stiff_run.color, "stiff"),
                (self.smooth_run.color, "smoothed"),
                (Color32::from_rgb(220, 220, 230), "moving goal"),
            ],
        );

        crate::ui_kit::section(ui, "Result");
        egui::Grid::new("admm_metrics")
            .num_columns(3)
            .striped(true)
            .show(ui, |ui| {
                ui.label("");
                ui.small("RMS accel");
                ui.small("tracking");
                ui.end_row();
                for (name, run) in [("Stiff", &self.stiff_run), ("Smoothed", &self.smooth_run)] {
                    ui.strong(name);
                    ui.label(format!("{:.3}", run.rms_accel));
                    ui.label(format!("{:.3} m", run.mean_tracking));
                    ui.end_row();
                }
            });
        if ui.button("Reset").clicked() {
            *self = Self::default();
        }
        crate::ui_kit::how_it_works(
            ui,
            "admm_help",
            "Four agents agree on a shared formation center via receding-horizon ADMM \
             while each one sees its own noisy copy of the goal. Smoothing rejects the \
             noise and cuts the jerk at the L-corner, at a small cost in tracking.",
        );
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.rebuild_if_needed();
        let (rect, side) = Self::world_rect(ui);
        let _ = ui.allocate_rect(rect, egui::Sense::hover());
        self.draw_scene(ui, rect, side);

        let max_idx = self.stiff_run.frames.len().saturating_sub(1);
        crate::ui_kit::playback(ui, &mut self.playing, &mut self.frame_idx, max_idx);

        if self.playing && self.frame_idx < max_idx {
            if crate::ui_kit::every(ctx, &mut self.last_advance, 0.1) {
                self.frame_idx += 1;
            }
        } else if self.frame_idx >= max_idx {
            self.playing = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::AdmmFormationDemo;

    #[test]
    fn share_query_round_trips_configuration() {
        let demo = AdmmFormationDemo {
            noise_amp: 0.4,
            show_stiff: false,
            frame_idx: 8,
            ..AdmmFormationDemo::default()
        };
        let query = demo.share_query();
        let mut restored = AdmmFormationDemo::default();
        restored.apply_share_query(&query);
        assert!((restored.noise_amp - 0.4).abs() < f32::EPSILON);
        assert!(!restored.show_stiff);
        assert!(restored.show_smooth);
        assert_eq!(restored.frame_idx, 8);
    }
}
