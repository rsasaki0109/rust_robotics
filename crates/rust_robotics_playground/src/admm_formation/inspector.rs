//! Recorded MPC plans, kept separate from the ADMM solver's iteration history.

mod view;

use super::{goal, AdmmFormationDemo, AdmmRun, CORNER, CYCLES};
use std::collections::BTreeMap;

const RATE: f64 = 5.0;

pub(super) struct Inspector {
    pub(super) active: bool,
    pub(super) horizon: bool,
    pub(super) observations: bool,
    last_time: Option<f64>,
    fraction: f64,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: usize,
    last_action: &'static str,
    revision: usize,
    graph: Option<egui::Rect>,
    maps: Vec<egui::Rect>,
    painted: Option<(usize, f32, bool, bool)>,
    witnesses: [String; 2],
}

impl Default for Inspector {
    fn default() -> Self {
        Self {
            active: false,
            horizon: true,
            observations: true,
            last_time: None,
            fraction: 0.0,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            action_count: 0,
            last_action: "",
            revision: 0,
            graph: None,
            maps: Vec::new(),
            painted: None,
            witnesses: ["[]".into(), "[]".into()],
        }
    }
}

impl Inspector {
    pub(super) fn reset_clock(&mut self) {
        self.last_time = None;
        self.fraction = 0.0;
    }

    fn tick(&mut self, frame: &mut usize, playing: &mut bool, now: f64, active: bool) {
        let previous = self.last_time.replace(now);
        if !active || !*playing {
            self.reset_clock();
            return;
        }
        if let Some(previous) = previous {
            let dt = now - previous;
            if (0.0..1.0).contains(&dt) {
                self.fraction += dt * RATE;
                let whole = (self.fraction + 1e-9).floor() as usize;
                self.fraction = (self.fraction - whole as f64).max(0.0);
                *frame = (*frame + whole).min(CYCLES);
            }
        }
        if *frame == CYCLES {
            *playing = false;
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

    pub(super) fn button(&mut self, name: &'static str, r: &egui::Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.acknowledge(name, &r.ctx);
            true
        } else {
            false
        }
    }
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Both statistics use only positions already applied through this frame.
fn metrics(run: &AdmmRun, frame: usize) -> (f64, f64) {
    let frames = &run.frames[..=frame];
    let mean = frames
        .iter()
        .enumerate()
        .map(|(i, f)| distance(f.center, goal(i)))
        .sum::<f64>()
        / frames.len() as f64;
    let sq = frames
        .windows(3)
        .map(|f| {
            (f[2].center[0] - 2.0 * f[1].center[0] + f[0].center[0]).powi(2)
                + (f[2].center[1] - 2.0 * f[1].center[1] + f[0].center[1]).powi(2)
        })
        .sum::<f64>();
    let rms = if frames.len() > 2 {
        (sq / (frames.len() - 2) as f64).sqrt()
    } else {
        0.0
    };
    (mean, rms)
}

impl AdmmFormationDemo {
    pub(super) fn seek(&mut self, frame: usize) {
        self.frame_idx = frame.min(CYCLES);
        self.playing = false;
        self.inspector.reset_clock();
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        self.inspector.tick(
            &mut self.frame_idx,
            &mut self.playing,
            ctx.input(|i| i.time),
            active && self.inspector.active,
        );
        if active && self.inspector.active && self.playing {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.inspector.revision += 1;
        let active = active && self.inspector.active;
        if !active {
            self.inspector.painted = None;
        }
        self.publish_formation(ctx, active);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::admm_formation::{admm_config, build_runs, offsets, HORIZON};

    #[test]
    fn elapsed_playback_is_independent_of_paint_rate_and_excludes_inactive_time() {
        for hz in [60.0, 2.5] {
            let mut state = Inspector::default();
            let (mut frame, mut playing) = (0, true);
            state.tick(&mut frame, &mut playing, 0.0, true);
            for i in 1..=(2.0 * hz) as usize {
                state.tick(&mut frame, &mut playing, i as f64 / hz, true);
            }
            assert_eq!(frame, 10);
            state.tick(&mut frame, &mut playing, 50.0, true);
            assert_eq!(frame, 10);
            state.tick(&mut frame, &mut playing, 50.5, false);
            state.tick(&mut frame, &mut playing, 100.0, true);
            assert_eq!(frame, 10);
            state.tick(&mut frame, &mut playing, 100.4, true);
            assert_eq!(frame, 12);
            playing = false;
            state.tick(&mut frame, &mut playing, 101.0, true);
            playing = true;
            state.tick(&mut frame, &mut playing, 200.0, true);
            assert_eq!(frame, 12);
        }
    }

    #[test]
    fn plans_share_observations_and_apply_exactly_the_reported_next_positions() {
        let (stiff, smooth) = build_runs(0.25);
        for c in 0..CYCLES {
            let a = stiff.frames[c].plan.as_ref().unwrap();
            let b = smooth.frames[c].plan.as_ref().unwrap();
            assert_eq!(a.references, b.references);
            for run in [&stiff, &smooth] {
                let current = &run.frames[c];
                let p = &current.plan.as_ref().unwrap().report;
                assert_eq!(p.center.len(), HORIZON);
                assert_eq!(p.center[0], current.center);
                assert_eq!(run.frames[c + 1].center, p.center[1]);
                assert_eq!(p.iterations, p.primal_residuals.len());
                assert_eq!(p.iterations, p.dual_residuals.len());
                assert!(p.primal_residuals.last().unwrap() < &admm_config().tol);
                assert!(p.dual_residuals.last().unwrap() < &admm_config().tol);
                for (i, offset) in offsets().iter().enumerate() {
                    assert_eq!(run.frames[c + 1].agents[i], p.trajectories[i][1]);
                    assert!(
                        distance(
                            p.trajectories[i][1],
                            [p.center[1][0] + offset[0], p.center[1][1] + offset[1]]
                        ) < 1e-7
                    );
                }
            }
        }
        assert!(stiff.frames[CYCLES].plan.is_none());
        assert!(smooth.frames[CYCLES].plan.is_none());
        assert!(smooth.rms_accel < stiff.rms_accel);
    }

    #[test]
    fn prefix_metrics_ignore_future_plans_and_match_complete_run_statistics() {
        let (mut run, _) = build_runs(0.25);
        assert_eq!(metrics(&run, 0), (0.0, 0.0));
        let before = metrics(&run, CORNER - 1);
        let final_metrics = metrics(&run, CYCLES);
        assert!((final_metrics.0 - run.mean_tracking).abs() < 1e-12);
        assert!((final_metrics.1 - run.rms_accel).abs() < 1e-12);
        run.frames[CYCLES].center = [1000.0, 2000.0];
        assert_eq!(metrics(&run, CORNER - 1), before);
        assert!(metrics(&run, CYCLES).0 > final_metrics.0);
    }

    #[test]
    fn links_restore_exact_noise_frame_and_layers_paused_and_preserve_legacy() {
        let mut demo = AdmmFormationDemo::default();
        demo.apply_share_query("formation_view=inspect&noise=0.12345678&frame=17&playing=1&formation_horizon=0&formation_observations=1");
        let q = demo.share_query();
        assert!(q.contains("playing=0"));
        let mut restored = AdmmFormationDemo::default();
        restored.apply_share_query(&q);
        assert_eq!(restored.noise_amp, demo.noise_amp);
        assert_eq!(restored.frame_idx, 17);
        assert_eq!(restored.share_query(), q);
        assert_eq!(
            restored.smooth_run.frames[17].center,
            demo.smooth_run.frames[17].center
        );
        assert!(!restored.playing && !restored.inspector.horizon);
        restored.apply_share_query("noise=0.4&frame=8&playing=1&stiff=0");
        assert!(!restored.inspector.active && !restored.show_stiff);
        assert!(restored.playing);
        restored.apply_share_query("formation_view=bad&noise=NaN&frame=999&formation_horizon=bad");
        assert!(!restored.inspector.active);
        assert_eq!(restored.frame_idx, 8);
        assert!(restored.inspector.horizon);
    }
}
