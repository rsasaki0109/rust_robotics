//! A recorded experiment built from the library's real PF operations. Seeking
//! changes only the view; sensor and filter RNGs are separate and seeded.
mod view;

use std::collections::BTreeMap;

use egui::{Context, Rect, Response};
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, StandardNormal};
use rust_robotics_localization::{
    PFControl, PFMeasurement, PFState, Particle, ParticleFilterConfig, ParticleFilterLocalizer,
};

const N: usize = 96;
const SCANS: usize = 24;
const STEP: f64 = 0.25;
const LAST: usize = SCANS * 4 - 1;
const PHASES: [&str; 4] = ["Prior", "Predict", "Weight", "Resample"];
const LANDMARKS: [[f64; 2]; 5] = [[2., 2.], [10., 2.], [2., 8.], [10., 8.], [6., 5.]];

struct Scan {
    clouds: [Vec<Particle>; 4],
    previous_truth: [f64; 3],
    truth: [f64; 3],
    observations: PFMeasurement,
    parents: Option<Vec<usize>>,
    ess: f64,
}

pub(super) struct BeliefLab {
    pub(super) active: bool,
    seed: u64,
    sigma: f64,
    landmarks: usize,
    cursor: usize,
    selected: usize,
    ranges: bool,
    playing: bool,
    clock: Option<f64>,
    pending: f64,
    scans: Vec<Scan>,
    generation: usize,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<Rect>,
    graph: Option<Rect>,
    painted: Option<(usize, usize, usize, bool)>,
    witnesses: String,
    bars: String,
}

impl Default for BeliefLab {
    fn default() -> Self {
        Self {
            active: false,
            seed: 7,
            sigma: 0.25,
            landmarks: 5,
            cursor: 2,
            selected: 0,
            ranges: true,
            playing: false,
            clock: None,
            pending: 0.,
            scans: Vec::new(),
            generation: 0,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            map: None,
            graph: None,
            painted: None,
            witnesses: "[]".into(),
            bars: "[]".into(),
        }
    }
}

fn ess(p: &[Particle]) -> f64 {
    1. / p.iter().map(|p| p.w * p.w).sum::<f64>()
}
fn values(p: &Particle) -> [f64; 5] {
    [p.x, p.y, p.yaw, p.v, p.w]
}
#[cfg(any(test, target_arch = "wasm32"))]
fn population_json(p: &[Particle]) -> String {
    format!(
        "[{}]",
        p.iter()
            .map(|p| format!("{:?}", values(p)))
            .collect::<Vec<_>>()
            .join(",")
    )
}

impl BeliefLab {
    fn record(&mut self) {
        let mut filter_rng = StdRng::seed_from_u64(self.seed);
        let mut sensor_rng = StdRng::seed_from_u64(self.seed ^ 0xb16e_f123_970a_c082);
        let mut pf = ParticleFilterLocalizer::try_with_initial_state_and_rng(
            PFState::new(5., 5., 0.3, 0.),
            ParticleFilterConfig {
                n_particles: N,
                range_noise: self.sigma,
                velocity_noise: 0.65,
                yaw_rate_noise: 0.2,
                dt: STEP,
                resample_threshold: 0.55,
            },
            &mut filter_rng,
        )
        .expect("valid belief experiment");
        let mut truth: [f64; 3] = [5., 5., 0.3];
        self.scans.clear();
        for _ in 0..SCANS {
            let previous_truth = truth;
            let prior = pf.get_particles().to_vec();
            let control = PFControl::new(1.2, 0.22);
            truth[0] += control[0] * truth[2].cos() * STEP;
            truth[1] += control[0] * truth[2].sin() * STEP;
            truth[2] += control[1] * STEP;
            // Always draw all five sensor errors: changing the enabled subset
            // does not change the truth, initialization or shared observations.
            let observations: PFMeasurement = LANDMARKS
                .iter()
                .map(|&[x, y]| {
                    let z: f64 = StandardNormal.sample(&mut sensor_rng);
                    (
                        ((truth[0] - x).hypot(truth[1] - y) + z * self.sigma).max(0.),
                        x,
                        y,
                    )
                })
                .take(5)
                .collect();
            pf.try_predict_with_control_and_rng(&control, &mut filter_rng)
                .expect("finite control");
            let predicted = pf.get_particles().to_vec();
            let observations = observations[..self.landmarks].to_vec();
            pf.try_update_with_observations(&observations)
                .expect("finite ranges");
            let weighted = pf.get_particles().to_vec();
            let effective = ess(&weighted);
            let parents = pf.resample_with_rng(&mut filter_rng);
            self.scans.push(Scan {
                clouds: [prior, predicted, weighted, pf.get_particles().to_vec()],
                previous_truth,
                truth,
                observations,
                parents,
                ess: effective,
            });
        }
        self.generation += 1;
        self.seek(self.cursor);
    }

    pub(super) fn enter(&mut self) {
        self.active = true;
        self.record();
    }
    pub(super) fn apply_query(&mut self, q: &str) {
        self.active = crate::share::value(q, "loc_view") == Some("belief");
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
        if !self.active {
            return;
        }
        self.seed = crate::share::value(q, "belief_seed")
            .and_then(|v| v.parse().ok())
            .unwrap_or(7);
        self.sigma = crate::share::value(q, "belief_sigma")
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|s| s.is_finite() && (0.1..=0.8).contains(s))
            .unwrap_or(0.25);
        self.landmarks = crate::share::value(q, "belief_landmarks")
            .and_then(|v| v.parse().ok())
            .filter(|n| [1, 2, 5].contains(n))
            .unwrap_or(5);
        self.cursor = crate::share::bounded_usize(q, "belief_at", LAST).unwrap_or(2);
        self.selected = crate::share::bounded_usize(q, "belief_particle", N - 1).unwrap_or(0);
        self.ranges = crate::share::boolean(q, "belief_ranges").unwrap_or(true);
        self.record();
        if crate::share::value(q, "belief_event") == Some("resample") {
            self.cursor = self
                .scans
                .iter()
                .position(|s| s.parents.is_some())
                .unwrap_or(0)
                * 4
                + 2;
        }
    }

    pub(super) fn share_query(&self) -> String {
        format!("tab=localization&loc_view=belief&belief_seed={}&belief_sigma={}&belief_landmarks={}&belief_at={}&belief_particle={}&belief_ranges={}",self.seed,self.sigma,self.landmarks,self.cursor,self.selected,u8::from(self.ranges))
    }
    fn scan(&self) -> &Scan {
        &self.scans[self.cursor / 4]
    }
    fn cloud(&self) -> &[Particle] {
        &self.scan().clouds[self.cursor % 4]
    }
    fn truth(&self) -> [f64; 3] {
        if self.cursor % 4 == 0 {
            self.scan().previous_truth
        } else {
            self.scan().truth
        }
    }
    fn seek(&mut self, cursor: usize) {
        self.cursor = cursor.min(LAST);
        self.playing = false;
        self.clock = None;
        self.pending = 0.;
    }
    fn advance(&mut self, now: f64) {
        let elapsed = self.clock.replace(now).map_or(0., |last| now - last);
        if !self.playing || !(0.0..1.0).contains(&elapsed) {
            self.pending = 0.;
            return;
        }
        self.pending += elapsed * 2.;
        let steps = (self.pending + 1e-9).floor() as usize;
        self.pending = (self.pending - steps as f64).max(0.);
        self.cursor = (self.cursor + steps).min(LAST);
        if self.cursor == LAST {
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
        }
    }
    pub(super) fn widget(&mut self, name: &'static str, r: &Response) {
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
            .map(|(k, h)| format!("\"{k}\":{h}"))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = ctx.input(|i| i.pointer.hover_pos()).map(|p| [p.x, p.y]);
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
        ] {
            let _ = c.set_attribute(&format!("data-belief-{k}"), &v);
        }
        if !active || self.scans.is_empty() {
            return;
        }
        let scan = self.scan();
        let clouds = scan
            .clouds
            .iter()
            .map(|c| population_json(c))
            .collect::<Vec<_>>()
            .join(",");
        let observations = scan
            .observations
            .iter()
            .map(|&(r, x, y)| format!("[{r},{x},{y}]"))
            .collect::<Vec<_>>()
            .join(",");
        let rect = |r: Option<Rect>| {
            r.map_or("null".into(), |r| {
                format!("{:?}", [r.left(), r.top(), r.width(), r.height()])
            })
        };
        for (k, v) in [
            ("seed", self.seed.to_string()),
            ("sigma", self.sigma.to_string()),
            ("landmarks", self.landmarks.to_string()),
            ("cursor", self.cursor.to_string()),
            ("last", LAST.to_string()),
            ("phase", (self.cursor % 4).to_string()),
            ("selected", self.selected.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("ranges", u8::from(self.ranges).to_string()),
            ("generation", self.generation.to_string()),
            ("clouds", format!("[{clouds}]")),
            ("truth", format!("{:?}", self.truth())),
            ("observations", format!("[{observations}]")),
            ("ess", scan.ess.to_string()),
            (
                "parents",
                scan.parents
                    .as_ref()
                    .map_or("null".into(), |p| format!("{p:?}")),
            ),
            (
                "timeline",
                format!("{:?}", self.scans.iter().map(|s| s.ess).collect::<Vec<_>>()),
            ),
            ("map", rect(self.map)),
            ("graph", rect(self.graph)),
            ("witnesses", self.witnesses.clone()),
            ("bars", self.bars.clone()),
            (
                "painted",
                self.painted
                    .map_or("null".into(), |(g, c, s, r)| format!("[{g},{c},{s},{r}]")),
            ),
            ("share", self.share_query()),
        ] {
            let _ = c.set_attribute(&format!("data-belief-{k}"), &v);
        }
        let _ = c.set_attribute(
            "aria-label",
            &format!(
                "Belief update lab, scan {}, {}, particle {}, {}",
                self.cursor / 4 + 1,
                PHASES[self.cursor % 4],
                self.selected,
                if self.playing { "playing" } else { "paused" }
            ),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lab(q: &str) -> BeliefLab {
        let mut d = BeliefLab::default();
        d.apply_query(&format!("tab=localization&loc_view=belief&{q}"));
        d
    }

    #[test]
    fn recorded_weights_and_ancestry_follow_actual_bayes_and_systematic_resampling() {
        let d = lab("");
        let mut resampled = 0;
        for (i, s) in d.scans.iter().enumerate() {
            let [prior, predict, weight, output] = &s.clouds;
            if i > 0 {
                assert_eq!(
                    population_json(prior),
                    population_json(&d.scans[i - 1].clouds[3])
                );
            }
            let raw = predict
                .iter()
                .map(|p| {
                    s.observations.iter().fold(p.w, |w, &(r, x, y)| {
                        let e = r - (p.x - x).hypot(p.y - y);
                        w * (-0.5 * (e / d.sigma).powi(2)).exp()
                            / (d.sigma * (2. * std::f64::consts::PI).sqrt())
                    })
                })
                .collect::<Vec<_>>();
            let total = raw.iter().sum::<f64>();
            assert!(total > 0.);
            for j in 0..N {
                assert!((weight[j].w - raw[j] / total).abs() < 1e-12);
                assert_eq!(weight[j].to_state(), predict[j].to_state());
                assert!(
                    (predict[j].x - prior[j].x - predict[j].v * prior[j].yaw.cos() * STEP).abs()
                        < 1e-12
                );
                assert!(
                    (predict[j].y - prior[j].y - predict[j].v * prior[j].yaw.sin() * STEP).abs()
                        < 1e-12
                );
            }
            assert_eq!(s.parents.is_some(), s.ess < (N as f64 * 0.55));
            if let Some(a) = &s.parents {
                resampled += 1;
                assert_eq!(a.len(), N);
                assert!(a.windows(2).all(|w| w[0] <= w[1]));
                for (p, &parent) in output.iter().zip(a) {
                    assert_eq!(p.to_state(), weight[parent].to_state());
                    assert_eq!(p.w, 1. / N as f64);
                }
                assert!((ess(output) - N as f64).abs() < 1e-10);
            } else {
                assert_eq!(population_json(output), population_json(weight));
            }
        }
        assert!(resampled > 0);
    }

    #[test]
    fn landmark_subsets_preserve_truth_initial_cloud_and_common_sensor_errors() {
        let a = lab("belief_landmarks=1");
        let b = lab("belief_landmarks=5");
        for (a, b) in a.scans.iter().zip(&b.scans) {
            assert_eq!(a.truth, b.truth);
            assert_eq!(a.previous_truth, b.previous_truth);
            assert_eq!(a.observations[0], b.observations[0]);
        }
        for phase in 0..2 {
            assert_eq!(
                population_json(&a.scans[0].clouds[phase]),
                population_json(&b.scans[0].clouds[phase])
            );
        }
        assert_ne!(
            population_json(&a.scans[0].clouds[2]),
            population_json(&b.scans[0].clouds[2])
        );
    }

    #[test]
    fn exact_share_restores_all_recorded_populations_and_pauses() {
        let mut d=lab("belief_seed=1234567&belief_sigma=0.54321&belief_landmarks=2&belief_at=47&belief_particle=35&belief_ranges=0");
        d.playing = true;
        let restored = lab(&d.share_query());
        assert_eq!(restored.share_query(), d.share_query());
        assert!(!restored.playing);
        for (a, b) in d.scans.iter().zip(&restored.scans) {
            assert_eq!(a.truth, b.truth);
            assert_eq!(a.observations, b.observations);
            assert_eq!(a.parents, b.parents);
            for i in 0..4 {
                assert_eq!(population_json(&a.clouds[i]), population_json(&b.clouds[i]));
            }
        }
        d.apply_query("tab=localization&filter=ekf");
        assert!(!d.active);
        let invalid = lab(
            "belief_sigma=NaN&belief_landmarks=4&belief_at=999&belief_particle=96&belief_seed=bad",
        );
        assert_eq!(invalid.sigma, 0.25);
        assert_eq!(invalid.landmarks, 5);
        assert_eq!(invalid.cursor, 2);
        assert_eq!(invalid.selected, 0);
        assert_eq!(invalid.seed, 7);
    }

    #[test]
    fn seeking_and_playback_only_read_recorded_phases_independent_of_paint_rate() {
        for (frames, dt) in [(1000, 0.01), (250, 0.04), (25, 0.4)] {
            let mut d = lab("belief_at=0");
            let g = d.generation;
            let p = population_json(&d.scans[5].clouds[0]);
            d.playing = true;
            d.advance(10.);
            for i in 1..=frames {
                d.advance(10. + i as f64 * dt);
            }
            assert_eq!(d.cursor, 20);
            assert_eq!(d.generation, g);
            assert_eq!(population_json(d.cloud()), p);
            d.seek(11);
            d.advance(200.);
            assert_eq!(d.cursor, 11);
            assert!(!d.playing);
            d.seek(LAST - 1);
            d.playing = true;
            d.advance(300.);
            d.advance(300.8);
            assert_eq!(d.cursor, LAST);
            assert!(!d.playing);
        }
    }

    #[test]
    fn a_new_seed_changes_hypotheses_and_measurements_but_not_the_true_trajectory() {
        let a = lab("belief_seed=7");
        let b = lab("belief_seed=8");
        assert_ne!(population_json(a.cloud()), population_json(b.cloud()));
        assert_ne!(a.scans[0].observations, b.scans[0].observations);
        assert_eq!(
            a.scans.iter().map(|s| s.truth).collect::<Vec<_>>(),
            b.scans.iter().map(|s| s.truth).collect::<Vec<_>>()
        );
        let event = lab("belief_event=resample");
        assert_eq!(event.cursor % 4, 2);
        assert!(event.scan().parents.is_some());
    }
}
