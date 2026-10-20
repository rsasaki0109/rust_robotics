//! A frozen, seeded MPPI decision. Candidate ranking and forecast time are
//! independent axes: selecting a sample never advances the robot.

mod view;

use super::{nominal_states, MppiDemo, MppiSampledPlan2D, MppiState2D};
use std::collections::BTreeMap;

pub(super) struct Inspector {
    pub(super) active: bool,
    pub(super) seed: u64,
    selected: usize,
    cursor: usize,
    choose_best: bool,
    order: Vec<usize>,
    generation: usize,
    cloud_json: String,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: usize,
    last_action: &'static str,
    revision: usize,
    map: Option<egui::Rect>,
    charts: [Option<egui::Rect>; 2],
    painted: Option<(usize, usize, usize, bool)>,
    witnesses: String,
    cloud_drawn: usize,
}

impl Default for Inspector {
    fn default() -> Self {
        Self {
            active: false,
            seed: 7,
            selected: 0,
            cursor: 10,
            choose_best: true,
            order: Vec::new(),
            generation: 0,
            cloud_json: String::new(),
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            action_count: 0,
            last_action: "",
            revision: 0,
            map: None,
            charts: [None, None],
            painted: None,
            witnesses: "[]".into(),
            cloud_drawn: 0,
        }
    }
}

impl Inspector {
    pub(super) fn record(&mut self, cloud: &MppiSampledPlan2D) {
        self.order = (0..cloud.rollouts.len()).collect();
        self.order.sort_by(|&a, &b| {
            cloud.rollouts[a]
                .cost
                .total_cmp(&cloud.rollouts[b].cost)
                .then(a.cmp(&b))
        });
        self.selected = if self.choose_best {
            self.order[0]
        } else {
            self.selected.min(cloud.rollouts.len() - 1)
        };
        self.choose_best = false;
        self.cursor = self.cursor.min(cloud.plan.nominal_controls.len());
        self.generation += 1;
        let costs: Vec<_> = cloud.rollouts.iter().map(|r| r.cost).collect();
        let first: Vec<_> = cloud
            .rollouts
            .iter()
            .map(|r| [r.controls[0].ax, r.controls[0].ay])
            .collect();
        let d = cloud.plan.sampling_diagnostics;
        self.cloud_json=format!("{{\"costs\":{costs:?},\"weights\":{:?},\"first\":{first:?},\"order\":{:?},\"ess\":{},\"entropy\":{},\"lambda\":{}}}",cloud.weights,self.order,d.effective_sample_size,d.weight_entropy,d.lambda);
    }

    fn widget(&mut self, name: &'static str, r: &egui::Response) {
        self.actions.insert(name, r.rect.center().into());
        self.hits.insert(name, r.hovered());
    }

    fn ack(&mut self, name: &'static str, ctx: &egui::Context) {
        self.action_count += 1;
        self.last_action = name;
        ctx.request_repaint();
    }

    pub(super) fn button(&mut self, name: &'static str, r: &egui::Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.ack(name, &r.ctx);
            true
        } else {
            false
        }
    }
}

impl MppiDemo {
    pub(super) fn inspector_circle(&mut self) {
        self.start = [5.0, 10.0];
        self.goal = [15.0, 10.0];
        self.obstacles = vec![(10.0, 10.0, 2.0), (10.0, 4.5, 1.2), (10.0, 15.5, 1.2)];
        self.moving = false;
        self.knobs.horizon = 40;
        self.inspector.cursor = 40;
        self.inspector.choose_best = true;
        self.restart();
        self.playing = false;
    }
    pub(super) fn enter_inspector(&mut self) {
        self.inspector.active = true;
        self.inspector.choose_best = true;
        self.restart();
        self.playing = false;
    }

    pub(super) fn apply_inspector_query(&mut self, query: &str) {
        self.inspector.active = crate::share::boolean(query, "mppi_inspect").unwrap_or(false);
        if !self.inspector.active {
            return;
        }
        self.playing = false;
        self.inspector.seed = crate::share::value(query, "mppi_seed")
            .and_then(|v| v.parse().ok())
            .unwrap_or(7);
        self.inspector.selected =
            crate::share::bounded_usize(query, "mppi_sample", self.knobs.samples).unwrap_or(0);
        self.inspector.choose_best = crate::share::value(query, "mppi_sample").is_none();
        self.inspector.cursor = crate::share::bounded_usize(query, "mppi_at", self.knobs.horizon)
            .unwrap_or(10.min(self.knobs.horizon));
        self.show_samples = crate::share::boolean(query, "mppi_cloud").unwrap_or(true);
    }

    pub(super) fn inspector_query(&self) -> String {
        let obstacles = self
            .obstacles
            .iter()
            .map(|&(x, y, r)| format!("{x},{y},{r}"))
            .collect::<Vec<_>>()
            .join(";");
        format!("tab=mppi&mppi_inspect=1&mppi_seed={}&mppi_sample={}&mppi_at={}&mppi_cloud={}&samples={}&horizon={}&lambda={}&sigma={}&moving={}&start={},{}&goal={},{}&obstacles={}",self.inspector.seed,self.inspector.selected,self.inspector.cursor,u8::from(self.show_samples),self.knobs.samples,self.knobs.horizon,self.knobs.lambda,self.knobs.sigma,u8::from(self.moving),self.start[0],self.start[1],self.goal[0],self.goal[1],obstacles)
    }

    fn ensure_inspector(&mut self) {
        if self.cloud.is_none() {
            self.step_preview();
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.inspector.revision += 1;
        let active = active && self.inspector.active;
        if !active {
            self.inspector.painted = None;
        }
        self.publish_inspector(ctx, active);
    }
}

fn state_values(s: MppiState2D) -> [f64; 4] {
    [s.x, s.y, s.vx, s.vy]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mppi::{MppiControl2D, DT};

    fn inspected(lambda: f64) -> MppiDemo {
        let mut d = MppiDemo::default();
        d.apply_share_query(&format!("tab=mppi&mppi_inspect=1&lambda={lambda}&moving=1"));
        d.ensure_inspector();
        d
    }

    #[test]
    fn preview_and_applied_plan_have_one_origin_and_include_the_first_control() {
        let mut preview = inspected(30.0);
        let origin = preview.state;
        let cloud = preview.cloud.as_ref().unwrap();
        let path = nominal_states(origin, cloud);
        assert_eq!(path.len(), preview.knobs.horizon + 1);
        assert_eq!(path[0], origin);
        assert_eq!(path[1], origin.step(cloud.plan.first_control, DT));
        assert!(path[1] != origin);
        for i in 0..preview.knobs.horizon {
            assert_eq!(
                path[i + 1],
                path[i].step(cloud.plan.nominal_controls[i], DT)
            );
        }
        let mut live = MppiDemo::default();
        live.step();
        assert_eq!(live.plan_start, Some(origin));
        assert_eq!(
            live.cloud, preview.cloud,
            "preview must include the same predicted movers as a live solve"
        );
        assert_eq!(live.state, path[1]);
        let cloud = preview.cloud.clone();
        let movers = preview.movers.clone();
        preview.ensure_inspector();
        preview.ensure_inspector();
        assert_eq!(preview.state, origin);
        assert_eq!(preview.cloud, cloud);
        assert_eq!(preview.movers, movers);
        assert_eq!(preview.elapsed, 0.0);
    }

    #[test]
    fn temperature_changes_weights_but_preserves_every_seeded_candidate() {
        let a = inspected(1.0);
        let b = inspected(30.0);
        let c = inspected(100.0);
        let a = a.cloud.unwrap();
        let b = b.cloud.unwrap();
        let c = c.cloud.unwrap();
        assert_eq!(a.rollouts, b.rollouts);
        assert_eq!(b.rollouts, c.rollouts);
        assert_ne!(a.weights, c.weights);
        assert!(
            a.plan.sampling_diagnostics.effective_sample_size
                < b.plan.sampling_diagnostics.effective_sample_size
        );
        assert!(
            b.plan.sampling_diagnostics.effective_sample_size
                < c.plan.sampling_diagnostics.effective_sample_size
        );
    }

    #[test]
    fn every_nominal_control_is_the_actual_normalized_weighted_sample_control() {
        let demo = inspected(30.0);
        let c = demo.cloud.unwrap();
        let beta = c
            .rollouts
            .iter()
            .map(|r| r.cost)
            .fold(f64::INFINITY, f64::min);
        let z = c
            .rollouts
            .iter()
            .map(|r| (-(r.cost - beta) / 30.0).exp())
            .sum::<f64>();
        for (r, w) in c.rollouts.iter().zip(&c.weights) {
            assert!((w - (-(r.cost - beta) / 30.0).exp() / z).abs() < 1e-12);
            assert!((r.stage_costs.iter().sum::<f64>() - r.model_cost).abs() < 1e-8);
        }
        for t in 0..demo.knobs.horizon {
            let expected = c.rollouts.iter().zip(&c.weights).fold(
                MppiControl2D::new(0.0, 0.0),
                |mut a, (r, w)| {
                    a.ax += w * r.controls[t].ax;
                    a.ay += w * r.controls[t].ay;
                    a
                },
            );
            assert!((expected.ax - c.plan.nominal_controls[t].ax).abs() < 1e-12);
            assert!((expected.ay - c.plan.nominal_controls[t].ay).abs() < 1e-12);
        }
    }

    #[test]
    fn shares_restore_precise_frozen_decisions_and_legacy_links_remain_live() {
        let mut d = MppiDemo::default();
        d.apply_share_query("tab=mppi&mppi_inspect=1&mppi_seed=19&mppi_sample=13&mppi_at=11&lambda=12.3456789&sigma=1.23456789&start=2.123456789,3.123456789&goal=17.23456789,16.3456789&obstacles=6.123456789,6.987654321,1.23456789");
        d.ensure_inspector();
        let q = d.share_query();
        let mut restored = MppiDemo::default();
        restored.apply_share_query(&q);
        restored.ensure_inspector();
        assert_eq!(restored.share_query(), q);
        assert_eq!(restored.cloud, d.cloud);
        assert_eq!(restored.plan_start, d.plan_start);
        assert!(!restored.playing);
        restored.apply_share_query("tab=mppi&lambda=30");
        assert!(!restored.inspector.active);
        let mut legacy = MppiDemo::default();
        legacy.apply_share_query("tab=mppi");
        assert!(!legacy.inspector.active);
        assert!(legacy.playing);
    }
}
