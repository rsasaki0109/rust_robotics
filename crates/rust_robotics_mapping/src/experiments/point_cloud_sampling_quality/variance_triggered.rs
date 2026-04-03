use super::{PointSamplingAggregationVariant, PointSamplingSamplingPlan, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct VarianceTriggeredPointSamplingAggregation {
    initial_slots: Vec<usize>,
    ratio_margin_threshold: f64,
}

impl VarianceTriggeredPointSamplingAggregation {
    pub fn new(initial_slots: Vec<usize>, ratio_margin_threshold: f64) -> Self {
        Self {
            initial_slots,
            ratio_margin_threshold,
        }
    }
}

impl PointSamplingAggregationVariant for VarianceTriggeredPointSamplingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "variance-triggered",
            design_style: "adaptive-two-stage",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/point_cloud_sampling_quality/variance_triggered.rs"
            ),
            knob_count: 2,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        let mut slots = self
            .initial_slots
            .iter()
            .copied()
            .filter(|slot| *slot < total_scenarios)
            .collect::<Vec<_>>();
        slots.sort_unstable();
        slots.dedup();
        if slots.is_empty() && total_scenarios > 0 {
            vec![0]
        } else {
            slots
        }
    }

    fn sampling_plan(&self, total_scenarios: usize) -> PointSamplingSamplingPlan {
        PointSamplingSamplingPlan {
            initial_slots: self.selected_slots(total_scenarios),
            escalation_slots: (0..total_scenarios).collect(),
            escalate_if_vote_split: true,
            escalate_if_ratio_margin_below: Some(self.ratio_margin_threshold),
        }
    }
}
