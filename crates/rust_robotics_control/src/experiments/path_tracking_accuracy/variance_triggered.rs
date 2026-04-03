use super::{TrackingAggregationVariant, TrackingSamplingPlan, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct VarianceTriggeredTrackingAggregation {
    initial_slots: Vec<usize>,
    ratio_margin_threshold: f64,
}

impl VarianceTriggeredTrackingAggregation {
    pub fn new(initial_slots: Vec<usize>, ratio_margin_threshold: f64) -> Self {
        Self {
            initial_slots,
            ratio_margin_threshold,
        }
    }
}

impl TrackingAggregationVariant for VarianceTriggeredTrackingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "variance-triggered",
            design_style: "adaptive-two-stage",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/path_tracking_accuracy/variance_triggered.rs"
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

    fn sampling_plan(&self, total_scenarios: usize) -> TrackingSamplingPlan {
        TrackingSamplingPlan {
            initial_slots: self.selected_slots(total_scenarios),
            escalation_slots: (0..total_scenarios).collect(),
            escalate_if_vote_split: true,
            escalate_if_ratio_margin_below: Some(self.ratio_margin_threshold),
        }
    }
}
