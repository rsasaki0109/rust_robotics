use super::{TrackingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone, Default)]
pub struct FirstScenarioTrackingAggregation;

impl FirstScenarioTrackingAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl TrackingAggregationVariant for FirstScenarioTrackingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "first-scenario",
            design_style: "direct-imperative",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/path_tracking_accuracy/first_scenario.rs"
            ),
            knob_count: 0,
            reports_dispersion: false,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        if total_scenarios == 0 {
            Vec::new()
        } else {
            vec![0]
        }
    }
}
