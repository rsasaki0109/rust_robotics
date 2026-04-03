use super::{AccuracyAggregationVariant, VariantDescriptor};

#[derive(Debug, Default)]
pub struct FirstScenarioAccuracyAggregation;

impl FirstScenarioAccuracyAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl AccuracyAggregationVariant for FirstScenarioAccuracyAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "first-scenario",
            design_style: "direct-imperative",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/ukf_ckf_accuracy/first_scenario.rs"
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
