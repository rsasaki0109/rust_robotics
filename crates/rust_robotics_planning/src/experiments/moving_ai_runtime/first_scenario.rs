use super::{RuntimeAggregationVariant, VariantDescriptor};

#[derive(Debug, Default)]
pub struct FirstScenarioRuntimeAggregation;

impl FirstScenarioRuntimeAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl RuntimeAggregationVariant for FirstScenarioRuntimeAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "first-scenario",
            design_style: "direct-imperative",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/moving_ai_runtime/first_scenario.rs"
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
