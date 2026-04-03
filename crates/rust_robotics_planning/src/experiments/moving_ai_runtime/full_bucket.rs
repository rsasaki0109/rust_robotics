use super::{RuntimeAggregationVariant, VariantDescriptor};

#[derive(Debug, Default)]
pub struct FullBucketRuntimeAggregation;

impl FullBucketRuntimeAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl RuntimeAggregationVariant for FullBucketRuntimeAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "full-bucket",
            design_style: "collector-aggregate",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/moving_ai_runtime/full_bucket.rs"
            ),
            knob_count: 0,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        (0..total_scenarios).collect()
    }
}
