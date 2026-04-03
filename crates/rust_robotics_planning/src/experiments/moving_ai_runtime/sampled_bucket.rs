use super::{RuntimeAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct SampledBucketRuntimeAggregation {
    sample_slots: Vec<usize>,
}

impl SampledBucketRuntimeAggregation {
    pub fn new(sample_slots: Vec<usize>) -> Self {
        Self { sample_slots }
    }
}

impl RuntimeAggregationVariant for SampledBucketRuntimeAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "sampled-bucket",
            design_style: "configurable-pipeline",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/moving_ai_runtime/sampled_bucket.rs"
            ),
            knob_count: 1,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        self.sample_slots
            .iter()
            .copied()
            .filter(|slot| *slot < total_scenarios)
            .collect()
    }
}
