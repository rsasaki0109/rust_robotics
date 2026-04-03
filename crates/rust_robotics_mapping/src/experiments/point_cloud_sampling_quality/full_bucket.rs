use super::{PointSamplingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone, Default)]
pub struct FullBucketPointSamplingAggregation;

impl FullBucketPointSamplingAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl PointSamplingAggregationVariant for FullBucketPointSamplingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "full-bucket",
            design_style: "exhaustive-reference",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/point_cloud_sampling_quality/full_bucket.rs"
            ),
            knob_count: 0,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        (0..total_scenarios).collect()
    }
}
