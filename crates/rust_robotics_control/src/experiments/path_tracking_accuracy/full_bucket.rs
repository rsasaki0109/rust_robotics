use super::{TrackingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone, Default)]
pub struct FullBucketTrackingAggregation;

impl FullBucketTrackingAggregation {
    pub fn new() -> Self {
        Self
    }
}

impl TrackingAggregationVariant for FullBucketTrackingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "full-bucket",
            design_style: "collector-aggregate",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/path_tracking_accuracy/full_bucket.rs"
            ),
            knob_count: 0,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        (0..total_scenarios).collect()
    }
}
