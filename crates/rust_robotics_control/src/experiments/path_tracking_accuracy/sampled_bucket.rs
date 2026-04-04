use super::{TrackingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct SampledBucketTrackingAggregation {
    slots: Vec<usize>,
}

impl SampledBucketTrackingAggregation {
    pub fn new(slots: Vec<usize>) -> Self {
        Self { slots }
    }
}

impl TrackingAggregationVariant for SampledBucketTrackingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "sampled-bucket",
            design_style: "configurable-pipeline",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/path_tracking_accuracy/sampled_bucket.rs"
            ),
            knob_count: 1,
            reports_dispersion: false,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        let mut slots = self
            .slots
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
}
