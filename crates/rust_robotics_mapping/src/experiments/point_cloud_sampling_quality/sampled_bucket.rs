use super::{PointSamplingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct SampledBucketPointSamplingAggregation {
    slots: Vec<usize>,
}

impl SampledBucketPointSamplingAggregation {
    pub fn new(slots: Vec<usize>) -> Self {
        Self { slots }
    }
}

impl PointSamplingAggregationVariant for SampledBucketPointSamplingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "sampled-bucket",
            design_style: "fixed-window",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs"
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
