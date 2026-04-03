use super::{PointSamplingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct PercentileBucketPointSamplingAggregation {
    percentiles: Vec<f64>,
}

impl PercentileBucketPointSamplingAggregation {
    pub fn new(percentiles: Vec<f64>) -> Self {
        Self { percentiles }
    }
}

impl PointSamplingAggregationVariant for PercentileBucketPointSamplingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "percentile-bucket",
            design_style: "percentile-spread",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs"
            ),
            knob_count: 1,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        if total_scenarios == 0 {
            return Vec::new();
        }

        let last = total_scenarios.saturating_sub(1) as f64;
        let mut slots = self
            .percentiles
            .iter()
            .map(|percentile| (percentile.clamp(0.0, 1.0) * last).round() as usize)
            .collect::<Vec<_>>();
        slots.sort_unstable();
        slots.dedup();
        slots
    }
}
