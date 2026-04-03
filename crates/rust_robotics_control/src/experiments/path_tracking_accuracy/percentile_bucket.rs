use super::{TrackingAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct PercentileBucketTrackingAggregation {
    percentiles: Vec<f64>,
}

impl PercentileBucketTrackingAggregation {
    pub fn new(percentiles: Vec<f64>) -> Self {
        Self { percentiles }
    }
}

impl TrackingAggregationVariant for PercentileBucketTrackingAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "percentile-bucket",
            design_style: "functional-percentile",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/path_tracking_accuracy/percentile_bucket.rs"
            ),
            knob_count: 1,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        if total_scenarios == 0 {
            return Vec::new();
        }

        let mut slots = self
            .percentiles
            .iter()
            .map(|percentile| {
                let clamped = percentile.clamp(0.0, 1.0);
                ((total_scenarios - 1) as f64 * clamped).round() as usize
            })
            .collect::<Vec<_>>();
        slots.sort_unstable();
        slots.dedup();
        slots
    }
}
