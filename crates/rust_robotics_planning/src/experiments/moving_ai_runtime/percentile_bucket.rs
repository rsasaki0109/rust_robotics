use super::{RuntimeAggregationVariant, VariantDescriptor};

#[derive(Debug, Clone)]
pub struct PercentileBucketRuntimeAggregation {
    percentiles: Vec<f64>,
}

impl PercentileBucketRuntimeAggregation {
    pub fn new(percentiles: Vec<f64>) -> Self {
        Self { percentiles }
    }
}

impl RuntimeAggregationVariant for PercentileBucketRuntimeAggregation {
    fn descriptor(&self) -> VariantDescriptor {
        VariantDescriptor {
            id: "percentile-bucket",
            design_style: "functional-percentile",
            source_path: concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/experiments/moving_ai_runtime/percentile_bucket.rs"
            ),
            knob_count: 1,
            reports_dispersion: true,
        }
    }

    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize> {
        if total_scenarios == 0 {
            return Vec::new();
        }
        if total_scenarios == 1 {
            return vec![0];
        }

        let max_index = (total_scenarios - 1) as f64;
        let mut slots = self
            .percentiles
            .iter()
            .copied()
            .map(|percentile| percentile.clamp(0.0, 1.0))
            .map(|percentile| (max_index * percentile).round() as usize)
            .collect::<Vec<_>>();
        slots.sort_unstable();
        slots.dedup();
        slots
    }
}
