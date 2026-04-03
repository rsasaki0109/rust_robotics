use rust_robotics_planning::experiments::moving_ai_runtime::{
    current_runtime_process_problem, run_variant_suite, FullBucketRuntimeAggregation,
    RuntimeAggregationVariant, RuntimeEvaluationConfig, RuntimeVariantReport,
    VarianceTriggeredRuntimeAggregation,
};

fn main() {
    let problem = current_runtime_process_problem();
    let config = RuntimeEvaluationConfig::default();
    let thresholds = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25];

    println!("| threshold | eval_ms | coverage | agreement | mean_ratio_error | mismatches |");
    println!("| ---: | ---: | ---: | ---: | ---: | --- |");

    for threshold in thresholds {
        let variants: Vec<Box<dyn RuntimeAggregationVariant>> = vec![
            Box::new(VarianceTriggeredRuntimeAggregation::new(
                vec![0, 4, 9],
                threshold,
            )),
            Box::new(FullBucketRuntimeAggregation::new()),
        ];
        let reports = run_variant_suite(&variants, &problem, config);
        let candidate = reports
            .iter()
            .find(|report| report.descriptor.id == "variance-triggered")
            .expect("variance-triggered report should exist");
        let reference = reports
            .iter()
            .find(|report| report.descriptor.id == "full-bucket")
            .expect("full-bucket report should exist");
        let mismatches = winner_mismatches(candidate, reference);

        println!(
            "| {:.2} | {:.2} | {:.3} | {:.3} | {:.3} | {} |",
            threshold,
            candidate.evaluation_runtime_ms,
            candidate.extensibility_metrics.average_coverage_ratio,
            candidate
                .agreement_vs_reference
                .expect("candidate should be annotated"),
            candidate
                .mean_ratio_error_vs_reference
                .expect("candidate should be annotated"),
            if mismatches.is_empty() {
                "-".to_string()
            } else {
                mismatches.join(", ")
            }
        );
    }
}

fn winner_mismatches(
    candidate: &RuntimeVariantReport,
    reference: &RuntimeVariantReport,
) -> Vec<String> {
    let mut mismatches = Vec::new();
    for observation in &candidate.observations {
        let reference_observation = reference
            .observations
            .iter()
            .find(|other| {
                other.family_name == observation.family_name && other.bucket == observation.bucket
            })
            .expect("reference observation should exist");
        if observation.winner() != reference_observation.winner() {
            mismatches.push(format!(
                "{}/{}",
                observation.family_name, observation.bucket
            ));
        }
    }
    mismatches
}
