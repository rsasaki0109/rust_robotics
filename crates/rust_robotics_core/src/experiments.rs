//! Shared experiment-contract helpers for cross-package exploratory workflows.
//!
//! This module intentionally stays small. It only carries the reusable pieces
//! that already proved stable across multiple experiment packages:
//! - variant descriptors
//! - sampling plans
//! - source/extensibility metrics
//! - generic reference annotation against a chosen baseline

use std::collections::HashMap;
use std::fs;
use std::hash::Hash;
use std::path::Path;

#[derive(Debug, Clone, Copy)]
pub struct VariantDescriptor {
    pub id: &'static str,
    pub design_style: &'static str,
    pub source_path: &'static str,
    pub knob_count: usize,
    pub reports_dispersion: bool,
}

#[derive(Debug, Clone)]
pub struct ExperimentSamplingPlan {
    pub initial_slots: Vec<usize>,
    pub escalation_slots: Vec<usize>,
    pub escalate_if_vote_split: bool,
    pub escalate_if_ratio_margin_below: Option<f64>,
}

impl ExperimentSamplingPlan {
    pub fn static_slots(slots: Vec<usize>) -> Self {
        Self {
            initial_slots: slots,
            escalation_slots: Vec::new(),
            escalate_if_vote_split: false,
            escalate_if_ratio_margin_below: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SourceMetrics {
    pub code_lines: usize,
    pub comment_lines: usize,
    pub branch_keywords: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct ExtensibilityMetrics {
    pub average_coverage_ratio: f64,
    pub knob_count: usize,
    pub reports_dispersion: bool,
}

#[derive(Debug, Clone)]
pub struct ExperimentVariantReport<T> {
    pub descriptor: VariantDescriptor,
    pub evaluation_runtime_ms: f64,
    pub observations: Vec<T>,
    pub source_metrics: SourceMetrics,
    pub extensibility_metrics: ExtensibilityMetrics,
    pub agreement_vs_reference: Option<f64>,
    pub mean_ratio_error_vs_reference: Option<f64>,
}

pub trait ExperimentObservation {
    type Key: Eq + Hash + Clone;

    fn comparison_key(&self) -> Self::Key;
    fn winner_label(&self) -> &'static str;
    fn ratio_value(&self) -> f64;
    fn coverage_ratio(&self) -> f64;
}

pub fn average_coverage_ratio<T: ExperimentObservation>(observations: &[T]) -> f64 {
    if observations.is_empty() {
        return 0.0;
    }

    observations
        .iter()
        .map(ExperimentObservation::coverage_ratio)
        .sum::<f64>()
        / observations.len() as f64
}

pub fn annotate_against_reference<T>(reports: &mut [ExperimentVariantReport<T>], reference_id: &str)
where
    T: ExperimentObservation,
{
    let Some(reference) = reports
        .iter()
        .find(|report| report.descriptor.id == reference_id)
    else {
        return;
    };

    let reference_lookup: HashMap<T::Key, (&'static str, f64)> = reference
        .observations
        .iter()
        .map(|observation| {
            (
                observation.comparison_key(),
                (observation.winner_label(), observation.ratio_value()),
            )
        })
        .collect();

    for report in reports.iter_mut() {
        let mut compared = 0usize;
        let mut agreements = 0usize;
        let mut total_ratio_error = 0.0f64;
        for observation in &report.observations {
            if let Some((reference_winner, reference_ratio)) =
                reference_lookup.get(&observation.comparison_key())
            {
                compared += 1;
                if observation.winner_label() == *reference_winner {
                    agreements += 1;
                }
                total_ratio_error += (observation.ratio_value() - *reference_ratio).abs();
            }
        }

        if compared > 0 {
            report.agreement_vs_reference = Some(agreements as f64 / compared as f64);
            report.mean_ratio_error_vs_reference = Some(total_ratio_error / compared as f64);
        }
    }
}

pub fn read_source_metrics(path: &Path) -> std::io::Result<SourceMetrics> {
    let content = fs::read_to_string(path)?;
    let mut code_lines = 0usize;
    let mut comment_lines = 0usize;
    let mut branch_keywords = 0usize;

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with("//") {
            comment_lines += 1;
            continue;
        }
        code_lines += 1;
        if trimmed.starts_with("if ")
            || trimmed.starts_with("for ")
            || trimmed.starts_with("while ")
            || trimmed.contains(" match ")
            || trimmed.starts_with("match ")
        {
            branch_keywords += 1;
        }
    }

    Ok(SourceMetrics {
        code_lines,
        comment_lines,
        branch_keywords,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone)]
    struct StubObservation {
        family: &'static str,
        bucket: u32,
        winner: &'static str,
        ratio: f64,
        coverage: f64,
    }

    impl ExperimentObservation for StubObservation {
        type Key = (&'static str, u32);

        fn comparison_key(&self) -> Self::Key {
            (self.family, self.bucket)
        }

        fn winner_label(&self) -> &'static str {
            self.winner
        }

        fn ratio_value(&self) -> f64 {
            self.ratio
        }

        fn coverage_ratio(&self) -> f64 {
            self.coverage
        }
    }

    #[test]
    fn annotate_against_reference_marks_agreement_and_ratio_error() {
        let mut reports = vec![
            ExperimentVariantReport {
                descriptor: VariantDescriptor {
                    id: "full-bucket",
                    design_style: "reference",
                    source_path: "ignored",
                    knob_count: 0,
                    reports_dispersion: true,
                },
                evaluation_runtime_ms: 10.0,
                observations: vec![StubObservation {
                    family: "case",
                    bucket: 10,
                    winner: "A",
                    ratio: 1.2,
                    coverage: 1.0,
                }],
                source_metrics: SourceMetrics {
                    code_lines: 1,
                    comment_lines: 0,
                    branch_keywords: 0,
                },
                extensibility_metrics: ExtensibilityMetrics {
                    average_coverage_ratio: 1.0,
                    knob_count: 0,
                    reports_dispersion: true,
                },
                agreement_vs_reference: None,
                mean_ratio_error_vs_reference: None,
            },
            ExperimentVariantReport {
                descriptor: VariantDescriptor {
                    id: "candidate",
                    design_style: "candidate",
                    source_path: "ignored",
                    knob_count: 0,
                    reports_dispersion: false,
                },
                evaluation_runtime_ms: 5.0,
                observations: vec![StubObservation {
                    family: "case",
                    bucket: 10,
                    winner: "A",
                    ratio: 1.1,
                    coverage: 0.3,
                }],
                source_metrics: SourceMetrics {
                    code_lines: 1,
                    comment_lines: 0,
                    branch_keywords: 0,
                },
                extensibility_metrics: ExtensibilityMetrics {
                    average_coverage_ratio: 0.3,
                    knob_count: 0,
                    reports_dispersion: false,
                },
                agreement_vs_reference: None,
                mean_ratio_error_vs_reference: None,
            },
        ];

        annotate_against_reference(&mut reports, "full-bucket");

        assert_eq!(reports[1].agreement_vs_reference, Some(1.0));
        assert!(
            (reports[1]
                .mean_ratio_error_vs_reference
                .expect("ratio error should be annotated")
                - 0.1)
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn average_coverage_ratio_uses_observation_contract() {
        let observations = vec![
            StubObservation {
                family: "case",
                bucket: 10,
                winner: "A",
                ratio: 1.0,
                coverage: 0.1,
            },
            StubObservation {
                family: "case",
                bucket: 20,
                winner: "B",
                ratio: 0.9,
                coverage: 0.5,
            },
        ];

        assert!((average_coverage_ratio(&observations) - 0.3).abs() < 1e-9);
    }
}
