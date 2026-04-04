//! Runtime-aggregation experiments for MovingAI planner comparisons.
//!
//! This module treats the *evaluation strategy itself* as the experiment target.
//! The concrete variants all answer the same question:
//! "On the same MovingAI family/bucket windows, how should we aggregate A* vs
//! JPS runtime observations?"
//!
//! The stable core here is intentionally small:
//! - `RuntimeAggregationVariant`
//! - `RuntimeSamplingPlan`
//! - `RuntimeExperimentCase`
//! - `RuntimeObservation`
//! - `RuntimeVariantReport`
//! - `run_variant_suite`
//!
//! Everything else is an experiment that can be discarded.

mod first_scenario;
mod full_bucket;
mod percentile_bucket;
mod sampled_bucket;
mod variance_triggered;

use std::sync::OnceLock;
use std::time::Instant;

use rust_robotics_core::{
    annotate_against_reference as annotate_reference_reports, average_coverage_ratio,
    read_source_metrics, ExperimentObservation, ExperimentSamplingPlan, ExperimentVariantReport,
    ExtensibilityMetrics, Path2D, Point2D, VariantDescriptor,
};

use crate::moving_ai::{MovingAiMap, MovingAiScenario};
use crate::{AStarConfig, AStarPlanner, JPSConfig, JPSPlanner};

pub use first_scenario::FirstScenarioRuntimeAggregation;
pub use full_bucket::FullBucketRuntimeAggregation;
pub use percentile_bucket::PercentileBucketRuntimeAggregation;
pub use sampled_bucket::SampledBucketRuntimeAggregation;
pub use variance_triggered::VarianceTriggeredRuntimeAggregation;

pub type RuntimeSamplingPlan = ExperimentSamplingPlan;

pub trait RuntimeAggregationVariant {
    fn descriptor(&self) -> VariantDescriptor;
    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;

    fn sampling_plan(&self, total_scenarios: usize) -> RuntimeSamplingPlan {
        RuntimeSamplingPlan::static_slots(self.selected_slots(total_scenarios))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RuntimeExperimentCase {
    pub family_name: &'static str,
    pub map_str: &'static str,
    pub scen_str: &'static str,
    pub buckets: &'static [u32],
}

#[derive(Debug, Clone)]
pub struct RuntimeObservation {
    pub family_name: &'static str,
    pub bucket: u32,
    pub total_scenarios: usize,
    pub initial_slots: Vec<usize>,
    pub selected_slots: Vec<usize>,
    pub escalated: bool,
    pub a_star_bucket_median_us: f64,
    pub jps_bucket_median_us: f64,
    pub a_star_min_us: f64,
    pub a_star_max_us: f64,
    pub jps_min_us: f64,
    pub jps_max_us: f64,
    pub jps_wins: usize,
}

impl RuntimeObservation {
    pub fn a_over_j(&self) -> f64 {
        self.a_star_bucket_median_us / self.jps_bucket_median_us
    }

    pub fn winner(&self) -> &'static str {
        if self.a_over_j() > 1.0 {
            "JPS"
        } else {
            "A*"
        }
    }

    pub fn coverage_ratio(&self) -> f64 {
        self.selected_slots.len() as f64 / self.total_scenarios as f64
    }
}

impl ExperimentObservation for RuntimeObservation {
    type Key = (&'static str, u32);

    fn comparison_key(&self) -> Self::Key {
        (self.family_name, self.bucket)
    }

    fn winner_label(&self) -> &'static str {
        RuntimeObservation::winner(self)
    }

    fn ratio_value(&self) -> f64 {
        RuntimeObservation::a_over_j(self)
    }

    fn coverage_ratio(&self) -> f64 {
        RuntimeObservation::coverage_ratio(self)
    }
}

pub type RuntimeVariantReport = ExperimentVariantReport<RuntimeObservation>;

#[derive(Debug, Clone, Copy)]
pub struct RuntimeEvaluationConfig {
    pub iterations_per_scenario: usize,
}

const SYNTHETIC_BUCKETS: &[u32] = &[20, 60, 100];
const SYNTHETIC_SCENARIOS_PER_BUCKET: usize = 10;
const SYNTHETIC_PROBE_COORDS: &[usize] = &[1, 8, 15, 22, 29, 36, 43];

impl Default for RuntimeEvaluationConfig {
    fn default() -> Self {
        Self {
            iterations_per_scenario: 3,
        }
    }
}

struct PreparedExperimentCase {
    family_name: &'static str,
    map: MovingAiMap,
    scenarios: Vec<MovingAiScenario>,
    a_star: AStarPlanner,
    jps: JPSPlanner,
    buckets: &'static [u32],
}

pub fn current_runtime_process_problem() -> Vec<RuntimeExperimentCase> {
    vec![
        RuntimeExperimentCase {
            family_name: "arena2",
            map_str: include_str!("../../../benchdata/moving_ai/dao/arena2.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/dao/arena2.map.scen"),
            buckets: &[86, 87, 88, 89, 90],
        },
        RuntimeExperimentCase {
            family_name: "8room_000",
            map_str: include_str!("../../../benchdata/moving_ai/room/8room_000.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/room/8room_000.map.scen"),
            buckets: &[80, 85, 90],
        },
        RuntimeExperimentCase {
            family_name: "random512-10-0",
            map_str: include_str!("../../../benchdata/moving_ai/random/random512-10-0.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/random/random512-10-0.map.scen"),
            buckets: &[130, 132, 135],
        },
        RuntimeExperimentCase {
            family_name: "Berlin_0_512",
            map_str: include_str!("../../../benchdata/moving_ai/street/Berlin_0_512.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
            buckets: &[120, 140, 160, 186],
        },
    ]
}

pub fn long_tail_runtime_process_problem() -> Vec<RuntimeExperimentCase> {
    vec![
        RuntimeExperimentCase {
            family_name: "arena2",
            map_str: include_str!("../../../benchdata/moving_ai/dao/arena2.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/dao/arena2.map.scen"),
            buckets: &[90],
        },
        RuntimeExperimentCase {
            family_name: "8room_000",
            map_str: include_str!("../../../benchdata/moving_ai/room/8room_000.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/room/8room_000.map.scen"),
            buckets: &[120, 160, 213],
        },
        RuntimeExperimentCase {
            family_name: "random512-10-0",
            map_str: include_str!("../../../benchdata/moving_ai/random/random512-10-0.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/random/random512-10-0.map.scen"),
            buckets: &[140, 177],
        },
        RuntimeExperimentCase {
            family_name: "maze512-1-0",
            map_str: include_str!("../../../benchdata/moving_ai/maze/maze512-1-0.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
            buckets: &[200, 800, 1211],
        },
        RuntimeExperimentCase {
            family_name: "Berlin_0_512",
            map_str: include_str!("../../../benchdata/moving_ai/street/Berlin_0_512.map"),
            scen_str: include_str!("../../../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
            buckets: &[160, 186],
        },
    ]
}

pub fn synthetic_runtime_process_problem() -> Vec<RuntimeExperimentCase> {
    static CASES: OnceLock<Vec<RuntimeExperimentCase>> = OnceLock::new();
    CASES
        .get_or_init(build_synthetic_runtime_process_problem)
        .clone()
}

fn build_synthetic_runtime_process_problem() -> Vec<RuntimeExperimentCase> {
    vec![
        build_synthetic_runtime_case(
            "synthetic_open_50x50",
            "synthetic_open_50x50.map",
            synthetic_open_tiles(),
        ),
        build_synthetic_runtime_case(
            "synthetic_maze_50x50",
            "synthetic_maze_50x50.map",
            synthetic_maze_tiles(),
        ),
        build_synthetic_runtime_case(
            "synthetic_dense_50x50",
            "synthetic_dense_50x50.map",
            synthetic_dense_tiles(),
        ),
    ]
}

fn build_synthetic_runtime_case(
    family_name: &'static str,
    map_name: &'static str,
    tiles: Vec<Vec<char>>,
) -> RuntimeExperimentCase {
    let map_str = leak_string(serialize_synthetic_moving_ai_map(&tiles));
    let scen_str = leak_string(build_synthetic_scenario_payload(map_name, map_str));
    RuntimeExperimentCase {
        family_name,
        map_str,
        scen_str,
        buckets: SYNTHETIC_BUCKETS,
    }
}

fn leak_string(content: String) -> &'static str {
    Box::leak(content.into_boxed_str())
}

fn synthetic_open_tiles() -> Vec<Vec<char>> {
    vec![vec!['.'; 50]; 50]
}

fn synthetic_maze_tiles() -> Vec<Vec<char>> {
    let mut tiles = synthetic_open_tiles();

    let vertical_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45];
    let vertical_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25];
    let vertical_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15];
    for ((x, y), len) in vertical_x
        .iter()
        .zip(vertical_y.iter())
        .zip(vertical_len.iter())
    {
        draw_vertical_bar(&mut tiles, *x as usize, *y as usize, *len as usize);
    }

    let horizontal_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40];
    let horizontal_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45];
    let horizontal_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5];
    for ((x, y), len) in horizontal_x
        .iter()
        .zip(horizontal_y.iter())
        .zip(horizontal_len.iter())
    {
        draw_horizontal_bar(&mut tiles, *x as usize, *y as usize, *len as usize);
    }

    tiles
}

fn synthetic_dense_tiles() -> Vec<Vec<char>> {
    let mut tiles = synthetic_open_tiles();
    for (y, row) in tiles.iter_mut().enumerate() {
        for (x, tile) in row.iter_mut().enumerate() {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash < 18 && !(x <= 1 || y <= 1 || x >= 48 || y >= 48) {
                *tile = '@';
            }
        }
    }
    tiles
}

fn draw_vertical_bar(tiles: &mut [Vec<char>], x: usize, start_y: usize, len: usize) {
    for y in start_y..start_y.saturating_add(len) {
        if let Some(tile) = tiles.get_mut(y).and_then(|row| row.get_mut(x)) {
            *tile = '@';
        }
    }
}

fn draw_horizontal_bar(tiles: &mut [Vec<char>], start_x: usize, y: usize, len: usize) {
    if let Some(row) = tiles.get_mut(y) {
        for x in start_x..start_x.saturating_add(len) {
            if let Some(tile) = row.get_mut(x) {
                *tile = '@';
            }
        }
    }
}

fn serialize_synthetic_moving_ai_map(tiles: &[Vec<char>]) -> String {
    let height = tiles.len();
    let width = tiles.first().map_or(0, Vec::len);
    let mut rows = Vec::with_capacity(height + 4);
    rows.push("type octile".to_string());
    rows.push(format!("height {height}"));
    rows.push(format!("width {width}"));
    rows.push("map".to_string());
    for row in tiles {
        rows.push(row.iter().collect());
    }
    rows.join("\n")
}

#[derive(Debug, Clone)]
struct SyntheticScenarioSeed {
    start_x: usize,
    start_y: usize,
    goal_x: usize,
    goal_y: usize,
    optimal_length: f64,
}

fn build_synthetic_scenario_payload(map_name: &str, map_payload: &str) -> String {
    let map = MovingAiMap::parse_str(map_payload)
        .unwrap_or_else(|_| panic!("{map_name} synthetic map should parse"));
    let planner = AStarPlanner::from_obstacle_points(
        &map.to_obstacles(),
        AStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap_or_else(|_| panic!("A* planner should build for synthetic map {map_name}"));

    let candidates = synthetic_candidate_cells(&map);
    let mut seeds = Vec::new();
    for (index, &(start_x, start_y)) in candidates.iter().enumerate() {
        for &(goal_x, goal_y) in candidates.iter().skip(index + 1) {
            let Ok(path) = planner.plan(
                map.planning_point(start_x, start_y)
                    .expect("synthetic start should be valid"),
                map.planning_point(goal_x, goal_y)
                    .expect("synthetic goal should be valid"),
            ) else {
                continue;
            };
            seeds.push(SyntheticScenarioSeed {
                start_x,
                start_y,
                goal_x,
                goal_y,
                optimal_length: path.total_length(),
            });
        }
    }

    assert!(
        seeds.len() >= SYNTHETIC_BUCKETS.len() * SYNTHETIC_SCENARIOS_PER_BUCKET,
        "synthetic scenario generation for {map_name} needs at least {} solved pairs, got {}",
        SYNTHETIC_BUCKETS.len() * SYNTHETIC_SCENARIOS_PER_BUCKET,
        seeds.len()
    );

    seeds.sort_by(|left, right| {
        left.optimal_length
            .partial_cmp(&right.optimal_length)
            .unwrap()
            .then_with(|| left.start_x.cmp(&right.start_x))
            .then_with(|| left.start_y.cmp(&right.start_y))
            .then_with(|| left.goal_x.cmp(&right.goal_x))
            .then_with(|| left.goal_y.cmp(&right.goal_y))
    });

    let mut lines = vec!["version 1".to_string()];
    for (bucket_index, &bucket_id) in SYNTHETIC_BUCKETS.iter().enumerate() {
        let start = bucket_index * seeds.len() / SYNTHETIC_BUCKETS.len();
        let end = (bucket_index + 1) * seeds.len() / SYNTHETIC_BUCKETS.len();
        let selected =
            evenly_spaced_scenario_seeds(&seeds[start..end], SYNTHETIC_SCENARIOS_PER_BUCKET);
        for seed in selected {
            lines.push(format!(
                "{} {} {} {} {} {} {} {} {:.8}",
                bucket_id,
                map_name,
                map.width,
                map.height,
                seed.start_x,
                seed.start_y,
                seed.goal_x,
                seed.goal_y,
                seed.optimal_length
            ));
        }
    }

    lines.join("\n")
}

fn synthetic_candidate_cells(map: &MovingAiMap) -> Vec<(usize, usize)> {
    let mut cells = Vec::new();
    for &y in SYNTHETIC_PROBE_COORDS {
        for &x in SYNTHETIC_PROBE_COORDS {
            if x >= map.width || y >= map.height {
                continue;
            }
            if map
                .is_passable(x, y)
                .expect("synthetic probe cell should be in-bounds")
            {
                cells.push((x, y));
            }
        }
    }
    cells
}

fn evenly_spaced_scenario_seeds(
    seeds: &[SyntheticScenarioSeed],
    count: usize,
) -> Vec<&SyntheticScenarioSeed> {
    assert!(
        seeds.len() >= count,
        "synthetic bucket needs at least {count} seeds, got {}",
        seeds.len()
    );
    if seeds.len() == count {
        return seeds.iter().collect();
    }
    if count == 1 {
        return vec![&seeds[seeds.len() / 2]];
    }

    let mut selected = Vec::with_capacity(count);
    for index in 0..count {
        let seed_index = index * (seeds.len() - 1) / (count - 1);
        selected.push(&seeds[seed_index]);
    }
    selected
}

pub fn default_runtime_variants() -> Vec<Box<dyn RuntimeAggregationVariant>> {
    vec![
        Box::new(FirstScenarioRuntimeAggregation::new()),
        Box::new(SampledBucketRuntimeAggregation::new(vec![0, 4, 9])),
        Box::new(PercentileBucketRuntimeAggregation::new(vec![
            0.0, 0.25, 0.5, 0.75, 1.0,
        ])),
        Box::new(VarianceTriggeredRuntimeAggregation::new(
            vec![0, 4, 9],
            0.10,
        )),
        Box::new(FullBucketRuntimeAggregation::new()),
    ]
}

pub fn run_variant_suite(
    variants: &[Box<dyn RuntimeAggregationVariant>],
    cases: &[RuntimeExperimentCase],
    config: RuntimeEvaluationConfig,
) -> Vec<RuntimeVariantReport> {
    let prepared_cases: Vec<PreparedExperimentCase> = cases.iter().map(prepare_case).collect();
    let mut reports = Vec::with_capacity(variants.len());

    for variant in variants {
        let started = Instant::now();
        let mut observations = Vec::new();
        for case in &prepared_cases {
            for &bucket in case.buckets {
                observations.push(measure_bucket_observation(
                    &**variant,
                    case,
                    bucket,
                    config.iterations_per_scenario,
                ));
            }
        }
        let descriptor = variant.descriptor();
        let source_metrics = read_source_metrics(std::path::Path::new(descriptor.source_path))
            .expect("experiment source metrics should be readable");
        let average_coverage_ratio = average_coverage_ratio(&observations);
        let extensibility_metrics = ExtensibilityMetrics {
            average_coverage_ratio,
            knob_count: descriptor.knob_count,
            reports_dispersion: descriptor.reports_dispersion,
        };
        reports.push(RuntimeVariantReport {
            descriptor,
            evaluation_runtime_ms: started.elapsed().as_secs_f64() * 1000.0,
            observations,
            source_metrics,
            extensibility_metrics,
            agreement_vs_reference: None,
            mean_ratio_error_vs_reference: None,
        });
    }

    annotate_reference_reports(&mut reports, "full-bucket");
    reports
}

fn prepare_case(case: &RuntimeExperimentCase) -> PreparedExperimentCase {
    let map = MovingAiMap::parse_str(case.map_str)
        .unwrap_or_else(|_| panic!("{} map should parse", case.family_name));
    let scenarios = MovingAiScenario::parse_str(case.scen_str)
        .unwrap_or_else(|_| panic!("{} scenarios should parse", case.family_name));
    let obstacles = map.to_obstacles();
    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap_or_else(|_| panic!("A* planner should build for {}", case.family_name));
    let jps = JPSPlanner::from_obstacle_points(
        &obstacles,
        JPSConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )
    .unwrap_or_else(|_| panic!("JPS planner should build for {}", case.family_name));

    PreparedExperimentCase {
        family_name: case.family_name,
        map,
        scenarios,
        a_star,
        jps,
        buckets: case.buckets,
    }
}

fn measure_bucket_observation(
    variant: &dyn RuntimeAggregationVariant,
    case: &PreparedExperimentCase,
    bucket: u32,
    iterations: usize,
) -> RuntimeObservation {
    let bucket_scenarios: Vec<&MovingAiScenario> = case
        .scenarios
        .iter()
        .filter(|scenario| scenario.bucket == bucket)
        .collect();
    assert!(
        !bucket_scenarios.is_empty(),
        "{} bucket {} should exist",
        case.family_name,
        bucket
    );
    let plan = variant.sampling_plan(bucket_scenarios.len());
    let initial_slots = normalize_slots(bucket_scenarios.len(), &plan.initial_slots);
    assert!(
        !initial_slots.is_empty(),
        "{} bucket {} should select at least one scenario",
        case.family_name,
        bucket
    );

    let mut slot_samples =
        measure_slot_samples(&bucket_scenarios, &initial_slots, case, bucket, iterations);
    let mut selected_slots = initial_slots.clone();
    let mut escalated = false;
    let escalation_slots = normalize_slots(bucket_scenarios.len(), &plan.escalation_slots);

    if should_escalate(&slot_samples, &plan) {
        let additional_slots: Vec<usize> = escalation_slots
            .into_iter()
            .filter(|slot| !selected_slots.contains(slot))
            .collect();
        if !additional_slots.is_empty() {
            slot_samples.extend(measure_slot_samples(
                &bucket_scenarios,
                &additional_slots,
                case,
                bucket,
                iterations,
            ));
            selected_slots.extend(additional_slots);
            selected_slots.sort_unstable();
            escalated = true;
        }
    }

    let a_star_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.a_star_us).collect();
    let jps_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.jps_us).collect();
    let jps_wins = slot_samples
        .iter()
        .filter(|sample| sample.jps_us < sample.a_star_us)
        .count();

    RuntimeObservation {
        family_name: case.family_name,
        bucket,
        total_scenarios: bucket_scenarios.len(),
        initial_slots,
        selected_slots,
        escalated,
        a_star_bucket_median_us: median_value(&a_star_samples),
        jps_bucket_median_us: median_value(&jps_samples),
        a_star_min_us: min_value(&a_star_samples),
        a_star_max_us: max_value(&a_star_samples),
        jps_min_us: min_value(&jps_samples),
        jps_max_us: max_value(&jps_samples),
        jps_wins,
    }
}

#[derive(Debug, Clone, Copy)]
struct SlotRuntimeSample {
    a_star_us: f64,
    jps_us: f64,
}

fn measure_slot_samples(
    bucket_scenarios: &[&MovingAiScenario],
    slots: &[usize],
    case: &PreparedExperimentCase,
    bucket: u32,
    iterations: usize,
) -> Vec<SlotRuntimeSample> {
    let mut samples = Vec::with_capacity(slots.len());
    for slot in slots {
        let scenario = bucket_scenarios[*slot];
        let start = case
            .map
            .planning_point(scenario.start_x, scenario.start_y)
            .unwrap_or_else(|_| panic!("{} start should be valid", case.family_name));
        let goal = case
            .map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .unwrap_or_else(|_| panic!("{} goal should be valid", case.family_name));
        warmup_and_assert(
            &case.a_star,
            start,
            goal,
            scenario.optimal_length,
            case.family_name,
            bucket,
            *slot,
            "A*",
        );
        warmup_and_assert(
            &case.jps,
            start,
            goal,
            scenario.optimal_length,
            case.family_name,
            bucket,
            *slot,
            "JPS",
        );
        let a_star_us = measure_planner_median_us(iterations, || {
            let path = case.a_star.plan(start, goal).unwrap_or_else(|_| {
                panic!("A* should solve {} bucket {}", case.family_name, bucket)
            });
            assert!(
                (path.total_length() - scenario.optimal_length).abs() <= 1e-6,
                "A* path length should match MovingAI optimal on {} bucket {} slot {}",
                case.family_name,
                bucket,
                slot
            );
            path
        });
        let jps_us = measure_planner_median_us(iterations, || {
            let path = case.jps.plan(start, goal).unwrap_or_else(|_| {
                panic!("JPS should solve {} bucket {}", case.family_name, bucket)
            });
            assert!(
                (path.total_length() - scenario.optimal_length).abs() <= 1e-6,
                "JPS path length should match MovingAI optimal on {} bucket {} slot {}",
                case.family_name,
                bucket,
                slot
            );
            path
        });
        samples.push(SlotRuntimeSample { a_star_us, jps_us });
    }
    samples
}

fn should_escalate(slot_samples: &[SlotRuntimeSample], plan: &RuntimeSamplingPlan) -> bool {
    if slot_samples.is_empty() || plan.escalation_slots.is_empty() {
        return false;
    }

    let vote_split = {
        let jps_wins = slot_samples
            .iter()
            .filter(|sample| sample.jps_us < sample.a_star_us)
            .count();
        jps_wins > 0 && jps_wins < slot_samples.len()
    };
    let ratio_close = plan
        .escalate_if_ratio_margin_below
        .map(|threshold| {
            let a_star_samples: Vec<f64> =
                slot_samples.iter().map(|sample| sample.a_star_us).collect();
            let jps_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.jps_us).collect();
            (median_value(&a_star_samples) / median_value(&jps_samples) - 1.0).abs() < threshold
        })
        .unwrap_or(false);

    (plan.escalate_if_vote_split && vote_split) || ratio_close
}

fn normalize_slots(total_scenarios: usize, slots: &[usize]) -> Vec<usize> {
    let mut normalized = slots
        .iter()
        .copied()
        .filter(|slot| *slot < total_scenarios)
        .collect::<Vec<_>>();
    normalized.sort_unstable();
    normalized.dedup();
    normalized
}

#[allow(clippy::too_many_arguments)]
fn warmup_and_assert(
    planner: &impl PlannerRuntime,
    start: Point2D,
    goal: Point2D,
    optimal_length: f64,
    family_name: &str,
    bucket: u32,
    slot: usize,
    label: &str,
) {
    let path = planner.plan_path(start, goal).unwrap_or_else(|_| {
        panic!("{label} should solve {family_name} bucket {bucket} slot {slot}")
    });
    assert!(
        (path.total_length() - optimal_length).abs() <= 1e-6,
        "{label} warmup path should match MovingAI optimal on {family_name} bucket {bucket} slot {slot}"
    );
}

trait PlannerRuntime {
    fn plan_path(&self, start: Point2D, goal: Point2D) -> Result<Path2D, ()>;
}

impl PlannerRuntime for AStarPlanner {
    fn plan_path(&self, start: Point2D, goal: Point2D) -> Result<Path2D, ()> {
        self.plan(start, goal).map_err(|_| ())
    }
}

impl PlannerRuntime for JPSPlanner {
    fn plan_path(&self, start: Point2D, goal: Point2D) -> Result<Path2D, ()> {
        self.plan(start, goal).map_err(|_| ())
    }
}

fn measure_planner_median_us<F>(iterations: usize, mut plan: F) -> f64
where
    F: FnMut() -> Path2D,
{
    let mut samples = Vec::with_capacity(iterations);
    for _ in 0..iterations {
        let started = Instant::now();
        let path = plan();
        std::hint::black_box(path.points.len());
        std::hint::black_box(path.total_length());
        samples.push(started.elapsed().as_secs_f64() * 1_000_000.0);
    }
    median_value(&samples)
}

fn median_value(samples: &[f64]) -> f64 {
    assert!(
        !samples.is_empty(),
        "median_value requires at least one sample"
    );
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    sorted[sorted.len() / 2]
}

fn min_value(samples: &[f64]) -> f64 {
    samples.iter().copied().fold(f64::INFINITY, f64::min)
}

fn max_value(samples: &[f64]) -> f64 {
    samples.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_variant_selects_one_slot() {
        let variant = FirstScenarioRuntimeAggregation::new();
        assert_eq!(variant.selected_slots(10), vec![0]);
    }

    #[test]
    fn sampled_variant_clamps_to_valid_slots() {
        let variant = SampledBucketRuntimeAggregation::new(vec![0, 4, 9]);
        assert_eq!(variant.selected_slots(3), vec![0]);
        assert_eq!(variant.selected_slots(10), vec![0, 4, 9]);
    }

    #[test]
    fn full_variant_selects_everything() {
        let variant = FullBucketRuntimeAggregation::new();
        assert_eq!(variant.selected_slots(4), vec![0, 1, 2, 3]);
    }

    #[test]
    fn percentile_variant_maps_percentiles_to_unique_slots() {
        let variant = PercentileBucketRuntimeAggregation::new(vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(variant.selected_slots(10), vec![0, 2, 5, 7, 9]);
        assert_eq!(variant.selected_slots(1), vec![0]);
    }

    #[test]
    fn variance_variant_builds_adaptive_sampling_plan() {
        let variant = VarianceTriggeredRuntimeAggregation::new(vec![0, 4, 9], 0.15);
        let plan = variant.sampling_plan(10);
        assert_eq!(plan.initial_slots, vec![0, 4, 9]);
        assert_eq!(plan.escalation_slots, (0..10).collect::<Vec<_>>());
        assert!(plan.escalate_if_vote_split);
        assert_eq!(plan.escalate_if_ratio_margin_below, Some(0.15));
    }

    #[test]
    fn adaptive_plan_escalates_on_vote_split() {
        let plan = RuntimeSamplingPlan {
            initial_slots: vec![0, 4, 9],
            escalation_slots: (0..10).collect(),
            escalate_if_vote_split: true,
            escalate_if_ratio_margin_below: None,
        };
        let slot_samples = vec![
            SlotRuntimeSample {
                a_star_us: 10.0,
                jps_us: 9.0,
            },
            SlotRuntimeSample {
                a_star_us: 10.0,
                jps_us: 11.0,
            },
        ];
        assert!(should_escalate(&slot_samples, &plan));
    }

    #[test]
    fn adaptive_plan_escalates_on_small_ratio_margin() {
        let plan = RuntimeSamplingPlan {
            initial_slots: vec![0, 4, 9],
            escalation_slots: (0..10).collect(),
            escalate_if_vote_split: false,
            escalate_if_ratio_margin_below: Some(0.15),
        };
        let slot_samples = vec![
            SlotRuntimeSample {
                a_star_us: 10.0,
                jps_us: 9.8,
            },
            SlotRuntimeSample {
                a_star_us: 10.2,
                jps_us: 10.1,
            },
            SlotRuntimeSample {
                a_star_us: 9.9,
                jps_us: 10.0,
            },
        ];
        assert!(should_escalate(&slot_samples, &plan));
    }
}
