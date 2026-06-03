//! Sweep benchmarks for hierarchical MAPF replanning.
//!
//! This complements `benchmark_hierarchical_mapf_scale` with two deterministic
//! sweeps:
//!
//! 1. A region-size sweep over a fixed scene of spatially separated local swaps.
//!    It shows how coarse region granularity trades repair-group size against
//!    region triggers: small regions keep each swap a 2-agent group, while large
//!    regions merge neighboring swaps into bigger CBS groups.
//! 2. A density sweep that adds more local swap clusters, with a *bounded*
//!    flat-CBS baseline. Flat CBS is run only on a small capped subset of agents
//!    (it does not scale to the full scene), so its runtime is a reference point
//!    against the hierarchical planner that decomposes the whole scene.
//!
//! The scenes are deliberately *local*: hierarchical MAPF only pays off when
//! conflicts cluster in space, so a uniformly random dense scene (one global
//! conflict component) is the wrong test. Each cluster is a head-on horizontal
//! swap with vertical detour room on the open grid; seeded jitter varies the
//! column so the scenes are randomized but reproducible. Randomness is a
//! self-contained splitmix64 PRNG with fixed seeds, so the scenarios and every
//! structural metric (group sizes, conflicts, costs) are deterministic across
//! runs and machines; only the measured wall-clock `*_ms` columns vary. (See the
//! BIT* lesson in `plan.md`: never depend on a nondeterministic `thread_rng`.)

use std::fmt::Write as _;
use std::path::Path;
use std::time::Instant;

use rust_robotics::planning::{
    cell_conflict_count, HierarchicalMapfAgent2D, HierarchicalMapfConfig2D, HierarchicalMapfPlan2D,
    HierarchicalMapfPlanner2D, StlCbsAgent, StlCbsConfig, StlCbsPlanner,
};
use rust_robotics::prelude::*;

const REGION_CSV: &str = "docs/assets/hierarchical-mapf-region-sweep.csv";
const REGION_SVG: &str = "docs/assets/hierarchical-mapf-region-sweep.svg";
const DENSITY_CSV: &str = "docs/assets/hierarchical-mapf-density-sweep.csv";
const DENSITY_SVG: &str = "docs/assets/hierarchical-mapf-density-sweep.svg";

const GRID: i32 = 40;
const MAX_TIME: u64 = 64;
const MAX_CBS_NODES: usize = 20_000;
/// Flat CBS is only tractable on a handful of agents; cap the baseline subset.
const FLAT_CBS_SUBSET_CAP: usize = 4;

/// Each local cluster is a two-agent head-on swap of this horizontal span. The
/// open grid leaves vertical room for one agent to detour, so the swap is always
/// resolvable by CBS.
const SWAP_SPAN: i32 = 2;
/// Diagonal spacing between successive cluster anchors. Clusters are placed on a
/// staircase (distinct rows *and* columns) so swaps never share a row and stay
/// non-interfering even when a coarse region merges several into one CBS group.
/// It is larger than `SWAP_SPAN` so a region equal to the spacing isolates each
/// cluster, and a region equal to a multiple of the spacing merges that many.
const CLUSTER_SPACING: i32 = 4;
const ANCHOR_MARGIN: i32 = 1;

/// Region edge lengths. With `CLUSTER_SPACING = 4` these merge 1, 2, 3, and 6
/// clusters into a single CBS group (group sizes 2, 4, 6, 12). Even the largest
/// stays under the node budget because the merged swaps are non-interfering, but
/// flat CBS cost still grows steeply — the motivation for region decomposition.
const REGION_SIZES: [i32; 4] = [4, 8, 12, 24];
const REGION_SWEEP_CLUSTERS: usize = 6;
const REGION_SWEEP_SEED: u64 = 0x5151_2719_ABCD_0001;

const DENSITY_CLUSTERS: [usize; 5] = [2, 4, 6, 8, 9];
const DENSITY_REGION: i32 = 4;
const DENSITY_SEED: u64 = 0x9E37_79B9_7F4A_7C15;

/// Self-contained deterministic PRNG (splitmix64).
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, bound: i32) -> i32 {
        (self.next_u64() % bound.max(1) as u64) as i32
    }
}

/// Anchor cell (left end of the swap) for the cluster at `index`, placed on a
/// diagonal staircase so every swap occupies its own row and column. A small
/// seeded horizontal jitter (kept inside the spare gap) randomizes the scene
/// without letting neighboring swaps touch.
fn cluster_anchor(rng: &mut SplitMix64, index: usize) -> (i32, i32) {
    let step = index as i32;
    let jitter_x = rng.below(CLUSTER_SPACING - SWAP_SPAN);
    (
        ANCHOR_MARGIN + step * CLUSTER_SPACING + jitter_x,
        ANCHOR_MARGIN + step * CLUSTER_SPACING,
    )
}

/// Build `clusters` spatially separated two-agent head-on swaps. Deterministic
/// for a given seed.
fn clustered_swaps(seed: u64, clusters: usize) -> Vec<HierarchicalMapfAgent2D> {
    let mut rng = SplitMix64::new(seed);
    let mut agents = Vec::with_capacity(clusters * 2);
    for index in 0..clusters {
        let (ax, ay) = cluster_anchor(&mut rng, index);
        let left = (ax, ay);
        let right = (ax + SWAP_SPAN, ay);
        let id = index * 2;
        agents.push(HierarchicalMapfAgent2D::new(id, left, right));
        agents.push(HierarchicalMapfAgent2D::new(id + 1, right, left));
    }
    agents
}

fn open_planner(grid: i32, region: i32) -> RoboticsResult<HierarchicalMapfPlanner2D> {
    HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
        max_time: MAX_TIME,
        max_cbs_nodes: MAX_CBS_NODES,
        ..HierarchicalMapfConfig2D::new(grid, grid, region, region)
    })
}

fn max_group_size(plan: &HierarchicalMapfPlan2D) -> usize {
    plan.replanned_groups
        .iter()
        .map(|group| group.agent_ids.len())
        .max()
        .unwrap_or(0)
}

// --------------------------------------------------------------------------
// Region-size sweep
// --------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct RegionSample {
    region: i32,
    elapsed_ms: f64,
    region_conflicts: usize,
    replanned_groups: usize,
    max_group_size: usize,
    final_conflicts: usize,
    fallback_full_replan: bool,
    total_cost: u64,
}

fn run_region_sweep() -> RoboticsResult<Vec<RegionSample>> {
    let agents = clustered_swaps(REGION_SWEEP_SEED, REGION_SWEEP_CLUSTERS);
    let mut samples = Vec::new();
    for region in REGION_SIZES {
        let planner = open_planner(GRID, region)?;
        let start = Instant::now();
        let plan = planner.plan(&agents)?;
        let elapsed_ms = start.elapsed().as_secs_f64() * 1_000.0;
        samples.push(RegionSample {
            region,
            elapsed_ms,
            region_conflicts: plan.region_conflicts.len(),
            replanned_groups: plan.replanned_groups.len(),
            max_group_size: max_group_size(&plan),
            final_conflicts: plan.final_cell_conflicts,
            fallback_full_replan: plan.fallback_full_replan,
            total_cost: plan.total_cost,
        });
    }
    Ok(samples)
}

fn render_region_csv(samples: &[RegionSample]) -> String {
    let mut csv = String::from(
        "region_size,elapsed_ms,region_conflicts,replanned_groups,max_group_size,final_conflicts,fallback_full_replan,total_cost\n",
    );
    for s in samples {
        writeln!(
            csv,
            "{}x{},{:.3},{},{},{},{},{},{}",
            s.region,
            s.region,
            s.elapsed_ms,
            s.region_conflicts,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan,
            s.total_cost
        )
        .unwrap();
    }
    csv
}

fn render_region_svg(samples: &[RegionSample]) -> String {
    let max_group = samples
        .iter()
        .map(|s| s.max_group_size)
        .max()
        .unwrap_or(1)
        .max(1) as f64;
    let max_ms = samples
        .iter()
        .map(|s| s.elapsed_ms)
        .fold(0.0_f64, f64::max)
        .max(1.0);

    let slot = 532.0 / samples.len() as f64;
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="760" height="470" viewBox="0 0 760 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Hierarchical MAPF region-size sweep</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Repair-group size and planning time as the coarse region granularity grows on a fixed scene of {REGION_SWEEP_CLUSTERS} local swaps.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="760" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Hierarchical MAPF region-size sweep</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Coarser regions merge agents into larger CBS repair groups.</text>"##
    )
    .unwrap();

    // Axis baseline.
    writeln!(
        svg,
        r##"<line x1="84" y1="342" x2="616" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="84" y1="128" x2="84" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="350" y="392" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">region size</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="26" y="236" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700" transform="rotate(-90 26 236)">max repair-group size</text>"##
    )
    .unwrap();

    let mut time_line = String::new();
    for (index, s) in samples.iter().enumerate() {
        let center = 84.0 + slot * (index as f64 + 0.5);
        let bar_height = s.max_group_size as f64 / max_group * 214.0;
        let y = 342.0 - bar_height;
        let fill = if s.fallback_full_replan {
            "#dc2626"
        } else {
            "#2563eb"
        };
        writeln!(
            svg,
            r##"<rect x="{:.1}" y="{y:.1}" width="46" height="{bar_height:.1}" rx="4" fill="{fill}" opacity="0.86"/>"##,
            center - 23.0
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{center:.1}" y="{:.1}" text-anchor="middle" fill="#0f172a" font-family="Consolas, Menlo, monospace" font-size="12">{}</text>"##,
            y - 9.0,
            s.max_group_size
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{center:.1}" y="365" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12" font-weight="700">{}x{}</text>"##,
            s.region, s.region
        )
        .unwrap();
        let time_y = 342.0 - s.elapsed_ms / max_ms * 214.0;
        if !time_line.is_empty() {
            time_line.push(' ');
        }
        write!(time_line, "{center:.1},{time_y:.1}").unwrap();
        writeln!(
            svg,
            r##"<circle cx="{center:.1}" cy="{time_y:.1}" r="6.5" fill="#f97316" stroke="#ffffff" stroke-width="2"/>"##
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<polyline points="{time_line}" fill="none" stroke="#f97316" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();

    writeln!(
        svg,
        r##"<rect x="634" y="126" width="12" height="12" rx="2" fill="#2563eb"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="654" y="136" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">max group</text>"##
    )
    .unwrap();
    writeln!(svg, r##"<circle cx="640" cy="160" r="6" fill="#f97316"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="654" y="164" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">time (ms)</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="634" y="180" width="12" height="12" rx="2" fill="#dc2626"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="654" y="190" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">fallback</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="430" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Fixed seed scene; bars turn red if the global full-CBS fallback fired.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    svg
}

// --------------------------------------------------------------------------
// Density sweep with bounded flat-CBS baseline
// --------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct DensitySample {
    agents: usize,
    hier_ms: f64,
    region_conflicts: usize,
    replanned_groups: usize,
    max_group_size: usize,
    final_conflicts: usize,
    fallback_full_replan: bool,
    hier_total_cost: u64,
    flat_subset: usize,
    flat_ms: f64,
    flat_subset_cost: u64,
    flat_subset_conflict_free: bool,
}

/// Run a bounded flat (non-hierarchical) CBS over the first `cap` agents as a
/// scaling reference. Returns (subset_size, elapsed_ms, total_cost, conflict_free).
fn flat_cbs_baseline(
    grid: i32,
    agents: &[HierarchicalMapfAgent2D],
    cap: usize,
) -> RoboticsResult<(usize, f64, u64, bool)> {
    let subset = cap.min(agents.len());
    let cbs_agents = agents
        .iter()
        .take(subset)
        .map(|a| StlCbsAgent::new(a.id, a.start, a.goal))
        .collect::<Vec<_>>();
    let planner = StlCbsPlanner::new(StlCbsConfig {
        max_time: MAX_TIME,
        max_cbs_nodes: MAX_CBS_NODES,
        ..StlCbsConfig::new(grid, grid)
    })?;
    let start = Instant::now();
    let plan = planner.plan(&cbs_agents)?;
    let elapsed_ms = start.elapsed().as_secs_f64() * 1_000.0;
    let conflict_free = cell_conflict_count(&plan.paths, MAX_TIME) == 0;
    Ok((subset, elapsed_ms, plan.total_cost, conflict_free))
}

fn run_density_sweep() -> RoboticsResult<Vec<DensitySample>> {
    let mut samples = Vec::new();
    for (index, &clusters) in DENSITY_CLUSTERS.iter().enumerate() {
        // Vary the seed per density so scenes are independent but reproducible.
        let seed = DENSITY_SEED ^ (index as u64).wrapping_mul(0x1000_0000_0000_0001);
        let agents = clustered_swaps(seed, clusters);
        let count = agents.len();
        let planner = open_planner(GRID, DENSITY_REGION)?;
        let start = Instant::now();
        let plan = planner.plan(&agents)?;
        let hier_ms = start.elapsed().as_secs_f64() * 1_000.0;

        let (flat_subset, flat_ms, flat_subset_cost, flat_subset_conflict_free) =
            flat_cbs_baseline(GRID, &agents, FLAT_CBS_SUBSET_CAP)?;

        samples.push(DensitySample {
            agents: count,
            hier_ms,
            region_conflicts: plan.region_conflicts.len(),
            replanned_groups: plan.replanned_groups.len(),
            max_group_size: max_group_size(&plan),
            final_conflicts: plan.final_cell_conflicts,
            fallback_full_replan: plan.fallback_full_replan,
            hier_total_cost: plan.total_cost,
            flat_subset,
            flat_ms,
            flat_subset_cost,
            flat_subset_conflict_free,
        });
    }
    Ok(samples)
}

fn render_density_csv(samples: &[DensitySample]) -> String {
    let mut csv = String::from(
        "agents,hier_ms,region_conflicts,replanned_groups,max_group_size,final_conflicts,fallback_full_replan,hier_total_cost,flat_subset,flat_ms,flat_subset_cost,flat_subset_conflict_free\n",
    );
    for s in samples {
        writeln!(
            csv,
            "{},{:.3},{},{},{},{},{},{},{},{:.3},{},{}",
            s.agents,
            s.hier_ms,
            s.region_conflicts,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan,
            s.hier_total_cost,
            s.flat_subset,
            s.flat_ms,
            s.flat_subset_cost,
            s.flat_subset_conflict_free
        )
        .unwrap();
    }
    csv
}

fn render_density_svg(samples: &[DensitySample]) -> String {
    let max_ms = samples
        .iter()
        .flat_map(|s| [s.hier_ms, s.flat_ms])
        .fold(0.0_f64, f64::max)
        .max(1.0);

    let slot = 532.0 / samples.len() as f64;
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="760" height="470" viewBox="0 0 760 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Hierarchical MAPF density sweep</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Hierarchical planning time over the whole randomized scene against a bounded flat-CBS baseline run only on a {FLAT_CBS_SUBSET_CAP}-agent subset.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="760" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Hierarchical MAPF density sweep</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Local repair stays cheap as more swap clusters are added.</text>"##
    )
    .unwrap();

    writeln!(
        svg,
        r##"<line x1="84" y1="342" x2="616" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="84" y1="128" x2="84" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="350" y="392" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">agent count</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="26" y="236" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700" transform="rotate(-90 26 236)">planning time ms</text>"##
    )
    .unwrap();

    for (index, s) in samples.iter().enumerate() {
        let center = 84.0 + slot * (index as f64 + 0.5);
        let hier_h = s.hier_ms / max_ms * 214.0;
        let flat_h = s.flat_ms / max_ms * 214.0;
        writeln!(
            svg,
            r##"<rect x="{:.1}" y="{:.1}" width="22" height="{hier_h:.1}" rx="3" fill="#2563eb" opacity="0.88"/>"##,
            center - 24.0,
            342.0 - hier_h
        )
        .unwrap();
        writeln!(
            svg,
            r##"<rect x="{:.1}" y="{:.1}" width="22" height="{flat_h:.1}" rx="3" fill="#10b981" opacity="0.88"/>"##,
            center + 2.0,
            342.0 - flat_h
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{center:.1}" y="365" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12" font-weight="700">{}</text>"##,
            s.agents
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{center:.1}" y="{:.1}" text-anchor="middle" fill="#0f172a" font-family="Consolas, Menlo, monospace" font-size="11">g{}</text>"##,
            342.0 - hier_h - 8.0,
            s.max_group_size
        )
        .unwrap();
    }

    writeln!(
        svg,
        r##"<rect x="634" y="126" width="12" height="12" rx="2" fill="#2563eb"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="654" y="136" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">hierarchical</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="634" y="150" width="12" height="12" rx="2" fill="#10b981"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="654" y="160" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">flat CBS ({FLAT_CBS_SUBSET_CAP})</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="430" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">gN = max repair-group size; flat CBS is bounded to a {FLAT_CBS_SUBSET_CAP}-agent subset baseline.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    svg
}

// --------------------------------------------------------------------------

fn write_text(path: &str, contents: &str) -> RoboticsResult<()> {
    if let Some(parent) = Path::new(path).parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(path, contents)?;
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let region_samples = run_region_sweep()?;
    for s in &region_samples {
        println!(
            "region-sweep: region={}x{} ms={:.2} region_conflicts={} groups={} max_group={} final={} fallback={}",
            s.region,
            s.region,
            s.elapsed_ms,
            s.region_conflicts,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan
        );
    }
    write_text(REGION_CSV, &render_region_csv(&region_samples))?;
    write_text(REGION_SVG, &render_region_svg(&region_samples))?;

    let density_samples = run_density_sweep()?;
    for s in &density_samples {
        println!(
            "density-sweep: agents={} hier_ms={:.2} groups={} max_group={} final={} fallback={} flat_subset={} flat_ms={:.2} flat_conflict_free={}",
            s.agents,
            s.hier_ms,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan,
            s.flat_subset,
            s.flat_ms,
            s.flat_subset_conflict_free
        );
    }
    write_text(DENSITY_CSV, &render_density_csv(&density_samples))?;
    write_text(DENSITY_SVG, &render_density_svg(&density_samples))?;

    println!("wrote {REGION_CSV}, {REGION_SVG}, {DENSITY_CSV}, {DENSITY_SVG}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clustered_swaps_are_deterministic() {
        let a = clustered_swaps(REGION_SWEEP_SEED, REGION_SWEEP_CLUSTERS);
        let b = clustered_swaps(REGION_SWEEP_SEED, REGION_SWEEP_CLUSTERS);
        assert_eq!(a, b);
        assert_eq!(a.len(), REGION_SWEEP_CLUSTERS * 2);
    }

    #[test]
    fn region_sweep_grows_groups_without_fallback() {
        let samples = run_region_sweep().expect("region sweep solves");
        // Coarser regions merge clusters into strictly larger CBS repair groups.
        for pair in samples.windows(2) {
            assert!(
                pair[1].max_group_size >= pair[0].max_group_size,
                "max group size should be monotonic in region size"
            );
        }
        assert_eq!(samples.first().unwrap().max_group_size, 2);
        assert!(samples.last().unwrap().max_group_size > 2);
        // Correct region sizing keeps every scenario locally resolvable.
        for s in &samples {
            assert_eq!(s.final_conflicts, 0);
            assert!(!s.fallback_full_replan);
        }
    }

    #[test]
    fn density_sweep_keeps_groups_minimal_and_conflict_free() {
        let samples = run_density_sweep().expect("density sweep solves");
        for s in &samples {
            // Local decomposition never grows a repair group past one swap pair.
            assert_eq!(s.max_group_size, 2);
            assert_eq!(s.final_conflicts, 0);
            assert!(!s.fallback_full_replan);
            assert!(s.flat_subset_conflict_free);
            assert!(s.flat_subset <= FLAT_CBS_SUBSET_CAP);
        }
    }
}
