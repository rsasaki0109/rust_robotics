//! Sweep benchmarks for hierarchical MAPF replanning.
//!
//! This complements `benchmark_hierarchical_mapf_scale` with four deterministic
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
//! 3. An anisotropic region sweep over `(region_width, region_height)` pairs on a
//!    purely horizontal swap row. Because the scene is horizontal, the repair-
//!    group size tracks region *width* and is invariant to region *height* —
//!    `(8, 4)` and `(8, 16)` give the same group size, while `(4, *)` stays
//!    smaller.
//! 4. A fallback-rate sweep: a single adjacent (span-1) edge-swap straddles a
//!    region boundary with probability ~`1/region`, so the region abstraction
//!    misses the vertex-free swap and the planner falls back to full CBS. The
//!    fallback rate falls as regions grow, and every scene still resolves —
//!    giving the explicit fallback-regime data the other sweeps lack.
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

const ANISO_CSV: &str = "docs/assets/hierarchical-mapf-anisotropic-sweep.csv";
const ANISO_SVG: &str = "docs/assets/hierarchical-mapf-anisotropic-sweep.svg";
const FALLBACK_CSV: &str = "docs/assets/hierarchical-mapf-fallback-sweep.csv";
const FALLBACK_SVG: &str = "docs/assets/hierarchical-mapf-fallback-sweep.svg";

/// Anisotropic region edge pairs `(region_width, region_height)`. The scene is a
/// horizontal row of swaps, so containment (and thus spurious group merging)
/// tracks `region_width`; `region_height` barely matters.
const ANISO_REGIONS: [(i32, i32); 5] = [(4, 4), (8, 4), (16, 4), (8, 16), (4, 16)];
const ANISO_SWAPS: usize = 4;
const ANISO_SEED: u64 = 0xA17E_0B5E_2C4D_6F90;

/// Region sizes for the fallback-rate sweep. A single adjacent (span-1) swap
/// straddles a region boundary with probability ~1/region, so small regions miss
/// the edge-swap conflict at the region level and fall back to full CBS.
const FALLBACK_REGIONS: [i32; 5] = [1, 2, 3, 4, 6];
const FALLBACK_TRIALS: usize = 32;
const FALLBACK_SEED: u64 = 0xF0A1_1BAC_3D5E_7C12;

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

// --------------------------------------------------------------------------
// Anisotropic region sweep
// --------------------------------------------------------------------------

fn open_planner_rect(
    grid: i32,
    region_width: i32,
    region_height: i32,
) -> RoboticsResult<HierarchicalMapfPlanner2D> {
    HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
        max_time: MAX_TIME,
        max_cbs_nodes: MAX_CBS_NODES,
        ..HierarchicalMapfConfig2D::new(grid, grid, region_width, region_height)
    })
}

/// A horizontal row of two-agent swaps, all on the same row and spaced in x.
/// Because the scene is purely horizontal, region *width* controls how many
/// neighboring swaps a coarse region spuriously merges into one CBS group, while
/// region *height* makes no difference.
fn horizontal_swaps(count: usize) -> Vec<HierarchicalMapfAgent2D> {
    let y = 4;
    let mut agents = Vec::with_capacity(count * 2);
    for i in 0..count {
        let ax = 1 + i as i32 * 4;
        let left = (ax, y);
        let right = (ax + SWAP_SPAN, y);
        let id = i * 2;
        agents.push(HierarchicalMapfAgent2D::new(id, left, right));
        agents.push(HierarchicalMapfAgent2D::new(id + 1, right, left));
    }
    agents
}

#[derive(Debug, Clone)]
struct AnisoSample {
    region_width: i32,
    region_height: i32,
    replanned_groups: usize,
    max_group_size: usize,
    final_conflicts: usize,
    fallback_full_replan: bool,
}

fn run_anisotropic_sweep() -> RoboticsResult<Vec<AnisoSample>> {
    let agents = horizontal_swaps(ANISO_SWAPS);
    let _ = ANISO_SEED; // scene is deterministic; seed kept for documentation.
    let mut samples = Vec::new();
    for (rw, rh) in ANISO_REGIONS {
        let planner = open_planner_rect(GRID, rw, rh)?;
        let plan = planner.plan(&agents)?;
        samples.push(AnisoSample {
            region_width: rw,
            region_height: rh,
            replanned_groups: plan.replanned_groups.len(),
            max_group_size: max_group_size(&plan),
            final_conflicts: plan.final_cell_conflicts,
            fallback_full_replan: plan.fallback_full_replan,
        });
    }
    Ok(samples)
}

fn render_aniso_csv(samples: &[AnisoSample]) -> String {
    let mut csv = String::from(
        "region_width,region_height,replanned_groups,max_group_size,final_conflicts,fallback_full_replan\n",
    );
    for s in samples {
        writeln!(
            csv,
            "{},{},{},{},{},{}",
            s.region_width,
            s.region_height,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan
        )
        .unwrap();
    }
    csv
}

fn render_aniso_svg(samples: &[AnisoSample]) -> String {
    let max_group = samples
        .iter()
        .map(|s| s.max_group_size)
        .max()
        .unwrap_or(1)
        .max(1) as f64;
    let width = 760.0;
    let height = 430.0;
    let left = 70.0;
    let bottom = 330.0;
    let plot_w = 620.0;
    let plot_h = bottom - 90.0;
    let slot = plot_w / samples.len() as f64;

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0}" height="{height:.0}" viewBox="0 0 {width:.0} {height:.0}">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect width="{width:.0}" height="{height:.0}" fill="#fbfbfd"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{left}" y="40" font-family="sans-serif" font-size="20" fill="#1d1d1f">Hierarchical MAPF anisotropic region sweep</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{left}" y="62" font-family="sans-serif" font-size="13" fill="#6e6e73">horizontal swap row: max repair-group size tracks region width, not region height</text>"##
    )
    .unwrap();
    for (i, s) in samples.iter().enumerate() {
        let cx = left + i as f64 * slot + slot / 2.0;
        let bar_h = s.max_group_size as f64 / max_group * plot_h;
        let bar_w = slot * 0.5;
        let bar_x = cx - bar_w / 2.0;
        let bar_y = bottom - bar_h;
        writeln!(
            svg,
            r##"<rect x="{bar_x:.1}" y="{bar_y:.1}" width="{bar_w:.1}" height="{bar_h:.1}" fill="#0a84ff" rx="3"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{cx:.1}" y="{:.1}" font-family="sans-serif" font-size="13" fill="#1d1d1f" text-anchor="middle">group {}</text>"##,
            bar_y - 8.0,
            s.max_group_size
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{cx:.1}" y="{:.1}" font-family="sans-serif" font-size="13" fill="#1d1d1f" text-anchor="middle">w{}×h{}</text>"##,
            bottom + 24.0,
            s.region_width,
            s.region_height
        )
        .unwrap();
        let note = if s.fallback_full_replan {
            "fallback"
        } else {
            "ok"
        };
        writeln!(
            svg,
            r##"<text x="{cx:.1}" y="{:.1}" font-family="sans-serif" font-size="11" fill="#6e6e73" text-anchor="middle">{} · conf {}</text>"##,
            bottom + 42.0,
            note,
            s.final_conflicts
        )
        .unwrap();
    }
    svg.push_str("</svg>\n");
    svg
}

// --------------------------------------------------------------------------
// Fallback-regime sweep
// --------------------------------------------------------------------------

/// A single adjacent (span-1) head-on swap at a seeded position. The edge swap
/// has no shared vertex, so when its two cells fall in different regions the
/// region abstraction misses it and the planner falls back to full CBS.
fn single_adjacent_swap(seed: u64) -> Vec<HierarchicalMapfAgent2D> {
    let mut rng = SplitMix64::new(seed);
    let x0 = 2 + rng.below(GRID - 6);
    let y = 2 + rng.below(GRID - 5);
    let left = (x0, y);
    let right = (x0 + 1, y);
    vec![
        HierarchicalMapfAgent2D::new(0, left, right),
        HierarchicalMapfAgent2D::new(1, right, left),
    ]
}

#[derive(Debug, Clone)]
struct FallbackSample {
    region: i32,
    trials: usize,
    fallbacks: usize,
    fallback_rate: f64,
    all_resolved: bool,
}

fn run_fallback_sweep() -> RoboticsResult<Vec<FallbackSample>> {
    // Use the same set of seeded scenes for every region, so the only variable
    // is the region size.
    let seeds: Vec<u64> = (0..FALLBACK_TRIALS)
        .map(|t| FALLBACK_SEED ^ (t as u64).wrapping_mul(0x1000_0000_0000_0001))
        .collect();
    let mut samples = Vec::new();
    for region in FALLBACK_REGIONS {
        let planner = open_planner(GRID, region)?;
        let mut fallbacks = 0;
        let mut all_resolved = true;
        for &seed in &seeds {
            let agents = single_adjacent_swap(seed);
            let plan = planner.plan(&agents)?;
            if plan.fallback_full_replan {
                fallbacks += 1;
            }
            if plan.final_cell_conflicts != 0 {
                all_resolved = false;
            }
        }
        samples.push(FallbackSample {
            region,
            trials: FALLBACK_TRIALS,
            fallbacks,
            fallback_rate: fallbacks as f64 / FALLBACK_TRIALS as f64,
            all_resolved,
        });
    }
    Ok(samples)
}

fn render_fallback_csv(samples: &[FallbackSample]) -> String {
    let mut csv = String::from("region_size,trials,fallbacks,fallback_rate,all_resolved\n");
    for s in samples {
        writeln!(
            csv,
            "{},{},{},{:.3},{}",
            s.region, s.trials, s.fallbacks, s.fallback_rate, s.all_resolved
        )
        .unwrap();
    }
    csv
}

fn render_fallback_svg(samples: &[FallbackSample]) -> String {
    let width = 760.0;
    let height = 430.0;
    let left = 70.0;
    let bottom = 330.0;
    let plot_w = 620.0;
    let plot_h = bottom - 90.0;
    let slot = plot_w / samples.len() as f64;

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0}" height="{height:.0}" viewBox="0 0 {width:.0} {height:.0}">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect width="{width:.0}" height="{height:.0}" fill="#fbfbfd"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{left}" y="40" font-family="sans-serif" font-size="20" fill="#1d1d1f">Hierarchical MAPF fallback-rate vs region size</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{left}" y="62" font-family="sans-serif" font-size="13" fill="#6e6e73">adjacent edge-swaps straddle a region boundary with probability ~1/region, forcing a full-CBS fallback (every scene still resolves)</text>"##
    )
    .unwrap();
    // y axis gridlines at 0, 0.5, 1.0.
    for step in 0..=2 {
        let v = step as f64 / 2.0;
        let y = bottom - v * plot_h;
        writeln!(
            svg,
            r##"<line x1="{left}" y1="{y:.1}" x2="{:.1}" y2="{y:.1}" stroke="#e5e5ea"/>"##,
            left + plot_w
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" font-family="sans-serif" font-size="11" fill="#6e6e73" text-anchor="end">{v:.1}</text>"##,
            left - 8.0,
            y + 4.0
        )
        .unwrap();
    }
    for (i, s) in samples.iter().enumerate() {
        let cx = left + i as f64 * slot + slot / 2.0;
        let bar_h = s.fallback_rate * plot_h;
        let bar_w = slot * 0.5;
        let bar_x = cx - bar_w / 2.0;
        let bar_y = bottom - bar_h;
        writeln!(
            svg,
            r##"<rect x="{bar_x:.1}" y="{bar_y:.1}" width="{bar_w:.1}" height="{bar_h:.1}" fill="#ff9f0a" rx="3"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{cx:.1}" y="{:.1}" font-family="sans-serif" font-size="12" fill="#1d1d1f" text-anchor="middle">{:.0}%</text>"##,
            bar_y - 8.0,
            s.fallback_rate * 100.0
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{cx:.1}" y="{:.1}" font-family="sans-serif" font-size="13" fill="#1d1d1f" text-anchor="middle">region {}</text>"##,
            bottom + 24.0,
            s.region
        )
        .unwrap();
    }
    svg.push_str("</svg>\n");
    svg
}

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

    let aniso_samples = run_anisotropic_sweep()?;
    for s in &aniso_samples {
        println!(
            "aniso-sweep: region={}x{} groups={} max_group={} final={} fallback={}",
            s.region_width,
            s.region_height,
            s.replanned_groups,
            s.max_group_size,
            s.final_conflicts,
            s.fallback_full_replan
        );
    }
    write_text(ANISO_CSV, &render_aniso_csv(&aniso_samples))?;
    write_text(ANISO_SVG, &render_aniso_svg(&aniso_samples))?;

    let fallback_samples = run_fallback_sweep()?;
    for s in &fallback_samples {
        println!(
            "fallback-sweep: region={} trials={} fallbacks={} rate={:.2} all_resolved={}",
            s.region, s.trials, s.fallbacks, s.fallback_rate, s.all_resolved
        );
    }
    write_text(FALLBACK_CSV, &render_fallback_csv(&fallback_samples))?;
    write_text(FALLBACK_SVG, &render_fallback_svg(&fallback_samples))?;

    println!(
        "wrote {REGION_CSV}, {REGION_SVG}, {DENSITY_CSV}, {DENSITY_SVG}, {ANISO_CSV}, {ANISO_SVG}, {FALLBACK_CSV}, {FALLBACK_SVG}"
    );
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
    fn anisotropic_group_size_tracks_width_not_height() {
        let samples = run_anisotropic_sweep().expect("anisotropic sweep solves");
        let group_for = |rw: i32, rh: i32| {
            samples
                .iter()
                .find(|s| s.region_width == rw && s.region_height == rh)
                .map(|s| s.max_group_size)
                .unwrap()
        };
        // Same width, different height -> identical repair-group size.
        assert_eq!(group_for(8, 4), group_for(8, 16));
        assert_eq!(group_for(4, 4), group_for(4, 16));
        // Wider regions spuriously merge neighboring swaps into bigger groups.
        assert!(group_for(8, 4) > group_for(4, 4));
        assert!(group_for(16, 4) > group_for(8, 4));
        // Every scenario stays resolvable.
        for s in &samples {
            assert_eq!(s.final_conflicts, 0);
        }
    }

    #[test]
    fn fallback_rate_falls_with_region_and_stays_solvable() {
        let samples = run_fallback_sweep().expect("fallback sweep solves");
        // The smallest region misses every edge swap -> always falls back.
        assert_eq!(samples.first().unwrap().fallback_rate, 1.0);
        // Fallback rate is non-increasing as regions grow to contain the swap.
        for pair in samples.windows(2) {
            assert!(
                pair[1].fallback_rate <= pair[0].fallback_rate + 1e-9,
                "fallback rate should not increase with region size"
            );
        }
        assert!(samples.last().unwrap().fallback_rate < 1.0);
        // Even when the hierarchy falls back, full CBS resolves every scene.
        for s in &samples {
            assert!(s.all_resolved);
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
