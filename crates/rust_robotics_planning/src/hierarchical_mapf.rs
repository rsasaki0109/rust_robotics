//! Hierarchical MAPF replanning foundation.
//!
//! This module adds a region-level trigger layer above STL-CBS. It first plans
//! independent shortest paths, finds coarse region conflicts, then replans only
//! the affected agent groups with CBS.

use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};

use rust_robotics_core::{RoboticsError, RoboticsResult};

use crate::stl_cbs::{StlCbsAgent, StlCbsConfig, StlCbsPath, StlCbsPlanner};

/// Agent query for hierarchical MAPF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HierarchicalMapfAgent2D {
    pub id: usize,
    pub start: (i32, i32),
    pub goal: (i32, i32),
}

impl HierarchicalMapfAgent2D {
    pub fn new(id: usize, start: (i32, i32), goal: (i32, i32)) -> Self {
        Self { id, start, goal }
    }

    fn as_stl_cbs_agent(self) -> StlCbsAgent {
        StlCbsAgent::new(self.id, self.start, self.goal)
    }
}

/// Coarse region id for hierarchical grouping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct HierarchicalMapfRegion2D {
    pub rx: i32,
    pub ry: i32,
}

impl HierarchicalMapfRegion2D {
    pub fn new(rx: i32, ry: i32) -> Self {
        Self { rx, ry }
    }
}

/// Planner configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct HierarchicalMapfConfig2D {
    pub width: i32,
    pub height: i32,
    pub obstacle_map: Vec<Vec<bool>>,
    pub region_width: i32,
    pub region_height: i32,
    pub max_time: u64,
    pub max_cbs_nodes: usize,
}

impl HierarchicalMapfConfig2D {
    pub fn new(width: i32, height: i32, region_width: i32, region_height: i32) -> Self {
        Self {
            width,
            height,
            obstacle_map: vec![vec![false; height.max(0) as usize]; width.max(0) as usize],
            region_width,
            region_height,
            max_time: 96,
            max_cbs_nodes: 4_096,
        }
    }
}

/// Compressed route through coarse regions for one agent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HierarchicalMapfRegionRoute2D {
    pub agent_id: usize,
    pub regions: Vec<HierarchicalMapfRegion2D>,
}

/// A coarse conflict used to trigger local replanning.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HierarchicalMapfRegionConflict2D {
    pub region: HierarchicalMapfRegion2D,
    pub t: u64,
    pub agent_ids: Vec<usize>,
}

/// One group that was replanned with CBS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HierarchicalMapfReplannedGroup2D {
    pub agent_ids: Vec<usize>,
    pub cbs_total_cost: u64,
    pub cbs_conflicts_resolved: usize,
}

/// Hierarchical MAPF plan summary.
#[derive(Debug, Clone, PartialEq)]
pub struct HierarchicalMapfPlan2D {
    pub paths: Vec<StlCbsPath>,
    pub independent_paths: Vec<StlCbsPath>,
    pub region_routes: Vec<HierarchicalMapfRegionRoute2D>,
    pub region_conflicts: Vec<HierarchicalMapfRegionConflict2D>,
    pub replanned_groups: Vec<HierarchicalMapfReplannedGroup2D>,
    pub independent_cell_conflicts: usize,
    pub final_cell_conflicts: usize,
    pub total_cost: u64,
    pub fallback_full_replan: bool,
}

/// Region-triggered MAPF replanner.
#[derive(Debug, Clone, PartialEq)]
pub struct HierarchicalMapfPlanner2D {
    config: HierarchicalMapfConfig2D,
    low_level: StlCbsPlanner,
}

impl HierarchicalMapfPlanner2D {
    pub fn new(config: HierarchicalMapfConfig2D) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let low_level = StlCbsPlanner::new(StlCbsConfig {
            width: config.width,
            height: config.height,
            obstacle_map: config.obstacle_map.clone(),
            max_time: config.max_time,
            max_cbs_nodes: config.max_cbs_nodes,
            allow_wait: true,
        })?;
        Ok(Self { config, low_level })
    }

    pub fn config(&self) -> &HierarchicalMapfConfig2D {
        &self.config
    }

    pub fn region_of_cell(&self, x: i32, y: i32) -> RoboticsResult<HierarchicalMapfRegion2D> {
        if !self.is_in_bounds(x, y) {
            return Err(RoboticsError::InvalidParameter(
                "hierarchical MAPF cell is out of bounds".to_string(),
            ));
        }
        Ok(HierarchicalMapfRegion2D::new(
            x / self.config.region_width,
            y / self.config.region_height,
        ))
    }

    pub fn region_route_for_path(
        &self,
        path: &StlCbsPath,
    ) -> RoboticsResult<HierarchicalMapfRegionRoute2D> {
        if path.waypoints.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "hierarchical MAPF path must contain waypoints".to_string(),
            ));
        }
        let mut regions = Vec::new();
        for waypoint in &path.waypoints {
            let region = self.region_of_cell(waypoint.x, waypoint.y)?;
            if regions.last().copied() != Some(region) {
                regions.push(region);
            }
        }
        Ok(HierarchicalMapfRegionRoute2D {
            agent_id: path.agent_id,
            regions,
        })
    }

    pub fn region_conflicts(
        &self,
        paths: &[StlCbsPath],
    ) -> RoboticsResult<Vec<HierarchicalMapfRegionConflict2D>> {
        let mut conflicts = Vec::new();
        for t in 0..=self.config.max_time {
            let mut occupancy: HashMap<HierarchicalMapfRegion2D, BTreeSet<usize>> = HashMap::new();
            for path in paths {
                let position = path.position_at(t);
                let region = self.region_of_cell(position.x, position.y)?;
                occupancy.entry(region).or_default().insert(path.agent_id);
            }
            for (region, agents) in occupancy {
                if agents.len() > 1 {
                    conflicts.push(HierarchicalMapfRegionConflict2D {
                        region,
                        t,
                        agent_ids: agents.into_iter().collect(),
                    });
                }
            }
        }
        Ok(conflicts)
    }

    pub fn plan(
        &self,
        agents: &[HierarchicalMapfAgent2D],
    ) -> RoboticsResult<HierarchicalMapfPlan2D> {
        validate_agents(agents)?;
        let cbs_agents = agents
            .iter()
            .copied()
            .map(HierarchicalMapfAgent2D::as_stl_cbs_agent)
            .collect::<Vec<_>>();
        let independent_paths = self.low_level.plan_independent(&cbs_agents)?;
        let region_routes = independent_paths
            .iter()
            .map(|path| self.region_route_for_path(path))
            .collect::<RoboticsResult<Vec<_>>>()?;
        let region_conflicts = self.region_conflicts(&independent_paths)?;
        let independent_cell_conflicts =
            cell_conflict_count(&independent_paths, self.config.max_time);

        let mut paths = independent_paths.clone();
        let mut replanned_groups = Vec::new();
        let affected_components = connected_agent_components(&region_conflicts);

        for component in affected_components {
            if component.len() <= 1 {
                continue;
            }
            let group_agents = cbs_agents
                .iter()
                .copied()
                .filter(|agent| component.contains(&agent.id))
                .collect::<Vec<_>>();
            let group_plan = self.low_level.plan(&group_agents)?;
            for replanned_path in &group_plan.paths {
                if let Some(index) = paths
                    .iter()
                    .position(|path| path.agent_id == replanned_path.agent_id)
                {
                    paths[index] = replanned_path.clone();
                }
            }
            replanned_groups.push(HierarchicalMapfReplannedGroup2D {
                agent_ids: sorted_component(&component),
                cbs_total_cost: group_plan.total_cost,
                cbs_conflicts_resolved: group_plan.conflicts_resolved,
            });
        }

        let mut fallback_full_replan = false;
        let mut final_cell_conflicts = cell_conflict_count(&paths, self.config.max_time);
        if final_cell_conflicts > 0 {
            let full_plan = self.low_level.plan(&cbs_agents)?;
            paths = full_plan.paths;
            replanned_groups.push(HierarchicalMapfReplannedGroup2D {
                agent_ids: agents.iter().map(|agent| agent.id).collect(),
                cbs_total_cost: full_plan.total_cost,
                cbs_conflicts_resolved: full_plan.conflicts_resolved,
            });
            fallback_full_replan = true;
            final_cell_conflicts = cell_conflict_count(&paths, self.config.max_time);
        }

        Ok(HierarchicalMapfPlan2D {
            total_cost: paths.iter().map(StlCbsPath::arrival_time).sum(),
            paths,
            independent_paths,
            region_routes,
            region_conflicts,
            replanned_groups,
            independent_cell_conflicts,
            final_cell_conflicts,
            fallback_full_replan,
        })
    }

    fn is_in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.config.width && y < self.config.height
    }
}

pub fn cell_conflict_count(paths: &[StlCbsPath], max_time: u64) -> usize {
    let mut count = 0;
    for t in 0..=max_time {
        for i in 0..paths.len() {
            for j in i + 1..paths.len() {
                let a = paths[i].position_at(t);
                let b = paths[j].position_at(t);
                if a.x == b.x && a.y == b.y {
                    count += 1;
                }
                if t > 0 {
                    let a_prev = paths[i].position_at(t - 1);
                    let b_prev = paths[j].position_at(t - 1);
                    if a_prev.x == b.x && a_prev.y == b.y && b_prev.x == a.x && b_prev.y == a.y {
                        count += 1;
                    }
                }
            }
        }
    }
    count
}

fn connected_agent_components(
    conflicts: &[HierarchicalMapfRegionConflict2D],
) -> Vec<HashSet<usize>> {
    let mut adjacency: HashMap<usize, HashSet<usize>> = HashMap::new();
    for conflict in conflicts {
        for &agent in &conflict.agent_ids {
            adjacency.entry(agent).or_default();
        }
        for i in 0..conflict.agent_ids.len() {
            for j in i + 1..conflict.agent_ids.len() {
                adjacency
                    .entry(conflict.agent_ids[i])
                    .or_default()
                    .insert(conflict.agent_ids[j]);
                adjacency
                    .entry(conflict.agent_ids[j])
                    .or_default()
                    .insert(conflict.agent_ids[i]);
            }
        }
    }

    let mut visited = HashSet::new();
    let mut components = Vec::new();
    for &seed in adjacency.keys() {
        if visited.contains(&seed) {
            continue;
        }
        let mut component = HashSet::new();
        let mut queue = VecDeque::new();
        queue.push_back(seed);
        visited.insert(seed);
        while let Some(agent) = queue.pop_front() {
            component.insert(agent);
            if let Some(neighbors) = adjacency.get(&agent) {
                for &neighbor in neighbors {
                    if visited.insert(neighbor) {
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

fn sorted_component(component: &HashSet<usize>) -> Vec<usize> {
    let mut ids = component.iter().copied().collect::<Vec<_>>();
    ids.sort_unstable();
    ids
}

fn validate_config(config: &HierarchicalMapfConfig2D) -> RoboticsResult<()> {
    if config.width <= 0
        || config.height <= 0
        || config.region_width <= 0
        || config.region_height <= 0
    {
        return Err(RoboticsError::InvalidParameter(
            "hierarchical MAPF dimensions and region sizes must be positive".to_string(),
        ));
    }
    if config.max_time == 0 || config.max_cbs_nodes == 0 {
        return Err(RoboticsError::InvalidParameter(
            "hierarchical MAPF max_time and max_cbs_nodes must be positive".to_string(),
        ));
    }
    if config.obstacle_map.len() != config.width as usize {
        return Err(RoboticsError::InvalidParameter(
            "hierarchical MAPF obstacle_map x-dimension must match width".to_string(),
        ));
    }
    for column in &config.obstacle_map {
        if column.len() != config.height as usize {
            return Err(RoboticsError::InvalidParameter(
                "hierarchical MAPF obstacle_map y-dimension must match height".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_agents(agents: &[HierarchicalMapfAgent2D]) -> RoboticsResult<()> {
    if agents.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "hierarchical MAPF requires at least one agent".to_string(),
        ));
    }
    let mut ids = HashSet::new();
    for agent in agents {
        if !ids.insert(agent.id) {
            return Err(RoboticsError::InvalidParameter(
                "hierarchical MAPF agent ids must be unique".to_string(),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planner() -> HierarchicalMapfPlanner2D {
        HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
            max_time: 18,
            ..HierarchicalMapfConfig2D::new(12, 8, 4, 4)
        })
        .unwrap()
    }

    fn agents() -> Vec<HierarchicalMapfAgent2D> {
        vec![
            HierarchicalMapfAgent2D::new(0, (0, 3), (11, 3)),
            HierarchicalMapfAgent2D::new(1, (11, 3), (0, 3)),
            HierarchicalMapfAgent2D::new(2, (5, 0), (5, 7)),
            HierarchicalMapfAgent2D::new(3, (0, 7), (3, 7)),
        ]
    }

    #[test]
    fn region_routes_are_compressed() {
        let planner = planner();
        let paths = planner
            .low_level
            .plan_independent(
                &agents()
                    .iter()
                    .copied()
                    .map(HierarchicalMapfAgent2D::as_stl_cbs_agent)
                    .collect::<Vec<_>>(),
            )
            .unwrap();
        let route = planner.region_route_for_path(&paths[0]).unwrap();

        assert_eq!(route.agent_id, 0);
        assert!(route.regions.len() < paths[0].waypoints.len());
        assert_eq!(route.regions[0], HierarchicalMapfRegion2D::new(0, 0));
    }

    #[test]
    fn hierarchical_replanning_resolves_cell_conflicts() {
        let planner = planner();
        let plan = planner.plan(&agents()).unwrap();

        assert!(plan.independent_cell_conflicts > 0);
        assert_eq!(plan.final_cell_conflicts, 0);
        assert!(!plan.replanned_groups.is_empty());
        assert!(!plan.region_conflicts.is_empty());
    }
}
