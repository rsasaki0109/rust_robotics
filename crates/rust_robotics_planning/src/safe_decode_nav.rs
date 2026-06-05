//! SafeDec-lite: STL-shielded constrained decoding for grid navigation.
//!
//! This reproduces the core idea of constrained decoding for safe navigation
//! policies without a learned model: a base navigation *policy* proposes a score
//! for each discrete action, and a *shield* expressed in Signal Temporal Logic
//! (STL) keeps the decoded action sequence safe.
//!
//! - The base policy here is a deterministic greedy goal-seeker — it always
//!   prefers the action that most reduces distance to the goal, which happily
//!   cuts straight through a hazard.
//! - The shield is a set of STL specifications reusing [`crate::stl_cbs`]
//!   primitives: an `always-avoid` geofence over a time interval (a hard
//!   constraint — candidates entering it are pruned) and an `eventually-reach`
//!   goal region (a soft reward that shapes the beam).
//! - [`SafeDecoder::decode`] runs a deterministic constrained beam search and
//!   returns both the greedy (unshielded) path and the shielded path, so the
//!   number of steps where the shield overrode the greedy choice — and the
//!   resulting robustness gain — is directly measurable.
//!
//! Everything is deterministic: ties are broken by action index and the beam is
//! sorted with a stable total order.

use crate::stl_cbs::{
    stl_always_avoid_robustness, stl_eventually_reach_robustness, StlCbsPath, StlRectangle2D,
    StlTimeInterval, StlTimedCell,
};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// A timed STL specification rectangle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimedRegion {
    pub region: StlRectangle2D,
    pub interval: StlTimeInterval,
}

impl TimedRegion {
    pub fn new(region: StlRectangle2D, interval: StlTimeInterval) -> Self {
        Self { region, interval }
    }
}

/// Configuration for the STL-shielded navigation decoder.
#[derive(Debug, Clone, PartialEq)]
pub struct SafeNavConfig {
    pub width: i32,
    pub height: i32,
    /// Static obstacle map indexed `[x][y]`.
    pub blocked: Vec<Vec<bool>>,
    /// Goal cell the base policy seeks.
    pub goal: (i32, i32),
    /// Number of decoded steps.
    pub horizon: usize,
    /// Beam width retained at each decoding step.
    pub beam_width: usize,
    /// `eventually-reach` goal region and the interval it must be reached in.
    pub reach: TimedRegion,
    /// `always-avoid` geofences (hard shield).
    pub avoid: Vec<TimedRegion>,
    /// Weight on the base-policy score.
    pub policy_weight: f64,
    /// Weight on the eventually-reach shaping reward.
    pub reach_weight: f64,
    /// Clearance the shield keeps from every geofence (in cells). With `0.0` the
    /// shield only prunes cells strictly inside a geofence; a positive margin
    /// keeps the decoded path that far outside.
    pub safety_margin: f64,
    /// Allow diagonal moves in the action set.
    pub diagonal: bool,
}

impl SafeNavConfig {
    /// A bounded empty grid with the given goal and reach region.
    pub fn new(
        width: i32,
        height: i32,
        goal: (i32, i32),
        reach: TimedRegion,
    ) -> RoboticsResult<Self> {
        if width <= 0 || height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav grid dimensions must be positive".to_string(),
            ));
        }
        Ok(Self {
            width,
            height,
            blocked: vec![vec![false; height as usize]; width as usize],
            goal,
            horizon: 40,
            beam_width: 16,
            reach,
            avoid: Vec::new(),
            policy_weight: 1.0,
            reach_weight: 0.5,
            safety_margin: 0.0,
            diagonal: true,
        })
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width && y < self.height
    }

    fn is_blocked(&self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return true;
        }
        self.blocked[x as usize][y as usize]
    }
}

/// Result of a shielded decode, paired with the greedy baseline.
#[derive(Debug, Clone, PartialEq)]
pub struct SafeDecodePlan {
    /// Shielded (STL-constrained) decoded path.
    pub shielded_path: Vec<StlTimedCell>,
    /// Greedy, unshielded decoded path (base policy argmax).
    pub greedy_path: Vec<StlTimedCell>,
    /// Eventually-reach robustness of the shielded path (>= 0 means satisfied).
    pub reach_robustness: f64,
    /// Worst always-avoid robustness of the shielded path (>= 0 means safe).
    pub avoid_robustness: f64,
    /// Eventually-reach robustness of the greedy path.
    pub greedy_reach_robustness: f64,
    /// Worst always-avoid robustness of the greedy path (< 0 means it cut
    /// through a geofence).
    pub greedy_avoid_robustness: f64,
    /// Number of steps where the shielded action differs from the greedy action.
    pub interventions: usize,
    /// Whether the shielded path satisfies the reach spec.
    pub reach_satisfied: bool,
    /// Whether the shielded path satisfies every avoid spec.
    pub avoid_satisfied: bool,
}

#[derive(Debug, Clone)]
struct Beam {
    path: Vec<StlTimedCell>,
    score: f64,
}

/// STL-shielded constrained decoder over a grid navigation policy.
#[derive(Debug, Clone)]
pub struct SafeDecoder {
    config: SafeNavConfig,
}

impl SafeDecoder {
    pub fn new(config: SafeNavConfig) -> RoboticsResult<Self> {
        if config.width <= 0 || config.height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav grid dimensions must be positive".to_string(),
            ));
        }
        if config.horizon == 0 {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav horizon must be positive".to_string(),
            ));
        }
        if config.beam_width == 0 {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav beam width must be positive".to_string(),
            ));
        }
        if config.blocked.len() != config.width as usize
            || config
                .blocked
                .iter()
                .any(|c| c.len() != config.height as usize)
        {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav blocked map must match grid dimensions".to_string(),
            ));
        }
        Ok(Self { config })
    }

    pub fn config(&self) -> &SafeNavConfig {
        &self.config
    }

    /// Action set: 4- or 8-connected moves plus a wait.
    fn actions(&self) -> Vec<(i32, i32)> {
        if self.config.diagonal {
            vec![
                (0, 0),
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]
        } else {
            vec![(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
        }
    }

    /// Base-policy score for moving to `(x, y)`: higher is better. Greedy
    /// goal-seeking — the negative Euclidean distance to the goal.
    fn policy_score(&self, x: i32, y: i32) -> f64 {
        let dx = (x - self.config.goal.0) as f64;
        let dy = (y - self.config.goal.1) as f64;
        -(dx * dx + dy * dy).sqrt()
    }

    /// Whether stepping to `(x, y)` at time `t` violates any active geofence.
    fn violates_geofence(&self, x: i32, y: i32, t: u64) -> bool {
        self.config.avoid.iter().any(|spec| {
            t >= spec.interval.start
                && t <= spec.interval.end
                && spec.region.inside_robustness(x as f64, y as f64) > -self.config.safety_margin
        })
    }

    /// Greedy unshielded decode: argmax base policy each step (ties by action
    /// index), respecting only the grid bounds and static obstacles.
    pub fn greedy_decode(&self, start: (i32, i32)) -> Vec<StlTimedCell> {
        let mut path = vec![StlTimedCell::new(start.0, start.1, 0)];
        let mut current = start;
        for step in 0..self.config.horizon {
            let t = (step + 1) as u64;
            let mut best: Option<(f64, (i32, i32))> = None;
            for (dx, dy) in self.actions() {
                let nx = current.0 + dx;
                let ny = current.1 + dy;
                if self.config.is_blocked(nx, ny) {
                    continue;
                }
                let score = self.policy_score(nx, ny);
                if best.is_none() || score > best.unwrap().0 {
                    best = Some((score, (nx, ny)));
                }
            }
            let Some((_, next)) = best else { break };
            current = next;
            path.push(StlTimedCell::new(next.0, next.1, t));
        }
        path
    }

    /// STL-shielded decode via deterministic constrained beam search.
    pub fn decode(&self, start: (i32, i32)) -> RoboticsResult<SafeDecodePlan> {
        if self.config.is_blocked(start.0, start.1) {
            return Err(RoboticsError::InvalidParameter(
                "safe-nav start cell is blocked or out of bounds".to_string(),
            ));
        }

        let mut beams = vec![Beam {
            path: vec![StlTimedCell::new(start.0, start.1, 0)],
            score: 0.0,
        }];

        for step in 0..self.config.horizon {
            let t = (step + 1) as u64;
            let mut next_beams: Vec<Beam> = Vec::new();
            for beam in &beams {
                let last = *beam.path.last().unwrap();
                for (dx, dy) in self.actions() {
                    let nx = last.x + dx;
                    let ny = last.y + dy;
                    if self.config.is_blocked(nx, ny) {
                        continue;
                    }
                    // Hard shield: prune candidates entering an active geofence.
                    if self.violates_geofence(nx, ny, t) {
                        continue;
                    }
                    let reach_shaping = self
                        .config
                        .reach
                        .region
                        .inside_robustness(nx as f64, ny as f64);
                    let step_score = self.config.policy_weight * self.policy_score(nx, ny)
                        + self.config.reach_weight * reach_shaping;
                    let mut path = beam.path.clone();
                    path.push(StlTimedCell::new(nx, ny, t));
                    next_beams.push(Beam {
                        path,
                        score: beam.score + step_score,
                    });
                }
            }

            if next_beams.is_empty() {
                // Shield pruned everything; keep the previous beam (wait in place
                // is always an option unless the cell itself became unsafe).
                break;
            }

            // Deterministic prune: sort by score desc, then by path tail for ties.
            next_beams.sort_by(|a, b| {
                b.score
                    .partial_cmp(&a.score)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| path_key(&a.path).cmp(&path_key(&b.path)))
            });
            next_beams.truncate(self.config.beam_width);
            beams = next_beams;
        }

        // Choose the best beam that satisfies eventually-reach, else best score.
        let mut best_satisfying: Option<&Beam> = None;
        let mut best_any: Option<&Beam> = None;
        for beam in &beams {
            let reach = self.reach_robustness(&beam.path)?;
            if best_any.is_none() || beam.score > best_any.unwrap().score {
                best_any = Some(beam);
            }
            if reach >= 0.0
                && (best_satisfying.is_none() || beam.score > best_satisfying.unwrap().score)
            {
                best_satisfying = Some(beam);
            }
        }
        let chosen = best_satisfying.or(best_any).unwrap().clone();

        let greedy_path = self.greedy_decode(start);
        let reach_robustness = self.reach_robustness(&chosen.path)?;
        let avoid_robustness = self.avoid_robustness(&chosen.path)?;
        let greedy_reach_robustness = self.reach_robustness(&greedy_path)?;
        let greedy_avoid_robustness = self.avoid_robustness(&greedy_path)?;
        let interventions = count_interventions(&chosen.path, &greedy_path);

        Ok(SafeDecodePlan {
            shielded_path: chosen.path,
            greedy_path,
            reach_robustness,
            avoid_robustness,
            greedy_reach_robustness,
            greedy_avoid_robustness,
            interventions,
            reach_satisfied: reach_robustness >= 0.0,
            avoid_satisfied: avoid_robustness >= 0.0,
        })
    }

    fn reach_robustness(&self, path: &[StlTimedCell]) -> RoboticsResult<f64> {
        let wrapped = StlCbsPath {
            agent_id: 0,
            waypoints: path.to_vec(),
        };
        stl_eventually_reach_robustness(
            &wrapped,
            self.config.reach.region,
            self.config.reach.interval,
        )
    }

    fn avoid_robustness(&self, path: &[StlTimedCell]) -> RoboticsResult<f64> {
        if self.config.avoid.is_empty() {
            return Ok(f64::INFINITY);
        }
        let wrapped = StlCbsPath {
            agent_id: 0,
            waypoints: path.to_vec(),
        };
        let mut worst = f64::INFINITY;
        for spec in &self.config.avoid {
            let r = stl_always_avoid_robustness(&wrapped, spec.region, spec.interval)?;
            worst = worst.min(r);
        }
        Ok(worst)
    }
}

/// Integer key for deterministic tie-breaking on a path's cells.
fn path_key(path: &[StlTimedCell]) -> Vec<(i32, i32)> {
    path.iter().map(|c| (c.x, c.y)).collect()
}

/// Count time steps where two paths occupy different cells.
fn count_interventions(a: &[StlTimedCell], b: &[StlTimedCell]) -> usize {
    let n = a.len().min(b.len());
    let mut count = 0;
    for i in 0..n {
        if a[i].x != b[i].x || a[i].y != b[i].y {
            count += 1;
        }
    }
    count + a.len().abs_diff(b.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> StlRectangle2D {
        StlRectangle2D::new(min_x, max_x, min_y, max_y).unwrap()
    }

    fn interval(a: u64, b: u64) -> StlTimeInterval {
        StlTimeInterval::new(a, b).unwrap()
    }

    /// A corridor scenario: goal straight ahead, a hazard right on the
    /// straight-line path so greedy decoding cuts through it.
    fn corridor() -> SafeNavConfig {
        let goal = (10, 0);
        let reach = TimedRegion::new(rect(9.0, 11.0, -1.0, 1.0), interval(0, 40));
        let mut config = SafeNavConfig::new(13, 9, goal, reach).unwrap();
        // Hazard band centered on y=0, between x=4 and x=6.
        config.avoid = vec![TimedRegion::new(rect(4.0, 6.0, -1.5, 1.5), interval(0, 40))];
        config.horizon = 30;
        config
    }

    #[test]
    fn greedy_cuts_through_hazard() {
        let decoder = SafeDecoder::new(corridor()).unwrap();
        let plan = decoder.decode((0, 0)).unwrap();
        // The unshielded greedy policy drives straight through the geofence.
        assert!(
            plan.greedy_avoid_robustness < 0.0,
            "greedy avoid robustness {} should be negative",
            plan.greedy_avoid_robustness
        );
    }

    #[test]
    fn shield_keeps_path_safe_and_reaches_goal() {
        let decoder = SafeDecoder::new(corridor()).unwrap();
        let plan = decoder.decode((0, 0)).unwrap();
        assert!(
            plan.avoid_satisfied,
            "shield avoid robustness {}",
            plan.avoid_robustness
        );
        assert!(plan.avoid_robustness >= 0.0);
        assert!(
            plan.reach_satisfied,
            "shield reach robustness {}",
            plan.reach_robustness
        );
        // The shield had to override the greedy choice at least once.
        assert!(plan.interventions > 0, "no interventions");
    }

    #[test]
    fn decode_is_deterministic() {
        let decoder = SafeDecoder::new(corridor()).unwrap();
        let first = decoder.decode((0, 0)).unwrap();
        let second = decoder.decode((0, 0)).unwrap();
        assert_eq!(first.shielded_path, second.shielded_path);
        assert_eq!(first.interventions, second.interventions);
    }

    #[test]
    fn no_hazard_means_no_intervention() {
        let goal = (8, 0);
        let reach = TimedRegion::new(rect(7.0, 9.0, -1.0, 1.0), interval(0, 30));
        let config = SafeNavConfig::new(11, 7, goal, reach).unwrap();
        let decoder = SafeDecoder::new(config).unwrap();
        let plan = decoder.decode((0, 0)).unwrap();
        // With no geofence, the shielded path matches greedy.
        assert_eq!(plan.interventions, 0);
        assert!(plan.reach_satisfied);
        assert!(plan.avoid_robustness.is_infinite());
    }

    #[test]
    fn rejects_blocked_start() {
        let goal = (5, 0);
        let reach = TimedRegion::new(rect(4.0, 6.0, -1.0, 1.0), interval(0, 20));
        let mut config = SafeNavConfig::new(7, 7, goal, reach).unwrap();
        config.blocked[0][0] = true;
        let decoder = SafeDecoder::new(config).unwrap();
        assert!(decoder.decode((0, 0)).is_err());
    }

    #[test]
    fn rejects_mismatched_blocked_map() {
        let goal = (5, 0);
        let reach = TimedRegion::new(rect(4.0, 6.0, -1.0, 1.0), interval(0, 20));
        let mut config = SafeNavConfig::new(7, 7, goal, reach).unwrap();
        config.blocked.pop();
        assert!(SafeDecoder::new(config).is_err());
    }
}
