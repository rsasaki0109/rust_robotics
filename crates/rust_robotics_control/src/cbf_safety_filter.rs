//! PolyMerge-lite: a control-barrier-function safety filter over convex
//! polytope obstacle covers.
//!
//! This reproduces the reusable safety core of polytope-covering work
//! (PolyMerge) without the 3D Gaussian-splat front end: obstacles are convex
//! polygons (the "cover"), and a control-barrier-function (CBF) quadratic-
//! program filter minimally corrects a nominal velocity command so the robot
//! stays outside every obstacle.
//!
//! - [`CbfConvexObstacle2D`] mirrors the half-space representation used by the
//!   rigid-body MIP planner (`a*x + b*y <= c` contains the obstacle interior),
//!   but its [`CbfConvexObstacle2D::barrier`] returns the *true* signed distance
//!   to the convex polygon and the outward gradient, so vertex regions are
//!   handled honestly rather than over-optimistically.
//! - [`CbfSafetyFilter::filter`] solves, for a single-integrator robot
//!   `x_dot = u`, the CBF quadratic program
//!   `min ||u - u_nom||^2  s.t.  grad_o . u >= -alpha * (h_o(x) - r)` for every
//!   obstacle `o`, using an exact 2-D active-set enumeration (at most two
//!   constraints bind in the plane), so the result is deterministic and
//!   correction-minimal.
//! - [`simulate_cbf_navigation`] drives a proportional go-to-goal nominal
//!   controller through the obstacle field with and without the filter and
//!   reports clearance, collisions, and intervention counts.

use rust_robotics_core::{RoboticsError, RoboticsResult};

const EPS: f64 = 1e-9;

/// A half-space `a*x + b*y <= c` containing the obstacle interior, stored with a
/// unit normal so `a`/`b` is the outward direction and the offset is metric.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CbfHalfspace2D {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

/// A convex polygon obstacle (the polytope "cover").
#[derive(Debug, Clone, PartialEq)]
pub struct CbfConvexObstacle2D {
    vertices: Vec<[f64; 2]>,
    halfspaces: Vec<CbfHalfspace2D>,
}

impl CbfConvexObstacle2D {
    /// Build a convex obstacle from polygon vertices (any winding order).
    pub fn polygon(vertices: Vec<[f64; 2]>) -> RoboticsResult<Self> {
        if vertices.len() < 3 {
            return Err(RoboticsError::InvalidParameter(
                "CBF convex obstacle needs at least 3 vertices".to_string(),
            ));
        }
        if !vertices
            .iter()
            .all(|v| v[0].is_finite() && v[1].is_finite())
        {
            return Err(RoboticsError::InvalidParameter(
                "CBF convex obstacle vertices must be finite".to_string(),
            ));
        }
        let area = signed_area(&vertices);
        if area.abs() <= EPS {
            return Err(RoboticsError::InvalidParameter(
                "CBF convex obstacle area must be non-zero".to_string(),
            ));
        }
        // Normalize to counter-clockwise winding.
        let mut vertices = vertices;
        if area < 0.0 {
            vertices.reverse();
        }
        let n = vertices.len();
        let mut halfspaces = Vec::with_capacity(n);
        for i in 0..n {
            let from = vertices[i];
            let to = vertices[(i + 1) % n];
            let dx = to[0] - from[0];
            let dy = to[1] - from[1];
            // Outward normal of a CCW edge is (dy, -dx); normalize it.
            let norm = (dx * dx + dy * dy).sqrt();
            if norm <= EPS {
                return Err(RoboticsError::InvalidParameter(
                    "CBF convex obstacle has a degenerate edge".to_string(),
                ));
            }
            let a = dy / norm;
            let b = -dx / norm;
            let c = a * from[0] + b * from[1];
            halfspaces.push(CbfHalfspace2D { a, b, c });
        }
        Ok(Self {
            vertices,
            halfspaces,
        })
    }

    /// Axis-aligned box obstacle.
    pub fn aabb(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> RoboticsResult<Self> {
        if min_x >= max_x || min_y >= max_y {
            return Err(RoboticsError::InvalidParameter(
                "CBF AABB bounds must be ordered".to_string(),
            ));
        }
        Self::polygon(vec![
            [min_x, min_y],
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y],
        ])
    }

    pub fn vertices(&self) -> &[[f64; 2]] {
        &self.vertices
    }

    /// Whether a point is strictly inside the polygon.
    pub fn contains(&self, point: [f64; 2]) -> bool {
        self.halfspaces
            .iter()
            .all(|h| h.a * point[0] + h.b * point[1] - h.c <= 0.0)
    }

    /// True signed distance to the polygon boundary (positive outside, negative
    /// inside) together with the unit outward gradient at `point`.
    ///
    /// Outside, the gradient points from the closest boundary feature (edge or
    /// vertex) toward the query point; inside, it points along the outward
    /// normal of the nearest face.
    pub fn barrier(&self, point: [f64; 2]) -> (f64, [f64; 2]) {
        if self.contains(point) {
            // Inside: nearest face is the least-violated half-space.
            let mut best_index = 0;
            let mut best_value = f64::NEG_INFINITY;
            for (index, h) in self.halfspaces.iter().enumerate() {
                let signed = h.a * point[0] + h.b * point[1] - h.c; // <= 0 inside
                if signed > best_value {
                    best_value = signed;
                    best_index = index;
                }
            }
            let h = self.halfspaces[best_index];
            (best_value, [h.a, h.b])
        } else {
            // Outside: closest point over all edges (handles vertices).
            let n = self.vertices.len();
            let mut best_dist = f64::INFINITY;
            let mut best_grad = [0.0, 0.0];
            for i in 0..n {
                let from = self.vertices[i];
                let to = self.vertices[(i + 1) % n];
                let closest = closest_point_on_segment(point, from, to);
                let dx = point[0] - closest[0];
                let dy = point[1] - closest[1];
                let dist = (dx * dx + dy * dy).sqrt();
                if dist < best_dist {
                    best_dist = dist;
                    best_grad = if dist > EPS {
                        [dx / dist, dy / dist]
                    } else {
                        let h = self.halfspaces[i];
                        [h.a, h.b]
                    };
                }
            }
            (best_dist, best_grad)
        }
    }
}

/// Configuration for the CBF safety filter.
#[derive(Debug, Clone, PartialEq)]
pub struct CbfSafetyFilter {
    pub obstacles: Vec<CbfConvexObstacle2D>,
    /// Class-K gain on the barrier (larger = the robot may approach faster).
    pub alpha: f64,
    /// Robot disc radius the obstacles are inflated by.
    pub robot_radius: f64,
    /// Extra clearance the filter enforces beyond `robot_radius`. Reported
    /// clearance is still measured to `robot_radius`, so a positive margin keeps
    /// the filtered path strictly clear rather than grazing the disc boundary.
    pub margin: f64,
}

/// Result of filtering a single nominal control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CbfFilterResult {
    /// The safe control after filtering.
    pub control: [f64; 2],
    /// Smallest effective barrier `h - robot_radius` over all obstacles.
    pub min_barrier: f64,
    /// Whether the filter changed the nominal control.
    pub intervened: bool,
}

impl CbfSafetyFilter {
    pub fn new(
        obstacles: Vec<CbfConvexObstacle2D>,
        alpha: f64,
        robot_radius: f64,
    ) -> RoboticsResult<Self> {
        if !alpha.is_finite() || alpha <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "CBF alpha must be finite and positive".to_string(),
            ));
        }
        if !robot_radius.is_finite() || robot_radius < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "CBF robot_radius must be finite and non-negative".to_string(),
            ));
        }
        Ok(Self {
            obstacles,
            alpha,
            robot_radius,
            margin: 0.0,
        })
    }

    /// Builder-style setter for the extra clearance margin.
    pub fn with_margin(mut self, margin: f64) -> RoboticsResult<Self> {
        if !margin.is_finite() || margin < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "CBF margin must be finite and non-negative".to_string(),
            ));
        }
        self.margin = margin;
        Ok(self)
    }

    /// Smallest effective barrier `h - robot_radius` over all obstacles.
    pub fn min_barrier(&self, position: [f64; 2]) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.barrier(position).0 - self.robot_radius)
            .fold(f64::INFINITY, f64::min)
    }

    /// Minimally correct `nominal` so the single-integrator robot stays safe.
    pub fn filter(&self, position: [f64; 2], nominal: [f64; 2]) -> CbfFilterResult {
        // Assemble the active CBF constraints g . u >= b.
        let mut constraints: Vec<([f64; 2], f64)> = Vec::new();
        let mut min_barrier = f64::INFINITY;
        for obstacle in &self.obstacles {
            let (h, grad) = obstacle.barrier(position);
            // Report true clearance to the disc boundary...
            min_barrier = min_barrier.min(h - self.robot_radius);
            // ...but enforce the constraint with the extra margin.
            let h_eff = h - self.robot_radius - self.margin;
            let b = -self.alpha * h_eff;
            if dot(grad, nominal) < b - EPS {
                constraints.push((grad, b));
            } else {
                // Still record near-active ones so the QP respects them.
                if dot(grad, nominal) < b + self.alpha.max(1.0) {
                    constraints.push((grad, b));
                }
            }
        }

        let control = solve_min_correction(nominal, &constraints);
        let intervened =
            (control[0] - nominal[0]).abs() > 1e-9 || (control[1] - nominal[1]).abs() > 1e-9;
        CbfFilterResult {
            control,
            min_barrier,
            intervened,
        }
    }
}

/// Solve `min ||u - u_nom||^2  s.t.  g_i . u >= b_i` for 2-D `u` by enumerating
/// active sets of size 0, 1, and 2 (the optimum binds at most two constraints in
/// the plane). Deterministic and correction-minimal.
fn solve_min_correction(u_nom: [f64; 2], constraints: &[([f64; 2], f64)]) -> [f64; 2] {
    let feasible = |u: [f64; 2]| constraints.iter().all(|(g, b)| dot(*g, u) >= *b - 1e-7);

    let mut best: Option<([f64; 2], f64)> = None;
    let consider = |u: [f64; 2]| -> Option<([f64; 2], f64)> {
        if feasible(u) {
            let cost = dist_sq(u, u_nom);
            Some((u, cost))
        } else {
            None
        }
    };

    // Active set of size 0.
    if let Some(candidate) = consider(u_nom) {
        update_best(&mut best, candidate);
    }

    // Active set of size 1.
    for (g, b) in constraints {
        // Project onto g . u = b (unit g): u = u_nom + (b - g.u_nom) g.
        let t = b - dot(*g, u_nom);
        if t <= 0.0 {
            continue; // already satisfied; not the binding face.
        }
        let u = [u_nom[0] + t * g[0], u_nom[1] + t * g[1]];
        if let Some(candidate) = consider(u) {
            update_best(&mut best, candidate);
        }
    }

    // Active set of size 2.
    for i in 0..constraints.len() {
        for j in (i + 1)..constraints.len() {
            let (gi, bi) = constraints[i];
            let (gj, bj) = constraints[j];
            let gij = dot(gi, gj);
            let det = 1.0 - gij * gij;
            if det.abs() <= 1e-9 {
                continue; // parallel constraints.
            }
            let ri = bi - dot(gi, u_nom);
            let rj = bj - dot(gj, u_nom);
            // Solve [[1, gij],[gij, 1]] lambda = [ri, rj].
            let li = (ri - gij * rj) / det;
            let lj = (rj - gij * ri) / det;
            if li < 0.0 || lj < 0.0 {
                continue; // KKT multipliers must be non-negative.
            }
            let u = [
                u_nom[0] + li * gi[0] + lj * gj[0],
                u_nom[1] + li * gi[1] + lj * gj[1],
            ];
            if let Some(candidate) = consider(u) {
                update_best(&mut best, candidate);
            }
        }
    }

    match best {
        Some((u, _)) => u,
        None => {
            // Infeasible (e.g. wedged): satisfy the most-violated constraint.
            let mut worst: Option<([f64; 2], f64)> = None;
            for (g, b) in constraints {
                let violation = b - dot(*g, u_nom);
                if worst.is_none() || violation > worst.unwrap().1 {
                    let u = [u_nom[0] + violation * g[0], u_nom[1] + violation * g[1]];
                    worst = Some((u, violation));
                }
            }
            worst.map(|(u, _)| u).unwrap_or(u_nom)
        }
    }
}

fn update_best(best: &mut Option<([f64; 2], f64)>, candidate: ([f64; 2], f64)) {
    match best {
        Some((_, cost)) if *cost <= candidate.1 => {}
        _ => *best = Some(candidate),
    }
}

/// Configuration for a CBF navigation rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct CbfNavConfig {
    pub dt: f64,
    pub max_steps: usize,
    /// Proportional gain of the go-to-goal nominal controller.
    pub goal_gain: f64,
    /// Maximum nominal speed.
    pub max_speed: f64,
    /// Distance to the goal at which the rollout stops.
    pub goal_tolerance: f64,
}

impl Default for CbfNavConfig {
    fn default() -> Self {
        Self {
            dt: 0.05,
            max_steps: 600,
            goal_gain: 1.5,
            max_speed: 1.4,
            goal_tolerance: 0.15,
        }
    }
}

/// Metrics from a CBF navigation rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct CbfNavReport {
    pub steps: usize,
    pub reached_goal: bool,
    pub path_length: f64,
    /// Smallest clearance (`distance - robot_radius`) to any obstacle.
    pub min_clearance: f64,
    /// Whether the robot ever entered an obstacle (clearance < 0).
    pub collided: bool,
    /// Steps where the filter changed the control.
    pub interventions: usize,
    /// Mean correction magnitude applied by the filter.
    pub mean_correction: f64,
    pub path: Vec<[f64; 2]>,
}

/// Drive a proportional go-to-goal controller through the obstacle field,
/// optionally applying the CBF safety filter, and report clearance/collision.
pub fn simulate_cbf_navigation(
    filter: &CbfSafetyFilter,
    start: [f64; 2],
    goal: [f64; 2],
    config: &CbfNavConfig,
    apply_filter: bool,
) -> RoboticsResult<CbfNavReport> {
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "CBF nav dt must be finite and positive".to_string(),
        ));
    }
    if !config.max_speed.is_finite() || config.max_speed <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "CBF nav max_speed must be finite and positive".to_string(),
        ));
    }

    let mut position = start;
    let mut path = vec![position];
    let mut path_length = 0.0;
    let mut min_clearance = filter.min_barrier(position);
    let mut interventions = 0usize;
    let mut correction_sum = 0.0;
    let mut steps = 0usize;
    let mut reached_goal = false;

    for _ in 0..config.max_steps {
        let to_goal = [goal[0] - position[0], goal[1] - position[1]];
        let goal_distance = (to_goal[0] * to_goal[0] + to_goal[1] * to_goal[1]).sqrt();
        if goal_distance <= config.goal_tolerance {
            reached_goal = true;
            break;
        }

        // Proportional nominal control, capped to the max speed.
        let mut nominal = [config.goal_gain * to_goal[0], config.goal_gain * to_goal[1]];
        let speed = (nominal[0] * nominal[0] + nominal[1] * nominal[1]).sqrt();
        if speed > config.max_speed && speed > 0.0 {
            let scale = config.max_speed / speed;
            nominal = [nominal[0] * scale, nominal[1] * scale];
        }

        let control = if apply_filter {
            let result = filter.filter(position, nominal);
            if result.intervened {
                interventions += 1;
                correction_sum += dist(result.control, nominal);
            }
            result.control
        } else {
            nominal
        };

        let next = [
            position[0] + control[0] * config.dt,
            position[1] + control[1] * config.dt,
        ];
        path_length += dist(next, position);
        position = next;
        path.push(position);
        min_clearance = min_clearance.min(filter.min_barrier(position));
        steps += 1;
    }

    let mean_correction = if interventions > 0 {
        correction_sum / interventions as f64
    } else {
        0.0
    };

    Ok(CbfNavReport {
        steps,
        reached_goal,
        path_length,
        min_clearance,
        collided: min_clearance < 0.0,
        interventions,
        mean_correction,
        path,
    })
}

// --- 2-D helpers ---------------------------------------------------------

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn dist_sq(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    dist_sq(a, b).sqrt()
}

fn signed_area(vertices: &[[f64; 2]]) -> f64 {
    let n = vertices.len();
    let mut sum = 0.0;
    for i in 0..n {
        let a = vertices[i];
        let b = vertices[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    0.5 * sum
}

fn closest_point_on_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let denom = dot(ab, ab);
    let t = if denom > EPS {
        (dot(ap, ab) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    [a[0] + t * ab[0], a[1] + t * ab[1]]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(cx: f64, cy: f64, half: f64) -> CbfConvexObstacle2D {
        CbfConvexObstacle2D::aabb(cx - half, cx + half, cy - half, cy + half).unwrap()
    }

    #[test]
    fn barrier_sign_and_gradient_point_outward() {
        let obstacle = square(0.0, 0.0, 1.0);
        // Outside to the right: distance 1, gradient +x.
        let (h, grad) = obstacle.barrier([2.0, 0.0]);
        assert!((h - 1.0).abs() < 1e-9, "h {h}");
        assert!((grad[0] - 1.0).abs() < 1e-9 && grad[1].abs() < 1e-9);
        // Inside: negative barrier.
        let (h_in, _) = obstacle.barrier([0.0, 0.0]);
        assert!(h_in < 0.0);
    }

    #[test]
    fn barrier_handles_vertex_region() {
        let obstacle = square(0.0, 0.0, 1.0);
        // Diagonally off the (1,1) corner: true distance is to the vertex.
        let (h, grad) = obstacle.barrier([2.0, 2.0]);
        let expected = ((1.0_f64).hypot(1.0)) - 0.0; // dist from (2,2) to (1,1)
        assert!((h - expected).abs() < 1e-9, "h {h} expected {expected}");
        // Gradient points along the diagonal away from the corner.
        assert!((grad[0] - grad[1]).abs() < 1e-9 && grad[0] > 0.0);
    }

    #[test]
    fn filter_is_identity_when_safe() {
        let filter = CbfSafetyFilter::new(vec![square(5.0, 5.0, 1.0)], 2.0, 0.2).unwrap();
        let result = filter.filter([0.0, 0.0], [1.0, 0.0]);
        assert!(!result.intervened);
        assert_eq!(result.control, [1.0, 0.0]);
    }

    #[test]
    fn filter_deflects_toward_obstacle() {
        // Obstacle straight ahead; nominal drives into it.
        let filter = CbfSafetyFilter::new(vec![square(2.0, 0.0, 1.0)], 1.0, 0.2).unwrap();
        // Right at the boundary, pushing in: the inward velocity component must
        // be removed.
        let position = [0.7, 0.0]; // distance 0.3 to face at x=1, minus radius 0.2
        let result = filter.filter(position, [1.0, 0.0]);
        assert!(result.intervened);
        // After filtering, moving along +x must respect grad . u >= -alpha*h_eff.
        let (h, grad) = filter.obstacles[0].barrier(position);
        let h_eff = h - filter.robot_radius;
        assert!(dot(grad, result.control) >= -filter.alpha * h_eff - 1e-6);
    }

    #[test]
    fn nominal_run_collides_filtered_run_does_not() {
        let filter = CbfSafetyFilter::new(vec![square(2.5, 0.0, 0.8)], 2.0, 0.2).unwrap();
        let config = CbfNavConfig::default();
        let nominal =
            simulate_cbf_navigation(&filter, [0.0, 0.0], [5.0, 0.0], &config, false).unwrap();
        let filtered =
            simulate_cbf_navigation(&filter, [0.0, 0.0], [5.0, 0.0], &config, true).unwrap();
        assert!(
            nominal.collided,
            "nominal min_clearance {}",
            nominal.min_clearance
        );
        assert!(
            !filtered.collided,
            "filtered min_clearance {}",
            filtered.min_clearance
        );
        assert!(filtered.interventions > 0);
    }

    #[test]
    fn filtered_run_is_deterministic() {
        let filter = CbfSafetyFilter::new(vec![square(2.5, 0.2, 0.8)], 2.0, 0.2).unwrap();
        let config = CbfNavConfig::default();
        let run =
            || simulate_cbf_navigation(&filter, [0.0, 0.0], [5.0, 0.0], &config, true).unwrap();
        assert_eq!(run().path, run().path);
    }

    #[test]
    fn rejects_invalid_alpha() {
        assert!(CbfSafetyFilter::new(vec![square(0.0, 0.0, 1.0)], 0.0, 0.2).is_err());
    }
}
