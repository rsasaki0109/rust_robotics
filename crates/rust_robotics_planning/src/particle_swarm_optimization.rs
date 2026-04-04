//! Particle Swarm Optimization (PSO) path planner
//!
//! Treats path planning as an optimization problem where particles explore the
//! search space to minimize distance to target while avoiding circular obstacles.
//! Each particle maintains a position, velocity, and personal best; the swarm
//! collectively converges towards the global best solution.
//!
//! Reference: Kennedy & Eberhart (1995), "Particle Swarm Optimization".

use rand::Rng;
use rust_robotics_core::{Point2D, RoboticsError, RoboticsResult};

/// A circular obstacle defined by centre and radius.
#[derive(Debug, Clone)]
pub struct CircleObstacle {
    pub center: Point2D,
    pub radius: f64,
}

impl CircleObstacle {
    pub fn new(cx: f64, cy: f64, radius: f64) -> Self {
        Self {
            center: Point2D::new(cx, cy),
            radius,
        }
    }
}

/// Axis-aligned rectangular bounds `[min, max]` for each of two dimensions.
#[derive(Debug, Clone)]
pub struct Bounds2D {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl Bounds2D {
    pub fn new(x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> Self {
        Self {
            x_min,
            x_max,
            y_min,
            y_max,
        }
    }
}

/// Configuration for the PSO planner.
#[derive(Debug, Clone)]
pub struct PsoConfig {
    /// Number of particles in the swarm.
    pub n_particles: usize,
    /// Maximum number of iterations.
    pub max_iter: usize,
    /// Target position.
    pub target: Point2D,
    /// Bounds defining the full search space.
    pub search_bounds: Bounds2D,
    /// Bounds defining the spawn area for particles.
    pub spawn_bounds: Bounds2D,
    /// Circular obstacles.
    pub obstacles: Vec<CircleObstacle>,
    /// Initial inertia weight (decays linearly to `w_end`).
    pub w_start: f64,
    /// Final inertia weight.
    pub w_end: f64,
    /// Cognitive coefficient (pull towards personal best).
    pub c1: f64,
    /// Social coefficient (pull towards global best).
    pub c2: f64,
    /// Penalty applied when a particle is inside an obstacle.
    pub inside_obstacle_penalty: f64,
    /// Proximity penalty factor for being near an obstacle.
    pub proximity_penalty_factor: f64,
    /// Proximity distance threshold beyond the obstacle radius.
    pub proximity_margin: f64,
}

impl Default for PsoConfig {
    fn default() -> Self {
        Self {
            n_particles: 15,
            max_iter: 150,
            target: Point2D::new(40.0, 35.0),
            search_bounds: Bounds2D::new(-50.0, 50.0, -50.0, 50.0),
            spawn_bounds: Bounds2D::new(-45.0, -35.0, -45.0, -35.0),
            obstacles: vec![],
            w_start: 0.9,
            w_end: 0.4,
            c1: 1.5,
            c2: 1.5,
            inside_obstacle_penalty: 1000.0,
            proximity_penalty_factor: 50.0,
            proximity_margin: 5.0,
        }
    }
}

impl PsoConfig {
    /// Validate configuration parameters.
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.n_particles == 0 {
            return Err(RoboticsError::InvalidParameter(
                "n_particles must be > 0".into(),
            ));
        }
        if self.max_iter == 0 {
            return Err(RoboticsError::InvalidParameter(
                "max_iter must be > 0".into(),
            ));
        }
        if self.w_start < self.w_end {
            return Err(RoboticsError::InvalidParameter(
                "w_start must be >= w_end".into(),
            ));
        }
        Ok(())
    }
}

/// A single particle in the swarm.
#[derive(Debug, Clone)]
struct Particle {
    position: [f64; 2],
    velocity: [f64; 2],
    personal_best_position: [f64; 2],
    personal_best_value: f64,
    max_velocity: [f64; 2],
    path: Vec<[f64; 2]>,
}

impl Particle {
    fn new<R: Rng>(search_bounds: &Bounds2D, spawn_bounds: &Bounds2D, rng: &mut R) -> Self {
        let position = [
            rng.gen_range(spawn_bounds.x_min..=spawn_bounds.x_max),
            rng.gen_range(spawn_bounds.y_min..=spawn_bounds.y_max),
        ];
        let velocity = [rng.gen_range(-0.1..0.1), rng.gen_range(-0.1..0.1)];
        let max_velocity = [
            (search_bounds.x_max - search_bounds.x_min) * 0.05,
            (search_bounds.y_max - search_bounds.y_min) * 0.05,
        ];
        Self {
            position,
            velocity,
            personal_best_position: position,
            personal_best_value: f64::INFINITY,
            max_velocity,
            path: vec![position],
        }
    }

    fn update_velocity<R: Rng>(
        &mut self,
        gbest_pos: &[f64; 2],
        w: f64,
        c1: f64,
        c2: f64,
        rng: &mut R,
    ) {
        for (i, &gbest) in gbest_pos.iter().enumerate() {
            let r1: f64 = rng.gen();
            let r2: f64 = rng.gen();
            let cognitive = c1 * r1 * (self.personal_best_position[i] - self.position[i]);
            let social = c2 * r2 * (gbest - self.position[i]);
            self.velocity[i] = w * self.velocity[i] + cognitive + social;
            self.velocity[i] = self.velocity[i].clamp(-self.max_velocity[i], self.max_velocity[i]);
        }
    }

    fn update_position(&mut self, search_bounds: &Bounds2D) {
        self.position[0] =
            (self.position[0] + self.velocity[0]).clamp(search_bounds.x_min, search_bounds.x_max);
        self.position[1] =
            (self.position[1] + self.velocity[1]).clamp(search_bounds.y_min, search_bounds.y_max);
        self.path.push(self.position);
    }
}

/// Result returned by the PSO planner.
#[derive(Debug, Clone)]
pub struct PsoResult {
    /// Best position found by the swarm.
    pub best_position: Point2D,
    /// Fitness value at the best position.
    pub best_fitness: f64,
    /// Sequence of global-best positions recorded each iteration.
    pub best_path: Vec<Point2D>,
    /// Number of iterations actually executed.
    pub iterations: usize,
}

/// The PSO path planner.
#[derive(Debug, Clone)]
pub struct PsoPlanner {
    config: PsoConfig,
}

impl PsoPlanner {
    pub fn new(config: PsoConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(Self { config })
    }

    /// Run the PSO optimisation and return the result.
    pub fn plan(&self) -> PsoResult {
        self.plan_with_rng(&mut rand::thread_rng())
    }

    /// Run with an explicit RNG (useful for deterministic tests).
    pub fn plan_with_rng<R: Rng>(&self, rng: &mut R) -> PsoResult {
        let cfg = &self.config;
        let mut particles: Vec<Particle> = (0..cfg.n_particles)
            .map(|_| Particle::new(&cfg.search_bounds, &cfg.spawn_bounds, rng))
            .collect();

        let mut gbest_position = [0.0_f64; 2];
        let mut gbest_value = f64::INFINITY;
        let mut gbest_path: Vec<Point2D> = Vec::new();
        let mut iterations = 0;

        for iter in 0..cfg.max_iter {
            let w = cfg.w_start - (cfg.w_start - cfg.w_end) * (iter as f64 / cfg.max_iter as f64);

            // Evaluate fitness and update personal/global best
            for p in &mut particles {
                let value = self.fitness(&p.position);
                if value < p.personal_best_value {
                    p.personal_best_value = value;
                    p.personal_best_position = p.position;
                }
                if value < gbest_value {
                    gbest_value = value;
                    gbest_position = p.position;
                }
            }
            gbest_path.push(Point2D::new(gbest_position[0], gbest_position[1]));

            // Update velocities and positions
            for p in &mut particles {
                p.update_velocity(&gbest_position, w, cfg.c1, cfg.c2, rng);

                // Collision check: reduce velocity if path crosses an obstacle
                let next_pos = [p.position[0] + p.velocity[0], p.position[1] + p.velocity[1]];
                let collision = cfg
                    .obstacles
                    .iter()
                    .any(|obs| check_line_circle_collision(&p.position, &next_pos, obs));
                if collision {
                    p.velocity[0] *= 0.2;
                    p.velocity[1] *= 0.2;
                }

                p.update_position(&cfg.search_bounds);
            }
            iterations = iter + 1;
        }

        PsoResult {
            best_position: Point2D::new(gbest_position[0], gbest_position[1]),
            best_fitness: gbest_value,
            best_path: gbest_path,
            iterations,
        }
    }

    /// Fitness function: distance to target + obstacle penalty.
    fn fitness(&self, pos: &[f64; 2]) -> f64 {
        let cfg = &self.config;
        let dx = pos[0] - cfg.target.x;
        let dy = pos[1] - cfg.target.y;
        let dist = (dx * dx + dy * dy).sqrt();

        let mut penalty = 0.0;
        for obs in &cfg.obstacles {
            let odx = pos[0] - obs.center.x;
            let ody = pos[1] - obs.center.y;
            let obs_dist = (odx * odx + ody * ody).sqrt();
            if obs_dist < obs.radius {
                penalty += cfg.inside_obstacle_penalty;
            } else if obs_dist < obs.radius + cfg.proximity_margin {
                penalty += cfg.proximity_penalty_factor / (obs_dist - obs.radius + 0.1);
            }
        }
        dist + penalty
    }
}

/// Check whether a line segment from `start` to `end` intersects a circle obstacle.
fn check_line_circle_collision(
    start: &[f64; 2],
    end: &[f64; 2],
    obstacle: &CircleObstacle,
) -> bool {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let fx = start[0] - obstacle.center.x;
    let fy = start[1] - obstacle.center.y;

    let a = dx * dx + dy * dy;
    if a < 1e-10 {
        // Near-zero length segment: check if start is inside obstacle
        return (fx * fx + fy * fy).sqrt() <= obstacle.radius;
    }

    let b = 2.0 * (fx * dx + fy * dy);
    let c = fx * fx + fy * fy - obstacle.radius * obstacle.radius;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return false;
    }

    let sqrt_disc = discriminant.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);
    (0.0..=1.0).contains(&t1) || (0.0..=1.0).contains(&t2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_obstacles() -> Vec<CircleObstacle> {
        vec![
            CircleObstacle::new(10.0, 15.0, 8.0),
            CircleObstacle::new(-20.0, 0.0, 12.0),
            CircleObstacle::new(20.0, -25.0, 10.0),
            CircleObstacle::new(-5.0, -30.0, 7.0),
        ]
    }

    #[test]
    fn test_pso_converges_toward_target() {
        let config = PsoConfig {
            n_particles: 20,
            max_iter: 200,
            target: Point2D::new(40.0, 35.0),
            search_bounds: Bounds2D::new(-50.0, 50.0, -50.0, 50.0),
            spawn_bounds: Bounds2D::new(-45.0, -35.0, -45.0, -35.0),
            obstacles: default_obstacles(),
            ..PsoConfig::default()
        };
        let planner = PsoPlanner::new(config).unwrap();
        let result = planner.plan();

        // The best position should be reasonably close to the target
        let dist = result.best_position.distance(&Point2D::new(40.0, 35.0));
        assert!(
            dist < 15.0,
            "Expected best position near target, got distance {dist:.2}"
        );
        assert_eq!(result.iterations, 200);
        assert!(!result.best_path.is_empty());
    }

    #[test]
    fn test_pso_no_obstacles() {
        let config = PsoConfig {
            n_particles: 10,
            max_iter: 100,
            target: Point2D::new(0.0, 0.0),
            search_bounds: Bounds2D::new(-10.0, 10.0, -10.0, 10.0),
            spawn_bounds: Bounds2D::new(-5.0, -3.0, -5.0, -3.0),
            obstacles: vec![],
            ..PsoConfig::default()
        };
        let planner = PsoPlanner::new(config).unwrap();
        let result = planner.plan();
        let dist = result.best_position.distance(&Point2D::new(0.0, 0.0));
        assert!(
            dist < 3.0,
            "Without obstacles the swarm should converge near the target, dist={dist:.2}"
        );
    }

    #[test]
    fn test_pso_deterministic_with_seed() {
        let config = PsoConfig {
            n_particles: 10,
            max_iter: 50,
            target: Point2D::new(5.0, 5.0),
            search_bounds: Bounds2D::new(-10.0, 10.0, -10.0, 10.0),
            spawn_bounds: Bounds2D::new(-8.0, -6.0, -8.0, -6.0),
            obstacles: vec![CircleObstacle::new(0.0, 0.0, 3.0)],
            ..PsoConfig::default()
        };
        let planner = PsoPlanner::new(config).unwrap();

        use rand::SeedableRng;
        let r1 = planner.plan_with_rng(&mut rand::rngs::StdRng::seed_from_u64(42));
        let r2 = planner.plan_with_rng(&mut rand::rngs::StdRng::seed_from_u64(42));
        assert_eq!(r1.best_fitness, r2.best_fitness);
        assert_eq!(r1.best_position.x, r2.best_position.x);
        assert_eq!(r1.best_position.y, r2.best_position.y);
    }

    #[test]
    fn test_line_circle_collision() {
        let obs = CircleObstacle::new(5.0, 0.0, 2.0);
        // Line passes through obstacle
        assert!(check_line_circle_collision(&[0.0, 0.0], &[10.0, 0.0], &obs));
        // Line misses obstacle
        assert!(!check_line_circle_collision(
            &[0.0, 5.0],
            &[10.0, 5.0],
            &obs
        ));
        // Zero-length segment inside obstacle
        assert!(check_line_circle_collision(&[5.0, 0.0], &[5.0, 0.0], &obs));
        // Zero-length segment outside obstacle
        assert!(!check_line_circle_collision(&[0.0, 0.0], &[0.0, 0.0], &obs));
    }

    #[test]
    fn test_fitness_inside_obstacle_penalty() {
        let config = PsoConfig {
            obstacles: vec![CircleObstacle::new(0.0, 0.0, 5.0)],
            target: Point2D::new(10.0, 0.0),
            ..PsoConfig::default()
        };
        let planner = PsoPlanner::new(config).unwrap();

        let fitness_inside = planner.fitness(&[0.0, 0.0]);
        let fitness_outside = planner.fitness(&[10.0, 0.0]);
        assert!(
            fitness_inside > fitness_outside,
            "Inside-obstacle fitness ({fitness_inside}) should be much larger than at target ({fitness_outside})"
        );
    }

    #[test]
    fn test_invalid_config() {
        let config = PsoConfig {
            n_particles: 0,
            ..PsoConfig::default()
        };
        assert!(PsoPlanner::new(config).is_err());

        let config = PsoConfig {
            max_iter: 0,
            ..PsoConfig::default()
        };
        assert!(PsoPlanner::new(config).is_err());
    }
}
