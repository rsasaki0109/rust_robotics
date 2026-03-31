//! Elastic Bands path deformation algorithm
//!
//! Deforms an initial path using internal contraction forces and external
//! repulsive forces computed from a distance field. Each waypoint is
//! surrounded by a "bubble" whose radius equals the clearance to the nearest
//! obstacle. The bubble chain is maintained by inserting / deleting bubbles
//! to satisfy an overlap constraint.
//!
//! Reference:
//! - Elastic Bands: Connecting Path Planning and Control
//!   <http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf>

/// A single bubble in the elastic band.
#[derive(Debug, Clone)]
pub struct Bubble {
    /// Centre position \[x, y\].
    pub pos: [f64; 2],
    /// Safety radius (distance to nearest obstacle, clamped).
    pub radius: f64,
}

/// Configuration parameters for the [`ElasticBands`] planner.
#[derive(Debug, Clone)]
pub struct ElasticBandsConfig {
    /// Maximum allowed bubble radius.
    pub max_bubble_radius: f64,
    /// Minimum allowed bubble radius.
    pub min_bubble_radius: f64,
    /// Distance threshold for repulsive force (rho0).
    pub rho0: f64,
    /// Contraction (internal) force gain.
    pub kc: f64,
    /// Repulsive (external) force gain (typically negative).
    pub kr: f64,
    /// Overlap constraint factor (lambda).
    pub lambda: f64,
    /// Finite-difference step size for gradient estimation.
    pub step_size: f64,
    /// Maximum number of optimisation iterations.
    pub max_iter: usize,
}

impl Default for ElasticBandsConfig {
    fn default() -> Self {
        Self {
            max_bubble_radius: 100.0,
            min_bubble_radius: 10.0,
            rho0: 20.0,
            kc: 0.05,
            kr: -0.1,
            lambda: 0.7,
            step_size: 3.0,
            max_iter: 50,
        }
    }
}

/// A signed-distance field represented as a 2-D grid.
///
/// `sdf\[ix\]\[iy\]` stores the distance to the nearest obstacle for the cell
/// at integer coordinates `(ix, iy)`.  Values are positive outside obstacles.
pub struct DistanceField {
    data: Vec<Vec<f64>>,
    width: usize,
    height: usize,
}

impl DistanceField {
    /// Build a distance field from a binary obstacle grid.
    ///
    /// `obstacles` is a row-major `width x height` grid where `true` means
    /// occupied.  The returned field stores the Euclidean distance to the
    /// nearest occupied cell (approximated via a two-pass sweeping algorithm).
    pub fn from_obstacle_grid(obstacles: &[Vec<bool>]) -> Self {
        let width = obstacles.len();
        assert!(width > 0, "obstacle grid must be non-empty");
        let height = obstacles[0].len();
        assert!(height > 0, "obstacle grid rows must be non-empty");

        let large = (width + height) as f64;
        let mut dist = vec![vec![large; height]; width];

        // Initialise: 0 at obstacles, large elsewhere.
        for ix in 0..width {
            for iy in 0..height {
                if obstacles[ix][iy] {
                    dist[ix][iy] = 0.0;
                }
            }
        }

        // Forward pass (top-left to bottom-right).
        for ix in 0..width {
            for iy in 0..height {
                if ix > 0 {
                    dist[ix][iy] = dist[ix][iy].min(dist[ix - 1][iy] + 1.0);
                }
                if iy > 0 {
                    dist[ix][iy] = dist[ix][iy].min(dist[ix][iy - 1] + 1.0);
                }
                if ix > 0 && iy > 0 {
                    dist[ix][iy] =
                        dist[ix][iy].min(dist[ix - 1][iy - 1] + std::f64::consts::SQRT_2);
                }
                if ix > 0 && iy + 1 < height {
                    dist[ix][iy] =
                        dist[ix][iy].min(dist[ix - 1][iy + 1] + std::f64::consts::SQRT_2);
                }
            }
        }

        // Backward pass (bottom-right to top-left).
        for ix in (0..width).rev() {
            for iy in (0..height).rev() {
                if ix + 1 < width {
                    dist[ix][iy] = dist[ix][iy].min(dist[ix + 1][iy] + 1.0);
                }
                if iy + 1 < height {
                    dist[ix][iy] = dist[ix][iy].min(dist[ix][iy + 1] + 1.0);
                }
                if ix + 1 < width && iy + 1 < height {
                    dist[ix][iy] =
                        dist[ix][iy].min(dist[ix + 1][iy + 1] + std::f64::consts::SQRT_2);
                }
                if ix + 1 < width && iy > 0 {
                    dist[ix][iy] =
                        dist[ix][iy].min(dist[ix + 1][iy - 1] + std::f64::consts::SQRT_2);
                }
            }
        }

        Self {
            data: dist,
            width,
            height,
        }
    }

    /// Query the distance at a continuous position (nearest-cell lookup).
    pub fn query(&self, x: f64, y: f64) -> f64 {
        let ix = (x.round() as isize).clamp(0, self.width as isize - 1) as usize;
        let iy = (y.round() as isize).clamp(0, self.height as isize - 1) as usize;
        self.data[ix][iy]
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }
}

/// Elastic Bands planner.
///
/// Iteratively deforms a path (represented as a chain of [`Bubble`]s) to be
/// shorter and further from obstacles.
pub struct ElasticBands {
    /// Current chain of bubbles.
    pub bubbles: Vec<Bubble>,
    config: ElasticBandsConfig,
    distance_field: DistanceField,
}

impl ElasticBands {
    /// Create a new elastic band from an initial path and a pre-computed
    /// distance field.
    pub fn new(
        initial_path: &[[f64; 2]],
        distance_field: DistanceField,
        config: ElasticBandsConfig,
    ) -> Self {
        let bubbles: Vec<Bubble> = initial_path
            .iter()
            .map(|p| {
                let r = distance_field.query(p[0], p[1]);
                Bubble {
                    pos: *p,
                    radius: r.clamp(config.min_bubble_radius, config.max_bubble_radius),
                }
            })
            .collect();

        let mut band = Self {
            bubbles,
            config,
            distance_field,
        };
        band.maintain_overlap();
        band
    }

    /// Create from an initial path and a binary obstacle grid.
    pub fn from_obstacles(
        initial_path: &[[f64; 2]],
        obstacles: &[Vec<bool>],
        config: ElasticBandsConfig,
    ) -> Self {
        let df = DistanceField::from_obstacle_grid(obstacles);
        Self::new(initial_path, df, config)
    }

    /// Run the full optimisation for `config.max_iter` iterations.
    pub fn optimise(&mut self) {
        for _ in 0..self.config.max_iter {
            self.update_bubbles();
        }
    }

    /// Perform a single optimisation step: move bubbles then maintain overlap.
    pub fn update_bubbles(&mut self) {
        let n = self.bubbles.len();
        let mut new_bubbles = Vec::with_capacity(n);

        for i in 0..n {
            if i == 0 || i == n - 1 {
                new_bubbles.push(self.bubbles[i].clone());
                continue;
            }

            let fc = self.contraction_force(i);
            let fr = self.repulsive_force(i);
            let f_total = [fc[0] + fr[0], fc[1] + fr[1]];

            // Direction between neighbours (for tangent removal).
            let v = [
                self.bubbles[i - 1].pos[0] - self.bubbles[i + 1].pos[0],
                self.bubbles[i - 1].pos[1] - self.bubbles[i + 1].pos[1],
            ];
            let v_norm_sq = v[0] * v[0] + v[1] * v[1] + 1e-6;

            // Remove tangential component.
            let f_dot_v = f_total[0] * v[0] + f_total[1] * v[1];
            let f_star = [
                f_total[0] - f_dot_v * v[0] / v_norm_sq,
                f_total[1] - f_dot_v * v[1] / v_norm_sq,
            ];

            let alpha = self.bubbles[i].radius;
            let max_x = (self.distance_field.width() as f64 - 1.0).max(0.0);
            let max_y = (self.distance_field.height() as f64 - 1.0).max(0.0);
            let new_x = (self.bubbles[i].pos[0] + alpha * f_star[0]).clamp(0.0, max_x);
            let new_y = (self.bubbles[i].pos[1] + alpha * f_star[1]).clamp(0.0, max_y);

            let r = self.distance_field.query(new_x, new_y);
            new_bubbles.push(Bubble {
                pos: [new_x, new_y],
                radius: r.clamp(self.config.min_bubble_radius, self.config.max_bubble_radius),
            });
        }

        self.bubbles = new_bubbles;
        self.maintain_overlap();
    }

    /// Extract the optimised path as a vector of `[x, y]` points.
    pub fn path(&self) -> Vec<[f64; 2]> {
        self.bubbles.iter().map(|b| b.pos).collect()
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    fn contraction_force(&self, i: usize) -> [f64; 2] {
        if i == 0 || i == self.bubbles.len() - 1 {
            return [0.0, 0.0];
        }
        let prev = &self.bubbles[i - 1].pos;
        let next = &self.bubbles[i + 1].pos;
        let cur = &self.bubbles[i].pos;

        let dp = [prev[0] - cur[0], prev[1] - cur[1]];
        let dn = [next[0] - cur[0], next[1] - cur[1]];
        let dp_len = (dp[0] * dp[0] + dp[1] * dp[1]).sqrt() + 1e-6;
        let dn_len = (dn[0] * dn[0] + dn[1] * dn[1]).sqrt() + 1e-6;

        [
            self.config.kc * (dp[0] / dp_len + dn[0] / dn_len),
            self.config.kc * (dp[1] / dp_len + dn[1] / dn_len),
        ]
    }

    fn repulsive_force(&self, i: usize) -> [f64; 2] {
        let b = &self.bubbles[i];
        if b.radius >= self.config.rho0 {
            return [0.0, 0.0];
        }

        let h = self.config.step_size;
        let grad_x = (self.distance_field.query(b.pos[0] - h, b.pos[1])
            - self.distance_field.query(b.pos[0] + h, b.pos[1]))
            / (2.0 * h);
        let grad_y = (self.distance_field.query(b.pos[0], b.pos[1] - h)
            - self.distance_field.query(b.pos[0], b.pos[1] + h))
            / (2.0 * h);

        let scale = self.config.kr * (self.config.rho0 - b.radius);
        [scale * grad_x, scale * grad_y]
    }

    fn maintain_overlap(&mut self) {
        // Insert bubbles where neighbours are too far apart.
        let mut i = 0;
        while i < self.bubbles.len().saturating_sub(1) {
            let dist = dist2d(&self.bubbles[i].pos, &self.bubbles[i + 1].pos);
            let threshold =
                self.config.lambda * (self.bubbles[i].radius + self.bubbles[i + 1].radius);
            if dist > threshold {
                let mid = [
                    (self.bubbles[i].pos[0] + self.bubbles[i + 1].pos[0]) * 0.5,
                    (self.bubbles[i].pos[1] + self.bubbles[i + 1].pos[1]) * 0.5,
                ];
                let r = self
                    .distance_field
                    .query(mid[0], mid[1])
                    .clamp(self.config.min_bubble_radius, self.config.max_bubble_radius);
                self.bubbles.insert(
                    i + 1,
                    Bubble {
                        pos: mid,
                        radius: r,
                    },
                );
                i += 2;
            } else {
                i += 1;
            }
        }

        // Delete redundant bubbles.
        let mut i = 1;
        while i < self.bubbles.len().saturating_sub(1) {
            let prev = &self.bubbles[i - 1];
            let next = &self.bubbles[i + 1];
            let dist = dist2d(&prev.pos, &next.pos);
            let threshold = self.config.lambda * (prev.radius + next.radius);
            if dist <= threshold {
                self.bubbles.remove(i);
            } else {
                i += 1;
            }
        }
    }
}

fn dist2d(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a 100x100 grid with a rectangular obstacle block.
    fn make_obstacle_grid() -> Vec<Vec<bool>> {
        let (w, h) = (100, 100);
        let mut grid = vec![vec![false; h]; w];
        // Block from (40,35) to (60,65).
        for ix in 40..60 {
            for iy in 35..65 {
                grid[ix][iy] = true;
            }
        }
        grid
    }

    #[test]
    fn test_distance_field_basic() {
        let grid = make_obstacle_grid();
        let df = DistanceField::from_obstacle_grid(&grid);
        // Inside obstacle should be 0.
        assert_eq!(df.query(50.0, 50.0), 0.0);
        // Far from obstacle should be positive.
        assert!(df.query(5.0, 5.0) > 10.0);
    }

    #[test]
    fn test_bubble_creation() {
        let config = ElasticBandsConfig::default();
        let grid = make_obstacle_grid();
        let df = DistanceField::from_obstacle_grid(&grid);
        let r = df.query(10.0, 10.0);
        let bubble = Bubble {
            pos: [10.0, 10.0],
            radius: r.clamp(config.min_bubble_radius, config.max_bubble_radius),
        };
        assert!(bubble.radius >= config.min_bubble_radius);
        assert!(bubble.radius <= config.max_bubble_radius);
    }

    #[test]
    fn test_elastic_bands_path_shortens() {
        let grid = make_obstacle_grid();
        let config = ElasticBandsConfig {
            max_iter: 30,
            ..Default::default()
        };

        // Deliberately detoured path that goes above the obstacle.
        let initial_path = vec![
            [10.0, 50.0],
            [20.0, 80.0],
            [35.0, 90.0],
            [50.0, 85.0],
            [65.0, 90.0],
            [80.0, 80.0],
            [90.0, 50.0],
        ];

        let path_len =
            |pts: &[[f64; 2]]| -> f64 { pts.windows(2).map(|w| dist2d(&w[0], &w[1])).sum() };

        let initial_length = path_len(&initial_path);

        let mut band = ElasticBands::from_obstacles(&initial_path, &grid, config);
        band.optimise();
        let optimised = band.path();

        let final_length = path_len(&optimised);
        // The contraction force should shorten the path.
        assert!(
            final_length < initial_length,
            "path should shorten: initial {initial_length:.1}, final {final_length:.1}"
        );
    }

    #[test]
    fn test_elastic_bands_endpoints_fixed() {
        let grid = make_obstacle_grid();
        let config = ElasticBandsConfig::default();

        let initial_path = vec![[10.0, 50.0], [50.0, 80.0], [90.0, 50.0]];
        let mut band = ElasticBands::from_obstacles(&initial_path, &grid, config);
        band.optimise();

        let path = band.path();
        assert!(
            (path.first().unwrap()[0] - 10.0).abs() < 1e-9,
            "start x should remain fixed"
        );
        assert!(
            (path.last().unwrap()[0] - 90.0).abs() < 1e-9,
            "end x should remain fixed"
        );
    }

    #[test]
    fn test_maintain_overlap_inserts_bubbles() {
        let grid = vec![vec![false; 100]; 100];
        let config = ElasticBandsConfig::default();

        // Two points very far apart — overlap maintenance should insert bubbles.
        let initial_path = vec![[0.0, 0.0], [99.0, 99.0]];
        let band = ElasticBands::from_obstacles(&initial_path, &grid, config);
        assert!(
            band.bubbles.len() > 2,
            "bubbles should be inserted between distant endpoints"
        );
    }

    #[test]
    fn test_no_obstacle_path_contracts() {
        // With no obstacles the repulsive force is zero and the contraction
        // force should pull the path straighter.
        let grid = vec![vec![false; 100]; 100];
        let config = ElasticBandsConfig {
            max_iter: 20,
            ..Default::default()
        };

        let initial_path = vec![
            [10.0, 10.0],
            [30.0, 50.0],
            [50.0, 10.0],
            [70.0, 50.0],
            [90.0, 10.0],
        ];
        let initial_length: f64 = initial_path.windows(2).map(|w| dist2d(&w[0], &w[1])).sum();

        let mut band = ElasticBands::from_obstacles(&initial_path, &grid, config);
        band.optimise();

        let final_length: f64 = band.path().windows(2).map(|w| dist2d(&w[0], &w[1])).sum();
        assert!(final_length < initial_length);
    }
}
