//! Interactive grid planner demo: A*, Dijkstra, JPS, and Theta*.

use std::time::Instant;

use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};
use rust_robotics_core::{Obstacles, Point2D, RoboticsResult};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::dijkstra::{dijkstra_plan, has_collision};
use rust_robotics_planning::grid_nalgebra;
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const GRID_W: usize = 32;
const GRID_H: usize = 24;
const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.35;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlannerKind {
    AStar,
    Dijkstra,
    Jps,
    ThetaStar,
}

impl PlannerKind {
    const ALL: [Self; 4] = [Self::AStar, Self::Dijkstra, Self::Jps, Self::ThetaStar];

    fn label(self) -> &'static str {
        match self {
            Self::AStar => "A*",
            Self::Dijkstra => "Dijkstra",
            Self::Jps => "JPS",
            Self::ThetaStar => "Theta*",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::AStar => "astar",
            Self::Dijkstra => "dijkstra",
            Self::Jps => "jps",
            Self::ThetaStar => "theta",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        match value {
            "astar" => Some(Self::AStar),
            "dijkstra" => Some(Self::Dijkstra),
            "jps" => Some(Self::Jps),
            "theta" => Some(Self::ThetaStar),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EditMode {
    Obstacles,
    StartGoal,
}

#[derive(Debug, Clone)]
struct PlanSnapshot {
    planner: PlannerKind,
    path_len: f64,
    waypoint_count: usize,
    elapsed_us: f64,
    success: bool,
}

#[derive(Debug, Clone)]
pub struct GridPlannerDemo {
    obstacles: Vec<Vec<bool>>,
    start: (usize, usize),
    goal: (usize, usize),
    planner: PlannerKind,
    edit_mode: EditMode,
    path: Vec<(usize, usize)>,
    last_plan: Option<PlanSnapshot>,
    compare_results: Vec<PlanSnapshot>,
    plans_per_sec: f64,
    dragging_start: bool,
    dragging_goal: bool,
}

impl Default for GridPlannerDemo {
    fn default() -> Self {
        let mut demo = Self {
            obstacles: vec![vec![false; GRID_H]; GRID_W],
            start: (2, GRID_H / 2),
            goal: (GRID_W - 3, GRID_H / 2),
            planner: PlannerKind::AStar,
            edit_mode: EditMode::Obstacles,
            path: Vec::new(),
            last_plan: None,
            compare_results: Vec::new(),
            plans_per_sec: 0.0,
            dragging_start: false,
            dragging_goal: false,
        };
        demo.set_border_obstacles();
        demo.add_default_wall();
        demo.replan();
        demo
    }
}

impl GridPlannerDemo {
    pub(crate) fn apply_share_query(&mut self, query: &str) {
        if let Some(planner) =
            crate::share::value(query, "planner").and_then(PlannerKind::from_slug)
        {
            self.planner = planner;
        }
        if let Some(map) = crate::share::value(query, "map").and_then(Self::decode_map) {
            self.obstacles = map;
        }
        if let Some(start) = crate::share::value(query, "start").and_then(Self::decode_point) {
            if !self.obstacles[start.0][start.1] {
                self.start = start;
            }
        }
        if let Some(goal) = crate::share::value(query, "goal").and_then(Self::decode_point) {
            if !self.obstacles[goal.0][goal.1] {
                self.goal = goal;
            }
        }
        self.compare_results.clear();
        self.replan();
    }

    pub(crate) fn share_query(&self) -> String {
        format!(
            "tab=grid&planner={}&start={},{}&goal={},{}&map={}",
            self.planner.slug(),
            self.start.0,
            self.start.1,
            self.goal.0,
            self.goal.1,
            self.encode_map()
        )
    }

    fn decode_point(value: &str) -> Option<(usize, usize)> {
        let (x, y) = value.split_once(',')?;
        let point = (x.parse().ok()?, y.parse().ok()?);
        (point.0 < GRID_W && point.1 < GRID_H).then_some(point)
    }

    fn encode_map(&self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut encoded = String::with_capacity(GRID_W * GRID_H / 4);
        for group in 0..(GRID_W * GRID_H / 4) {
            let mut nibble = 0_u8;
            for bit in 0..4 {
                let index = group * 4 + bit;
                let x = index % GRID_W;
                let y = index / GRID_W;
                if self.obstacles[x][y] {
                    nibble |= 1 << bit;
                }
            }
            encoded.push(HEX[nibble as usize] as char);
        }
        encoded
    }

    fn decode_map(value: &str) -> Option<Vec<Vec<bool>>> {
        if value.len() != GRID_W * GRID_H / 4 {
            return None;
        }
        let mut obstacles = vec![vec![false; GRID_H]; GRID_W];
        for (group, digit) in value.bytes().enumerate() {
            let nibble = match digit {
                b'0'..=b'9' => digit - b'0',
                b'a'..=b'f' => digit - b'a' + 10,
                b'A'..=b'F' => digit - b'A' + 10,
                _ => return None,
            };
            for bit in 0..4 {
                let index = group * 4 + bit;
                let x = index % GRID_W;
                let y = index / GRID_W;
                obstacles[x][y] = nibble & (1 << bit) != 0;
            }
        }
        Some(obstacles)
    }

    fn set_border_obstacles(&mut self) {
        for x in 0..GRID_W {
            self.obstacles[x][0] = true;
            self.obstacles[x][GRID_H - 1] = true;
        }
        for y in 0..GRID_H {
            self.obstacles[0][y] = true;
            self.obstacles[GRID_W - 1][y] = true;
        }
    }

    fn add_default_wall(&mut self) {
        for y in 6..GRID_H - 6 {
            self.obstacles[15][y] = true;
        }
        self.obstacles[15][GRID_H / 2] = false;
    }

    fn clear_interior(&mut self) {
        for x in 1..GRID_W - 1 {
            for y in 1..GRID_H - 1 {
                self.obstacles[x][y] = false;
            }
        }
        self.set_border_obstacles();
        self.replan();
    }

    fn cell_obstacles(&self) -> Obstacles {
        let mut obstacles = Obstacles::new();
        for x in 0..GRID_W {
            for y in 0..GRID_H {
                if self.obstacles[x][y] {
                    obstacles.push(Point2D::new(x as f64, y as f64));
                }
            }
        }
        obstacles
    }

    fn dijkstra_map(&self) -> grid_nalgebra::Map {
        let mut data = vec![0i32; GRID_W * GRID_H];
        for x in 0..GRID_W {
            for y in 0..GRID_H {
                data[y * GRID_W + x] = if self.obstacles[x][y] { 1 } else { 0 };
            }
        }
        let matrix = nalgebra::DMatrix::from_row_slice(GRID_H, GRID_W, &data);
        grid_nalgebra::Map::new(matrix, 1).expect("valid dijkstra map")
    }

    fn plan_with(planner: PlannerKind, demo: &Self) -> PlanSnapshot {
        let start = Instant::now();
        let obstacles = demo.cell_obstacles();
        let start_pt = Point2D::new(demo.start.0 as f64, demo.start.1 as f64);
        let goal_pt = Point2D::new(demo.goal.0 as f64, demo.goal.1 as f64);

        let (success, path_len, waypoint_count) = match planner {
            PlannerKind::AStar => {
                let result = AStarPlanner::from_obstacle_points(
                    &obstacles,
                    AStarConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt));
                Self::snapshot_from_path(result)
            }
            PlannerKind::Jps => {
                let result = JPSPlanner::from_obstacle_points(
                    &obstacles,
                    JPSConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt));
                Self::snapshot_from_path(result)
            }
            PlannerKind::ThetaStar => {
                let result = ThetaStarPlanner::from_obstacle_points(
                    &obstacles,
                    ThetaStarConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt));
                Self::snapshot_from_path(result)
            }
            PlannerKind::Dijkstra => {
                let map = demo.dijkstra_map();
                let (sr, sc) = (demo.start.1, demo.start.0);
                let (gr, gc) = (demo.goal.1, demo.goal.0);
                if has_collision(&map, sr, sc) || has_collision(&map, gr, gc) {
                    (false, 0.0, 0)
                } else if let Some(path) = dijkstra_plan(&map, (sr, sc), (gr, gc)) {
                    let len = Self::path_length_cells(&path);
                    (true, len, path.len())
                } else {
                    (false, 0.0, 0)
                }
            }
        };

        let elapsed_us = start.elapsed().as_secs_f64() * 1_000_000.0;
        PlanSnapshot {
            planner,
            path_len,
            waypoint_count,
            elapsed_us,
            success,
        }
    }

    fn snapshot_from_path(
        result: RoboticsResult<rust_robotics_core::Path2D>,
    ) -> (bool, f64, usize) {
        match result {
            Ok(path) => (true, path.total_length(), path.len()),
            Err(_) => (false, 0.0, 0),
        }
    }

    fn path_length_cells(path: &[(usize, usize)]) -> f64 {
        path.windows(2)
            .map(|w| {
                let dx = w[1].0 as f64 - w[0].0 as f64;
                let dy = w[1].1 as f64 - w[0].1 as f64;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    fn path_for(&self, planner: PlannerKind) -> Vec<(usize, usize)> {
        let obstacles = self.cell_obstacles();
        let start_pt = Point2D::new(self.start.0 as f64, self.start.1 as f64);
        let goal_pt = Point2D::new(self.goal.0 as f64, self.goal.1 as f64);

        match planner {
            PlannerKind::AStar => Self::path_from_result(
                AStarPlanner::from_obstacle_points(
                    &obstacles,
                    AStarConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt)),
            ),
            PlannerKind::Jps => Self::path_from_result(
                JPSPlanner::from_obstacle_points(
                    &obstacles,
                    JPSConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt)),
            ),
            PlannerKind::ThetaStar => Self::path_from_result(
                ThetaStarPlanner::from_obstacle_points(
                    &obstacles,
                    ThetaStarConfig {
                        resolution: RESOLUTION,
                        robot_radius: ROBOT_RADIUS,
                        heuristic_weight: 1.0,
                    },
                )
                .and_then(|p| p.plan(start_pt, goal_pt)),
            ),
            PlannerKind::Dijkstra => {
                let map = self.dijkstra_map();
                let (sr, sc) = (self.start.1, self.start.0);
                let (gr, gc) = (self.goal.1, self.goal.0);
                dijkstra_plan(&map, (sr, sc), (gr, gc))
                    .map(|path| path.into_iter().map(|(r, c)| (c, r)).collect())
                    .unwrap_or_default()
            }
        }
    }

    fn path_from_result(result: RoboticsResult<rust_robotics_core::Path2D>) -> Vec<(usize, usize)> {
        result
            .ok()
            .map(|path| {
                path.points
                    .iter()
                    .map(|p| (p.x.round() as usize, p.y.round() as usize))
                    .collect()
            })
            .unwrap_or_default()
    }

    fn replan(&mut self) {
        let snapshot = Self::plan_with(self.planner, self);
        if snapshot.elapsed_us > 0.0 {
            self.plans_per_sec = 1_000_000.0 / snapshot.elapsed_us;
        }
        self.path = if snapshot.success {
            self.path_for(self.planner)
        } else {
            Vec::new()
        };
        self.last_plan = Some(snapshot);
    }

    fn compare_all(&mut self) {
        self.compare_results = PlannerKind::ALL
            .iter()
            .map(|&kind| Self::plan_with(kind, self))
            .collect();
        self.replan();
    }

    fn grid_rect(&self, ui: &egui::Ui) -> (Rect, f32) {
        let side = ui.available_width().min(ui.available_height() - 8.0);
        let cell = side / GRID_W as f32;
        let size = Vec2::new(cell * GRID_W as f32, cell * GRID_H as f32);
        let origin = ui.cursor().min;
        (Rect::from_min_size(origin, size), cell)
    }

    fn cell_at(&self, rect: Rect, cell: f32, pos: Pos2) -> Option<(usize, usize)> {
        if !rect.contains(pos) {
            return None;
        }
        let local = pos - rect.min;
        let x = (local.x / cell).floor() as usize;
        let y = (local.y / cell).floor() as usize;
        if x < GRID_W && y < GRID_H {
            Some((x, y))
        } else {
            None
        }
    }

    fn handle_pointer(&mut self, rect: Rect, cell: f32, response: &egui::Response) {
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some((x, y)) = self.cell_at(rect, cell, pos) {
                if self.edit_mode == EditMode::Obstacles && response.clicked() {
                    if x > 0 && x + 1 < GRID_W && y > 0 && y + 1 < GRID_H {
                        self.obstacles[x][y] = !self.obstacles[x][y];
                        self.replan();
                    }
                } else if self.edit_mode == EditMode::StartGoal {
                    if response.drag_started() {
                        self.dragging_start = (x, y) == self.start;
                        self.dragging_goal = (x, y) == self.goal;
                    }
                    if response.dragged() {
                        if self.dragging_start && !self.obstacles[x][y] {
                            self.start = (x, y);
                            self.replan();
                        } else if self.dragging_goal && !self.obstacles[x][y] {
                            self.goal = (x, y);
                            self.replan();
                        }
                    }
                }
            }
        }
        if response.drag_stopped() {
            self.dragging_start = false;
            self.dragging_goal = false;
        }
    }

    fn draw_grid(&self, ui: &mut egui::Ui, rect: Rect, cell: f32) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(28));

        for x in 0..GRID_W {
            for y in 0..GRID_H {
                let min = rect.min + Vec2::new(x as f32 * cell, y as f32 * cell);
                let cell_rect = Rect::from_min_size(min, Vec2::splat(cell));
                let color = if self.obstacles[x][y] {
                    Color32::from_rgb(55, 55, 70)
                } else {
                    Color32::from_rgb(34, 38, 46)
                };
                painter.rect_filled(cell_rect.shrink(0.5), 1.0, color);
            }
        }

        if self.path.len() >= 2 {
            let points: Vec<Pos2> = self
                .path
                .iter()
                .map(|&(x, y)| {
                    rect.min + Vec2::new((x as f32 + 0.5) * cell, (y as f32 + 0.5) * cell)
                })
                .collect();
            painter.add(egui::Shape::line(
                points,
                Stroke::new(2.5_f32, Color32::from_rgb(80, 180, 255)),
            ));
        }

        let draw_marker = |painter: &egui::Painter, center: (usize, usize), color: Color32| {
            let c = rect.min
                + Vec2::new(
                    (center.0 as f32 + 0.5) * cell,
                    (center.1 as f32 + 0.5) * cell,
                );
            painter.circle_filled(c, cell * 0.32, color);
        };
        draw_marker(&painter, self.start, Color32::from_rgb(90, 220, 120));
        draw_marker(&painter, self.goal, Color32::from_rgb(240, 90, 90));
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Planner:");
            for kind in PlannerKind::ALL {
                if ui
                    .selectable_label(self.planner == kind, kind.label())
                    .clicked()
                {
                    self.planner = kind;
                    self.replan();
                }
            }
            ui.separator();
            ui.label("Edit:");
            ui.selectable_value(&mut self.edit_mode, EditMode::Obstacles, "Obstacles");
            ui.selectable_value(&mut self.edit_mode, EditMode::StartGoal, "Start / Goal");
            if ui.button("Compare all").clicked() {
                self.compare_all();
            }
            if ui.button("Reset map").clicked() {
                *self = Self::default();
            }
            if ui.button("Clear interior").clicked() {
                self.clear_interior();
            }
        });

        if let Some(plan) = &self.last_plan {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "{}: {} waypoints, length {:.1}, {:.1} µs ({:.0} plans/s)",
                    plan.planner.label(),
                    plan.waypoint_count,
                    plan.path_len,
                    plan.elapsed_us,
                    self.plans_per_sec
                ));
                if !plan.success {
                    ui.colored_label(Color32::LIGHT_RED, "no path");
                }
            });
        }

        if !self.compare_results.is_empty() {
            egui::Grid::new("compare_grid").show(ui, |ui| {
                ui.label("Planner");
                ui.label("Length");
                ui.label("Waypoints");
                ui.label("Time (µs)");
                ui.end_row();
                for row in &self.compare_results {
                    ui.label(row.planner.label());
                    ui.label(if row.success {
                        format!("{:.1}", row.path_len)
                    } else {
                        "—".to_string()
                    });
                    ui.label(if row.success {
                        row.waypoint_count.to_string()
                    } else {
                        "—".to_string()
                    });
                    ui.label(format!("{:.1}", row.elapsed_us));
                    ui.end_row();
                }
            });
        }

        ui.add_space(8.0);
        let (rect, cell) = self.grid_rect(ui);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_pointer(rect, cell, &response);
        self.draw_grid(ui, rect, cell);
    }
}

#[cfg(test)]
mod tests {
    use super::{GridPlannerDemo, PlannerKind};

    #[test]
    fn shared_grid_state_round_trips() {
        let original = GridPlannerDemo::default();
        let query = original.share_query();
        let mut restored = GridPlannerDemo::default();
        restored.clear_interior();
        restored.apply_share_query(&query);

        assert_eq!(restored.planner, PlannerKind::AStar);
        assert_eq!(restored.start, original.start);
        assert_eq!(restored.goal, original.goal);
        assert_eq!(restored.obstacles, original.obstacles);
    }

    #[test]
    fn malformed_shared_state_is_ignored() {
        let mut demo = GridPlannerDemo::default();
        let original_map = demo.obstacles.clone();
        demo.apply_share_query("planner=unknown&start=99,99&map=xyz");
        assert_eq!(demo.planner, PlannerKind::AStar);
        assert_eq!(demo.obstacles, original_map);
    }
}
