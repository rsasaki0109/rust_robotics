//! Interactive grid planner demo: A*, Dijkstra, JPS, and Theta*.

mod search;

use web_time::Instant;

use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};
use rust_robotics_core::{Obstacles, Point2D, RoboticsResult};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::dijkstra::{DijkstraConfig, DijkstraPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const GRID_W: usize = 32;
const GRID_H: usize = 24;
const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.35;
type Cell = (usize, usize);

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

    fn observes_search(self) -> bool {
        matches!(self, Self::AStar | Self::Dijkstra)
    }
}

/// What a drag on the grid does, decided by where it started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DragTool {
    Start,
    Goal,
    /// Paint walls (`true`) or erase them (`false`).
    Paint(bool),
}

/// Grid cells on the line from `a` to `b` (Bresenham), both ends included.
fn cells_between(a: (usize, usize), b: (usize, usize)) -> Vec<(usize, usize)> {
    let (mut x, mut y) = (a.0 as i64, a.1 as i64);
    let (x1, y1) = (b.0 as i64, b.1 as i64);
    let (dx, dy) = ((x1 - x).abs(), -(y1 - y).abs());
    let (sx, sy) = ((x1 - x).signum(), (y1 - y).signum());
    let mut err = dx + dy;
    let mut cells = vec![(x as usize, y as usize)];
    while (x, y) != (x1, y1) {
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
        cells.push((x as usize, y as usize));
    }
    cells
}

#[derive(Debug, Clone)]
struct PlanSnapshot {
    planner: PlannerKind,
    path_len: f64,
    waypoint_count: usize,
    elapsed_us: f64,
    success: bool,
    expanded: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct GridPlannerDemo {
    obstacles: Vec<Vec<bool>>,
    start: (usize, usize),
    goal: (usize, usize),
    planner: PlannerKind,
    path: Vec<(usize, usize)>,
    last_plan: Option<PlanSnapshot>,
    compare_results: Vec<PlanSnapshot>,
    drag: Option<DragTool>,
    /// Cell painted by the previous drag event.
    last_paint: Option<(usize, usize)>,
    search: search::SearchReplay,
    actions: std::collections::BTreeMap<&'static str, [f32; 2]>,
    action_hits: std::collections::BTreeMap<&'static str, bool>,
    scene_rect: Option<Rect>,
    #[cfg(target_arch = "wasm32")]
    action_count: u64,
    #[cfg(target_arch = "wasm32")]
    last_action: &'static str,
    #[cfg(target_arch = "wasm32")]
    ui_revision: u64,
}

impl Default for GridPlannerDemo {
    fn default() -> Self {
        let mut demo = Self {
            obstacles: vec![vec![false; GRID_H]; GRID_W],
            start: (2, GRID_H / 2),
            goal: (GRID_W - 3, GRID_H / 2),
            planner: PlannerKind::AStar,
            path: Vec::new(),
            last_plan: None,
            compare_results: Vec::new(),
            drag: None,
            last_paint: None,
            search: search::SearchReplay::default(),
            actions: Default::default(),
            action_hits: Default::default(),
            scene_rect: None,
            #[cfg(target_arch = "wasm32")]
            action_count: 0,
            #[cfg(target_arch = "wasm32")]
            last_action: "",
            #[cfg(target_arch = "wasm32")]
            ui_revision: 0,
        };
        demo.set_border_obstacles();
        demo.add_default_wall();
        demo.replan();
        demo
    }
}

impl GridPlannerDemo {
    pub(crate) fn run_guided_comparison(&mut self) {
        self.compare_all();
    }

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
        self.search.apply_query(query);
    }

    pub(crate) fn share_query(&self) -> String {
        format!(
            "tab=grid&planner={}&start={},{}&goal={},{}&map={}&search={}&search_step={}&search_playing={}&search_rate={}",
            self.planner.slug(),
            self.start.0,
            self.start.1,
            self.goal.0,
            self.goal.1,
            self.encode_map(),
            u8::from(self.search.visible),
            self.search.cursor,
            u8::from(self.search.playing),
            self.search.rate,
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

    fn run_plan(&self, planner: PlannerKind) -> (PlanSnapshot, Vec<Cell>, Vec<Cell>) {
        let timer = Instant::now();
        let obstacles = self.cell_obstacles();
        let start = Point2D::new(self.start.0 as f64, self.start.1 as f64);
        let goal = Point2D::new(self.goal.0 as f64, self.goal.1 as f64);
        let mut expanded = Vec::new();
        let mut observe = |p: Point2D| expanded.push((p.x.round() as usize, p.y.round() as usize));
        let result = match planner {
            PlannerKind::AStar => AStarPlanner::from_obstacle_points(
                &obstacles,
                AStarConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                    heuristic_weight: 1.0,
                },
            )
            .and_then(|p| p.plan_with_observer(start, goal, &mut observe)),
            PlannerKind::Dijkstra => DijkstraPlanner::from_obstacle_points(
                &obstacles,
                DijkstraConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                },
            )
            .and_then(|p| p.plan_with_observer(start, goal, &mut observe)),
            PlannerKind::Jps => JPSPlanner::from_obstacle_points(
                &obstacles,
                JPSConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                    heuristic_weight: 1.0,
                },
            )
            .and_then(|p| p.plan(start, goal)),
            PlannerKind::ThetaStar => ThetaStarPlanner::from_obstacle_points(
                &obstacles,
                ThetaStarConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                    heuristic_weight: 1.0,
                },
            )
            .and_then(|p| p.plan(start, goal)),
        };
        let snapshot = PlanSnapshot {
            planner,
            path_len: result.as_ref().map_or(0.0, |p| p.total_length()),
            waypoint_count: result.as_ref().map_or(0, |p| p.len()),
            elapsed_us: timer.elapsed().as_secs_f64() * 1_000_000.0,
            success: result.is_ok(),
            expanded: planner.observes_search().then_some(expanded.len()),
        };
        (snapshot, Self::path_from_result(result), expanded)
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
        let (snapshot, path, expanded) = self.run_plan(self.planner);
        self.path = path;
        self.search.reset(expanded);
        self.last_plan = Some(snapshot);
        self.compare_results.clear();
    }

    fn compare_all(&mut self) {
        let results = PlannerKind::ALL
            .into_iter()
            .map(|kind| self.run_plan(kind).0)
            .collect();
        self.replan();
        self.compare_results = results;
    }

    fn preset(&mut self, blocked: bool) {
        self.obstacles = vec![vec![false; GRID_H]; GRID_W];
        self.set_border_obstacles();
        for y in 1..GRID_H - 1 {
            self.obstacles[15][y] = blocked || y != 5;
        }
        self.start = (2, GRID_H / 2);
        self.goal = (GRID_W - 3, GRID_H / 2);
        self.replan();
    }

    fn action(&mut self, name: &'static str, response: &egui::Response) {
        self.actions
            .insert(name, [response.rect.center().x, response.rect.center().y]);
        self.action_hits.insert(name, response.hovered());
        if response.clicked() {
            #[cfg(target_arch = "wasm32")]
            {
                self.action_count += 1;
                self.last_action = name;
            }
            response.ctx.request_repaint();
        }
    }

    fn grid_rect(ui: &egui::Ui) -> (Rect, f32) {
        let rect = crate::ui_kit::fit_rect(ui, GRID_H as f32 / GRID_W as f32, 122.0);
        (rect, rect.width() / GRID_W as f32)
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

    /// Click toggles a wall; dragging from the start or goal moves it,
    /// dragging anywhere else paints (or erases) walls.
    fn handle_pointer(&mut self, rect: Rect, cell: f32, response: &egui::Response) {
        let interior = |(x, y): (usize, usize)| x > 0 && x + 1 < GRID_W && y > 0 && y + 1 < GRID_H;
        let pointer_cell = response
            .interact_pointer_pos()
            .and_then(|pos| self.cell_at(rect, cell, pos));
        if response.clicked() {
            if let Some(at) = pointer_cell {
                if interior(at) && at != self.start && at != self.goal {
                    self.obstacles[at.0][at.1] = !self.obstacles[at.0][at.1];
                    self.replan();
                }
            }
        }
        if response.drag_started() {
            let origin = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .and_then(|pos| self.cell_at(rect, cell, pos));
            self.drag = origin.map(|at| {
                if at == self.start {
                    DragTool::Start
                } else if at == self.goal {
                    DragTool::Goal
                } else {
                    DragTool::Paint(!self.obstacles[at.0][at.1])
                }
            });
            // Paint from where the press started, not from where the drag
            // threshold was crossed.
            self.last_paint = origin;
        }
        if response.dragged() {
            if let (Some(tool), Some((x, y))) = (self.drag, pointer_cell) {
                let free = !self.obstacles[x][y];
                match tool {
                    DragTool::Start if free && (x, y) != self.goal && (x, y) != self.start => {
                        self.start = (x, y);
                        self.replan();
                    }
                    DragTool::Goal if free && (x, y) != self.start && (x, y) != self.goal => {
                        self.goal = (x, y);
                        self.replan();
                    }
                    DragTool::Paint(wall) => {
                        // Fill the whole segment since the last event, so a
                        // fast drag still draws a solid wall.
                        let from = self.last_paint.unwrap_or((x, y));
                        let mut changed = false;
                        for (cx, cy) in cells_between(from, (x, y)) {
                            if interior((cx, cy))
                                && (cx, cy) != self.start
                                && (cx, cy) != self.goal
                                && self.obstacles[cx][cy] != wall
                            {
                                self.obstacles[cx][cy] = wall;
                                changed = true;
                            }
                        }
                        self.last_paint = Some((x, y));
                        if changed {
                            self.replan();
                        }
                    }
                    _ => {}
                }
            }
        }
        if response.drag_stopped() {
            self.drag = None;
            self.last_paint = None;
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

        if self.search.visible && self.planner.observes_search() {
            for (i, &(x, y)) in self.search.cells[..self.search.cursor].iter().enumerate() {
                let min = rect.min + Vec2::new(x as f32 * cell, y as f32 * cell);
                let r = Rect::from_min_size(min, Vec2::splat(cell)).shrink(1.0);
                let recent = self.search.cursor.saturating_sub(i) <= 5;
                painter.rect_filled(
                    r,
                    2.0,
                    if recent {
                        Color32::from_rgb(42, 112, 143)
                    } else {
                        Color32::from_rgb(29, 64, 85)
                    },
                );
            }
            if let Some(&(x, y)) = self.search.cells[..self.search.cursor].last() {
                let min = rect.min + Vec2::new(x as f32 * cell, y as f32 * cell);
                painter.rect_stroke(
                    Rect::from_min_size(min, Vec2::splat(cell)).shrink(1.0),
                    2.0,
                    Stroke::new(2.0_f32, Color32::from_rgb(255, 195, 92)),
                    egui::StrokeKind::Inside,
                );
            }
        }

        if self.path.len() >= 2 && self.search.complete() {
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

    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Planner");
        ui.horizontal_wrapped(|ui| {
            for kind in PlannerKind::ALL {
                let response = ui.selectable_label(self.planner == kind, kind.label());
                self.action(kind.slug(), &response);
                if response.clicked() {
                    self.planner = kind;
                    self.replan();
                }
            }
        });
        let compare = ui.button("Compare all four").on_hover_text(
            "Same map, radius, and diagonal collision rules; replay A* and Dijkstra below",
        );
        self.action("compare", &compare);
        if compare.clicked() {
            self.compare_all();
        }

        if !self.compare_results.is_empty() {
            crate::ui_kit::section(ui, "Comparison");
            egui::Grid::new("compare_grid")
                .striped(true)
                .num_columns(5)
                .show(ui, |ui| {
                    ui.label("");
                    ui.small("length");
                    ui.small("points");
                    ui.small("cells");
                    ui.small("µs");
                    ui.end_row();
                    for row in &self.compare_results {
                        ui.strong(row.planner.label());
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
                        ui.label(
                            row.expanded
                                .map_or_else(|| "—".to_string(), |count| count.to_string()),
                        );
                        ui.label(format!("{:.0}", row.elapsed_us));
                        ui.end_row();
                    }
                });
        }

        crate::ui_kit::section(ui, "Map");
        ui.horizontal_wrapped(|ui| {
            let clear = ui.button("Clear walls");
            self.action("clear", &clear);
            if clear.clicked() {
                self.clear_interior();
            }
            let detour = ui.button("Detour");
            self.action("detour", &detour);
            if detour.clicked() {
                self.preset(false);
            }
            let blocked = ui.button("No route");
            self.action("blocked", &blocked);
            if blocked.clicked() {
                self.preset(true);
            }
            let reset = ui.button("Reset");
            self.action("reset", &reset);
            if reset.clicked() {
                *self = Self {
                    actions: std::mem::take(&mut self.actions),
                    action_hits: std::mem::take(&mut self.action_hits),
                    scene_rect: self.scene_rect,
                    #[cfg(target_arch = "wasm32")]
                    action_count: self.action_count,
                    #[cfg(target_arch = "wasm32")]
                    last_action: self.last_action,
                    #[cfg(target_arch = "wasm32")]
                    ui_revision: self.ui_revision,
                    ..Self::default()
                };
            }
        });
        crate::ui_kit::hint(
            ui,
            "Click or drag on the grid to draw walls. Drag the green start or the red goal to move them.",
        );
        if self.planner.observes_search() {
            crate::ui_kit::section(ui, "Replay speed");
            ui.add(egui::Slider::new(&mut self.search.rate, 5..=200).text("cells/s"));
            crate::ui_kit::hint(ui, "Viewing speed, not computation time. Both planners use the same grid and robot radius.");
        }
        // On phones controls follow the scene in a scroll area. Publish after
        // both have been laid out, so their coordinates refer to this frame.
        #[cfg(target_arch = "wasm32")]
        {
            self.ui_revision += 1;
        }
        if let Some(rect) = self.scene_rect {
            self.publish_state(ctx, rect);
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.search.advance(ctx.input(|i| i.time));
        if self.search.playing {
            ctx.request_repaint();
        }
        let (rect, cell) = Self::grid_rect(ui);
        self.scene_rect = Some(rect);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.action("map", &response);
        self.handle_pointer(rect, cell, &response);
        self.draw_grid(ui, rect, cell);
        self.search_controls(ui);
        if self.search.playing {
            ctx.request_repaint();
        }
        if let Some(plan) = &self.last_plan {
            ui.horizontal_wrapped(|ui| {
                if !self.search.complete() {
                    ui.label(format!(
                        "{}: exploring · {} / {} expanded cells",
                        self.planner.label(),
                        self.search.cursor,
                        self.search.cells.len()
                    ));
                } else if plan.success {
                    let time = if plan.elapsed_us >= 1.0 {
                        format!("  ·  {:.0} µs", plan.elapsed_us)
                    } else {
                        String::new()
                    };
                    ui.label(format!(
                        "{}  ·  length {:.1}  ·  {} waypoints{time}",
                        plan.planner.label(),
                        plan.path_len,
                        plan.waypoint_count,
                    ));
                } else {
                    ui.colored_label(
                        Color32::LIGHT_RED,
                        format!("{}: no path to the goal", plan.planner.label()),
                    );
                }
            });
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.ui_revision += 1;
        }
        self.publish_state(ctx, rect);
    }

    fn search_controls(&mut self, ui: &mut egui::Ui) {
        if !self.planner.observes_search() {
            for key in ["replay", "pause", "next", "result", "visible"] {
                self.actions.remove(key);
            }
            crate::ui_kit::hint(ui, "Select A* or Dijkstra to replay expanded cells.");
            return;
        }
        ui.horizontal_wrapped(|ui| {
            let replay = ui.add_enabled(
                !self.search.cells.is_empty(),
                egui::Button::new("Replay search"),
            );
            self.action("replay", &replay);
            if replay.clicked() {
                self.search.replay();
            }
            let pause = ui.add_enabled(
                !self.search.complete(),
                egui::Button::new(if self.search.playing { "Pause" } else { "Play" }),
            );
            self.action("pause", &pause);
            if pause.clicked() {
                self.search.playing = !self.search.playing;
            }
            let next = ui.add_enabled(!self.search.complete(), egui::Button::new("Next"));
            self.action("next", &next);
            if next.clicked() {
                self.search.seek(self.search.cursor + 1);
            }
            let result = ui.add_enabled(!self.search.complete(), egui::Button::new("Result"));
            self.action("result", &result);
            if result.clicked() {
                self.search.seek(self.search.cells.len());
            }
        });
        ui.horizontal_wrapped(|ui| {
            let mut cursor = self.search.cursor;
            let slider =
                ui.add(egui::Slider::new(&mut cursor, 0..=self.search.cells.len()).text("step"));
            if slider.changed() {
                self.search.seek(cursor);
            }
            let visible = ui.checkbox(&mut self.search.visible, "Search cells");
            self.action("visible", &visible);
            if visible.changed() && !self.search.visible {
                self.search.seek(self.search.cells.len());
            }
        });
        crate::ui_kit::legend(
            ui,
            &[
                (Color32::from_rgb(42, 112, 143), "Expanded"),
                (Color32::from_rgb(255, 195, 92), "Current"),
                (Color32::from_rgb(80, 180, 255), "Route"),
            ],
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_state(&self, _ctx: &egui::Context, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_state(&self, ctx: &egui::Context, rect: Rect) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let encode = |cells: &[Cell]| {
            cells
                .iter()
                .map(|(x, y)| format!("{x},{y}"))
                .collect::<Vec<_>>()
                .join(";")
        };
        let actions = self
            .actions
            .iter()
            .map(|(key, p)| format!("\"{key}\":[{:.2},{:.2}]", p[0], p[1]))
            .collect::<Vec<_>>()
            .join(",");
        let hits = self
            .action_hits
            .iter()
            .map(|(key, hit)| format!("\"{key}\":{hit}"))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = ctx
            .input(|i| i.pointer.hover_pos())
            .map_or_else(|| "null".to_owned(), |p| format!("[{:.2},{:.2}]", p.x, p.y));
        let found = if !self.search.complete() {
            "pending"
        } else if self.path.is_empty() {
            "0"
        } else {
            "1"
        };
        for (key, value) in [
            ("data-grid-planner", self.planner.slug().to_owned()),
            ("data-grid-search-step", self.search.cursor.to_string()),
            (
                "data-grid-search-total",
                self.search.cells.len().to_string(),
            ),
            (
                "data-grid-playing",
                u8::from(self.search.playing).to_string(),
            ),
            ("data-grid-found", found.to_owned()),
            (
                "data-grid-path",
                if self.search.complete() {
                    encode(&self.path)
                } else {
                    String::new()
                },
            ),
            (
                "data-grid-expanded",
                if self.search.visible {
                    encode(&self.search.cells[..self.search.cursor])
                } else {
                    String::new()
                },
            ),
            ("data-grid-share", self.share_query()),
            (
                "data-grid-rect",
                format!(
                    "[{:.2},{:.2},{:.2},{:.2}]",
                    rect.left(),
                    rect.top(),
                    rect.width(),
                    rect.height()
                ),
            ),
            ("data-grid-actions", format!("{{{actions}}}")),
            ("data-grid-hits", format!("{{{hits}}}")),
            ("data-grid-pointer", pointer),
            ("data-grid-ui-revision", self.ui_revision.to_string()),
            ("data-grid-action-count", self.action_count.to_string()),
            ("data-grid-last-action", self.last_action.to_owned()),
            (
                "aria-label",
                format!(
                    "Grid search, {}: {} of {} expanded cells, {}",
                    self.planner.label(),
                    self.search.cursor,
                    self.search.cells.len(),
                    if found == "pending" {
                        "exploring"
                    } else if found == "1" {
                        "path found"
                    } else {
                        "no route"
                    }
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{GridPlannerDemo, PlannerKind, GRID_W};
    use egui::{Pos2, Rect, Vec2};

    /// Drags the pointer over grid cells inside a full frame loop.
    fn drag(demo: &mut GridPlannerDemo, cells: &[(usize, usize)]) {
        let ctx = egui::Context::default();
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 800.0));
        let grid = std::cell::Cell::new(None);
        let run = |events: Vec<egui::Event>, demo: &mut GridPlannerDemo| {
            let input = egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    grid.set(Some(GridPlannerDemo::grid_rect(ui)));
                    demo.scene(ctx, ui);
                });
            });
        };
        run(Vec::new(), demo);
        let (rect, cell) = grid.get().expect("grid drawn");
        let at = |(x, y): (usize, usize)| {
            rect.min + Vec2::new((x as f32 + 0.5) * cell, (y as f32 + 0.5) * cell)
        };
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        };
        run(vec![egui::Event::PointerMoved(at(cells[0]))], demo);
        run(vec![button(at(cells[0]), true)], demo);
        for &c in &cells[1..] {
            run(vec![egui::Event::PointerMoved(at(c))], demo);
        }
        let last = *cells.last().unwrap();
        run(vec![button(at(last), false)], demo);
    }

    #[test]
    fn dragging_paints_walls_and_moves_the_start() {
        let mut demo = GridPlannerDemo::default();
        demo.clear_interior();
        // A fast stroke: only its two ends arrive as pointer events.
        drag(&mut demo, &[(8, 5), (9, 5), (14, 5)]);
        assert!((8..=14).all(|x| demo.obstacles[x][5]), "gaps in the wall");

        // Dragging from a wall erases instead.
        drag(&mut demo, &[(14, 5), (12, 5), (10, 5)]);
        assert!(!demo.obstacles[12][5] && !demo.obstacles[10][5]);

        let start = demo.start;
        let target = (start.0 + 3, start.1 + 2);
        drag(&mut demo, &[start, (start.0 + 1, start.1 + 1), target]);
        assert_eq!(demo.start, target);
        assert!(
            !demo.obstacles[start.0 + 1][start.1 + 1],
            "moving the start painted"
        );
        assert!(target.0 < GRID_W);
    }

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

    #[test]
    fn comparison_uses_the_same_gap_and_discards_results_after_edits() {
        let mut demo = GridPlannerDemo::default();
        demo.compare_all();
        let a = &demo.compare_results[0];
        let d = &demo.compare_results[1];
        assert!(
            a.success && d.success,
            "the one-cell gap must be open to both planners"
        );
        assert!((a.path_len - d.path_len).abs() < 1e-10);
        assert!(a.expanded.unwrap() < d.expanded.unwrap());
        demo.planner = PlannerKind::Dijkstra;
        demo.preset(true);
        assert!(
            demo.compare_results.is_empty(),
            "stale comparison survived a map edit"
        );
        assert!(!demo.last_plan.as_ref().unwrap().success);
        assert!(!demo.search.cells.is_empty());
        assert!(demo.search.cells.iter().all(|&(x, _)| x < 15));
        assert!(demo.search.complete() && !demo.search.playing);
        demo.clear_interior();
        assert!(demo.last_plan.as_ref().unwrap().success);
    }

    #[test]
    fn sharing_restores_the_exact_paused_search_and_validates_playback_values() {
        let mut original = GridPlannerDemo {
            planner: PlannerKind::Dijkstra,
            ..Default::default()
        };
        original.preset(false);
        original.search.rate = 5;
        original.search.seek(17);
        let mut restored = GridPlannerDemo::default();
        restored.apply_share_query(&original.share_query());
        assert_eq!(restored.path, original.path);
        assert_eq!(restored.search.cells, original.search.cells);
        assert_eq!(restored.search.cursor, 17);
        assert_eq!(restored.search.rate, 5);
        assert!(!restored.search.playing);
        restored.apply_share_query("tab=grid&search_step=999999999&search_rate=0&search_playing=1");
        assert!(restored.search.complete());
        assert!(!restored.search.playing);
        assert_eq!(restored.search.rate, 5);
        restored.apply_share_query("tab=grid&search=0&search_step=0&search_playing=1");
        assert!(restored.search.complete());
        assert!(!restored.search.visible && !restored.search.playing);
    }
}
