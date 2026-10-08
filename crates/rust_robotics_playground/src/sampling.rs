//! Sampling-based planners: RRT, RRT*, Informed RRT*, and PRM on a field of
//! circular obstacles. Each plan is replayed as a growing tree (or roadmap),
//! so the difference in how they explore is visible.

use web_time::Instant;

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};
use rust_robotics_planning::{
    AreaBounds, CircleObstacle, InformedRRTStar, PRMPlanner, RRTConfig, RRTPlanner, RRTStar,
};

/// The world is the square `[0, WORLD]²` \[m\].
const WORLD: f64 = 20.0;
const ROBOT_RADIUS: f64 = 0.3;
const EXPAND_DIS: f64 = 1.5;
/// Iterations of the asymptotically optimal planners (they keep improving
/// until the budget is spent).
const STAR_ITERATIONS: usize = 700;
const MAX_OBSTACLES: usize = 24;
/// How close \[m\] a press must be to the start or goal to drag it.
const GRAB_RADIUS: f64 = 0.8;
/// Seconds the tree takes to grow on screen.
const GROW_SECONDS: f64 = 1.6;

const OBSTACLE: Color32 = Color32::from_rgb(70, 76, 92);
const TREE: Color32 = Color32::from_rgba_premultiplied(70, 130, 190, 150);
const PATH: Color32 = Color32::from_rgb(255, 152, 56);
const START: Color32 = Color32::from_rgb(90, 220, 120);
const GOAL: Color32 = Color32::from_rgb(240, 90, 90);
const ELLIPSE: Color32 = Color32::from_rgba_premultiplied(120, 100, 40, 120);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SamplingKind {
    Rrt,
    RrtStar,
    InformedRrtStar,
    Prm,
}

impl SamplingKind {
    const ALL: [Self; 4] = [Self::Rrt, Self::RrtStar, Self::InformedRrtStar, Self::Prm];

    fn label(self) -> &'static str {
        match self {
            Self::Rrt => "RRT",
            Self::RrtStar => "RRT*",
            Self::InformedRrtStar => "Informed RRT*",
            Self::Prm => "PRM",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::Rrt => "rrt",
            Self::RrtStar => "rrtstar",
            Self::InformedRrtStar => "informed",
            Self::Prm => "prm",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| kind.slug() == value)
    }

    fn note(self) -> &'static str {
        match self {
            Self::Rrt => {
                "RRT grows a tree toward random samples and stops at the first path that \
                 reaches the goal: fast, but the path is jagged and far from shortest."
            }
            Self::RrtStar => {
                "RRT* also picks the cheapest parent for every new node and rewires its \
                 neighbors through it, so the path keeps shortening as the tree grows."
            }
            Self::InformedRrtStar => {
                "Informed RRT* is RRT* that, once it has a path, only samples inside the \
                 ellipse of points that could still shorten it (yellow): the same budget \
                 goes much further."
            }
            Self::Prm => {
                "PRM scatters random collision-free samples, links each to its nearest \
                 neighbors into a roadmap, and searches it with Dijkstra. The roadmap does \
                 not depend on the start and goal, so it can answer many queries."
            }
        }
    }
}

/// One planner run, ready to replay.
#[derive(Debug, Clone)]
struct SearchResult {
    kind: SamplingKind,
    /// Tree (or roadmap) edges in the order they were added.
    edges: Vec<([f64; 2], [f64; 2])>,
    path: Vec<[f64; 2]>,
    nodes: usize,
    elapsed_ms: f64,
}

impl SearchResult {
    fn length(&self) -> Option<f64> {
        (self.path.len() >= 2).then(|| {
            self.path
                .windows(2)
                .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                .sum()
        })
    }
}

/// What a drag on the field moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Grab {
    Start,
    Goal,
}

pub struct SamplingDemo {
    kind: SamplingKind,
    /// Circles `(x, y, radius)`.
    obstacles: Vec<(f64, f64, f64)>,
    start: [f64; 2],
    goal: [f64; 2],
    /// Radius of the next obstacle placed \[m\].
    new_radius: f32,
    result: Option<SearchResult>,
    compare: Vec<SearchResult>,
    animate: bool,
    /// Input time the replay started \[s\]; `None` until the scene sees it.
    grow_from: Option<f64>,
    grab: Option<Grab>,
}

fn default_obstacles() -> Vec<(f64, f64, f64)> {
    vec![
        (5.0, 5.0, 1.5),
        (5.0, 11.0, 2.0),
        (9.0, 15.5, 1.6),
        (10.0, 8.0, 2.2),
        (14.5, 4.0, 1.8),
        (15.0, 12.0, 2.0),
        (17.5, 17.0, 1.2),
        (2.5, 16.0, 1.0),
    ]
}

impl Default for SamplingDemo {
    fn default() -> Self {
        let mut demo = Self {
            kind: SamplingKind::RrtStar,
            obstacles: default_obstacles(),
            start: [1.5, 1.5],
            goal: [18.5, 18.5],
            new_radius: 1.4,
            result: None,
            compare: Vec::new(),
            animate: true,
            grow_from: None,
            grab: None,
        };
        demo.replan();
        demo
    }
}

/// Runs one planner from `start` to `goal` among `obstacles`.
fn plan(
    kind: SamplingKind,
    obstacles: &[(f64, f64, f64)],
    start: [f64; 2],
    goal: [f64; 2],
) -> SearchResult {
    let timer = Instant::now();
    let tree_edges = |nodes: Vec<([f64; 2], Option<[f64; 2]>)>| -> Vec<_> {
        nodes
            .into_iter()
            .filter_map(|(node, parent)| parent.map(|parent| (parent, node)))
            .collect()
    };
    let (edges, path, nodes) = match kind {
        SamplingKind::Rrt => {
            let mut planner = RRTPlanner::new(
                obstacles
                    .iter()
                    .map(|&(x, y, r)| CircleObstacle::new(x, y, r))
                    .collect(),
                AreaBounds::new(0.0, WORLD, 0.0, WORLD),
                Some(AreaBounds::new(0.0, WORLD, 0.0, WORLD)),
                RRTConfig {
                    expand_dis: EXPAND_DIS,
                    path_resolution: 0.25,
                    goal_sample_rate: 5,
                    max_iter: 4000,
                    robot_radius: ROBOT_RADIUS,
                },
            );
            let path = planner.planning(start, goal).unwrap_or_default();
            let tree = planner.get_tree();
            let nodes = tree
                .iter()
                .map(|n| {
                    let parent = n.parent.map(|p| [tree[p].x, tree[p].y]);
                    ([n.x, n.y], parent)
                })
                .collect();
            (tree_edges(nodes), path, tree.len())
        }
        SamplingKind::RrtStar => {
            let mut planner = RRTStar::new(
                (start[0], start[1]),
                (goal[0], goal[1]),
                obstacles.to_vec(),
                (0.0, WORLD),
                EXPAND_DIS,
                0.25,
                10,
                STAR_ITERATIONS as i32,
                15.0,
                true,
                ROBOT_RADIUS,
            );
            let path = planner.planning().unwrap_or_default();
            let tree = planner.get_tree();
            let nodes = tree
                .iter()
                .map(|n| {
                    let parent = n.parent.map(|p| [tree[p].x, tree[p].y]);
                    ([n.x, n.y], parent)
                })
                .collect();
            (tree_edges(nodes), path, tree.len())
        }
        SamplingKind::InformedRrtStar => {
            // This planner has no robot radius: inflate the obstacles instead.
            let inflated = obstacles
                .iter()
                .map(|&(x, y, r)| (x, y, r + ROBOT_RADIUS))
                .collect();
            let mut planner = InformedRRTStar::new(
                (start[0], start[1]),
                (goal[0], goal[1]),
                inflated,
                (0.0, WORLD),
                EXPAND_DIS,
                10,
                STAR_ITERATIONS,
            );
            let path = planner.planning().unwrap_or_default();
            let tree = &planner.node_list;
            let nodes = tree
                .iter()
                .map(|n| {
                    let parent = n.parent.map(|p| [tree[p].x, tree[p].y]);
                    ([n.x, n.y], parent)
                })
                .collect();
            (tree_edges(nodes), path, tree.len())
        }
        SamplingKind::Prm => {
            let (ox, oy) = obstacle_points(obstacles);
            let planner = PRMPlanner::new(
                &ox,
                &oy,
                (start[0], start[1]),
                (goal[0], goal[1]),
                ROBOT_RADIUS,
            );
            let path = planner
                .plan()
                .map(|(xs, ys)| xs.into_iter().zip(ys).map(|(x, y)| [x, y]).collect())
                .unwrap_or_default();
            let edges = planner
                .get_edges()
                .into_iter()
                .map(|((ax, ay), (bx, by))| ([ax, ay], [bx, by]))
                .collect();
            (edges, path, planner.get_samples().0.len())
        }
    };
    SearchResult {
        kind,
        edges,
        path,
        nodes,
        elapsed_ms: timer.elapsed().as_secs_f64() * 1000.0,
    }
}

/// PRM works on obstacle points: the world border plus a lattice filling
/// every circle, dense enough that no edge slips between them.
fn obstacle_points(obstacles: &[(f64, f64, f64)]) -> (Vec<f64>, Vec<f64>) {
    let (mut ox, mut oy) = (Vec::new(), Vec::new());
    let border = 0.5;
    let steps = (WORLD / border) as usize;
    for i in 0..=steps {
        let t = i as f64 * border;
        for (x, y) in [(t, 0.0), (t, WORLD), (0.0, t), (WORLD, t)] {
            ox.push(x);
            oy.push(y);
        }
    }
    let spacing = 0.35;
    for &(cx, cy, r) in obstacles {
        let n = (r / spacing).ceil() as i64;
        for i in -n..=n {
            for j in -n..=n {
                let (x, y) = (cx + i as f64 * spacing, cy + j as f64 * spacing);
                if (x - cx).hypot(y - cy) <= r {
                    ox.push(x);
                    oy.push(y);
                }
            }
        }
        // The rim itself, so the drawn circle is what blocks.
        let rim = ((2.0 * std::f64::consts::PI * r) / spacing).ceil() as usize;
        for k in 0..rim {
            let a = k as f64 / rim as f64 * std::f64::consts::TAU;
            ox.push(cx + r * a.cos());
            oy.push(cy + r * a.sin());
        }
    }
    (ox, oy)
}

fn parse_point(value: &str) -> Option<[f64; 2]> {
    let (x, y) = value.split_once(',')?;
    let point = [x.parse::<f64>().ok()?, y.parse::<f64>().ok()?];
    point
        .iter()
        .all(|v| v.is_finite() && (0.0..=WORLD).contains(v))
        .then_some(point)
}

fn to_screen(rect: Rect, p: [f64; 2]) -> Pos2 {
    let scale = rect.width() / WORLD as f32;
    Pos2::new(
        rect.left() + p[0] as f32 * scale,
        rect.bottom() - p[1] as f32 * scale,
    )
}

fn to_world(rect: Rect, pos: Pos2) -> [f64; 2] {
    let scale = rect.width() / WORLD as f32;
    [
        f64::from((pos.x - rect.left()) / scale).clamp(0.0, WORLD),
        f64::from((rect.bottom() - pos.y) / scale).clamp(0.0, WORLD),
    ]
}

impl SamplingDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("sampling") {
            return;
        }
        if let Some(kind) = crate::share::value(query, "planner").and_then(SamplingKind::from_slug)
        {
            self.kind = kind;
        }
        if let Some(start) = crate::share::value(query, "start").and_then(parse_point) {
            self.start = start;
        }
        if let Some(goal) = crate::share::value(query, "goal").and_then(parse_point) {
            self.goal = goal;
        }
        if let Some(value) = crate::share::value(query, "obstacles") {
            self.obstacles = value
                .split(';')
                .filter_map(|circle| {
                    let mut parts = circle.split(',').map(|v| v.parse::<f64>().ok());
                    let (x, y, r) = (parts.next()??, parts.next()??, parts.next()??);
                    ([x, y, r].iter().all(|v| v.is_finite()) && r > 0.0).then_some((
                        x.clamp(0.0, WORLD),
                        y.clamp(0.0, WORLD),
                        r.min(4.0),
                    ))
                })
                .take(MAX_OBSTACLES)
                .collect();
        }
        self.compare.clear();
        self.replan();
    }

    pub fn share_query(&self) -> String {
        let obstacles: Vec<String> = self
            .obstacles
            .iter()
            .map(|(x, y, r)| format!("{x:.1},{y:.1},{r:.1}"))
            .collect();
        format!(
            "tab=sampling&planner={}&start={:.1},{:.1}&goal={:.1},{:.1}&obstacles={}",
            self.kind.slug(),
            self.start[0],
            self.start[1],
            self.goal[0],
            self.goal[1],
            obstacles.join(";")
        )
    }

    fn replan(&mut self) {
        self.result = Some(plan(self.kind, &self.obstacles, self.start, self.goal));
        self.grow_from = None;
    }

    fn compare_all(&mut self) {
        self.compare = SamplingKind::ALL
            .into_iter()
            .map(|kind| plan(kind, &self.obstacles, self.start, self.goal))
            .collect();
        if let Some(current) = self.compare.iter().find(|r| r.kind == self.kind) {
            self.result = Some(current.clone());
            self.grow_from = None;
        }
    }

    /// Whether `p` is free for the start or goal.
    fn is_free(&self, p: [f64; 2]) -> bool {
        self.obstacles
            .iter()
            .all(|&(x, y, r)| (p[0] - x).hypot(p[1] - y) > r + ROBOT_RADIUS)
    }

    fn handle_pointer(&mut self, rect: Rect, response: &egui::Response) {
        let near = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) < GRAB_RADIUS;
        if response.drag_started() {
            let origin = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .map(|pos| to_world(rect, pos));
            self.grab = origin.and_then(|p| {
                if near(p, self.start) {
                    Some(Grab::Start)
                } else if near(p, self.goal) {
                    Some(Grab::Goal)
                } else {
                    None
                }
            });
        }
        if response.dragged() {
            if let (Some(grab), Some(pos)) = (self.grab, response.interact_pointer_pos()) {
                let p = to_world(rect, pos);
                if self.is_free(p) {
                    match grab {
                        Grab::Start => self.start = p,
                        Grab::Goal => self.goal = p,
                    }
                }
            }
        }
        if response.drag_stopped() && self.grab.take().is_some() {
            self.compare.clear();
            self.replan();
        }
        if response.clicked() {
            let Some(p) = response
                .interact_pointer_pos()
                .map(|pos| to_world(rect, pos))
            else {
                return;
            };
            if near(p, self.start) || near(p, self.goal) {
                return;
            }
            let hit = self
                .obstacles
                .iter()
                .position(|&(x, y, r)| (p[0] - x).hypot(p[1] - y) <= r);
            match hit {
                Some(index) => {
                    self.obstacles.remove(index);
                }
                None if self.obstacles.len() < MAX_OBSTACLES => {
                    let r = f64::from(self.new_radius);
                    let blocks = |q: [f64; 2]| (q[0] - p[0]).hypot(q[1] - p[1]) <= r + ROBOT_RADIUS;
                    if blocks(self.start) || blocks(self.goal) {
                        return;
                    }
                    self.obstacles.push((p[0], p[1], r));
                }
                None => return,
            }
            self.compare.clear();
            self.replan();
        }
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Planner");
        ui.horizontal_wrapped(|ui| {
            for kind in SamplingKind::ALL {
                if ui
                    .selectable_label(self.kind == kind, kind.label())
                    .clicked()
                    && self.kind != kind
                {
                    self.kind = kind;
                    match self.compare.iter().find(|r| r.kind == kind) {
                        Some(done) => {
                            self.result = Some(done.clone());
                            self.grow_from = None;
                        }
                        None => self.replan(),
                    }
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .button("New random run")
                .on_hover_text("Plan again with new random samples")
                .clicked()
            {
                self.compare.clear();
                self.replan();
            }
            if ui.button("Compare all four").clicked() {
                self.compare_all();
            }
        });
        ui.checkbox(&mut self.animate, "Animate the search");

        if !self.compare.is_empty() {
            crate::ui_kit::section(ui, "Comparison (one random run each)");
            egui::Grid::new("sampling_compare")
                .striped(true)
                .num_columns(4)
                .show(ui, |ui| {
                    ui.label("");
                    ui.small("length");
                    ui.small("nodes");
                    ui.small("ms");
                    ui.end_row();
                    for run in &self.compare {
                        ui.strong(run.kind.label());
                        ui.label(run.length().map_or("—".to_string(), |l| format!("{l:.1}")));
                        ui.label(run.nodes.to_string());
                        ui.label(format!("{:.0}", run.elapsed_ms));
                        ui.end_row();
                    }
                });
        }

        crate::ui_kit::section(ui, "Obstacles");
        ui.add(
            egui::Slider::new(&mut self.new_radius, 0.4..=3.0)
                .suffix(" m")
                .text("new radius"),
        );
        ui.horizontal_wrapped(|ui| {
            if ui.button("Clear").clicked() {
                self.obstacles.clear();
                self.compare.clear();
                self.replan();
            }
            if ui.button("Reset").clicked() {
                *self = Self::default();
            }
        });
        crate::ui_kit::hint(
            ui,
            "Click the field to add an obstacle, click one to remove it. Drag the green \
             start or the red goal.",
        );
        crate::ui_kit::legend(
            ui,
            &[
                (TREE, "tree / roadmap"),
                (PATH, "path"),
                (START, "start"),
                (GOAL, "goal"),
            ],
        );
        crate::ui_kit::how_it_works(ui, "sampling_help", self.kind.note());
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 34.0);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_pointer(rect, &response);

        let now = ctx.input(|i| i.time);
        let start_time = *self.grow_from.get_or_insert(now);
        let progress = if self.animate && self.grab.is_none() {
            ((now - start_time) / GROW_SECONDS).clamp(0.0, 1.0)
        } else {
            1.0
        };
        if progress < 1.0 {
            ctx.request_repaint();
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));
        let scale = rect.width() / WORLD as f32;
        for &(x, y, r) in &self.obstacles {
            painter.circle_filled(to_screen(rect, [x, y]), r as f32 * scale, OBSTACLE);
        }

        let dragging = self.grab.is_some();
        if let Some(result) = self.result.as_ref().filter(|_| !dragging) {
            let shown = (result.edges.len() as f64 * progress).round() as usize;
            for (a, b) in &result.edges[..shown] {
                painter.line_segment(
                    [to_screen(rect, *a), to_screen(rect, *b)],
                    Stroke::new(1.0_f32, TREE),
                );
            }
            if progress >= 1.0 {
                if let (SamplingKind::InformedRrtStar, Some(best)) = (result.kind, result.length())
                {
                    draw_informed_ellipse(&painter, rect, self.start, self.goal, best);
                }
                if result.path.len() >= 2 {
                    let points = result.path.iter().map(|p| to_screen(rect, *p)).collect();
                    painter.add(Shape::line(points, Stroke::new(3.0_f32, PATH)));
                }
            }
        }
        painter.circle_filled(to_screen(rect, self.start), 7.0, START);
        painter.circle_filled(to_screen(rect, self.goal), 7.0, GOAL);

        let status = match &self.result {
            _ if dragging => "Release to plan".to_string(),
            Some(result) if progress < 1.0 => {
                format!("{} searching…", result.kind.label())
            }
            Some(result) => match result.length() {
                Some(length) => format!(
                    "{}  ·  length {length:.1} m  ·  {} nodes  ·  {:.0} ms",
                    result.kind.label(),
                    result.nodes,
                    result.elapsed_ms
                ),
                None => format!(
                    "{}: no path found ({} nodes)",
                    result.kind.label(),
                    result.nodes
                ),
            },
            None => String::new(),
        };
        ui.label(status);
    }
}

/// The Informed RRT* sampling region: points whose start + goal distance
/// is below the best path length.
fn draw_informed_ellipse(
    painter: &egui::Painter,
    rect: Rect,
    start: [f64; 2],
    goal: [f64; 2],
    best: f64,
) {
    let c_min = (goal[0] - start[0]).hypot(goal[1] - start[1]);
    if best <= c_min {
        return;
    }
    let a = best / 2.0;
    let b = (best * best - c_min * c_min).sqrt() / 2.0;
    let center = [(start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0];
    let angle = (goal[1] - start[1]).atan2(goal[0] - start[0]);
    let points: Vec<Pos2> = (0..=64)
        .map(|k| {
            let t = k as f64 / 64.0 * std::f64::consts::TAU;
            let (x, y) = (a * t.cos(), b * t.sin());
            to_screen(
                rect,
                [
                    center[0] + x * angle.cos() - y * angle.sin(),
                    center[1] + x * angle.sin() + y * angle.cos(),
                ],
            )
        })
        .collect();
    painter.add(Shape::line(points, Stroke::new(1.5_f32, ELLIPSE)));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clear_of_obstacles(result: &SearchResult, obstacles: &[(f64, f64, f64)]) -> bool {
        result.path.windows(2).all(|w| {
            (0..=20).all(|k| {
                let t = k as f64 / 20.0;
                let p = [
                    w[0][0] + t * (w[1][0] - w[0][0]),
                    w[0][1] + t * (w[1][1] - w[0][1]),
                ];
                obstacles
                    .iter()
                    .all(|&(x, y, r)| (p[0] - x).hypot(p[1] - y) > r)
            })
        })
    }

    #[test]
    fn every_planner_finds_a_collision_free_path_on_the_default_field() {
        let demo = SamplingDemo::default();
        for kind in SamplingKind::ALL {
            // Sampling planners are random: allow one unlucky run of three.
            let found = (0..3)
                .filter(|_| {
                    let result = plan(kind, &demo.obstacles, demo.start, demo.goal);
                    result.length().is_some() && clear_of_obstacles(&result, &demo.obstacles)
                })
                .count();
            assert!(found >= 2, "{} found {found}/3 paths", kind.label());
        }
    }

    #[test]
    fn rrt_star_paths_are_shorter_than_rrt_paths() {
        let demo = SamplingDemo::default();
        let mean = |kind| {
            (0..4)
                .filter_map(|_| plan(kind, &demo.obstacles, demo.start, demo.goal).length())
                .sum::<f64>()
                / 4.0
        };
        let (rrt, star) = (mean(SamplingKind::Rrt), mean(SamplingKind::RrtStar));
        assert!(star < rrt, "RRT* {star:.1} m vs RRT {rrt:.1} m");
    }

    #[test]
    fn share_query_round_trips() {
        let mut demo = SamplingDemo {
            kind: SamplingKind::Prm,
            start: [2.0, 3.0],
            goal: [17.0, 16.5],
            obstacles: vec![(10.0, 10.0, 2.5)],
            ..SamplingDemo::default()
        };
        demo.replan();
        let query = demo.share_query();
        let mut restored = SamplingDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.kind, SamplingKind::Prm);
        assert_eq!(restored.start, [2.0, 3.0]);
        assert_eq!(restored.goal, [17.0, 16.5]);
        assert_eq!(restored.obstacles, vec![(10.0, 10.0, 2.5)]);

        // Other tabs' links and junk leave the field alone.
        let mut untouched = SamplingDemo::default();
        untouched.apply_share_query("tab=grid&obstacles=1,1,1");
        assert_eq!(untouched.obstacles, default_obstacles());
        untouched.apply_share_query("tab=sampling&start=99,1&obstacles=a,b,c;5,5,-1");
        assert_eq!(untouched.start, [1.5, 1.5]);
        assert!(untouched.obstacles.is_empty());
    }
}
