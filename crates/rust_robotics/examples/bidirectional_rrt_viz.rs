//! Bidirectional RRT visualization example.
//!
//! Plots start tree (blue), goal tree (red), circular obstacles, and the
//! final path (green). Output:
//! `img/path_planning/bidirectional_rrt.png`.

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSymbol};
use rust_robotics::planning::bidirectional_rrt::{
    AreaBounds, BidirectionalRRTConfig, CircleObstacle,
};
use rust_robotics::prelude::Point2D;

#[derive(Clone, Copy)]
struct Node {
    x: f64,
    y: f64,
    parent: Option<usize>,
}

#[derive(Clone, Copy)]
struct Edge {
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
}

fn halton(mut index: usize, base: usize) -> f64 {
    let mut f = 1.0;
    let mut r = 0.0;
    while index > 0 {
        f /= base as f64;
        r += f * (index % base) as f64;
        index /= base;
    }
    r
}

fn sample_point(iter: usize, area: &AreaBounds) -> Point2D {
    let hx = halton(iter + 1, 2);
    let hy = halton(iter + 1, 3);
    Point2D::new(
        area.xmin + hx * (area.xmax - area.xmin),
        area.ymin + hy * (area.ymax - area.ymin),
    )
}

fn distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = ax - bx;
    let dy = ay - by;
    (dx * dx + dy * dy).sqrt()
}

fn nearest_index(tree: &[Node], target: Point2D) -> usize {
    tree.iter()
        .enumerate()
        .map(|(i, node)| (i, distance(node.x, node.y, target.x, target.y)))
        .min_by(|a, b| a.1.partial_cmp(&b.1).expect("distance should be finite"))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

fn point_in_collision(x: f64, y: f64, obstacles: &[CircleObstacle], robot_radius: f64) -> bool {
    obstacles
        .iter()
        .any(|obs| distance(x, y, obs.x, obs.y) <= obs.radius + robot_radius)
}

fn collision_free_segment(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    config: &BidirectionalRRTConfig,
    obstacles: &[CircleObstacle],
) -> bool {
    let d = distance(ax, ay, bx, by);
    let n = (d / config.path_resolution).ceil() as usize;
    for i in 0..=n {
        let t = if n == 0 { 0.0 } else { i as f64 / n as f64 };
        let x = ax + t * (bx - ax);
        let y = ay + t * (by - ay);
        if point_in_collision(x, y, obstacles, config.robot_radius) {
            return false;
        }
    }
    true
}

fn steer(
    from: Node,
    to: Point2D,
    parent_idx: usize,
    config: &BidirectionalRRTConfig,
    obstacles: &[CircleObstacle],
) -> Option<Node> {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let d = (dx * dx + dy * dy).sqrt();
    let theta = dy.atan2(dx);
    let step = d.min(config.expand_dis);
    let n_steps = (step / config.path_resolution).floor() as usize;

    let mut cx = from.x;
    let mut cy = from.y;
    for _ in 0..n_steps {
        cx += config.path_resolution * theta.cos();
        cy += config.path_resolution * theta.sin();
        if point_in_collision(cx, cy, obstacles, config.robot_radius) {
            return None;
        }
    }

    if distance(cx, cy, to.x, to.y) <= config.path_resolution {
        cx = to.x;
        cy = to.y;
    }

    if point_in_collision(cx, cy, obstacles, config.robot_radius) {
        return None;
    }

    Some(Node {
        x: cx,
        y: cy,
        parent: Some(parent_idx),
    })
}

fn trace_path(tree: &[Node], mut idx: usize) -> Vec<Point2D> {
    let mut points = Vec::new();
    loop {
        let n = tree[idx];
        points.push(Point2D::new(n.x, n.y));
        match n.parent {
            Some(parent) => idx = parent,
            None => break,
        }
    }
    points.reverse();
    points
}

fn grow_bidirectional_trees(
    start: Point2D,
    goal: Point2D,
    area: &AreaBounds,
    obstacles: &[CircleObstacle],
    config: &BidirectionalRRTConfig,
) -> (Vec<Edge>, Vec<Edge>, Vec<Point2D>) {
    let mut start_tree = vec![Node {
        x: start.x,
        y: start.y,
        parent: None,
    }];
    let mut goal_tree = vec![Node {
        x: goal.x,
        y: goal.y,
        parent: None,
    }];

    let mut start_edges = Vec::new();
    let mut goal_edges = Vec::new();
    let mut grow_from_start = true;

    for iter in 0..config.max_iter {
        let sample = sample_point(iter, area);

        if grow_from_start {
            let nearest_start = nearest_index(&start_tree, sample);
            if let Some(new_node) = steer(
                start_tree[nearest_start],
                sample,
                nearest_start,
                config,
                obstacles,
            ) {
                start_tree.push(new_node);
                let new_idx = start_tree.len() - 1;
                let parent = start_tree[nearest_start];
                start_edges.push(Edge {
                    x0: parent.x,
                    y0: parent.y,
                    x1: new_node.x,
                    y1: new_node.y,
                });

                let nearest_goal = nearest_index(&goal_tree, Point2D::new(new_node.x, new_node.y));
                let goal_node = goal_tree[nearest_goal];
                if distance(new_node.x, new_node.y, goal_node.x, goal_node.y) <= config.expand_dis
                    && collision_free_segment(
                        new_node.x,
                        new_node.y,
                        goal_node.x,
                        goal_node.y,
                        config,
                        obstacles,
                    )
                {
                    let mut path = trace_path(&start_tree, new_idx);
                    let mut goal_branch = trace_path(&goal_tree, nearest_goal);
                    goal_branch.reverse();
                    path.extend(goal_branch);
                    return (start_edges, goal_edges, path);
                }
            }
        } else {
            let nearest_goal = nearest_index(&goal_tree, sample);
            if let Some(new_node) = steer(
                goal_tree[nearest_goal],
                sample,
                nearest_goal,
                config,
                obstacles,
            ) {
                goal_tree.push(new_node);
                let new_idx = goal_tree.len() - 1;
                let parent = goal_tree[nearest_goal];
                goal_edges.push(Edge {
                    x0: parent.x,
                    y0: parent.y,
                    x1: new_node.x,
                    y1: new_node.y,
                });

                let nearest_start =
                    nearest_index(&start_tree, Point2D::new(new_node.x, new_node.y));
                let start_node = start_tree[nearest_start];
                if distance(new_node.x, new_node.y, start_node.x, start_node.y) <= config.expand_dis
                    && collision_free_segment(
                        new_node.x,
                        new_node.y,
                        start_node.x,
                        start_node.y,
                        config,
                        obstacles,
                    )
                {
                    let mut path = trace_path(&start_tree, nearest_start);
                    let mut goal_branch = trace_path(&goal_tree, new_idx);
                    goal_branch.reverse();
                    path.extend(goal_branch);
                    return (start_edges, goal_edges, path);
                }
            }
        }

        grow_from_start = !grow_from_start;
    }

    (start_edges, goal_edges, Vec::new())
}

fn obstacle_outline(obs: &CircleObstacle, samples: usize) -> (Vec<f64>, Vec<f64>) {
    let mut xs = Vec::with_capacity(samples + 1);
    let mut ys = Vec::with_capacity(samples + 1);
    for i in 0..=samples {
        let theta = 2.0 * std::f64::consts::PI * i as f64 / samples as f64;
        xs.push(obs.x + obs.radius * theta.cos());
        ys.push(obs.y + obs.radius * theta.sin());
    }
    (xs, ys)
}

fn main() {
    std::fs::create_dir_all("img/path_planning")
        .expect("failed to create img/path_planning directory");

    let start = Point2D::new(0.0, 0.0);
    let goal = Point2D::new(14.0, 14.0);
    let area = AreaBounds::new(-2.0, 16.0, -2.0, 16.0);
    let obstacles = vec![
        CircleObstacle::new(5.0, 5.0, 1.5),
        CircleObstacle::new(8.0, 8.0, 1.5),
        CircleObstacle::new(10.0, 4.0, 1.2),
        CircleObstacle::new(4.0, 11.0, 1.3),
    ];
    let config = BidirectionalRRTConfig {
        max_iter: 1500,
        ..BidirectionalRRTConfig::default()
    };

    let (start_edges, goal_edges, path) =
        grow_bidirectional_trees(start, goal, &area, &obstacles, &config);
    if path.is_empty() {
        eprintln!("No path found for visualization; plotting tree growth only.");
    }

    let mut fg = Figure::new();
    {
        let axes = fg
            .axes2d()
            .set_title("Bidirectional RRT Tree Growth", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
            .set_x_range(gnuplot::Fix(area.xmin), gnuplot::Fix(area.xmax))
            .set_y_range(gnuplot::Fix(area.ymin), gnuplot::Fix(area.ymax));

        for (i, edge) in start_edges.iter().enumerate() {
            let style = if i == 0 {
                vec![Caption("Start Tree"), Color("blue")]
            } else {
                vec![Color("blue")]
            };
            axes.lines([edge.x0, edge.x1], [edge.y0, edge.y1], &style);
        }
        for (i, edge) in goal_edges.iter().enumerate() {
            let style = if i == 0 {
                vec![Caption("Goal Tree"), Color("red")]
            } else {
                vec![Color("red")]
            };
            axes.lines([edge.x0, edge.x1], [edge.y0, edge.y1], &style);
        }

        for (i, obs) in obstacles.iter().enumerate() {
            let (ox, oy) = obstacle_outline(obs, 48);
            if i == 0 {
                axes.lines(&ox, &oy, &[Caption("Obstacles"), Color("black")]);
            } else {
                axes.lines(&ox, &oy, &[Color("black")]);
            }
        }

        if !path.is_empty() {
            let px: Vec<f64> = path.iter().map(|p| p.x).collect();
            let py: Vec<f64> = path.iter().map(|p| p.y).collect();
            axes.lines(&px, &py, &[Caption("Final Path"), Color("green")]);
        }

        axes.points(
            [start.x],
            [start.y],
            &[Caption("Start"), Color("blue"), PointSymbol('O')],
        );
        axes.points(
            [goal.x],
            [goal.y],
            &[Caption("Goal"), Color("red"), PointSymbol('X')],
        );
    }

    let output_path = "img/path_planning/bidirectional_rrt.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show()
        .expect("failed to render gnuplot figure for bidirectional_rrt_viz");
    println!("Saved {}", output_path);
}
