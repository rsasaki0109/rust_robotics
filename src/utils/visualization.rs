//! Visualization utilities for rust_robotics
//!
//! Provides a unified interface for plotting using gnuplot.

use gnuplot::{Figure, Caption, Color, PointSymbol, PointSize, LineWidth, AxesCommon, AutoOption};
use crate::common::{Point2D, Pose2D, Path2D, Obstacles};

/// Color palette for consistent styling
pub mod colors {
    pub const BLACK: &str = "#000000";
    pub const RED: &str = "#FF0000";
    pub const GREEN: &str = "#00FF00";
    pub const BLUE: &str = "#0000FF";
    pub const YELLOW: &str = "#FFFF00";
    pub const CYAN: &str = "#00FFFF";
    pub const MAGENTA: &str = "#FF00FF";
    pub const ORANGE: &str = "#FFA500";
    pub const PURPLE: &str = "#800080";
    pub const GRAY: &str = "#808080";

    // Semantic colors
    pub const OBSTACLE: &str = BLACK;
    pub const START: &str = GREEN;
    pub const GOAL: &str = BLUE;
    pub const PATH: &str = RED;
    pub const ROBOT: &str = CYAN;
    pub const ESTIMATED: &str = "#35C788";
    pub const GROUND_TRUTH: &str = BLUE;
    pub const MEASUREMENT: &str = "#DD3355";
    pub const DEAD_RECKONING: &str = YELLOW;
}

/// Style for path rendering
#[derive(Debug, Clone)]
pub struct PathStyle {
    pub color: String,
    pub line_width: f64,
    pub caption: String,
}

impl PathStyle {
    pub fn new(color: &str, caption: &str) -> Self {
        Self {
            color: color.to_string(),
            line_width: 2.0,
            caption: caption.to_string(),
        }
    }

    pub fn with_line_width(mut self, width: f64) -> Self {
        self.line_width = width;
        self
    }
}

impl Default for PathStyle {
    fn default() -> Self {
        Self {
            color: colors::PATH.to_string(),
            line_width: 2.0,
            caption: "Path".to_string(),
        }
    }
}

/// Style for point rendering
#[derive(Debug, Clone)]
pub struct PointStyle {
    pub color: String,
    pub size: f64,
    pub symbol: char,
    pub caption: String,
}

impl PointStyle {
    pub fn new(color: &str, caption: &str) -> Self {
        Self {
            color: color.to_string(),
            size: 1.0,
            symbol: 'O',
            caption: caption.to_string(),
        }
    }

    pub fn with_size(mut self, size: f64) -> Self {
        self.size = size;
        self
    }

    pub fn with_symbol(mut self, symbol: char) -> Self {
        self.symbol = symbol;
        self
    }
}

/// Main visualizer struct
pub struct Visualizer {
    figure: Figure,
    title: String,
    x_label: String,
    y_label: String,
    x_range: Option<(f64, f64)>,
    y_range: Option<(f64, f64)>,
    aspect_ratio: Option<f64>,
}

impl Visualizer {
    /// Create a new visualizer
    pub fn new() -> Self {
        Self {
            figure: Figure::new(),
            title: String::new(),
            x_label: "X [m]".to_string(),
            y_label: "Y [m]".to_string(),
            x_range: None,
            y_range: None,
            aspect_ratio: Some(1.0),
        }
    }

    /// Set the plot title
    pub fn set_title(&mut self, title: &str) -> &mut Self {
        self.title = title.to_string();
        self
    }

    /// Set X axis label
    pub fn set_x_label(&mut self, label: &str) -> &mut Self {
        self.x_label = label.to_string();
        self
    }

    /// Set Y axis label
    pub fn set_y_label(&mut self, label: &str) -> &mut Self {
        self.y_label = label.to_string();
        self
    }

    /// Set X axis range
    pub fn set_x_range(&mut self, min: f64, max: f64) -> &mut Self {
        self.x_range = Some((min, max));
        self
    }

    /// Set Y axis range
    pub fn set_y_range(&mut self, min: f64, max: f64) -> &mut Self {
        self.y_range = Some((min, max));
        self
    }

    /// Set aspect ratio (None for auto)
    pub fn set_aspect_ratio(&mut self, ratio: Option<f64>) -> &mut Self {
        self.aspect_ratio = ratio;
        self
    }

    /// Get mutable reference to the internal figure for advanced usage
    pub fn figure_mut(&mut self) -> &mut Figure {
        &mut self.figure
    }

    /// Plot a path
    pub fn plot_path(&mut self, path: &Path2D, style: &PathStyle) -> &mut Self {
        let x: Vec<f64> = path.points.iter().map(|p| p.x).collect();
        let y: Vec<f64> = path.points.iter().map(|p| p.y).collect();

        self.figure.axes2d()
            .lines(&x, &y, &[
                Caption(&style.caption),
                Color(&style.color),
                LineWidth(style.line_width),
            ]);
        self
    }

    /// Plot a path from x,y vectors
    pub fn plot_path_xy(&mut self, x: &[f64], y: &[f64], style: &PathStyle) -> &mut Self {
        self.figure.axes2d()
            .lines(x, y, &[
                Caption(&style.caption),
                Color(&style.color),
                LineWidth(style.line_width),
            ]);
        self
    }

    /// Plot obstacles
    pub fn plot_obstacles(&mut self, obstacles: &Obstacles) -> &mut Self {
        let x: Vec<f64> = obstacles.points.iter().map(|p| p.x).collect();
        let y: Vec<f64> = obstacles.points.iter().map(|p| p.y).collect();

        self.figure.axes2d()
            .points(&x, &y, &[
                Caption("Obstacles"),
                Color(colors::OBSTACLE),
                PointSymbol('S'),
                PointSize(0.5),
            ]);
        self
    }

    /// Plot obstacles from x,y vectors
    pub fn plot_obstacles_xy(&mut self, ox: &[f64], oy: &[f64]) -> &mut Self {
        self.figure.axes2d()
            .points(ox, oy, &[
                Caption("Obstacles"),
                Color(colors::OBSTACLE),
                PointSymbol('S'),
                PointSize(0.5),
            ]);
        self
    }

    /// Plot a single point (start, goal, etc.)
    pub fn plot_point(&mut self, point: Point2D, style: &PointStyle) -> &mut Self {
        self.figure.axes2d()
            .points(&[point.x], &[point.y], &[
                Caption(&style.caption),
                Color(&style.color),
                PointSymbol(style.symbol),
                PointSize(style.size),
            ]);
        self
    }

    /// Plot multiple points
    pub fn plot_points(&mut self, points: &[Point2D], style: &PointStyle) -> &mut Self {
        let x: Vec<f64> = points.iter().map(|p| p.x).collect();
        let y: Vec<f64> = points.iter().map(|p| p.y).collect();

        self.figure.axes2d()
            .points(&x, &y, &[
                Caption(&style.caption),
                Color(&style.color),
                PointSymbol(style.symbol),
                PointSize(style.size),
            ]);
        self
    }

    /// Plot points from x,y vectors
    pub fn plot_points_xy(&mut self, x: &[f64], y: &[f64], style: &PointStyle) -> &mut Self {
        self.figure.axes2d()
            .points(x, y, &[
                Caption(&style.caption),
                Color(&style.color),
                PointSymbol(style.symbol),
                PointSize(style.size),
            ]);
        self
    }

    /// Plot robot pose with direction indicator
    pub fn plot_robot(&mut self, pose: &Pose2D, size: f64) -> &mut Self {
        // Plot robot position
        self.figure.axes2d()
            .points(&[pose.x], &[pose.y], &[
                Caption("Robot"),
                Color(colors::ROBOT),
                PointSymbol('O'),
                PointSize(size),
            ]);

        // Plot direction line (arrow substitute)
        let arrow_len = size * 0.5;
        let end_x = pose.x + arrow_len * pose.yaw.cos();
        let end_y = pose.y + arrow_len * pose.yaw.sin();

        self.figure.axes2d()
            .lines(&[pose.x, end_x], &[pose.y, end_y], &[
                Color(colors::ROBOT),
                LineWidth(2.0),
            ]);
        self
    }

    /// Plot start position
    pub fn plot_start(&mut self, point: Point2D) -> &mut Self {
        self.plot_point(point, &PointStyle::new(colors::START, "Start").with_size(1.5))
    }

    /// Plot goal position
    pub fn plot_goal(&mut self, point: Point2D) -> &mut Self {
        self.plot_point(point, &PointStyle::new(colors::GOAL, "Goal").with_size(1.5))
    }

    /// Finalize and show the plot
    pub fn show(&mut self) -> Result<(), String> {
        self.apply_settings();
        self.figure.show().map_err(|e| e.to_string()).map(|_| ())
    }

    /// Save plot to PNG file
    pub fn save_png(&mut self, path: &str, width: u32, height: u32) -> Result<(), String> {
        self.apply_settings();
        self.figure.save_to_png(path, width, height).map_err(|e| e.to_string())
    }

    /// Save plot to SVG file
    pub fn save_svg(&mut self, path: &str) -> Result<(), String> {
        self.apply_settings();
        self.figure.save_to_svg(path, 800, 600).map_err(|e| e.to_string())
    }

    fn apply_settings(&mut self) {
        let axes = self.figure.axes2d();

        if !self.title.is_empty() {
            axes.set_title(&self.title, &[]);
        }
        axes.set_x_label(&self.x_label, &[]);
        axes.set_y_label(&self.y_label, &[]);

        if let Some((min, max)) = self.x_range {
            axes.set_x_range(AutoOption::Fix(min), AutoOption::Fix(max));
        }
        if let Some((min, max)) = self.y_range {
            axes.set_y_range(AutoOption::Fix(min), AutoOption::Fix(max));
        }
        if let Some(ratio) = self.aspect_ratio {
            axes.set_aspect_ratio(AutoOption::Fix(ratio));
        }
    }
}

impl Default for Visualizer {
    fn default() -> Self {
        Self::new()
    }
}

/// Quick plot function for simple path visualization
pub fn quick_plot_path(
    path: &Path2D,
    obstacles: Option<&Obstacles>,
    start: Option<Point2D>,
    goal: Option<Point2D>,
    title: &str,
) -> Visualizer {
    let mut vis = Visualizer::new();
    vis.set_title(title);

    if let Some(obs) = obstacles {
        vis.plot_obstacles(obs);
    }
    if let Some(s) = start {
        vis.plot_start(s);
    }
    if let Some(g) = goal {
        vis.plot_goal(g);
    }
    vis.plot_path(path, &PathStyle::default());

    vis
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_visualizer_creation() {
        let vis = Visualizer::new();
        assert!(vis.aspect_ratio.is_some());
    }

    #[test]
    fn test_path_style() {
        let style = PathStyle::new(colors::RED, "Test Path")
            .with_line_width(3.0);
        assert_eq!(style.line_width, 3.0);
        assert_eq!(style.color, colors::RED);
    }
}
