//! Dependency-light animated GIF recording for algorithm demos.
//!
//! Renders frames with a small built-in software rasterizer (no gnuplot, no
//! system packages) and encodes them with the pure-Rust `gif` crate, so the
//! gallery assets can be regenerated in CI with a single `cargo run`.
//!
//! ```no_run
//! use rust_robotics_viz::{GifCanvasConfig, GifFrame, GifRecorder, colors, rgb};
//!
//! let cfg = GifCanvasConfig::new(480, 480, (-5.0, 55.0), (-5.0, 55.0));
//! let mut rec = GifRecorder::new("demo.gif", cfg.clone()).unwrap();
//! for step in 0..30 {
//!     let mut frame = GifFrame::new(&cfg);
//!     frame.draw_point(step as f64, step as f64, rgb(colors::ROBOT), 4.0);
//!     rec.add_frame(frame).unwrap();
//! }
//! rec.finish().unwrap();
//! ```

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use gif::{Encoder, Frame, Repeat};
use rust_robotics_core::{Path2D, Pose2D};

/// RGB color used by the rasterizer.
pub type Rgb = (u8, u8, u8);

/// Parse a `#RRGGBB` hex string (as used in [`crate::colors`]) into [`Rgb`].
///
/// Falls back to black for malformed input so demo code stays panic-free.
pub fn rgb(hex: &str) -> Rgb {
    let h = hex.trim_start_matches('#');
    if h.len() != 6 {
        return (0, 0, 0);
    }
    let parse = |s: &str| u8::from_str_radix(s, 16).unwrap_or(0);
    (parse(&h[0..2]), parse(&h[2..4]), parse(&h[4..6]))
}

const GRID: Rgb = (225, 225, 225);
const BORDER: Rgb = (90, 90, 90);

/// Canvas geometry and timing shared by every frame of one recording.
#[derive(Debug, Clone)]
pub struct GifCanvasConfig {
    /// Output GIF width in pixels.
    pub width: u32,
    /// Output GIF height in pixels.
    pub height: u32,
    /// World-coordinate range mapped to the horizontal axis.
    pub x_range: (f64, f64),
    /// World-coordinate range mapped to the vertical axis.
    pub y_range: (f64, f64),
    /// Frame delay in hundredths of a second.
    pub delay_cs: u16,
    /// Blank margin around the plot area in output pixels.
    pub margin_px: u32,
    /// Spacing of light grid lines in world units (`None` disables the grid).
    pub grid_step: Option<f64>,
    /// Supersampling factor used for antialiasing (rendered at `factor` x
    /// resolution, then box-downsampled).
    pub supersample: u32,
}

impl GifCanvasConfig {
    /// Create a config with sensible defaults (10 fps, light grid, 2x AA).
    ///
    /// The world ranges are expanded so one world unit measures the same
    /// number of pixels on both axes (equal aspect), matching the behavior
    /// of the gnuplot-backed [`crate::Visualizer`].
    pub fn new(width: u32, height: u32, x_range: (f64, f64), y_range: (f64, f64)) -> Self {
        let mut cfg = Self {
            width,
            height,
            x_range,
            y_range,
            delay_cs: 10,
            margin_px: 16,
            grid_step: Some(10.0),
            supersample: 2,
        };
        cfg.equalize_aspect();
        cfg
    }

    /// Set the frame delay in hundredths of a second.
    pub fn with_delay_cs(mut self, delay_cs: u16) -> Self {
        self.delay_cs = delay_cs;
        self
    }

    /// Set the grid spacing in world units (`None` disables the grid).
    pub fn with_grid_step(mut self, step: Option<f64>) -> Self {
        self.grid_step = step;
        self
    }

    fn plot_size_px(&self) -> (f64, f64) {
        (
            (self.width - 2 * self.margin_px) as f64,
            (self.height - 2 * self.margin_px) as f64,
        )
    }

    /// Expand the shorter world range so both axes share one scale.
    fn equalize_aspect(&mut self) {
        let (pw, ph) = self.plot_size_px();
        let (xmin, xmax) = self.x_range;
        let (ymin, ymax) = self.y_range;
        let sx = pw / (xmax - xmin);
        let sy = ph / (ymax - ymin);
        if sx < sy {
            // X is the limiting scale; widen Y around its center.
            let half = ph / sx / 2.0;
            let cy = (ymin + ymax) / 2.0;
            self.y_range = (cy - half, cy + half);
        } else {
            let half = pw / sy / 2.0;
            let cx = (xmin + xmax) / 2.0;
            self.x_range = (cx - half, cx + half);
        }
    }
}

/// One frame of an animated GIF, backed by a supersampled RGB buffer.
pub struct GifFrame {
    buf: Vec<u8>,
    w: usize,
    h: usize,
    ss: f64,
    margin: f64,
    x_range: (f64, f64),
    y_range: (f64, f64),
}

impl GifFrame {
    /// Create a blank frame (white background, grid, and border).
    pub fn new(cfg: &GifCanvasConfig) -> Self {
        let ss = cfg.supersample.max(1) as usize;
        let w = cfg.width as usize * ss;
        let h = cfg.height as usize * ss;
        let mut frame = Self {
            buf: vec![255; w * h * 3],
            w,
            h,
            ss: ss as f64,
            margin: (cfg.margin_px as usize * ss) as f64,
            x_range: cfg.x_range,
            y_range: cfg.y_range,
        };
        frame.draw_grid_and_border(cfg);
        frame
    }

    fn draw_grid_and_border(&mut self, cfg: &GifCanvasConfig) {
        if let Some(step) = cfg.grid_step {
            if step > 0.0 {
                let (xmin, xmax) = self.x_range;
                let (ymin, ymax) = self.y_range;
                let mut gx = (xmin / step).ceil() * step;
                while gx <= xmax {
                    self.line_px(self.px(gx, ymin), self.px(gx, ymax), GRID, self.ss);
                    gx += step;
                }
                let mut gy = (ymin / step).ceil() * step;
                while gy <= ymax {
                    self.line_px(self.px(xmin, gy), self.px(xmax, gy), GRID, self.ss);
                    gy += step;
                }
            }
        }
        let (xmin, xmax) = self.x_range;
        let (ymin, ymax) = self.y_range;
        let corners = [
            self.px(xmin, ymin),
            self.px(xmax, ymin),
            self.px(xmax, ymax),
            self.px(xmin, ymax),
        ];
        for i in 0..4 {
            self.line_px(corners[i], corners[(i + 1) % 4], BORDER, self.ss);
        }
    }

    /// Map world coordinates to (supersampled) pixel coordinates.
    fn px(&self, x: f64, y: f64) -> (f64, f64) {
        let (xmin, xmax) = self.x_range;
        let (ymin, ymax) = self.y_range;
        let pw = self.w as f64 - 2.0 * self.margin;
        let ph = self.h as f64 - 2.0 * self.margin;
        let u = self.margin + (x - xmin) / (xmax - xmin) * pw;
        let v = self.margin + (ymax - y) / (ymax - ymin) * ph;
        (u, v)
    }

    fn put(&mut self, x: i64, y: i64, c: Rgb) {
        if x < 0 || y < 0 || x >= self.w as i64 || y >= self.h as i64 {
            return;
        }
        let i = (y as usize * self.w + x as usize) * 3;
        self.buf[i] = c.0;
        self.buf[i + 1] = c.1;
        self.buf[i + 2] = c.2;
    }

    fn disc_px(&mut self, cx: f64, cy: f64, r: f64, c: Rgb) {
        let r = r.max(0.5);
        let (x0, x1) = ((cx - r).floor() as i64, (cx + r).ceil() as i64);
        let (y0, y1) = ((cy - r).floor() as i64, (cy + r).ceil() as i64);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let (dx, dy) = (x as f64 - cx, y as f64 - cy);
                if dx * dx + dy * dy <= r * r {
                    self.put(x, y, c);
                }
            }
        }
    }

    /// Stamp discs along a segment to draw a line of the given thickness.
    fn line_px(&mut self, a: (f64, f64), b: (f64, f64), c: Rgb, width: f64) {
        let r = (width / 2.0).max(0.5);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len = (dx * dx + dy * dy).sqrt();
        let steps = (len / (r * 0.5)).ceil().max(1.0) as usize;
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            self.disc_px(a.0 + dx * t, a.1 + dy * t, r, c);
        }
    }

    /// Draw a polyline through world-coordinate points.
    pub fn draw_path_xy(&mut self, xs: &[f64], ys: &[f64], color: Rgb, width: f64) {
        let w = width * self.ss;
        for i in 1..xs.len().min(ys.len()) {
            let a = self.px(xs[i - 1], ys[i - 1]);
            let b = self.px(xs[i], ys[i]);
            self.line_px(a, b, color, w);
        }
    }

    /// Draw a [`Path2D`] as a polyline.
    pub fn draw_path(&mut self, path: &Path2D, color: Rgb, width: f64) {
        let xs: Vec<f64> = path.points.iter().map(|p| p.x).collect();
        let ys: Vec<f64> = path.points.iter().map(|p| p.y).collect();
        self.draw_path_xy(&xs, &ys, color, width);
    }

    /// Draw a single line segment between world-coordinate endpoints.
    pub fn draw_segment(&mut self, a: (f64, f64), b: (f64, f64), color: Rgb, width: f64) {
        let pa = self.px(a.0, a.1);
        let pb = self.px(b.0, b.1);
        self.line_px(pa, pb, color, width * self.ss);
    }

    /// Draw a filled dot at a world-coordinate point (`radius` in output px).
    pub fn draw_point(&mut self, x: f64, y: f64, color: Rgb, radius: f64) {
        let (u, v) = self.px(x, y);
        self.disc_px(u, v, radius * self.ss, color);
    }

    /// Draw filled dots at world-coordinate points (`radius` in output px).
    pub fn draw_points_xy(&mut self, xs: &[f64], ys: &[f64], color: Rgb, radius: f64) {
        for i in 0..xs.len().min(ys.len()) {
            self.draw_point(xs[i], ys[i], color, radius);
        }
    }

    /// Draw an `x` marker at a world-coordinate point (`size` in output px).
    pub fn draw_cross(&mut self, x: f64, y: f64, color: Rgb, size: f64) {
        let (u, v) = self.px(x, y);
        let s = size * self.ss;
        let w = 1.2 * self.ss;
        self.line_px((u - s, v - s), (u + s, v + s), color, w);
        self.line_px((u - s, v + s), (u + s, v - s), color, w);
    }

    /// Draw a circle outline (`radius` in world units, `width` in output px).
    pub fn draw_circle(&mut self, cx: f64, cy: f64, radius: f64, color: Rgb, width: f64) {
        self.draw_ellipse(cx, cy, radius, radius, 0.0, color, width);
    }

    /// Draw a rotated ellipse outline, e.g. a covariance ellipse.
    ///
    /// `a` / `b` are the semi-axes in world units, `angle` is the rotation of
    /// the `a` axis in radians, `width` is the stroke width in output px.
    #[allow(clippy::too_many_arguments)]
    pub fn draw_ellipse(
        &mut self,
        cx: f64,
        cy: f64,
        a: f64,
        b: f64,
        angle: f64,
        color: Rgb,
        width: f64,
    ) {
        const SEGMENTS: usize = 72;
        let (sin, cos) = angle.sin_cos();
        let mut prev: Option<(f64, f64)> = None;
        for i in 0..=SEGMENTS {
            let t = i as f64 / SEGMENTS as f64 * std::f64::consts::TAU;
            let (ex, ey) = (a * t.cos(), b * t.sin());
            let p = self.px(cx + ex * cos - ey * sin, cy + ex * sin + ey * cos);
            if let Some(q) = prev {
                self.line_px(q, p, color, width * self.ss);
            }
            prev = Some(p);
        }
    }

    /// Fill a circle (`radius` in world units), e.g. a circular obstacle.
    pub fn fill_circle(&mut self, cx: f64, cy: f64, radius: f64, color: Rgb) {
        let (u, v) = self.px(cx, cy);
        let (u2, _) = self.px(cx + radius, cy);
        self.disc_px(u, v, (u2 - u).abs(), color);
    }

    /// Fill an axis-aligned world-coordinate rectangle (e.g. a grid cell).
    pub fn fill_rect(&mut self, x0: f64, y0: f64, x1: f64, y1: f64, color: Rgb) {
        let (u0, v1) = self.px(x0.min(x1), y0.min(y1));
        let (u1, v0) = self.px(x0.max(x1), y0.max(y1));
        for y in v0.round() as i64..=v1.round() as i64 {
            for x in u0.round() as i64..=u1.round() as i64 {
                self.put(x, y, color);
            }
        }
    }

    /// Draw a robot as a circle with a heading tick (`radius` in world units).
    pub fn draw_robot(&mut self, pose: &Pose2D, radius: f64, color: Rgb) {
        self.draw_circle(pose.x, pose.y, radius, color, 2.0);
        let tip = (
            pose.x + 1.6 * radius * pose.yaw.cos(),
            pose.y + 1.6 * radius * pose.yaw.sin(),
        );
        self.draw_segment((pose.x, pose.y), tip, color, 2.0);
    }

    /// Box-downsample the supersampled buffer to the output resolution.
    fn downsample(&self, ss: usize) -> Vec<u8> {
        if ss <= 1 {
            return self.buf.clone();
        }
        let (ow, oh) = (self.w / ss, self.h / ss);
        let mut out = vec![0u8; ow * oh * 3];
        let n = (ss * ss) as u32;
        for oy in 0..oh {
            for ox in 0..ow {
                let mut acc = [0u32; 3];
                for sy in 0..ss {
                    let row = ((oy * ss + sy) * self.w + ox * ss) * 3;
                    for sx in 0..ss {
                        let i = row + sx * 3;
                        acc[0] += self.buf[i] as u32;
                        acc[1] += self.buf[i + 1] as u32;
                        acc[2] += self.buf[i + 2] as u32;
                    }
                }
                let o = (oy * ow + ox) * 3;
                out[o] = (acc[0] / n) as u8;
                out[o + 1] = (acc[1] / n) as u8;
                out[o + 2] = (acc[2] / n) as u8;
            }
        }
        out
    }
}

/// Streams [`GifFrame`]s into a looping animated GIF file.
pub struct GifRecorder {
    encoder: Encoder<BufWriter<File>>,
    cfg: GifCanvasConfig,
}

impl GifRecorder {
    /// Create a recorder writing to `path`. The GIF loops forever.
    pub fn new<P: AsRef<Path>>(path: P, cfg: GifCanvasConfig) -> Result<Self, String> {
        if let Some(dir) = path.as_ref().parent() {
            if !dir.as_os_str().is_empty() {
                std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
            }
        }
        let file = File::create(path).map_err(|e| e.to_string())?;
        let mut encoder = Encoder::new(
            BufWriter::new(file),
            cfg.width as u16,
            cfg.height as u16,
            &[],
        )
        .map_err(|e| e.to_string())?;
        encoder
            .set_repeat(Repeat::Infinite)
            .map_err(|e| e.to_string())?;
        Ok(Self { encoder, cfg })
    }

    /// Append a frame using the configured delay.
    pub fn add_frame(&mut self, frame: GifFrame) -> Result<(), String> {
        self.add_frame_with_delay(frame, self.cfg.delay_cs)
    }

    /// Append a frame with an explicit delay in hundredths of a second.
    ///
    /// Useful to hold the final result on screen before the loop restarts.
    pub fn add_frame_with_delay(&mut self, frame: GifFrame, delay_cs: u16) -> Result<(), String> {
        let ss = self.cfg.supersample.max(1) as usize;
        let pixels = frame.downsample(ss);
        let mut f =
            Frame::from_rgb_speed(self.cfg.width as u16, self.cfg.height as u16, &pixels, 10);
        f.delay = delay_cs;
        self.encoder.write_frame(&f).map_err(|e| e.to_string())
    }

    /// Finish the recording and flush the file.
    pub fn finish(self) -> Result<(), String> {
        // Dropping the encoder finalizes the GIF trailer.
        drop(self.encoder);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rgb_parsing() {
        assert_eq!(rgb("#FF8000"), (255, 128, 0));
        assert_eq!(rgb("#000000"), (0, 0, 0));
        assert_eq!(rgb("bogus"), (0, 0, 0));
    }

    #[test]
    fn test_equal_aspect_expands_shorter_axis() {
        let cfg = GifCanvasConfig::new(400, 400, (0.0, 100.0), (0.0, 10.0));
        let yspan = cfg.y_range.1 - cfg.y_range.0;
        assert!((yspan - 100.0).abs() < 1e-9);
        // Center is preserved.
        assert!(((cfg.y_range.0 + cfg.y_range.1) / 2.0 - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_recorder_writes_looping_gif() {
        let path = std::env::temp_dir().join("rust_robotics_gif_recorder_test.gif");
        let cfg = GifCanvasConfig::new(120, 90, (0.0, 12.0), (0.0, 9.0));
        let mut rec = GifRecorder::new(&path, cfg.clone()).unwrap();
        for i in 0..3 {
            let mut frame = GifFrame::new(&cfg);
            frame.draw_path_xy(&[1.0, 11.0], &[1.0, i as f64 + 1.0], (255, 0, 0), 2.0);
            frame.draw_point(6.0, 4.5, (0, 0, 255), 3.0);
            frame.draw_robot(&Pose2D::new(3.0, 3.0, 0.5), 0.8, (0, 128, 128));
            rec.add_frame(frame).unwrap();
        }
        rec.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert!(bytes.starts_with(b"GIF89a"));
        assert!(bytes.len() > 100);
        std::fs::remove_file(&path).ok();
    }
}
