//! Gate feedback derived from the selected physical history, never wall time.

use egui::{Color32, FontId, Rect, Stroke, Vec2};

use super::flight::{Flight, Status, DT};
use super::renderer::{CYAN, GOLD};

const DURATION: f64 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Checkpoint {
    pub gate: usize,
    pub frame: usize,
    pub age: f64,
    pub finish: bool,
}

impl Checkpoint {
    pub fn at(flight: &Flight, cursor: usize, time: f64) -> Option<Self> {
        let cursor = cursor.min(flight.last_index());
        let gate = flight.frames[cursor].passed;
        if gate == 0 || !time.is_finite() {
            return None;
        }
        // Restrict the search to the selected prefix: cached future gates
        // must not leak into an earlier replay frame or photograph.
        let frame = flight.frames[..=cursor]
            .iter()
            .position(|frame| frame.passed >= gate)?;
        let crossing_time = frame as f64 * DT;
        (time >= crossing_time && time < crossing_time + DURATION).then_some(Self {
            gate,
            frame,
            age: time - crossing_time,
            finish: flight.frames[frame].status == Status::Finished,
        })
    }

    pub fn time(self) -> f64 {
        self.frame as f64 * DT
    }

    pub fn progress(self) -> f64 {
        self.age / DURATION
    }

    pub fn fade(self) -> f32 {
        (1.0 - self.progress()).powi(2) as f32
    }

    pub fn paint(self, painter: &egui::Painter, scene: Rect, total: usize, split: Option<f64>) {
        let compact = super::pad::compact(scene);
        let height = if compact { 64.0 } else { 76.0 };
        let width = (scene.width() - 32.0).min(if compact { 228.0 } else { 260.0 });
        let y = if compact {
            174.0
        } else if scene.width() < 600.0 {
            302.0
        } else {
            210.0
        };
        let center = egui::pos2(
            scene.center().x,
            (scene.top() + y).min(scene.bottom() - height * 0.5 - 12.0),
        );
        let card = Rect::from_center_size(center, Vec2::new(width, height));
        let fade = (1.0 - self.progress()).min(0.25) as f32 * 4.0;
        painter.rect_stroke(
            scene.shrink(3.0),
            8.0,
            Stroke::new(2.0_f32, CYAN.gamma_multiply(self.fade() * 0.55)),
            egui::StrokeKind::Inside,
        );
        painter.rect_filled(card.expand(4.0), 12.0, CYAN.gamma_multiply(0.05 * fade));
        painter.rect_filled(card, 9.0, Color32::from_black_alpha((225.0 * fade) as u8));
        painter.rect_stroke(
            card,
            9.0,
            Stroke::new(1.0_f32, CYAN.gamma_multiply(0.8 * fade)),
            egui::StrokeKind::Inside,
        );
        painter.text(
            center - Vec2::new(0.0, 10.0),
            egui::Align2::CENTER_CENTER,
            format!("GATE {:02} / {total:02} CLEAR", self.gate),
            FontId::proportional(if compact { 16.0 } else { 19.0 }),
            CYAN.gamma_multiply(fade),
        );
        painter.text(
            center + Vec2::new(0.0, 12.0),
            egui::Align2::CENTER_CENTER,
            split.map_or_else(
                || format!("{:.2} s  /  CHECKPOINT", self.time()),
                |split| format!("{:.2} s  /  {split:+.2} s vs BEST", self.time()),
            ),
            FontId::monospace(if compact { 9.5 } else { 10.5 }),
            if split.is_some_and(|split| split > 0.0) {
                GOLD
            } else {
                Color32::WHITE
            }
            .gamma_multiply(fade),
        );
        painter.line_segment(
            [
                card.left_bottom() + Vec2::new(12.0, -5.0),
                card.left_bottom()
                    + Vec2::new(12.0 + (width - 24.0) * (1.0 - self.progress()) as f32, -5.0),
            ],
            Stroke::new(2.0_f32, CYAN.gamma_multiply(fade)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_history_controls_checkpoint_time_and_lifetime() {
        let flight = super::super::records::tests::finish(super::super::flight::Course::CLIMB, 0);
        assert!(Checkpoint::at(&flight, 0, 0.0).is_none());
        for gate in 1..=5 {
            let frame = flight.frames.iter().position(|f| f.passed >= gate).unwrap();
            let time = frame as f64 * DT;
            let event = Checkpoint::at(&flight, frame, time).unwrap();
            assert_eq!(
                (event.gate, event.frame, event.finish),
                (gate, frame, gate == 5)
            );
            assert_eq!(event.time(), time);
            assert_eq!(event.age, 0.0);
            assert!(Checkpoint::at(&flight, frame - 1, time)
                .map_or(true, |earlier| earlier.gate < gate));
            assert!(Checkpoint::at(&flight, frame, time - 0.001).is_none());
            assert!(Checkpoint::at(&flight, frame, time + DURATION).is_none());
        }
        for time in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Checkpoint::at(&flight, flight.last_index(), time).is_none());
        }
    }
}
