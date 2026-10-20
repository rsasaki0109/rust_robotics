//! Bounded playback of cells emitted by the real planner's expansion callback.

use super::Cell;

#[derive(Debug, Clone)]
pub(super) struct SearchReplay {
    pub cells: Vec<Cell>,
    pub cursor: usize,
    pub playing: bool,
    pub visible: bool,
    pub rate: usize,
    last_time: Option<f64>,
    phase: f64,
}

impl Default for SearchReplay {
    fn default() -> Self {
        Self {
            cells: Vec::new(),
            cursor: 0,
            playing: false,
            visible: true,
            rate: 40,
            last_time: None,
            phase: 0.0,
        }
    }
}

impl SearchReplay {
    pub fn reset(&mut self, cells: Vec<Cell>) {
        self.cells = cells;
        self.seek(self.cells.len());
    }

    pub fn seek(&mut self, cursor: usize) {
        self.cursor = cursor.min(self.cells.len());
        self.playing = false;
        self.last_time = None;
        self.phase = 0.0;
    }

    pub fn replay(&mut self) {
        self.seek(0);
        self.visible = true;
        self.playing = !self.cells.is_empty();
    }

    pub fn complete(&self) -> bool {
        self.cursor == self.cells.len()
    }

    pub fn advance(&mut self, now: f64) {
        let elapsed = self.last_time.replace(now).map_or(0.0, |last| {
            let dt = now - last;
            if (0.0..1.0).contains(&dt) {
                dt.min(0.1)
            } else {
                0.0
            }
        });
        if !self.playing {
            return;
        }
        self.phase += elapsed * self.rate as f64;
        let count = self.phase.floor() as usize;
        self.phase -= count as f64;
        self.cursor = (self.cursor + count).min(self.cells.len());
        if self.complete() {
            self.playing = false;
        }
    }

    pub fn apply_query(&mut self, query: &str) {
        self.visible = crate::share::boolean(query, "search").unwrap_or(self.visible);
        if let Some(rate) =
            crate::share::bounded_usize(query, "search_rate", 200).filter(|r| *r >= 5)
        {
            self.rate = rate;
        }
        if let Some(cursor) =
            crate::share::bounded_usize(query, "search_step", super::GRID_W * super::GRID_H)
        {
            self.seek(cursor);
        }
        self.playing = self.visible
            && !self.complete()
            && crate::share::boolean(query, "search_playing").unwrap_or(false);
        if !self.visible {
            self.seek(self.cells.len());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn playback_pauses_seeks_and_discards_background_gaps() {
        let mut replay = SearchReplay::default();
        replay.reset((0..100).map(|i| (i, 0)).collect());
        replay.replay();
        replay.advance(0.0);
        replay.advance(0.1);
        assert_eq!(replay.cursor, 4);
        replay.seek(replay.cursor);
        replay.advance(0.2);
        replay.advance(0.3);
        assert_eq!(replay.cursor, 4);
        replay.playing = true;
        replay.advance(500.0);
        assert_eq!(replay.cursor, 4);
        replay.advance(500.1);
        assert_eq!(replay.cursor, 8);
        replay.seek(usize::MAX);
        assert!(replay.complete());
        assert!(!replay.playing);
        replay.reset(vec![(2, 3)]);
        assert_eq!(replay.cursor, 1);
        assert!(!replay.playing);
    }
}
