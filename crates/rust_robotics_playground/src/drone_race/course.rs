//! Five ordered gate centers on a bounded decimeter grid. The same validated
//! course is used by the editor, physical flight, opponent, and shared replay.

use rust_robotics_control::{RacingGateLap3D, RacingGatePlane3D};

use super::flight::sub;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Course {
    points: [[i16; 3]; 5],
}

impl Default for Course {
    fn default() -> Self {
        Self::NEON
    }
}

impl Course {
    pub const NEON: Self = Self {
        points: [
            [40, 12, 24],
            [80, -12, 34],
            [120, 12, 25],
            [160, -12, 34],
            [200, 0, 24],
        ],
    };
    pub const CLIMB: Self = Self {
        points: [
            [40, 0, 28],
            [80, 0, 40],
            [120, 8, 52],
            [160, -6, 56],
            [200, 0, 40],
        ],
    };
    pub const ALL: [Self; 2] = [Self::NEON, Self::CLIMB];

    pub fn is_custom(self) -> bool {
        !Self::ALL.contains(&self)
    }

    pub fn slug(self) -> &'static str {
        if self == Self::NEON {
            "neon"
        } else if self == Self::CLIMB {
            "climb"
        } else {
            "custom"
        }
    }

    pub fn title(self) -> &'static str {
        if self == Self::NEON {
            "Neon slalom"
        } else if self == Self::CLIMB {
            "Sky climb"
        } else {
            "Your circuit"
        }
    }

    pub fn from_slug(value: &str) -> Option<Self> {
        match value {
            "neon" => Some(Self::NEON),
            "climb" => Some(Self::CLIMB),
            _ => None,
        }
    }

    pub fn centers(self) -> [[f64; 3]; 5] {
        self.points.map(|p| p.map(|v| f64::from(v) / 10.0))
    }

    pub fn x_range(self, gate: usize) -> (f64, f64) {
        let low = if gate == 0 {
            20
        } else {
            self.points[gate - 1][0] + 20
        };
        let high = if gate == 4 {
            240
        } else {
            self.points[gate + 1][0] - 20
        };
        (f64::from(low) / 10.0, f64::from(high) / 10.0)
    }

    /// Editing clamps to the design envelope and preserves gate order.
    pub fn moved(mut self, gate: usize, point: [f64; 3]) -> Self {
        if gate >= 5 || point.iter().any(|v| !v.is_finite()) {
            return self;
        }
        let (low, high) = self.x_range(gate);
        self.points[gate] = [
            (point[0].clamp(low, high) * 10.0).round() as i16,
            (point[1].clamp(-3.0, 3.0) * 10.0).round() as i16,
            (point[2].clamp(1.2, 6.0) * 10.0).round() as i16,
        ];
        self
    }

    pub fn encode(self) -> String {
        format!(
            "v1.{}",
            self.points
                .iter()
                .map(|p| format!("{}_{}_{}", p[0], p[1], p[2]))
                .collect::<Vec<_>>()
                .join(".")
        )
    }

    /// Only canonical, bounded layouts are accepted. Separators are URL-safe,
    /// including when a receiving application uses URLSearchParams.
    pub fn decode(encoded: &str) -> Option<Self> {
        if encoded.len() > 128 {
            return None;
        }
        let mut rows = encoded.strip_prefix("v1.")?.split('.');
        let mut points = [[0_i16; 3]; 5];
        let mut previous_x = 0;
        for point in &mut points {
            let mut values = rows.next()?.split('_');
            for value in point.iter_mut() {
                *value = values.next()?.parse().ok()?;
            }
            if values.next().is_some()
                || !(previous_x + 20..=240).contains(&point[0])
                || !(-30..=30).contains(&point[1])
                || !(12..=60).contains(&point[2])
            {
                return None;
            }
            previous_x = point[0];
        }
        if rows.next().is_some() {
            return None;
        }
        let course = Self { points };
        (course.encode() == encoded).then_some(course)
    }

    pub fn lap(self) -> RacingGateLap3D {
        let mut previous = [0.0, 0.0, 2.0];
        let gates = self
            .centers()
            .into_iter()
            .map(|center| {
                let direction = sub(center, previous);
                previous = center;
                RacingGatePlane3D::new(center, direction, [0.0, 0.0, 1.0], 0.9, 0.9)
                    .expect("validated course has separated gate centers")
            })
            .collect();
        RacingGateLap3D::open(gates).expect("validated course has five gates")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edited_layout_roundtrips_and_clamps_without_reordering_gates() {
        let course = Course::CLIMB.moved(1, [4.0, -0.7, 3.6]);
        assert_eq!(course.centers()[1], [6.0, -0.7, 3.6]);
        assert!(course.is_custom());
        assert_eq!(Course::decode(&course.encode()), Some(course));
        assert_eq!(course.lap().gates[1].center(), [6.0, -0.7, 3.6]);
        let bounded = course.moved(1, [100.0, -100.0, 100.0]);
        assert_eq!(bounded.centers()[1], [10.0, -3.0, 6.0]);
        assert_eq!(bounded.moved(1, [f64::NAN, 0.0, 2.0]), bounded);
        assert_eq!(bounded.moved(99, [1.0; 3]), bounded);
        assert_eq!(Course::decode(&bounded.encode()), Some(bounded));
    }

    #[test]
    fn links_reject_invalid_versions_order_coordinates_and_oversized_input() {
        let valid = Course::CLIMB.encode();
        for invalid in [
            valid.replace("v1.", "v2."),
            valid.replace("80_0_40", "40_0_40"),
            valid.replace("80_0_40", "80_31_40"),
            valid.replace("80_0_40", "80_0_11"),
            valid.replace("80_0_40", "80_0_61"),
            valid.replace("80_0_40", "NaN_0_40"),
            valid.replace("80_0_40", "080_0_40"),
            format!("{valid}.240_0_40"),
            "v1.40_0_28".to_string(),
            "x".repeat(129),
        ] {
            assert!(Course::decode(&invalid).is_none(), "{invalid}");
        }
    }
}
