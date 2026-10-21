use super::*;
fn lab(q: &str) -> MappingLab {
    let mut s = MappingLab::default();
    s.apply_query(&format!("slam_view=mapping&{q}"));
    s
}
fn near(a: f32, b: f32) {
    assert!((a - b).abs() < 2e-6, "{a} != {b}");
}

#[test]
fn mapping_cases_distinguish_clearing_absence_and_hit_priority() {
    let mut s = lab("");
    let i = s.index();
    near(s.frames[4].values[i], 3.6);
    near(s.frames[5].values[i], 3.2);
    assert_eq!(s.cleared(), Some(14));
    assert_eq!(s.frames[20].states[i], 1);
    for case in [Case::Repeat, Case::Absent, Case::Conflict] {
        s.case = case;
        s.record();
        near(
            s.frames[20].values[i],
            if case == Case::Absent { 3.6 } else { 5. },
        );
        assert_eq!(s.cleared(), None);
        if case == Case::Absent {
            assert!(s.frames[5..]
                .iter()
                .all(|f| f.points.is_empty() && f.updates.is_empty()));
            assert_eq!(s.frames[4].values, s.frames[20].values);
            assert_eq!(s.frames[4].seen, s.frames[20].seen);
        }
        if case == Case::Conflict {
            for f in &s.frames[1..] {
                let u: Vec<_> = f
                    .updates
                    .iter()
                    .filter(|u| [u.x, u.y] == s.selected)
                    .collect();
                assert_eq!(u.len(), 1);
                assert_eq!(u[0].evidence, OccupancyEvidence::Hit);
            }
        }
    }
}

// Independent integer rasterization, per-scan union, and Bayesian accumulation.
// It does not call the library's traversal, state classification, or trace API.
fn oracle_marks(origin: Point, points: &[Point]) -> BTreeMap<usize, u8> {
    let mut marks = BTreeMap::new();
    for p in points {
        let (mut x, mut y) = (
            (origin[0] / 0.5).floor() as i32,
            (origin[1] / 0.5).floor() as i32,
        );
        let (tx, ty) = ((p[0] / 0.5).floor() as i32, (p[1] / 0.5).floor() as i32);
        let dx = (tx - x).abs();
        let dy = (ty - y).abs();
        let sx = (tx - x).signum();
        let sy = (ty - y).signum();
        let mut err = dx - dy;
        loop {
            let index = y as usize * WIDTH + x as usize;
            let mark = if (x, y) == (tx, ty) { 2 } else { 1 };
            marks
                .entry(index)
                .and_modify(|v: &mut u8| *v = (*v).max(mark))
                .or_insert(mark);
            if (x, y) == (tx, ty) {
                break;
            }
            let e = 2 * err;
            if e >= -dy {
                err -= dy;
                x += sx;
            }
            if e <= dx {
                err += dx;
                y += sy;
            }
        }
    }
    marks
}
#[test]
fn mapping_all_cells_match_independent_scan_union_and_float32_accumulation() {
    let mut s = lab("");
    for case in Case::ALL {
        for hit in [0.4, 0.9, 1.4] {
            for miss in [-0.2, -0.4, -0.8] {
                for duplicates in [false, true] {
                    for sensor in [[2.25, 4.25], [1.125, 2.875]] {
                        s.case = case;
                        s.hit = hit;
                        s.miss = miss;
                        s.duplicates = duplicates;
                        s.sensor = sensor;
                        s.record();
                        let mut values = vec![0_f32; WIDTH * HEIGHT];
                        let mut seen = vec![0; WIDTH * HEIGHT];
                        for (at, f) in s.frames.iter().enumerate() {
                            let shift = if case == Case::Clear && at > 4 {
                                3.
                            } else {
                                0.
                            };
                            let mut expected = if at == 0 || (case == Case::Absent && at > 4) {
                                vec![]
                            } else if case == Case::Conflict {
                                vec![[6.25, 4.25], [9.25, 4.45]]
                            } else {
                                vec![
                                    [6.25 + shift, 3.25],
                                    [6.25 + shift, 4.25],
                                    [6.25 + shift, 5.25],
                                ]
                            };
                            if duplicates {
                                expected.extend(expected.clone());
                            }
                            assert_eq!(f.points, expected);
                            let marks = oracle_marks(sensor, &expected);
                            assert_eq!(marks.len(), f.updates.len());
                            let mut actual = BTreeMap::new();
                            for u in &f.updates {
                                let i = u.y * WIDTH + u.x;
                                assert!(actual.insert(i, u).is_none());
                                assert_eq!(u.before, values[i]);
                                assert_eq!(
                                    u.evidence,
                                    if marks[&i] == 2 {
                                        OccupancyEvidence::Hit
                                    } else {
                                        OccupancyEvidence::Miss
                                    }
                                );
                            }
                            for (i, mark) in marks {
                                values[i] =
                                    (values[i] + if mark == 2 { hit } else { miss }).clamp(-5., 5.);
                                seen[i] += 1;
                                assert_eq!(actual[&i].after, values[i]);
                            }
                            assert_eq!(f.values, values);
                            assert_eq!(f.seen, seen);
                            for (i, &v) in values.iter().enumerate() {
                                assert_eq!(
                                    f.states[i],
                                    if v > 0.6 {
                                        2
                                    } else if v < -0.3 {
                                        1
                                    } else {
                                        0
                                    }
                                );
                                let p = 1. / (1. + (-f64::from(v)).exp());
                                assert_eq!(probability(f.values[i]), p);
                            }
                        }
                    }
                }
            }
        }
    }
}
#[test]
fn mapping_observed_uncertainty_counts_and_duplicate_invariance() {
    let mut s = lab("mapping_hit=0.4&mapping_miss=-0.2&mapping_at=1");
    let i = s.index();
    assert_eq!(s.frame().states[i], 0);
    assert_eq!(s.frame().seen[i], 1);
    assert_eq!(s.frame().values[i], 0.4);
    assert!(s.stats()[1] > 0);
    assert_eq!(s.stats().iter().sum::<usize>(), WIDTH * HEIGHT);
    assert!(s
        .frame()
        .states
        .iter()
        .zip(&s.frame().seen)
        .any(|(&c, &n)| c == 0 && n == 0));
    let frames = s.frames.clone();
    s.duplicates = true;
    s.record();
    for (a, b) in frames.iter().zip(&s.frames) {
        assert_eq!(a.values, b.values);
        assert_eq!(a.updates, b.updates);
        assert_eq!(a.states, b.states);
        assert_eq!(a.seen, b.seen);
        assert_eq!(b.points.len(), a.points.len() * 2);
    }
    s.seek(4);
    let value = s.frame().values[i];
    s.seek(1);
    assert_ne!(s.frame().values[i], value);
    assert_eq!(
        s.frame()
            .updates
            .iter()
            .find(|u| u.y * WIDTH + u.x == i)
            .unwrap()
            .before,
        0.
    );
    // Equality at a threshold remains unknown in the library.
    for (hit, miss, expected) in [(0.6, -0.3, 0), (0.60001, -0.30001, 2)] {
        let mut grid = OccupancyGrid::new(
            Vector2::zeros(),
            Vector2::new(2., 1.),
            OccupancyConfig {
                resolution: 0.5,
                hit,
                miss,
                ..OccupancyConfig::default()
            },
        );
        grid.insert_scan(Pose2D::new(0.25, 0.25, 0.), &[Vector2::new(1., 0.)]);
        assert_eq!(
            grid.state(2, 0),
            if expected == 0 {
                CellState::Unknown
            } else {
                CellState::Occupied
            }
        );
        assert_eq!(
            grid.state(0, 0),
            if expected == 0 {
                CellState::Unknown
            } else {
                CellState::Free
            }
        );
    }
}
#[test]
fn mapping_share_edits_bounds_and_reentry_preserve_actual_experiment() {
    let mut s=lab("mapping_case=conflict&mapping_sensor=1.23456789,3.456789&mapping_return=7.12345678,5.12345678&mapping_hit=1.4&mapping_miss=-0.8&mapping_at=11&mapping_cell=14,10&mapping_future=0&mapping_rays=0&mapping_duplicates=1&mapping_edit=return");
    s.playing = true;
    let restored = lab(&s.share_query());
    assert_eq!(restored.frames, s.frames);
    assert_eq!(restored.cursor, 11);
    assert!(!restored.playing);
    assert_eq!(restored.share_query(), s.share_query());
    let generation = s.generation;
    s.active = false;
    s.enter();
    assert_eq!(s.generation, generation);
    assert_eq!(s.cursor, 11);
    assert!(!s.playing);
    s.edit = Edit::Inspect;
    s.set_point([0.01, 7.99]);
    assert_eq!(s.selected, [0, 15]);
    assert_eq!(s.generation, generation);
    s.edit = Edit::Sensor;
    s.set_point([-100., 100.]);
    assert_eq!(s.sensor, [0.75, 7.25]);
    assert_eq!(s.cursor, 0);
    assert_eq!(s.generation, generation + 1);
    s.edit = Edit::Return;
    s.set_point([100., -100.]);
    assert_eq!(s.target, [8.25, 2.25]);
    assert_eq!(s.selected, [16, 4]);
    let invalid=lab("mapping_sensor=NaN,inf&mapping_return=oops&mapping_hit=99&mapping_miss=NaN&mapping_at=999&mapping_cell=99,99");
    assert_eq!(invalid.sensor, [2.25, 4.25]);
    assert_eq!(invalid.target, [6.25, 4.25]);
    assert_eq!(invalid.hit, 0.9);
    assert_eq!(invalid.miss, -0.4);
    assert_eq!(invalid.cursor, LAST);
    assert_eq!(invalid.selected, [12, 8]);
    assert_eq!(lab("mapping_event=change").cursor, 5);
    assert_eq!(lab("mapping_event=change&mapping_at=2").cursor, 2);
    let inactive = MappingLab::default();
    assert!(!inactive.active);
    assert!(inactive.frames.is_empty());
}
#[test]
fn mapping_replay_uses_elapsed_time_and_seeking_never_rebuilds() {
    for hz in [100., 25., 2.5] {
        let mut s = lab("");
        let generation = s.generation;
        s.playing = true;
        for tick in 0..=(hz * 4.) as usize {
            s.advance(tick as f64 / hz);
        }
        assert_eq!(s.cursor, 8);
        assert_eq!(s.generation, generation);
        s.advance(100.);
        assert_eq!(s.cursor, 8);
        s.seek(3);
        s.advance(100.4);
        assert_eq!(s.cursor, 3);
        assert!(!s.playing);
        assert_eq!(s.generation, generation);
        s.playing = true;
        s.clock = None;
        for tick in 0..=(hz * 10.) as usize {
            s.advance(200. + tick as f64 / hz);
        }
        assert_eq!(s.cursor, LAST);
        assert!(!s.playing);
    }
}
