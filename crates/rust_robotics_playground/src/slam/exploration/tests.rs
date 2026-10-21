use super::*;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
fn lab(q: &str) -> ExploreLab {
    let mut s = ExploreLab::default();
    s.apply_query(&format!("slam_view=explore&{q}"));
    s
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9, "{a} != {b}");
}
fn neighbors(c: Cell, diagonal: bool) -> Vec<Cell> {
    (-1_i32..=1)
        .flat_map(|dx| (-1_i32..=1).map(move |dy| (dx, dy)))
        .filter(|&(dx, dy)| (dx != 0 || dy != 0) && (diagonal || dx.abs() + dy.abs() == 1))
        .map(|(dx, dy)| (c.0 + dx, c.1 + dy))
        .filter(|&(x, y)| x >= 0 && y >= 0 && x < WIDTH as i32 && y < HEIGHT as i32)
        .collect()
}
fn index(c: Cell) -> usize {
    c.1 as usize * WIDTH + c.0 as usize
}
// An independent label-correcting queue, rather than the library's binary heap.
fn costs(known: &[u8], start: Cell) -> Vec<f64> {
    let mut d = vec![f64::INFINITY; WIDTH * HEIGHT];
    d[index(start)] = 0.;
    let mut queue = VecDeque::from([start]);
    let mut queued = vec![false; WIDTH * HEIGHT];
    queued[index(start)] = true;
    while let Some(c) = queue.pop_front() {
        queued[index(c)] = false;
        for q in neighbors(c, true) {
            if known[index(q)] != 1 {
                continue;
            }
            let candidate = d[index(c)]
                + if c.0 != q.0 && c.1 != q.1 {
                    2_f64.sqrt()
                } else {
                    1.
                };
            if candidate < d[index(q)] {
                d[index(q)] = candidate;
                if !queued[index(q)] {
                    queued[index(q)] = true;
                    queue.push_back(q);
                }
            }
        }
    }
    d
}
fn visible(occupied: impl Fn(Cell) -> bool, from: Cell, to: Cell) -> bool {
    let n = (3 * (to.0 - from.0).abs().max((to.1 - from.1).abs())).max(1);
    for k in 1..=n {
        let t = f64::from(k) / f64::from(n);
        let c = (
            (f64::from(from.0) + t * f64::from(to.0 - from.0)).round() as i32,
            (f64::from(from.1) + t * f64::from(to.1 - from.1)).round() as i32,
        );
        if c == to {
            return true;
        }
        if occupied(c) {
            return false;
        }
    }
    true
}
fn observe(world: &FrontierNavWorld, known: &mut [u8], from: Cell, range: f64) {
    for y in 0..HEIGHT as i32 {
        for x in 0..WIDTH as i32 {
            let c = (x, y);
            if distance(c, from) <= range
                && visible(|p| world.occupied[p.0 as usize][p.1 as usize], from, c)
            {
                known[index(c)] = if world.occupied[x as usize][y as usize] {
                    2
                } else {
                    1
                };
            }
        }
    }
}
fn clusters(known: &[u8]) -> BTreeMap<Cell, (BTreeSet<Cell>, usize)> {
    let unknowns = |c| {
        neighbors(c, false)
            .iter()
            .filter(|&&p| known[index(p)] == 0)
            .count()
    };
    let mut todo: BTreeSet<_> = (0..WIDTH * HEIGHT)
        .map(|i| ((i % WIDTH) as i32, (i / WIDTH) as i32))
        .filter(|&c| known[index(c)] == 1 && unknowns(c) > 0)
        .collect();
    let mut result = BTreeMap::new();
    while let Some(&seed) = todo.first() {
        let mut component = BTreeSet::new();
        let mut stack = vec![seed];
        todo.remove(&seed);
        while let Some(c) = stack.pop() {
            component.insert(c);
            for q in neighbors(c, true) {
                if todo.remove(&q) {
                    stack.push(q);
                }
            }
        }
        let x = component.iter().map(|c| f64::from(c.0)).sum::<f64>() / component.len() as f64;
        let y = component.iter().map(|c| f64::from(c.1)).sum::<f64>() / component.len() as f64;
        let metric = |c: Cell| (f64::from(c.0) - x).powi(2) + (f64::from(c.1) - y).powi(2);
        let rep = *component
            .iter()
            .min_by(|&&a, &&b| metric(a).total_cmp(&metric(b)).then(a.cmp(&b)))
            .unwrap();
        let openness = component.iter().map(|&c| unknowns(c)).sum();
        result.insert(rep, (component, openness));
    }
    result
}
fn verify(s: &ExploreLab) {
    let report = s.report.as_ref().unwrap();
    let mut known = vec![0; WIDTH * HEIGHT];
    observe(&s.world, &mut known, s.world.start, s.range);
    assert_eq!(report.path[0], s.world.start);
    let mut length = 0.;
    for (i, step) in s.steps.iter().enumerate() {
        assert_eq!(step.robot, report.path[step.path_index]);
        assert_eq!(flat(&step.known), known);
        if step.phase == FrontierNavPhase::Frontier {
            let expected = clusters(&known);
            assert_eq!(step.candidates.len(), expected.len());
            let max = expected.values().map(|c| c.1).max().unwrap().max(1) as f64;
            let d = costs(&known, step.robot);
            let [a, b, c, e] = s.strategy.weights();
            for candidate in &step.candidates {
                let (members, openness) = &expected[&candidate.representative];
                assert_eq!(
                    &candidate.members.iter().copied().collect::<BTreeSet<_>>(),
                    members
                );
                assert_eq!(candidate.openness, *openness);
                near(candidate.normalized_openness, *openness as f64 / max);
                let gain = distance(step.robot, s.world.goal)
                    - distance(candidate.representative, s.world.goal);
                near(candidate.goal_gain, gain);
                let los = visible(
                    |p| known[index(p)] == 2,
                    step.robot,
                    candidate.representative,
                );
                assert_eq!(candidate.line_of_sight, los);
                let cost = d[index(candidate.representative)];
                assert_eq!(candidate.travel_cost.is_some(), cost.is_finite());
                if cost.is_finite() {
                    near(candidate.travel_cost.unwrap(), cost);
                    near(
                        candidate.score.unwrap(),
                        a * gain - b * cost
                            + c * if los { 1. } else { 0. }
                            + e * (*openness as f64 / max),
                    );
                } else {
                    assert_eq!(candidate.score, None);
                }
            }
            let chosen = &step.candidates[step.selected.unwrap()];
            let max = step
                .candidates
                .iter()
                .filter_map(|c| c.score)
                .fold(f64::NEG_INFINITY, f64::max);
            assert_eq!(chosen.score, Some(max));
            for c in &step.candidates {
                if c.score == chosen.score {
                    assert!(chosen.representative <= c.representative);
                }
            }
        }
        if let Some(next) = s.steps.get(i + 1) {
            let target = match step.phase {
                FrontierNavPhase::Frontier => {
                    step.candidates[step.selected.unwrap()].representative
                }
                FrontierNavPhase::Goal => {
                    assert_eq!(known[index(s.world.goal)], 1);
                    s.world.goal
                }
                _ => panic!("terminal step has a successor"),
            };
            let d = costs(&known, target);
            for pair in report.path[step.path_index..=next.path_index].windows(2) {
                let (p, q) = (pair[0], pair[1]);
                assert!(neighbors(p, true).contains(&q));
                assert_eq!(known[index(q)], 1);
                let min = neighbors(p, true)
                    .iter()
                    .filter(|&&v| known[index(v)] == 1)
                    .map(|&v| d[index(v)])
                    .fold(f64::INFINITY, f64::min);
                near(d[index(q)], min);
                assert!(d[index(q)] < d[index(p)]);
                length += distance(p, q);
            }
            if step.phase == FrontierNavPhase::Frontier && next.path_index > step.path_index {
                observe(&s.world, &mut known, next.robot, s.range);
            }
        }
    }
    near(length, report.path_length);
    assert_eq!(known, flat(&report.known));
    let last = s.steps.last().unwrap();
    assert_eq!(last.path_index, s.last());
    assert_eq!(report.reached_goal, last.phase == FrontierNavPhase::Arrived);
    match last.phase {
        FrontierNavPhase::Arrived => assert_eq!(*report.path.last().unwrap(), s.world.goal),
        FrontierNavPhase::NoFrontiers => assert!(clusters(&known).is_empty()),
        FrontierNavPhase::IterationLimit => assert_eq!(report.iterations, LIMIT),
        FrontierNavPhase::NoProgress => {
            assert_eq!(s.steps[s.steps.len() - 2].path_index, last.path_index)
        }
        FrontierNavPhase::NoReachableFrontier => assert!(clusters(&known)
            .keys()
            .all(|c| !costs(&known, last.robot)[index(*c)].is_finite())),
        _ => panic!("missing terminal result"),
    }
}
#[test]
fn exploration_default_worlds_expose_occlusion_and_real_stopping_results() {
    for case in Case::ALL {
        let mut s = lab("");
        s.preset(case);
        verify(&s);
        let r = s.report.as_ref().unwrap();
        println!(
            "{} reached={} last={} phase={:?} firstwall={} decisions={}",
            case.slug(),
            r.reached_goal,
            s.last(),
            s.steps.last().unwrap().phase,
            s.first_wall(),
            s.steps.len()
        );
        assert!(s.last() > 0);
        assert!(s.coverage[0] < WIDTH * HEIGHT);
        assert!(s.coverage.windows(2).all(|a| a[0] <= a[1]));
        assert_eq!(
            s.coverage[0],
            flat(&s.steps[0].known).iter().filter(|&&k| k != 0).count()
        );
        if case == Case::Sealed {
            assert!(!r.reached_goal);
        } else {
            assert!(r.reached_goal);
        }
        if case == Case::Corner {
            assert!(s.first_wall() > 0);
        }
    }
}
#[test]
fn exploration_matches_independent_sensor_clusters_costs_scores_and_gradient_over_72_runs() {
    let mut s = lab("");
    for case in Case::ALL {
        for strategy in Strategy::ALL {
            for range in [3., 5., 7.] {
                for budget in [1, 3] {
                    s.case = case;
                    s.world = world(case);
                    s.strategy = strategy;
                    s.range = range;
                    s.budget = budget;
                    s.record();
                    verify(&s);
                }
            }
        }
    }
}
#[test]
fn exploration_shares_exact_world_decisions_selection_and_paused_cursor() {
    let mut s=lab("explore_case=doors&explore_range=7&explore_strategy=curious&explore_budget=1&explore_at=8&explore_truth=1&explore_route=0&explore_edit=walls");
    s.selected = s.step().candidates.len().saturating_sub(1);
    s.playing = true;
    let received = lab(&s.share_query());
    assert!(!received.playing);
    assert_eq!(received.cursor, s.cursor);
    assert_eq!(received.selected, s.selected);
    assert_eq!(received.world, s.world);
    assert_eq!(received.steps, s.steps);
    assert_eq!(received.report, s.report);
    assert_eq!(received.share_query(), s.share_query());
    let generation = s.generation;
    s.active = false;
    s.enter();
    assert_eq!(s.generation, generation);
    assert!(!s.playing);
    let initial = ExploreLab::default();
    assert!(initial.report.is_none() && initial.steps.is_empty());
    let invalid=lab("explore_range=NaN&explore_budget=0&explore_start=99,2&explore_map=bad&explore_at=99999&explore_candidate=999");
    assert_eq!(invalid.range, 5.);
    assert_eq!(invalid.budget, 3);
    assert_eq!(invalid.world.start, (1, 8));
    assert_eq!(invalid.cursor, invalid.last());
    assert_eq!(invalid.selected, 0);
    assert_eq!(lab("explore_event=wall").cursor, lab("").first_wall());
    assert_eq!(lab("explore_event=wall&explore_at=1").cursor, 1);
}
#[test]
fn exploration_edits_protect_endpoints_and_overlays_do_not_change_knowledge_or_plans() {
    let mut s = lab("");
    s.seek(s.first_wall());
    let generation = s.generation;
    let data = (s.steps.clone(), s.report.clone(), s.coverage.clone());
    s.truth = true;
    s.route = false;
    s.edit = Edit::Inspect;
    s.set_cell((7, 5));
    assert_eq!(s.generation, generation);
    assert_eq!(
        (s.steps.clone(), s.report.clone(), s.coverage.clone()),
        data
    );
    s.edit = Edit::Goal;
    s.set_cell((12, 8));
    assert_eq!(s.generation, generation);
    assert!(!s.note.is_empty());
    s.set_cell((21, 9));
    assert_eq!(s.world.goal, (21, 9));
    assert_eq!(s.cursor, 0);
    assert_eq!(s.generation, generation + 1);
    s.edit = Edit::Start;
    s.set_cell((2, 7));
    assert_eq!(s.world.start, (2, 7));
    assert_eq!(s.generation, generation + 2);
    s.paint((0, 7), (23, 7));
    assert!(s.world.occupied[0][7] && s.world.occupied[23][7]);
    assert!(!s.world.occupied[2][7]);
    assert!(!s.world.occupied[21][9]);
    verify(&s);
    let decoded = lab(&format!(
        "explore_start=2,7&explore_goal=21,9&explore_map={}",
        "1".repeat(WIDTH * HEIGHT)
    ));
    assert!(!decoded.world.occupied[2][7] && !decoded.world.occupied[21][9]);
    assert!(decoded.world.occupied[0][0]);
    assert!(!decoded.report.as_ref().unwrap().reached_goal);
}
#[test]
fn exploration_replay_uses_elapsed_time_and_keeps_observations_at_actual_boundaries() {
    for hz in [100., 25., 2.5] {
        let mut s = lab("");
        let generation = s.generation;
        s.playing = true;
        for tick in 0..=(hz * 2.) as usize {
            s.advance(tick as f64 / hz);
        }
        assert_eq!(s.cursor, 8.min(s.last()));
        assert_eq!(s.generation, generation);
        s.advance(100.);
        assert_eq!(s.cursor, 8.min(s.last()));
        s.seek(2);
        assert!(!s.playing);
        s.advance(100.4);
        assert_eq!(s.cursor, 2);
        assert_eq!(s.generation, generation);
        let step = s.step();
        assert_eq!(
            s.coverage[2],
            flat(&step.known).iter().filter(|&&v| v != 0).count()
        );
    }
}
