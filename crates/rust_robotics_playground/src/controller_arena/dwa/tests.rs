use super::*;

#[test]
fn sampled_costs_and_recorded_commands_match_an_independent_evaluator() {
    for case in Case::ALL {
        for policy in Policy::ALL {
            let mut lab = DwaLab {
                case,
                policy,
                probe: case.probe(),
                ..DwaLab::default()
            };
            lab.enter();
            for at in [0, 1, 17, lab.last()] {
                lab.seek(at);
                let mut best = None;
                let mut minimum = f64::MAX;
                let trace = lab.trace();
                let obstacles = lab.obstacles(lab.cursor);
                for (i, c) in trace.candidates.iter().enumerate() {
                    let states = &c.trajectory.states;
                    assert_eq!(states[0], *lab.state());
                    let mut nearest = f64::MAX;
                    for pair in states.windows(2) {
                        let yaw = pair[0][2] + c.trajectory.control[1] * DT;
                        assert!(
                            (pair[1][0] - pair[0][0] - c.trajectory.control[0] * yaw.cos() * DT)
                                .abs()
                                < 1e-12
                        );
                        assert!(
                            (pair[1][1] - pair[0][1] - c.trajectory.control[0] * yaw.sin() * DT)
                                .abs()
                                < 1e-12
                        );
                    }
                    for s in states {
                        for &p in &obstacles {
                            nearest = nearest.min(distance(point(s), p));
                        }
                    }
                    assert_eq!(c.obstacle_cost.is_none(), nearest <= lab.radius);
                    let end = states.last().unwrap();
                    let angle = (lab.goal[1] - end[1]).atan2(lab.goal[0] - end[0]) - end[2];
                    let w = policy.weights();
                    let goal = w[0] * angle.sin().atan2(angle.cos()).abs();
                    let speed = w[1] * (1.2 - end[3]);
                    assert!((goal - c.goal_cost).abs() < 1e-12);
                    assert!((speed - c.speed_cost).abs() < 1e-12);
                    if let Some(obstacle) = c.obstacle_cost {
                        assert!((obstacle - w[2] / nearest).abs() < 1e-12);
                        let cost = goal + speed + obstacle;
                        assert!((cost - c.trajectory.cost).abs() < 1e-12);
                        if cost <= minimum {
                            minimum = cost;
                            best = Some(i);
                        }
                    }
                }
                assert_eq!(trace.selected, best);
                if lab.cursor < lab.last() {
                    let command = trace.applied_control;
                    assert_eq!(lab.controls[lab.cursor], [command[0], command[1]]);
                    assert_eq!(
                        motion(*lab.state(), command, DT),
                        lab.states[lab.cursor + 1]
                    );
                }
                if case == Case::Trapped {
                    assert!(best.is_none());
                    assert_eq!(trace.applied_control, nalgebra::Vector2::zeros());
                    assert_eq!(
                        lab.status(),
                        if lab.cursor == LIMIT {
                            "limit"
                        } else {
                            "blocked"
                        }
                    );
                }
            }
        }
    }
}

#[test]
fn share_restores_exact_paused_decision_and_rejects_invalid_geometry() {
    let mut a = DwaLab::default();
    a.apply_query("tab=arena&arena_view=dwa&dwa_case=crossing&dwa_policy=speed&dwa_horizon=1.6&dwa_radius=0.7&dwa_goal=10.2,3.7&dwa_probe=4.3,2.6&dwa_at=27&dwa_selected=2&dwa_cloud=0&dwa_edit=edit_goal");
    let q = a.share_query();
    let mut b = DwaLab::default();
    b.apply_query(&q);
    assert_eq!(q, b.share_query());
    assert_eq!(a.states, b.states);
    assert_eq!(a.controls, b.controls);
    assert_eq!(a.trace().applied_control, b.trace().applied_control);
    assert!(!b.playing);
    b.apply_query("tab=arena&arena_view=dwa&dwa_goal=NaN,4&dwa_probe=200,2&dwa_radius=0&dwa_horizon=inf&dwa_at=9999");
    assert_eq!(b.goal, [10., 4.]);
    assert_eq!(b.probe, Case::Wall.probe());
    assert_eq!(b.radius, 0.45);
    assert_eq!(b.horizon, 2.4);
    assert!(parse_point("p=1,1", "p").is_some());
    assert!(parse_point("p=1,inf", "p").is_none());
}

#[test]
fn replay_is_clock_independent_and_drops_background_time() {
    let mut outputs = Vec::new();
    for hz in [100., 25., 2.5] {
        let mut a = DwaLab::default();
        a.enter();
        a.playing = true;
        a.advance(0.);
        for i in 1..=(2. * hz) as usize {
            a.advance(i as f64 / hz);
        }
        assert_eq!(a.cursor, 10);
        outputs.push(*a.state());
        a.advance(20.);
        assert_eq!(a.cursor, 10);
        a.playing = false;
        a.advance(21.);
        assert_eq!(a.cursor, 10);
        a.seek(3);
        assert_eq!(a.cursor, 3);
        assert_eq!(a.pending, 0.);
    }
    assert_eq!(outputs[0], outputs[1]);
    assert_eq!(outputs[0], outputs[2]);
}
