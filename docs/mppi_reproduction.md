# Vanilla MPPI Baseline

This is a deterministic, seedable baseline for reproducing MPPI-family control
papers such as TD-CD-MPPI.

Implemented slice:

1. 2-D double-integrator dynamics.
2. Seeded Gaussian control perturbations.
3. Exponential path-integral weighting.
4. Receding-horizon nominal control shift.
5. Goal, velocity, control, and terminal costs.
6. Circular obstacle penalties with temporal constraint discounting.
7. Grid-sampled terminal value estimates with bilinear interpolation.
8. Online terminal-value grid updates from discounted rollout cost-to-go.
9. Replay-buffer terminal-value updates from recent best rollouts.
10. Static SVG export for the learned replay value grid and executed rollout.
11. Sampling diagnostics and adaptive MPPI temperature selection by effective sample size.
12. Waypoint-track progress terminal values for race-track style MPPI.
13. Static SVG export comparing single-goal and track-progress MPPI on a slalom course.
14. Reference-free racing gates with gate-crossing logic and a gate-progress MPPI objective.
15. Static SVG export comparing waypoint-reference MPPI against gate-progress MPPI.
16. Moving circular obstacles and horizon-aligned goal trajectories for
    prediction-aware MPPI.
17. 3-D rectangular gate planes, a point-mass drone with drag and actuation
    limits, open/closed laps, and lap-progress metrics.
18. CSV/SVG benchmark sweeping planar, undulating, climbing, and high-drag 3-D
    racing courses.
19. Full quadrotor attitude model (collective thrust + body-rate inputs) whose
    orientation is driven by the gate-progress objective, with tilt and
    body-rate metrics.
20. Motor-level rotor-mixing quadrotor (four rotor thrusts -> collective thrust
    and body torques, body rates as states) with rotor saturation, so the
    thrust/torque trade-off enters racing.
21. Motor-lag and battery-sag powertrain layered on the rotor model (first-order
    spin-up lag plus a thrust ceiling that droops with load and state of charge),
    so an idealized plan degrades when it meets a laggy, draining powertrain.
22. Powertrain-aware MPPI controller that rolls candidates out through the lag and
    battery model, so it plans within the authority the pack can deliver and
    conserves charge for later gates.
23. Charge-budget term on the aware controller that penalizes below-reserve
    current draw, exposing a tunable endurance/progress trade-off over a draining
    multi-lap race.
24. Battery-recovery (relaxation-overpotential) model so the terminal voltage —
    and thus the thrust ceiling — recovers when the load eases, even though the
    state of charge only ever falls.

Run:

```bash
cargo run -p rust_robotics --example headless_mppi_double_integrator --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_constraint_discount --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_terminal_value --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_value_learning --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_replay_value_learning --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_value_grid_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_adaptive_temperature --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_track_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_track_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_racing_gate_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_racing_gate_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_adap_rpf_mppi --no-default-features --features control
cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_quadrotor --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_motor --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_aware --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_budget --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_recovery --no-default-features --features control
```

The constraint-discounted example compares vanilla MPPI against rollouts that
penalize circular obstacle violations with a discount factor over the horizon.

The terminal-value example compares a short-horizon controller against the same
controller with a sampled goal-distance value estimate at the rollout endpoint.

The value-learning example refines that grid online: it walks backward through
the best rollout's model costs, computes discounted cost-to-go targets, and
updates the nearest value-grid cells with an exponential moving average.

The replay-value example keeps a finite buffer of recent best rollouts and
reuses them for additional value-grid backups after each planning step.

The SVG export writes `docs/assets/mppi-replay-value-grid.svg`, combining the
learned terminal-value heatmap, obstacle safety margin, goal, and final executed
trajectory.

The adaptive-temperature example reports effective sample size and entropy for
fixed versus adaptive MPPI sampling weights.

The track-progress example converts a slalom waypoint course into terminal
values that reward progress along the course and penalize lateral deviation.

The track-progress SVG export writes `docs/assets/mppi-track-progress.svg`,
showing the slalom track, obstacles, terminal-value heatmap, and both MPPI
trajectories.

The racing-gate example follows the reference-free objective from "Rethinking
Reference Trajectories in Agile Drone Racing": each rollout minimizes the
change in squared distance to the active gate, switches gates only after a
geometric crossing through the gate width, and uses no goal-tracking cost in
the gate-progress controller.

The racing-gate SVG export writes
`docs/assets/mppi-racing-gate-progress.svg`, showing oriented gates, obstacle
margins, and waypoint-reference versus gate-progress MPPI trajectories.

The Adap-RPF-lite example uses MPPI's moving-obstacle and goal-trajectory
extensions to track predicted person-following goals while proactively avoiding
predicted pedestrian motion. See `docs/adap_rpf_mppi_reproduction.md`.

## 3-D Gate Racing with Lap Progress

The racing objective extends from 2-D oriented gates to full 3-D in
`racing_mppi_3d`:

- `RacingGatePlane3D` is a rectangular gate aperture in 3-D, built from a center,
  a race-direction normal, and an in-plane up hint that is orthonormalized into
  `right`/`up` axes. A rollout passes the gate when a segment crosses the plane
  from behind to in front and the crossing point lies inside the rectangle, so
  gates can be tilted and stacked at any height.
- `RacingDroneDynamics3D` is a point-mass drone with linear aerodynamic drag, an
  optional gravity term, a speed cap, and an acceleration-magnitude cap — richer
  than the pure double integrator, so a draggier, lower-top-speed drone is
  honestly slower around the same course.
- `RacingGateLap3D` arranges gates into an open course or a closed lap; closed
  laps wrap the active gate modulo the gate count so the drone can fly repeated
  laps.
- `simulate_lap_race` drives a seeded, deterministic MPPI controller around the
  lap and returns `RacingLapReport3D`: laps completed, the fraction of the
  current lap, first-lap time, mean and peak speed, executed path length, mean
  control effort, and the minimum aperture margin at any gate crossing (1 at the
  gate center, 0 at the aperture edge, slightly negative when the drone clips a
  frame within the crossing tolerance).

The benchmark sweeps a planar closed square (two laps), an undulating closed
square where the drone climbs and descends each lap, an open five-gate ascending
helix, and a high-drag agile slalom, writing `docs/assets/racing-mppi-3d.csv`
and `.svg`:

```bash
cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control
```

On the bundled courses the reference-free controller completes two laps of both
square courses, climbs the full helix, and threads the slalom, with the draggier
slalom drone reaching a lower mean speed — the lap-progress, speed, and aperture
trade-offs the 3-D counters expose.

## Quadrotor Attitude Model

`racing_mppi_quadrotor` replaces the point-mass drone with a full quadrotor
whose orientation matters. The control input is the standard agile-racing
low-level abstraction used by differential-flatness controllers: a mass-
normalized collective thrust along the body z-axis plus three body rates.

- `QuadrotorState` carries position, velocity, and a unit-quaternion attitude.
- `QuadrotorParams` integrates the translational dynamics
  (`a = thrust * body_z - gravity - drag * v`, with a speed cap) together with
  quaternion attitude kinematics driven by the commanded body rates, then
  renormalizes the quaternion each step.
- `QuadrotorMppiController` samples thrust/body-rate perturbations around a hover
  nominal and scores the resulting positions with the same reference-free
  gate-progress objective from `racing_mppi_3d`. Because horizontal motion can
  only come from tilting the thrust vector, the position objective drives the
  attitude: the drone learns to pitch and roll toward the next gate. A small
  level regularizer keeps it from flipping under noise.
- `simulate_quadrotor_race` returns `QuadrotorLapReport`, adding tilt angle and
  body-rate effort to the lap-progress metrics.

The benchmark flies a planar slalom, an ascending course, a closed square lap,
and a heavier high-gravity drone, writing `docs/assets/racing-quadrotor.csv` and
`.svg`:

```bash
cargo run -p rust_robotics --example benchmark_racing_quadrotor --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_motor --no-default-features --features control
```

All four courses finish, including a full closed lap flown purely through
attitude control (peak tilt around 69 degrees on the square corners). The heavy,
high-gravity drone has to reach a noticeably larger peak tilt to thread the same
slalom gates — the attitude-coupling the quadrotor model exposes that the
point-mass model cannot.

## Motor-Level Rotor-Mixing Model

`racing_mppi_motor` deepens the attitude model one more level: the control input
is the four rotor thrusts, not body rates.

- A rotor-mixing map turns the four thrusts into a collective thrust and roll/
  pitch/yaw torques (`MotorQuadParams`); the body angular velocity is now a
  *state* driven by those torques with light aerodynamic rate damping, and every
  rotor saturates at `max_rotor_thrust`.
- Because a rotor cannot supply maximum lift and a large differential torque at
  once, demanding aggressive attitude changes steals thrust authority. The MPPI
  samples per-rotor perturbations around an even hover command and scores the
  reference-free gate progress as before.
- `simulate_motor_race` returns `MotorRacingLapReport`, adding the rotor
  saturation fraction to the lap-progress and attitude metrics.

The benchmark contrasts an agile drone against a thrust-limited one on the same
slalom, plus a climb and a closed square lap, writing `docs/assets/racing-motor.csv`
and `.svg`:

```bash
cargo run -p rust_robotics --example benchmark_racing_motor --no-default-features --features control
```

The agile drone completes the slalom, while the thrust-limited drone (rotors
topping out near 1.6 g) saturates far more often (around 57% of steps versus
44%), flies slower, and falls short of the last gate — the thrust/torque
trade-off that the body-rate model, which commands rates for free, cannot show.

## Motor-Lag and Battery-Sag Powertrain

`racing_mppi_powertrain` inserts a *powertrain* between the commanded rotor
thrusts and the rigid-body physics, modelling two effects every real racing quad
has and that the planner does not know about:

- **First-order motor lag.** A rotor cannot change thrust instantly; the actual
  thrust tracks the command through a spin-up lag with time constant `motor_tau`
  (`actual += (target - actual) * (1 - exp(-dt / tau))`). Aggressive attitude
  flicks are smeared in time.
- **Battery sag.** The per-rotor thrust ceiling is not constant. Open-circuit
  voltage falls linearly with state of charge from full to `min_voltage_scale`,
  instantaneous sag subtracts `sag_coeff * load` on top of that, and the pack
  discharges in proportion to load. A drone that flies hard early has less
  authority later.

The layer is built by *composition*: it reuses `MotorQuadParams::step` verbatim
for the translational and rotational physics and only adds the actuator
front-end (the lagged rotor thrusts plus the battery state). With
`PowertrainParams::ideal` — zero lag, no discharge, no sag — it reduces exactly
to the motor-level model, which is the benchmark's baseline.

`simulate_powertrain_race` drives the *powertrain-unaware* `MotorMppiController`
(which still plans assuming ideal actuators) through the real powertrain, so the
benchmark measures how much an idealized plan costs against a laggy, draining
one. The benchmark flies the same seeded controller down the same slalom under
four powertrains, writing `docs/assets/racing-powertrain.csv` and `.svg`:

```bash
cargo run -p rust_robotics --example benchmark_racing_powertrain --no-default-features --features control
```

On the bundled slalom the `ideal` and `motor-lag` powertrains both thread all
four gates (lag alone barely changes a short course). A fresh `lag+battery` pack
also finishes, but its commands clip the battery ceiling on roughly three out of
four steps. A `drained-pack` starting at 25% state of charge runs at a far lower
terminal voltage, saturates that lowered ceiling on nearly every step, drains to
empty, and stalls after a single gate — the same plan the fresh drone flew
cleanly. That gap is the lag/sag cost a planner that assumes ideal actuators
silently pays.

### Powertrain-Aware Controller

`PowertrainMppiController` closes that gap: instead of rolling candidate commands
out through the ideal motor model, it rolls them out through the full
`PowertrainParams::step`, so its samples already feel the motor lag and the
battery-limited thrust ceiling. Commands above the (causal) ceiling are clamped
away inside the rollout, so over-commanding earns no gate progress while still
paying the rotor-effort penalty — the controller is pushed to plan within the
authority the pack can actually deliver, and it sees the charge drain over the
horizon rather than assuming an infinite pack. On an ideal powertrain it reduces
to the motor controller (a unit test checks the two plan the same command).

`simulate_powertrain_race_aware` runs it through the same closed loop, and
`benchmark_racing_powertrain_aware` pits aware against unaware on one slalom flown
through the same lagging, sagging powertrain at two states of charge:

```bash
cargo run -p rust_robotics --example benchmark_racing_powertrain_aware --no-default-features --features control
```

On a fresh full pack both controllers thread all four gates (the aware one a
little faster, since planning through the lag lets it commit to a cleaner line).
On a pack drained to 25% the difference is decisive: the unaware controller keeps
commanding thrust the battery cannot supply, stalls after a single gate, and
drains the pack to about 2%; the aware controller budgets within the lowered
ceiling, threads all four gates, and still finishes with roughly 18% charge in
reserve. Modelling the powertrain in the rollout turns a one-gate failure into a
completed lap with energy to spare.

### Charge Budget

A [`ChargeBudget`] adds an explicit energy term to the aware controller: once a
rollout step's state of charge falls below a protected `reserve`, the cost gains
`weight * (reserve - soc) * load`. Because the penalty scales with the electrical
load (current draw), it gives the rollout an actionable gradient — throttle back
when the pack runs low — that a slow-moving state-of-charge penalty cannot.
`simulate_powertrain_race_budgeted` runs it; a zero-weight budget recovers the
plain aware controller (unit-tested).

`benchmark_racing_powertrain_budget` sweeps the budget weight over a draining
multi-lap square through the same powertrain:

```bash
cargo run -p rust_robotics --example benchmark_racing_powertrain_budget --no-default-features --features control
```

The sweep traces one Pareto frontier: with no budget the controller flies hard
(mean speed about 2.9 m/s), completes the most lap progress (about 3.5 laps), and
drains the pack to roughly 1%; as the weight rises it eases off below the
reserve, slows (down to about 2.3 m/s), completes fewer laps, and ends with
progressively more charge in reserve (up to about 7%). The honest takeaway is
that on a hover-dominated quad with no regeneration, pacing buys reserve rather
than extra laps — covering distance faster is the more energy-efficient way to
spend a fixed pack, so the budget is a knob for *where on that trade-off to sit*
rather than a free win.

### Battery Recovery

`PowertrainParams::with_recovery` adds a relaxation-overpotential state to the
pack. Real cells do not just sag instantaneously with current; a slower
overpotential builds under sustained load and relaxes when the load eases, so the
terminal voltage recovers during a rest even though no charge returns. The state
`relaxation` builds toward the load at `relax_build`, decays at `relax_recover`,
and depresses the terminal voltage by `relax_coeff` per unit.
`terminal_voltage_scale` is the open-circuit-minus-sag `voltage_scale` less that
relaxation term; with recovery off it is exactly `voltage_scale`, so all earlier
results are unchanged.

`benchmark_racing_powertrain_recovery` is a dynamics-only demo (no MPPI): it
drives the powertrain through a scripted hard/rest/hard/rest load profile and
compares the terminal voltage with and without the recovery model:

```bash
cargo run -p rust_robotics --example benchmark_racing_powertrain_recovery --no-default-features --features control
```

On the bundled profile the relaxation overpotential climbs to about 0.8 by the
end of the first full-throttle phase, dragging the recovery model's terminal
voltage well below the no-recovery trace. Across the following hover phase the
overpotential decays and the terminal voltage climbs back (about 0.585 to 0.614
on the bundled run) while the state of charge keeps falling monotonically. That
recovery is the lever the charge budget needs to eventually buy laps rather than
only reserve: a paced pack that rests between bursts regains ceiling a hard-flown
pack never gets back.
