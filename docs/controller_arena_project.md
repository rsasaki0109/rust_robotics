# Controller Arena Project

Status: implemented and verified on `feature/controller-arena`

Decision date: 2026-07-28

Owner: maintainer

Target: first development slice after the `0.2.0` publish gate

## Decision

Build an interactive **Controller Arena** in the RustRobotics Playground. It
runs Pure Pursuit, Stanley, and LQR Steer against the same path, initial state,
simulation clock, and actuation model, then displays their trajectories and
metrics side by side.

This is the selected development project. Publishing `0.2.0`, tagging it, and
creating the GitHub Release remain the operational P0 gate in `plan.md`; local
development may start before that gate, but the arena must not displace the
release or its announcement.

## Why This Project

RustRobotics already contains the three controllers, a shared `PathTracker`
contract, repeatable path-tracking experiments, and a browser Playground. The
missing piece is a user-facing comparison that connects those assets.

The project turns an existing strength—many controllers under one Rust
workspace—into a concrete answer to a common user question: **which controller
should I try for this path and disturbance?**

### Candidate evaluation

Scores use a 1–5 scale. Higher is better. The weighted total is out of 5.

| Candidate | User value 30% | Differentiation 25% | Reuse 20% | Delivery ease 15% | Strategy fit 10% | Total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Controller Arena | 5 | 5 | 4 | 4 | 5 | **4.65** |
| Five-minute navigation guide | 5 | 3 | 5 | 5 | 5 | 4.50 |
| Pusher-slider Playground tab | 4 | 5 | 4 | 3 | 3 | 4.00 |
| Benchmark regression gate | 4 | 4 | 4 | 3 | 5 | 3.95 |
| Embedded EKF reference | 4 | 5 | 3 | 2 | 5 | 3.85 |

The five-minute guide remains the best follow-up adoption task. The embedded
demo and benchmark gate remain important Phase 3 work, but they offer less
immediate interactive value and have larger environment or CI dependencies.
The existing Grid Planners tab was not scored because it already provides the
proposed browser planner comparison.

## Objective

Enable a visitor to select a path scenario, run three classical path trackers
under identical conditions, and understand their trade-offs from motion and
metrics without writing code.

The engineering objective is equally important: create one reusable,
deterministic, headless comparison engine so the Playground, tests, and future
benchmark gate do not implement separate simulation semantics.

## Target Users

1. A robotics learner choosing between geometric and state-feedback tracking.
2. A Rust developer evaluating whether `rust_robotics_control` fits a prototype.
3. A contributor adding or tuning a controller and needing a repeatable visual
   comparison.
4. The maintainer, who needs an explainable demo and a future regression input.

## User Story

As a visitor, I select a straight, slalom, or hairpin path, adjust speed or
actuation mismatch, press **Run**, and see all controllers follow the same path.
I can inspect cross-track error, final goal error, and control smoothness, then
copy a link that reopens the same scenario.

## Version 1 Scope

### In scope

- Pure Pursuit, Stanley, and LQR Steer through the existing `PathTracker`
  contract.
- Three deterministic presets: straight recovery, slalom recovery, and
  hairpin/tight-turn recovery.
- A shared initial pose and simulation clock for every controller.
- Adjustable target speed and one bounded disturbance control:
  steering/turn-rate response gain.
- Simultaneous path, robot-pose, and trajectory rendering.
- Per-controller metrics:
  - cross-track RMSE;
  - final goal distance;
  - maximum absolute cross-track error;
  - control smoothness as RMS change in angular command.
- Run, pause, reset, and single-step controls.
- A shareable URL preserving the tab, preset, speed, and disturbance setting.
- Deterministic headless tests for traces and metrics.
- Playground documentation plus one screenshot or short GIF for the gallery.

### Out of scope

- Automatic controller selection or mid-run switching.
- Declaring one controller universally best.
- MPPI, MPC, and controllers that require a different state/dynamics contract.
- Obstacle avoidance or replanning.
- User-drawn paths in version 1.
- Gain auto-tuning.
- A new public workspace-wide `Controller` trait.
- Benchmark CI failure thresholds; the arena only produces suitable inputs.

MPPI and automatic mode selection are possible version 2 work after a fair
comparison contract is defined. Keeping them out of version 1 avoids comparing
a point-mass stochastic optimizer against bicycle/unicycle trackers under
incompatible dynamics and compute budgets.

## Product Behavior

The Playground adds a **Controller Arena** tab with:

- a top control row for preset, target speed, response gain, and playback;
- a main plot showing the reference path and color-coded traces;
- a compact metrics table using the same colors;
- an explanation panel describing what each metric means;
- a copy-link action using the existing share-link mechanism.

All three controllers start from the same `State2D`. The headless engine owns
state propagation, command limits, metric accumulation, and termination.
Rendering only consumes snapshots from that engine.

## Technical Shape

### Control crate

Add a small reusable module, tentatively
`rust_robotics_control::controller_arena`, containing:

```rust
pub enum ArenaControllerKind {
    PurePursuit,
    Stanley,
    LqrSteer,
}

pub struct ArenaScenario { /* path, start, dt, speed, response gain */ }
pub struct ArenaSample { /* time, state, command, cross-track error */ }
pub struct ArenaMetrics { /* RMSE, final/max error, smoothness */ }
pub struct ArenaRun { /* controller kind, samples, metrics */ }

pub fn run_controller_arena(
    scenario: &ArenaScenario,
    controllers: &[ArenaControllerKind],
) -> RoboticsResult<Vec<ArenaRun>>;
```

Exact names may change during implementation, but the invariants may not:
identical scenario input, deterministic output, one propagation model, and
metrics derived from the returned samples.

Reuse the existing path builders or extract neutral preset builders from
`experiments/path_tracking_accuracy`. Do not make the Playground depend on
private experiment internals.

### Playground crate

Add `controller_arena.rs`, register the tab in `app.rs`, and extend share-query
parsing. The UI must not duplicate controller or simulation logic.

### Validation

- Unit tests: preset validity, deterministic repeat, finite metrics, metric
  recomputation from samples, and identical initial conditions.
- Behavior tests: each controller produces a non-empty trace for all presets;
  no NaN/Inf; command limits are respected.
- Share-query tests: round trip and invalid-value fallback.
- Build gates: native control/playground tests and release WASM build.

## Deliverables

1. Reusable deterministic arena engine in `rust_robotics_control`.
2. Controller Arena Playground tab.
3. Three built-in scenarios and three controller adapters.
4. Metrics table and explanatory copy.
5. Reproducible share links.
6. Unit/behavior/share-query tests.
7. Updated Playground README and root README entry.
8. Gallery screenshot or GIF and a short launch-ready description.

## Success Metrics

Version 1 is complete only when all of the following are true:

1. All three controllers run from identical initial conditions on all three
   presets and return finite traces and metrics.
2. Repeating a run with the same settings returns the same samples and metrics
   within a documented floating-point tolerance.
3. The displayed cross-track RMSE, final error, maximum error, and smoothness
   match values recomputed from the headless samples in tests.
4. A copied link restores the selected preset, target speed, and response gain.
5. `cargo test -p rust_robotics_control` and
   `cargo test -p rust_robotics_playground` pass.
6. The release WASM build completes using the command documented in the
   Playground README.
7. The UI stays responsive during default runs by precomputing a bounded trace
   and replaying it, rather than solving an unbounded simulation in the paint
   loop.
8. README documentation lets a new visitor open the tab and explain the three
   metrics without consulting source code.

Post-release adoption can additionally be measured with Playground visits,
shared scenario links, and issue/discussion feedback, but those analytics are
not an implementation completion gate.

## Milestones

### M0 — Release gate check (0.5 day)

- Reconfirm crates.io and GitHub Release state.
- Publish/tag `0.2.0` before announcing this feature or including it in release
  messaging.
- This milestone is owned by the existing release workflow and is not part of
  the arena code diff.

### M1 — Headless comparison engine (2–3 days)

- Define scenarios, results, samples, and controller adapters.
- Extract/reuse deterministic path presets.
- Implement propagation and metrics once.
- Add deterministic and invariant tests.

Exit: the three-by-three controller/scenario matrix passes headless tests.

### M2 — Interactive Playground tab (2–3 days)

- Render reference path, traces, and robot poses.
- Add playback and bounded parameter controls.
- Add the metrics table and explanations.
- Avoid computation in the rendering loop.

Exit: native Playground runs all presets without panic or non-finite output.

### M3 — Reproduction and documentation (1–2 days)

- Add share-query round trips and invalid-input fallbacks.
- Update Playground and root READMEs.
- Produce the gallery asset.
- Run native tests and the release WASM build.

Exit: every success metric has recorded command or test evidence.

Expected implementation effort: **5–8 maintainer days**, excluding the external
`0.2.0` publish action.

## Risks and Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Controller models or gains make the comparison unfair | Misleading results | Use the shared `PathTracker` contract, disclose presets/gains, and avoid universal winner language |
| LQR has higher per-step cost in WASM | UI stalls | Precompute bounded traces outside paint logic and cap steps |
| Existing experiment logic is private and score-only | Duplicate semantics | Extract a neutral trace-producing engine, then let experiments consume it later |
| Too many controls obscure the lesson | Weak onboarding | Ship only preset, speed, and one response-gain disturbance |
| URL parameters become unstable | Broken shared demos | Version/validate accepted values and fall back to defaults |
| Scope expands into Meta-Control | Delayed delivery | Keep switching, auto-selection, MPPI, and tuning in version 2 |

## Follow-up Options

After version 1 evidence exists:

1. Use the deterministic scenarios as inputs to the benchmark regression gate.
2. Add a five-minute tutorial that embeds one arena scenario.
3. Define a fair compute/dynamics contract for MPC or MPPI.
4. Build Meta-Control as an explicit selector and compare it against fixed
   controllers, rather than presenting selection without a baseline.

## Start Checklist

- [ ] Confirm the `0.2.0` publish gate owner and timing.
- [x] Open an implementation branch referencing this document.
- [x] Freeze version 1 controller and metric lists.
- [x] Implement M1 before adding UI code.
- [x] Record verification evidence against every success metric.

## Implementation Evidence

Verified on 2026-07-28:

- `rust_robotics_control::controller_arena` owns the shared scenarios,
  propagation, bounded commands, samples, and four metrics.
- The three-controller by three-preset matrix is tested for finite output,
  identical initial state, deterministic repeat, command limits, and metric
  recomputation.
- The Playground tab renders only precomputed traces and provides Run, Play /
  Pause, Reset, single-step, and timeline controls.
- Share-query round trips preserve preset, target speed, and turn response;
  malformed and out-of-range values fall back safely.
- `cargo test -p rust_robotics_control`: 295 unit tests passed, 29 explicitly
  ignored long-running cases, 10 integration tests passed, and 1 doc-test
  passed.
- `cargo test -p rust_robotics_playground`: 5 tests passed.
- `cargo clippy -p rust_robotics_control -p rust_robotics_playground
  --all-targets -- -D warnings`: passed.
- `RUSTFLAGS='--cfg getrandom_backend="wasm_js"' trunk build --release
  --public-url /rust_robotics/playground/`: passed.
- `render_controller_arena_svg` regenerates the gallery SVG from the headless
  engine; `docs/assets/controller-arena.png` was rendered and visually
  inspected.

The remaining unchecked item is the existing external `0.2.0` publish gate. It
does not block the local Controller Arena implementation, but it still gates
public announcement and deployment.
