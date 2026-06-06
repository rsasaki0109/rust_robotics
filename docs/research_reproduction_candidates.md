# Reproduction Candidate Paper Survey

Checked: 2026-06-04

Scope: robotics papers that are useful reproduction targets for this workspace.
Priority is given to algorithms that can become deterministic RustRobotics
library code, examples, benchmarks, or SVG gallery artifacts. ROS2-only work is
kept out of the immediate queue unless it has a pure-Rust slice.

Important: "code coming soon" means the official project page, author page, or
arXiv abstract indicated no usable implementation at this check time. This is
not a claim that no unofficial implementation exists.

## What We Already Started

These were high-priority candidates from the first survey and now have pure-Rust
reproduction slices in this workspace.

| Paper | Implemented slice | Current artifact |
| --- | --- | --- |
| Time-aware Motion Planning in Dynamic Environments with Conformal Prediction | CP-SIPP style conformal safe-interval planner | `conformal_sipp.rs`, headless demo, benchmark, docs |
| TRG-planner: Traversal Risk Graph-Based Path Planning in Unstructured Environments | Terrain risk graph planner, elevation risk, smoothing, clearance exposure, SVG/CSV sweep | `traversal_risk_graph.rs`, demos, docs assets |
| Adaptive Costmap-based Path Planning in Partially Known Environments with Movable Obstacles | Pure-Rust adaptive movable-obstacle costmap and replanning demo | `adaptive_costmap_namo.rs`, headless demo, docs |
| TD-CD-MPPI | MPPI baseline, constraint discounting, terminal value grid, online/replay value learning, adaptive lambda, track-progress terminal values, SVGs | `mppi.rs`, multiple demos, `docs/assets/mppi-*.svg` |
| Rethinking Reference Trajectories in Agile Drone Racing | Reference-free gate-progress objective, oriented gate crossing logic, waypoint-reference comparison, SVG; 3-D gate planes, drag-limited drone dynamics, open/closed laps, lap-progress metrics, CSV/SVG; full quadrotor attitude model (collective thrust + body rates) with tilt/body-rate metrics; motor-level four-rotor model with rotor saturation | `MppiGateRace2D`, `racing_mppi_3d.rs`, `racing_mppi_quadrotor.rs`, `racing_mppi_motor.rs`, `headless_mppi_racing_gate_progress.rs`, `benchmark_racing_mppi_3d.rs`, `benchmark_racing_quadrotor.rs`, `benchmark_racing_motor.rs`, `docs/assets/mppi-racing-gate-progress.svg`, `docs/assets/racing-mppi-3d.svg`, `docs/assets/racing-quadrotor.svg`, `docs/assets/racing-motor.svg` |
| Adap-RPF | Target-centric following-point sampling, visibility/proximity/distance/travel/stickiness scoring, prediction-aware MPPI with moving pedestrians, SVG | `person_following_mppi.rs`, `headless_adap_rpf_mppi.rs`, `docs/assets/adap-rpf-lite-mppi.svg` |
| BranchOut | Lane-level multimodal driving scene, compact mode/GMM-style trajectory head, distributional metrics, SVG | `branchout_multimodal.rs`, `headless_branchout_multimodal_driving.rs`, `docs/assets/branchout-multimodal-driving.svg` |
| Multi-Robot Trajectory Planning via cBOT and STL-KCBS | Grid CBS, vertex/edge constraints, STL robustness, heading/time kinodynamic search, continuous-time pairwise occupancy checks, SVGs | `stl_cbs.rs`, `kinodynamic_stl_cbs.rs`, `docs/assets/stl-cbs-multi-robot.svg`, `docs/assets/kinodynamic-stl-cbs.svg` |
| Hierarchical Large-Scale Multi-Robot Path / Trajectory Replanning | Region hierarchy over grid MAPF, affected-agent CBS group repair, 50/100/200-agent scale benchmark, CSV/SVG | `hierarchical_mapf.rs`, `benchmark_hierarchical_mapf_scale.rs`, `docs/assets/hierarchical-mapf-scale.svg` |
| Rigid Body Path Planning using Mixed-Integer Linear Programming | Rectangular rigid body over discretized SE(2), convex polygon half-space obstacles, pose/segment binary-style separation certificates, narrow-slot SVG; backend trait with lattice A*, sampling RRT, and an exact length-optimal branch-and-bound backend; backend-comparison benchmark | `rigid_body_mip.rs`, `headless_rigid_body_mip_planning.rs`, `benchmark_rigid_body_backends.rs`, `docs/assets/rigid-body-mip-planning.svg`, `docs/assets/rigid-body-backend-benchmark.svg` |
| SafeDec: Constrained Decoding for Safe Robot Navigation Policies | STL-shielded constrained beam search over a greedy grid policy (always-avoid geofence + eventually-reach), greedy-vs-shielded comparison, SVG | `safe_decode_nav.rs`, `render_safe_decode_nav_svg.rs`, `docs/assets/safe-decode-nav.svg` |
| PolyMerge: polytope-covering safety / CBF filtering | Convex-polygon obstacle cover, true point-to-polygon CBF barrier, exact 2-D active-set CBF-QP velocity filter, raw-vs-filtered benchmark, CSV/SVG | `cbf_safety_filter.rs`, `benchmark_cbf_safety_filter.rs`, `docs/assets/cbf-safety-filter.svg` |
| Long Range Navigator | Occlusion-aware sensing, frontier extraction/clustering, affordance scoring (goal/cost/line-of-sight/openness), Dijkstra local-planner handoff, SVG | `frontier_navigator.rs`, `render_frontier_navigator_svg.rs`, `docs/assets/frontier-navigator.svg` |

## Recommended Implementation Order

This is the current "do next" order. It favors high visual payoff and strong
reuse of the MPPI/planning modules already added.

1. Rethinking Reference Trajectories in Agile Drone Racing
   - Source: https://zhaofangguo.github.io/racing_mppi/
   - Status observed: `Code (coming soon)`; initial pure-Rust slice implemented.
   - Implemented slice: 2-D gate course, oriented gate-crossing reward,
     reference-free MPPI objective, waypoint-reference comparison, SVG visual;
     3-D gate planes, drag-limited point-mass dynamics, open/closed laps and
     lap-progress metrics; full quadrotor attitude model (collective thrust +
     body rates) whose orientation is driven by the gate-progress objective;
     motor-level four-rotor rotor-mixing model with rotor saturation; a
     motor-lag + battery-sag powertrain layer (`racing_mppi_powertrain.rs`,
     `benchmark_racing_powertrain.rs`) that degrades an idealized plan; and a
     powertrain-aware controller (`PowertrainMppiController`,
     `benchmark_racing_powertrain_aware.rs`) that rolls out through the lag/
     battery model and finishes a drained-pack course the unaware one stalls on;
     a charge-budget term (`ChargeBudget`, `benchmark_racing_powertrain_budget.rs`)
     tracing the endurance/progress Pareto frontier; and a battery-recovery model
     (`PowertrainParams::with_recovery`, `benchmark_racing_powertrain_recovery.rs`)
     whose terminal voltage relaxes back on ease-off as charge keeps falling; and
     a budget x recovery capstone (`benchmark_racing_powertrain_endurance.rs`)
     where recovery reverses the budget from a lap loss into a Pareto win.
   - Remaining extensions: a low-hover-overhead platform or true-idle-rest course
     where pacing wins laps outright, and a per-rotor torque/current map so yaw
     authority sags separately from thrust.

2. Adap-RPF: Adaptive Trajectory Sampling for Robot Person Following
   - Source: https://adap-rpf.github.io/
   - Status observed: `Code (Coming soon)`; initial pure-Rust slice implemented.
   - Implemented slice: moving target and pedestrians, following-point candidate
     sampler, occlusion/proximity/distance/travel/stickiness scoring,
     horizon goal trajectories, prediction-aware MPPI tracking demo.
   - Remaining extensions: Sobol sequence proper, richer human prediction,
     unicycle dynamics, and benchmark-style target-visibility metrics.

3. BranchOut: Capturing Realistic Multimodality in Autonomous Driving Decisions
   - Source: https://branchoutcorl.github.io/
   - Status observed: `Code (coming soon)`; initial pure-Rust slice implemented.
   - Implemented slice: lane-level driving scene, keep/yield/lane-change
     trajectory modes, GMM-style mixture probabilities, pairwise diversity,
     Frechet coverage, NLL, speed JSD, expected route completion, SVG.
   - Remaining extensions: richer interaction scenarios, closed-loop rollout
     counters, and learned/optimization-based mode proposal.

4. Multi-Robot Trajectory Planning via cBOT and STL-KCBS
   - Source: https://arxiv.org/abs/2603.05767
   - Status observed: arXiv says the planner will be released as OSS; pure-Rust
     STL-CBS and kinodynamic-oriented slices implemented.
   - Implemented slice: grid CBS, time-indexed vertex/edge constraints,
     independent conflict baseline, STL eventually-reach, always-avoid, and
     pairwise-separation robustness, heading-aware `(x, y, heading, t)`
     low-level search, time-consuming forward/turn/wait primitives, terminal
     heading constraints, exact closest-approach continuous pairwise occupancy
     checks between integer ticks, SVGs.
   - Remaining extensions: use continuous conflicts as CBS branching
     constraints, cBOT-style cost-map learning hook, and richer STL task
     specifications.

5. Hierarchical Large-Scale Multi-Robot Path / Trajectory Replanning
   - Source: https://www.panlishuo.com/
   - Status observed: author page lists `Code (Coming Soon!)`; initial
     pure-Rust slice and scale benchmark implemented.
   - Implemented slice: grid MAPF baseline, coarse rectangular regions,
     time-indexed region conflicts, connected affected-agent grouping, CBS
     group repair, full-replan fallback guard, SVG comparison, 50/100/200-agent
     corridor-swap scale benchmark with local repair groups of size 2.
   - Remaining extensions: randomized dense-agent benchmark, region-size sweep,
     dynamic task insertions, hierarchical corridor reservation, and
     trajectory-duration costs.

6. Rigid Body Path Planning using Mixed-Integer Linear Programming
   - Source: https://mingxiny.github.io/
   - Status observed: author page lists `Code (Coming Soon!)`; initial
     pure-Rust slice implemented.
   - Implemented slice: 2D rectangular rigid body, convex polygon/AABB
     obstacles as half-spaces, SE(2) lattice search, one active
     obstacle-separating half-space certificate per obstacle and pose,
     sampled segment-level certificates, narrow-slot demo and SVG.
   - Remaining extensions: continuous swept-volume certificates and a
     heading-constrained variant of the exact backend (the backend trait, an
     exact length-optimal branch-and-bound backend, and the sampling-planner
     comparison benchmark are implemented).

7. Long Range Navigator
   - Source: https://personalrobotics.github.io/lrn/
   - Status observed: `Code (Coming Soon)` and `Data (Coming Soon)`.
   - Why next: extends local maps into long-horizon frontier decision making.
   - Minimum slice: synthetic frontier graph, affordance scores, local planner
     handoff, compare frontier choices under occlusion.

8. SafeDec: Constrained Decoding for Safe Robot Navigation Policies
   - Source: https://constrained-robot-fms.github.io/
   - Status observed: `Code (coming soon)`.
   - Why next: good safety layer that can be reproduced without a VLM by
     shielding a discrete navigation policy with STL constraints.
   - Minimum slice: action decoder over grid/local planner choices, STL
     geofence/avoid/visit constraints, constrained beam search.

9. PolyMerge: 3D Gaussian Splat Compression with Polytope Coverings
   - Source: https://athlon76.github.io/PolyMerge-website/
   - Status observed: paper/arXiv and code marked coming soon.
   - Why next: geometric safety and CBF filtering are reusable even without 3DGS.
   - Minimum slice: 2D/3D obstacle point cloud to convex cover, cover-size sweep,
     CBF filter over nominal controls.

10. Push Anything: Single- and Multi-Object Pushing with Contact-Implicit MPC
    - Source: https://dairlab.github.io/push-anything/
    - Status observed: project page links an implementation in DAIRLab/dairlib
      on the `push_anything_dev` branch; initial pure-Rust slice implemented.
    - Why next: high-payoff manipulation/control target, but heavier than MPPI.
    - Implemented slice: quasi-static pusher-slider with the ellipsoidal limit
      surface, single back-face point contact with stick/slide contact modes, and
      a seeded MPPI pusher that translates/steers/reorients a square slider to
      goal poses within ~1 cm (`pusher_slider.rs`, `benchmark_pusher_slider.rs`,
      `docs/pusher_slider_reproduction.md`). First manipulation target in the repo.
    - Remaining extensions: face-switching / second contact for
      rotation-without-lateral goals, multi-object pushing, and contact-implicit
      MPC over discrete contact modes.

## Expanded Candidate Pool

### Planning, Navigation, and Control

| Priority | Paper / project | Status observed | Fit | First reproduction slice |
| --- | --- | --- | --- | --- |
| A | Rethinking Reference Trajectories in Agile Drone Racing | Code coming soon | Direct continuation of MPPI track progress | Gate progress reward and reference-free MPPI on simplified race tracks |
| A | Adap-RPF | Initial slice implemented; official code coming soon | MPPI + dynamic agents/person following | Add visibility/spacing benchmark metrics after current sampler + MPPI demo |
| A | BranchOut | Initial slice implemented; official code coming soon | Multimodal trajectory planning metrics | Add richer traffic interactions and closed-loop route-completion benchmark |
| A | cBOT + STL-KCBS | STL-CBS plus kinodynamic/continuous-occupancy slice implemented; OSS release planned | Formal multi-robot planning | Branch on continuous conflicts, add richer STL specs, and cBOT-style local cost-map learner |
| A | Hierarchical Multi-Robot Path / Trajectory Replanning | Initial slice plus scale benchmark implemented; official code coming soon | MAPF/swarm planning gap | Add region-size sweeps and randomized dense-agent benchmarks |
| A- | Rigid Body MILP Planning | Convex-polygon and segment-certificate slice implemented; official code coming soon | Optimization planning gap | Add exact MILP backend trait and swept-volume benchmarks |
| A- | SafeDec | Code coming soon | Safety layer for navigation policies | STL-constrained action decoding/shielding |
| A- | Long Range Navigator | Code/data coming soon | Frontier planning beyond local maps | Affordance frontier graph planner |
| A- | PolyMerge | Code coming soon | Geometry + CBF safety | Convex obstacle covers and CBF filter |
| B+ | EARTH Excavation Autonomy | Code coming soon | Traversability/terrain handling | Excavation terrain grid with traversability and material-handling state |
| B+ | ACLM distributed MPC | Code coming soon | Distributed MPC and payload transport | ADMM consensus MPC for simplified 2D payload |
| B+ | Meta-Control | Code coming soon | Controller selection/synthesis | Task requirement parser + controller mode selector |
| B+ | Deep Reactive Policy | Code coming soon | Reactive local planning | Point-obstacle reactive goal proposal module in joint/SE2 space |
| B | STAMP | Code coming soon | Task and motion planning | SVGD particle optimizer for a toy geometric TAMP task |
| B | CLIMB | Code coming soon | Mission/task planning | PDDL predicate refinement from failed plans in blocksworld |
| B | PACR | Code coming soon | Constraint-based manipulation | Point-axis geometric constraint scoring for planar manipulation |
| B | Grasp-MPC | Code coming soon | Value-guided MPC | 2D grasp-affordance value grid + MPC/MPPI scoring |
| B | DMMP / MMFP motion manifold primitives | Code coming soon | Motion generation | Basis/manifold trajectory primitives for 2D/SE2/arm toy tasks |

### Mapping, SLAM, and Perception-Heavy Targets

These are useful to track but should not outrank planning/control targets unless
we intentionally move into learned perception.

| Priority | Paper / project | Status observed | Fit | First reproduction slice |
| --- | --- | --- | --- | --- |
| B | ACE-SLAM | Code coming soon | SLAM, but neural RGB-D heavy | Synthetic scene-coordinate regression + pose alignment toy |
| B | HS-SLAM | Code coming soon | Dense SLAM, neural representation heavy | Hybrid grid/tri-plane map toy with tracking residuals |
| B | STRIVE | Code coming soon | Object navigation, graph planning | Room-object-viewpoint graph planner without VLM calls |
| B- | GaussianFormer3D | Code coming soon | Semantic occupancy, learned fusion heavy | Gaussian occupancy grid abstraction and memory/IoU benchmark |
| B- | WideDepth | Code/dataset coming soon | Dataset/perception more than planner | Fisheye projection/depth conversion utilities if needed |

### Manipulation / Learning-Heavy Watchlist

These are interesting but higher dependency risk for this repo.

| Priority | Paper / project | Status observed | Fit | First reproduction slice |
| --- | --- | --- | --- | --- |
| B | Push Anything | Code available in DAIRLab/dairlib branch | Contact MPC and manipulation | 2D contact-mode MPC for pushing blocks |
| B | ADCS bimanual constrained diffusion | Code coming soon | Constraint sampling | Non-learned constrained SE(2)/SE(3) sampler baseline |
| B | GLIDE planning-guided diffusion policy | Code coming soon | Contact-rich bimanual manipulation | Planner-generated synthetic demos in a toy contact task |
| B | SPARTA visual spatial progress | Code coming soon | Dense progress reward | Non-visual spatial progress reward for deformable/continuous state toy |
| B- | ROPA | Code coming soon | Data augmentation more than core robotics | Synthetic robot pose augmentation utility if vision stack grows |
| C+ | CHIP | Code coming soon | Humanoid compliance, heavy sim | Spring/damper compliance mode selector in 2D payload task |
| C+ | HYPERmotion | Code coming soon | Humanoid behavior planning, heavy | Hybrid behavior graph planner only |

## Exclusions / Lower Immediate Priority

- ROS2-only integrations are postponed unless they have a standalone Rust
  algorithmic core.
- Learned RGB-D/NeRF/Gaussian-splat SLAM targets are watchlist items, not first
  queue items, because they require GPU training/data infrastructure.
- Manipulation diffusion/VLA papers are deferred unless we reproduce only a
  small planner, cost, or constraint submodule.

## Next Concrete Build Queue

The first build queue is fully landed (2026-06-04/05):

1. ~~Extend hierarchical MAPF benchmarking.~~ Done — region-size and density
   sweeps with a bounded flat-CBS baseline.
2. ~~Extend rigid-body MIP planning.~~ Done — backend trait plus an RRT backend
   and comparison benchmark (exact MILP backend still pending; see v2).
3. ~~Extend STL-CBS continuous conflict handling.~~ Done — continuous conflicts
   promoted into CBS branching constraints.
4. ~~Extend racing MPPI to 3-D gates.~~ Done — 3-D gate planes, drag-limited
   dynamics, lap-progress metrics, and a full quadrotor attitude model.
5. ~~Extend Adap-RPF-lite metrics.~~ Done — visibility and spacing ratio sweep.
6. ~~Extend BranchOut-lite closed-loop metrics.~~ Done — receding-horizon route
   completion, no-collision, comfort, and TTC counters.

### Queue v2 (2026-06-05): breadth-first new papers + depth extensions

1. ~~SafeDec-lite: STL-shielded discrete navigation decoding~~ — Done
   (`safe_decode_nav.rs`, `render_safe_decode_nav_svg.rs`).
2. ~~CBF safety filter (PolyMerge-lite): convex-polytope obstacle covers plus a
   control-barrier-function filter over nominal controls~~ — Done
   (`cbf_safety_filter.rs`, `benchmark_cbf_safety_filter.rs`).
3. ~~Long Range Navigator-lite: affordance-scored frontier graph with
   occlusion-aware selection and a local-planner handoff~~ — Done
   (`frontier_navigator.rs`, `render_frontier_navigator_svg.rs`).
4. ~~Hierarchical MAPF anisotropic region sweep (`region_width != region_height`)
   plus a solvable fallback-regime data point~~ — Done (anisotropic +
   fallback-rate sweeps in `benchmark_hierarchical_mapf_sweeps.rs`).
5. ~~Rigid-body exact MILP backend behind `RigidBodyPlanningBackend`, benchmarked
   for path optimality against the lattice/RRT backends~~ — Done
   (`RigidBodyExactBackend2D`, length-optimal branch-and-bound).
6. ~~Racing motor-level model: thrust-and-torque quadrotor with a rotor mixing
   matrix so motor saturation enters the trade-off~~ — Done
   (`racing_mppi_motor.rs`, `benchmark_racing_motor.rs`).

All six v2 queue items are landed.

### Follow-on depth extensions (post-v2)

1. ~~Motor-lag + battery-sag powertrain layered on the rotor model so an
   idealized plan degrades when it meets a laggy, draining powertrain~~ — Done
   (2026-06-07; `racing_mppi_powertrain.rs`, `benchmark_racing_powertrain.rs`).
2. ~~Powertrain-aware controller that rolls candidates out through the lag/battery
   model and conserves authority for later gates~~ — Done (2026-06-07;
   `PowertrainMppiController`, `benchmark_racing_powertrain_aware.rs`).
3. ~~Charge-budget term for the aware controller (multi-lap energy/progress
   trade-off)~~ — Done (2026-06-07; `ChargeBudget`,
   `benchmark_racing_powertrain_budget.rs`; traces the endurance/progress
   frontier — honest finding: pacing buys reserve, not extra laps, on a
   hover-dominated quad without regeneration).
4. ~~Battery-recovery model so easing off partly restores terminal voltage~~ —
   Done (2026-06-07; `PowertrainParams::with_recovery`, `terminal_voltage_scale`,
   `benchmark_racing_powertrain_recovery.rs`; relaxation overpotential that
   recovers on ease-off while state of charge keeps falling).
5. ~~Combine the charge budget with recovery on a multi-lap race~~ — Done
   (2026-06-07; `benchmark_racing_powertrain_endurance.rs`, the budget x recovery
   2x2). Honest finding: recovery reverses the budget's sign (lap loss -> Pareto
   win), but a strict more-laps flip needs low hover overhead or true idle rests.
