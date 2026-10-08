const galleryItems = [
  {
    title: "Extended Kalman Filter",
    category: "Localization",
    image: "img/localization/ekf.svg",
    command: "cargo run --example ekf",
    description: "GPS, dead reckoning, and EKF estimates layered into one clean tracking visual.",
    size: "wide"
  },
  {
    title: "Particle Filter",
    category: "Localization",
    image: "img/localization/particle_filter_result.png",
    command: "cargo run --example particle_filter",
    description: "Particles and path estimates rendered against the ground-truth trajectory."
  },
  {
    title: "Unscented Kalman Filter",
    category: "Localization",
    image: "img/localization/ukf_result.png",
    command: "cargo run --example unscented_kalman_filter",
    description: "Sigma-point localization with uncertainty ellipses and observation overlays."
  },
  {
    title: "Histogram Filter",
    category: "Localization",
    image: "img/localization/histogram_filter.svg",
    command: "cargo run --example histogram_filter",
    description: "Grid-based localization with landmarks and probability mass you can read at a glance."
  },
  {
    title: "Cubature Kalman Filter",
    category: "Localization",
    image: "img/localization/ukf_result.png",
    command: "cargo test --lib cubature_kalman_filter",
    description: "Same accuracy as UKF, 30% faster, zero tuning parameters. The recommended default filter."
  },
  {
    title: "Ensemble Kalman Filter",
    category: "Localization",
    image: "img/localization/ekf.svg",
    command: "cargo test --lib ensemble_kalman_filter",
    description: "Stochastic ensemble-based filter with 50 particles for non-Gaussian state estimation."
  },
  {
    title: "Adaptive EKF/CKF Filter",
    category: "Localization",
    image: "img/localization/ekf.svg",
    command: "cargo test --lib adaptive_filter",
    description: "Auto-switches between EKF and CKF based on NIS. Fast in calm, robust in noise."
  },
  {
    title: "NDT Map",
    category: "Mapping",
    image: "img/mapping/ndt.svg",
    command: "cargo run --example ndt",
    description: "Normal distributions mapped over occupancy cells for a denser sense of structure."
  },
  {
    title: "Gaussian Grid Map",
    category: "Mapping",
    image: "img/mapping/gaussian_grid_map.svg",
    command: "cargo run --example gaussian_grid_map",
    description: "Obstacle probability fields rendered as a soft occupancy surface."
  },
  {
    title: "Ray Casting Grid Map",
    category: "Mapping",
    image: "img/mapping/ray_casting_grid_map.svg",
    command: "cargo run --example ray_casting_grid_map",
    description: "Classic free, occupied, and unknown cells from a simple ray-cast world model."
  },
  {
    title: "ICP Matching",
    category: "SLAM",
    image: "img/slam/icp_summary.png",
    command: "cargo run --example icp_matching",
    description: "Reference, initial, and aligned point clouds packed into one before-and-after frame.",
    size: "wide"
  },
  {
    title: "FastSLAM 1.0",
    category: "SLAM",
    image: "img/slam/fastslam1.svg",
    command: "cargo run --example fastslam1",
    description: "Particle-based SLAM with landmark estimates and pose history on the same plot."
  },
  {
    title: "FastSLAM 2.0",
    category: "SLAM",
    image: "img/slam/fastslam1.svg",
    command: "cargo test --lib fastslam2",
    description: "Improved proposal distribution over FastSLAM 1.0 for better particle diversity."
  },
  {
    title: "EKF SLAM",
    category: "SLAM",
    image: "img/slam/ekf_slam.svg",
    command: "cargo run --example ekf_slam",
    description: "Joint state estimation for pose and landmarks visualized in a single pass."
  },
  {
    title: "Graph-Based SLAM",
    category: "SLAM",
    image: "img/slam/graph_based_slam.svg",
    command: "cargo run --example graph_based_slam",
    description: "Pose graph optimization rendered as a path correction story instead of raw math."
  },
  {
    title: "A* Search",
    category: "Path Planning",
    image: "img/path_planning/a_star_result.png",
    command: "cargo run --example a_star",
    description: "A clean shortest-path shot that reads instantly in a social feed."
  },
  {
    title: "Traversal-Risk Graph",
    category: "Path Planning",
    image: "assets/traversal-risk-graph-demo.svg",
    command: "cargo run -p rust_robotics --example render_traversal_risk_graph_svg --no-default-features --features planning",
    description: "Terrain-risk heatmap comparing the short risky route against a longer safer detour.",
    size: "wide"
  },
  {
    title: "Traversal-Risk Pareto Sweep",
    category: "Path Planning",
    image: "assets/traversal-risk-weight-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_traversal_risk_sweep --no-default-features --features planning",
    description: "Risk-weight sweep chart for distance versus terrain exposure on elevation-derived terrain.",
    size: "wide"
  },
  {
    title: "BranchOut Multimodal Driving",
    category: "Path Planning",
    image: "assets/branchout-multimodal-driving.svg",
    command: "cargo run -p rust_robotics --example render_branchout_multimodal_driving_svg --no-default-features --features planning",
    description: "Multimodal driving decisions with yield and lane-change branches, mixture weights, and distribution metrics.",
    size: "wide"
  },
  {
    title: "BranchOut Closed-Loop Metrics",
    category: "Path Planning",
    image: "assets/branchout-closed-loop.svg",
    command: "cargo run -p rust_robotics --example benchmark_branchout_closed_loop --no-default-features --features planning",
    description: "Receding-horizon closed-loop driving across overtake, yield, lead-follow, and oncoming scenes: route completion, no-collision rate, comfort, and time-to-collision.",
    size: "wide"
  },
  {
    title: "STL-CBS Multi-Robot",
    category: "Path Planning",
    image: "assets/stl-cbs-multi-robot.svg",
    command: "cargo run -p rust_robotics --example render_stl_cbs_multi_robot_svg --no-default-features --features planning",
    description: "Conflict-based multi-robot plans with STL goal and separation robustness checks.",
    size: "wide"
  },
  {
    title: "Kinodynamic STL-CBS",
    category: "Path Planning",
    image: "assets/kinodynamic-stl-cbs.svg",
    command: "cargo run -p rust_robotics --example render_kinodynamic_stl_cbs_svg --no-default-features --features planning",
    description: "Heading-aware CBS with time-consuming primitives and continuous-time pairwise occupancy checks.",
    size: "wide"
  },
  {
    title: "SafeDec-lite STL Shield",
    category: "Path Planning",
    image: "assets/safe-decode-nav.svg",
    command: "cargo run -p rust_robotics --example render_safe_decode_nav_svg --no-default-features --features planning",
    description: "Constrained decoding for safe navigation: a greedy policy cuts through hazards while an STL always-avoid/eventually-reach shield reroutes it via deterministic beam search, with measurable robustness gain.",
    size: "wide"
  },
  {
    title: "Frontier Navigator (LRN-lite)",
    category: "Path Planning",
    image: "assets/frontier-navigator.svg",
    command: "cargo run -p rust_robotics --example render_frontier_navigator_svg --no-default-features --features planning",
    description: "Long Range Navigator-lite: occlusion-aware sensing, affordance-scored frontier selection (goal progress, travel cost, line of sight, openness), and a Dijkstra local-planner handoff that routes a robot around walls it cannot see past.",
    size: "wide"
  },
  {
    title: "Rigid-Body MIP Planning",
    category: "Path Planning",
    image: "assets/rigid-body-mip-planning.svg",
    command: "cargo run -p rust_robotics --example render_rigid_body_mip_planning_svg --no-default-features --features planning",
    description: "Rectangular rigid-body planning through a convex-polygon slot with pose and segment half-space certificates.",
    size: "wide"
  },
  {
    title: "Rigid-Body Backend Benchmark",
    category: "Path Planning",
    image: "assets/rigid-body-backend-benchmark.svg",
    command: "cargo run -p rust_robotics --example benchmark_rigid_body_backends --no-default-features --features planning",
    description: "Deterministic lattice A* fallback vs sampling RRT-SE(2) backend behind one trait, compared on identical scenes for path length, search effort, and clearance.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Replanning",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-replanning.svg",
    command: "cargo run -p rust_robotics --example render_hierarchical_mapf_replanning_svg --no-default-features --features planning",
    description: "Region-triggered CBS group replanning that repairs multi-robot conflicts without a full global replan.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Scale",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-scale.svg",
    command: "cargo run -p rust_robotics --example benchmark_hierarchical_mapf_scale --no-default-features --features planning",
    description: "50/100/200-agent corridor-swap benchmark showing local CBS repair groups stay size two.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Region Sweep",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-region-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_hierarchical_mapf_sweeps --no-default-features --features planning",
    description: "Coarser regions merge neighboring swaps into larger CBS groups; runtime climbs steeply as decomposition is lost.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Density Sweep",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-density-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_hierarchical_mapf_sweeps --no-default-features --features planning",
    description: "Adding swap clusters keeps repair groups at size two against a bounded flat-CBS subset baseline.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Anisotropic Sweep",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-anisotropic-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_hierarchical_mapf_sweeps --no-default-features --features planning",
    description: "On a horizontal swap row the repair-group size tracks region width and is invariant to region height: width 8 merges the same swaps at height 4 or 16.",
    size: "wide"
  },
  {
    title: "Hierarchical MAPF Fallback Rate",
    category: "Path Planning",
    image: "assets/hierarchical-mapf-fallback-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_hierarchical_mapf_sweeps --no-default-features --features planning",
    description: "An adjacent edge-swap straddles a region boundary with probability about 1/region, so the region abstraction misses it and falls back to full CBS; the fallback rate decays as regions grow while every scene still resolves.",
    size: "wide"
  },
  {
    title: "Theta*",
    category: "Path Planning",
    image: "img/path_planning/theta_star_result.svg",
    command: "cargo run --example theta_star",
    description: "Any-angle planning with a path that visibly cuts out the grid bias."
  },
  {
    title: "Jump Point Search",
    category: "Path Planning",
    image: "img/path_planning/jps_result.svg",
    command: "cargo run --example jps",
    description: "Aggressive pruning with the same visual clarity as a direct A* comparison."
  },
  {
    title: "Lazy Theta*",
    category: "Path Planning",
    image: "img/path_planning/theta_star_result.svg",
    command: "cargo test --lib lazy_theta_star",
    description: "Deferred line-of-sight evaluation: same path quality as Theta*, 1.7x faster (p=0.025 on 160 MovingAI scenarios)."
  },
  {
    title: "Enhanced Lazy Theta*",
    category: "Path Planning",
    image: "img/path_planning/theta_star_result.svg",
    command: "cargo test --lib enhanced_lazy_theta_star",
    description: "2-ring neighborhood + ancestor chain search at expansion. Near-optimal any-angle paths (+0.11% gap)."
  },
  {
    title: "Anya (Optimal Any-Angle)",
    category: "Path Planning",
    image: "img/path_planning/a_star_result.png",
    command: "cargo test --lib anya",
    description: "Visibility-graph Dijkstra over all free cells. True optimal any-angle baseline for benchmarking."
  },
  {
    title: "Path Smoothing",
    category: "Path Planning",
    image: "img/path_planning/a_star_result.png",
    command: "cargo test --lib path_smoothing",
    description: "LOS shortcutting + waypoint relaxation. A*+smooth matches Theta* quality at 2.3x speed."
  },
  {
    title: "D* Lite",
    category: "Path Planning",
    image: "img/path_planning/d_star_lite_result.png",
    command: "cargo run --example d_star_lite",
    description: "Dynamic replanning in a grid map, ideal for a fast obstacle-update clip."
  },
  {
    title: "Bezier Path",
    category: "Path Planning",
    image: "img/path_planning/bezier_custom_result.png",
    command: "cargo run --example bezier_path",
    description: "Smooth curvature and a more polished path aesthetic than a hard grid route."
  },
  {
    title: "Bezier Curvature Profile",
    category: "Path Planning",
    image: "img/path_planning/bezier_curvature_profile.png",
    command: "cargo run --example bezier_path",
    description: "Turns spline geometry into a technical chart that still looks good in a carousel."
  },
  {
    title: "Cubic Spline",
    category: "Path Planning",
    image: "img/path_planning/cubic_spline_result.png",
    command: "cargo run --example cubic_spline_planner",
    description: "Trajectory smoothing with enough context to show shape and endpoint intent."
  },
  {
    title: "Dynamic Window Approach",
    category: "Path Planning",
    image: "img/path_planning/dwa.svg",
    command: "cargo run --example dwa",
    description: "Search arcs, obstacle field, and chosen control all visible in one frame.",
    size: "wide"
  },
  {
    title: "Informed RRT*",
    category: "Path Planning",
    image: "img/path_planning/informed_rrt_star_result.png",
    command: "cargo run --example informed_rrt_star",
    description: "Sampling-based planning with a final path that still reads well as a static image."
  },
  {
    title: "Potential Field",
    category: "Path Planning",
    image: "img/path_planning/potential_field_result.png",
    command: "cargo run --example potential_field",
    description: "A vector-field look that works especially well when cropped into a tweet image."
  },
  {
    title: "PRM",
    category: "Path Planning",
    image: "img/path_planning/prm.svg",
    command: "cargo run --example prm",
    description: "Road-map nodes and final route rendered as a dense network visual."
  },
  {
    title: "Quintic Trajectory",
    category: "Path Planning",
    image: "img/path_planning/quintic_polynomials_result.png",
    command: "cargo run --example quintic_polynomials",
    description: "Multi-constraint trajectory generation shown as a neat motion design panel."
  },
  {
    title: "Reeds-Shepp",
    category: "Path Planning",
    image: "img/path_planning/reeds_shepp_result.png",
    command: "cargo run --example reeds_shepp_path",
    description: "Forward and reverse maneuvering with car-like constraints visible in the final path."
  },
  {
    title: "State Lattice Planner",
    category: "Path Planning",
    image: "img/path_planning/state_lattice_lane.svg",
    command: "cargo run --example state_lattice",
    description: "Lattice motion primitives turned into a high-density planner poster."
  },
  {
    title: "Voronoi Road Map",
    category: "Path Planning",
    image: "img/path_planning/voronoi_road_map.svg",
    command: "cargo run --example voronoi_road_map",
    description: "A graph-heavy image that still stays legible when scaled down."
  },
  {
    title: "Frenet Optimal Trajectory",
    category: "Path Planning",
    image: "img/path_planning/frenet_optimal_trajectory.svg",
    command: "cargo run --example frenet_optimal_trajectory",
    description: "Lane-relative candidate trajectories rendered with an autonomous-driving feel.",
    size: "wide"
  },
  {
    title: "LQR Steer Control",
    category: "Path Tracking",
    image: "img/path_tracking/lqr_steer_control.png",
    command: "cargo run --example lqr_steer_control",
    description: "Tracking behavior and control performance shown without needing extra explanation."
  },
  {
    title: "Move to Pose",
    category: "Path Tracking",
    image: "img/path_tracking/move_to_pose.png",
    command: "cargo run --example move_to_pose",
    description: "Goal-seeking controller visuals that are simple enough for a wider audience."
  },
  {
    title: "Pure Pursuit",
    category: "Path Tracking",
    image: "img/path_tracking/pure_pursuit.png",
    command: "cargo run --example pure_pursuit",
    description: "A reliable control demo with a thumbnail that reads well even on mobile."
  },
  {
    title: "Stanley Controller",
    category: "Path Tracking",
    image: "img/path_tracking/stanley_controller.png",
    command: "cargo run --example stanley_controller",
    description: "Lateral control with a direct road-following visual and strong contrast."
  },
  {
    title: "Rear Wheel Feedback",
    category: "Path Tracking",
    image: "img/path_tracking/rear_wheel_feedback.svg",
    command: "cargo run --example rear_wheel_feedback",
    description: "Another controller family with enough variety to keep the feed from feeling repetitive."
  },
  {
    title: "Model Predictive Control",
    category: "Path Tracking",
    image: "img/path_tracking/mpc.svg",
    command: "cargo run --example mpc",
    description: "Constraint-aware tracking shown as a tighter, more technical planning panel."
  },
  {
    title: "C-GMRES NMPC",
    category: "Path Tracking",
    image: "img/path_tracking/cgmres_nmpc.svg",
    command: "cargo run --example cgmres_nmpc",
    description: "Nonlinear predictive control with a denser optimization feel."
  },
  {
    title: "MPPI Replay Value Grid",
    category: "Control",
    image: "assets/mppi-replay-value-grid.svg",
    command: "cargo run -p rust_robotics --example render_mppi_value_grid_svg --no-default-features --features control",
    description: "Replay-learned terminal value heatmap with obstacle margin and the final MPPI rollout.",
    size: "wide"
  },
  {
    title: "MPPI Track Progress",
    category: "Control",
    image: "assets/mppi-track-progress.svg",
    command: "cargo run -p rust_robotics --example render_mppi_track_progress_svg --no-default-features --features control",
    description: "Slalom-course MPPI comparison with track terminal values, obstacles, and two rollouts.",
    size: "wide"
  },
  {
    title: "Racing Gate MPPI",
    category: "Control",
    image: "assets/mppi-racing-gate-progress.svg",
    command: "cargo run -p rust_robotics --example render_mppi_racing_gate_progress_svg --no-default-features --features control",
    description: "Reference-free gate-progress MPPI racing through oriented gates beside the waypoint-reference baseline.",
    size: "wide"
  },
  {
    title: "Racing MPPI 3-D Gates",
    category: "Control",
    image: "assets/racing-mppi-3d.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control",
    description: "Reference-free MPPI flying a drag-limited drone through 3-D gate planes, with laps-completed, first-lap-time, speed, and aperture-margin metrics across planar, undulating, climbing, and high-drag courses.",
    size: "wide"
  },
  {
    title: "Quadrotor Racing MPPI",
    category: "Control",
    image: "assets/racing-quadrotor.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_quadrotor --no-default-features --features control",
    description: "Reference-free MPPI on a full quadrotor attitude model (collective thrust + body rates): the gate-progress objective drives orientation, so the drone pitches and rolls to aim its thrust at each gate. Reports lap progress plus tilt and body-rate metrics across slalom, climbing, closed-lap, and heavy-drone courses.",
    size: "wide"
  },
  {
    title: "CBF Safety Filter",
    category: "Control",
    image: "assets/cbf-safety-filter.svg",
    command: "cargo run -p rust_robotics --example benchmark_cbf_safety_filter --no-default-features --features control",
    description: "PolyMerge-lite control-barrier-function filter over convex polytope obstacles: an exact 2-D active-set QP minimally corrects a go-to-goal velocity so the robot stays clear, compared against the colliding raw policy across four scenarios.",
    size: "wide"
  },
  {
    title: "Motor-Level Racing MPPI",
    category: "Control",
    image: "assets/racing-motor.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_motor --no-default-features --features control",
    description: "Reference-free MPPI on a four-rotor quadrotor: rotor thrusts mix into collective thrust and body torques, body rates are states, and rotors saturate. A thrust-limited drone saturates far more and flies slower on the same slalom, exposing the thrust/torque trade-off.",
    size: "wide"
  },
  {
    title: "Motor-Lag and Battery-Sag Powertrain",
    category: "Control",
    image: "assets/racing-powertrain.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_powertrain --no-default-features --features control",
    description: "The same MPPI plan flown down one slalom through four powertrains: ideal, first-order motor lag, a fresh battery that sags under load, and a pack drained to 25 percent. The drained pack saturates its lowered thrust ceiling on nearly every step and stalls after one gate, while the fresh drone finishes.",
    size: "wide"
  },
  {
    title: "Powertrain-Aware vs Unaware MPPI",
    category: "Control",
    image: "assets/racing-powertrain-aware.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_powertrain_aware --no-default-features --features control",
    description: "Two controllers fly the same lagging, sagging powertrain. The aware one rolls candidates out through the lag and battery model, so it plans within the authority the pack can deliver. On a drained 25-percent pack the unaware controller stalls after one gate while the aware one threads all four and finishes with charge to spare.",
    size: "wide"
  },
  {
    title: "Charge-Budget Endurance Sweep",
    category: "Control",
    image: "assets/racing-powertrain-budget.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_powertrain_budget --no-default-features --features control",
    description: "A charge-budget term on the powertrain-aware controller penalizes drawing the pack below a protected reserve. Sweeping its weight over a draining multi-lap race traces one Pareto frontier: heavier budgets fly slower and complete fewer laps but end with more charge in reserve. On a hover-dominated quad, pacing buys reserve, not extra laps.",
    size: "wide"
  },
  {
    title: "Battery Recovery (Voltage Relaxation)",
    category: "Control",
    image: "assets/racing-powertrain-recovery.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_powertrain_recovery --no-default-features --features control",
    description: "A relaxation-overpotential model so the pack's terminal voltage recovers when the load eases. Driven through a scripted hard/rest profile, the recovery model's terminal voltage is dragged down during full throttle and climbs back over the following hover, even as the state of charge only ever falls — the lever a charge budget needs to buy laps, not just reserve.",
    size: "wide"
  },
  {
    title: "Charge Budget x Battery Recovery",
    category: "Control",
    image: "assets/racing-powertrain-endurance.svg",
    command: "cargo run -p rust_robotics --example benchmark_racing_powertrain_endurance --no-default-features --features control",
    description: "The capstone 2x2 of greedy vs budgeted and recovery off vs on, on a draining undulating multi-lap square. Without recovery the budget gives up a full lap; with recovery it pulls even on laps while flying faster and ending with more charge — recovery reverses pacing from a strict loss to a Pareto win. An honest finding: a clean more-laps flip needs lower hover overhead or true idle rests.",
    size: "wide"
  },
  {
    title: "Quasi-Static Planar Pushing",
    category: "Control",
    image: "assets/pusher-slider.svg",
    command: "cargo run -p rust_robotics --example benchmark_pusher_slider --no-default-features --features control",
    description: "Contact-rich manipulation in the spirit of Push Anything: a point pusher that may switch among the slider's four faces drives a square slider to goal poses under the quasi-static ellipsoidal limit-surface model with stick/slide contact modes. A face-aware MPPI pusher translates, steers, and even spins the slider in place (pure rotation, reachable only by switching faces) to within a centimetre.",
    size: "wide"
  },
  {
    title: "Multi-Object Pushing",
    category: "Control",
    image: "assets/pusher-slider-multi.svg",
    command: "cargo run -p rust_robotics --example benchmark_pusher_slider_multi --no-default-features --features control",
    description: "The multi-object setting of Push Anything: three sliders are pushed into goal slots one at a time, with the other objects treated as keep-out discs so the active slider routes around blocks in its path. Object 0 detours around object 1 sitting in its straight-line path; all three settle within a centimetre.",
    size: "wide"
  },
  {
    title: "Two-Contact Pushing",
    category: "Control",
    image: "assets/pusher-slider-two-contact.svg",
    command: "cargo run -p rust_robotics --example benchmark_pusher_slider_two_contact --no-default-features --features control",
    description: "Two simultaneous pushers solved contact-implicitly: per-contact stick/slide modes are enumerated and the 4x4 contact-force system solved. A single off-center push curves; a symmetric two-point push tracks dead straight; an antipodal couple spins the slider ~278 degrees in place with zero net translation — motion a single contact cannot produce.",
    size: "wide"
  },
  {
    title: "Distributed Formation Consensus (ADMM)",
    category: "Control",
    image: "assets/admm-formation.svg",
    command: "cargo run -p rust_robotics --example benchmark_admm_formation --no-default-features --features control",
    description: "The coordination layer of distributed MPC: six agents agree on a hexagon formation center via consensus ADMM while each is pulled toward its own preferred position. Two agents confined to a corridor pull the consensus center left, the case where ADMM beats the closed-form average. Primal and dual residuals fall below 1e-7 in ~91 iterations.",
    size: "wide"
  },
  {
    title: "Decentralized Graph Consensus (ADMM)",
    category: "Control",
    image: "assets/admm-graph-consensus.svg",
    command: "cargo run -p rust_robotics --example benchmark_admm_graph_consensus --no-default-features --features control",
    description: "Edge-based decentralized ADMM with no central coordinator: agents agree on a formation only by talking to graph neighbors. The same eight agents converge over a line, ring, and complete graph — all to the same center, but the complete graph converges in ~36 iterations versus ~80 for the sparse graphs, so connectivity sets the rate.",
    size: "wide"
  },
  {
    title: "Receding-Horizon Trajectory Consensus (ADMM)",
    category: "Control",
    image: "assets/admm-horizon-consensus.svg",
    command: "cargo run -p rust_robotics --example benchmark_admm_horizon_consensus --no-default-features --features control",
    description: "Consensus over short trajectories, not static points: a four-agent formation follows a moving goal past a sharp L-corner via a receding-horizon MPC loop. A temporal-smoothness penalty couples the shared center across time, turning the consensus z-update into a banded Cholesky solve. Under noisy per-agent perception, smoothing rejects noise both spatially and temporally — cutting executed jerk ~66% while also improving tracking; with clean sensing the classic corner-cutting lag trade-off returns.",
    size: "wide"
  },
  {
    title: "Adap-RPF-lite MPPI",
    category: "Control",
    image: "assets/adap-rpf-lite-mppi.svg",
    command: "cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control",
    description: "Adaptive person-following point sampling with prediction-aware MPPI around a moving pedestrian occluder.",
    size: "wide"
  },
  {
    title: "Adap-RPF Metric Sweep",
    category: "Control",
    image: "assets/adap-rpf-metrics-sweep.svg",
    command: "cargo run -p rust_robotics --example benchmark_adap_rpf_metrics --no-default-features --features control",
    description: "Closed-loop target visibility and spacing metrics across occlusion and moving-pedestrian scenarios: fixed back-point vs adaptive following.",
    size: "wide"
  },
  {
    title: "Inverted Pendulum LQR",
    category: "Control",
    image: "img/inverted_pendulum/inverted_pendulum_lqr.png",
    command: "cargo run --example inverted_pendulum_lqr",
    description: "A classic control benchmark with a frame that instantly communicates stability."
  },
  {
    title: "Inverted Pendulum MPC",
    category: "Control",
    image: "img/inverted_pendulum/mpc/mpc_summary.png",
    command: "cargo run --example inverted_pendulum_mpc",
    description: "Prediction horizon behavior compressed into a single summary frame.",
    size: "wide"
  },
  {
    title: "MPC Sequence Frame",
    category: "Control",
    image: "img/inverted_pendulum/mpc/mpc_frame_0010.png",
    command: "cargo run --example inverted_pendulum_mpc",
    description: "Frame-level output that turns well into a GIF or timeline crop."
  },
  {
    title: "Two Joint Arm Control",
    category: "Arm Navigation",
    image: "img/arm_navigation/two_joint_arm_control.png",
    command: "cargo run --example two_joint_arm_control",
    description: "Manipulator motion gives the wall a different silhouette than ground robots."
  },
  {
    title: "Arm Demo Summary",
    category: "Arm Navigation",
    image: "img/arm_navigation/random_demo_summary.png",
    command: "cargo run --example two_joint_arm_control",
    description: "A compact montage that feels built for a repository hero panel."
  },
  {
    title: "Arm Sequence Frame",
    category: "Arm Navigation",
    image: "img/arm_navigation/target_01/frame_0004.png",
    command: "cargo run --example two_joint_arm_control",
    description: "Single-frame arm motion that works as a supporting card in a dense layout.",
    size: "tall"
  },
  {
    title: "Arm Sequence Finale",
    category: "Arm Navigation",
    image: "img/arm_navigation/target_03/frame_0006.png",
    command: "cargo run --example two_joint_arm_control",
    description: "Another manipulator frame to keep the image wall from flattening into one motif.",
    size: "tall"
  },
  {
    title: "State Machine",
    category: "Mission Planning",
    image: "img/mission_planning/state_machine_diagram.png",
    command: "cargo run --example state_machine",
    description: "Mission flow visualized as a product-grade diagram instead of a code dump."
  },
  {
    title: "Behavior Tree",
    category: "Mission Planning",
    image: "assets/behavior-tree-teaser.svg",
    command: "cargo run --example behavior_tree",
    description: "A fresh diagram card to show off the new mission planning module."
  },
  {
    title: "3D Grid A*",
    category: "Aerial Navigation",
    image: "assets/grid-a-star-3d-teaser.svg",
    command: "cargo run --example grid_a_star_3d",
    description: "An aerial route teaser that gives the showcase a new dimension."
  }
];

const playTiles = [
  {
    title: "Drive LiDAR SLAM",
    text: "Map a world live, close loops, navigate, explore, get kidnapped.",
    image: "assets/playground/drive.png",
    link: "playground/?tab=slam&algorithm=drive"
  },
  {
    title: "Grid planners",
    text: "Draw walls and race A*, Dijkstra, JPS, and Theta*.",
    image: "assets/playground/grid.png",
    link: "playground/?tab=grid"
  },
  {
    title: "Sampling planners",
    text: "Watch RRT, RRT*, Informed RRT*, and PRM explore.",
    image: "assets/playground/sampling.png",
    link: "playground/?tab=sampling"
  },
  {
    title: "Parking",
    text: "Hybrid A* parks a car, reversing where it has to.",
    image: "assets/playground/parking.png",
    link: "playground/?tab=parking"
  },
  {
    title: "Localization",
    text: "Particle filter vs EKF under adjustable sensor noise.",
    image: "assets/playground/localization.png",
    link: "playground/?tab=localization"
  },
  {
    title: "Controller arena",
    text: "Draw a course and race Pure Pursuit, Stanley, and LQR on it.",
    image: "assets/playground/arena.png",
    link: "playground/?tab=arena"
  },
  {
    title: "MPPI",
    text: "See every sampled rollout as MPPI dodges moving obstacles.",
    image: "assets/playground/mppi.png",
    link: "playground/?tab=mppi"
  },
  {
    title: "Pushing",
    text: "Face-switching MPPI pushes a box to the goal you drag.",
    image: "assets/playground/pushing.png",
    link: "playground/?tab=pushing"
  },
  {
    title: "ADMM formation",
    text: "Four agents agree on a formation while tracking a noisy goal.",
    image: "assets/playground/admm.png",
    link: "playground/?tab=admm"
  }
];

const sourceByCategory = {
  Localization: "crates/rust_robotics_localization/src",
  Mapping: "crates/rust_robotics_mapping/src",
  SLAM: "crates/rust_robotics_slam/src",
  "Path Planning": "crates/rust_robotics_planning/src",
  "Path Tracking": "crates/rust_robotics_control/src",
  Control: "crates/rust_robotics_control/src",
  "Arm Navigation": "crates/rust_robotics_control/src",
  "Mission Planning": "crates/rust_robotics_control/src",
  "Aerial Navigation": "crates/rust_robotics_planning/src"
};
const repo = "https://github.com/rsasaki0109/rust_robotics";

// Cards shown before "Show all".
const PREVIEW_COUNT = 12;

const galleryGrid = document.getElementById("gallery-grid");
const filtersRoot = document.getElementById("category-filters");
const showAll = document.getElementById("show-all");
let category = "All";
let expanded = false;

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text) node.textContent = text;
  return node;
}

function image(src, alt) {
  const img = element("img");
  img.src = src;
  img.alt = alt;
  img.loading = "lazy";
  img.decoding = "async";
  return img;
}

function renderTiles() {
  const root = document.getElementById("play-tiles");
  root.replaceChildren(
    ...playTiles.map((tile) => {
      const link = element("a", "tile");
      link.href = tile.link;
      const body = element("div", "tile-body");
      body.append(element("h3", "", tile.title), element("p", "", tile.text));
      link.append(image(tile.image, `${tile.title} in the playground`), body);
      return link;
    })
  );
}

function card(item) {
  const link = element("a", "card");
  link.href = `${repo}/tree/main/${sourceByCategory[item.category] || ""}`;
  link.target = "_blank";
  link.rel = "noreferrer";
  link.title = `${item.description}\n\n${item.command}`;
  const media = element("div", "card-media");
  media.append(image(item.image, item.title));
  const body = element("div", "card-body");
  body.append(element("span", "card-category", item.category), element("h3", "", item.title));
  link.append(media, body);
  return link;
}

function renderGallery() {
  const items =
    category === "All" ? galleryItems : galleryItems.filter((item) => item.category === category);
  const visible = expanded ? items : items.slice(0, PREVIEW_COUNT);
  galleryGrid.replaceChildren(...visible.map(card));
  showAll.hidden = visible.length === items.length;
  showAll.textContent = `Show all ${items.length}`;
}

function renderFilters() {
  const categories = ["All", ...new Set(galleryItems.map((item) => item.category))];
  filtersRoot.replaceChildren(
    ...categories.map((name) => {
      const button = element("button", "chip", name);
      button.type = "button";
      button.setAttribute("aria-pressed", String(name === category));
      button.addEventListener("click", () => {
        category = name;
        expanded = false;
        filtersRoot
          .querySelectorAll(".chip")
          .forEach((chip) => chip.setAttribute("aria-pressed", String(chip === button)));
        renderGallery();
      });
      return button;
    })
  );
}

showAll.addEventListener("click", () => {
  expanded = true;
  renderGallery();
});

renderTiles();
renderFilters();
renderGallery();
