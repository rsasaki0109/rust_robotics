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

const galleryGrid = document.getElementById("gallery-grid");
const filtersRoot = document.getElementById("category-filters");
const marqueeRoot = document.getElementById("hero-marquee");
const template = document.getElementById("card-template");
const galleryCount = document.getElementById("gallery-count");
const visualCount = document.getElementById("visual-count");
const moduleCount = document.getElementById("module-count");

const stats = {
  visuals: 101,
  modules: 12
};

visualCount.textContent = stats.visuals.toString();
moduleCount.textContent = stats.modules.toString();

function createCard(item, index) {
  const fragment = template.content.cloneNode(true);
  const card = fragment.querySelector(".gallery-card");
  const image = fragment.querySelector("img");
  const category = fragment.querySelector(".category-pill");
  const command = fragment.querySelector(".command-chip");
  const title = fragment.querySelector("h3");
  const description = fragment.querySelector(".card-description");

  card.dataset.category = item.category;
  card.style.animationDelay = `${Math.min(index * 45, 540)}ms`;

  if (item.size === "wide") {
    card.classList.add("card-wide");
  }

  if (item.size === "tall") {
    card.classList.add("card-tall");
  }

  image.src = item.image;
  image.alt = `${item.title} showcase image`;
  category.textContent = item.category;
  command.textContent = item.command.replace("cargo run --", "");
  title.textContent = item.title;
  description.textContent = item.description;

  return fragment;
}

function renderGallery(items) {
  galleryGrid.replaceChildren();
  items.forEach((item, index) => {
    galleryGrid.appendChild(createCard(item, index));
  });
  galleryCount.textContent = items.length.toString();
}

function renderFilters(items) {
  const categories = ["All", ...new Set(items.map((item) => item.category))];
  const buttons = categories.map((category) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "filter-chip";
    button.textContent = category;
    button.dataset.category = category;
    if (category === "All") {
      button.classList.add("is-active");
      button.setAttribute("aria-pressed", "true");
    } else {
      button.setAttribute("aria-pressed", "false");
    }

    button.addEventListener("click", () => {
      filtersRoot.querySelectorAll(".filter-chip").forEach((chip) => {
        const isActive = chip === button;
        chip.classList.toggle("is-active", isActive);
        chip.setAttribute("aria-pressed", isActive ? "true" : "false");
      });

      const filtered =
        category === "All" ? galleryItems : galleryItems.filter((item) => item.category === category);
      renderGallery(filtered);
    });

    return button;
  });

  filtersRoot.replaceChildren(...buttons);
}

function renderMarquee(items) {
  const featured = items.slice(0, 10);
  const cards = [...featured, ...featured].map((item) => {
    const figure = document.createElement("figure");
    figure.className = "marquee-card";

    const image = document.createElement("img");
    image.src = item.image;
    image.alt = `${item.title} featured frame`;
    image.loading = "lazy";
    image.decoding = "async";

    const caption = document.createElement("figcaption");
    caption.textContent = `${item.title} / ${item.category}`;

    figure.append(image, caption);
    return figure;
  });

  marqueeRoot.replaceChildren(...cards);
}

renderMarquee(galleryItems);
renderFilters(galleryItems);
renderGallery(galleryItems);
