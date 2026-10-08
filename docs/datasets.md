# Dataset ingestion and VIO replay

RustRobotics reads the standard extracted layouts of the
[EuRoC MAV datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
and the [KITTI odometry benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).
Dataset archives are not redistributed by this repository.

## EuRoC

Pass either the sequence directory or its `mav0` directory to
`EurocDataset::load`. The loader reads:

```text
MH_01_easy/
└── mav0/
    ├── cam0/
    │   ├── data.csv
    │   ├── data/*.png
    │   └── sensor.yaml
    ├── imu0/
    │   ├── data.csv
    │   └── sensor.yaml
    └── state_groundtruth_estimate0/data.csv  # optional
```

It validates increasing timestamps, parses camera intrinsics, resolution and
the camera/IMU `T_BS` transforms, and exposes efficient IMU interval slices.
IMU samples are transformed into the body frame before preintegration,
including centripetal and tangential acceleration caused by a non-zero sensor
lever arm. A missing legacy `imu0/sensor.yaml` falls back to an identity
transform. Images remain paths so applications can choose their own decoder.
RustRobotics includes a PNG-based sparse frontend CLI:

```bash
cargo run --release -p rust_robotics \
  --example generate_euroc_feature_tracks \
  --no-default-features --features slam -- \
  /datasets/EuRoC/MH_01_easy
```

The frontend distributes Shi-Tomasi corners, tracks them with pyramidal
Lucas-Kanade optical flow, applies a forward/backward consistency check, and
triangulates persistent tracks from the metric IMU-predicted camera
trajectory. Only the first ground-truth state initializes pose, velocity and
IMU biases. Existing sidecars are never replaced unless `--force` is passed.
Use `--output DIR` to write elsewhere and `--max-features N` to adjust the
per-frame cap.

The offline VIO example consumes generated or externally extracted tracks
from this optional sidecar:

```text
mav0/rust_robotics/
├── landmarks.csv     # landmark_id,x,y,z
└── observations.csv  # timestamp_ns,landmark_id,u,v
```

Landmark IDs must be contiguous and zero-based. Observation timestamps must
match `cam0/data.csv`. A different frontend can export the same interchange
format without coupling the SLAM crate itself to an image library.

```bash
# Checked-in miniature replay
cargo run -p rust_robotics --example headless_euroc_vio \
  --no-default-features --features slam

# Extracted sequence after running generate_euroc_feature_tracks
cargo run --release -p rust_robotics --example headless_euroc_vio \
  --no-default-features --features slam -- /datasets/EuRoC/MH_01_easy
```

The replay uses ground truth only for the initial state and acceptance report.
Later ground-truth states are not optimizer inputs:

```text
EuRoC IMU + T_BS ──> lever-arm correction ──> bias-aware preintegration
feature tracks ─────────────────────────────> camera/landmark BA (Schur)
BA poses + IMU factors ─────────────────────> navigation-state/bias refinement
IMU relative edges + visual closure ────────> block-sparse SE(3) pose graph
```

## KITTI odometry

`KittiOdometryDataset::load(root, "00")` reads the official layout:

```text
dataset/
├── poses/00.txt
└── sequences/00/
    ├── calib.txt
    ├── times.txt
    ├── image_0/ ... image_3/
    └── velodyne/*.bin
```

The API exposes image/LiDAR paths, 3×4 camera projections,
Velodyne-to-camera calibration, timestamps and optional ground-truth poses.
`read_velodyne` decodes the official little-endian `(x, y, z, reflectance)`
float32 tuples.

Checked-in fixtures contain only original synthetic numeric rows in the
official layouts; they do not copy dataset imagery or measurements.

## CARMEN 2D laser logs (SLAM benchmark)

`rust_robotics_slam::carmen` reads the `FLASER` and `ROBOTLASER1` lines of
CARMEN logs, the format of the classic 2D laser datasets (Intel Research Lab,
Freiburg 079 / campus, MIT Killian Court, ACES). `FLASER` has no beam
geometry, so the CARMEN front-laser convention is used (`-π/2`, `π / n`).

`rust_robotics_slam::slam_benchmark` implements the relative-pose metric of
Kümmerle et al., "On measuring the accuracy of SLAM algorithms" (2009): each
relation `t_i t_j x y z roll pitch yaw` is a verified relative pose, and the
error of an estimate is `(x_i⁻¹ ⊕ x_j) ⊖ δ*_ij`, averaged separately for
translation and rotation (absolute and squared). It is frame-independent, so
any trajectory can be scored.

```bash
# Real data: download a log and its relations from the benchmark page
# (http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php), then
cargo run --release -p rust_robotics --example carmen_lidar_slam \
  --no-default-features --features slam -- intel.clf intel.relations --map intel_map.png

# No arguments: a synthetic 180° front-laser corridor-loop log is generated,
# written as CARMEN text, parsed back, and scored (this runs in CI).
cargo run -p rust_robotics --example carmen_lidar_slam --no-default-features --features slam
```

Synthetic run (978 scans, 61 relations: local 2 m relations plus loop
relations):

| estimator | translation error \[m\] | rotation error \[deg\] |
| --- | ---: | ---: |
| odometry | 2.58 ± 5.05 | 19.9 ± 34.4 |
| scan-to-map | 0.057 ± 0.091 | 0.45 ± 0.64 |
| graph SLAM | 0.015 ± 0.017 | 0.14 ± 0.12 |

Real-data results are not recorded yet: the benchmark host was not reachable
from the development environment. The defaults (0.1–25 m usable range, a scan
processed every 0.1 m / 0.05 rad) are starting points, not tuned values.

