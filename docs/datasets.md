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
