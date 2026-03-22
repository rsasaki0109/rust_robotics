# CLAUDE.md

## プロジェクト概要

Rustで実装したロボティクスアルゴリズム集。PythonRoboticsのRust移植。
Cargo workspaceで複数crateに分割。

## ワークスペース構成

```
crates/
├── rust_robotics_core        # 共通型・トレイト (Point2D, Pose2D, Path2D, etc.)
├── rust_robotics_planning    # 経路計画 (A*, JPS, Theta*, RRT, DWA, etc.)
├── rust_robotics_localization # 自己位置推定 (EKF, UKF, PF, Histogram Filter)
├── rust_robotics_control     # 制御 (Pure Pursuit, Stanley, LQR, MPC, etc.)
├── rust_robotics_mapping     # 地図生成 (Gaussian Grid Map, Ray Casting)
├── rust_robotics_slam        # SLAM (ICP, EKF-SLAM, FastSLAM)
├── rust_robotics_viz         # 可視化 (gnuplot wrapper)
└── rust_robotics             # Umbrella crate (feature-gated re-exports)
```

## ビルド・テスト

```bash
cargo build --workspace              # 全体ビルド
cargo test --workspace               # 全テスト実行
cargo clippy --workspace --all-features  # lint
cargo fmt --all -- --check           # フォーマットチェック
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps  # doc警告チェック
```

## Example実行

```bash
# Headless (GUIなし)
cargo run -p rust_robotics --example headless_grid_planners --features planning
cargo run -p rust_robotics --example headless_localizers --features localization
cargo run -p rust_robotics --example headless_navigation_loop --features "planning,localization,control"

# Visualization (gnuplot必要)
cargo run -p rust_robotics --example a_star --features "planning,viz"
cargo run -p rust_robotics --example rear_wheel_feedback --features "control,viz"
```

## Feature flags (umbrella crate)

- `planning`, `localization`, `control`, `mapping`, `slam` — 各ドメインcrate
- `viz` — 可視化 (gnuplot依存)
- `full` — 全feature有効
- デフォルト: `planning`, `localization`, `control`, `mapping`, `slam`

## CI

- GitHub Actions: `.github/workflows/ci.yml`
- ステップ: build → test → test (no-default-features) → headless examples → clippy → rustdoc → fmt → cargo-deny
- 別ジョブ: coverage (cargo-tarpaulin → Codecov)
- Clippy/doc/fmtは `-D warnings` でエラー扱い

## 注意事項

- docコメントで `[m]`, `[rad]` 等を書く場合は `\[m\]` とエスケープ（rustdocがリンクと誤認するため）
- bare URLは `<https://...>` で囲む（rustdoc警告回避）
