# CLAUDE.md - RustRobotics プロジェクトガイド

このファイルはAIアシスタントがこのコードベースを効率的に理解・操作するためのガイドです。

## プロジェクト概要

RustRoboticsは、[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)のRust実装です。ロボティクス分野で使用される様々なアルゴリズム（自己位置推定、マッピング、経路計画、経路追従、制御）を提供します。

## リポジトリ構造

```
rust_robotics/
├── src/
│   ├── lib.rs                 # ライブラリのエントリーポイント
│   ├── common/                # 共通型・トレイト・エラー定義
│   │   ├── types.rs           # Point2D, Pose2D, State2D, Path2D等
│   │   ├── traits.rs          # PathPlanner, StateEstimator等のトレイト
│   │   └── error.rs           # RoboticsError列挙型
│   ├── utils/                 # ユーティリティ
│   │   ├── grid_map.rs        # グリッドマップ実装
│   │   ├── grid_map_planner.rs
│   │   └── visualization.rs   # 可視化ヘルパー
│   ├── localization/          # 自己位置推定アルゴリズム
│   │   ├── ekf.rs             # 拡張カルマンフィルタ
│   │   ├── particle_filter.rs # パーティクルフィルタ
│   │   ├── unscented_kalman_filter.rs
│   │   └── histogram_filter.rs
│   ├── mapping/               # マッピングアルゴリズム
│   │   ├── ndt.rs             # NDTマップ
│   │   ├── gaussian_grid_map.rs
│   │   └── ray_casting_grid_map.rs
│   ├── slam/                  # SLAMアルゴリズム
│   │   ├── icp_matching.rs    # ICP点群マッチング
│   │   ├── fastslam1.rs       # FastSLAM 1.0
│   │   ├── ekf_slam.rs        # EKF SLAM
│   │   └── graph_based_slam.rs
│   ├── path_planning/         # 経路計画アルゴリズム
│   │   ├── a_star.rs          # A*アルゴリズム
│   │   ├── dijkstra.rs        # ダイクストラ法
│   │   ├── d_star_lite.rs     # D* Lite
│   │   ├── jps.rs             # Jump Point Search
│   │   ├── theta_star.rs      # Theta*
│   │   ├── rrt.rs             # RRT
│   │   ├── rrt_star.rs        # RRT*
│   │   ├── informed_rrt_star.rs
│   │   ├── prm.rs             # PRM
│   │   ├── dwa.rs             # Dynamic Window Approach
│   │   ├── potential_field.rs # ポテンシャル場法
│   │   ├── frenet_optimal_trajectory.rs
│   │   ├── state_lattice/     # State Lattice Planner
│   │   ├── bezier_path.rs     # ベジェ曲線
│   │   ├── cubic_spline_planner.rs
│   │   ├── quintic_polynomials.rs
│   │   └── reeds_shepp_path.rs
│   ├── path_tracking/         # 経路追従アルゴリズム
│   │   ├── lqr_steer_control.rs
│   │   ├── pure_pursuit.rs    # Pure Pursuit
│   │   ├── stanley_controller.rs
│   │   ├── mpc.rs             # Model Predictive Control
│   │   ├── move_to_pose.rs
│   │   └── rear_wheel_feedback.rs
│   ├── control/               # 制御アルゴリズム
│   │   ├── lqr_control.rs     # LQR（倒立振子）
│   │   └── mpc_control.rs     # MPC（倒立振子）
│   ├── arm_navigation/        # アーム制御
│   │   └── two_joint_arm_control.rs
│   ├── aerial_navigation/     # 空中ナビゲーション
│   └── mission_planning/      # ミッション計画
│       └── state_machine.rs   # 状態機械
├── examples/                  # 実行可能なサンプル
│   ├── path_planning/
│   │   ├── a_star.rs
│   │   ├── theta_star.rs
│   │   ├── jps.rs
│   │   └── state_lattice.rs
│   └── path_tracking/
│       └── rear_wheel_feedback.rs
├── img/                       # アルゴリズムの出力画像
├── .github/workflows/ci.yml   # CI設定
└── .devcontainer/             # Dev Container設定
```

## ビルド・テスト・実行コマンド

### ビルド
```bash
cargo build           # ライブラリとバイナリをビルド
cargo build --lib     # ライブラリのみビルド
cargo build --release # リリースビルド
```

### テスト
```bash
cargo test --lib              # ライブラリテストを実行
cargo test --lib -- --skip slam  # SLAMテストをスキップして実行（CI設定）
cargo test <テスト名>          # 特定のテストを実行
```

### アルゴリズムの実行
```bash
# Examples（推奨形式）
cargo run --example a_star
cargo run --example theta_star
cargo run --example jps
cargo run --example state_lattice
cargo run --example rear_wheel_feedback

# Legacy binaries
cargo run --bin rrt
cargo run --bin particle_filter
cargo run --bin inverted_pendulum_lqr
cargo run --bin two_joint_arm_control
```

### コード品質チェック
```bash
cargo clippy --lib -- -W clippy::all  # Lint警告チェック
cargo fmt -- --check                   # フォーマットチェック
cargo fmt                              # 自動フォーマット
```

## 依存関係

| クレート | バージョン | 用途 |
|---------|-----------|------|
| `nalgebra` | 0.33 | 線形代数（行列・ベクトル演算） |
| `plotlib` | 0.5 | プロット生成 |
| `gnuplot` | 0.0.43 | グラフ可視化 |
| `rand` | 0.8 | 乱数生成 |
| `rand_distr` | 0.4 | 確率分布 |
| `itertools` | 0.13 | イテレータ拡張 |
| `ordered-float` | 4.2 | 順序付き浮動小数点 |

## コーディング規約

### 型定義
- `Point2D`: 2D座標点 (x, y)
- `Pose2D`: 2D姿勢 (x, y, yaw)
- `State2D`: 2D状態 (x, y, yaw, v)
- `Path2D`: 2D経路（Point2Dのベクトル）
- `ControlInput`: 制御入力 (v, omega)
- `Obstacles`: 障害物集合

### トレイト
- `PathPlanner`: 経路計画アルゴリズム用トレイト
- `StateEstimator`: 状態推定アルゴリズム用トレイト
- `PathTracker`: 経路追従アルゴリズム用トレイト
- `Controller`: 制御器用トレイト
- `MotionModel`: 運動モデル用トレイト

### エラーハンドリング
`RoboticsError`列挙型を使用:
- `PlanningError`: 経路計画失敗
- `EstimationError`: 状態推定失敗
- `ControlError`: 制御計算失敗
- `InvalidParameter`: 無効なパラメータ
- `NumericalError`: 数値計算エラー

`RoboticsResult<T>`型エイリアスを使用して結果を返す。

### モジュール構成パターン
各アルゴリズムモジュールは以下の構成に従う:
1. モジュールドキュメンテーションコメント (`//!`)
2. `Config`構造体（設定パラメータ）
3. メインのプランナー/コントローラー構造体
4. トレイト実装
5. `#[cfg(test)] mod tests`でユニットテスト

### コードスタイル
- 関数・変数名: snake_case
- 型名: PascalCase
- 定数: SCREAMING_SNAKE_CASE
- ドキュメンテーションコメント: `///`（アイテム用）、`//!`（モジュール用）
- `Default`トレイトの実装を推奨

## CI/CD

GitHub Actions (`.github/workflows/ci.yml`):
- `main`ブランチと`feat/*`ブランチへのプッシュ時に実行
- `main`ブランチへのPR時に実行
- ステップ:
  1. `cargo build --lib` - ライブラリビルド
  2. `cargo test --lib -- --skip slam` - テスト実行（SLAMスキップ）
  3. `cargo clippy --lib` - Lint（警告のみ、失敗しない）
  4. `cargo fmt -- --check` - フォーマットチェック（警告のみ）

## 新しいアルゴリズムの追加手順

1. 適切なモジュールディレクトリに新しいファイルを作成
2. モジュールの`mod.rs`にpub modを追加
3. 必要に応じてre-exportを追加
4. Configの構造体と`Default`実装を作成
5. 適切なトレイトを実装（例: `PathPlanner`）
6. ユニットテストを追加
7. `examples/`にサンプルコードを追加
8. `Cargo.toml`にexampleまたはbinエントリを追加
9. `README.md`にドキュメントと画像を追加

## 可視化

`utils::Visualizer`を使用してグラフを生成:
```rust
use rust_robotics::utils::{Visualizer, PathStyle, PointStyle, colors};

let mut vis = Visualizer::new();
vis.set_title("タイトル");
vis.plot_obstacles_xy(&ox, &oy);
vis.plot_start(start);
vis.plot_goal(goal);
vis.plot_path(&path, &PathStyle::default());
vis.save_png("img/path_planning/result.png", 800, 600);
vis.show();
```

## 注意点

- 多くのアルゴリズムは`gnuplot`を使用して可視化するため、システムにgnuplotがインストールされている必要がある
- Dev Container環境ではデスクトップ機能が利用可能（ポート6080）
- 一部のSLAMテストはCIでスキップされる（実行時間の問題）
- レガシーバイナリ（`[[bin]]`）は徐々にexamples（`[[example]]`）に移行中

## よく使うファイルパス

- ライブラリエントリ: `src/lib.rs`
- 共通型: `src/common/types.rs`
- 共通トレイト: `src/common/traits.rs`
- エラー定義: `src/common/error.rs`
- グリッドマップ: `src/utils/grid_map.rs`
- 可視化: `src/utils/visualization.rs`
