# hakoniwa-mujoco-robots

[English](README.md) | 日本語

## TL;DR
- 本リポジトリは、**Hakoniwa 向けの MuJoCo ベース robot simulation assets** を提供します。
- ROS/URDF 由来のロボットモデルを MuJoCo 上で利用できます。TurtleBot3 Burger サンプルを含みます。
- C++ MuJoCo simulator と Python controller / visualizer を **Hakoniwa PDU** で接続できます。
- TurtleBot3 サンプルでは、gamepad 制御と 2D LiDAR の `LaserScan` 互換 PDU 出力が動作します。
- LiDAR ノイズプロファイルを切り替えられます。`LDS-01` 風の noisy profile と、`URG-04LX-UG01` 相当の cleaner profile を含みます。
- フォークリフト sample、context save/restore、RD-light handoff は **上級者向けサンプル** として含まれます。
- 設定は **C++/Python ともに compact JSON** を既定で使います（`hakoniwa-pdu >= 1.3.7`）。

## Demo Videos
- TurtleBot3 + 2D LiDAR / sensor noise demo
  - [![Watch the demo](https://img.youtube.com/vi/B5h-KKH4tpg/hqdefault.jpg)](https://www.youtube.com/watch?v=B5h-KKH4tpg)
- Runtime handoff デモ（RD-light / フォークリフト2アセット）  
  - [![Watch the demo](https://img.youtube.com/vi/xaJJ1wEgNR8/hqdefault.jpg)](https://www.youtube.com/watch?v=xaJJ1wEgNR8)

### TurtleBot3 + 2D LiDAR デモの説明

MuJoCo 上で動作する TurtleBot3 Burger に、箱庭 PDU 経由の 2D LiDAR シミュレーションを実装しました。

今回は、単に LiDAR の点群を表示するだけでなく、センサごとのノイズ特性の違いも再現しています。

- TurtleBot3 標準の `LDS-01` ではノイズが大きく、点群がかなり揺らぐ
- `URG-04LX-UG01` 風の cleaner profile では、障害物の輪郭がくっきり見える

実機で経験する「センサを変えると見え方が変わる」という差を、シミュレーション上で扱えるようにすること。  
これは、箱庭で目指している Sim2Real の重要なテーマの一つです。

構成:
- ROS 由来の TurtleBot3 モデルを MuJoCo で実行
- 選択した sensor profile に基づいて 2D LiDAR を raycast
- scan frequency、角度範囲、角度分解能、noise model は JSON から読み込む
- `LaserScan` PDU として箱庭上に出力
- Python visualizer で点群を可視化
- `LDS-01` / `URG-04LX-UG01` 相当のノイズ差を再現

### Forklift RD-light Handoff Demo

これは上級者向けの実験デモです。  
フォークリフトの MuJoCo asset を 2 つ動かし、単一ノード上で ownership handoff と context save/restore を行います。

- RD-light は **advanced / experimental** な handoff デモです
- RD-full の制御プレーンではありません
- 詳細は後半の RD-light セクションと [rd-design.md](rd-design.md) を参照してください

---

## What This Repository Provides

- MuJoCo robot models
- Hakoniwa 連携 C++ simulators
- Python controllers
- Python visualizers
- PDU config files
- sensor config files
- フォークリフト向け context save/restore
- RD-light handoff デモ（advanced example）

### ディレクトリ
- `models/` MuJoCo XMLモデル
- `config/` PDU設定JSON
- `config/sensors/` LiDAR / sensor spec JSON
- `src/` C++シミュレータ実装
- `python/` Python制御コード / visualizer
- `docker/` Dockerfile/実行スクリプト
- `logs/` 実行ログ（生成物）
- `tmp/` 状態ファイル（生成物）

---

## Architecture

Hakoniwa PDU をハブとして、MuJoCo（C++）と Python controller / visualizer が接続されます。

- **Hakoniwa**: 実行同期・PDU基盤
- **MuJoCo C++ Asset**: 物理計算とPDU read/write
- **Python Controller / Visualizer**: 操作入力・確認ツール
- **PDU JSON**: 双方の契約（チャネル・型・サイズ）

```text
+-----------------------------+      PDU (shared contract)      +----------------------+
| Python Controller / Viewer  |  <----------------------------> | MuJoCo C++ Simulator |
| (gamepad / visualizer)      |                                  | (tb3_sim / forklift) |
+--------------+--------------+                                  +----------+-----------+
               |                                                            |
               |                        Hakoniwa runtime                     |
               +------------------------(sync / mmap / PDU)-----------------+
```

---

## Quick Start: TurtleBot3 + 2D LiDAR

ここでは **MuJoCo + Hakoniwa + TurtleBot3 + gamepad + LiDAR** を最短で確認する手順を示します。

ターミナルを 4 つ用意してください。

1. simulator
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim
```

2. gamepad controller
```bash
python python/tb3_gamepad.py
```

3. LiDAR visualizer
```bash
python python/lidar_visualizer.py
```

4. start trigger
```bash
hako-cmd start
```

LiDAR spec を切り替える場合:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

## Quick Start: Forklift

フォークリフト単体サンプルを最短で確認する手順です。

ターミナルを 3 つ用意してください。

1. simulator
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

2. Python controller
```bash
python -m python.forklift_simple_auto config/forklift-unit-compact.json \
  --forward-distance 2.0 --backward-distance 2.0 --move-speed 0.7
```

3. start trigger
```bash
hako-cmd start
```

互換のため `controll.bash` は当面残し、内部で `control.bash` を呼び出します。

## 前提環境

### 1) hakoniwa-core-pro の導入（必須）

```bash
git clone --recursive https://github.com/hakoniwalab/hakoniwa-core-pro.git
cd hakoniwa-core-pro
bash build.bash
bash install.bash
```

必要に応じてパスを設定：

Linux:
```bash
export PATH=/usr/local/hakoniwa/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$LD_LIBRARY_PATH
```

macOS:
```bash
export PATH=/usr/local/hakoniwa/bin:$PATH
export DYLD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$DYLD_LIBRARY_PATH
```

### 2) OS別補足

- macOS: `brew install glfw`
- Ubuntu: OpenGL/GLFW関連を導入
```bash
sudo apt-get update
sudo apt-get install -y libgl1 libgl1-mesa-dri libglx-mesa0 mesa-utils libglfw3-dev
```

## セットアップ

```bash
git clone https://github.com/hakoniwalab/hakoniwa-mujoco-robots.git
cd hakoniwa-mujoco-robots
git submodule update --init --recursive
./build.bash
```

- MuJoCoバージョンは `MUJOCO_VERSION.txt` で管理します。
- クリーンビルド:
```bash
./build.bash clean
```

---

## Detailed Run Commands

### C++サンプル

- 通常フォークリフト:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

- 単体（荷物なし、自動制御検証向け）:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

- TurtleBot3（Hakoniwa asset + endpoint gamepad + 2D LiDAR）:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim
```

- TurtleBot3（LiDAR spec を切替）:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

### Pythonサンプル

- 最小自動操縦:
```bash
python -m python.forklift_simple_auto config/custom-compact.json
```

- 単体モデル向け:
```bash
python -m python.forklift_simple_auto config/forklift-unit-compact.json --forward-distance 1.5 --backward-distance 1.5 --move-speed 0.7
```

- API制御サンプル:
```bash
python -m python.forklift_api_control config/safety-forklift-pdu-compact.json config/monitor_camera_config.json
```

- ゲームパッド制御:
```bash
python -m python.forklift_gamepad config/custom-compact.json
```

- TurtleBot3 ゲームパッド制御:
```bash
python python/tb3_gamepad.py
```

- LiDAR 可視化:
```bash
python python/lidar_visualizer.py
```

---

## TurtleBot3 2D LiDAR

TurtleBot3 Burger サンプルには、MuJoCo ベースの 2D LiDAR 実装が含まれます。

- 360 度 raycast
- 選択した sensor profile に基づく scan frame 生成
  - 例: `urg-04lx-ug01.json` では 10 Hz / 100 ms
  - 例: `lds-01.json` と `lds-02.json` では 5 Hz / 200 ms
- `LaserScan` 互換 PDU を Hakoniwa 上に publish
- Python visualizer で点群を確認可能

MuJoCo ray の self / near-body 干渉を避けるために、実装では self-geometry hit を検出し、その少し先から raycast を再試行します。大きな固定 origin offset に頼らず、近接障害物でもより自然な見え方を保ちます。

## Sensor Noise Profiles

LiDAR の見え方は sensor config JSON で切り替えられます。

- `config/sensors/lidar/lds-01.json`
  - TurtleBot3 標準 LiDAR の `LDS-01` に近い noisy profile
  - range: 0.12-3.5 m
  - scan: 5 Hz, 1.0 deg resolution
  - spec: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
- `config/sensors/lidar/lds-02.json`
  - TurtleBot3 `LDS-02` に近い longer-range profile
  - range: 0.16-8.0 m
  - scan: 5 Hz, 1.0 deg resolution
  - spec: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/
- `config/sensors/lidar/urg-04lx-ug01.json`
  - 北陽電機 `URG-04LX-UG01` ベースの cleaner profile
  - range: 0.02-5.56 m
  - scan: 10 Hz, 0.3515625 deg resolution
  - LDS 系 profile と比べて、よりクリアな障害物輪郭の比較に向いています

切替例:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

この差により、実機で経験する「センサを変えると見え方が変わる」状態を simulation 上でも扱えます。

## Docker（Ubuntu 24.04）

イメージ作成:
```bash
bash docker/create-image.bash
```

起動:
```bash
bash docker/run.bash
```

コンテナ内ビルド:
```bash
bash build.bash
```

注意:
- Ubuntu + Docker: GUIサポート対象
- macOS + Docker: **headless推奨**（GUI非サポート扱い）
```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

---

## Advanced: Forklift / Context Save-Restore

フォークリフト sample には、MuJoCo context save/restore を含む上級者向け機能が入っています。  
長時間実験の継続、中断/再開、障害復旧、将来の handoff 実験の前提機能として位置付けています。

## Advanced: Context Save/Restore（MuJoCoコンテキスト退避・復旧）

### 狙い

- 長時間実験の継続
- 中断/再開の運用性
- 障害復旧
- 将来のRD（所有権移動）に向けた前提機能

### 境界設計（退避対象）

`HakoniwaMujocoContext`（`include/hakoniwa_mujoco_context.hpp`）で退避。

- フォークリフトサブツリー状態（位置・姿勢・リフトを含む）
- フォークリフトサブツリーの物理状態（全体世界ではない）
- 制御状態（`phase`, `target_v`, `target_yaw`, `target_lift`, `step`）

### 採用したMuJoCo Context仕様（実装確定）

`HakoniwaMujocoContext`（`include/hakoniwa_mujoco_context.hpp`）で、以下を保存します。

- `ForkliftState`
  - フォークリフトサブツリーの `qpos[]`
  - フォークリフトサブツリーの `qvel[]`
  - フォークリフトサブツリーの `qacc[]`
  - フォークリフトサブツリーの `qacc_warmstart[]`
  - フォークリフトサブツリーの `qfrc_applied[]`
  - フォークリフトサブツリーの `xfrc_applied[]`
  - フォークリフト用 actuator `ctrl[]`（`left_motor`, `right_motor`, `lift_motor`）
  - `act[]`（`mjData.act`）
  - （互換/デバッグ用）base/lift の姿勢・速度スナップショット
- `ControlState`
  - `phase`
  - `target_linear_velocity`
  - `target_yaw_rate`
  - `target_lift_z`
  - `sim_step`
  - PID内部状態（`lift`, `drive_v`, `drive_w`）

実装上の保存仕様:
- 状態ファイル形式は `v8`（後方互換で `v7` / `v6` / `v5` / `v4` / `v3` / `v2` / `v1` 読み込み対応）
- autosave間隔は `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`（既定 `1000`）
- 保存先は `HAKO_FORKLIFT_STATE_FILE`（未指定時は `./tmp/...`）
- `v8` の設計意図: fork/lift文脈不足で復帰差分が出た知見を踏まえ、フォークリフトサブツリー動力学 + actuator/PID内部状態まで保存境界を拡張。

復帰時の適用仕様（`main_unit.cpp`）:
- 物理状態適用後、`lift target` を復元
- `target_linear_velocity` / `target_yaw_rate` を初期適用
- `phase=2` はセッション内ラッチ（不要な反転を抑制）

### 境界設計（退避対象外）

- 荷物・棚など外部オブジェクト状態
- 外部プロセス側の内部状態（Python内部状態など）

つまり、**完全世界スナップショットではない**ことを明示します。

### 保存/復帰

- 保存:
  - 定期autosave（step間隔）
  - 終了時保存
- 復帰:
  - 状態ファイルがあれば復元
  - なければ新規開始

互換性:
- state は **同一モデルXML** と **同一MuJoCoバージョン**（`MUJOCO_VERSION.txt`）を前提に復元します。

環境変数:
- `HAKO_FORKLIFT_STATE_FILE` 保存先
- `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS` autosave間隔
- `HAKO_FORKLIFT_MOTION_GAIN` 運動ゲイン
- `HAKO_FORKLIFT_TRACE_FILE`（既定: `./logs/forklift-unit-trace.csv`）
- `HAKO_FORKLIFT_TRACE_EVERY_STEPS`（既定: `10`）
- `HAKO_CONTROLLER_MODE`（`asset` / `external`）
  - `control.bash` の既定は `asset`（tick同期）
- `HAKO_CONTROLLER_ASSET_NAME`（既定: `forklift-controller`）
- `HAKO_CONTROLLER_DELTA_USEC`（既定: `1000`）
- `STARTUP_WAIT_SEC`（既定: `0.0`）
- `PHASE_TIMEOUT_SEC`（既定: `0.8`）

例:
```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=1000 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

### phase運用

- `phase=2`（帰り）はセッション内でラッチ運用
- 復帰時に保存済み `target_v/target_yaw` も適用し、復帰直後の挙動を安定化

### ログ確認

- `logs/forklift-unit-run.log` C++実行ログ
- `logs/control-run.log` Python実行ログ
- `logs/forklift-unit-recovery.log` 監査ログ（`START/AUTOSAVE/END`）
- `logs/forklift-unit-trace.csv` 客観評価用トレース（時系列）

成功判定（Phase1復帰 / `phase=2` ラッチ確認）:
- `START restored=yes ... phase=2 ...`
- `Resume control phase=2 ...`
- 復帰後 `AUTOSAVE` で `phase=2` 維持
Note: ここでの `phase=2` はフォークリフト制御の帰路フラグであり、FAQの「Phase2（複雑接触・外部物体込み）」とは別概念です。

ログは `tee -a` 追記です。  
1回目/2回目は `START restored=no/yes` で区切って判定します。

### 連続性グラフ（客観評価）

トレースCSVから連続性グラフを生成します。

```bash
python -m python.plot_forklift_continuity \
  --csv logs/forklift-unit-trace.csv \
  --output logs/forklift-unit-continuity.png \
  --window-sec 8
```

復帰前後セッションを重ね描きし、以下を比較できます。
- `pos_x`
- `body_vx`（`target_v` と重ね表示）
- `yaw`
- `lift_z`
- `phase`

見方:
- `pos_x`: 復帰直後の開始値が、停止前末尾に近いこと。
- `body_vx` と `target_v`: 復帰直後の一時差分は許容しつつ、短時間で収束すること。
- `phase`: 復帰後に意図しない初期化が入らないこと。
- `yaw` / `lift_z`: 微小な段差は許容、継続的な乖離は要調査。

実運用での合格目安:
- `logs/forklift-unit-recovery.log` に `START restored=yes` がある。
- 再開後の数百msで、軌跡が停止前カーブへ再収束する。
- `phase` の連続性が復帰後の autosave まで維持される。

Acceptance（Phase1, `sim_step` 整列, strict）:
- `mean(|Δbody_vx|) <= 1e-3`
- `max(|Δbody_vx|) <= 1e-2`
- `max(|Δpos_x|) <= 1e-3`
- `phase` 連続性: 一致
- `max(|Δlift_z|) <= 1e-4`
Note: 公式エビデンスでゼロ差が観測された回でも、再現性と環境差（OS/CPU/微小ノイズ）を見越して閾値は工学的許容として設定します。

評価プロトコル:
- 公式比較ウィンドウ: `4010..4860`
- 整列方法: `sim_step`（baseline と resumed の同一step比較）
- Traceソース: `logs/forklift-unit-trace.csv`（列定義に従う）
- `Δx` 定義: `(baseline - resumed)`、閾値は `|Δx|` に適用
- Traceサンプリング: `HAKO_FORKLIFT_TRACE_EVERY_STEPS=10`（公式エビデンス条件）
- 指標評価は復帰直後区間を含める（運用手順を含む、より厳しい条件）

補足:
- `python.plot_forklift_continuity` は、`Ctrl+C` 等で発生する途中書き込み行を自動スキップします。

### 復旧確認の実測事実

`forklift_unit` の再起動試験で、以下を確認済みです（2026-02-23 実施 / MuJoCo v3.5.0（`MUJOCO_VERSION.txt`）/ state format `v8`）。

- `logs/forklift-unit-recovery.log` に
  - `START restored=yes ... step=4000 ... target_v=0.700000 ... target_lift=0.171200 ...`
- `logs/forklift-unit-run.log` に
  - `Resume control phase=1 target(v,yaw,lift)=(0.7, -0, 0.1712) step=4000`
- 同一step比較（`4010..4860`, 86点）で
  - `delta_vx mean/min/max = 0.0 / 0.0 / 0.0`
  - `delta_target_lift mean/min/max = 0.0 / 0.0 / 0.0`

これにより、少なくとも現行スコープ（フォークリフトサブツリー + 制御状態）では  
**step整列で連続復帰** が成立することを確認しています。

正式resumeエビデンス:
- `evidence/official-resume-2026-02-23-v8/`
- 生ログ一式 + グラフ + `summary.txt` を格納

位置（`pos_x`）:
![Resume Position Overlay](evidence/official-resume-2026-02-23-v8/position_overlay.png)

速度（`body_vx`）:
![Resume Velocity Overlay](evidence/official-resume-2026-02-23-v8/velocity_overlay.png)

加速度（`body_vx` の差分近似）:
![Resume Acceleration Overlay](evidence/official-resume-2026-02-23-v8/acceleration_overlay.png)

---

## 結合テスト（forklift_unit）

目的:
- 同一引数でPython制御を再実行できること
- C++側が保存/復帰できること
- Phase継続（特にPhase2復帰）を確認すること

手順（3ターミナル）:

1. sim
```bash
bash forklift-unit.bash
```

2. control
```bash
FORWARD_GOAL_X=5.0 HOME_GOAL_X=0.0 GOAL_TOLERANCE=0.03 bash control.bash
```
注記:
- `control.bash` は既定で `HAKO_CONTROLLER_MODE=asset`（Hakoniwa assetとしてtick同期）で実行します。
- 旧方式に戻す場合は `HAKO_CONTROLLER_MODE=external bash control.bash` を使用してください。

3. start
```bash
hako-cmd start
```

再開テスト:
1. simを `Ctrl+C` で停止
2. 同じコマンドでsim再起動
3. controlを同じ引数で再実行
4. ログで `restored=yes` / `phase=2` を確認

---

## エビデンス運用（Phase1）

推奨手順:
1. ベースライン（無停止）を実行してグラフ生成後、成果物を退避
2. 停止/再開テストを実行してグラフ生成後、成果物を退避
3. 2つのエビデンスディレクトリを比較

`logs/` の成果物を `evidence/<case_name>/` に `mv` で退避:

```bash
bash evidence/move-logs-to-evidence.bash phase1-baseline-01
bash evidence/move-logs-to-evidence.bash phase1-resume-01
```

各エビデンスに保存されるもの:
- `control-run.log`
- `forklift-unit-run.log`
- `forklift-unit-recovery.log`
- `forklift-unit-trace.csv`
- `forklift-unit-continuity.png`
- `meta.txt`（取得時刻 + MuJoCo version）

グラフの見方は以下に記載しています:
- `Context Save/Restore -> 連続性グラフ（客観評価）`
  - `見方`
  - `実運用での合格目安`

---

## Advanced: RD-light Handoff

RD-light は、フォークリフト資産を対象にした advanced / experimental な単一ノード ownership handoff デモです。  
RD-full ではなく、最終的な制御プレーン意味論は本リポジトリのスコープ外です。

### RD アーキテクチャ上の位置づけ

本リポジトリは RD 制御プレーンそのものを実装する層ではなく、  
**RD が成立するための物理実行基盤（Data Plane 側アセット）** を提供します。

- `hakoniwa-rd-core` の責務:
  - Ownership 遷移（Owner/NonOwner）
  - Epoch / commit-point の確定
  - Bridge 再配線と制御プレーン整合
- `hakoniwa-mujoco-robots` の責務:
  - MuJoCo での高忠実度 EU 実行
  - PDU 入出力
  - コンテキスト退避・復旧（継続実行の前提）

要点:
- 本リポジトリは **RD 完成機能ではなく前提実装** です。
- Ownership 切替や commit-point 成立判定は、引き続き RD コア側の責務です。
- 分散 epoch 保証や bridge 再配線完了確認は扱いません。

### Guarantees / Non-goals / Interfaces

Guarantees:
- MuJoCo EU の Data Plane 実行基盤
- 単一ノード RD-light 最小 handoff
  - ownership release / activation
  - RuntimeContext save/restore による継続実行
  - single-owner 運用
  - standby 非干渉運用

Non-goals:
- RD-full Control Plane 意味論の最終確定
- commit-point のグローバル確定
- 分散ノード間の epoch 整合保証
- `d_max` 保証および drift 修復
- bridge 再配線完了確認

Interfaces:
- PDU インターフェース（RuntimeStatus / RuntimeContext / 各 robot PDU）
- context save/restore payload のコールバック境界
- asset 側 ownership 状態（`owner=yes/no`）と handoff ログ

### RD-light（本リポジトリ実装）

本リポジトリには、単一ノードで ownership handoff を実験するための **RD-light** 実装を含みます。  
これは RD 制御プレーン全体の代替ではなく、MuJoCo アセット側の前提機能を検証するための軽量実装です。

- 目的:
  - 2 つの独立 MuJoCo アセット間で ownership を切替
  - `RuntimeStatus` / `RuntimeContext` による状態受け渡し
  - handoff 後の運動継続（Data Plane 継続性）を確認
- 範囲:
  - 単一ホスト / 単一共有メモリ領域（mmap）内の実験用
  - 同一ホスト内の複数プロセス・複数アセット handoff を対象
  - commit-point 制御や分散再配線の最終意味論は `hakoniwa-rd-core` 側
- 実行入口:
  - `forklift-1.bash`（初期 owner）
  - `forklift-2.bash`（初期 standby）
- 詳細:
  - [`rd-design.md`](rd-design.md)

### Safe Handoff Condition（ユーザ責務）

handoff の実行タイミングはユーザ / 上位層の責務です。  
RD-light は任意タイミングの物理安全性を自動保証しません。

MUST NOT:
- 接触中・衝突中に handoff しない
- 衝突直前に handoff しない
- 把持 / 拘束が有効な状態で handoff しない
- 外部物体との強い接触拘束がある状態で handoff しない

SHOULD:
- 自由空間で handoff する
- 安定運動（加速度小、角速度有界、接触点数低）で handoff する
- シナリオ定義で明示的な安全境界を設計する

### 設計原則: 「決めないことを決める」

本リポジトリは、あらゆる状況の物理完全性を無条件保証する方針を取りません。  
代わりに、**衝突・拘束が強くなる前の安全境界で ownership を委譲する**ことを前提にします。

## Advanced Reading: Hakoniwa Core Summary

このリポジトリは、次の 3 層で読むと理解しやすくなります。

- `hakoniwa-core-cpp`
  - 共有メモリ（`mmap`）と同期のコア
  - `hako-master` が実行状態と PDU 領域を管理
- `hakoniwa-core-pro`
  - asset API、コマンドツール、実行制御
  - 代表 API: `hako_conductor_start()`, `hako_asset_register()`, `hako_asset_start()`, `hako_asset_pdu_read()`, `hako_asset_pdu_write()`
- `hakoniwa-pdu-registry`
  - ROS msg 由来の型・サイズ・変換器アーティファクト
  - バイナリ配置: `[MetaData(24B)] + [BaseData] + [HeapData]`

最低限の読み方:
- `pdu_size` は metadata を含む総サイズ
- config 形式は C++ / Python ともに compact（`pdudef + pdutypes`）
- Python は `hakoniwa-pdu >= 1.3.7` を前提

## Advanced Reading: 設定ルール

### compactのみ（必読）

- C++ シミュレータ / Python コントローラ: **compact JSON**
  - 例: `forklift-unit-compact.json`, `custom-compact.json`, `safety-forklift-pdu-compact.json`
- Python 実行環境: `hakoniwa-pdu >= 1.3.7`

`hakoniwa-pdu` が古い場合、初期化は通っても PDU 解決に失敗することがあります（`channel=-1`, `size=-1`）。

---

## FAQ

### Q1. これはRD（Runtime Delegation）を実装していますか？
A. RD-light は実装済み、RD-full は未実装です。  
本リポジトリが実装するのは Data Plane 上の最小handoff（ownership release/activation、context handoff、single-owner運用）です。  
RD-full制御プレーン（commit-point意味論の最終確定、分散epoch保証、`d_max`保証、bridge再配線完了確認）は範囲外です。

### Q2. commit-pointとMuJoCo保存はどう関係しますか？
A. 現状では、commit-pointと自動保存は連動していません。保存は運用上のautosaveです。将来的には、RDのcommit-point到達時に保存をフックする設計を想定しています。
接続点は明示的で、RD制御面で commit-point 到達 -> data plane 側で context-save API 呼び出し、という形を想定しています。
したがって現状は、実用的継続性の実装であり commit-point意味論の最終実装ではありません。

### Q3. d_maxとの整合はどうなりますか？
A. 本リポジトリは単一ノード内の物理ExecutionUnit実装です。`d_max` 保証は上位RD層の意味論であり、本リポジトリ単体では `d_max` を保証しません。

### Q4. なぜ完全世界スナップショットではないのですか？
A. 段階的設計のためです。現在は「フォークリフトサブツリー」と「制御状態」（＋選択したMuJoCo動力学バッファ）を保存対象としています。外部オブジェクト（棚・荷物）は将来の拡張対象です。

### Q5. MuJoCoの内部solver状態は保存していますか？
A. 一部を保存しています。保存対象は `qpos` / `qvel` / `qacc` / 制御状態に加えて、`act`、`ctrl`、`qacc_warmstart`、`qfrc_applied`、`xfrc_applied` です。ただし、MuJoCo内部状態の完全スナップショットではないため、復帰後にsolver内部の一時状態差分が残る可能性はあります。
これらは「接触・駆動の再開安定化」を目的に選定したバッファであり、MuJoCo内部状態の完全再現を狙うものではありません。

### Q6. 物理的に完全に連続しますか？
A. 保証しません。本実装は「意味論的継続」を目的とし、ミクロな物理連続性までは保証しません。

### Q7. モデル変更やMuJoCoバージョン変更時に復旧できますか？
A. 保証しません。stateは同一モデルXML・同一MuJoCoバージョンを前提に復元します。
これは「PDU疎結合」と矛盾しません。  
PDU疎結合は**プロセス間インターフェース契約**の話であり、ここで要求している一致条件は**物理エンジン内部状態の互換性**の話です。責務レイヤが異なります。

### Q8. legacy/compact混在は設計上の問題では？
A. 現在は compact 統一方針です。本リポジトリから legacy 設定・例は削除しました。

### Q9. HLAやFMIと何が違いますか？
A. 本設計は、PDUによる明示的データ契約、ExecutionUnit単位でのOwnership管理、commit-point意味論を中核にしています。マスターアルゴリズム依存の並列同期設計とは立ち位置が異なります。

### Q10. 復帰境界の入力制御はどう扱っていますか？
A. controller は Hakoniwa asset（tick同期）として動作し、復帰境界でも即時に入力を反映します。  
したがって「hold時間を調整する」運用は採用していません。

### Q11. なぜ Phase2（荷物・棚・複雑接触込み）の復旧エビデンスを作らないのですか？
A. 現時点の目的が、RD前提となる **ExecutionUnit の継続性（Phase1）** の成立確認だからです。
本リポジトリは Data Plane 側の継続実装を提供します。RD制御面（commit-point連動保存、Ownership移譲規則、Epoch整合の受け渡し時点）と結合しない Phase2 は、評価軸が不安定になり、責務境界が崩れます。
したがって Phase2 非対応は「価値否定」ではなく、保証範囲を先に固定するための境界設計です。

Phase2（荷物・棚・複雑接触）は、以下が揃った段階で実施します:
- RD制御面からの commit-point 到達フックによる context-save
- Handoff仕様（どの時点の状態を正とみなすか）の確定
- 外部オブジェクトを含む退避境界仕様の確定

したがって現行スコープでは Phase1 を公式エビデンスとし、Phase2 はスコープ外（将来課題）として明示します。

Note: Phase1エビデンスは、定義済みスコープでの意味論的継続性を示すものであり、完全世界の物理決定論を主張するものではありません。

### Q12. 「ガラスの城」では？（特殊条件でしか成立しないのでは？）
A. この懸念は妥当です。  
本リポジトリは、汎用の「何でも連続復元」ではなく、**範囲を限定したData Plane実装**です。

- 成立条件:
  - single-owner運用
  - Safe Handoff Condition（接触・衝突直前・拘束中を避ける）
  - 同一モデルXML / 同一MuJoCoバージョン
- できること:
  - その範囲内での ownership handoff と継続実行（Phase1）
- まだ非保証:
  - 接触・外部物体込みでの汎用連続性（Phase2）
つまり本リポジトリは「特殊条件の偶然動作」ではなく、**適用範囲を明示して成立性を保証する段階設計**です。

本FAQは現時点の実装スコープに基づく回答です。RD意味論および分散拡張に関する最終定義は [Hakoniwa Design Docs](https://github.com/hakoniwalab/hakoniwa-design-docs) を参照してください。

---

## サンプル一覧

- `src/main_for_sample/forklift/main.cpp` フォークリフト基本連携
- `src/main_for_sample/forklift/main_unit.cpp` 単体モデル検証向け
- `src/main_for_sample/tb3/main.cpp` TurtleBot3 サンプル（Hakoniwa asset / endpoint / 2D LiDAR）
- `python/tb3_gamepad.py` TurtleBot3 用 Python controller asset（PS4/DualSense）
- `python/lidar_visualizer.py` 汎用 LiDAR 可視化ツール（world view）
- `config/sensors/lidar/lds-01.json` TurtleBot3 LDS-01-like noisy LiDAR profile
- `config/sensors/lidar/lds-02.json` TurtleBot3 LDS-02-like longer-range LiDAR profile
- `config/sensors/lidar/urg-04lx-ug01.json` Hokuyo URG-04LX-UG01-like cleaner LiDAR profile

---

## Roadmap

- Windows実行フローの整備（ビルド/実行/ログ）
- compact統一運用の検証強化（`hakoniwa-pdu` バージョン/PDU解決診断）
- Python controller asset 実装の運用安定化（tick同期）
- 退避対象の段階的拡張（荷物・棚など）
- 復帰整合性チェックの自動化（ログ検査スクリプト）
- handoff可否の可視化（任意ログ: `reason=contact_active` / `near_collision` / `constraint_active`）
- RD（Runtime Delegation）連携に向けたコンテキスト受け渡し設計の具体化
- TODO: 英語版 README.md へ同等の最終文言を同期

---

## ライセンス

MIT License
