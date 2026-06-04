# hakoniwa-mujoco-robots

[English](README.md) | 日本語

## TL;DR
- 本リポジトリは、**Hakoniwa 向けの MuJoCo ベース robot simulation assets** を提供します。
- ROS/URDF 由来のロボットモデルを MuJoCo 上で利用できます。TurtleBot3 Burger サンプルを含みます。
- C++ MuJoCo simulator と Python controller / visualizer を **Hakoniwa PDU** で接続できます。
- TurtleBot3 サンプルでは、gamepad 制御と 2D LiDAR の `LaserScan` 互換 PDU 出力が動作します。
- LiDAR ノイズプロファイルを切り替えられます。`LDS-01` 風の noisy profile と、`URG-04LX-UG01` 相当の cleaner profile を含みます。
- フォークリフト sample、context save/restore、RD-light handoff は **上級者向けサンプル** として含まれます。
- 設定は **C++/Python ともに compact JSON** を既定で使います（`hakoniwa-pdu >= 1.6.1`）。

## まず読むところ

最初は以下の入口から見てください。この README の後半には advanced note や実験的な背景説明も含まれます。

| 目的 | 読む / 実行するところ |
| --- | --- |
| リポジトリを build する | [前提環境](#前提環境)、続いて [セットアップ](#セットアップ) |
| ローカル環境を診断する | [`./doctor.bash`](#環境診断) |
| TurtleBot3 を gamepad + LiDAR で動かす | [Quick Start: TurtleBot3 + 2D LiDAR](#quick-start-turtlebot3--2d-lidar) |
| forklift sample を動かす | [Quick Start: Forklift](#quick-start-forklift) |
| 小さい sensor example を試す | [examples/sensors/README.md](examples/sensors/README.md) |
| color-camera の PNG capture を試す | [examples/sensors/color_camera/README.md](examples/sensors/color_camera/README.md) |
| MJCF の position / velocity actuator を試す | [examples/actuators/joint/README.md](examples/actuators/joint/README.md) |
| RD-light / context save-restore を読む | [docs/forklift-context-rd-ja.md](docs/forklift-context-rd-ja.md) |

現在の standalone examples:

```text
examples/sensors/ultrasonic/        ultrasonic range sensor + viewer ray
examples/sensors/color_camera/      RGB camera sensor + PNG capture
examples/actuators/joint/           MuJoCo position / velocity joint actuators
```

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
- 詳細は [docs/forklift-context-rd-ja.md](docs/forklift-context-rd-ja.md) と [rd-design.md](rd-design.md) を参照してください

---

## What This Repository Provides

- MuJoCo robot models
- Hakoniwa 連携 C++ simulators
- Python controllers
- Python visualizers
- PDU config files
- sensor config files
- `src/sensors/` 配下の再利用可能な sensor components
- `src/actuator/` 配下の再利用可能な actuator components
- `examples/` 配下の単機能 standalone examples
- フォークリフト向け context save/restore
- RD-light handoff デモ（advanced example）

### ディレクトリ
- `models/` MuJoCo XMLモデル
- `config/` PDU設定JSON
- `config/sensors/` LiDAR / sensor spec JSON
- `config/actuator/` actuator binding JSON
- `src/` C++シミュレータ実装
- `src/sensors/` sensor 実装と PDU 変換
- `src/actuator/` joint actuator 実装
- `python/` Python制御コード / visualizer
- `examples/` 個別機能を試す小さな standalone examples
- `tests/sensors/` sensor unit / smoke tests
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

### 2) hakoniwa-pdu-endpoint の導入（必須）

このリポジトリは、install 済みの C++ `hakoniwa-pdu-endpoint` package に link します。

```bash
git clone https://github.com/hakoniwalab/hakoniwa-pdu-endpoint.git
cd hakoniwa-pdu-endpoint
bash build.bash
sudo bash install.bash
```

`/usr/local/hakoniwa` 以外へ install した場合:

```bash
export HAKONIWA_PDU_ENDPOINT_ROOT=/path/to/hakoniwa-pdu-endpoint/install
```

Hakoniwa core も `/usr/local/hakoniwa` 以外にある場合:

```bash
export HAKONIWA_CORE_ROOT=/path/to/hakoniwa-core-pro/install
```

### 3) hakoniwa-pdu Python package の導入（Python tools では必須）

`python/` 以下の controller / visualizer は `hakoniwa_pdu` module を使います。

```bash
python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

導入確認:

```bash
python -m pip show hakoniwa-pdu
```

以下のような Python コマンドを使う場合に必要です:

```bash
python python/tb3_gamepad.py
python -m python.forklift_simple_auto config/forklift-unit-compact.json
python python/lidar_visualizer.py
```

### 4) OS別補足

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
./doctor.bash
./build.bash
```

- MuJoCoバージョンは `MUJOCO_VERSION.txt` で管理します。
- `./build.bash` は CMake 実行前に preflight check を行い、`hakoniwa-core-pro`、`hakoniwa-pdu-endpoint`、`glfw3` など不足している前提を表示します。
- 独自環境で一時的に preflight check を回避したい場合は、`HAKO_SKIP_PREFLIGHT=1 ./build.bash` を使えます。
- クリーンビルド:
```bash
./build.bash clean
```

## 環境診断

新しい machine で setup する場合は、`./build.bash` の前に実行してください。

```bash
./doctor.bash
```

`doctor.bash` は build せずに、以下の前提を確認します。

- CMake / Git
- `hakoniwa-core-pro` package config
- `hakoniwa-pdu-endpoint` package config
- Python tools 用の `hakoniwa-pdu >= 1.6.1`
- `glfw3`
- `MUJOCO_VERSION.txt`

表示された failure を解消してから `./build.bash` を実行してください。

### Windows (MSVC + PowerShell)

Windows 版は、`hakoniwa-core-pro` と `hakoniwa-pdu-endpoint` を先に Windows 上で build/install してから使う前提です。

前提:
- `hakoniwa-core-pro` が `C:\project\hakoniwa-core-pro\install` のような prefix に install 済み
- `hakoniwa-pdu-endpoint` が `C:\project\hakoniwa-pdu-endpoint\install` のような prefix に install 済み
- `vcpkg` の Windows package が `C:\project\vcpkg\installed\x64-windows` に入っている
- 可能なら Windows PowerShell から直接ビルドする

PowerShell:

```powershell
.\build-win.ps1 -Clean `
  -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install `
  -HakoniwaPduEndpointRoot C:\project\hakoniwa-pdu-endpoint\install `
  -ExtraPrefixPaths C:\project\vcpkg\installed\x64-windows
```

WSL / Git Bash ラッパー:

```bash
./build-win.bash
```

メモ:
- `build-win.ps1` は CMake 実行前に preflight check を行い、install root の未指定、package config 不足、`glfw3` / `vcpkg` prefix 情報不足を表示します。
- 既存の Unix ビルドと同じく、`-S src` を使って configure します。
- `HakoniwaCoreRoot` は `HAKONIWA_INSTALL_PREFIX` に渡されます。
- `HakoniwaPduEndpointRoot` は `HAKONIWA_PDU_ENDPOINT_PREFIX` に渡されます。
- `ExtraPrefixPaths` は、`glfw3` や `hakoniwa_pdu_endpoint` の依存解決に使います。
- `HakoniwaPduEndpointRoot` には `build-win` や `build-shared` ではなく、`cmake --install` 済みの install root を指定してください。`hakoniwa-pdu-endpoint` の build tree 直下にある `hakoniwa_pdu_endpointConfig.cmake` は、このリポジトリからそのままは使えません。
- `hakoniwa-pdu-endpoint` を自分で build した場合は、先に install tree を作ってください:

```powershell
cmake --install C:\project\hakoniwa-pdu-endpoint\build-win --config Release --prefix C:\project\hakoniwa-pdu-endpoint\install
```

- 成功時は `build-win/Release/mujoco-common.lib` が生成されます。
- 成功時は `build-win/sensors/Release/msensors.lib` が生成されます。
- 成功時は `build-win/main_for_sample/forklift/Release/forklift_sim.exe` が生成されます。
- 成功時は `build-win/main_for_sample/forklift/Release/forklift_unit_sim.exe` が生成されます。
- 成功時は `build-win/main_for_sample/tb3/Release/tb3_sim.exe` が生成されます。
- Windows では、`hakoniwa_pdu_endpoint.dll` などの実行時 DLL を build 後に各 `.exe` の隣へコピーします。
- `forklift_simulation_loop.obj: Permission denied` が出る場合はコードではなく Windows のファイルロックです。`MSBuild.exe`、`cl.exe`、`devenv.exe` を閉じて再ビルドしてください。
- Python の endpoint binding は C++ サンプルとは別に build されます。`python/tb3_gamepad.py` を使う場合は、vendored `thirdparty/hakoniwa-pdu-endpoint` の Python runtime を Hakoniwa core 対応で build してください:

```powershell
cd .\thirdparty\hakoniwa-pdu-endpoint
.\build-python-win.ps1 `
  -Clean `
  -BuildNative `
  -BuildFfi `
  -EnableHakoniwaCore `
  -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install `
  -BuildDirName build-win `
  -Configuration Release `
  -PythonCommand python `
  -ToolchainFile C:\project\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -VcpkgTriplet x64-windows `
  -Platform x64
```

- `python/tb3_gamepad.py` は `thirdparty/hakoniwa-pdu-endpoint/python` を優先して import するため、site-packages に古い `hakoniwa-pdu-endpoint` が入っていても、ローカルで build した runtime を使えます。

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

### Standalone examples

- ultrasonic sensor example:
```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

- color camera sensor example（`i/k/j/l` でカメラ移動、viewer または terminal で `s` キーを押すと PNG 出力）:
```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

- joint actuator example（MuJoCo viewer 上で `a/d` が position target、`j/l` が velocity target）:
```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

現在の example 一覧は [examples/README.md](examples/README.md) を参照してください。

---

## TurtleBot3 2D LiDAR

TurtleBot3 Burger サンプルには、MuJoCo ベースの 2D LiDAR 実装が含まれます。

注記:
- `models/tb3/turtlebot3_burger_world.xml` は現在、外部 mesh ではなく body / wheel / LiDAR housing を primitive geom で表現しています。
- これは Windows 実行時に、環境依存の絶対 mesh パスでモデルロードが失敗するのを避けるためです。
- 車輪 actuator は MuJoCo の `<velocity>` actuator です。gamepad 入力は左右 wheel angular velocity target に変換され、`JointActuatorImpl` 経由で書き込まれます。
- 発進、停止、ヨー回転をなめらかにするため、linear velocity target と yaw-rate target をレート制限してから wheel angular velocity に変換しています。既定値には `HAKO_TB3_MAX_YAW_RATE=1.2`、`HAKO_TB3_MAX_LINEAR_ACCELERATION=0.1`、`HAKO_TB3_MAX_YAW_ACCELERATION=0.5`、`HAKO_TB3_COMMAND_DEADZONE=0.1` が含まれます。
- 後部 caster が駆動輪の挙動を支配しないよう、caster の摩擦は低めにしています。

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

## Camera Depth Status

camera / depth / RGBD sensor の実装は `include/sensors/camera/` と `src/sensors/camera/` にあります。

- depth 変換は、MuJoCo offscreen path において `mjr_readPixels` が OpenGL-style depth buffer を返す前提で実装しています。
- clip plane は `vis.map.znear/zfar * stat.extent` から実効距離を計算して使います。
- `config/sensors/camera/*.json` の camera / depth / RGBD / multicamera profile は C++ config として読み込めます。
- profile の構造は `config/sensors/schema/` にあり、JSON loader は既存の `LoadConfig(config)` validation を再利用します。
- camera の unit test は `tests/sensors/camera/unit/` にあり、config loader、PDU converter、local depth encoding をカバーします。
- これらの unit test は OpenGL render context を必要とせず、CI 実行を前提にしています。
- 現在の経路は、固定カメラ + box シーンで `0.2`、`0.5`、`1.0`、`2.0`、`5.0`、`9.0` m の smoke test を通過しています。
- あわせて、複数の画面位置、複数の horizontal FOV、clip による NaN マスクも確認済みです。
- render smoke test は `tests/sensors/camera/smoke/` にあり、MuJoCo + OpenGL context が必要なため、ローカル手動または専用CI向けです。
- ただし、斜め面、極端なカメラ設定、別の depth-map convention など、任意シーン全般に対して完全検証済みとはまだ言いません。

## Sensor Components And Examples

再利用可能な sensor components は `src/sensors/` にあります。JSON profile は `config/sensors/`、schema は `config/sensors/schema/` にあります。

現在の主な sensor 領域:
- camera / depth / RGBD / multicamera
- color camera PNG example
- 2D LiDAR
- ultrasonic range
- IMU
- joint state
- odometry
- TF
- noise helpers
- debug ray visualization

standalone examples は、TurtleBot3 や forklift の大きなデモより小さく、個別機能を単体で確認するためのものです。
- [examples/README.md](examples/README.md)
- [examples/sensors/README.md](examples/sensors/README.md)
- [examples/sensors/ultrasonic/README.md](examples/sensors/ultrasonic/README.md)
- [examples/sensors/color_camera/README.md](examples/sensors/color_camera/README.md)
- [examples/actuators/README.md](examples/actuators/README.md)
- [examples/actuators/joint/README.md](examples/actuators/joint/README.md)

sensor unit tests は任意の build target です。
```bash
cmake -S src -B src/cmake-build -DHAKO_BUILD_SENSOR_TESTS=ON
cmake --build src/cmake-build --target run_sensor_unit_tests
```

camera render smoke tests は MuJoCo / OpenGL runtime が必要です。
```bash
cmake -S src -B src/cmake-build -DHAKO_BUILD_CAMERA_SMOKE_TESTS=ON
cmake --build src/cmake-build --target camera_smoke_tests
```

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

## FAQ

### Q1. 最初に何を実行すればよいですか？

まず環境診断を実行してください。

```bash
./doctor.bash
```

`FAIL` が出た項目を直してから、build します。

```bash
./build.bash
```

### Q2. `hakoniwa-pdu` を install したのに Python が失敗します。

`pip` と `python` が別の Python 環境を見ている可能性があります。実行に使う Python と同じ command で確認してください。

```bash
python -m pip show hakoniwa-pdu
python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

`./doctor.bash` は確認対象の Python executable も表示します。

### Q3. CMake が `hakoniwa-core-pro` や `hakoniwa-pdu-endpoint` を見つけられません。

まず両方を install してから、`./doctor.bash` を再実行してください。

`/usr/local/hakoniwa` 以外に install している場合は、以下を設定します。

```bash
export HAKONIWA_CORE_ROOT=/path/to/hakoniwa-core-pro/install
export HAKONIWA_PDU_ENDPOINT_ROOT=/path/to/hakoniwa-pdu-endpoint/install
```

### Q4. CMake が `glfw3` を見つけられません。

GLFW を install してください。

macOS:

```bash
brew install glfw
```

Ubuntu:

```bash
sudo apt-get install -y libglfw3-dev
```

### Q5. color-camera example の PNG はどこに出力されますか？

既定では、command を実行した directory から見た `./camera_color_sample.png` に出力します。第3引数で明示的な出力 path も指定できます。

### Q6. すべての example で MuJoCo viewer が必要ですか？

いいえ。robot demo は既定で viewer を使い、standalone sensor / actuator examples は viewer で確認しやすいように作っています。一方、unit test や config check の一部は viewer なしで動きます。

macOS の Docker では headless mode を使ってください。

```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

## Advanced Topics

forklift context save/restore と RD-light の詳細は、トップ README から分離しました。

- [Forklift Context Save/Restore と RD-light](docs/forklift-context-rd-ja.md)
- [RD design notes](rd-design.md)

トップ README は入口、setup、sample commands に絞ります。restore evidence、RD-light handoff、continuity evaluation workflow が必要な場合は advanced documents を参照してください。

## サンプル一覧

- `src/main_for_sample/forklift/main.cpp` フォークリフト基本連携
- `src/main_for_sample/forklift/main_unit.cpp` 単体モデル検証向け
- `src/main_for_sample/tb3/main.cpp` TurtleBot3 サンプル（Hakoniwa asset / endpoint / 2D LiDAR）
- `python/tb3_gamepad.py` TurtleBot3 用 Python controller asset（PS4/DualSense）
- `python/lidar_visualizer.py` 汎用 LiDAR 可視化ツール（world view）
- `examples/README.md` standalone example 一覧
- `examples/sensors/ultrasonic/README.md` ultrasonic sensor example
- `examples/sensors/color_camera/README.md` color camera PNG example
- `examples/actuators/joint/README.md` MJCF-native position / velocity joint actuator example
- `src/sensors/` 再利用可能な sensor components と PDU 変換 helper
- `config/sensors/lidar/lds-01.json` TurtleBot3 LDS-01-like noisy LiDAR profile
- `config/sensors/lidar/lds-02.json` TurtleBot3 LDS-02-like longer-range LiDAR profile
- `config/sensors/lidar/urg-04lx-ug01.json` Hokuyo URG-04LX-UG01-like cleaner LiDAR profile
- `config/sensors/ultrasonic/lego-spike-distance-sensor.json` standalone example で使う ultrasonic range sensor profile
- `config/sensors/color_camera/simple-color-camera.json` PNG example で使う color camera profile

---

## Contributing

- issue / pull request の前に [CONTRIBUTING.md](CONTRIBUTING.md) を読んでください。
- coding agent は [AGENTS.md](AGENTS.md) も読んでください。
- setup failure の issue には `./doctor.bash` の出力を含めてください。
- viewer example の変更では、manual check の有無を書いてください。

---

## Roadmap

- Windows実行フローの整備（ビルド/実行/ログ）
- compact統一運用の検証強化（`hakoniwa-pdu` バージョン/PDU解決診断）
- Python controller asset 実装の運用安定化（tick同期）
- 退避対象の段階的拡張（荷物・棚など）
- 復帰整合性チェックの自動化（ログ検査スクリプト）
- handoff可否の可視化（任意ログ: `reason=contact_active` / `near_collision` / `constraint_active`）
- RD（Runtime Delegation）連携に向けたコンテキスト受け渡し設計の具体化

---

## ライセンス

MIT License
