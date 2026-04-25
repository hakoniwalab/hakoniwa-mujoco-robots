# hakoniwa-mujoco-robots

[English](README.md) | 日本語

## TL;DR
- 本リポジトリは、**Hakoniwa向けにMuJoCo物理アセットを接続する資産集**です。  
- 単一ノード向け **RD-light** を実装済みです（ownership release/activation、context save/restore、single-owner運用）。  
- 目的は、PDU連携で**C++シミュレータ（MuJoCo）とPython制御**を統合し、分散実験の基盤を作ることです。  
- 重要機能として、**フォークリフト本体＋制御状態のコンテキスト退避・復旧**を実装しています。  
- RD-full制御プレーン（commit-point意味論確定、epoch整合保証、`d_max`保証、bridge再配線完了確認）は本リポジトリの範囲外です。  
- 設定は **C++/Python ともに compact JSON** を使用します（`hakoniwa-pdu >= 1.3.7`）。  
- 最短実行は「3ターミナル（sim / python / `hako-cmd start`）」です。  

## デモ動画
- Runtime handoff デモ（RD-light / フォークリフト2アセット）  
  - [![Watch the demo](https://img.youtube.com/vi/xaJJ1wEgNR8/hqdefault.jpg)](https://www.youtube.com/watch?v=xaJJ1wEgNR8)

### この動画で示していること（意図）
- 左右2つの MuJoCo Viewer は、同一EU（フォークリフト）を担当する2つのAsset Instanceです。
- **前方1m地点を「別世界との境界」として定義**し、そこを handoff point（切替点）として使っています。
- これは強化学習デモではなく、**RD-light の所有権移譲（handoff）デモ**です。
- 通常は片側のみが owner として制御・PDU publish を実行し、もう片側は standby で待機します。
- しきい値到達時に owner が `RuntimeStatus/RuntimeContext` を更新し、standby 側が状態を受け取って owner 化します。
- standby 側は半透明表示・非干渉化され、owner 側のみが有効に動作します（切替後に役割が反転）。
- 左右は**それぞれ独立したMuJoCo物理計算アセット**であり、同一プロセス内の切替ではありません。
- このデモの要点は、独立アセット間で**シームレスに実行権を委譲**し、運動を継続できることです。

### ログの見方
- `ownership release requested`: 現ownerが handoff を要求
- `ownership activated`: 相手側が context 復元して owner 化
- `status step=... owner=yes/no`: その時点のローカル所有権状態
- `dist_to_release`, `dist_to_home`: 切替判定の距離指標

### このデモで主張する範囲
- 本デモは **RD制御プレーンの完全実装**ではなく、MuJoCoアセット側の前提実装（Data Plane）を示します。
- 具体的には、所有権切替時にコンテキストを受け渡して継続実行できることを確認します。

---

## Why（なぜこのリポジトリが必要か）

Hakoniwaは、PDUベースのI/Fで複数プロセス・複数言語・分散構成を接続するシミュレーション基盤です。  
このリポジトリは、その中にMuJoCoを「高忠実度の物理アセット」として組み込みます。

狙いは次の3点です。

- **高忠実度物理の実装層**をHakoniwaに差し込む
- **Python制御（自動操縦/外部ロジック）**と疎結合に連携する
- **実験継続性（退避・復旧）**を担保し、将来のRD（Runtime Delegation）に備える

特にコンテキスト退避・復旧は、以下の前提機能です。

- 長時間実験の途中再開
- 手動停止/再起動の継続
- 障害復旧
- 実行所有権移動（RD）の土台

---

## What（何が入っているか）

- MuJoCoベースのロボットサンプル（フォークリフト、TurtleBot3）
- Hakoniwa連携C++サンプル
- PythonコントローラAPI/サンプル
- Docker実行環境（Ubuntu 24.04）
- フォークリフト向けコンテキスト退避・復旧（状態ファイル + 監査ログ）

### ディレクトリ
- `models/` MuJoCo XMLモデル
- `config/` PDU設定JSON
- `src/` C++シミュレータ実装
- `python/` Python制御コード
- `docker/` Dockerfile/実行スクリプト
- `logs/` 実行ログ（生成物）
- `tmp/` 状態ファイル（生成物）

---

## Architecture

Hakoniwaをハブとして、MuJoCo（C++）とPython制御がPDUで接続されます。

- **Hakoniwa**: 実行同期・PDU基盤
- **MuJoCo C++ Asset**: 物理計算とPDU read/write
- **Python Controller**: 目標値指令（API制御）
- **PDU JSON**: 双方の契約（チャネル・型・サイズ）

```text
+-------------------+         PDU (shared contract)         +----------------------+
| Python Controller |  <---------------------------------->  | MuJoCo C++ Simulator |
| (forklift_* .py)  |                                        | (forklift*_sim)      |
+---------+---------+                                        +----------+-----------+
          |                                                            |
          |                   Hakoniwa runtime                         |
          +--------------------(sync / mmap / PDU)---------------------+
```

---

## 箱庭コア機能サマリー（サンプル読解ガイド）

このリポジトリのサンプルは、次の3層を前提に読むと全体像を掴みやすくなります。

- `hakoniwa-core-cpp`（シミュレーションハブ本体）
  - 共有メモリ（主に `mmap`）上で、時刻同期とPDUバッファ管理を担う中核層
  - `hako-master` が実行状態とPDU領域を管理
- `hakoniwa-core-pro`（運用/API拡張層）
  - アセットAPI、コマンド、実行制御を提供
  - 典型API: `hako_conductor_start()` / `hako_asset_register()` / `hako_asset_start()` / `hako_asset_pdu_read()` / `hako_asset_pdu_write()`
  - `hako-cmd` による `start` などの外部操作はこの層
- `hakoniwa-pdu-registry`（PDU型定義・生成資産層）
  - ROS msg由来のPDU型、サイズ、オフセット、変換コードを生成・管理
  - バイナリは `[MetaData(24B)] + [BaseData] + [HeapData]` のレイアウト
  - MetaData は現行PDU仕様で固定長 24B（参照: [`hakoniwa-pdu-registry` README](thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/README.md)）

### サンプル読解に必要な最小知識

- `pdu_size` は「型そのもののサイズ」ではなく、**PDU総サイズ（MetaData込み）**で扱う
  - 例: `Int32` 本体サイズ 8B でも、設定上は `24 + 8 = 32` を使う
- 設定フォーマット
  - C++/Python側: compact（`pdudef + pdutypes`）
  - Python実行環境: `hakoniwa-pdu >= 1.3.7`
- 実行時共有領域は通常 `/var/lib/hakoniwa/mmap`

### このリポジトリでの追い方（推奨順）

1. `src/main_for_sample/forklift/main_unit.cpp`
   - MuJoCo側（C++アセット）の実行とコンテキスト保存/復帰の本体
2. `python/forklift_simple_auto.py`
   - 目標値ベースの制御ロジック（現在状態に依存しすぎない設計）
3. `config/forklift-unit*.json` と `config/safety-forklift-pdu*.json`
   - PDU契約（どのチャネルを、どのサイズでやり取りするか）

---

## RDアーキテクチャとの位置づけ（現状）

### 要約（3分）

- 本リポジトリは、MuJoCo EU の **Data Plane実装** を提供する。
- **RD-light は実装済み**（ownership release/activation、context handoff、single-owner運用）。
- ここでいう「単一ノード」は、**単一ホスト/単一共有メモリ領域（mmap）** を前提とする。
- したがって、同一ホスト内での **複数プロセス・複数アセット間 handoff** は扱う。
- 一方で、分散ノード間の epoch整合保証、`d_max` 保証、bridge再配線完了保証は扱わない。
- つまり、RD-full Control Plane の最終意味論は `hakoniwa-rd-core` の責務。

### 詳細

本リポジトリは、Runtime Delegation（RD）そのものを実装する層ではなく、  
**RDが成立するための物理実行基盤（Data Plane側アセット）**を提供します。

- `hakoniwa-rd-core` の責務:
  - Ownership遷移（Owner/NonOwner）
  - Epoch/Commit Pointの確定
  - Bridge再配線と制御プレーン整合
- `hakoniwa-mujoco-robots` の責務:
  - MuJoCoでの高忠実度EU実行
  - PDU入出力
  - コンテキスト退避・復旧（実行継続の前提）

要点:
- 本リポジトリは **RD完成機能ではなく前提実装** です。
- Ownership切替やCommit Point成立判定は、引き続きRDコア側の責務です。
- 本リポジトリは **RD-full の制御API（分散epoch保証やcommit-point確定を含む）** は持ちません。  
  ただし RD-light としての最小handoff制御（release/activation）は実装済みです。

### Guarantees / Non-goals / Interfaces

Guarantees（本リポジトリが提供するもの）:
- MuJoCo EU の Data Plane 実行基盤
- 単一ノード RD-light 最小 handoff:
  - ownership release / activation
  - RuntimeContext save/restore による継続実行
  - single-owner運用
  - standby非干渉運用
- レイヤ責務: **Data Plane（本リポジトリ）**

Non-goals（本リポジトリの範囲外）:
- RD-full Control Plane 意味論の最終確定
- commit-point 意味論の最終決定およびグローバル判定権
- 分散ノード間の epoch整合保証
- `d_max` 保証および drift修復
- bridge再配線完了確認
- レイヤ責務: **Control Plane（`hakoniwa-rd-core` / 上位層）**

Interfaces:
- PDUインターフェース（RuntimeStatus / RuntimeContext / 各robot PDU）
- context save/restore payload のコールバック境界
- asset側 ownership状態（`owner=yes/no`）および handoffログ
- レイヤ責務: **Data Plane と Control Plane の接続境界**

### RD設計サマリー（ExecutionUnit / Ownership / commit-point / d_max）

RD（Runtime Delegation）は、分散実行中にExecutionUnit（EU）の実行責任を安全に移すための制御規約です。ExecutionUnit は論理的な実行単位、ExecutionUnitInstance は各ノード上の具体実体であり、同一EUに対して複数Instanceが存在しても Ownership（Owner/NonOwner）は常に一意でなければなりません。

切替は状態遷移に従って明示的に実行され、Epochで世代を識別し、Bridge再配線の完了確認を経て次Ownerを有効化します。重要なのは commit-point が物理同時開始点ではなく、責任と因果境界を意味論的に確定する境界である点です。したがって commit-point と新Ownerの実行開始時刻に差があっても、どの結果がどのOwner/Epochに属するかは曖昧になりません。

時間整合は bounded drift 前提で、設計時上限 d_max（分散経路では `2*d_max`）内で論理時間差を扱います。一方、d_max超過時の自動補正や障害復旧は保証外であり、運用層の判断に委ねる境界設計です。

**Note:** RDは bounded drift（d_max）前提の意味論を提供しますが、d_max超過時の自動修復・障害復旧は本仕様の範囲外です（運用層の判断）。

本READMEは実装/運用ガイド（Informative）です。  
RD意味論の最終定義（Normative）は次を参照してください。

- **Normative**: 仕様（意味論）の最終定義
- **Informative**: 実装・運用ガイド（本README）
- [Hakoniwa Design Docs](https://github.com/hakoniwalab/hakoniwa-design-docs)
- [Core Functions (JA)](https://github.com/hakoniwalab/hakoniwa-design-docs/blob/main/src/architecture/core-functions-ja.md)
- [Glossary (JA)](https://github.com/hakoniwalab/hakoniwa-design-docs/blob/main/src/glossary-ja.md)

### RD-light（本リポジトリ実装）

本リポジトリには、単一ノードで ownership handoff を実験するための **RD-light** 実装を含みます。  
これは RD 制御プレーン全体の代替ではなく、MuJoCoアセット側の前提機能を検証するための軽量実装です。

- 目的:
  - 2つの独立MuJoCoアセット間で ownership を切替
  - `RuntimeStatus` / `RuntimeContext` による状態受け渡し
  - handoff 後の運動継続（データプレーン継続性）を確認
- 範囲:
  - 単一ホスト/単一共有メモリ領域（mmap）内の実験用
  - 同一ホスト内の複数プロセス・複数アセット handoff を対象
  - commit-point 制御や分散再配線の最終意味論は `hakoniwa-rd-core` 側
- 実行入口:
  - `forklift-1.bash`（初期 owner）
  - `forklift-2.bash`（初期 standby）
- 設計詳細:
  - [`rd-design.md`](rd-design.md)
  - クイックリンク:
    - [状態遷移ルール](rd-design.md#rd-state-transition-rules)
    - [コンテキスト仕様](rd-design.md#rd-context-specification)
    - [切替トリガ](rd-design.md#rd-switch-trigger)
    - [実装済みの安定化機能](rd-design.md#rd-stabilization-features)
    - [失敗時動作（現行）](rd-design.md#rd-failure-behavior)
    - [クラス設計（実装反映）](rd-design.md#14-クラス設計実装反映)
    - [未実装 / TODO](rd-design.md#rd-todo)

## Safe Handoff Condition（ユーザ責務）

handoff の実行タイミングはユーザ/上位層の責務です。  
RD-light は任意タイミングの物理安全性を自動保証しません。

本プロジェクトの立ち位置は明確です。  
**複雑な物理シミュレーションをどこまで精密に実施するかの責任主体はユーザ（シナリオ設計者/上位制御）**であり、  
本リポジトリはそのための Data Plane と handoff 機構を提供します。  
したがって、**本当に重要な局面のシミュレーションを開始する前の条件設定（handoff境界、速度条件、接触状態制約など）はユーザ側で定義**します。

MUST NOT:
- 接触中・衝突中に handoff しない
- 衝突直前に handoff しない
- 把持/拘束が有効な状態で handoff しない
- 外部物体との強い接触拘束がある状態で handoff しない
- 実装運用例（判定指標）:
  - `contact_points > 0`
  - `dist_to_collision < threshold`
  - `constraint_active == true`

SHOULD:
- 自由空間で handoff する
- 安定運動（加速度小、角速度有界、接触点数低）で handoff する
- シナリオ定義で明示的な安全境界を設計する
- 実装運用例（判定指標）:
  - `contact_points == 0`
  - `|linear_acc| < a_threshold`
  - `|angular_rate| < w_threshold`
  - `dist_to_collision >= threshold`

違反時の Failure semantics:
- 物理連続性は **非保証**
- 観測されうる現象例: `jump`, `bounce`, `divergence`, 急峻なトルク/速度スパイク

現行デモの位置づけ:
- 前方1m境界は handoff の **安全境界設計例**
- RD-full の安全証明を自動で与えるものではない

### 設計原則: 「決めないことを決める」

本リポジトリは、あらゆる状況の物理完全性を無条件保証する方針を取りません。  
代わりに、**衝突・拘束が強くなる前の安全境界で ownership を委譲する**ことを前提にします。

- 難しい状態（接触中/衝突直前/拘束中）まで抱え込んで保証範囲を広げすぎると、実装・運用は破綻しやすい
- そのため「どこまで保証し、どこからは上位層と運用責務に渡すか」を先に固定する
- これは機能不足ではなく、分散シミュレーションを成立させるための境界設計

---

## 重要な設定ルール

### compactのみ（必読）

- C++シミュレータ / Pythonコントローラ: **compact JSON**
  - 例: `forklift-unit-compact.json`, `custom-compact.json`, `safety-forklift-pdu-compact.json`
- Python実行環境: `hakoniwa-pdu >= 1.3.7`

`hakoniwa-pdu` が古い場合、初期化は通っても PDU 解決に失敗することがあります（`channel=-1`, `size=-1`）。

---

## 前提環境

## 1) hakoniwa-core-pro の導入（必須）

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

## 2) OS別補足

- macOS: `brew install glfw`
- Ubuntu: OpenGL/GLFW関連を導入
```bash
sudo apt-get update
sudo apt-get install -y libgl1 libgl1-mesa-dri libglx-mesa0 mesa-utils libglfw3-dev
```

---

## セットアップ

```bash
git clone https://github.com/toppers/hakoniwa-mujoco-robots.git
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

## Quick Start（最短）

ここでは**ホスト実行**を最短手順として示します（迷わないため）。

ターミナルを3つ用意してください。

1. シミュレータ
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

2. Pythonコントローラ（compact）
```bash
python -m python.forklift_simple_auto config/forklift-unit-compact.json \
  --forward-distance 2.0 --backward-distance 2.0 --move-speed 0.7
```

3. 開始トリガ
```bash
hako-cmd start
```

互換のため `controll.bash` は当面残し、内部で `control.bash` を呼び出します。

---

## How（実行手順の詳細）

## C++サンプル

- 通常フォークリフト:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

- 単体（荷物なし、自動制御検証向け）:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

- TurtleBot3（サンプル骨格、endpoint 統合は今後追加）:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim
```

## Pythonサンプル

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

---

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

## Context Save/Restore（MuJoCoコンテキスト退避・復旧）

## 狙い

- 長時間実験の継続
- 中断/再開の運用性
- 障害復旧
- 将来のRD（所有権移動）に向けた前提機能

## 境界設計（退避対象）

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

## 境界設計（退避対象外）

- 荷物・棚など外部オブジェクト状態
- 外部プロセス側の内部状態（Python内部状態など）

つまり、**完全世界スナップショットではない**ことを明示します。

## 保存/復帰

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

## phase運用

- `phase=2`（帰り）はセッション内でラッチ運用
- 復帰時に保存済み `target_v/target_yaw` も適用し、復帰直後の挙動を安定化

## ログ確認

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
- `src/main_for_sample/tb3/main.cpp` TurtleBot3 サンプル骨格

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
