# hakoniwa-mujoco-robots

本リポジトリは、箱庭(Hakoniwa)シミュレーションフレームワーク向けに作成した
MuJoCoベースのロボットアセット集です。フォークリフトをはじめとした複数の
モデルを提供し、Hakoniwa環境と連携することでリアルな物理シミュレーションを
実現します。

## ディレクトリ構成

- `models/` … MuJoCo 用のXMLモデルファイル
- `config/` … 各ロボットのPDU(入出力データ)設定
- `src/` … C++によるシミュレーション実装およびサンプルプログラム
- `python/` … フォークリフト操作用APIやゲームパッド制御スクリプト

## セットアップ

本プロジェクトをビルドするには、C++コンパイラ、CMake、Gitがインストールされている必要があります。

### WSL2 Ubuntu (22.04 / 24.04)の場合

OpenGL/GLX/Mesa関連のライブラリをインストールします。

```bash
sudo apt-get update
sudo apt-get install -y libgl1 libgl1-mesa-dri libglx-mesa0 mesa-utils
sudo apt-get install -y libglfw3-dev
```

### macOSの場合

macOSでは、描画に必要なライブラリ `glfw` をインストールしてください。MuJoCo本体はビルド時に自動ダウンロードされます。

**1. GLFWのインストール**

[Homebrew](https://brew.sh/index_ja) を使用して、描画に必要なライブラリ `glfw` をインストールします。

```bash
brew install glfw
```

### Windowsの場合

TODO


## リポジトリのクローンとサブモジュールの初期化

まず、リポジトリをクローンし、依存するサブモジュールを初期化します。

```bash
git clone https://github.com/toppers/hakoniwa-mujoco-robots.git # (ご自身のフォークしたリポジトリURLに置き換えてください)
cd hakoniwa-mujoco-robots
git submodule update --init --recursive
```

> 補足：`MUJOCO_VERSION.txt` を編集することで、使用する MuJoCo のバージョンを指定できます。

## ビルド手順

各OSのセットアップが完了したら、以下のコマンドでプロジェクトをビルドします。

```bash
./build.bash
```

> **Note:** MuJoCoライブラリは Linux / macOS / Windows でビルド時に自動ダウンロードされます。

### 3. ビルド成果物

ビルドが成功すると `src/cmake-build/` 以下にサンプル実行ファイル `forklift_sim` が生成されます。

### ビルドのクリーン

ビルド生成物を削除する場合は、`clean`オプションを使用します。
```bash
./build.bash clean
```

## Docker (Ubuntu 24.04)

`docker/` 配下に、本リポジトリ向けの Ubuntu 24.04 ベース実行環境を用意しています。
Dockerイメージ作成時に `hakoniwa-core-pro` を GitHub から取得してソースビルド/インストールするため、初回ビルドには時間がかかります。

> 注意:
> - Ubuntu + Docker は viewer(GUI) 利用をサポートします。
> - macOS + Docker は viewer(GUI) をサポートしません（headless実行のみサポート）。
> - macOSでGUI表示したい場合は、Dockerではなくホスト(macOS)で実行してください。

### イメージ作成

```bash
bash docker/create-image.bash
```

### コンテナ起動

```bash
bash docker/run.bash
```

### コンテナ内ビルド

```bash
bash build.bash
```

### Docker内での最小自動操縦サンプル実行

ターミナルを3つ用意して、同一コンテナ内で以下を実行します。

1. ターミナル1（シミュレータ）:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

2. ターミナル2（Python自動操縦）:
```bash
python -m python.forklift_simple_auto config/custom.json
```
※ Python側は当面 legacy 形式（`custom.json` / `forklift-unit.json` / `safety-forklift-pdu.json`）を使用してください。
compact 形式（`*-compact.json`）は C++ シミュレータ側で使用します。

`forklift_simple_auto.py` 向けの単体モデル版（荷物なし）:

```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

フォークリフト状態の保存/再開（C++サンプル）:
- `forklift_sim` / `forklift_unit_sim` は、フォークリフト本体の状態（位置・姿勢・リフト高さ・各速度・各加速度）を自動保存します。
- 次回起動時、保存ファイルがあればそこから自動再開します（荷物・棚などの状態は復元対象外）。
- 保存先は `HAKO_FORKLIFT_STATE_FILE` で変更可能（未指定時は `./tmp/hakoniwa-forklift.state` または `./tmp/hakoniwa-forklift-unit.state`）。
- 保存間隔は `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`（シミュレーションstep数、既定: `1000`）で変更可能。
- 共通実装クラスは `include/hakoniwa_mujoco_context.hpp` の `HakoniwaMujocoContext` です。

例:
```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-demo.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=500 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

```bash
python -m python.forklift_simple_auto config/forklift-unit.json
```

3. ターミナル3（シミュレーション開始トリガ）:
```bash
hako-cmd start
```

### macOS + Docker の実行方針

macOS + Docker は headless で利用してください。

```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

## サンプルの実行

本サンプルは、**C++製のシミュレータ**と**Python製のコントローラ**を連携させて動作させます。それぞれを別のターミナルで起動する必要があります。

### 前提条件

1.  **Hakoniwaコアのセットアップ**
    シミュレーションの中核を担うHakoniwaコアライブラリのセットアップが必須です。
    詳細は **[hakoniwa-core-pro のREADME](https://github.com/hakoniwalab/hakoniwa-core-pro/blob/main/README.md)** を参照してインストールを完了してください。これにより、Pythonの `hakopy` ライブラリも同時にセットアップされます。
    既定のインストール先は `/usr/local/hakoniwa` です（必要なら `HAKONIWA_INSTALL_PREFIX` で変更可能）。
    例:
    Linux:
    ```bash
    git clone --recursive https://github.com/hakoniwalab/hakoniwa-core-pro.git
    cd hakoniwa-core-pro
    bash build.bash
    bash install.bash
    ```
    インストール後、必要に応じてライブラリ検索パスを設定してください。
    ```bash
    export PATH=/usr/local/hakoniwa/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$LD_LIBRARY_PATH
    ```
    macOS:
    ```bash
    git clone --recursive https://github.com/hakoniwalab/hakoniwa-core-pro.git
    cd hakoniwa-core-pro
    bash build.bash
    bash install.bash
    ```
    インストール後、必要に応じてライブラリ検索パスを設定してください。
    ```bash
    export PATH=/usr/local/hakoniwa/bin:$PATH
    export DYLD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$DYLD_LIBRARY_PATH
    ```

2.  **Python追加ライブラリのインストール（ゲームパッド操作時のみ）**
    ゲームパッド操作を使う場合のみ `pygame` が必要です。`pip`でインストールしてください。
    ```bash
    pip install pygame
    ```

3.  **ゲームパッドの接続（手動操作時のみ）**
    手動操作を試す場合は、事前にPCにゲームパッドを接続しておいてください。

### 実行手順

3つのターミナル（以降、**ターミナル1**, **ターミナル2**, **ターミナル3**と呼びます）を用意してください。

**1. シミュレータの起動 (ターミナル1)**

まず、C++でビルドされたシミュレータを起動します。
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```
実行後、MuJoCoのシミュレーションGUIウィンドウが立ち上がり、フォークリフトが表示されます。この時点ではまだ動きません。

**2. コントローラの起動 (ターミナル2)**

次に、Python製のコントローラを起動してフォークリフトを動かします。自動操縦（方法A）を推奨します。
Pythonコントローラで指定する設定ファイルは、現時点では legacy 形式を使用してください。

#### 方法A: APIで自動制御する（推奨）

`ForkliftAPI`を利用して、あらかじめプログラムされた動作を自動で実行します。このサンプルは、荷物をピックアップして棚に運ぶまでの一連の動作を行います。
```bash
python -m python.forklift_api_control config/safety-forklift-pdu.json config/monitor_camera_config.json
```
実行すると、フォークリフトが定義されたミッションを自動で開始します。

最小構成の自動操縦サンプル（monitor camera不要）:
```bash
python -m python.forklift_simple_auto config/custom.json
```
注: `custom-compact.json` / `forklift-unit-compact.json` / `safety-forklift-pdu-compact.json` は C++ シミュレータ向けです。
Python側で使用すると入力PDUが正しく処理されない場合があります。
このサンプルは、前進・旋回・リフト上下の基本操作のみで構成されるため、最初の動作確認に向いています。
移動距離を指定する例:
```bash
python -m python.forklift_simple_auto config/forklift-unit.json --forward-distance 1.5 --backward-distance 1.5 --turn-degree -90
```
速度も指定する例:
```bash
python -m python.forklift_simple_auto config/forklift-unit.json --forward-distance 1.5 --backward-distance 1.5 --move-speed 0.7
```
さらに速くしたい場合（シミュレータ側ゲイン）:
```bash
HAKO_FORKLIFT_MOTION_GAIN=0.4 ./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

#### 方法B: ゲームパッドで手動操作する（任意）

ゲームパッドを使ってリアルタイムにフォークリフトを操作します。
```bash
python -m python.forklift_gamepad config/custom.json
```
このコマンドを実行すると、ゲームパッドの入力がシミュレータに送られ、フォークリフトを自由に動かせるようになります。

**3. シミュレーション開始 (ターミナル3)**

最後に、以下を実行してシミュレーションを開始します。
```bash
hako-cmd start
```

## 結合テスト（forklift_unit）

### 目的

- Pythonコントローラが同一引数で目標値を送れることを確認する
- MuJoCo（C++）側がフォークリフト状態と制御状態を保持し、再起動後に再開できることを確認する
- 対象は `forklift_unit_sim`（フォークリフト単体）とし、荷物/棚は対象外とする

### テスト手順

ターミナルを3つ使用します。

1. ターミナル1（C++シミュレータ）
```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```
または:
```bash
bash forklift-unit.bash
```

2. ターミナル2（Pythonコントローラ）
```bash
python -m python.forklift_simple_auto config/forklift-unit.json \
  --forward-distance 2.0 \
  --backward-distance 2.0 \
  --move-speed 0.7
```
または:
```bash
bash controll.bash
```
絶対目標モード（現在位置から逆算してグローバル目標へ移動）:
```bash
FORWARD_GOAL_X=5.0 HOME_GOAL_X=0.0 GOAL_TOLERANCE=0.03 bash controll.bash
```

3. ターミナル3（開始トリガ）
```bash
hako-cmd start
```

### 再開確認テスト

1. いったん `forklift_unit_sim` を停止する（`Ctrl+C`）
2. 同じコマンドで `forklift_unit_sim` を再起動する
3. ログに以下が出ることを確認する
   - `Resume forklift state from: ...`
   - `Resume control phase=...`
4. Pythonコントローラは同じ引数で再実行し、継続して制御できることを確認する

### ログ確認（復旧判定）

`logs/` 配下に以下のログが出力されます。

- `logs/forklift-unit-run.log` : C++シミュレータ実行ログ
- `logs/control-run.log` : Pythonコントローラ実行ログ
- `logs/forklift-unit-recovery.log` : 復旧監査ログ（初期位置・phase・target・step）

復旧判定は `logs/forklift-unit-recovery.log` の `START` 行で確認します。

- `restored=yes` なら保存状態から復旧
- `restored=no` なら新規開始
- 併せて `pos=...`, `phase=...`, `target_v=...`, `target_yaw=...`, `target_lift=...`, `step=...` を確認

### Phase2復帰の成功判定（推奨チェック）

`Phase2`（帰りフェーズ）で停止して再起動した場合、以下が揃っていれば成功です。

1. `logs/forklift-unit-recovery.log` に 2 回目の開始行として
   - `START restored=yes ... phase=2 ... target_v=-0.700000 ...`
2. `logs/forklift-unit-run.log` に
   - `Resume control phase=2 ...`
3. 2回目開始後の `AUTOSAVE` が `phase=2` を維持している

### ログの追記仕様（tee -a）

`forklift-unit.bash` / `controll.bash` はどちらも `tee -a` でログへ追記します。
そのため、1回目と2回目の実行結果は同じファイルに連続して残ります。

- 1回目/2回目の境界は `START` 行（`restored=no` / `restored=yes`）で判定してください。
- 必要に応じてテスト前に `logs/*.log` を退避または削除してから実行してください。

## コンテキスト退避の設計と狙い

### 狙い

- 再起動後にシミュレーションを継続できるようにする
- 長時間実験を中断/再開しやすくする
- `phase` を含む制御状態の再現性を高める

### 退避対象

`HakoniwaMujocoContext`（`include/hakoniwa_mujoco_context.hpp`）で、主に以下を保存します。

- フォークリフト本体の状態（位置・姿勢・リフト高さ）
- 各速度・各加速度
- 制御状態（`phase`, `target_v`, `target_yaw`, `target_lift`, `step`）

### 退避対象外

- 荷物・棚など外部オブジェクトの状態
- 実験ごとの補助プロセス状態（外部Pythonプロセスの内部状態など）

### 保存/復帰方針

- 保存:
  - 定期 autosave（`HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`）
  - 終了時保存
  - 保存先は `HAKO_FORKLIFT_STATE_FILE`（未指定時は `./tmp/...`）
- 復帰:
  - 起動時に保存ファイルがあれば復帰（`restored=yes`）
  - 保存ファイルがなければ原点開始（`restored=no`）

### phase運用方針

- `phase=2`（帰りフェーズ）をセッション内でラッチし、不要な反転を抑制
- 復帰時は保存済みの `target_v/target_yaw` も初期適用し、復帰直後の挙動を安定化

### 監査ログ

- `logs/forklift-unit-recovery.log` に `START / AUTOSAVE / END` を出力
- 復帰判定時は `START` 行の `restored`, `phase`, `target_v`, `step` を確認

## サンプルコード

`src/main_for_sample/` 以下に C++ 製のサンプルプログラムを収録しています。

- `forklift/main.cpp` … フォークリフトモデルを Hakoniwa と連携して動かす最小構成例
- `forklift/main_unit.cpp` … 荷物なし単体モデル (`models/forklift/forklift-unit.xml`) 用サンプル
- `rover/main.cpp` … ローバー型ロボットの制御例

設定ファイルの使い分け:
- C++シミュレータ (`forklift_sim` / `forklift_unit_sim`) は compact 形式（`*-compact.json`）を使用
- Pythonコントローラ (`forklift_api_control` / `forklift_simple_auto` / `forklift_gamepad`) は legacy 形式（`*.json`）を使用

ビルド後は `forklift_sim` などの実行ファイルが生成され、上記サンプルでは
実際にシミュレーションを起動して Hakoniwa に登録する処理を確認できます。

## Python API の利用方法

`python/api/`には、シミュレーション上のフォークリフトを簡単に操作するための`ForkliftAPI`が用意されています。
以下に、現在のAPI仕様に合わせた基本的な使い方を示します。このコードは、フォークリフトを1m前進させた後、90度旋回させる簡単なサンプルです。

```python
import sys
import time
import hakopy
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from api.forklift_api import ForkliftAPI

def main():
    # C++シミュレータ側と合わせたPDU設定ファイルのパス
    # このファイルで、どのロボットがどのデータをやり取りするかが定義されます
    config_path = "config/safety-forklift-pdu.json"

    # PDUマネージャーを初期化し、通信を開始します
    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()

    # Hakoniwaに外部コントローラとして接続します
    if not hakopy.init_for_external():
        raise RuntimeError("Failed to initialize hakopy")

    # ForkliftAPIのインスタンスを作成します
    forklift = ForkliftAPI(pdu_manager)

    try:
        print("フォークリフトを1m前進させます...")
        forklift.move(1.0)
        time.sleep(1) # 動作完了を待つ

        print("フォークリフトを90度旋回させます...")
        forklift.set_yaw_degree(90)
        time.sleep(1) # 動作完了を待つ

        print("処理が完了しました。")

    except KeyboardInterrupt:
        print("プログラムを終了します。")
    finally:
        # 終了処理
        forklift.stop()
        pdu_manager.stop_service()
        hakopy.fin()

if __name__ == "__main__":
    main()
```

このAPIを利用することで、より複雑な自動制御アプリケーションをPythonで開発できます。

## ライセンス

本リポジトリのコードは MIT ライセンスで提供されます。
