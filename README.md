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

ターミナルを2つ用意して、同一コンテナ内で以下を実行します。

1. ターミナル1（シミュレータ）:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

2. ターミナル2（Python自動操縦）:
```bash
python -m python.forklift_simple_auto config/custom.json
```

## サンプルの実行

本サンプルは、**C++製のシミュレータ**と**Python製のコントローラ**を連携させて動作させます。それぞれを別のターミナルで起動する必要があります。

### 前提条件

1.  **Hakoniwaコアのセットアップ**
    シミュレーションの中核を担うHakoniwaコアライブラリのセットアップが必須です。
    詳細は **[thirdparty/hakoniwa-core-pro のREADME](https://github.com/hakoniwalab/hakoniwa-core-pro/blob/main/README.md)** を参照してインストールを完了してください。これにより、Pythonの `hakopy` ライブラリも同時にセットアップされます。

2.  **Python追加ライブラリのインストール（ゲームパッド操作時のみ）**
    ゲームパッド操作を使う場合のみ `pygame` が必要です。`pip`でインストールしてください。
    ```bash
    pip install pygame
    ```

3.  **ゲームパッドの接続（手動操作時のみ）**
    手動操作を試す場合は、事前にPCにゲームパッドを接続しておいてください。

### 実行手順

2つのターミナル（以降、**ターミナル1**, **ターミナル2**と呼びます）を用意してください。

**1. シミュレータの起動 (ターミナル1)**

まず、C++でビルドされたシミュレータを起動します。
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```
実行後、MuJoCoのシミュレーションGUIウィンドウが立ち上がり、フォークリフトが表示されます。この時点ではまだ動きません。

**2. コントローラの起動 (ターミナル2)**

次に、Python製のコントローラを起動してフォークリフトを動かします。自動操縦（方法A）を推奨します。

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
このサンプルは、前進・旋回・リフト上下の基本操作のみで構成されるため、最初の動作確認に向いています。

#### 方法B: ゲームパッドで手動操作する（任意）

ゲームパッドを使ってリアルタイムにフォークリフトを操作します。
```bash
python -m python.forklift_gamepad config/custom.json
```
このコマンドを実行すると、ゲームパッドの入力がシミュレータに送られ、フォークリフトを自由に動かせるようになります。

## サンプルコード

`src/main_for_sample/` 以下に C++ 製のサンプルプログラムを収録しています。

- `forklift/main.cpp` … フォークリフトモデルを Hakoniwa と連携して動かす最小構成例
- `rover/main.cpp` … ローバー型ロボットの制御例

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
