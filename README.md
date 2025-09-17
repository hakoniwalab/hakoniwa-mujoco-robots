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

macOSでは、依存ライブラリとMuJoCoを手動でセットアップする必要があります。

**1. GLFWのインストール**

[Homebrew](https://brew.sh/index_ja) を使用して、描画に必要なライブラリ `glfw` をインストールします。

```bash
brew install glfw
```

**2. MuJoCoライブラリの配置**

[MuJoCoのGitHubリリースページ](https://github.com/google-deepmind/mujoco/releases)から、お使いのMacのアーキテクチャに合ったDMGファイルをダウンロードします。

次に、プロジェクトのルートに`vendor/mujoco`ディレクトリを作成し、DMGに含まれる`include`と`lib`フォルダをその中にコピーしてください。最終的なディレクトリ構成は以下のようになります。

```
hakoniwa-mujoco-robots/
└── vendor/
    └── mujoco/
        ├── include/mujoco/
        └── lib/
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

> 補足：`src/CMakeLists.txt` 内で、`MUJOCO_VERSION` 変数を変更することで、使用する MuJoCo のバージョンを指定できます。

## ビルド手順

各OSのセットアップが完了したら、以下のコマンドでプロジェクトをビルドします。

```bash
./build.bash
```

> **Note:** Linux/WSL2ではMuJoCoライブラリが自動でダウンロードされますが、macOSではセットアップ手順に従って手動で配置する必要があります。

### 3. ビルド成果物

ビルドが成功すると `src/cmake-build/` 以下にサンプル実行ファイル `forklift_sim` が生成されます。

### ビルドのクリーン

ビルド生成物を削除する場合は、`clean`オプションを使用します。
```bash
./build.bash clean
```

## サンプルの実行

本サンプルは、**C++製のシミュレータ**と**Python製のコントローラ**を連携させて動作させます。それぞれを別のターミナルで起動する必要があります。

### 前提条件

1.  **Hakoniwaコアのセットアップ**
    シミュレーションの中核を担うHakoniwaコアライブラリのセットアップが必須です。
    詳細は **[thirdparty/hakoniwa-core-pro のREADME](./thirdparty/hakoniwa-core-pro/README.md)** を参照してインストールを完了してください。これにより、Pythonの `hakopy` ライブラリも同時にセットアップされます。

2.  **Python追加ライブラリのインストール**
    ゲームパッド操作には `pygame` が必要です。`pip`でインストールしてください。
    ```bash
    pip install pygame
    ```

3.  **ゲームパッドの接続**
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

次に、Python製のコントローラを起動してフォークリフトを動かします。操作方法に応じて2種類のスクリプトが用意されています。

#### 方法A: ゲームパッドで手動操作する

ゲームパッドを使ってリアルタイムにフォークリフトを操作します。
```bash
python -m python.forklift_gamepad config/custom.json
```
このコマンドを実行すると、ゲームパッドの入力がシミュレータに送られ、フォークリフトを自由に動かせるようになります。

#### 方法B: APIで自動制御する

`ForkliftAPI`を利用して、あらかじめプログラムされた動作を自動で実行します。このサンプルは、荷物をピックアップして棚に運ぶまでの一連の動作を行います。
```bash
python -m python.forklift_api_control config/safety-forklift-pdu.json config/monitor_camera_config.json
```
実行すると、フォークリフトが定義されたミッションを自動で開始します。

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

