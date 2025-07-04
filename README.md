# hakoniwa-mujoco-robots

本リポジトリは、箱庭(Hakoniwa)シミュレーションフレームワーク向けに作成した
MuJoCoベースのロボットアセット集です。フォークリフトをはじめとした複数の
モデルを提供し、Hakoniwa環境と連携することでリアルな物理シミュレーションを
実現します。

## ディレクトリ構成

- `models/` … MuJoCo 用のXMLモデルファイル
- `config/custom.json` … 各ロボットのPDU(入出力データ)設定
- `src/` … C++によるシミュレーション実装およびサンプルプログラム
- `python/` … フォークリフト操作用APIやゲームパッド制御スクリプト

## ビルド方法

```bash
./build.bash            # CMakeによるビルドを実行
./build.bash clean      # ビルド生成物の削除
```

ビルドが成功すると `src/cmake-build/` 以下にサンプル実行ファイル
`forklift_sim` 及び `rover_sim` が生成されます。

## サンプルの実行

```bash
# フォークリフトのシミュレーション起動
./src/cmake-build/forklift_sim
```

ゲームパッドを利用して操作する場合は Python スクリプトを実行します。

```bash
python3 python/forklift_gamepad.py config/custom.json
```

## サンプルコード

`src/main_for_sample/` 以下に C++ 製のサンプルプログラムを収録しています。

- `forklift/main.cpp` … フォークリフトモデルを Hakoniwa と連携して動かす最小構成例
- `rover/main.cpp` … ローバー型ロボットの制御例

ビルド後は `forklift_sim` などの実行ファイルが生成され、上記サンプルでは
実際にシミュレーションを起動して Hakoniwa に登録する処理を確認できます。

## Python API の利用方法

`python/api/` にはゲームパッド操作を抽象化した `ForkliftAPI` を用意しています。
以下は基本的な使い方の一例です。

```python
import hakopy
import hako_pdu
from api.forklift_api import ForkliftAPI

if not hakopy.init_for_external():
    raise RuntimeError("failed to init hakopy")

pdu_manager = hako_pdu.HakoPduManager('/usr/local/lib/hakoniwa/hako_binary/offset', 'config/custom.json')
forklift = ForkliftAPI(pdu_manager)
forklift.move(1.0)          # 前方へ1m移動
forklift.set_yaw_degree(90) # 90度旋回
```

この API を利用することで、外部アプリケーションから容易にフォークリフトを
制御できます。

## ライセンス

本リポジトリのコードは MIT ライセンスで提供されます。

