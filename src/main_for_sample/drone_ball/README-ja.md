# drone_ball サンプル

このサンプルは、**Hakoniwa のドローン PDU** と **MuJoCo のローカル物理** を組み合わせる最小構成です。

やりたいことは次の 2 つです。

- ドローンは Hakoniwa 側の状態を MuJoCo に反映して、**アバター**として表示する
- ボールは MuJoCo の剛体としてローカルに物理シミュレーションし、必要な状態や操作だけを PDU に出す

つまり、**ドローンはミラー表示対象**、**ボールは物理対象**です。

## 何ができるか

このサンプルでは次を実装しています。

- `Drone-1`, `Drone-2`
  - MuJoCo 上では `mirrored` な剛体
  - 外部の `pos` PDU の最新値を読んで、MuJoCo body に反映する
- `Ball-1`
  - MuJoCo 上では `controllable` な剛体
  - `pos`, `velocity` を PDU 出力する
  - `set_pos`, `add_force` を **イベント入力**として受ける

## 実行方法

前提:

- ドローンシミュレータを先に起動しておく

その後、このリポジトリ側で次を実行します。

```bash
./src/cmake-build/main_for_sample/drone_ball/drone_ball_sim --disable-conductor-start
```

その後で、シミュレーション開始トリガーを入れます。

```bash
hako-cmd start
```

必要なら viewer も止められます。

```bash
./src/cmake-build/main_for_sample/drone_ball/drone_ball_sim \
  --disable-conductor-start \
  --disable-viewer
```

## Python からのボール操作

Python から `Ball-1` を操作するサンプルとして、[python/drone_ball_control.py](/Users/tmori/project/oss/hakoniwa-mujoco-robots/python/drone_ball_control.py:1) を用意しています。

前提:

- `hakoniwa-pdu >= 1.6.1`
- Hakoniwa core 対応の Python runtime

が利用できること。通常は次で導入します。

```bash
python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

このスクリプトは内部で [models/drone/drone_ball_service.json](/Users/tmori/project/oss/hakoniwa-mujoco-robots/models/drone/drone_ball_service.json:1) を参照して、`Ball-1` の channel id を解決します。

### 1. まず位置を設定して、その後に力を加える

一番基本の使い方は次です。

```bash
python3 python/drone_ball_control.py \
  --set-pos 0 -2 1.0 0 0 0 \
  --add-force 0 6 10
```

意味:

- `--set-pos 0 -2 1.0 0 0 0`
  - 位置を `(x=0, y=-2, z=1.0)` に置く
  - 姿勢を `(roll=0, pitch=0, yaw=0)` にする
- `--add-force 0 6 10`
  - world 座標系で `(fx=0, fy=6, fz=10)` の力を 1 回だけ加える

このサンプルでは、

1. `set_pos` を送る
2. 少し待つ
3. `add_force` を送る

という順番で送信します。

### 2. 位置だけを設定したい場合

```bash
python3 python/drone_ball_control.py \
  --set-pos 0 -2 1.0 0 0 0
```

これは「ボールを今そこへ置く」イベントです。

### 3. 力だけを加えたい場合

```bash
python3 python/drone_ball_control.py \
  --add-force 0 6 10
```

これは現在位置のボールに対して、1 回だけ力を加えるイベントです。

### 4. トルクも送りたい場合

`add_force` は内部的には `Twist` を使っているので、必要なら torque も一緒に送れます。

```bash
python3 python/drone_ball_control.py \
  --set-pos 0 -2 1.0 0 0 0 \
  --add-force 0 6 10 \
  --add-torque 0 0 0
```

### 5. 補足

- `set_pos` と `add_force` はどちらも **event** です
- latest 値ではなく、送った瞬間に 1 回だけ適用されます
- `add_force` の座標系は world 座標系です
- `add_force` は内部で短時間だけ保持して加えるので、1 回の送信でも飛ばしやすくしています
- 姿勢は Euler 角で指定します
- `--sleep-after-set-pos` で `set_pos` と `add_force` の間隔を秒で調整できます
- SHM では最初の `read` が PDU 実体の load を兼ねるため、このスクリプトは起動時に `Ball-1.pos` を 1 回 read して warmup してから write します
- 起動直後は SHM 上の PDU 作成タイミングと競合することがあるため、このスクリプトは write 失敗時に短時間だけ自動リトライします

## アーキテクチャ

今回の構成は、PDU と MuJoCo の責務を分けています。

### 1. ドローン

ドローンは **Hakoniwa 上のアバター**です。

- 外部のドローンシミュレータが `pos` を PDU に書く
- `drone_ball_sim` はその `pos` を読む
- MuJoCo の `d1_b_drone_base`, `d2_b_drone_base` に反映する

ここでは、ドローン自身をこのプロセスで制御しているわけではありません。
**外部状態を MuJoCo に写しているだけ**です。

### 2. ボール

ボールは **MuJoCo ローカルの物理剛体**です。

- MuJoCo world の中で接触、落下、反発を計算する
- `pos`, `velocity` は外部へ PDU 出力する
- `set_pos`, `add_force` は外部からの入力として受ける

つまり、ボールはアバターではなく、**PDU 接続された物理 object** です。

### 2.5 衝突 impulse 送信

`drone_ball_sim` では、`Ball-1` が `Drone-1` または `Drone-2` に当たった時に、`impulse` PDU を送る sender も入れています。

この sender は現在、少なくとも次のチューニング値を持っています。

- `restitution_coefficient`
- `relative_normal_speed_threshold`
- `cooldown_steps`

特に `cooldown_steps` は、同じ接触が連続している間に何度も impulse を送りすぎないための抑制用パラメータです。

現時点では、これらの値は `src/main_for_sample/drone_ball/main.cpp` 側で sender に渡しています。

### 3. PDU 設定の考え方

今回の実装では、判断を C++ コードに埋め込みすぎないようにしています。

- `models/drone/mujoco-pdu-bindings.json`
  - `robot name` と `bodyName`
  - `type` (`mirrored` / `controllable`)
  - 使用する channel 名
- `models/drone/endpoint/...`
  - endpoint 設定
  - SHM callback
  - `notify_on_recv`

loader は **binding JSON** と **endpoint 定義** の両方を読んで、実際の runtime 構成を作ります。

## クラス構成

主なクラスは次の 3 つです。

- `PduBoundRigidBody`
  - MuJoCo body と PDU 設定の共通基底
- `MirroredRigidBody`
  - PDU の最新値を読んで MuJoCo body に反映する
- `ControllableRigidBody`
  - `set_pos`, `add_force` をイベントとして受けて MuJoCo body を操作する

## 今回のポイント

`add_force` と `set_pos` は **latest 値**ではなく **event** として扱っています。

理由:

- `add_force` は 1 回だけ効くべき
- `set_pos` も「今そこへ置く」という操作であり、状態ではない
- latest で毎フレーム読むと、未送信時の扱いや多重適用が分かりづらくなる

このため、`Ball-1` の `set_pos` と `add_force` は `notify_on_recv: true` にしています。

## 未テスト事項

現時点で、次は**未テスト**です。

- Python サンプルから `Ball-1.set_pos` を送ること
- Python サンプルから `Ball-1.add_force` を送ること

つまり、

- ドローンアバター表示
- MuJoCo world 上でのボール存在

までは動作確認済みですが、**Python 側からのボール操作入力はまだ確認していません**。

## 関連ファイル

- `models/drone/drone.xml`
- `models/drone/drone-pdudef-current.json`
- `models/drone/ball-pdutypes.json`
- `models/drone/mujoco-pdu-bindings.json`
- `models/drone/endpoint/drone_ball_endpoint.json`
- `models/drone/endpoint/comm/shm_drone_ball_comm.json`
- `src/main_for_sample/drone_ball/main.cpp`

## 補足

このサンプルは、厳密な物理制御や最終設計ではなく、**学生でも追いやすい最小の接続例**を意識しています。

まずは、

- ドローンを MuJoCo 上で見えるようにする
- ボールを接触させる
- ボール状態を PDU に出す
- 必要なら外部からボールを動かす

という順番で理解するのがおすすめです。
