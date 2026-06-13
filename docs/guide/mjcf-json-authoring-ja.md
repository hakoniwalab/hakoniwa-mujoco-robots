# MJCF / JSON 作成ガイド

この文書は、初めて Hakoniwa MuJoCo 用のロボット、センサ、アクチュエータを追加する人向けの作業手順です。
目的は、C++ プログラムを書く前に、MJCF XML と JSON 設定を単体で確認できる状態にすることです。

## 全体の順番

おすすめの順番は次の通りです。

1. MuJoCo の MJCF XML を書く。
2. MuJoCo 単体で XML がロードできることを確認する。
3. センサ / アクチュエータ用 JSON を書く。
4. JSON の構文と schema を確認する。
5. MJCF 内の名前と JSON の `mjcf_binding` が対応しているか確認する。
6. 小さい example か最小 C++ で `LoadConfig()` まで通す。
7. 最後に robot sample の制御ループへ組み込む。

最初から C++ に組み込むと、XML、JSON、名前解決、PDU、制御周期のどこで失敗しているか分かりにくくなります。
まずファイル単体で壊れていない状態を作るのが重要です。

## MJCF XML で決めること

MJCF 側では、物理世界と MJCF object の名前を決めます。
Hakoniwa 側の JSON や C++ は、この名前を使って body、site、camera、joint、actuator を探します。

最低限、次を意識してください。

| 目的 | MJCF で用意するもの | JSON / C++ から参照する名前 |
| --- | --- | --- |
| ロボット姿勢を読む | `<body name="...">` | body name |
| LiDAR の位置を読む | `<body name="base_scan">` など | sensor body name |
| 超音波センサの原点と向き | `<site name="front_ultrasonic_site">` | `mjcf_binding.source_site` |
| RGB / depth camera | `<camera name="color_camera">` | camera name |
| joint state | `<joint name="...">` | `joints[].mjcf_joint` |
| joint actuator | `<position>`, `<velocity>`, `<motor>` | `mjcf_binding.actuator_name` |

名前は後からデバッグしやすいように、用途が分かるものにします。
例: `base_link`, `base_scan`, `front_ultrasonic_site`, `left_wheel_joint`, `left_wheel_velocity`.

## センサ向け MJCF の考え方

センサ設定では「どこから読むか」が最初に重要です。

LiDAR は現在の TB3 実装では `base_scan` body を使います。
この body の位置と yaw が scan の基準になります。

超音波センサは `source_site` を使うのが分かりやすいです。
site のローカル `+X` 方向が測距方向になります。

カメラは MuJoCo の `<camera name="...">` を使います。
C++ の `CameraSensor` に渡す camera name と MJCF の camera name を一致させます。

## アクチュエータ向け MJCF の考え方

joint actuator では、joint と actuator の両方を MJCF に定義します。
JSON の `spec.type` と MJCF の actuator 種別は一致させます。

```xml
<joint name="wheel_joint" type="hinge" axis="0 1 0"/>
<velocity name="wheel_velocity" joint="wheel_joint" kv="3"/>
```

対応する JSON は次のようになります。

```json
{
  "$schema": "../../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "wheel_joint",
    "type": "velocity"
  },
  "mjcf_binding": {
    "actuator_name": "wheel_velocity"
  }
}
```

`spec.joint_name` は joint の名前、`mjcf_binding.actuator_name` は actuator の名前です。
ここを同じものだと思って書くと、モデルによっては解決に失敗したり、意図しない actuator を見に行ったりします。

## JSON で決めること

JSON 側では、センサやアクチュエータの設定値と、MJCF object への binding を書きます。

センサ profile の例:

```json
{
  "$schema": "../schema/ultrasonic.schema.json",
  "frame_id": "front_ultrasonic",
  "DetectionDistance": {
    "Min": 0.05,
    "Max": 2.0
  },
  "UpdateRate": 20,
  "RuntimeBinding": {
    "source_site": "front_ultrasonic_site"
  }
}
```

PDU output config の例:

```json
{
  "$schema": "../schema/joint-state-output.schema.json",
  "spec": {
    "type": "joint_state",
    "name": "wheel_joint_states",
    "joints": [
      { "name": "left_wheel_joint" }
    ]
  },
  "mjcf_binding": {
    "joints": [
      {
        "name": "left_wheel_joint",
        "mjcf_joint": "left_wheel_joint"
      }
    ]
  },
  "pdu_config": {
    "pdu_name": "joint_states",
    "update_rate_hz": 20,
    "message_type": "sensor_msgs/JointState"
  }
}
```

`frame_id` は PDU/ROS 互換メッセージ上の論理フレーム名です。
`RuntimeBinding` や `mjcf_joint` は MuJoCo XML 内の実体名です。
同じ名前にしても構いませんが、役割は別です。

## 単体チェック

このリポジトリには軽量な確認コマンドがあります。

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  --json config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

このコマンドは次を確認します。

- JSON として parse できるか。
- `$schema` がローカル schema を指している場合、schema ファイルが存在するか。
- Python package `jsonschema` が入っている場合、JSON Schema validation が通るか。
- Python package `mujoco` が入っている場合、MJCF XML を MuJoCo として load できるか。

依存 package がない場合は `WARN` を出して、その確認だけを skip します。
厳密に確認したい場合は次を入れてください。

```bash
python3 -m pip install jsonschema mujoco
```

## 名前の対応チェック

schema validation は、JSON の形が正しいかは見られます。
一方で、`RuntimeBinding.source_site` や `actuator_name` が本当に MJCF に存在するかは、最終的には runtime load で確認します。

確認の目安:

- `source_site` は MJCF の `<site name="...">` と一致しているか。
- camera name は MJCF の `<camera name="...">` と一致しているか。
- `mjcf_joint` は MJCF の `<joint name="...">` と一致しているか。
- `mjcf_binding.actuator_name` は `<position>`, `<velocity>`, `<motor>` の `name` と一致しているか。
- actuator JSON の `spec.type` は MJCF actuator 種別と一致しているか。

## C++ に入る前の最小確認

C++ へ組み込む前に、近い example で `LoadConfig()` が通るか確認します。

超音波センサ:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example \
  models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

RGB カメラ:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example \
  models/sensors/color_camera/color-camera-sample.xml \
  config/sensors/color_camera/simple-color-camera.json \
  ./camera_color_sample.png
```

joint actuator:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

新しいモデルや JSON を使う場合は、まず既存 example の model/config 引数を差し替える形にすると、C++ integration 前の切り分けがしやすくなります。

## C++ integration の入口

ファイル単体の確認が終わったら、C++ で次を行います。

1. `WorldImpl` で MJCF を load する。
2. センサまたは actuator class を作る。
3. JSON を `LoadConfig()` する。
4. 1 step だけ `Measure()`, `Capture()`, `Build()`, `SetTarget()` などを呼ぶ。
5. 必要になってから PDU adapter を接続する。

PDU 接続は最後で構いません。
まず MuJoCo object 名と JSON 設定が正しく解決できることを確認してください。

## よくある失敗

- XML は well-formed だが MuJoCo model として load できない。
- body/site/camera/joint/actuator の名前が JSON と一致していない。
- `frame_id` と MJCF object name を混同している。
- actuator JSON の `spec.type` と MJCF actuator 種別が一致していない。
- センサの向きが想定と違う。超音波は site の local `+X`、LiDAR は実装側の基準 body yaw を確認します。
- JSON schema validation は通るが、MJCF binding の参照先が存在しない。schema は MJCF の中身までは見ません。
