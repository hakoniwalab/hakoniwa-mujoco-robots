# JSON 設定ガイド

この文書は、このリポジトリで使う JSON 設定の全体像を説明します。
センサやアクチュエータを追加するときは、MJCF XML と C++ の前に、まず JSON の役割分担を理解してください。

詳細な schema 一覧は [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md)、
MJCF との対応は [`mjcf-json-authoring-ja.md`](mjcf-json-authoring-ja.md) を参照してください。

## JSON の種類

主な JSON は次の種類に分かれます。

| 種類 | 置き場所 | 役割 | 例 |
| --- | --- | --- | --- |
| PDU robot definition | `config/*-compact.json` | robot 名と PDU type list を対応づける | `config/tb3-pdudef-compact.json` |
| PDU type list | `config/*pdutypes*.json` | PDU channel 名、型、サイズを定義する | `config/tb3-pdutypes.json` |
| endpoint config | `config/endpoint/*.json` | C++ / Python endpoint が使う PDU 定義、cache、comm を指定する | `config/endpoint/tb3_sim_endpoint.json` |
| sensor profile | `config/sensors/<type>/*.json` | センサそのものの特性を定義する | `config/sensors/lidar/lds-02.json` |
| PDU output config | `config/sensors/<type>/*.json` | MuJoCo state をどの PDU として出すかを定義する | `config/sensors/imu/tb3-imu.json` |
| actuator profile | `config/actuator/<type>/*.json` | MuJoCo actuator への command の入れ方を定義する | `config/actuator/joint/tb3_left_wheel.json` |
| asset manifest | `config/assets/*.json` | 1つの箱庭アセットを構成する既存ファイル群の目次 | `config/assets/tb3-hakoniwa-asset.json` |
| schema | `config/**/schema/*.schema.json` | JSON の形を検証するための定義 | `config/sensors/schema/lidar-2d.schema.json` |

最初に触ることが多いのは、sensor profile、PDU output config、actuator profile です。
PDU definition と endpoint config は、Python や別 asset と通信する段階で必要になります。

## Asset manifest は目次

`config/assets/*.json` は、センサやアクチュエータの設定値を再定義するためのファイルではありません。
各 component JSON はすでに実装コードが読む正本です。
Asset manifest は、それらの正本JSON、MJCF、PDU definition、endpoint config を1つの箱庭アセットとして束ねる目次です。

例:

```json
{
  "$schema": "schema/hakoniwa-asset-manifest.schema.json",
  "name": "tb3_mujoco_asset",
  "model": "../../models/tb3/turtlebot3_burger_world.xml",
  "pdu_def": "../tb3-pdudef-compact.json",
  "endpoint": "../endpoint/tb3_sim_endpoint.json",
  "components": [
    {
      "id": "color_camera",
      "kind": "sensor",
      "type": "color_camera",
      "config": "../sensors/color_camera/tb3-color-camera-320x240.json",
      "pdu_robot": "CameraAsset"
    }
  ]
}
```

ここに `pdu_name`、`update_rate_hz`、`mjcf_binding` の詳細は書きません。
それらは各 component JSON の `pdu_config` / `mjcf_binding` にあります。

`pdu_robot` は `PduKey(robot_name, channel_name)` の第1引数に使う PDU definition 上の robot 名です。
`pdu_config.pdu_name` とは別の namespace なので、manifest 側に置きます。
PDU 接続しない local-only component は `pdu_robot` を省略できます。

TB3 runtime は、この manifest を入口にして MJCF、PDU definition、endpoint config、各 component JSON をロードします。
デフォルトは `config/assets/tb3-hakoniwa-asset.json` です。
別の manifest を使う場合は次のように指定します。

```bash
HAKO_TB3_MANIFEST_PATH=config/assets/tb3-hakoniwa-asset.json \
  ./src/cmake-build/main_for_sample/tb3/tb3_sim
```

manifest から一括チェックするには次を使います。

```bash
python3 tools/validate_assets.py \
  --manifest config/assets/tb3-hakoniwa-asset.json
```

MJCF binding と PDU 接続の関係を一覧表示するには inspector を使います。

```bash
python3 tools/inspect_asset_manifest.py \
  config/assets/tb3-hakoniwa-asset.json
```

## 3つの設定コンテナで考える

センサ / アクチュエータ設定は、次の3つのコンテナに分けて考えると整理しやすくなります。

| コンテナ | 役割 | 代表フィールド |
| --- | --- | --- |
| `spec` | センサやアクチュエータそのものの仕様 | `DetectionDistance`, `AngleRange`, `DistanceAccuracy`, `type`, `limit` |
| `mjcf_binding` | MJCF XML 内の object 名との対応 | `source_body`, `source_site`, `actuator_name`, `mjcf_joint` |
| `pdu_config` | Hakoniwa PDU の入出力設定 | `pdu_name`, `update_rate_hz`, PDU message type |

概念的には次の形です。

```json
{
  "spec": {
    "frame_id": "laser",
    "DetectionDistance": {
      "Min": 160,
      "Max": 8000
    }
  },
  "mjcf_binding": {
    "source_body": "base_scan"
  },
  "pdu_config": {
    "pdu_name": "laser_scan",
    "update_rate_hz": 5
  }
}
```

新しく書くセンサ / アクチュエータ設定では、この3つのキーをそのまま使います。
このリポジトリの schema は `spec` / `mjcf_binding` / `pdu_config` を公開上の正規形として扱います。

## 現行 JSON の共通構造

多くの設定 JSON は、次のような構造を持ちます。

```json
{
  "$schema": "../schema/example.schema.json",
  "spec": {
    "type": "example",
    "name": "example_name",
    "frame_id": "example_frame"
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "source_body": "example_body",
    "source_site": "example_site"
  },
  "pdu_config": {
    "pdu_name": "example_pdu",
    "update_rate_hz": 20,
    "message_type": "example/Message"
  }
}
```

すべての JSON が全項目を持つわけではありません。
どの項目が必要かは、対応する schema と既存サンプルを見てください。

## 重要な共通フィールド

### `$schema`

この JSON がどの schema に従うかを示します。
エディタ補完や `tools/validate_assets.py` の validation に使います。

```json
"$schema": "../schema/lidar-2d.schema.json"
```

remote URL の形を使っている既存ファイルもあります。

```json
"$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json"
```

`tools/validate_assets.py` は `https://hakoniwa.dev/schemas/<name>.schema.json` を、リポジトリ内の schema に対応づけて確認します。

### `type`

PDU output config や actuator profile の種類を表します。

例:

```json
"type": "joint_state"
```

```json
"type": "velocity"
```

sensor profile では、`type` ではなく `DetectionDistance` や `AngleRange` のような sensor 固有フィールドで種類が決まるものもあります。

### `name`

Hakoniwa / application 内での設定名です。
人間が見て識別しやすい名前にします。

```json
"name": "tb3_imu"
```

### `frame_id`

PDU / ROS-compatible message に入る論理フレーム名です。
MJCF の body や site の名前とは別物です。

```json
"frame_id": "base_scan"
```

同じ文字列にしても構いませんが、意味は違います。
`frame_id` は publish されるデータの座標系名、`mjcf_binding.*` は MuJoCo model 内の実体名です。

### `pdu_name`

Hakoniwa PDU channel 名です。
`config/*pdutypes*.json` に同じ `name` が必要です。

```json
"pdu_name": "joint_states"
```

TB3 では `config/tb3-pdutypes.json` に次のような channel があります。

```json
{
  "name": "joint_states",
  "type": "sensor_msgs/JointState"
}
```

### `update_rate_hz`

Hz 単位の更新周期です。
sensor spec と PDU config のどちらも `update_rate_hz` を使います。
runtime が sampling 周期と publish 周期を意図的に分ける場合を除き、同じ値にしてください。

```json
"update_rate_hz": 20
```

## `mjcf_binding` / MJCF Binding

`mjcf_binding` は、JSON 設定を MJCF 内の object に結びつける block です。

```json
"mjcf_binding": {
  "config_style": "hakoniwa-sdf-like",
  "runtime_source": "mjcf",
  "source_site": "front_ultrasonic_site"
}
```

重要なのは、`mjcf_binding` は PDU 名ではなく MuJoCo XML 内の名前を書く場所だという点です。

| フィールド | 意味 | 例 |
| --- | --- | --- |
| `config_style` | 設定形式。現在は `hakoniwa-sdf-like` | `hakoniwa-sdf-like` |
| `runtime_source` | binding 先の出所。現在は主に `mjcf` | `mjcf` |
| `parent_body` | 親 body 名 | `base_link` |
| `source_body` | センサや state の基準 body 名 | `base_scan` |
| `source_site` | センサ原点・向きに使う site 名 | `front_ultrasonic_site` |
| `actuator_name` | MJCF actuator 名 | `left_wheel_velocity` |
| `frame_id_override` | publish 時の frame_id override | `front_ultrasonic` |

`mjcf_binding` の値は schema validation では存在確認できません。
たとえば `source_site` が MJCF 内に本当にあるかは、MuJoCo model を load して `LoadConfig()` する段階で確認されます。

## `spec` と `pdu_config` の違い

`spec` は、センサやアクチュエータの物理的・仕様的な特性を表します。

例:

```json
{
  "frame_id": "laser",
  "DetectionDistance": {
    "Min": 160,
    "Max": 8000
  },
  "AngleRange": {
    "Min": -180.0,
    "Max": 180.0,
    "Resolution": 1.0,
    "ScanFrequency": 5
  }
}
```

`pdu_config` は、その component を Hakoniwa PDU にどう接続するかを表します。
センサでは MuJoCo state を publish する PDU、アクチュエータでは受信する command PDU channel を表します。

例:

```json
{
  "spec": {
    "type": "joint_state",
    "name": "tb3_wheel_joint_states",
    "joints": [
      { "name": "left_wheel_joint" }
    ]
  },
  "mjcf_binding": {
    "joints": [
      {
        "name": "left_wheel_joint",
        "mjcf_joint": "wheel_left_joint"
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

Joint state も `spec`、`mjcf_binding`、`pdu_config` に分けて定義します。

## Actuator Profile

actuator profile は、PDU や C++ command から受け取った target を MuJoCo actuator にどう書き込むかを表します。

```json
{
  "$schema": "../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "left_wheel_joint",
    "type": "velocity",
    "limit": {
      "velocity": 12.0
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "actuator_name": "left_wheel_velocity"
  },
  "pdu_config": {
    "pdu_name": "left_wheel_cmd",
    "message_type": "std_msgs/Float64"
  }
}
```

`spec.joint_name` は MJCF joint 名です。
`mjcf_binding.actuator_name` は MJCF actuator 名です。
`spec.type` は MJCF actuator 種別と合わせます。

| JSON `spec.type` | MJCF |
| --- | --- |
| `position` | `<position>` |
| `velocity` | `<velocity>` |
| `torque` | `<motor>` |

## PDU Definition と Endpoint Config

Python や別 asset と通信する場合は、PDU definition と endpoint config も必要です。

`config/tb3-pdudef-compact.json` は robot 名と PDU type list を対応づけます。

```json
{
  "paths": [
    {
      "id": "tb3-endpoint",
      "path": "tb3-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "TB3",
      "pdutypes_id": "tb3-endpoint"
    }
  ]
}
```

`config/tb3-pdutypes.json` は channel を定義します。

```json
{
  "name": "laser_scan",
  "type": "sensor_msgs/LaserScan",
  "pdu_size": 8192
}
```

`config/endpoint/tb3_sim_endpoint.json` は C++ endpoint がどの PDU 定義、cache、comm を使うかを指定します。

```json
{
  "name": "tb3_sim_endpoint",
  "pdu_def_path": "../tb3-pdudef-compact.json",
  "cache": "cache/buffer.json",
  "comm": "comm/shm_sim_comm.json"
}
```

## 確認方法

JSON 単体の確認には `tools/validate_assets.py` を使います。

```bash
python3 tools/validate_assets.py \
  --json config/sensors/lidar/lds-02.json \
  --json config/actuator/joint/sample_velocity_actuator.json
```

MJCF と一緒に確認する場合:

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  --json config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

この確認で分かること:

- JSON として parse できるか。
- `$schema` が指す schema に合っているか。
- Python `mujoco` がある場合、MJCF が load できるか。

この確認だけでは分からないこと:

- `pdu_name` が実行中 endpoint に本当に存在するか。
- `mjcf_binding.source_site` や `mjcf_binding.actuator_name` が意図した MJCF object か。
- センサの向きや actuator の制御特性が期待通りか。

そこは example 実行や C++ の `LoadConfig()`、viewer、PDU visualizer で確認します。

## 初学者向けの作業順

1. 近い既存 JSON をコピーする。
2. `$schema` を保つ。
3. `spec`, `mjcf_binding`, `pdu_config` のどれを書いているかを意識する。
4. `name`, `frame_id`, `pdu_name` の意味を分けて決める。
5. MJCF に合わせて `mjcf_binding` や `mjcf_joint` を書く。
6. `tools/validate_assets.py` で JSON を確認する。
7. 近い example で `LoadConfig()` まで確認する。
8. 必要になってから PDU definition と endpoint config を変更する。
