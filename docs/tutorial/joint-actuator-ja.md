# Joint Actuator 設定チュートリアル

このチュートリアルでは、既存の joint actuator example を題材に、MuJoCo の MJCF XML に joint と actuator を置き、JSON で `spec` / `mjcf_binding` を書き、単体チェックして、C++ example で動かすところまでを確認します。

対象ファイル:

```text
models/actuators/joint/position-velocity-actuator-sample.xml
config/actuator/joint/sample_position_actuator.json
config/actuator/joint/sample_velocity_actuator.json
examples/actuators/joint/joint-actuator-example.cpp
examples/actuators/joint/README.md
```

全体の作業順は次です。

1. MJCF XML に joint と actuator を定義する。
2. actuator 用 JSON を `spec` / `mjcf_binding` で書く。
3. MJCF と JSON を単体チェックする。
4. example をビルドして実行する。
5. C++ で JSON と MJCF actuator name がどう接続されるか確認する。

## 1. MJCF XML に joint と actuator を置く

Joint actuator では、MJCF に **joint** と **actuator** の両方が必要です。
この example では、position 制御用と velocity 制御用の 2 つを定義しています。

```xml
<body name="position_demo" pos="-0.5 0 0.5">
  <joint name="position_hinge" type="hinge" axis="0 0 1"
         limited="true" range="-1.0 1.0"/>
  <geom name="position_arm" type="capsule"
        fromto="0 0 0 0.45 0 0" size="0.035"/>
</body>

<body name="velocity_demo" pos="0.5 0 0.5">
  <joint name="velocity_hinge" type="hinge" axis="0 0 1"/>
  <geom name="velocity_wheel" type="cylinder" size="0.18 0.04"/>
</body>

<actuator>
  <position name="position_servo" joint="position_hinge"
            kp="6" dampratio="1.0"
            ctrllimited="true" ctrlrange="-0.8 0.8"/>
  <velocity name="velocity_servo" joint="velocity_hinge"
            kv="3"
            ctrllimited="true" ctrlrange="-4.0 4.0"/>
</actuator>
```

ここで重要な名前は次です。

| MJCF object | 役割 | JSON での対応 |
| --- | --- | --- |
| `position_hinge` | position 制御される joint | `spec.joint_name` |
| `position_servo` | `position_hinge` を動かす MJCF actuator | `mjcf_binding.actuator_name` |
| `velocity_hinge` | velocity 制御される joint | `spec.joint_name` |
| `velocity_servo` | `velocity_hinge` を動かす MJCF actuator | `mjcf_binding.actuator_name` |

`joint_name` と `actuator_name` は別物です。
`joint_name` は `<joint name="...">`、`actuator_name` は `<position name="...">` や `<velocity name="...">` の名前です。

### `<position>` / `<velocity>` / `<motor>` の違い

MuJoCo の actuator 種別によって、`data->ctrl[]` に書く値の意味が変わります。

| MJCF actuator | JSON `spec.type` | `ctrl[]` に書く値 |
| --- | --- | --- |
| `<position>` | `position` | 目標 joint 位置 |
| `<velocity>` | `velocity` | 目標 joint 速度 |
| `<motor>` | `torque` | torque / force command |

この example の `JointActuatorImpl` は、ソフトウェア PID controller を実装しているわけではありません。
JSON を読み、MJCF actuator を解決し、必要なら target を clamp して `data->ctrl[actuator_id]` に書きます。
実際の position / velocity 制御は MuJoCo の `<position>` / `<velocity>` actuator が行います。

## 2. Joint Actuator JSON を書く

position actuator の設定は `config/actuator/joint/sample_position_actuator.json` です。

```json
{
  "$schema": "../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "position_hinge",
    "type": "position",
    "limit": {
      "lower": -0.8,
      "upper": 0.8
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "actuator_name": "position_servo"
  },
  "pdu_config": {
    "pdu_name": "position_target",
    "update_rate_hz": 50.0,
    "message_type": "std_msgs/Float64"
  }
}
```

velocity actuator の設定は `config/actuator/joint/sample_velocity_actuator.json` です。

```json
{
  "$schema": "../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "velocity_hinge",
    "type": "velocity",
    "limit": {
      "velocity": 4.0
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "actuator_name": "velocity_servo"
  },
  "pdu_config": {
    "pdu_name": "velocity_target",
    "update_rate_hz": 50.0,
    "message_type": "std_msgs/Float64"
  }
}
```

### `spec`

`spec` は actuator command の仕様です。

| フィールド | 意味 |
| --- | --- |
| `joint_name` | 目標 joint の MJCF 名 |
| `type` | command 種別。`position` / `velocity` / `torque` |
| `limit.lower` / `limit.upper` | position target の clamp 範囲 |
| `limit.velocity` | velocity target の絶対値上限 |
| `limit.effort` | torque target の絶対値上限 |
| `dynamics` | damping / friction metadata |

`spec.type` は MJCF actuator 種別と一致させます。
`spec.type = "velocity"` なのに `<position>` actuator を指すと、runtime load 時に type mismatch になります。

### `mjcf_binding`

`mjcf_binding` は JSON と MJCF XML の object 名を対応づけます。

```json
"mjcf_binding": {
  "actuator_name": "position_servo"
}
```

`actuator_name` は MJCF の actuator 名です。
`<joint name="position_hinge">` ではなく、`<position name="position_servo">` の名前を書きます。

### `pdu_config`

`pdu_config` は、この actuator を Hakoniwa PDU command に接続するときの設定です。
この `joint-actuator-example` は PDU を使わず、keyboard 入力から直接 `SetTarget()` を呼びます。
ただし、Hakoniwa asset 化するときに必要な channel 名と message type を同じ JSON に置いています。

```json
"pdu_config": {
  "pdu_name": "position_target",
  "update_rate_hz": 50.0,
  "message_type": "std_msgs/Float64"
}
```

`pdu_name` は PDU type list の `name` と、`PduKey(robot_name, channel_name)` の `channel_name` に一致させます。
PDU 接続手順は [Joint Actuator 箱庭アセット化チュートリアル](joint-actuator-hakoniwa-ja.md) を参照してください。

## 3. 単体チェックする

MJCF と JSON を C++ に組み込む前に、単体チェックします。

```bash
python3 tools/validate_assets.py \
  --mjcf models/actuators/joint/position-velocity-actuator-sample.xml \
  --json config/actuator/joint/sample_position_actuator.json \
  --json config/actuator/joint/sample_velocity_actuator.json
```

期待する結果は、MJCF load と JSON schema validation が `OK` になることです。

```text
OK   MJCF load: models/actuators/joint/position-velocity-actuator-sample.xml ...
OK   JSON parse: config/actuator/joint/sample_position_actuator.json
OK   JSON schema: config/actuator/joint/sample_position_actuator.json -> ...
OK   JSON parse: config/actuator/joint/sample_velocity_actuator.json
OK   JSON schema: config/actuator/joint/sample_velocity_actuator.json -> ...
```

このチェックで分かること:

- MJCF XML が MuJoCo model として load できる。
- JSON が parse できる。
- JSON が `joint-actuator.schema.json` に合っている。

このチェックだけでは分からないこと:

- `mjcf_binding.actuator_name` が実際に MJCF の actuator として存在するか。
- `spec.joint_name` と `mjcf_binding.actuator_name` が同じ joint に結びついているか。
- `spec.type` と MJCF actuator 種別が一致しているか。

そこは `JointActuatorImpl::LoadConfig()` が runtime load 時に確認します。

## 4. example をビルドして実行する

ビルド:

```bash
cmake --build src/cmake-build --target joint-actuator-example
```

実行:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

viewer が開いたら、keyboard で target を変えます。

```text
a / d  : decrease / increase position target
j / l  : decrease / increase velocity target
Space  : stop velocity target
r      : reset simulation state and targets
p      : pause / resume physics
h      : show help
q / Esc: quit
```

ターミナルには、target と実測値が出ます。

```text
time= 0.000 pos_target= 0.000 pos_angle=  0.000 vel_target= 0.000 vel_qvel=  0.000
```

明示的に model / config を渡す場合:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example \
  models/actuators/joint/position-velocity-actuator-sample.xml \
  config/actuator/joint/sample_position_actuator.json \
  config/actuator/joint/sample_velocity_actuator.json
```

## 5. C++ でどこを見ればよいか

example の主要部分は `examples/actuators/joint/joint-actuator-example.cpp` です。

JSON を読むところ:

```cpp
auto position_actuator = world->createJointActuator();
auto velocity_actuator = world->createJointActuator();

position_actuator->LoadConfig(position_config_path);
velocity_actuator->LoadConfig(velocity_config_path);
```

target を書くところ:

```cpp
position_actuator->SetTarget(state.position_target);
velocity_actuator->SetTarget(state.velocity_target);
```

実装側では、`JointActuatorImpl::LoadConfig()` が次を行います。

```text
JSON spec.joint_name
        |
        v
MJCF joint id を確認

JSON mjcf_binding.actuator_name
        |
        v
MJCF actuator id を確認

spec.type
        |
        v
MJCF actuator 種別と一致するか確認
```

解決できたら、`SetTarget()` が `data->ctrl[actuator_id]` に target を書きます。

## 6. つまづきポイント

- **`joint_name` と `actuator_name` は別物**:
  `spec.joint_name` は `<joint name="...">`、`mjcf_binding.actuator_name` は `<position name="...">` / `<velocity name="...">` / `<motor name="...">` です。

- **JSON `spec.type` と MJCF actuator 種別を合わせる**:
  `spec.type = "position"` なら MJCF は `<position>`、`spec.type = "velocity"` なら `<velocity>`、`spec.type = "torque"` なら `<motor>` にします。

- **`limit` は command target の clamp**:
  `limit.velocity = 4.0` は velocity target を `[-4.0, 4.0]` に制限します。
  MJCF 側の `ctrlrange` とも意味が近いので、通常は矛盾しない値にします。

- **`dynamics` は MuJoCo actuator を自動変更しない**:
  現在の `JointActuatorImpl` は JSON の `dynamics.damping` / `friction` を読めますが、MJCF model の joint damping を書き換える処理はしていません。
  実際の物理パラメータは MJCF XML 側で設定します。

- **schema validation だけでは MJCF 名の存在確認はできない**:
  JSON の形は schema で確認できますが、`position_servo` が MJCF に存在するかは runtime load で確認します。
  まず `tools/validate_assets.py`、次に example 実行で切り分けます。

## 関連資料

- [examples/actuators/joint/README.md](../../examples/actuators/joint/README.md)
- [JSON 設定ガイド](../guide/json-config-ja.md)
- [MJCF / JSON 作成ガイド](../guide/mjcf-json-authoring-ja.md)
- [Sensor/actuator config schemas](../spec/sensor-actuator-config-schema.md)
