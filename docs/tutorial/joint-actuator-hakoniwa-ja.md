# Joint Actuator 箱庭アセット化チュートリアル

このチュートリアルでは、[Joint Actuator 設定チュートリアル](joint-actuator-ja.md)で作成した MuJoCo joint actuator を、Hakoniwa PDU command に接続する流れを説明します。

Joint actuator はセンサと向きが逆です。
カメラや超音波は MuJoCo から PDU へ publish しますが、joint actuator は PDU から command を receive し、`IJointActuator::SetTarget()` で MuJoCo の `ctrl[]` に書き込みます。

```text
Hakoniwa PDU std_msgs/Float64
        |
        v
JointActuatorPduAdapter
        |
        v
JointActuatorTarget
        |
        v
IJointActuator::SetTarget()
        |
        v
MuJoCo data->ctrl[actuator_id]
```

---

## 1. 全体ワークフロー

```text
  [ C++ Joint Actuator Asset ]
             |
             v (Endpoint名指定でロード)
    [ joint_actuator_endpoint.json ]
             |
    +--------+--------+----------------------------+
    | (参照)           | (参照)                      | (参照)
    v                 v                            v
[ joint-actuator-pdudef-compact.json ] [ cache/buffer.json ] [ comm/shm_joint_actuator_comm.json ]
    |
    v (参照)
[ joint-actuator-pdutypes.json ]
```

C++ 側の受信には、既存の `JointActuatorPduAdapter` を使います。

```cpp
#include "hakoniwa/pdu/adapter/std_msgs/float64.hpp"
```

`JointActuatorPduAdapter::recv_and_apply()` は、以下の変換と適用を内部で行います。

```text
std_msgs/Float64 PDU
        |
        v
JointActuatorTarget
        |
        v
IJointActuator::SetTarget()
```

## 2. actuator JSON の `pdu_config`

position actuator の JSON は次のように `pdu_config` を持ちます。

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

velocity actuator は channel 名だけ変えます。

```json
"pdu_config": {
  "pdu_name": "velocity_target",
  "update_rate_hz": 50.0,
  "message_type": "std_msgs/Float64"
}
```

`pdu_config` の意味は次です。

| フィールド | 意味 |
| --- | --- |
| `pdu_name` | command PDU の channel 名 |
| `update_rate_hz` | command を処理する想定周期 |
| `message_type` | PDU message type。ここでは `std_msgs/Float64` |

`pdu_name` は、後述の PDU type list の `name` と、`PduKey(robot_name, channel_name)` の `channel_name` に一致させます。

## 3. PDU 設定ファイル

ここでは、PDU 上の robot 名を `"JointActuatorAsset"`、command channel 名を `"position_target"` / `"velocity_target"` とします。
`PduKey` の第1引数は asset 登録名そのものではなく、PDU 定義上の robot 名です。
この example では分かりやすさのため、C++ asset 登録名も `"JointActuatorAsset"` にそろえる想定です。

### `config/joint-actuator-pdutypes.json`

```json
[
  {
    "channel_id": 0,
    "pdu_size": 32,
    "name": "position_target",
    "type": "std_msgs/Float64"
  },
  {
    "channel_id": 1,
    "pdu_size": 32,
    "name": "velocity_target",
    "type": "std_msgs/Float64"
  }
]
```

`std_msgs/Float64` の基本サイズは `8` bytes です。
endpoint の channel buffer では PDU metadata `24` bytes も含めるため、`pdu_size` は `32` にします。

```text
32 = 24 (PDU metadata) + 8 (std_msgs/Float64)
```

### `config/joint-actuator-pdudef-compact.json`

```json
{
  "paths": [
    {
      "id": "joint-actuator-command",
      "path": "joint-actuator-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "JointActuatorAsset",
      "pdutypes_id": "joint-actuator-command"
    }
  ]
}
```

`robots[].name` は PDU robot 名です。
`PduKey("JointActuatorAsset", "position_target")` の `"JointActuatorAsset"` と一致させます。

### `config/endpoint/joint_actuator_endpoint.json`

```json
{
  "name": "joint_actuator_endpoint",
  "pdu_def_path": "../joint-actuator-pdudef-compact.json",
  "cache": "cache/buffer.json",
  "comm": "comm/shm_joint_actuator_comm.json"
}
```

### `config/endpoint/comm/shm_joint_actuator_comm.json`

```json
{
  "protocol": "shm",
  "impl_type": "callback",
  "name": "joint_actuator_shm",
  "direction": "inout",
  "io": {
    "robots": [
      {
        "name": "JointActuatorAsset",
        "pdu": [
          { "name": "position_target", "notify_on_recv": false },
          { "name": "velocity_target", "notify_on_recv": false }
        ]
      }
    ]
  }
}
```

## 4. C++ asset の実装方針

Joint actuator asset の endpoint lifecycle は camera / ultrasonic と同じ順序にします。

1. `main()` で `hako_asset_register()` を呼ぶ。
2. `main()` で `endpoint.open()` を呼ぶ。
3. `main()` で `endpoint.start()` を呼ぶ。
4. worker thread で `hako_asset_start_no_wait(IsForceStop)` を呼ぶ。
5. `on_initialize` callback で `endpoint.post_start()` を呼ぶ。
6. `post_start()` 完了後、manual timing loop で PDU command を受信する。
7. command を受信できたら `JointActuatorPduAdapter::recv_and_apply()` で actuator に反映する。
8. 終了時に `endpoint.stop()` / `endpoint.close()` を呼ぶ。

`hako_asset_start_no_wait()` は名前に `no_wait` とありますが、箱庭の start trigger は待ちます。
ここでは `hako_asset_start()` ではなく、停止判定 callback を渡せる `hako_asset_start_no_wait(IsForceStop)` を worker thread で呼びます。
これにより、MuJoCo viewer を main thread で動かしたまま、viewer close や `q` 入力で asset 側も停止できます。

### adapter の作成

```cpp
#include "hakoniwa/pdu/adapter/std_msgs/float64.hpp"
#include "hakoniwa/pdu/endpoint.hpp"

std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::std_msgs::JointActuatorPduAdapter>
    position_adapter;
std::unique_ptr<hako::robots::pdu::adapter::std_msgs::JointActuatorPduAdapter>
    velocity_adapter;
std::atomic_bool endpoint_ready {false};
```

```cpp
auto position_actuator = world->createJointActuator();
auto velocity_actuator = world->createJointActuator();

position_actuator->LoadConfig("config/actuator/joint/sample_position_actuator.json");
velocity_actuator->LoadConfig("config/actuator/joint/sample_velocity_actuator.json");

const hakoniwa::pdu::PduKey position_key {
    "JointActuatorAsset",
    position_actuator->GetConfig().pdu_config.pdu_name
};
const hakoniwa::pdu::PduKey velocity_key {
    "JointActuatorAsset",
    velocity_actuator->GetConfig().pdu_config.pdu_name
};

position_adapter =
    std::make_unique<hako::robots::pdu::adapter::std_msgs::JointActuatorPduAdapter>(
        *endpoint,
        position_key,
        *position_actuator);
velocity_adapter =
    std::make_unique<hako::robots::pdu::adapter::std_msgs::JointActuatorPduAdapter>(
        *endpoint,
        velocity_key,
        *velocity_actuator);
```

### command の受信と適用

manual timing loop では、受信できた command だけ actuator に反映します。
受信できない場合は、前回 target のまま MuJoCo simulation を進めます。

```cpp
while (running_flag) {
    if (endpoint_ready.load()) {
        (void)position_adapter->recv_and_apply();
        (void)velocity_adapter->recv_and_apply();
    }

    world->advanceTimeStep();
    hako_asset_usleep(delta_time_usec);
}
```

`JointActuatorPduAdapter::recv_and_apply()` は `std_msgs/Float64.data` を `JointActuatorTarget.value` に変換し、`IJointActuator::SetTarget()` を呼びます。

## 5. command sender の考え方

command を送る側は、同じ PDU robot/channel に `std_msgs/Float64` を書き込みます。

```text
PduKey("JointActuatorAsset", "position_target") -> position target [rad]
PduKey("JointActuatorAsset", "velocity_target") -> velocity target [rad/s]
```

`PduKey` は asset 名ではなく `robot 名 + channel 名` で受信対象を指定します。
sender asset 自身の asset 登録名と、command の宛先になる PDU robot 名を混同しないでください。

## 6. 実行時の確認ポイント

- C++ asset 起動時に `hako_asset_register()` が成功する。
- `endpoint.open()` / `endpoint.start()` が `main()` で成功する。
- `hako-cmd start` 後、`on_initialize` で `endpoint.post_start()` が成功する。
- sender が `position_target` / `velocity_target` に `std_msgs/Float64` を書く。
- viewer 上で position arm / velocity wheel が command に応じて動く。

## 7. つまづきポイント

- **`pdu_config.pdu_name` と PDU type list の `name` を合わせる**:
  JSON 側が `"position_target"` なら、`PduKey("JointActuatorAsset", "position_target")` と `joint-actuator-pdutypes.json` の `name` も `"position_target"` にします。

- **`PduKey` は asset 名ではなく robot 名 + channel 名**:
  `PduKey("JointActuatorAsset", "position_target")` の `"JointActuatorAsset"` は `joint-actuator-pdudef-compact.json` の `robots[].name` です。

- **`pdu_size` は metadata を含める**:
  `std_msgs/Float64` は `8` bytes、endpoint buffer は PDU metadata `24` bytes を含めるため `32` bytes にします。

- **command が来ないと target は更新されない**:
  `recv_and_apply()` は受信できたときだけ `SetTarget()` を呼びます。
  command sender がまだ書いていない場合、actuator は前回 target のままです。

- **`spec.type` と MJCF actuator 種別を合わせる**:
  PDU command が届いていても、JSON `spec.type` と MJCF actuator 種別が一致しない場合は `LoadConfig()` で失敗します。

## 関連資料

- [Joint Actuator 設定チュートリアル](joint-actuator-ja.md)
- [examples/actuators/joint/README.md](../../examples/actuators/joint/README.md)
- [JSON 設定ガイド](../guide/json-config-ja.md)
- [Sensor/actuator config schemas](../spec/sensor-actuator-config-schema.md)
