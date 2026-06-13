# Joint Actuator 箱庭アセット化チュートリアル

このチュートリアルでは、[Joint Actuator 設定チュートリアル](joint-actuator-ja.md)で作成した MuJoCo joint actuator を、Hakoniwa PDU command に接続します。

Joint actuator はセンサと PDU の向きが逆です。カメラや超音波は MuJoCo の計測結果を PDU へ publish しますが、joint actuator は PDU から command を receive し、`IJointActuator::SetTarget()` で MuJoCo の `data->ctrl[]` に書き込みます。

```text
Python sender asset
        |
        v
Hakoniwa PDU std_msgs/Float64
        |
        v
Float64PduAdapter
        |
        v
example側の command 適用処理
        |
        v
IJointActuator::SetTarget()
        |
        v
MuJoCo data->ctrl[actuator_id]
```

## 1. 全体ワークフロー

```text
  [ C++ Joint Actuator Asset ]
             |
             v (endpoint名指定でロード)
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

C++ 側の受信には、共通の `Float64PduAdapter` を使います。

```cpp
#include "hakoniwa/pdu/adapter/std_msgs/float64.hpp"
```

`Float64PduAdapter` は `std_msgs/Float64` の送受信だけを担当します。
joint actuator への適用は example 側で行います。

```text
std_msgs/Float64 PDU
        |
        v
double target
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

velocity actuator は channel 名を変えます。

```json
"pdu_config": {
  "pdu_name": "velocity_target",
  "update_rate_hz": 50.0,
  "message_type": "std_msgs/Float64"
}
```

| フィールド | 意味 |
| --- | --- |
| `pdu_name` | command PDU の channel 名 |
| `update_rate_hz` | `IActuator::ShouldUpdate()` が command 更新タイミングを判定する周期 |
| `message_type` | PDU message type。ここでは `std_msgs/Float64` |

`pdu_name` は、PDU type list の `name` と、`PduKey(robot_name, channel_name)` の `channel_name` に一致させます。
`update_rate_hz` が `50.0` の場合、`IJointActuator::ShouldUpdate()` は約 20 ms ごとに `true` を返します。値が `0` 以下の場合は、MuJoCo の simulation step ごとに `true` を返します。

## 3. PDU 設定ファイル

この example では、PDU 上の robot 名を `"JointActuatorAsset"`、command channel 名を `"position_target"` / `"velocity_target"` とします。

`PduKey` の第1引数は sender の asset 名ではなく、PDU 定義上の robot 名です。この example では分かりやすさのため、C++ asset 登録名と PDU robot 名をどちらも `"JointActuatorAsset"` にしています。

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

`std_msgs/Float64` の値そのものは `8` bytes ですが、endpoint の channel buffer では PDU metadata `24` bytes も含めるため、`pdu_size` は `32` にします。

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

`robots[].name` が PDU robot 名です。`PduKey("JointActuatorAsset", "position_target")` の `"JointActuatorAsset"` と一致させます。

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

## 4. C++ asset の実装

実装は `examples/actuators/joint/joint-actuator-hakoniwa-asset.cpp` です。

endpoint lifecycle は camera / ultrasonic の Hakoniwa 例と同じ順序にします。

1. `main()` で `hako_asset_register()` を呼ぶ。
2. `main()` で `endpoint.open()` を呼ぶ。
3. `main()` で `endpoint.start()` を呼ぶ。
4. worker thread で `hako_asset_start_no_wait(IsForceStop)` を呼ぶ。
5. `on_initialize` callback で `endpoint.post_start()` を呼ぶ。
6. `post_start()` 完了後、manual timing loop で PDU command を受信する。
7. command を受信できたら example 側で `IJointActuator::SetTarget()` を呼ぶ。
8. main thread の MuJoCo viewer で動作を確認する。

`hako_asset_start_no_wait()` は名前に `no_wait` とありますが、箱庭の start trigger は待ちます。ここでは `hako_asset_start()` ではなく、停止判定 callback を渡せる `hako_asset_start_no_wait(IsForceStop)` を worker thread で呼びます。これにより、MuJoCo viewer を main thread で動かしたまま、viewer close や `q` 入力で asset 側も停止できます。

manual timing loop では、`IJointActuator::ShouldUpdate()` が `true` を返した時だけ command 受信を試みます。受信できた command だけ actuator に反映します。受信できない場合は、前回 target のまま MuJoCo simulation を進めます。
MuJoCo の simulation step は MJCF の `option timestep` から決まり、この example では `0.002` 秒です。C++ asset は loop の最後で `hako_asset_usleep(delta_time_usec)` を呼ぶため、simulation step に対応する実時間 sleep も入ります。

```cpp
while (running.load() && viewer_running.load()) {
    if (endpoint_ready.load()) {
        std::lock_guard<std::mutex> lock(mujoco_mutex);
        if (position_actuator->ShouldUpdate(sim_timestep)) {
            double target = 0.0;
            if (position_adapter->recv(target)) {
                position_actuator->SetTarget(target);
            }
        }
        if (velocity_actuator->ShouldUpdate(sim_timestep)) {
            double target = 0.0;
            if (velocity_adapter->recv(target)) {
                velocity_actuator->SetTarget(target);
            }
        }
        world->advanceTimeStep();
    }
    hako_asset_usleep(delta_time_usec);
}
```

## 5. Python sender の実装

実装は `examples/actuators/joint/joint_actuator_sender.py` です。

Python 側も Hakoniwa asset として登録します。`hakopy.asset_register()` で sender asset を登録し、`on_initialize` で `endpoint.post_start()` を呼び、manual timing loop で `std_msgs/Float64` を送信します。
sender 側は loop の最後で `hakopy.usleep(args.delta_usec)` を呼び、さらに `time.sleep(args.delta_usec / 1_000_000.0)` で実時間 pacing します。既定値では callback loop は 20 ms、command 送信は 20 Hz です。

```python
position_key = PduKey("JointActuatorAsset", "position_target")
velocity_key = PduKey("JointActuatorAsset", "velocity_target")

endpoint.send_by_name(position_key, py_to_pdu_Float64(position_msg))
endpoint.send_by_name(velocity_key, py_to_pdu_Float64(velocity_msg))
```

ここで `"JointActuatorAsset"` は command の宛先になる PDU robot 名です。Python sender 自身の asset 名は既定では `"JointActuatorSender"` で、宛先 robot 名とは別です。
既定の sender は position target を sine wave、velocity target を一定時間ごとのステップ入力として送ります。

## 6. ビルド

リポジトリ root から実行します。

```bash
cmake --build src/cmake-build --target joint-actuator-hakoniwa-asset -j4
```

Python sender の構文確認は次でできます。

```bash
python3 -m py_compile examples/actuators/joint/joint_actuator_sender.py
```

## 7. 実行手順

3つのターミナルを使います。

### ターミナル1: C++ actuator asset

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-hakoniwa-asset
```

MuJoCo viewer が開きます。この時点では箱庭 start trigger を待っています。

### ターミナル2: Python command sender

```bash
python3 examples/actuators/joint/joint_actuator_sender.py
```

sender も箱庭 start trigger を待ちます。
入力を変えたい場合は、例えば `--position-amplitude`、`--position-frequency-hz`、`--velocity-target` を変更します。

### ターミナル3: simulation start

```bash
hako-cmd start
```

start 後、Python sender が `position_target` / `velocity_target` に command を書き、C++ asset が受信して MuJoCo の actuator に反映します。

C++ viewer では position hinge と velocity hinge の動きを確認できます。terminal には position joint angle と velocity joint velocity が定期的に出力されます。

## 8. Viewer 操作

```text
p      : pause / resume MuJoCo stepping
r      : reset MuJoCo simulation state
h      : show help
q / Esc: quit asset and viewer
```

target 値は viewer のキー操作ではなく、Python sender から送られる PDU command で変わります。

## 9. つまづきポイント

- **`pdu_config.pdu_name` と PDU type list の `name` を合わせる**: JSON 側が `"position_target"` なら、`PduKey("JointActuatorAsset", "position_target")` と `joint-actuator-pdutypes.json` の `name` も `"position_target"` にします。

- **`PduKey` は sender asset 名ではなく robot 名 + channel 名**: `PduKey("JointActuatorAsset", "position_target")` の `"JointActuatorAsset"` は `joint-actuator-pdudef-compact.json` の `robots[].name` です。

- **`pdu_size` は metadata を含める**: `std_msgs/Float64` は `8` bytes、endpoint buffer は PDU metadata `24` bytes を含めるため `32` bytes にします。

- **command が来ないと target は更新されない**: `Float64PduAdapter::recv()` が成功したときだけ example 側で `SetTarget()` を呼びます。command sender がまだ書いていない場合、actuator は前回 target のままです。

- **`endpoint.open()` / `endpoint.start()` は `main()` で行う**: `endpoint.post_start()` は `on_initialize` で行います。camera / ultrasonic と同じ lifecycle にそろえています。

- **`spec.type` と MJCF actuator 種別を合わせる**: PDU command が届いていても、JSON `spec.type` と MJCF actuator 種別が一致しない場合は `LoadConfig()` で失敗します。

## 関連資料

- [Joint Actuator 設定チュートリアル](joint-actuator-ja.md)
- [examples/actuators/joint/README.md](../../examples/actuators/joint/README.md)
- [JSON 設定ガイド](../guide/json-config-ja.md)
- [Sensor/actuator config schemas](../spec/sensor-actuator-config-schema.md)
