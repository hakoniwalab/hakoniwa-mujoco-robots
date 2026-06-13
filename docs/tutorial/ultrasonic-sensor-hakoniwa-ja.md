# 超音波センサ箱庭アセット化チュートリアル

このチュートリアルでは、[超音波センサー設定チュートリアル](./ultrasonic-sensor-ja.md)で作成した MuJoCo 超音波センサを、Hakoniwa PDU に接続する流れを説明します。

超音波センサの出力は 1 回の測定につき 1 個の距離値です。Hakoniwa PDU では ROS 互換の `sensor_msgs/Range` として送ります。

実装例は `examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset.cpp` と
`examples/sensors/ultrasonic/read_range.py` にあります。

---

## 1. 全体ワークフロー

```text
  [ C++ Ultrasonic Asset ]
             |
             v (Endpoint名指定でロード)
    [ ultrasonic_endpoint.json ]
             |
    +--------+--------+----------------------+
    | (参照)           | (参照)                | (参照)
    v                 v                      v
[ ultrasonic-pdudef-compact.json ] [ cache/buffer.json ] [ comm/shm_ultrasonic_comm.json ]
    |
    v (参照)
[ ultrasonic-pdutypes.json ]
```

C++ 側の送信には、既存の `RangePduAdapter` を使います。

```cpp
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
```

`RangePduAdapter::send(config, frame)` は、以下の変換を内部で行います。

```text
UltrasonicConfig + UltrasonicFrame
        |
        v
sensor_msgs/Range PDU
```

---

## 2. センサ JSON

超音波センサの設定は `config/sensors/ultrasonic/lego-spike-distance-sensor.json` です。
現在の形式は、カメラセンサと同じく `spec` / `mjcf_binding` / `pdu_config` に分けます。

```json
{
  "$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json",
  "spec": {
    "frame_id": "spike_distance_sensor_link",
    "DetectionDistance": {
      "Min": 0.05,
      "Max": 2.0
    },
    "DistanceAccuracy": [
      {
        "Range": {
          "Min": 0.05,
          "Max": 2.0
        },
        "StdDev": 0.0,
        "Precision": 0.0,
        "NoiseDistribution": "none"
      }
    ],
    "Cone": {
      "Horizontal": 0.0,
      "Vertical": 0.0,
      "RayCount": 1
    },
    "RadiationType": "ultrasound",
    "UpdateRate": 100.0
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "parent_body": "base_footprint",
    "source_site": "front_ultrasonic_site"
  },
  "pdu_config": {
    "pdu_name": "range",
    "update_rate_hz": 100.0,
    "message_type": "sensor_msgs/Range"
  }
}
```

### `spec`

`spec` はセンサそのものの物理仕様です。

- `frame_id`: publish される Range message の frame 名
- `DetectionDistance.Min`: 最小検出距離 `[m]`
- `DetectionDistance.Max`: 最大検出距離 `[m]`
- `DistanceAccuracy`: 距離ごとのノイズ・分解能
- `Cone`: 超音波の検出コーン
- `RadiationType`: `ultrasound` または `infrared`
- `UpdateRate`: センサ更新周期 `[Hz]`

### `mjcf_binding`

`mjcf_binding` は JSON と MuJoCo XML の object 名を結びつけます。

```json
"mjcf_binding": {
  "source_site": "front_ultrasonic_site"
}
```

`source_site` は MJCF の `<site name="front_ultrasonic_site">` と一致させます。
この site の位置が測定原点になり、site の local `+X` 方向が測定方向になります。

### `pdu_config`

`pdu_config` は Hakoniwa PDU として出すときの設定です。

```json
"pdu_config": {
  "pdu_name": "range",
  "update_rate_hz": 100.0,
  "message_type": "sensor_msgs/Range"
}
```

`pdu_name` は、後述の `PduKey(asset_name, pdu_name)` と PDU type list の `name` に一致させます。

---

## 3. PDU 設定ファイル

ここでは、C++ publisher の asset 名を `"UltrasonicAsset"`、PDU channel 名を `"range"` とします。

### `config/ultrasonic-pdutypes.json`

```json
[
  {
    "channel_id": 0,
    "pdu_size": 184,
    "name": "range",
    "type": "sensor_msgs/Range"
  }
]
```

`sensor_msgs/Range` の基本サイズは `160` bytes です。endpoint の channel buffer では PDU metadata `24` bytes も含めるため、`pdu_size` は `184` にします。

```text
184 = 24 (PDU metadata) + 160 (sensor_msgs/Range)
```

### `config/ultrasonic-pdudef-compact.json`

```json
{
  "paths": [
    {
      "id": "ultrasonic-range",
      "path": "ultrasonic-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "UltrasonicAsset",
      "pdutypes_id": "ultrasonic-range"
    }
  ]
}
```

### `config/endpoint/ultrasonic_endpoint.json`

```json
{
  "name": "ultrasonic_endpoint",
  "pdu_def_path": "../ultrasonic-pdudef-compact.json",
  "cache": "cache/buffer.json",
  "comm": "comm/shm_ultrasonic_comm.json"
}
```

### `config/endpoint/comm/shm_ultrasonic_comm.json`

```json
{
  "protocol": "shm",
  "impl_type": "callback",
  "name": "ultrasonic_shm",
  "direction": "inout",
  "io": {
    "robots": [
      {
        "name": "UltrasonicAsset",
        "pdu": [
          { "name": "range", "notify_on_recv": false }
        ]
      }
    ]
  }
}
```

画像と同じく、常に最新値だけ読めればよいので cache は `config/endpoint/cache/buffer.json` の `latest` を使います。

---

## 4. C++ Publisher の実装方針

`ultrasonic-example.cpp` で行っている測定処理に、Hakoniwa endpoint と `RangePduAdapter` を追加します。

重要な点は、PDU 変換を自前で書かないことです。
既存の adapter を使います。

```cpp
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
#include "hakoniwa/pdu/endpoint.hpp"

std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter> range_adapter;
std::atomic_bool endpoint_ready {false};
```

### Endpoint lifecycle

camera publisher と同じ順序にします。
`hako_asset_start_no_wait()` は名前に `no_wait` とありますが、箱庭の start trigger は待ちます。
ここでは `hako_asset_start()` ではなく、停止判定 callback を渡せる `hako_asset_start_no_wait(IsForceStop)` を worker thread で呼びます。
これにより、MuJoCo viewer を main thread で動かしたまま、viewer close や `q` 入力で asset 側も停止できます。

1. `main()` で `hako_asset_register()` を呼ぶ。
2. `main()` で `endpoint.open()` を呼ぶ。
3. `main()` で `endpoint.start()` を呼ぶ。
4. worker thread で `hako_asset_start_no_wait()` を呼ぶ。
5. `on_initialize` callback で `endpoint.post_start()` を呼ぶ。
6. `post_start()` 完了後、simulation step ごとに `UltrasonicSensor::ShouldUpdate(step_dt)` を確認する。
7. `ShouldUpdate(step_dt)` が `true` の周期で測定し、測定値を `RangePduAdapter::send()` で送る。
8. 終了時に `endpoint.stop()` / `endpoint.close()` を呼ぶ。

`endpoint.open()` / `endpoint.start()` を `on_manual_timing_control` の中で呼ばないでください。
`endpoint.post_start()` は `main()` ではなく `on_initialize` callback で呼びます。

### 送信コードの要点

```cpp
const hakoniwa::pdu::PduKey range_key {
    "UltrasonicAsset",
    ultrasonic_sensor->GetConfig().pdu_config.pdu_name
};

range_adapter =
    std::make_unique<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter>(
        *endpoint,
        range_key);
```

simulation を 1 step 進めたあと、センサの更新周期に達していれば測定します。
`step_dt` は MuJoCo model の `model->opt.timestep` です。
`ShouldUpdate(step_dt)` は内部のセンサ用クロックを進め、JSON の `spec.UpdateRate` に従って `true` を返します。
測定したら、`UltrasonicConfig` と `UltrasonicFrame` を渡して送信します。

```cpp
const double step_dt = world->getModel()->opt.timestep;
const auto& config = ultrasonic_sensor->GetConfig();

while (running_flag) {
    if (endpoint_ready.load()) {
        std::lock_guard<std::mutex> lock(mujoco_mutex);
        world->advanceTimeStep();

        if (ultrasonic_sensor->ShouldUpdate(step_dt)) {
            hako::robots::sensor::ultrasonic::UltrasonicFrame frame {};
            ultrasonic_sensor->Measure(frame);

            if (!range_adapter->send(config, frame)) {
                std::cerr << "[WARN] Failed to send ultrasonic range PDU." << std::endl;
            }
        }
    }

    hako_asset_usleep(static_cast<hako_time_t>(step_dt * 1e6));
}
```

`RangePduAdapter::send(config, frame)` が内部で `sensor_msgs/Range` に変換します。
対応関係は次の通りです。

```text
UltrasonicConfig.frame_id                -> Range.header.frame_id
UltrasonicConfig.radiation_type          -> Range.radiation_type
UltrasonicConfig.cone.horizontal         -> Range.field_of_view
UltrasonicConfig.detection_distance.min  -> Range.min_range
UltrasonicConfig.detection_distance.max  -> Range.max_range
UltrasonicFrame.range                    -> Range.range
```

---

## 5. Python Reader の実装方針

Python 側も camera reader と同じく、`hakopy` で箱庭 asset として登録し、`hakoniwa_pdu_endpoint` で endpoint を開きます。

概略は次の形です。

```python
import hakopy
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Range import pdu_to_py_Range

endpoint = Endpoint("ultrasonic_reader", "inout")
range_key = PduKey("UltrasonicAsset", "range")


def on_initialize(_context):
    endpoint.post_start()
    return 0


def on_manual_timing_control(_context):
    pdu_size = endpoint.get_pdu_size(range_key)

    while True:
        raw = endpoint.recv_by_name(range_key, pdu_size)
        if raw:
            try:
                msg = pdu_to_py_Range(raw)
            except Exception:
                # publisher が初回値を書き込む前の初期データは skip する。
                hakopy.usleep(30_000)
                continue

            print(
                f"range={msg.range:.3f} "
                f"min={msg.min_range:.3f} "
                f"max={msg.max_range:.3f} "
                f"fov={msg.field_of_view:.3f}"
            )

        if hakopy.usleep(30_000) is False:
            break

    return 0
```

Python reader 自身の asset 名と、読む対象の publisher asset 名を混同しないでください。

```text
reader asset name       : UltrasonicReader
publisher asset name    : UltrasonicAsset
PDU key                 : PduKey("UltrasonicAsset", "range")
```

---

## 6. 実行手順

実行手順は次の通りです。

Terminal 1: C++ publisher

```bash
cmake --build src/cmake-build --target ultrasonic-hakoniwa-asset -j4
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset
```

Terminal 2: Python reader

```bash
python3 examples/sensors/ultrasonic/read_range.py
```

Terminal 3: start trigger

```bash
hako-cmd start
```

`hako-cmd start` 後、C++ publisher の `on_initialize` で `endpoint.post_start()` が呼ばれます。
その後、C++ publisher は MuJoCo simulation を進め、`UltrasonicSensor::ShouldUpdate(step_dt)` が `true` になった周期で測定値を `RangePduAdapter` 経由の `sensor_msgs/Range` として送信します。

---

## 7. つまづきポイント

- **`RangePduAdapter` を使う**:
  `UltrasonicFrame` を直接 PDU buffer に詰めないでください。`RangePduAdapter::send(config, frame)` を使います。

- **PDU type を手書き定義しない**:
  `sensor_msgs/Range` は PDU registry に存在します。チュートリアル内で ROS message 構造を独自に `def` する必要はありません。

- **`pdu_size` は metadata を含める**:
  `sensor_msgs/Range` は `160` bytes、endpoint buffer は PDU metadata `24` bytes を含めるため `184` bytes にします。

- **`pdu_config.pdu_name` と PDU type list の `name` を合わせる**:
  JSON 側が `"range"` なら、`PduKey("UltrasonicAsset", "range")` と `ultrasonic-pdutypes.json` の `name` も `"range"` にします。

- **送信周期は `UltrasonicSensor::ShouldUpdate(step_dt)` に任せる**:
  `step_dt` は MuJoCo model の `model->opt.timestep` です。
  `ShouldUpdate(step_dt)` は内部のセンサ用クロックを進め、`spec.UpdateRate` の周期に達したときだけ `true` を返します。
  PDU 送信用に別の周期カウンタを持つと、センサ設定と publish 周期がずれやすくなります。

- **`mjcf_binding.source_site` と `frame_id` は別物**:
  `source_site` は MuJoCo XML の site 名です。`frame_id` は publish される `sensor_msgs/Range.header.frame_id` です。

- **初回未書き込み PDU は skip する**:
  reader が publisher の初回送信前に shared memory を読むと decode error になることがあります。最初の有効値が来るまで skip します。
