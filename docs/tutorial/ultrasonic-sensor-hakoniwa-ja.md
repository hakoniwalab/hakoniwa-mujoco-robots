# 超音波センサ箱庭アセット化チュートリアル

このチュートリアルでは、[超音波センサー設定チュートリアル](./ultrasonic-sensor-ja.md)で作成した MuJoCo 超音波センサを、Hakoniwa PDU に接続する流れを説明します。

超音波センサの出力は 1 回の測定につき 1 個の距離値です。Hakoniwa PDU では ROS 互換の `sensor_msgs/Range` として送ります。

実装例は `examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset.cpp` と
`examples/sensors/ultrasonic/read_range.py` にあります。

---

## 1. 全体ワークフロー

このチュートリアルでは、設定ファイル群を直接ばらばらに追うのではなく、asset manifest を入口にします。

```text
config/assets/ultrasonic-hakoniwa-asset.json
  |
  +-- model    -> models/sensors/ultrasonic/ultrasonic-sensor-test.xml
  +-- component-> config/sensors/ultrasonic/lego-spike-distance-sensor.json
  +-- pdu_def  -> config/ultrasonic-pdudef-compact.json
  +-- endpoint -> config/endpoint/ultrasonic_endpoint.json
```

それぞれの責務は次の通りです。

- component JSON: `spec` / `mjcf_binding` / `pdu_config` の正本
- MJCF: MuJoCo 上の site/body/geom/camera などの正本
- PDU definition: PDU robot 名、channel 名、message type、PDU size の正本
- endpoint/comm: 通信方式と channel の正本
- manifest: それらを束ねる薄い目次

まず manifest から一括検証します。

```bash
python3 tools/validate_assets.py \
  --manifest config/assets/ultrasonic-hakoniwa-asset.json
```

人間向けに接続関係を見るには inspector を使います。

```bash
python3 tools/inspect_asset_manifest.py \
  config/assets/ultrasonic-hakoniwa-asset.json
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

`pdu_name` は、後述の `PduKey(robot_name, channel_name)` の `channel_name` と PDU type list の `name` に一致させます。

---

## 3. PDU 設定ファイル

ここでは、PDU 上の robot 名を `"UltrasonicAsset"`、PDU channel 名を `"range"` とします。
この example では C++ publisher の asset 登録名も `"UltrasonicAsset"` にしていますが、`PduKey` の第1引数は asset 登録名ではなく PDU 定義上の robot 名です。

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

`examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset.cpp` で行っている測定処理を、
manifest と Hakoniwa lifecycle wrapper を入口にした形で整理します。

重要な点は、PDU 変換を自前で書かないことです。
既存の adapter を使います。

```cpp
#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"

std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter> range_adapter;
```

### Hakoniwa lifecycle

箱庭アセットとしての定型処理は `HakoniwaAssetLifecycle` に寄せるのが、現在の推奨形です。
`endpoint.open()` / `endpoint.start()` / `endpoint.post_start()` / `endpoint.stop()` / `endpoint.close()` の順序をアプリケーション側で手書きしないためです。

1. `main()` で manifest と sensor JSON を読み込む。
2. `main()` で MuJoCo model を読み込む。
3. `main()` で `HakoniwaAssetLifecycle::OpenEndpoint()` を呼ぶ。
4. worker thread で `HakoniwaAssetLifecycle::RegisterAndRunAssetNoWait()` を呼ぶ。
5. wrapper 内の `on_initialize` callback で `endpoint.post_start()` が呼ばれる。
6. `post_start()` 完了後、manual timing callback で `UltrasonicSensor::ShouldUpdate(step_dt)` を確認する。
7. `ShouldUpdate(step_dt)` が `true` の周期で測定し、測定値を `RangePduAdapter::send()` で送る。
8. 終了時に `HakoniwaAssetLifecycle::StopAndClose()` が `endpoint.stop()` / `endpoint.close()` を行う。

`endpoint.open()` / `endpoint.start()` を `on_manual_timing_control` の中で呼ばないでください。
`endpoint.post_start()` は `main()` ではなく `on_initialize` callback で呼びます。

この順序は重要ですが、アプリケーションコードでは manifest から設定を解決したうえで wrapper に任せます。

```cpp
hako::robots::config::AssetManifest manifest {};
std::string manifest_error;
if (!hako::robots::config::LoadAssetManifestFromJson(
        "config/assets/ultrasonic-hakoniwa-asset.json",
        manifest,
        &manifest_error))
{
    return 1;
}

const auto* component = manifest.FindComponent("front_ultrasonic");
if (component == nullptr || component->pdu_robot.empty()) {
    return 1;
}

const std::string model_path = manifest.model;
const std::string sensor_config_path = component->config;
const std::string pdu_def_path = manifest.pdu_def;
const std::string endpoint_config_path = manifest.endpoint;
const std::string pdu_robot_name = component->pdu_robot;
const std::string asset_registration_name = pdu_robot_name;
```

その後、endpoint JSON の `name` を endpoint 名として読み、`HakoniwaAssetLifecycle` を作ります。

```cpp
hako::robots::runtime::HakoniwaAssetLifecycle lifecycle({
    endpoint_name,
    endpoint_config_path,
    asset_registration_name,
    pdu_def_path,
    static_cast<hako_time_t>(world->getModel()->opt.timestep * 1.0e6),
    HAKO_ASSET_MODEL_PLANT
});

if (!lifecycle.OpenEndpoint(&error_message)) {
    return 1;
}

std::thread asset_thread([&]() {
    lifecycle.RegisterAndRunAssetNoWait(
        [&](hakoniwa::pdu::Endpoint& endpoint) {
            return run_manual_timing_control(endpoint);
        },
        [&]() {
            return running.load() && app_state.running.load() ? 0 : 1;
        },
        {},
        &error_message);
});
```

`RegisterAndRunAssetNoWait()` の内部で `hako_conductor_start()`、`hako_asset_register()`、`hako_asset_start_no_wait()` が呼ばれます。
viewer を閉じた場合や `q` が押された場合は、force stop callback によって start 待ち状態からも抜けられます。
`endpoint.post_start()` は wrapper の `on_initialize` でだけ呼ばれます。

### 送信コードの要点

PDU robot 名は manifest の `components[].pdu_robot`、channel 名は component JSON の `pdu_config.pdu_name` から決めます。
この example では、それぞれ `"UltrasonicAsset"` と `"range"` です。

```cpp
const hakoniwa::pdu::PduKey range_key {
    pdu_robot_name,
    ultrasonic_sensor->GetConfig().pdu_config.pdu_name
};

range_adapter =
    std::make_unique<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter>(
        endpoint,
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
    if (lifecycle.IsReady()) {
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
`PduKey` は `PduKey(robot_name, channel_name)` です。
第1引数は Python reader 自身の asset 名ではなく、読みたい PDU を持つ robot 名です。

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

Python reader 自身の asset 登録名と、読む対象の PDU robot 名を混同しないでください。

```text
reader asset registration name : UltrasonicReader
PDU robot name                 : UltrasonicAsset
PDU channel name               : range
PDU key                        : PduKey("UltrasonicAsset", "range")
```

---

## 6. 実行手順

実行手順は次の通りです。

Terminal 1: C++ publisher

```bash
cmake --build src/cmake-build --target ultrasonic-hakoniwa-asset -j4
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset
```

manifest を差し替える場合は、第1引数に manifest path を渡します。

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset \
  config/assets/ultrasonic-hakoniwa-asset.json
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

このチュートリアルでは、実行時の意味を保ったまま framework 化した構成として説明しています。
違いは、`on_initialize` / `post_start` / endpoint stop/close などの lifecycle 定型処理を `HakoniwaAssetLifecycle` に隠す点です。

---

## 7. つまづきポイント

- **`RangePduAdapter` を使う**:
  `UltrasonicFrame` を直接 PDU buffer に詰めないでください。`RangePduAdapter::send(config, frame)` を使います。

- **manifest は値を再定義しない**:
  `DetectionDistance`、`source_site`、`pdu_name` は manifest ではなく component JSON に書きます。
  manifest は `model`、`pdu_def`、`endpoint`、component JSON の参照だけを持ちます。

- **`pdu_robot` と `pdu_config.pdu_name` を分けて考える**:
  `pdu_robot` は manifest の `components[].pdu_robot` です。
  `pdu_config.pdu_name` は component JSON の channel 名です。
  この2つから `PduKey("UltrasonicAsset", "range")` を作ります。

- **lifecycle の順序を手で崩さない**:
  `endpoint.open()` / `endpoint.start()` は manual timing callback の外で行い、`endpoint.post_start()` は `on_initialize` に紐づけます。
  `HakoniwaAssetLifecycle` を使うと、この順序をアプリケーション側に露出させずに済みます。

- **PDU type を手書き定義しない**:
  `sensor_msgs/Range` は PDU registry に存在します。チュートリアル内で ROS message 構造を独自に `def` する必要はありません。

- **`pdu_size` は metadata を含める**:
  `sensor_msgs/Range` は `160` bytes、endpoint buffer は PDU metadata `24` bytes を含めるため `184` bytes にします。

- **`pdu_config.pdu_name` と PDU type list の `name` を合わせる**:
  JSON 側が `"range"` なら、`PduKey("UltrasonicAsset", "range")` の channel 名と `ultrasonic-pdutypes.json` の `name` も `"range"` にします。

- **`PduKey` は asset 名ではなく robot 名 + channel 名**:
  `PduKey("UltrasonicAsset", "range")` の `"UltrasonicAsset"` は `ultrasonic-pdudef-compact.json` の `robots[].name` です。
  この example では asset 登録名も同じ文字列にしていますが、`PduKey` の概念としては PDU 定義上の robot 名を指定します。

- **送信周期は `UltrasonicSensor::ShouldUpdate(step_dt)` に任せる**:
  `step_dt` は MuJoCo model の `model->opt.timestep` です。
  `ShouldUpdate(step_dt)` は内部のセンサ用クロックを進め、`spec.UpdateRate` の周期に達したときだけ `true` を返します。
  PDU 送信用に別の周期カウンタを持つと、センサ設定と publish 周期がずれやすくなります。

- **`mjcf_binding.source_site` と `frame_id` は別物**:
  `source_site` は MuJoCo XML の site 名です。`frame_id` は publish される `sensor_msgs/Range.header.frame_id` です。

- **初回未書き込み PDU は skip する**:
  reader が publisher の初回送信前に shared memory を読むと decode error になることがあります。最初の有効値が来るまで skip します。
