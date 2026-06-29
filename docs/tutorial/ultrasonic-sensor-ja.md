# 超音波センサー設定チュートリアル

このチュートリアルでは、既存の ultrasonic example を題材に、MuJoCo の MJCF XML に超音波センサーを置き、JSON で `spec` / `mjcf_binding` / `pdu_config` を書き、単体チェックして、C++ example で測定できるところまでを確認します。

対象ファイル:

```text
models/sensors/ultrasonic/ultrasonic-sensor-test.xml
config/sensors/ultrasonic/lego-spike-distance-sensor.json
examples/sensors/ultrasonic/ultrasonic-example.cpp
examples/sensors/ultrasonic/README.md
```

全体の作業順は次です。

1.  MJCF XML にセンサーを定義する。
2.  センサー用 JSON を `spec` / `mjcf_binding` / `pdu_config` で書く。
3.  MJCF と JSON を単体チェックする。
4.  example をビルドして実行する。
5.  C++ で JSON と MJCF サイト名がどう接続されるか確認する。

## 1. MJCF XML にセンサーを置く

超音波センサーは MuJoCo の `<site>` として定義し、そこからレイキャストを行います。
この example では、センサー本体を表す body の中にサイトを置いています。

```xml
<body name="base_footprint" pos="0 0 0.12">
    <freejoint name="base_freejoint"/>
    <geom name="base_geom" type="box" size="0.1 0.1 0.05" pos="0 0 0" rgba="0 0.4 1 1"/>
    <site name="front_ultrasonic_site" pos="0.12 0 0" size="0.02"
        rgba="1 0 0 1"/>
</body>
```

ここで重要な名前は3つです。

| MJCF object | 役割 | JSON での対応 |
| --- | --- | --- |
| `front_ultrasonic_site` | センサーの計測原点/方向を示すサイト | `mjcf_binding.source_site` |
| `base_footprint` | センサーを載せている body | N/A (除外ボディとして使用) |
| `base_freejoint` | example でセンサー body を動かす joint | `ultrasonic-example.cpp` で使用 |

`<site name="front_ultrasonic_site">` の名前は、C++ の `UltrasonicSensor` が測定を行うときに使います。
`freejoint_name` は、この example の `i/k/j/l` 移動用です。

### `<site>` の `pos`, `size`, `rgba`

この example のサイト定義は次の行です。

```xml
<site name="front_ultrasonic_site"
      pos="0.12 0 0"
      size="0.02"
      rgba="1 0 0 1"/>
```

初学者が最初に見るべき属性は `pos`, `size`, `rgba` です。

| 属性 | 意味 | この example での値 |
| --- | --- | --- |
| `pos` | 親 body から見たサイト位置 | `0.12 0 0` |
| `size` | viewer 上で表示される site の大きさ | `0.02` |
| `rgba` | サイトの表示色 (赤 緑 青 透明度) | `1 0 0 1` (赤) |

`pos="0.12 0 0"` は、親 body の原点から見たサイト位置です。
この example では、サイトは `base_footprint` body の前方 (`+X`方向) に 0.12m 移動した位置に置かれています。
超音波センサーは、このサイトのローカル `+X` 軸方向を測定方向として使用します。

## 2. センサー JSON を書く

超音波センサー設定は `config/sensors/ultrasonic/lego-spike-distance-sensor.json` にあります。
現在の推奨形は、概念的に次の3コンテナで読むことです。

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
    "update_rate_hz": 100.0
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "source_site": "front_ultrasonic_site",
    "parent_body": "base_footprint"
  },
  "pdu_config": {
    "pdu_name": "range",
    "update_rate_hz": 100.0,
    "message_type": "sensor_msgs/Range"
  }
}
```

### `spec`

`spec` は超音波センサーそのものの仕様です。

| フィールド | 意味 |
| --- | --- |
| `frame_id` | PDU / ROS-compatible message に載る論理フレーム名 |
| `DetectionDistance.Min`/`Max` | 検出可能な最小/最大距離 |
| `DistanceAccuracy` | 測定精度、ノイズに関する設定 |
| `Cone.RayCount` | 測定に使うレイの数（この例では1） |
| `RadiationType` | 放射タイプ（`ultrasound`） |
| `update_rate_hz` | センサー更新周期 Hz |

`frame_id` は MJCF サイト名ではありません。
MJCF サイト名は `mjcf_binding.source_site` に書きます。

### `DetectionDistance.Min`/`Max`

`DetectionDistance` は、センサーが距離を検出できる範囲です。

```json
"DetectionDistance": {
  "Min": 0.05,
  "Max": 2.0
}
```

`Min` より近い物体、または `Max` より遠い物体は検出できません。
この例では、最小 0.05m (5cm)、最大 2.0m (200cm) の範囲で距離を測定します。

### `mjcf_binding`

`mjcf_binding` は JSON と MJCF XML の object 名を対応づけます。

```json
"mjcf_binding": {
  "source_site": "front_ultrasonic_site",
  "parent_body": "base_footprint"
}
```

`source_site` は MJCF の `<site name="...">` と一致している必要があります。
この名前が間違っていると、C++ 側でセンサー測定が期待通りに動きません。
`parent_body` は、ユーザが「どの body に付いたセンサーか」を読み取るための MJCF binding 情報です。
現在の `ultrasonic-example.cpp` では、自身を除外する body は `kExcludeBodyName` として C++ 側で指定しています。

### `pdu_config`

`pdu_config` は、この超音波センサーを Hakoniwa PDU に接続するときの出力設定です。
この `ultrasonic-example` は PDU への publish は行いません。
ただし、将来 PDU に接続するときに必要な情報として同じ JSON に置いています。

```json
"pdu_config": {
  "pdu_name": "range",
  "update_rate_hz": 100.0,
  "message_type": "sensor_msgs/Range"
}
```

PDU として実際に publish する段階では、`pdu_name` と同じ channel を PDU type list 側にも追加します。
Hakoniwa publisher での送信周期は `spec.update_rate_hz` から初期化される `UltrasonicSensor::ShouldUpdate(dt)` が基準になります。
`pdu_config.update_rate_hz` は PDU 出力として期待する周期を明示するための設定なので、通常は `spec.update_rate_hz` と同じ値にします。
PDU への接続手順は [超音波センサ箱庭アセット化チュートリアル](ultrasonic-sensor-hakoniwa-ja.md) を参照してください。

## 3. 単体チェックする

MJCF と JSON を C++ に組み込む前に、単体チェックします。

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  --json config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

期待する結果は、MJCF load と JSON schema validation が `OK` になることです。

```text
OK   MJCF load: models/sensors/ultrasonic/ultrasonic-sensor-test.xml ...
OK   JSON parse: config/sensors/ultrasonic/lego-spike-distance-sensor.json
OK   JSON schema: config/sensors/ultrasonic/lego-spike-distance-sensor.json -> ...
```

このチェックで分かること:

- MJCF XML が MuJoCo model として load できる。
- JSON が parse できる。
- JSON が `ultrasonic.schema.json` に合っている。

このチェックだけでは分からないこと:

- `source_site` が意図した向きのセンサーか。
- 測定結果が期待した値になるか。
- PDU channel が実行時 endpoint に存在するか。

そこは次の example 実行で確認します。

## 4. example をビルドして実行する

ビルド:

```bash
cmake --build src/cmake-build --target ultrasonic-example
```

実行:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

viewer が開いたら `s` を押すと測定値がターミナルに出力され、ビューア上にレイが表示されます。

```text
Hakoniwa Ultrasonic Sensor Example
model : models/sensors/ultrasonic/ultrasonic-sensor-test.xml
config: config/sensors/ultrasonic/lego-spike-distance-sensor.json
site  : front_ultrasonic_site

Controls:
  i : move forward  (+X)
  k : move backward (-X)
  j : move left     (+Y)
  l : move right    (-Y)
  s : sense and print ultrasonic range
  h : help
  q : quit

base_pos=(0.000, 0.000, 0.100)
> s
range=0.860 m, status=OK, variance=0.000e+00
```

明示的に model / config を渡す場合:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example \
  models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

センサーボディは `i/k/j/l` で動かせます。

```text
i : move forward  (+X)
k : move backward (-X)
j : move left     (+Y)
l : move right    (-Y)
s : sense and print ultrasonic range
h : help
q : quit
```

ビューアには、測定されたレイが表示されます。ヒットした場合は緑色のレイ、ヒットしなかった場合は赤色のレイが表示され、センサーの動作を視覚的に確認できます。

## 5. C++ でどこを見ればよいか

example の主要部分は `examples/sensors/ultrasonic/ultrasonic-example.cpp` です。

JSON を読むところ:

```cpp
// UltrasonicSensor のコンストラクタで、サイト名と除外ボディ名を指定
hako::robots::sensor::ultrasonic::UltrasonicSensor sensor(
    world,
    kSensorSiteName, // "front_ultrasonic_site"
    kExcludeBodyName // "base_footprint"
);
// 設定ファイル (JSON) をロード
if (!sensor.LoadConfig(config_path)) {
    // ... エラー処理
}
```

MuJoCo model を load するところ:

```cpp
auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
world->loadModel(model_path);
```

`UltrasonicSensor` を作るところ:

```cpp
hako::robots::sensor::ultrasonic::UltrasonicSensor sensor(
    world,
    kSensorSiteName,
    kExcludeBodyName);
sensor.LoadConfig(config_path);
```

測定するところ:

```cpp
hako::robots::sensor::ultrasonic::UltrasonicFrame frame {};
sensor.Measure(frame);
```

測定結果を表示するところ:

```cpp
hako::examples::sensors::ultrasonic::PrintFrame(frame);
```

この流れが、他のロボットサンプルへ超音波センサーを組み込むときの基本形です。

## 6. PDU へ接続する場合

この `ultrasonic-example` は PDU への publish は行いませんが、Hakoniwa PDU に接続する場合の PDU マッピングは以下の通りです。

**UltrasonicFrame**

```text
UltrasonicFrame
  frame_id
  range
  variance
  status
```

**ROS-compatible Hakoniwa PDU type: `sensor_msgs/Range`**

```text
sensor_msgs/Range
  std_msgs/Header header
  uint8 radiation_type
  float32 field_of_view
  float32 min_range
  float32 max_range
  float32 range
```

**マッピング**

| UltrasonicConfig/Frame フィールド | Range PDU フィールド |
| :-------------------------------- | :------------------- |
| `UltrasonicConfig.frame_id`       | `Range.header.frame_id` |
| `UltrasonicConfig.radiation_type` | `Range.radiation_type` |
| `UltrasonicConfig.cone.horizontal` | `Range.field_of_view` |
| `UltrasonicConfig.detection_distance.min` | `Range.min_range` |
| `UltrasonicConfig.detection_distance.max` | `Range.max_range` |
| `UltrasonicFrame.range`           | `Range.range` |

`variance` と `status` は内部診断用フィールドであり、ROS `sensor_msgs/Range` の一部ではありません。

PDU に接続する場合は、追加で次を行います。

1.  PDU type list に `pdu_config.pdu_name` の channel を追加する。
2.  C++ 側で `RangePduAdapter` を利用して変換する。
3.  `UltrasonicSensor::Measure()` で得た `UltrasonicFrame` を adapter で送る。
4.  Python 側または別 asset 側で同じ PDU definition を使って読む。

関連する設計は [`sensor-actuator-design.md`](../spec/sensor-actuator-design.md) と [`json-config-ja.md`](../guide/json-config-ja.md) を参照してください。

## よくある失敗

-   `spec.frame_id` と `mjcf_binding.source_site` を混同している。
-   `mjcf_binding.source_site` が MJCF の `<site name="...">` と一致していない。
-   `DetectionDistance.Min` が大きすぎて、近くの物体が検出できない。
-   `DetectionDistance.Max` が小さすぎて、遠くの物体が検出できない。
-   サイトの向きが想定と違う。MJCF の `<site>` の `pos` や親 body の姿勢を確認します。
-   `pdu_config.pdu_name` を書いたが、PDU type list に channel を追加していない。

## 次に読むもの

-   [`examples/sensors/ultrasonic/README.md`](../../examples/sensors/ultrasonic/README.md)
-   [`json-config-ja.md`](../guide/json-config-ja.md)
-   [`mjcf-json-authoring-ja.md`](../guide/mjcf-json-authoring-ja.md)
-   [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md)
