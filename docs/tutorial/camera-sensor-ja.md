# カメラセンサー設定チュートリアル

このチュートリアルでは、既存の color camera example を題材に、MuJoCo の MJCF XML にカメラを置き、JSON で `spec` / `mjcf_binding` / `pdu_config` を書き、単体チェックして、C++ example で撮影できるところまでを確認します。

対象ファイル:

```text
models/sensors/color_camera/color-camera-sample.xml
config/sensors/color_camera/simple-color-camera.json
examples/sensors/color_camera/color-camera-example.cpp
examples/sensors/color_camera/README.md
```

全体の作業順は次です。

1. MJCF XML にカメラを定義する。
2. カメラ用 JSON を `spec` / `mjcf_binding` / `pdu_config` で書く。
3. MJCF と JSON を単体チェックする。
4. example をビルドして実行する。
5. C++ で JSON と MJCF camera name がどう接続されるか確認する。

## 1. MJCF XML にカメラを置く

カメラは MuJoCo の `<camera>` として定義します。
この example では、カメラ本体を表す body の中に fixed camera を置いています。

```xml
<body name="color_sensor_body" pos="0 0 0.45">
  <freejoint name="color_sensor_freejoint"/>
  <geom name="color_sensor_case" type="box" size="0.04 0.05 0.04" material="sensor_mat"/>
  <geom name="color_sensor_lens" type="cylinder" size="0.025 0.012" pos="0.045 0 0"
        euler="0 1.57079632679 0" material="lens_mat"/>
  <geom name="color_sensor_forward_marker" type="capsule" size="0.008 0.12"
        fromto="0.07 0 0 0.31 0 0" material="camera_forward_mat"/>
  <camera name="color_camera" mode="fixed" pos="0 0 0" xyaxes="0 -1 0 0 0 1"/>
</body>
```

ここで重要な名前は3つです。

| MJCF object | 役割 | JSON での対応 |
| --- | --- | --- |
| `color_camera` | MuJoCo camera 名 | `mjcf_binding.camera_name` |
| `color_sensor_body` | カメラを載せている body | `mjcf_binding.body_name` |
| `color_sensor_freejoint` | example でカメラ body を動かす joint | `mjcf_binding.freejoint_name` |

`<camera name="color_camera">` の名前は、C++ の `CameraSensor` が画像を撮るときに使います。
`freejoint_name` は、この example の `i/k/j/l` 移動用です。固定カメラだけを読む用途なら必須ではありません。

### `<camera>` の `mode`, `pos`, `xyaxes`

この example の camera 定義は次の行です。

```xml
<camera name="color_camera" mode="fixed" pos="0 0 0" xyaxes="0 -1 0 0 0 1"/>
```

初学者が最初に見るべき属性は `mode`, `pos`, `xyaxes` です。

| 属性 | 意味 | この example での値 |
| --- | --- | --- |
| `mode` | カメラ姿勢をどう扱うか | `fixed` |
| `pos` | 親 body から見たカメラ位置 | `0 0 0` |
| `xyaxes` | カメラのローカル X 軸と Y 軸の向き | `0 -1 0 0 0 1` |

`mode="fixed"` は、カメラが親 body に固定されるという意味です。
この example では `<camera>` が `color_sensor_body` の中にあるので、カメラは `color_sensor_body` と一緒に動きます。
`i/k/j/l` で `color_sensor_freejoint` を動かすと、body と一緒に camera も動きます。

`pos="0 0 0"` は、親 body の原点から見たカメラ位置です。
この example では、camera は `color_sensor_body` の原点に置かれています。
カメラを body の前方にずらしたい場合は、たとえば `pos="0.05 0 0"` のようにします。

`xyaxes` は少し難しいですが、カメラの向きを決める指定です。
6個の数字を、前半3個と後半3個に分けて読みます。

```text
xyaxes="0 -1 0  0 0 1"
        X axis  Y axis
```

この example では次の意味です。

```text
camera local X axis = world/body の -Y 方向
camera local Y axis = world/body の +Z 方向
```

MuJoCo の camera は、`xyaxes` で指定した X 軸と Y 軸からカメラ姿勢を決めます。
カメラの「前方」は、指定した X 軸と Y 軸に直交する方向として決まります。
この example では、黄色の capsule `color_sensor_forward_marker` が示す向きと撮影方向が合うように `xyaxes` を設定しています。

最初は `xyaxes` を暗算で作ろうとせず、既存 example の値をコピーして、viewer と PNG で向きを確認しながら調整するのが現実的です。
向きが分からなくなったら、次を確認してください。

- camera body に「前方マーカー」を置く。
- PNG に期待した物体が写るか確認する。
- `pos` で位置を動かし、`xyaxes` で向きを変える。
- `xyaxes` の前半3個と後半3個は、互いに直交する単位ベクトルにする。

## 2. カメラ JSON を書く

カメラ設定は `config/sensors/color_camera/simple-color-camera.json` にあります。
現在の推奨形は、概念的に次の3コンテナで読むことです。

```json
{
  "$schema": "../schema/camera.schema.json",
  "spec": {
    "frame_id": "color_camera_frame",
    "update_rate": 10,
    "horizontal_fov": 1.2,
    "image": {
      "width": 256,
      "height": 128,
      "format": "R8G8B8"
    },
    "clip": {
      "near": 0.05,
      "far": 10.0
    },
    "noise": {
      "type": "none",
      "mean": 0.0,
      "stddev": 0.0
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "camera_name": "color_camera",
    "body_name": "color_sensor_body",
    "freejoint_name": "color_sensor_freejoint"
  },
  "pdu_config": {
    "pdu_name": "camera_image",
    "update_rate_hz": 10,
    "message_type": "sensor_msgs/Image"
  }
}
```

### `spec`

`spec` はカメラセンサーそのものの仕様です。

| フィールド | 意味 |
| --- | --- |
| `frame_id` | PDU / ROS-compatible message に載る論理フレーム名 |
| `update_rate` | センサー更新周期 Hz |
| `horizontal_fov` | 水平方向 FOV rad |
| `image.width` / `image.height` | 画像サイズ |
| `image.format` | RGB などのピクセル形式 |
| `clip.near` / `clip.far` | 描画対象の近端 / 遠端 |
| `noise` | 画像ノイズ設定 |

`frame_id` は MJCF camera name ではありません。
MJCF camera name は `mjcf_binding.camera_name` に書きます。

### `image.width` / `image.height` と MuJoCo の offscreen buffer

カメラセンサーの画像サイズは、viewer window の大きさではなく JSON の `image.width` / `image.height` で決まります。

このカメラセンサーは、MuJoCo viewer の画面を切り取っているのではありません。
MuJoCo の offscreen buffer にカメラ視点を描画し、そのピクセルを読み出しています。

そのため、MJCF 側にも offscreen buffer の最大サイズを指定しておくと安全です。

```xml
<visual>
  <global offwidth="256" offheight="128"/>
  <map znear="0.05" zfar="10"/>
</visual>
```

- `offwidth` / `offheight`: offscreen rendering buffer の最大サイズ。
- `map.znear` / `map.zfar`: MuJoCo model 側の既定深度範囲。カメラセンサーの capture 時は JSON の `clip.near` / `clip.far` を一時的に反映します。

viewer のメイン表示は on-screen rendering なので、window サイズに応じて大きく表示されます。
一方、カメラセンサー画像は offscreen rendering なので、JSON の `image.width` / `image.height` と MJCF の `offwidth` / `offheight` の範囲で決まります。

例えば JSON を `320x240` に変更するなら、MJCF 側も少なくとも次のようにします。

```xml
<visual>
  <global offwidth="320" offheight="240"/>
  <map znear="0.05" zfar="10"/>
</visual>
```

複数カメラを使う場合、`offwidth` / `offheight` はカメラごとではなく model 全体で1つです。
一番大きい画像サイズのカメラに合わせて設定します。

### `clip.near` / `clip.far`

`clip` は、カメラが描画する距離範囲です。

```json
"clip": {
  "near": 0.05,
  "far": 10.0
}
```

`near` より近いもの、または `far` より遠いものは、カメラ画像に正しく写らないことがあります。
この値はメートル単位です。広い MuJoCo world でも、センサー実行時にはこの JSON 値が投影の clip plane として使われます。

```text
camera
  |--- near より近い範囲: 描画対象外
  |--- near から far まで: 描画対象
  |--- far より遠い範囲: 描画対象外
```

この example では、カメラの前にある赤・緑・青パネルが約 1 m 先にあるので、`near=0.05`、`far=10.0` なら十分に範囲内です。

よくある設定ミス:

- `near` が大きすぎて、近くの物体が消える。
- `far` が小さすぎて、遠くの物体が消える。
- `near <= 0` にして schema validation で失敗する。
- `near` と `far` の差が極端で、depth camera の精度確認が難しくなる。

RGB カメラでも `clip` は必要です。
depth camera では特に重要で、`clip.near` / `clip.far` が depth 値の有効範囲にも関係します。

### `mjcf_binding`

`mjcf_binding` は JSON と MJCF XML の object 名を対応づけます。

```json
"mjcf_binding": {
  "camera_name": "color_camera",
  "body_name": "color_sensor_body",
  "freejoint_name": "color_sensor_freejoint"
}
```

`camera_name` は MJCF の `<camera name="...">` と一致している必要があります。
この名前が間違っていると、C++ 側で camera capture が期待通りに動きません。

### `pdu_config`

`pdu_config` は、このカメラを Hakoniwa PDU に接続するときの出力設定です。
この color camera example は PNG 出力の単体 example なので、endpoint への publish はまだ行いません。
ただし、将来 PDU に接続するときに必要な情報として同じ JSON に置いています。

```json
"pdu_config": {
  "pdu_name": "camera_image",
  "update_rate_hz": 10,
  "message_type": "sensor_msgs/Image"
}
```

PDU として実際に publish する段階では、`pdu_name` と同じ channel を PDU type list 側にも追加します。
Hakoniwa publisher での送信周期は `spec.update_rate` から初期化される `CameraSensor::ShouldUpdate(dt)` が基準になります。
`pdu_config.update_rate_hz` は PDU 出力として期待する周期を明示するための設定なので、通常は `spec.update_rate` と同じ値にします。
PDU への接続手順は [カメラセンサ箱庭アセット化チュートリアル](camera-sensor-hakoniwa-ja.md) を参照してください。

## 3. 単体チェックする

MJCF と JSON を C++ に組み込む前に、単体チェックします。

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/color_camera/color-camera-sample.xml \
  --json config/sensors/color_camera/simple-color-camera.json
```

期待する結果は、MJCF load と JSON schema validation が `OK` になることです。

```text
OK   MJCF load: models/sensors/color_camera/color-camera-sample.xml ...
OK   JSON parse: config/sensors/color_camera/simple-color-camera.json
OK   JSON schema: config/sensors/color_camera/simple-color-camera.json -> ...
```

このチェックで分かること:

- MJCF XML が MuJoCo model として load できる。
- JSON が parse できる。
- JSON が `camera.schema.json` に合っている。

このチェックだけでは分からないこと:

- `camera_name` が意図した向きのカメラか。
- 撮影結果が期待した画角になっているか。
- PDU channel が実行時 endpoint に存在するか。

そこは次の example 実行で確認します。

## 4. example をビルドして実行する

ビルド:

```bash
cmake --build src/cmake-build --target color-camera-example
```

実行:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

viewer が開いたら `s` を押すと PNG が出力されます。

```text
./camera_color_sample.png
```

明示的に model / config / output path を渡す場合:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example \
  models/sensors/color_camera/color-camera-sample.xml \
  config/sensors/color_camera/simple-color-camera.json \
  ./camera_my_color_sample.png
```

カメラ body は `i/k/j/l` で動かせます。

```text
i : move camera forward  (+X)
k : move camera backward (-X)
j : move camera left     (+Y)
l : move camera right    (-Y)
s : capture and write PNG
q : quit
```

PNG に赤、緑、青のパネルが写れば、MJCF camera、JSON spec、C++ capture path がつながっています。
ターミナルには、RGB の生バイト値と、中央ピクセルを正規化した RGBA 値も出力されます。

```text
Captured color_camera 256x128 format=R8G8B8
left    pixel=( 42,  64) rgb=(...)
center  pixel=(128,  64) rgb=(...)
right   pixel=(213,  64) rgb=(...)
center_rgba pixel=(128, 64) rgba=(..., ..., ..., 1.000)
region_average_rgba rect=(120, 56, 16, 16) rgba=(..., ..., ..., 1.000)
```

`rgb=(...)` は 0-255 の整数値です。
`rgba=(...)` は `RGBAColor` として扱いやすい 0.0-1.0 の値です。
`a` は RGB 画像から作るため `1.000` になります。

ここで注意したいのは、`CameraSensor` の主出力は画像全体だという点です。

```cpp
ImageFrame frame {};
camera_sensor->Capture(frame);
```

`CaptureAsRGBA(x, y)` は、画像全体ではなく、指定した 1 ピクセルを `RGBAColor` として読む convenience API です。
一般的な単体カラーセンサに近い使い方をしたい場合に、「カメラ画像の一点を色センサ値として読む」ために使えます。

```cpp
RGBAColor color = camera_sensor->CaptureAsRGBA(x, y);
```

一点ではなく、一定範囲の平均色を読みたい場合は `CaptureRegionAverageRGBA(x, y, width, height)` を使います。

```cpp
RGBAColor average = camera_sensor->CaptureRegionAverageRGBA(x, y, width, height);
```

領域平均は、1 ピクセルだけ読むよりノイズや境界の影響を受けにくくなります。

つまり、この example では次の2つを同時に確認しています。

| API | 取得するもの | 用途 |
| --- | --- | --- |
| `Capture(frame)` | RGB 画像全体 | camera sensor / image PDU / PNG 出力 |
| `CaptureAsRGBA(x, y)` | 指定ピクセル 1 点の RGBA 値 | color sensor 的な一点サンプル |
| `CaptureRegionAverageRGBA(x, y, width, height)` | 指定矩形領域の平均 RGBA 値 | color sensor 的な領域平均 |

## 5. C++ でどこを見ればよいか

example の主要部分は `examples/sensors/color_camera/color-camera-example.cpp` です。

JSON を読むところ:

```cpp
CameraProfileConfig profile {};
LoadCameraProfileConfigFromJson(config_path, profile);
```

MJCF binding から camera name を取り出すところ:

```cpp
const std::string camera_name = profile.mjcf_binding.camera_name.empty()
    ? kCameraName
    : profile.mjcf_binding.camera_name;
```

MuJoCo model を load するところ:

```cpp
auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
world->loadModel(model_path);
```

`CameraSensor` を作るところ:

```cpp
auto camera_sensor = std::make_unique<CameraSensor>(
    sensor_renderer,
    camera_name);
camera_sensor->LoadConfig(profile.spec);
```

撮影するところ:

```cpp
ImageFrame frame {};
camera_sensor->Capture(frame);
WriteImageFrameToPng(frame, output_path);
```

既存 API で中央ピクセル 1 点を RGBA として取り出すところ:

```cpp
RGBAColor color = camera_sensor->CaptureAsRGBA(frame.width / 2, frame.height / 2);
```

中央付近の矩形領域を平均 RGBA として取り出すところ:

```cpp
RGBAColor average = camera_sensor->CaptureRegionAverageRGBA(
    frame.width / 2 - 8,
    frame.height / 2 - 8,
    16,
    16);
```

この流れが、他のロボットサンプルへカメラを組み込むときの基本形です。

## 6. PDU へ接続する場合

この example は PNG 出力までで、Hakoniwa endpoint へ `sensor_msgs/Image` を publish するところまでは行いません。
PDU に接続する場合は、追加で次を行います。

1. PDU type list に `pdu_config.pdu_name` の channel を追加する。
2. C++ 側で `ImagePduAdapter` を作る。
3. `CameraSensor::Capture()` で得た `ImageFrame` を adapter で送る。
4. Python 側または別 asset 側で同じ PDU definition を使って読む。

関連する設計は [`sensor-actuator-design.md`](../spec/sensor-actuator-design.md) と [`json-config-ja.md`](../guide/json-config-ja.md) を参照してください。

## よくある失敗

- `spec.frame_id` と `mjcf_binding.camera_name` を混同している。
- `mjcf_binding.camera_name` が MJCF の `<camera name="...">` と一致していない。
- `image.format` が example の期待値 `R8G8B8` と違う。
- `clip.near` が 0 以下、または `clip.far` が近すぎる。
- camera の向きが想定と違う。MJCF の `<camera>` の `xyaxes` や body の姿勢を確認します。
- `pdu_config.pdu_name` を書いたが、PDU type list に channel を追加していない。

## 次に読むもの

- [`examples/sensors/color_camera/README.md`](../../examples/sensors/color_camera/README.md)
- [`json-config-ja.md`](../guide/json-config-ja.md)
- [`mjcf-json-authoring-ja.md`](../guide/mjcf-json-authoring-ja.md)
- [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md)
