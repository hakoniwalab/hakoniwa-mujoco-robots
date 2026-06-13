# センサ / アクチュエータ利用ガイド

このガイドは、Hakoniwa MuJoCo のセンサとアクチュエータを使うための実用的な入口です。
何を実行するか、どの設定ファイルを編集するか、MuJoCo と Hakoniwa PDU の間でデータがどう流れるかを中心に説明します。

MJCF XML と JSON を最初から書く場合は、先に
[`mjcf-json-authoring-ja.md`](mjcf-json-authoring-ja.md) を読んでください。
JSON の共通構造と共通フィールドは
[`json-config-ja.md`](json-config-ja.md) にまとめています。

内部設計の境界は [`sensor-actuator-design.md`](../spec/sensor-actuator-design.md)、
JSON フィールドの詳細は [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md) を参照してください。

## まず見る場所

```text
MuJoCo モデル        models/
センサ設定           config/sensors/
アクチュエータ設定   config/actuator/
C++ コンポーネント    src/sensors/, src/actuator/
PDU 変換             include/hakoniwa/pdu/converter/
PDU endpoint I/O     include/hakoniwa/pdu/adapter/
小さいサンプル        examples/sensors/, examples/actuators/
```

コンポーネント単体を理解するときは `examples/` から見てください。
ロボット全体の制御ループを見たい場合は TurtleBot3 と forklift のサンプルが入口です。

## データの流れ

センサ出力は通常この向きに流れます。

```text
MuJoCo world -> sensor component -> frame struct -> PDU converter -> PDU adapter -> Hakoniwa endpoint
```

アクチュエータ入力は通常この向きに流れます。

```text
Hakoniwa endpoint -> PDU adapter -> command/target struct -> actuator component -> MuJoCo ctrl[]
```

センサやアクチュエータ本体は、Hakoniwa endpoint I/O なしでも使える状態に保ちます。
Endpoint への read/write は adapter 層に置くのがこのリポジトリの基本方針です。

## 小さいサンプルを動かす

まず全体をビルドします。

```bash
./build.bash
```

超音波距離センサのサンプル:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

RGB カメラの PNG 出力サンプル:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

joint actuator のサンプル:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

これらは viewer で確認する対話的なサンプルです。
README に明記されていない限り、headless な自動確認用とは考えないでください。

## センサコンポーネント

| コンポーネント | 主な設定 | 主な出力 | 入口 |
| --- | --- | --- | --- |
| 2D LiDAR | `config/sensors/lidar/*.json` | `sensor_msgs/LaserScan` | TB3 demo |
| IMU | `config/sensors/imu/tb3-imu.json` | `sensor_msgs/Imu` | TB3 demo |
| Joint state | `config/sensors/joint_state/tb3-wheel-joint-states.json` | `sensor_msgs/JointState` | TB3 demo |
| Odometry | `config/sensors/odometry/tb3-ground-truth-odom.json` | `nav_msgs/Odometry` | TB3 demo |
| TF | `config/sensors/tf/tb3-basic-tf.json` | `tf2_msgs/TFMessage` | TB3 demo |
| Ultrasonic | `config/sensors/ultrasonic/lego-spike-distance-sensor.json` | `sensor_msgs/Range` | `examples/sensors/ultrasonic/` |
| RGB camera | `config/sensors/color_camera/simple-color-camera.json` | `sensor_msgs/Image` | `examples/sensors/color_camera/` |
| Depth / RGBD camera | `config/sensors/camera/*.json` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | sensor tests |

センサ設定を作るときは、近いサンプル JSON をコピーしてから、対応する schema を `config/sensors/schema/` で確認するのが現実的です。

## アクチュエータコンポーネント

現在の再利用可能なアクチュエータ経路は MuJoCo joint actuator です。

```text
src/actuator/joint_actuator_impl.hpp
config/actuator/joint/*.json
examples/actuators/joint/
```

`JointActuatorImpl` は JSON から MuJoCo actuator を解決し、目標値を `mjData::ctrl[]` に書き込みます。
実際の制御特性は MJCF 側の actuator 種別で決まります。

| JSON `spec.type` | 対応する MJCF actuator | `ctrl[]` の意味 |
| --- | --- | --- |
| `position` | `<position>` | 目標 joint 位置 |
| `velocity` | `<velocity>` | 目標 joint 速度 |
| `torque` | `<motor>` | effort / torque command |

JSON の `spec.type` と MJCF actuator 種別は一致している必要があります。
たとえば `velocity` の設定を MuJoCo の `<position>` actuator に向けることはできません。

## ロボットサンプルにセンサを追加する

1. センサが読む MuJoCo body、site、camera、joint、frame をモデルに追加または再利用します。
2. `config/sensors/<sensor_type>/` に JSON profile を追加します。
3. robot/application class で profile を読み込みます。
4. MuJoCo state から sensor frame を作ります。
5. 対応する PDU adapter で変換・publish します。
6. Python や別の Hakoniwa asset から読む必要がある場合は、robot PDU definition に出力 PDU を追加します。

統合例としては TB3 が一番わかりやすいです。

```text
src/main_for_sample/tb3/main.cpp
src/robots/tb3/tb3_robot.cpp
config/tb3-pdudef-compact.json
config/tb3-pdutypes.json
```

## joint actuator を追加する

1. MuJoCo model に joint と対応する MJCF actuator を定義します。
2. `config/actuator/joint/` に JSON binding を追加します。
3. `world->createJointActuator()` で actuator を作ります。
4. モデル読み込み後に `LoadConfig()` を一度呼びます。
5. 制御周期ごとに `SetTarget()` を呼びます。
6. 目標値が PDU から来る場合は、actuator 内で endpoint を読むのではなく、対応する adapter で受信します。

最小の C++ 使用例:

```cpp
auto actuator = world->createJointActuator();
actuator->LoadConfig("config/actuator/joint/sample_velocity_actuator.json");
actuator->SetTarget(target_velocity);
```

## よく見る確認ポイント

- build や runtime の問題を調べる前に `./doctor.bash` を実行します。
- Python 側で decode に失敗する場合は、Python と C++ が同じ PDU definition JSON を使っているか確認します。
- センサが publish しない場合は、update rate と MJCF の source body/site/camera 名を確認します。
- joint actuator が動かない場合は、`mjcf_binding.actuator_name` が joint 名ではなく MuJoCo actuator 名に一致しているか確認します。
  旧 JSON の `RuntimeBinding.actuator_name` も互換性のため読めます。
- actuator type mismatch が出る場合は、JSON `spec.type` と MJCF の `<position>` / `<velocity>` / `<motor>` を一致させます。
- viewer サンプルには GUI/OpenGL context が必要です。headless 確認には CMake configure、unit test、viewer なし target を使います。

## 次に読むもの

- [`mjcf-json-authoring-ja.md`](mjcf-json-authoring-ja.md)
- [`json-config-ja.md`](json-config-ja.md)
- [`camera-sensor-ja.md`](../tutorial/camera-sensor-ja.md)
- [`examples/sensors/README.md`](../../examples/sensors/README.md)
- [`examples/actuators/README.md`](../../examples/actuators/README.md)
- [`examples/sensors/ultrasonic/README.md`](../../examples/sensors/ultrasonic/README.md)
- [`examples/sensors/color_camera/README.md`](../../examples/sensors/color_camera/README.md)
- [`examples/actuators/joint/README.md`](../../examples/actuators/joint/README.md)
- [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md)
- [`sensor-actuator-design.md`](../spec/sensor-actuator-design.md)
