# カメラセンサ箱庭アセット化チュートリアル

このチュートリアルでは、MuJoCo上のカメラ（カラー）センサから取得した画像データを箱庭の通信エンドポイント（PDU）に流し、箱庭アセットとして動作させるための設定手順とワークフローを解説します。

前段階の MuJoCo へのカメラの配置と設定については、[カメラセンサー設定チュートリアル](camera-sensor-ja.md) を参照してください。

---

## 1. 全体ワークフロー

カメラセンサを箱庭アセット化するために必要な設定ファイルと、その関係性は以下の通りです。

```text
  [ C++ Asset (プログラム) ]
             |
             v (Endpoint名指定でロード)
    [ camera_endpoint.json ]
             |
    +--------+--------+------------------+
    | (参照)           | (参照)            | (参照)
    v                 v                  v
[ camera-pdudef-compact.json ] [ cache/buffer.json ] [ comm/shm_camera_comm.json ]
    |
    v (参照)
[ camera-pdutypes.json ]
```

### 必要な作業手順
1. **`camera-pdutypes.json` の設定**: PDUチャネルのデータ型やバッファサイズを定義する。
2. **`pdudef.json` (`*-compact.json`) の設定**: ロボット（アセット）名と PDU型定義を紐付ける。
3. **Endpoint設定 (`endpoint_config.json`) の定義**: エンドポイント名に PDU定義・Cache・Comm の設定を紐付ける。
4. **Cache設定 (`cache/buffer.json`) の定義**: バッファの動作モード（最新データのみを保持するなど）を指定する。
5. **Comm設定 (`comm/shm_camera_comm.json`) の定義**: 共有メモリなどの通信プロトコルと、アセットごとの通信チャネルを指定する。
6. **C++ アプリケーションコードの実装**: アダプタを利用した送信処理。

---

## 2. 各設定ファイルの書き方

ここでは、アセット名 `"CameraAsset"`、カメラ画像のチャネル名 `"camera_image"` を新しく定義するケースを例にして説明します。

### ステップ 1: PDU 型定義の編集 (`camera-pdutypes.json`)
箱庭アセットで送受信する PDU チャネルの一覧を配列形式で定義します。
カメラ画像（RGB）を送る場合は、ROS互換の `sensor_msgs/Image` を使用します。

```json
[
  {
    "channel_id": 0,
    "pdu_size": 98616,
    "name": "camera_image",
    "type": "sensor_msgs/Image"
  }
]
```

- **`channel_id`**: 0始まりのユニークなチャネルID。
- **`pdu_size`**: 画像サイズ（解像度とチャンネル数）に応じたバッファサイズ（バイト）。（計算方法の詳細は後述）
- **`name`**: C++ アプリケーションコード側で `PduKey` に指定するチャネル名。
- **`type`**: ROS 互換の型名（`sensor_msgs/Image`）。

> [!IMPORTANT]
> **PDUサイズ（`pdu_size`）の計算方法と注意点**
> 
> カメラ画像（`sensor_msgs/Image`）のように可変長配列を含むメッセージの `pdu_size` は、以下のように計算します。
> 
> **総PDUサイズ ＝ PDU metadata（24バイト） ＋ 固定部サイズ（288バイト） ＋ 可変長データ（ピクセルデータ）領域**
> 
> * **固定部サイズ**: 可変長配列のデータ部分を除いたメッセージ構造自体の基本サイズ。
>   * 型ごとの基本サイズは `thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/pdu_size/` 以下にテキストとして定義されています。
>   * `sensor_msgs/Image` の基本サイズは **`288` バイト** です（`sensor_msgs/CompressedImage` は **`272` バイト**）。
>   * endpoint の channel buffer には、これに加えて PDU metadata **`24` バイト** も含めます。
> * **可変長データ領域**: 実際に送信するピクセルデータの最大容量。
>   * **このチュートリアルの計算例**: 256x128 解像度の RGB24 (1ピクセル3バイト) 画像を送信する場合：
>     * ピクセルデータサイズ = `256 * 128 * 3 = 98,304` バイト
>     * 総PDUサイズ = `24 (metadata) + 288 (固定部) + 98,304 = 98,616` バイト
> * **注意**: 
>   * 画像の解像度やカラーフォーマットを変更した場合は可変長部のサイズが変化するため、**実際に撮影して得られるサイズ感に合わせて `pdu_size` を調整する**必要があります。
>   * サイズが不足していると、`[ConvertorError][Image] buffer too small` となり、データが全く書き込まれなくなります（送信されなくなります）。

### ステップ 2: PDU 定義インスタンスの編集 (`*-compact.json`)
ロボット（アセット）名と上記で作成した型定義ファイルをマッピングします。

`config/camera-pdudef-compact.json`:
```json
{
  "paths": [
    {
      "id": "camera-image",
      "path": "camera-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "CameraAsset",
      "pdutypes_id": "camera-image"
    }
  ]
}
```
- **`robots[].name`**: 箱庭内で識別するアセット名です。

### ステップ 3: エンドポイント設定 (`endpoint_config.json`)
アセット起動時に読み込むメインのエンドポイントファイルを作成します。

`config/endpoint/camera_endpoint.json`:
```json
{
  "name": "camera_endpoint",
  "pdu_def_path": "../camera-pdudef-compact.json",
  "cache": "cache/buffer.json",
  "comm": "comm/shm_camera_comm.json"
}
```

### ステップ 4: キャッシュ設定の編集 (`cache/buffer.json`)
画像のように、常に最新の1フレームを処理したいデータには `latest` モードを指定します。

`config/endpoint/cache/buffer.json`:
```json
{
  "type": "buffer",
  "name": "default_latest_buffer",
  "store": {
    "mode": "latest"
  }
}
```

### ステップ 5: 通信設定の編集 (`comm/shm_camera_comm.json`)
使用するプロトコル（通常は共有メモリ `shm`）と、アセットが通信するチャネルのリストを指定します。

`config/endpoint/comm/shm_camera_comm.json`:
```json
{
  "protocol": "shm",
  "impl_type": "callback",
  "name": "camera_shm",
  "direction": "inout",
  "io": {
    "robots": [
      {
        "name": "CameraAsset",
        "pdu": [
          { "name": "camera_image", "notify_on_recv": false }
        ]
      }
    ]
  }
}
```
- **`notify_on_recv`**: 画像データなどの受信時に同期トリガー（シミュレータ側の一時停止制御など）をかける必要がない場合は `false` にします。

---

## 3. C++ 側でのエンドポイント連携

設定ファイルの準備が整ったら、C++のメインプログラムからこれらのエンドポイントを制御します。

### ライフサイクルと呼び出し順
この example は MuJoCo viewer を表示しながら、viewer 用の OpenGL context でカメラ画像を capture します。
そのため、`CameraSensor::Capture()` と PDU 送信は viewer の pre-render callback 側で行い、
Hakoniwa の manual timing callback はアセットのライフサイクル維持に使います。

前述の通り、`hako_asset_start_no_wait()` は名前に `no_wait` とありますが、箱庭の start trigger は待ちます。
ここでのポイントは、`hako_asset_start()` と違って `IsForceStop` のような停止判定 callback を渡せることです。
viewer を main thread で動かす場合は、この待機・実行処理を worker thread に逃がし、viewer close や `q` 入力で停止できるようにします。
endpoint の呼び出し順は次を守ります。

1. `main()` で `hako_asset_register()` を呼び、PDU channel を作成する。
2. `main()` で `endpoint.open()` を呼ぶ。
3. `main()` で `endpoint.start()` を呼ぶ。
4. worker thread で `hako_asset_start_no_wait()` を呼び、simulation start trigger を待つ。
5. `on_initialize` callback で `endpoint.post_start()` を呼ぶ。
6. `post_start()` 完了後、viewer pre-render callback で `world->advanceTimeStep()` を行う。
7. `CameraSensor::ShouldUpdate()` が `true` の周期で `CameraSensor::Capture()`、`ImagePduAdapter::send()` を行う。
8. viewer 終了後に `endpoint.stop()` / `endpoint.close()` を呼ぶ。

`endpoint.open()` / `endpoint.start()` を `on_manual_timing_control` の中で呼ばないでください。
manual timing callback は start trigger 後に呼ばれるため、endpoint の初期化場所としては遅すぎます。
また、`endpoint.post_start()` は `main()` ではなく `on_initialize` callback で呼びます。

### CameraSensor と viewer renderer

この example では、`color-camera-example.cpp` と同じく `MujocoRenderRuntime::CreateCameraRenderer(world)` を使います。
`CameraSensor` はこの renderer を受け取り、MuJoCo XML の `camera` 名と JSON の `spec` から画像を取得します。
OpenGL context を持つ viewer thread で capture するため、capture 処理は pre-render callback 側に置きます。
pre-render callback は `mjv_updateScene()` の前に呼ばれるため、MuJoCo viewer に描かれる姿勢と capture に使う姿勢が一致します。
測定・publish の周期は `CameraSensor::ShouldUpdate()` に任せ、`spec.update_rate` に従わせます。

```cpp
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/image.hpp"

std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter> image_adapter;
std::unique_ptr<hako::robots::sensor::camera::CameraSensor> camera_sensor;
std::atomic_bool endpoint_ready {false};

// --- アセット初期化コールバック ---
static int my_on_initialize(hako_asset_context_t* context)
{
    (void)context;
    
    // 【重要】 post_start() は初期化タイミングでコールして有効化する
    if (endpoint->post_start() != HAKO_PDU_ERR_OK) {
        std::cerr << "Failed to complete endpoint post_start." << std::endl;
        return -1;
    }
    endpoint_ready.store(true);
    return 0;
}

static int my_on_reset(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

// --- シミュレーション・送信ループコールバック ---
static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;
    const double sim_timestep = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);

    while (running_flag) {
        // viewer 側の pre-render callback が capture/send を行う。
        // manual timing callback は箱庭アセットの実行状態を維持する。
        hako_asset_usleep(delta_time_usec);
    }
    return 0;
}

// --- メイン処理の抜粋 ---
int main()
{
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    world->loadModel("models/sensors/color_camera/color-camera-sample.xml");

    CameraProfileConfig profile {};
    LoadCameraProfileConfigFromJson(
        "config/sensors/color_camera/simple-color-camera.json",
        profile);

    // コールバック設定とアセット登録
    hako_asset_callbacks_t my_callback {};
    my_callback.on_initialize = my_on_initialize;
    my_callback.on_reset = my_on_reset;
    my_callback.on_manual_timing_control = my_manual_timing_control;

    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(world->getModel()->opt.timestep * 1.0e6);

    hako_asset_register("CameraAsset", "config/camera-pdudef-compact.json", &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);

    // hako_asset_register() 後、hako_asset_start() 前に open と start
    endpoint = std::make_unique<hakoniwa::pdu::Endpoint>(
        "camera_endpoint",
        HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    endpoint->open("config/endpoint/camera_endpoint.json");
    endpoint->start();

    const hakoniwa::pdu::PduKey image_key{"CameraAsset", "camera_image"};
    image_adapter = std::make_unique<ImagePduAdapter>(*endpoint, image_key);

    MujocoRenderRuntime render_runtime(
        world->getModel(),
        world->getData(),
        viewer_running,
        mujoco_mutex,
        MujocoRenderWindowMode::Visible);

    auto renderer = render_runtime.CreateCameraRenderer(world);
    camera_sensor = std::make_unique<CameraSensor>(
        renderer,
        profile.mjcf_binding.camera_name);
    camera_sensor->LoadConfig(profile.spec);

    const double sim_timestep = world->getModel()->opt.timestep;
    ImageFrame image_frame {};
    render_runtime.SetPreRenderCallback([&]() {
        if (!endpoint_ready.load()) {
            return;
        }

        // カメラ body を動かす場合は、pose 更新も pre-render 側で行う。
        world->advanceTimeStep();
        if (!camera_sensor->ShouldUpdate(sim_timestep)) {
            return;
        }

        camera_sensor->Capture(image_frame);
        if (!image_frame.data.empty()) {
            (void)image_adapter->send(image_frame);
        }
    });

    // アセット開始。ここで start trigger を待つ。
    // 別 terminal から hako-cmd start を実行すると manual timing callback に入る。
    std::thread asset_thread([&]() {
        hako_asset_start_no_wait(IsForceStop);
    });

    render_runtime.Run();

    running_flag = false;
    asset_thread.join();
    endpoint->stop();
    endpoint->close();
}
```

---

## 4. Python による画像受信アセットの作成例

シミュレータ（C++）が書き込んだカメラ画像データを、Python 側の別アセットで読み取ります。
ここでは、PDU の読み書きに `hakoniwa_pdu_endpoint`、箱庭アセットとしての登録と実行に `hakopy` を使います。

単に `Endpoint.recv_by_name()` を呼ぶだけでは、Python プロセスは箱庭アセットとして登録されません。
箱庭のライフサイクルに参加する場合は、Python 側でも次の順序にします。

1. `Endpoint.open()` / `Endpoint.start()` で PDU endpoint を準備する。
2. `hakopy.asset_register()` で Python reader を箱庭アセットとして登録する。
3. `on_initialize` callback で `endpoint.post_start()` を呼ぶ。
4. `on_manual_timing_control` callback の中で PDU を受信する。
5. `hakopy.start()` で箱庭アセットの実行を開始する。

この例では、Python 側のアセット名を `"CameraReader"`、読む対象の PDU を
`PduKey("CameraAsset", "camera_image")` とします。
`"CameraReader"` は Python reader 自身の箱庭アセット名で、`"CameraAsset"` は画像を publish する C++ 側アセット名です。

### 受信スクリプトの作成例 (`examples/sensors/color_camera/read_camera.py`)

実装例は [`examples/sensors/color_camera/read_camera.py`](../../examples/sensors/color_camera/read_camera.py) にあります。
OpenCV の GUI は main thread で動かす必要がある環境があるため、この example では
`cv2.imshow()` / `cv2.waitKey()` を main thread に置き、`hakopy.start()` を worker thread で起動します。
以下は構造を示す抜粋です。`image_to_bgr()` や `put_latest()` などの補助関数は実ファイルを参照してください。
`pdu_to_py_Image()` が返す `Image.data` は環境によって `bytes` ではなく tuple/list になるため、
実装では `np.frombuffer()` 固定ではなく `np.asarray(..., dtype=np.uint8)` でも受けられるようにします。

```python
import queue
import threading

import cv2
import hakopy

from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Image import pdu_to_py_Image

ENDPOINT_CONFIG_PATH = "config/endpoint/camera_endpoint.json"
PDU_DEF_PATH = "config/camera-pdudef-compact.json"

READER_ASSET_NAME = "CameraReader"
PRODUCER_ASSET_NAME = "CameraAsset"
IMAGE_PDU_NAME = "camera_image"
STEP_USEC = 30_000

endpoint = Endpoint("camera_reader", "inout")
image_key = PduKey(PRODUCER_ASSET_NAME, IMAGE_PDU_NAME)
frame_queue = queue.Queue(maxsize=1)
shutdown = threading.Event()
callback_state = {"result": 0}
skipped_invalid = {"count": 0}


def on_initialize(_context):
    # endpoint.post_start() は箱庭の initialize callback で呼ぶ。
    try:
        endpoint.post_start()
    except Exception:
        print("[ERROR] endpoint.post_start() failed")
        return 1
    return 0


def on_reset(_context):
    return 0


def on_manual_timing_control(_context):
    pdu_size = endpoint.get_pdu_size(image_key)

    try:
        while not shutdown.is_set():
            raw_bytes = endpoint.recv_by_name(image_key, pdu_size)

            if raw_bytes:
                try:
                    image_msg = pdu_to_py_Image(raw_bytes)
                except Exception as e:
                    # publisher が最初の frame を書き込む前は、
                    # shared memory 上の初期値を読んで decode に失敗することがある。
                    skipped_invalid["count"] += 1
                    if skipped_invalid["count"] == 1:
                        print(f"[INFO] Skipping invalid initial image PDU: {e}")
                    if hakopy.usleep(STEP_USEC) is False:
                        break
                    continue

                bgr_img = image_to_bgr(image_msg)
                if bgr_img is not None:
                    if skipped_invalid["count"] > 0:
                        print(
                            "[INFO] First valid image PDU received after "
                            f"{skipped_invalid['count']} skipped reads"
                        )
                        skipped_invalid["count"] = 0
                    put_latest(frame_queue, bgr_img)

            if hakopy.usleep(STEP_USEC) is False:
                break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] camera reader loop failed: {e}")
        callback_state["result"] = 1
    finally:
        shutdown.set()
    return 0


CALLBACK = {
    "on_initialize": on_initialize,
    "on_simulation_step": None,
    "on_manual_timing_control": on_manual_timing_control,
    "on_reset": on_reset,
}


def main():
    endpoint.open(ENDPOINT_CONFIG_PATH)
    endpoint.start()

    model = hakopy.HAKO_ASSET_MODEL_CONTROLLER
    ret = hakopy.asset_register(
        READER_ASSET_NAME,
        PDU_DEF_PATH,
        CALLBACK,
        STEP_USEC,
        model,
    )
    if ret is False:
        print("[ERROR] hakopy.asset_register() failed")
        endpoint.stop()
        endpoint.close()
        return 1

    worker = threading.Thread(target=hakopy.start, daemon=True)
    worker.start()

    # main thread 側で cv2.imshow() / cv2.waitKey() を実行する。
    while not shutdown.is_set():
        try:
            bgr_img = frame_queue.get(timeout=0.03)
            cv2.imshow("Hakoniwa Camera Data", bgr_img)
        except queue.Empty:
            pass
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            shutdown.set()
            break

    cv2.destroyAllWindows()

    return int(callback_state["result"])

if __name__ == "__main__":
    raise SystemExit(main())
```

> [!NOTE]
> `asset_register()` に渡す `READER_ASSET_NAME` は Python reader 自身のアセット名です。
> 一方、`PduKey(PRODUCER_ASSET_NAME, IMAGE_PDU_NAME)` の `PRODUCER_ASSET_NAME` は、画像 PDU を publish している C++ 側アセット名です。
> 受信対象を指定するときは、reader の名前ではなく publish 元の robot / asset 名を指定します。

> [!IMPORTANT]
> `recv_by_name()` が bytes を返しても、それが有効な `sensor_msgs/Image` とは限りません。
> publisher が最初の frame を書き込む前に reader が shared memory を読むと、`MetaData not found or corrupted` のような decode error になることがあります。
> これは異常終了にせず、最初の正常な frame が来るまで skip します。

---

## 5. 動作検証方法

箱庭アセット化の整合性を確認するためには、以下の点を確認します。
1. **JSON Validation**: `tools/validate_assets.py` を使って設定ファイルの構文エラーが無いか検証する。
2. **C++ アセット起動**: コンダクターを起動した状態でシミュレータプロセスを立ち上げ、`hako_asset_register()` および `endpoint.post_start()` がエラー無く通過することを確認する。
3. **Python アセット登録**: `read_camera.py` を実行し、`hakopy.asset_register()` と `hakopy.start()` がエラー無く通過することを確認する。
4. **Start trigger**: 別 terminal から `hako-cmd start` を実行する。
5. **Python アセットからの購読**: GUI ウィンドウ上にシミュレータのカメラ映像がリアルタイムで描画されることを確認する。

### 実行例

まず C++ publisher をビルドします。Python reader はスクリプトなのでビルドは不要です。

```bash
cmake --build src/cmake-build --target color-camera-hakoniwa-asset -j4
```

Terminal 1 で C++ publisher を起動します。
このプロセスは MuJoCo viewer を開き、viewer の pre-render callback でカメラ画像を capture して PDU に送信します。

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-hakoniwa-asset
```

Terminal 2 で Python reader を起動します。
OpenCV の window 操作は main thread で行い、`hakopy.start()` は worker thread で動かします。

```bash
python3 examples/sensors/color_camera/read_camera.py
```

Terminal 3 で箱庭の start trigger を送ります。

```bash
hako-cmd start
```

`hako-cmd start` 後、C++ publisher 側では `on_initialize` が呼ばれ、`endpoint.post_start()` が完了します。
その後、MuJoCo viewer の描画 loop 内で `world->advanceTimeStep()` が実行され、`CameraSensor::ShouldUpdate(sim_timestep)` が `true` になった周期で `CameraSensor::Capture()`、`ImagePduAdapter::send()` が実行されます。
Python reader 側の OpenCV window にカメラ画像が表示されれば成功です。

### つまづきポイント

- **`CreateCameraRenderer()` を使う**:
  viewer 付き example では、`MujocoRenderRuntime::CreateCameraRenderer(world)` で renderer を作り、その renderer を `CameraSensor` に渡します。
  これにより `color-camera-example.cpp` と同じ OpenGL / viewer context 上で capture できます。

- **`endpoint.open()` / `endpoint.start()` は `main()` で呼ぶ**:
  `on_manual_timing_control` の中で呼ぶと、箱庭 lifecycle と endpoint lifecycle の順序が崩れます。
  `post_start()` だけを `on_initialize` callback で呼びます。

- **capture/send は viewer 側で行う**:
  カメラ画像の capture は OpenGL context に依存します。
  この example では `hako_asset_start_no_wait()` を worker thread で動かし、main thread の MuJoCo viewer pre-render callback で capture/send します。
  `mjv_updateScene()` 後の overlay callback で camera pose を更新すると、viewer 表示と capture 画像の位置がずれるため、pose 更新と capture は pre-render 側で行います。

- **送信周期は `CameraSensor::ShouldUpdate(sim_timestep)` に任せる**:
  `sim_timestep` は MuJoCo model の `model->opt.timestep` です。
  `ShouldUpdate(sim_timestep)` は内部のセンサ用クロックを進め、`spec.update_rate` の周期に達したときだけ `true` を返します。
  PDU 用に別の手動周期カウンタを持つと、JSON の sensor spec と実際の publish 周期がずれやすくなります。

- **Python の OpenCV は main thread に置く**:
  `cv2.imshow()` / `cv2.waitKey()` は環境によって main thread でないと表示されません。
  そのため `read_camera.py` は `hakopy.start()` を worker thread に逃がし、main thread で OpenCV window を更新します。

- **最初の PDU はまだ有効でないことがある**:
  reader が publisher の初回書き込み前に shared memory を読むと、`MetaData not found or corrupted` のような decode error になります。
  これは起動順の race として起きるため、最初の有効 frame が来るまで skip します。

- **`Image.data` は bytes とは限らない**:
  Python の `pdu_to_py_Image()` が返す `Image.data` は、環境によって `tuple` / `list` のことがあります。
  `bytes` 前提の `np.frombuffer()` だけでなく、`np.asarray(image_msg.data, dtype=np.uint8)` でも受けるようにします。

- **`pdu_size` は endpoint buffer 用の metadata も含める**:
  256x128 RGB24 の場合、ピクセルデータは `98,304` bytes ですが、`sensor_msgs/Image` 固定部 `288` bytes と PDU metadata `24` bytes も必要です。
  `config/camera-pdutypes.json` の `pdu_size` は `98,616` にします。
  不足すると `[ConvertorError][Image] buffer too small` で送信に失敗します。

- **アセット登録に失敗した場合は箱庭状態を確認する**:
  `ERROR: Can not register asset` や `hakopy.asset_register() failed` が出る場合、同名 asset が既に登録済み、または前回実行の shared memory / master 状態が残っている可能性があります。
  同時起動している同名プロセスがないか確認し、必要に応じて箱庭環境を停止・初期化してから再実行します。
