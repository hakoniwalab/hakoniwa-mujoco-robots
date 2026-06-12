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
    [ endpoint_config.json ] 
             |
    +--------+--------+------------------+
    | (参照)           | (参照)            | (参照)
    v                 v                  v
[ pdudef-compact.json ] [ cache/buffer.json ] [ comm/shm_sim_comm.json ]
    |
    v (参照)
[ pdutypes.json ]
```

### 必要な作業手順
1. **`pdutypes.json` の設定**: PDUチャネルのデータ型やバッファサイズを定義する。
2. **`pdudef.json` (`*-compact.json`) の設定**: ロボット（アセット）名と PDU型定義を紐付ける。
3. **Endpoint設定 (`endpoint_config.json`) の定義**: エンドポイント名に PDU定義・Cache・Comm の設定を紐付ける。
4. **Cache設定 (`cache/buffer.json`) の定義**: バッファの動作モード（最新データのみを保持するなど）を指定する。
5. **Comm設定 (`comm/shm_sim_comm.json`) の定義**: 共有メモリなどの通信プロトコルと、アセットごとの通信チャネルを指定する。
6. **C++ アプリケーションコードの実装**: アダプタを利用した送信処理。

---

## 2. 各設定ファイルの書き方

ここでは、アセット名 `"CameraAsset"`、カメラ画像のチャネル名 `"camera_image"` を新しく定義するケースを例にして説明します。

### ステップ 1: PDU 型定義の編集 (`pdutypes.json`)
箱庭アセットで送受信する PDU チャネルの一覧を配列形式で定義します。
カメラ画像（RGB）を送る場合は、ROS互換の `sensor_msgs/Image` を使用します。

```json
[
  {
    "channel_id": 0,
    "pdu_size": 1555488,
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
> **総PDUサイズ ＝ 固定部サイズ（288バイト） ＋ 可変長データ（ピクセルデータ）領域**
> 
> * **固定部サイズ**: 可変長配列のデータ部分を除いたメッセージ構造自体の基本サイズ。
>   * 型ごとの基本サイズは `thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/pdu_size/` 以下にテキストとして定義されています。
>   * `sensor_msgs/Image` の基本サイズは **`288` バイト** です（`sensor_msgs/CompressedImage` は **`272` バイト**）。
> * **可変長データ領域**: 実際に送信するピクセルデータの最大容量。
>   * **計算例**: 960x540 解像度の RGB24 (1ピクセル3バイト) 画像を送信する場合：
>     * ピクセルデータサイズ = `960 * 540 * 3 = 1,555,200` バイト
>     * 総PDUサイズ = `288 (固定部) + 1,555,200 = 1,555,488` バイト
> * **注意**: 
>   * 画像の解像度やカラーフォーマットを変更した場合は可変長部のサイズが変化するため、**実際に撮影して得られるサイズ感に合わせて `pdu_size` を調整する**必要があります。
>   * サイズが不足していると、データ送信時にエラーとなり、データが全く書き込まれなくなります（送信されなくなります）。

### ステップ 2: PDU 定義インスタンスの編集 (`*-compact.json`)
ロボット（アセット）名と上記で作成した型定義ファイルをマッピングします。

`config/camera-pdudef-compact.json`:
```json
{
  "paths": [
    {
      "id": "camera-endpoint",
      "path": "camera-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "CameraAsset",
      "pdutypes_id": "camera-endpoint"
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
前述の通り、`hako_asset_start()` は復帰しないため、スレッド分離したライフサイクル管理が基本となります。

```cpp
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/image.hpp"

// エンドポイントインスタンスの定義
hakoniwa::pdu::Endpoint endpoint("camera_endpoint", HAKO_PDU_ENDPOINT_DIRECTION_INOUT);

// --- アセット初期化コールバック ---
static int my_on_initialize(hako_asset_context_t* context)
{
    (void)context;
    
    // 【重要】 post_start() は初期化タイミングでコールして有効化する
    if (endpoint.post_start() != HAKO_PDU_ERR_OK) {
        std::cerr << "Failed to complete endpoint post_start." << std::endl;
        return -1;
    }
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
    const double sim_timestep = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);

    // Image アダプタを作成 (アセット名: CameraAsset, PDUチャネル名: camera_image)
    const hakoniwa::pdu::PduKey image_key{"CameraAsset", "camera_image"};
    hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter image_adapter(endpoint, image_key);

    hako::robots::sensor::camera::ImageFrame image_frame {};

    while (running_flag) {
        // 1. シミュレーションを進める
        world->step();

        // 2. カメラセンサから画像（RGB24等）を取得
        camera_sensor->Capture(image_frame);

        // 3. アダプタ経由で箱庭エンドポイントへ画像データを送信
        (void)image_adapter.send(image_frame);

        // 4. 時間ステップ進行のスリープ
        hako_asset_usleep(delta_time_usec);
    }
    return 0;
}

// --- メイン処理 ---
int main()
{
    // アセット開始前に open と start
    endpoint.open("config/endpoint/camera_endpoint.json");
    endpoint.start();

    // コールバック設定とアセット登録
    hako_asset_callbacks_t my_callback {};
    my_callback.on_initialize = my_on_initialize;
    my_callback.on_reset = my_on_reset;
    my_callback.on_manual_timing_control = my_manual_timing_control;

    hako_asset_register("CameraAsset", "config/camera-pdudef-compact.json", &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);

    // アセット開始 (ブロックされる)
    hako_asset_start();

    endpoint.stop();
    endpoint.close();
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

### 受信スクリプトの作成例 (`read_camera.py`)

```python
import numpy as np
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
callback_state = {"result": 0}


def on_initialize(_context):
    # endpoint.post_start() は箱庭の initialize callback で呼ぶ。
    if endpoint.post_start() is False:
        print("[ERROR] endpoint.post_start() failed")
        return 1
    return 0


def on_reset(_context):
    return 0


def on_manual_timing_control(_context):
    pdu_size = endpoint.get_pdu_size(image_key)
    print("Camera reader asset started. Press Ctrl+C or 'q' in viewer to exit.")

    try:
        while True:
            raw_bytes = endpoint.recv_by_name(image_key, pdu_size)

            if raw_bytes:
                image_msg = pdu_to_py_Image(raw_bytes)
                img_data = np.frombuffer(image_msg.data, dtype=np.uint8)

                # この例は RGB24 前提。JSON の spec.image.format と合わせる。
                expected_data_len = image_msg.height * image_msg.width * 3
                if len(img_data) == expected_data_len:
                    img = img_data.reshape((image_msg.height, image_msg.width, 3))
                    bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Hakoniwa Camera Data", bgr_img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

            if hakopy.usleep(STEP_USEC) is False:
                break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] camera reader loop failed: {e}")
        callback_state["result"] = 1
    finally:
        cv2.destroyAllWindows()
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

    try:
        start_ret = hakopy.start()
        print(f"[INFO] hakopy.start() returns {start_ret}")
    finally:
        endpoint.stop()
        endpoint.close()

    return int(callback_state["result"])

if __name__ == "__main__":
    raise SystemExit(main())
```

> [!NOTE]
> `asset_register()` に渡す `READER_ASSET_NAME` は Python reader 自身のアセット名です。
> 一方、`PduKey(PRODUCER_ASSET_NAME, IMAGE_PDU_NAME)` の `PRODUCER_ASSET_NAME` は、画像 PDU を publish している C++ 側アセット名です。
> 受信対象を指定するときは、reader の名前ではなく publish 元の robot / asset 名を指定します。

---

## 5. 動作検証方法

箱庭アセット化の整合性を確認するためには、以下の点を確認します。
1. **JSON Validation**: `tools/validate_assets.py` を使って設定ファイルの構文エラーが無いか検証する。
2. **C++ アセット起動**: コンダクターを起動した状態でシミュレータプロセスを立ち上げ、`hako_asset_register()` および `endpoint.post_start()` がエラー無く通過することを確認する。
3. **Python アセット登録**: `read_camera.py` を実行し、`hakopy.asset_register()` と `hakopy.start()` がエラー無く通過することを確認する。
4. **Python アセットからの購読**: GUI ウィンドウ上にシミュレータのカメラ映像がリアルタイムで描画されることを確認する。
