# Color Camera Sensor Example

This example opens a MuJoCo viewer and captures a simple color-camera shot when you press `s`.

The goal is to make the Color Sensor API easy to understand from the example code:

- the model contains red, green, and blue panels
- the config separates camera `spec`, `mjcf_binding`, and `pdu_config`
- `CameraSensor` is created in `color-camera-example.cpp`
- `CameraSensor::LoadConfig()` applies the JSON config
- `CameraSensor::Capture()` captures the camera named by `mjcf_binding.camera_name`
- `CameraSensor::CaptureAsRGBA()` reads a normalized RGBA sample from the camera
- `WriteImageFrameToPng()` writes the captured RGB frame to `./camera_color_sample.png`

The viewer, keyboard input, and pixel-print helpers are intentionally kept as support code so the main file shows the sensor usage directly.

## Files

```text
examples/sensors/color_camera/
  README.md
  color-camera-example.cpp
  color-camera-hakoniwa-asset.cpp
  read_camera.py
  support/color_camera_example_support.hpp
  support/color_camera_example_support.cpp

models/sensors/color_camera/
  color-camera-sample.xml

config/sensors/color_camera/
  simple-color-camera.json

config/
  camera-pdudef-compact.json
  camera-pdutypes.json
  endpoint/camera_endpoint.json
  endpoint/comm/shm_camera_comm.json
```

Read these first:

- [`color-camera-example.cpp`](./color-camera-example.cpp): the Color Sensor API usage
- [`color-camera-hakoniwa-asset.cpp`](./color-camera-hakoniwa-asset.cpp): publishes captured images as Hakoniwa PDU
- [`read_camera.py`](./read_camera.py): Python Hakoniwa asset that receives and displays the image PDU
- [`simple-color-camera.json`](../../../config/sensors/color_camera/simple-color-camera.json): the camera config
- [`color-camera-sample.xml`](../../../models/sensors/color_camera/color-camera-sample.xml): the MuJoCo model and camera name
- [`docs/tutorial/camera-sensor-ja.md`](../../../docs/tutorial/camera-sensor-ja.md): step-by-step camera setup tutorial in Japanese
- [`docs/tutorial/camera-sensor-hakoniwa-ja.md`](../../../docs/tutorial/camera-sensor-hakoniwa-ja.md): Hakoniwa PDU asset tutorial in Japanese

## Color Sensor API

The example uses the RGB color camera through `CameraSensor`.

```cpp
CameraProfileConfig profile {};
LoadCameraProfileConfigFromJson(config_path, profile);

auto renderer = std::make_shared<MujocoCameraRenderer>(world, false);
auto camera_sensor = std::make_unique<CameraSensor>(
    renderer,
    profile.mjcf_binding.camera_name);
camera_sensor->LoadConfig(profile.spec);

ImageFrame frame {};
camera_sensor->Capture(frame);
WriteImageFrameToPng(frame, output_path);

RGBAColor center = camera_sensor->CaptureAsRGBA(frame.width / 2, frame.height / 2);
RGBAColor average = camera_sensor->CaptureRegionAverageRGBA(
    frame.width / 2 - 8,
    frame.height / 2 - 8,
    16,
    16);
```

The full version is in [`color-camera-example.cpp`](./color-camera-example.cpp). The surrounding code only prepares the MuJoCo model, OpenGL context, viewer, and keyboard controls.

## Model

The model contains three colored panels:

```text
left   : red
center : green
right  : blue
```

The camera is named:

```text
color_camera
```

In the viewer, the camera body is the small black box. The black cylinder is the lens, and the yellow capsule shows the camera's fixed forward direction. The captured image uses the same direction as the yellow marker.

## Sensor Config

The example uses this camera profile:

```text
config/sensors/color_camera/simple-color-camera.json
```

Important fields:

```json
{
  "spec": {
    "frame_id": "color_camera_frame",
    "update_rate": 10,
    "horizontal_fov": 1.2,
    "image": {
      "width": 256,
      "height": 128,
      "format": "R8G8B8"
    }
  },
  "mjcf_binding": {
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

`spec` is the camera sensor specification. `mjcf_binding` names the MJCF camera
and optional movable body/freejoint used by this example. `pdu_config` records
the intended PDU output settings when this camera is connected to a Hakoniwa
endpoint. Noise is disabled so the output is easy to inspect.

`Capture()` returns the full RGB image. `CaptureAsRGBA(x, y)` is a convenience
API for reading one pixel as a normalized color value, which is useful when you
want a color-sensor-like sample from a camera image.
`CaptureRegionAverageRGBA(x, y, width, height)` averages a rectangular image
region and is useful when a single pixel is too noisy.

## Build

From the repository root:

```bash
./build.bash
```

Or, if CMake has already been configured:

```bash
cmake --build src/cmake-build --target color-camera-example
```

To build the Hakoniwa PDU publisher example only:

```bash
cmake --build src/cmake-build --target color-camera-hakoniwa-asset
```

## Run

From the repository root:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

Default output:

```text
./camera_color_sample.png
```

Press `s` in either the MuJoCo viewer window or the terminal to write the PNG.
Press `q` or `Esc` to quit.

Movement keys work in either the MuJoCo viewer window or the terminal:

```text
i : move camera forward  (+X)
k : move camera backward (-X)
j : move camera left     (+Y)
l : move camera right    (-Y)
```

You can also pass paths explicitly:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example \
  models/sensors/color_camera/color-camera-sample.xml \
  config/sensors/color_camera/simple-color-camera.json \
  ./camera_my_color_sample.png
```

## Example Output

```text
Hakoniwa Color Camera Example
model : models/sensors/color_camera/color-camera-sample.xml
config: config/sensors/color_camera/simple-color-camera.json
camera: color_camera
output: ./camera_color_sample.png

Controls:
  i      : move camera forward  (+X)
  k      : move camera backward (-X)
  j      : move camera left     (+Y)
  l      : move camera right    (-Y)
  s      : capture color_camera and write PNG
  h      : show help
  q / Esc: quit

Captured color_camera 256x128
left    pixel=( 42,  64) rgb=(...)
center  pixel=(128,  64) rgb=(...)
right   pixel=(213,  64) rgb=(...)
center_rgba pixel=(128, 64) rgba=(..., ..., ..., 1.000)
region_average_rgba rect=(120, 56, 16, 16) rgba=(..., ..., ..., 1.000)

Wrote PNG: ./camera_color_sample.png
```

Open the PNG and you should see the red, green, and blue panels.

## Hakoniwa PDU Publisher / Reader

The Hakoniwa version publishes the same captured image as `sensor_msgs/Image`.

Terminal 1:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-hakoniwa-asset
```

This process opens a MuJoCo viewer, starts the local Hakoniwa conductor,
registers `CameraAsset`, opens/starts the endpoint, and then waits for the
simulation start trigger.

Terminal 2:

```bash
python3 examples/sensors/color_camera/read_camera.py
```

Terminal 3:

```bash
hako-cmd start
```

After the start trigger, the C++ asset enters its manual timing loop and
publishes `CameraAsset/camera_image` periodically.

While the publisher terminal or MuJoCo viewer is active, move the camera with:

```text
i : move camera forward  (+X)
k : move camera backward (-X)
j : move camera left     (+Y)
l : move camera right    (-Y)
h : show controls
q / Esc : quit publisher
```

The MuJoCo viewer shows the model and movable camera body. The Python OpenCV
window shows the image published from that camera over Hakoniwa PDU.

The default PDU key is a PDU robot name plus a channel name:

```text
robot/channel: CameraAsset/camera_image
type         : sensor_msgs/Image
size         : 98616 bytes
```

`read_camera.py` registers itself as a Hakoniwa controller asset named
`CameraReader`. It receives `CameraAsset/camera_image`, where `CameraAsset` is
the PDU robot name from `camera-pdudef-compact.json` and `camera_image` is the
channel name from `camera-pdutypes.json`. It converts `rgb8` to BGR and displays
the image with OpenCV.

If the Python window only shows a waiting screen, no image PDU has arrived yet.
Check that `color-camera-hakoniwa-asset` is running, the PDU robot name is
`CameraAsset`, the PDU channel name is `camera_image`, and `hako-cmd start` has
been executed.
The reader skips invalid initial PDU bytes until the C++ publisher writes the
first complete `sensor_msgs/Image` frame.

On platforms where OpenCV GUI work must run on the main thread, keep this
threading model: the Python main thread owns `cv2.imshow()` / `cv2.waitKey()`,
and `hakopy.start()` runs on a worker thread.

## Notes

- This is an RGB camera example, not a full camera pipeline demo.
- PNG output uses the shared `WriteImageFrameToPng()` helper and does not add an external dependency.
- The example needs a MuJoCo / OpenGL render context.
- The Hakoniwa publisher uses a hidden GLFW/OpenGL context for camera capture.
