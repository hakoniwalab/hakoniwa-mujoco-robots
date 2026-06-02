# Color Camera Sensor Example

This example opens a MuJoCo viewer and captures a simple color-camera shot when you press `s`.

The goal is to make the Color Sensor API easy to understand from the example code:

- the model contains red, green, and blue panels
- the config defines the RGB image size, format, and field of view
- `CameraSensor` is created in `color-camera-example.cpp`
- `CameraSensor::LoadConfig()` applies the JSON config
- `CameraSensor::Capture()` captures the camera named `color_camera`
- `WriteImageFrameToPng()` writes the captured RGB frame to `./camera_color_sample.png`

The viewer, keyboard input, and pixel-print helpers are intentionally kept as support code so the main file shows the sensor usage directly.

## Files

```text
examples/sensors/color_camera/
  README.md
  color-camera-example.cpp
  support/color_camera_example_support.hpp
  support/color_camera_example_support.cpp

models/sensors/color_camera/
  color-camera-sample.xml

config/sensors/color_camera/
  simple-color-camera.json
```

Read these first:

- [`color-camera-example.cpp`](./color-camera-example.cpp): the Color Sensor API usage
- [`simple-color-camera.json`](../../../config/sensors/color_camera/simple-color-camera.json): the camera config
- [`color-camera-sample.xml`](../../../models/sensors/color_camera/color-camera-sample.xml): the MuJoCo model and camera name

## Color Sensor API

The example uses the RGB color camera through `CameraSensor`.

```cpp
CameraConfig config {};
LoadCameraConfigFromJson(config_path, config);

auto renderer = std::make_shared<MujocoCameraRenderer>(world, false);
auto camera_sensor = std::make_unique<CameraSensor>(renderer, "color_camera");
camera_sensor->LoadConfig(config);

ImageFrame frame {};
camera_sensor->Capture(frame);
WriteImageFrameToPng(frame, output_path);
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
  "frame_id": "color_camera_frame",
  "update_rate": 10,
  "horizontal_fov": 1.2,
  "image": {
    "width": 256,
    "height": 128,
    "format": "R8G8B8"
  }
}
```

Noise is disabled so the output is easy to inspect.

## Build

From the repository root:

```bash
./build.bash
```

Or, if CMake has already been configured:

```bash
cmake --build src/cmake-build --target color-camera-example
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

Wrote PNG: ./camera_color_sample.png
```

Open the PNG and you should see the red, green, and blue panels.

## Notes

- This is an RGB camera example, not a full camera pipeline demo.
- PNG output uses the shared `WriteImageFrameToPng()` helper and does not add an external dependency.
- The example needs a MuJoCo / OpenGL render context.
