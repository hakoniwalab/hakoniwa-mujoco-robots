# Color Camera Sensor Example

This example opens a MuJoCo viewer and captures a simple color-camera shot when you press `s`.

The goal is to make the sensor behavior visible at a glance:

- the model contains red, green, and blue panels
- the viewer shows the small color-panel scene
- pressing `i` / `k` / `j` / `l` moves the camera body
- pressing `s` captures the camera named `color_camera`
- the example prints RGB values from left / center / right pixels
- the captured image is written to `./camera_color_sample.png`

## Files

```text
examples/sensors/color_camera/
  README.md
  color-camera-example.cpp

models/sensors/color_camera/
  color-camera-sample.xml

config/sensors/color_camera/
  simple-color-camera.json
```

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
- The PNG writer is intentionally local to the example and does not add an external dependency.
- The example needs a MuJoCo / OpenGL render context.
