#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include <vector>
#include <string>
#include <cstddef> // for size_t
#include <cmath> // for isnan, NAN
#include <limits> // for std::numeric_limits

namespace hako::robots::sensor::camera
{

bool EncodeImage(const RawCameraFrame& raw, const CameraConfig& config, ImageFrame& out)
{
    out.width = raw.width;
    out.height = raw.height;
    out.format = config.image.format;
    out.frame_id = config.frame_id;
    out.timestamp = raw.timestamp;

    if (config.image.format == "R8G8B8") {
        out.channels = 3;
        out.data = raw.rgb;
        return true;
    }
    if (config.image.format == "B8G8R8") {
        out.channels = 3;
        out.data.resize(raw.rgb.size());
        for (size_t i = 0; i < raw.rgb.size(); i += 3) {
            out.data[i + 0] = raw.rgb[i + 2]; // B
            out.data[i + 1] = raw.rgb[i + 1]; // G
            out.data[i + 2] = raw.rgb[i + 0]; // R
        }
        return true;
    }
    if (config.image.format == "L8") {
        out.channels = 1;
        out.data.resize(raw.width * raw.height);
        for (int i = 0; i < raw.width * raw.height; ++i) {
            const uint8_t r = raw.rgb[i * 3 + 0];
            const uint8_t g = raw.rgb[i * 3 + 1];
            const uint8_t b = raw.rgb[i * 3 + 2];
            out.data[i] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
        }
        return true;
    }
    return false; // Unsupported format
}

namespace {
    /*
     * Converts `mjr_readPixels` depth samples to metric distances for the current
     * offscreen MuJoCo render path.
     *
     * Contract:
     * - `buffer` is treated as an OpenGL-style depth buffer in the [0, 1] range.
     * - `znear` / `zfar` are the effective clip-plane distances in meters used by the OpenGL projection.
     *
     * Validation status:
     * - This conversion has been smoke-tested against fixed-camera box scenes at 0.2/0.5/1/2/5/9 m.
     * - Center and several off-center pixels were checked, along with multiple horizontal FOV settings
     *   and clip-range masking behavior.
     *
     * Caveat:
     * - It is still not fully validated for arbitrary scenes such as oblique geometry, extreme camera setups,
     *   or alternate depth-map conventions.
     */
    void LinearizeDepthBuffer(std::vector<float>& buffer, double znear, double zfar) {
        for (size_t i = 0; i < buffer.size(); ++i) {
            const double z_ndc = 2.0 * static_cast<double>(buffer[i]) - 1.0;
            buffer[i] = static_cast<float>((2.0 * znear * zfar) / (zfar + znear - z_ndc * (zfar - znear)));
        }
    }
}

bool EncodeDepth(const RawCameraFrame& raw, const DepthCameraConfig& config, DepthFrame& out)
{
    out.width = raw.width;
    out.height = raw.height;
    out.format = config.image.format;
    out.frame_id = config.frame_id;
    out.timestamp = raw.timestamp;

    if (config.image.format != "DEPTH_F32_M" && config.image.format != "DEPTH_U16_MM") {
        return false;
    }

    out.data = raw.depth_buffer;
    // `mjr_readPixels` depth samples are treated as OpenGL depth-buffer values.
    // Reversed depth maps are not handled here yet.
    if (raw.depth_map != mjDEPTH_ZERONEAR) {
        return false;
    }
    LinearizeDepthBuffer(out.data, raw.znear, raw.zfar);

    // Apply the sensor's specific clipping planes
    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < out.data.size(); ++i) {
        if (out.data[i] < config.clip.near || out.data[i] > config.clip.far) {
            out.data[i] = nan_val;
        }
    }

    // If the requested format is DEPTH_U16_MM, this layer still returns float meters.
    // Conversion to uint16 millimeters belongs to downstream PDU serialization.
    return true;
}

void ClearImageFrame(ImageFrame& out)
{
    out.width = 0;
    out.height = 0;
    out.channels = 0;
    out.format.clear();
    out.frame_id.clear();
    out.data.clear();
    out.timestamp = 0.0;
}

void ClearDepthFrame(DepthFrame& out)
{
    out.width = 0;
    out.height = 0;
    out.format.clear();
    out.frame_id.clear();
    out.data.clear();
    out.timestamp = 0.0;
}

}
