#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr const char* kDefaultModelPath =
    "models/sensors/color_camera/color-camera-sample.xml";
constexpr const char* kDefaultConfigPath =
    "config/sensors/color_camera/simple-color-camera.json";
constexpr const char* kDefaultOutputPath =
    "./camera_color_sample.png";
constexpr const char* kCameraName = "color_camera";
constexpr const char* kSensorJointName = "color_sensor_freejoint";
constexpr double kMoveStep = 0.05;

struct Rgb
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

struct AppState
{
    std::atomic_bool running {true};
    std::atomic_bool pending_shot {false};
    std::atomic_bool print_help {false};
    std::atomic_int move_forward {0};
    std::atomic_int move_left {0};
};

class CameraFovyOverrideGuard
{
public:
    CameraFovyOverrideGuard(mjModel* model, int cam_id, mjtNum new_fovy)
        : model_(model), cam_id_(cam_id), original_fovy_(0), active_(false)
    {
        if (model_ == nullptr || cam_id_ < 0) {
            return;
        }
        original_fovy_ = model_->cam_fovy[cam_id_];
        model_->cam_fovy[cam_id_] = new_fovy;
        active_ = true;
    }

    ~CameraFovyOverrideGuard()
    {
        if (active_) {
            model_->cam_fovy[cam_id_] = original_fovy_;
        }
    }

    CameraFovyOverrideGuard(const CameraFovyOverrideGuard&) = delete;
    CameraFovyOverrideGuard& operator=(const CameraFovyOverrideGuard&) = delete;

private:
    mjModel* model_;
    int cam_id_;
    mjtNum original_fovy_;
    bool active_;
};

uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t size)
{
    static uint32_t table[256] {};
    static bool initialized = false;
    if (!initialized) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1U) ? (0xedb88320U ^ (c >> 1U)) : (c >> 1U);
            }
            table[i] = c;
        }
        initialized = true;
    }

    crc = crc ^ 0xffffffffU;
    for (size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ data[i]) & 0xffU] ^ (crc >> 8U);
    }
    return crc ^ 0xffffffffU;
}

uint32_t adler32(const std::vector<uint8_t>& data)
{
    constexpr uint32_t kMod = 65521U;
    uint32_t a = 1U;
    uint32_t b = 0U;
    for (uint8_t byte : data) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16U) | a;
}

void append_be32(std::vector<uint8_t>& out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
    out.push_back(static_cast<uint8_t>(value & 0xffU));
}

void append_chunk(
    std::vector<uint8_t>& png,
    const char type[4],
    const std::vector<uint8_t>& payload)
{
    append_be32(png, static_cast<uint32_t>(payload.size()));
    const size_t type_offset = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), payload.begin(), payload.end());

    const uint32_t crc = crc32_update(
        0U,
        png.data() + type_offset,
        4 + payload.size());
    append_be32(png, crc);
}

std::vector<uint8_t> zlib_store(const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> out;
    out.push_back(0x78);
    out.push_back(0x01);

    size_t offset = 0;
    while (offset < data.size()) {
        const size_t remaining = data.size() - offset;
        const uint16_t block_size = static_cast<uint16_t>(
            remaining > 65535U ? 65535U : remaining);
        const bool final_block = (offset + block_size) == data.size();

        out.push_back(final_block ? 0x01 : 0x00);
        out.push_back(static_cast<uint8_t>(block_size & 0xffU));
        out.push_back(static_cast<uint8_t>((block_size >> 8U) & 0xffU));
        const uint16_t nlen = static_cast<uint16_t>(~block_size);
        out.push_back(static_cast<uint8_t>(nlen & 0xffU));
        out.push_back(static_cast<uint8_t>((nlen >> 8U) & 0xffU));
        out.insert(out.end(), data.begin() + static_cast<long>(offset),
                   data.begin() + static_cast<long>(offset + block_size));
        offset += block_size;
    }

    append_be32(out, adler32(data));
    return out;
}

bool write_png_rgb8(
    const std::filesystem::path& path,
    int width,
    int height,
    const std::vector<uint8_t>& rgb)
{
    if (width <= 0 || height <= 0 ||
        rgb.size() != static_cast<size_t>(width * height * 3)) {
        return false;
    }

    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>((width * 3 + 1) * height));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0);
        const size_t row_offset = static_cast<size_t>(y * width * 3);
        raw.insert(
            raw.end(),
            rgb.begin() + static_cast<long>(row_offset),
            rgb.begin() + static_cast<long>(row_offset + width * 3));
    }

    std::vector<uint8_t> png {
        0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'
    };

    std::vector<uint8_t> ihdr;
    append_be32(ihdr, static_cast<uint32_t>(width));
    append_be32(ihdr, static_cast<uint32_t>(height));
    ihdr.push_back(8);
    ihdr.push_back(2);
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(0);
    append_chunk(png, "IHDR", ihdr);

    append_chunk(png, "IDAT", zlib_store(raw));
    append_chunk(png, "IEND", {});

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) {
        return false;
    }
    ofs.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    return ofs.good();
}

Rgb sample_pixel(const std::vector<uint8_t>& rgb, int width, int x, int y)
{
    const size_t index = static_cast<size_t>((y * width + x) * 3);
    return Rgb {rgb.at(index + 0), rgb.at(index + 1), rgb.at(index + 2)};
}

void print_sample(
    const std::string& label,
    const std::vector<uint8_t>& rgb,
    int width,
    int x,
    int y)
{
    const Rgb sample = sample_pixel(rgb, width, x, y);
    std::cout
        << std::left << std::setw(8) << label
        << " pixel=(" << std::right << std::setw(3) << x << ", "
        << std::setw(3) << y << ")"
        << " rgb=("
        << std::setw(3) << static_cast<int>(sample.r) << ", "
        << std::setw(3) << static_cast<int>(sample.g) << ", "
        << std::setw(3) << static_cast<int>(sample.b) << ")"
        << std::endl;
}

void print_help()
{
    std::cout << R"(
Controls:
  i      : move camera forward  (+X)
  k      : move camera backward (-X)
  j      : move camera left     (+Y)
  l      : move camera right    (-Y)
  s      : capture color_camera and write PNG
  h      : show help
  q / Esc: quit

Viewer:
  Use the mouse to rotate / zoom the MuJoCo viewer.
  Press 's' in either the viewer window or this terminal to save a sensor shot.
)" << std::endl;
}

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [model.xml] [camera-config.json] [output.png]\n\n"
        << "Default:\n"
        << "  model : " << kDefaultModelPath << "\n"
        << "  config: " << kDefaultConfigPath << "\n"
        << "  output: " << kDefaultOutputPath << "\n";
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;
    if (action != GLFW_PRESS) {
        return;
    }

    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state == nullptr) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state->running.store(false);
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_S) {
        state->pending_shot.store(true);
    } else if (key == GLFW_KEY_H) {
        state->print_help.store(true);
    } else if (key == GLFW_KEY_I) {
        state->move_forward.fetch_add(1);
    } else if (key == GLFW_KEY_K) {
        state->move_forward.fetch_sub(1);
    } else if (key == GLFW_KEY_J) {
        state->move_left.fetch_add(1);
    } else if (key == GLFW_KEY_L) {
        state->move_left.fetch_sub(1);
    }
}

void terminal_command_loop(AppState& state)
{
    while (state.running.load()) {
        char key = '\0';
        std::cin >> key;
        if (!std::cin) {
            return;
        }

        if (key == 'q') {
            state.running.store(false);
            return;
        }
        if (key == 's') {
            state.pending_shot.store(true);
            continue;
        }
        if (key == 'h') {
            state.print_help.store(true);
            continue;
        }
        if (key == 'i') {
            state.move_forward.fetch_add(1);
            continue;
        }
        if (key == 'k') {
            state.move_forward.fetch_sub(1);
            continue;
        }
        if (key == 'j') {
            state.move_left.fetch_add(1);
            continue;
        }
        if (key == 'l') {
            state.move_left.fetch_sub(1);
            continue;
        }

        std::cout << "unknown command: " << key << std::endl;
        state.print_help.store(true);
    }
}

int find_freejoint_qpos_addr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        throw std::runtime_error(std::string("joint was not found: ") + joint_name);
    }
    if (model->jnt_type[joint_id] != mjJNT_FREE) {
        throw std::runtime_error(std::string("joint is not a freejoint: ") + joint_name);
    }
    return model->jnt_qposadr[joint_id];
}

void print_camera_position(const mjData* data, int qpos_addr)
{
    std::cout
        << "camera_pos=("
        << std::fixed << std::setprecision(3)
        << data->qpos[qpos_addr + 0] << ", "
        << data->qpos[qpos_addr + 1] << ", "
        << data->qpos[qpos_addr + 2] << ")"
        << std::defaultfloat
        << std::endl;
}

void move_camera(mjModel* model, mjData* data, int qpos_addr, int forward_steps, int left_steps)
{
    if (forward_steps == 0 && left_steps == 0) {
        return;
    }

    data->qpos[qpos_addr + 0] += static_cast<double>(forward_steps) * kMoveStep;
    data->qpos[qpos_addr + 1] += static_cast<double>(left_steps) * kMoveStep;
    data->qpos[qpos_addr + 3] = 1.0;
    data->qpos[qpos_addr + 4] = 0.0;
    data->qpos[qpos_addr + 5] = 0.0;
    data->qpos[qpos_addr + 6] = 0.0;
    mj_forward(model, data);

    std::cout << "moved camera: "
              << "x " << (forward_steps >= 0 ? "+= " : "-= ")
              << std::abs(forward_steps) * kMoveStep
              << ", y " << (left_steps >= 0 ? "+= " : "-= ")
              << std::abs(left_steps) * kMoveStep
              << ", ";
    print_camera_position(data, qpos_addr);
}

bool capture_camera_rgb(
    mjModel* model,
    mjData* data,
    mjrContext& context,
    const hako::robots::sensor::camera::CameraConfig& config,
    std::vector<uint8_t>& out_rgb)
{
    const int cam_id = mj_name2id(model, mjOBJ_CAMERA, kCameraName);
    if (cam_id < 0) {
        std::cerr << "Camera not found: " << kCameraName << std::endl;
        return false;
    }

    mjr_setBuffer(mjFB_OFFSCREEN, &context);
    if (context.currentBuffer != mjFB_OFFSCREEN) {
        std::cerr << "Offscreen rendering is not available." << std::endl;
        return false;
    }

    if (config.image.width > context.offWidth ||
        config.image.height > context.offHeight) {
        std::cerr << "Requested image size " << config.image.width << "x"
                  << config.image.height << " exceeds MuJoCo offscreen buffer "
                  << context.offWidth << "x" << context.offHeight << std::endl;
        return false;
    }

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjv_makeScene(model, &scene, 2000);

    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = cam_id;

    const double vfov_rad = 2.0 * std::atan(
        std::tan(config.horizontal_fov / 2.0) *
        (config.image.height / static_cast<double>(config.image.width)));
    CameraFovyOverrideGuard fovy_guard(
        model,
        cam_id,
        static_cast<mjtNum>(vfov_rad * 180.0 / M_PI));

    mjrRect viewport = {0, 0, config.image.width, config.image.height};
    mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);

    std::vector<uint8_t> raw(static_cast<size_t>(config.image.width * config.image.height * 3));
    mjr_readPixels(raw.data(), nullptr, viewport, &context);

    out_rgb.resize(raw.size());
    const int row_size = config.image.width * 3;
    for (int y = 0; y < config.image.height; ++y) {
        std::copy_n(
            raw.data() + (config.image.height - 1 - y) * row_size,
            row_size,
            out_rgb.data() + y * row_size);
    }

    mjv_freeScene(&scene);
    return true;
}

bool save_sensor_shot(
    mjModel* model,
    mjData* data,
    mjrContext& context,
    const hako::robots::sensor::camera::CameraConfig& config,
    const std::filesystem::path& output_path)
{
    std::vector<uint8_t> rgb;
    if (!capture_camera_rgb(model, data, context, config, rgb)) {
        return false;
    }

    const int y = config.image.height / 2;
    std::cout << "\nCaptured " << kCameraName << " "
              << config.image.width << "x" << config.image.height << std::endl;
    print_sample("left", rgb, config.image.width, config.image.width / 6, y);
    print_sample("center", rgb, config.image.width, config.image.width / 2, y);
    print_sample("right", rgb, config.image.width, (config.image.width * 5) / 6, y);

    if (!write_png_rgb8(output_path, config.image.width, config.image.height, rgb)) {
        std::cerr << "Failed to write PNG: " << output_path << std::endl;
        return false;
    }

    std::cout << "Wrote PNG: " << output_path << "\n" << std::endl;
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }

    const std::string model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    const std::string config_path = argc > 2 ? argv[2] : kDefaultConfigPath;
    const std::filesystem::path output_path = argc > 3 ? argv[3] : kDefaultOutputPath;

    hako::robots::sensor::camera::CameraConfig config {};
    if (!hako::robots::sensor::camera::LoadCameraConfigFromJson(config_path, config)) {
        std::cerr << "Failed to load camera config: " << config_path << std::endl;
        return 1;
    }
    if (config.image.format != "R8G8B8") {
        std::cerr << "This example expects R8G8B8 format, got: "
                  << config.image.format << std::endl;
        return 1;
    }

    char load_error[1024] {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, load_error, sizeof(load_error));
    if (model == nullptr) {
        std::cerr << "Failed to load MuJoCo model: " << model_path
                  << "\n" << load_error << std::endl;
        return 1;
    }
    mjData* data = mj_makeData(model);
    if (data == nullptr) {
        mj_deleteModel(model);
        std::cerr << "Failed to allocate MuJoCo data." << std::endl;
        return 1;
    }
    mj_forward(model, data);
    int camera_qpos_addr = -1;
    try {
        camera_qpos_addr = find_freejoint_qpos_addr(model, kSensorJointName);
    } catch (const std::exception& e) {
        mj_deleteData(data);
        mj_deleteModel(model);
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    if (!glfwInit()) {
        mj_deleteData(data);
        mj_deleteModel(model);
        std::cerr << "GLFW initialization failed." << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(900, 650, "Hakoniwa Color Camera Example", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        std::cerr << "GLFW window creation failed." << std::endl;
        return 1;
    }

    AppState state {};
    glfwSetWindowUserPointer(window, &state);
    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera viewer_camera;
    mjvOption viewer_option;
    mjvScene viewer_scene;
    mjrContext context;
    mjv_defaultCamera(&viewer_camera);
    mjv_defaultOption(&viewer_option);
    mjv_defaultScene(&viewer_scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &viewer_scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    std::cout << "Hakoniwa Color Camera Example" << std::endl;
    std::cout << "model : " << model_path << std::endl;
    std::cout << "config: " << config_path << std::endl;
    std::cout << "output: " << output_path << std::endl;
    print_camera_position(data, camera_qpos_addr);
    print_help();

    std::thread terminal_thread(terminal_command_loop, std::ref(state));

    while (state.running.load() && !glfwWindowShouldClose(window)) {
        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        mjr_setBuffer(mjFB_WINDOW, &context);
        mjv_updateScene(
            model,
            data,
            &viewer_option,
            nullptr,
            &viewer_camera,
            mjCAT_ALL,
            &viewer_scene);
        mjr_render(viewport, &viewer_scene, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();

        const int forward_steps = state.move_forward.exchange(0);
        const int left_steps = state.move_left.exchange(0);
        move_camera(model, data, camera_qpos_addr, forward_steps, left_steps);

        if (state.pending_shot.exchange(false)) {
            save_sensor_shot(model, data, context, config, output_path);
        }
        if (state.print_help.exchange(false)) {
            print_help();
        }
    }

    state.running.store(false);
    if (terminal_thread.joinable()) {
        terminal_thread.detach();
    }

    mjr_freeContext(&context);
    mjv_freeScene(&viewer_scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    std::cout << "bye" << std::endl;
    return 0;
}
