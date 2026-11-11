// Render server without OpenGL: a fake drawer stands in for MuJoCo's renderer.
#include "physics/render_server.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace sa = hakoniwa::sensor_adapter;
using hako::robots::physics::impl::IFrameDrawer;
using hako::robots::physics::impl::RenderServer;
using hako::robots::physics::impl::RenderServerClient;
using namespace std::chrono_literals;

void Expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// Writes the request's camera name length into every byte, and records the
// order in which requests were drawn (to check that batches are not interleaved).
class FakeDrawer final : public IFrameDrawer
{
public:
    FakeDrawer(std::vector<std::string>& log, std::mutex& log_mutex, std::chrono::milliseconds delay)
        : log_(log)
        , log_mutex_(log_mutex)
        , delay_(delay)
    {
    }

    bool draw(const sa::RenderRequest* requests, std::size_t count, sa::RenderStatus* statuses) override
    {
        for (std::size_t i = 0; i < count; ++i) {
            std::this_thread::sleep_for(delay_);
            const auto& r = requests[i];
            {
                std::lock_guard<std::mutex> lock(log_mutex_);
                log_.push_back(r.camera_name);
            }
            const std::size_t bytes = static_cast<std::size_t>(r.camera.width) * r.camera.height * 3;
            statuses[i] = {};
            if (r.rgb.capacity < bytes) {
                statuses[i].error = r.camera_name + ": RGB destination too small";
                continue;
            }
            std::memset(r.rgb.data, static_cast<int>(r.camera_name.size()), bytes);
            statuses[i].ok = true;
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (!statuses[i].ok) {
                return false;
            }
        }
        return true;
    }

private:
    std::vector<std::string>& log_;
    std::mutex& log_mutex_;
    std::chrono::milliseconds delay_;
};

sa::RenderRequest Request(const std::string& name, std::vector<std::uint8_t>& buffer)
{
    sa::RenderRequest r {};
    r.camera_name = name;
    r.camera.width = 4;
    r.camera.height = 2;
    r.outputs = sa::kRenderRgb;
    buffer.assign(4 * 2 * 3, 0);
    r.rgb = {buffer.data(), buffer.size()};
    return r;
}

// Many threads submit two-camera batches at once; everyone gets their own images
// and the two cameras of a batch are drawn back to back.
void ConcurrentRequesters()
{
    RenderServer server;
    std::vector<std::string> log;
    std::mutex log_mutex;
    server.set_drawer_factory([&](const mjModel*, mjData*, std::string&) {
        return std::make_unique<FakeDrawer>(log, log_mutex, 1ms);
    });
    std::atomic_bool running {true};
    std::thread server_thread([&]() { server.run(running, 5ms); });

    constexpr int kThreads = 6;
    std::vector<std::thread> requesters;
    std::atomic_int good {0};
    for (int t = 0; t < kThreads; ++t) {
        requesters.emplace_back([&, t]() {
            const std::string left = "t" + std::to_string(t) + std::string(static_cast<std::size_t>(t), 'x') + "L";
            const std::string right = "t" + std::to_string(t) + std::string(static_cast<std::size_t>(t), 'x') + "R";
            std::vector<std::uint8_t> a;
            std::vector<std::uint8_t> b;
            sa::RenderRequest batch[2] = {Request(left, a), Request(right, b)};
            sa::RenderStatus statuses[2] {};
            RenderServerClient client(nullptr, nullptr, server, 5s);
            const bool ok = client.render(batch, 2, statuses);
            if (ok && statuses[0].ok && statuses[1].ok && a[0] == left.size() && b[23] == right.size()) {
                ++good;
            }
        });
    }
    for (auto& r : requesters) {
        r.join();
    }
    running = false;
    server_thread.join();
    Expect(good == kThreads, "every requester got its own images");
    Expect(log.size() == 2 * kThreads, "every request was drawn once");
    for (std::size_t i = 0; i < log.size(); i += 2) {
        Expect(log[i].back() == 'L' && log[i + 1].back() == 'R'
                   && log[i].substr(0, log[i].size() - 1) == log[i + 1].substr(0, log[i + 1].size() - 1),
               "the two cameras of a batch are drawn back to back");
    }
}

// Nobody serves: the request fails after the wait limit and the buffer is untouched.
void NotRunningTimesOut()
{
    RenderServer server;
    std::vector<std::uint8_t> buffer;
    sa::RenderRequest r = Request("cam", buffer);
    buffer[0] = 77;
    sa::RenderStatus status {};
    const auto start = std::chrono::steady_clock::now();
    const bool ok = server.submit(nullptr, nullptr, &r, 1, &status, 50ms);
    const auto waited = std::chrono::steady_clock::now() - start;
    Expect(!ok && !status.ok, "a request nobody serves fails");
    Expect(status.error.find("not running on the main thread") != std::string::npos,
           "the error says the server is not running");
    Expect(waited >= 50ms, "it waited for the limit");
    Expect(buffer[0] == 77, "the buffer is untouched");
    Expect(server.process_pending() == 0, "the withdrawn job is gone from the queue");
}

// A job the server has started is waited for even past the limit (the buffers are the caller's).
void DrawingIsWaitedFor()
{
    RenderServer server;
    std::vector<std::string> log;
    std::mutex log_mutex;
    server.set_drawer_factory([&](const mjModel*, mjData*, std::string&) {
        return std::make_unique<FakeDrawer>(log, log_mutex, 200ms);
    });
    std::atomic_bool running {true};
    std::thread server_thread([&]() { server.run(running, 1ms); });
    std::vector<std::uint8_t> buffer;
    sa::RenderRequest r = Request("slow", buffer);
    sa::RenderStatus status {};
    const bool ok = server.submit(nullptr, nullptr, &r, 1, &status, 50ms);
    running = false;
    server_thread.join();
    Expect(ok && status.ok && buffer[0] == 4, "a started job completes even past the wait limit");
}

// Shutdown fails queued jobs and later submissions.
void ShutdownFailsPending()
{
    RenderServer server;
    std::vector<std::uint8_t> buffer;
    sa::RenderRequest r = Request("cam", buffer);
    sa::RenderStatus status {};
    std::thread requester([&]() { server.submit(nullptr, nullptr, &r, 1, &status, 10s); });
    std::this_thread::sleep_for(50ms);
    server.shutdown();
    requester.join();
    Expect(!status.ok && status.error.find("stopped") != std::string::npos, "a queued job fails on shutdown");
    sa::RenderStatus later {};
    Expect(!server.submit(nullptr, nullptr, &r, 1, &later, 10s) && !later.ok,
           "a submission after shutdown fails at once");
}

// A drawer that cannot be made fails the job with its reason.
void DrawerFactoryError()
{
    RenderServer server;
    server.set_drawer_factory([](const mjModel*, mjData*, std::string& error) {
        error = "no display";
        return std::unique_ptr<IFrameDrawer> {};
    });
    std::atomic_bool running {true};
    std::thread server_thread([&]() { server.run(running, 1ms); });
    std::vector<std::uint8_t> buffer;
    sa::RenderRequest r = Request("cam", buffer);
    sa::RenderStatus status {};
    const bool ok = server.submit(nullptr, nullptr, &r, 1, &status, 5s);
    running = false;
    server_thread.join();
    Expect(!ok && status.error.find("no display") != std::string::npos, "the factory's error reaches the caller");
}

}  // namespace

int main()
{
    try {
        ConcurrentRequesters();
        NotRunningTimesOut();
        DrawingIsWaitedFor();
        ShutdownFailsPending();
        DrawerFactoryError();
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "render_server_test: OK" << std::endl;
    return 0;
}
