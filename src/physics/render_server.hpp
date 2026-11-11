#pragma once

// Render function server: one thread (the main thread) owns the graphics
// context and draws; any thread asks for images and waits. Rendering is a
// function of the request and the world state the requester froze by waiting.
// Requires the hakoniwa-sensor-adapter headers (header-only, C++17).
//
// Lock order (no other order is allowed when locks are held together):
//   1. the world's mjData mutex (held by a requester for its whole physics step)
//   2. RenderServer::queue_mutex_
//   3. RenderJob::mutex
// 2 and 3 are held only to move a job in or out, never while drawing or waiting
// on something else. The server never takes 1: a requester may hold 1 while it
// waits for its job, and nothing else moves the world meanwhile.

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace hako::robots::physics::impl
{

/// Draws requests into the caller's buffers. Created and used only on the server thread.
class IFrameDrawer
{
public:
    virtual ~IFrameDrawer() = default;
    virtual bool draw(const hakoniwa::sensor_adapter::RenderRequest* requests, std::size_t count,
                      hakoniwa::sensor_adapter::RenderStatus* statuses) = 0;
};

/// Makes the drawer of one world, on the server thread. Returns nullptr (with error) on failure.
using FrameDrawerFactory =
    std::function<std::unique_ptr<IFrameDrawer>(const mjModel* model, mjData* data, std::string& error)>;

/// One submitted batch and its ticket.
struct RenderJob
{
    enum class State { Queued, Drawing, Done };

    const mjModel* model {nullptr};
    mjData* data {nullptr};
    const hakoniwa::sensor_adapter::RenderRequest* requests {nullptr};
    std::size_t count {0};
    hakoniwa::sensor_adapter::RenderStatus* statuses {nullptr};

    std::mutex mutex;
    std::condition_variable done;
    State state {State::Queued};
    bool ok {false};
};

class RenderServer
{
public:
    /// The process-wide server. Creating it does not start anything.
    static RenderServer& instance()
    {
        static RenderServer server;
        return server;
    }

    RenderServer() = default;
    RenderServer(const RenderServer&) = delete;
    RenderServer& operator=(const RenderServer&) = delete;

    /// Server side, before serving: how to make the drawer of a world.
    void set_drawer_factory(FrameDrawerFactory factory)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        factory_ = std::move(factory);
    }

    /// Server side: called after a job is queued, to wake a loop that waits on
    /// something else (for example glfwPostEmptyEvent for a viewer).
    void set_wakeup(std::function<void()> wakeup)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        wakeup_ = std::move(wakeup);
    }

    /**
     * Requester side, any thread: queue one batch and wait until it is drawn.
     * If no server takes the job within the timeout, it is withdrawn and fails.
     * Once the server has started drawing it, the call waits until it is done:
     * the buffers belong to the caller and must not be written after return.
     */
    bool submit(const mjModel* model, mjData* data, const hakoniwa::sensor_adapter::RenderRequest* requests,
                std::size_t count, hakoniwa::sensor_adapter::RenderStatus* statuses,
                std::chrono::milliseconds timeout)
    {
        if (count == 0) {
            return true;
        }
        auto job = std::make_shared<RenderJob>();
        job->model = model;
        job->data = data;
        job->requests = requests;
        job->count = count;
        job->statuses = statuses;

        std::function<void()> wakeup;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (stopped_) {
                fail_all(requests, count, statuses, "the render server has stopped");
                return false;
            }
            queue_.push_back(job);
            wakeup = wakeup_;
        }
        work_.notify_one();
        if (wakeup) {
            wakeup();
        }

        {
            std::unique_lock<std::mutex> lock(job->mutex);
            if (job->done.wait_for(lock, timeout, [&job]() { return job->state == RenderJob::State::Done; })) {
                return job->ok;
            }
        }
        // Timed out: withdraw the job if the server has not taken it yet.
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            for (auto it = queue_.begin(); it != queue_.end(); ++it) {
                if (*it == job) {
                    queue_.erase(it);
                    fail_all(requests, count, statuses,
                             "the render server is not running on the main thread "
                             "(no request was taken within the wait limit)");
                    return false;
                }
            }
        }
        // The server is drawing it into our buffers: wait until it has finished.
        std::unique_lock<std::mutex> lock(job->mutex);
        job->done.wait(lock, [&job]() { return job->state == RenderJob::State::Done; });
        return job->ok;
    }

    /// Server side: draw every queued job. Returns the number of jobs processed.
    std::size_t process_pending()
    {
        std::size_t processed = 0;
        while (true) {
            std::shared_ptr<RenderJob> job;
            FrameDrawerFactory factory;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (queue_.empty()) {
                    break;
                }
                job = queue_.front();
                queue_.pop_front();
                factory = factory_;
            }
            {
                std::lock_guard<std::mutex> lock(job->mutex);
                job->state = RenderJob::State::Drawing;
            }
            const bool ok = draw(*job, factory);
            {
                std::lock_guard<std::mutex> lock(job->mutex);
                job->ok = ok;
                job->state = RenderJob::State::Done;
            }
            job->done.notify_all();
            ++processed;
        }
        return processed;
    }

    /// Server side: wait until a job is queued, the timeout passes or the server stops.
    void wait_for_work(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        work_.wait_for(lock, timeout, [this]() { return !queue_.empty() || stopped_; });
    }

    /// Server side: serve until running becomes false or the server stops.
    void run(const std::atomic_bool& running, std::chrono::milliseconds poll = std::chrono::milliseconds(20))
    {
        while (running.load() && !stopped()) {
            wait_for_work(poll);
            process_pending();
        }
    }

    /// Stop serving: queued jobs fail, and later submissions fail at once.
    void shutdown()
    {
        std::deque<std::shared_ptr<RenderJob>> pending;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stopped_ = true;
            pending.swap(queue_);
        }
        for (auto& job : pending) {
            fail_all(job->requests, job->count, job->statuses, "the render server has stopped");
            {
                std::lock_guard<std::mutex> lock(job->mutex);
                job->ok = false;
                job->state = RenderJob::State::Done;
            }
            job->done.notify_all();
        }
        work_.notify_all();
    }

    bool stopped() const
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return stopped_;
    }

    /// Server side, on the server thread: drop the drawers (their graphics resources).
    void release_drawers()
    {
        drawers_.clear();
    }

private:
    static void fail_all(const hakoniwa::sensor_adapter::RenderRequest* requests, std::size_t count,
                         hakoniwa::sensor_adapter::RenderStatus* statuses, const std::string& reason)
    {
        for (std::size_t i = 0; i < count; ++i) {
            statuses[i] = {};
            statuses[i].error = requests[i].camera_name + ": " + reason;
        }
    }

    // Runs on the server thread, without any lock held.
    bool draw(RenderJob& job, const FrameDrawerFactory& factory)
    {
        const auto key = std::make_pair(job.model, job.data);
        auto found = drawers_.find(key);
        if (found == drawers_.end()) {
            if (!factory) {
                fail_all(job.requests, job.count, job.statuses, "the render server has no drawer");
                return false;
            }
            std::string error;
            auto drawer = factory(job.model, job.data, error);
            if (!drawer) {
                fail_all(job.requests, job.count, job.statuses, error);
                return false;
            }
            found = drawers_.emplace(key, std::move(drawer)).first;
        }
        return found->second->draw(job.requests, job.count, job.statuses);
    }

    mutable std::mutex queue_mutex_;
    std::condition_variable work_;
    std::deque<std::shared_ptr<RenderJob>> queue_;
    bool stopped_ {false};
    FrameDrawerFactory factory_;
    std::function<void()> wakeup_;
    // Only touched on the server thread.
    std::map<std::pair<const mjModel*, mjData*>, std::unique_ptr<IFrameDrawer>> drawers_;
};

/// The world's IImageRenderer: hands every call to the render server and waits.
class RenderServerClient final : public hakoniwa::sensor_adapter::IImageRenderer
{
public:
    RenderServerClient(const mjModel* model, mjData* data, RenderServer& server,
                       std::chrono::milliseconds timeout)
        : model_(model)
        , data_(data)
        , server_(server)
        , timeout_(timeout)
    {
    }

    bool render(const hakoniwa::sensor_adapter::RenderRequest* requests, std::size_t count,
                hakoniwa::sensor_adapter::RenderStatus* statuses) override
    {
        if (count != 0 && (requests == nullptr || statuses == nullptr)) {
            return false;
        }
        return server_.submit(model_, data_, requests, count, statuses, timeout_);
    }

    hakoniwa::sensor_adapter::ImageRendererCapabilities capabilities() const override
    {
        hakoniwa::sensor_adapter::ImageRendererCapabilities value {};
        value.outputs = hakoniwa::sensor_adapter::kRenderRgb | hakoniwa::sensor_adapter::kRenderDepth;
        value.lens_distortion = false;
        value.non_square_pixels = false;
        return value;
    }

private:
    const mjModel* model_;
    mjData* data_;
    RenderServer& server_;
    std::chrono::milliseconds timeout_;
};

}  // namespace hako::robots::physics::impl
