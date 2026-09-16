#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using hako::robots::physics::impl::WorldImpl;

void Expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::filesystem::path RepoRoot()
{
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path()
            / ("hakoniwa-mujoco-model-loader-" + std::to_string(suffix));
        std::filesystem::create_directories(path_);
    }

    ~TemporaryDirectory()
    {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

void ExpectEquivalent(const mjModel* xml, const mjModel* mjb)
{
    Expect(xml != nullptr, "XML model should be loaded");
    Expect(mjb != nullptr, "MJB model should be loaded");
    Expect(xml->nbody == mjb->nbody, "body counts should match");
    Expect(xml->ngeom == mjb->ngeom, "geom counts should match");
    Expect(xml->njnt == mjb->njnt, "joint counts should match");
    Expect(xml->nq == mjb->nq, "qpos sizes should match");
    Expect(xml->nv == mjb->nv, "DOF counts should match");
    Expect(mj_name2id(mjb, mjOBJ_BODY, "base_link") >= 0, "base_link should exist in MJB");
    Expect(
        mj_name2id(mjb, mjOBJ_JOINT, "base_freejoint") >= 0,
        "base_freejoint should exist in MJB");
}

void TestXmlAndMjbLoading()
{
    const auto xml_path = RepoRoot() / "models/sensors/imu/rotated-inertial-frame-test.xml";
    auto xml_world = std::make_shared<WorldImpl>();
    xml_world->loadModel(xml_path.string());

    TemporaryDirectory temporary;
    const auto mjb_path = temporary.path() / "model.mjb";
    mj_saveModel(xml_world->getModel(), mjb_path.string().c_str(), nullptr, 0);
    Expect(std::filesystem::file_size(mjb_path) > 0, "MuJoCo should write a non-empty MJB");

    auto mjb_world = std::make_shared<WorldImpl>();
    mjb_world->loadModel(mjb_path.string());
    ExpectEquivalent(xml_world->getModel(), mjb_world->getModel());
    mjb_world->advanceTimeStep();
    Expect(mjb_world->getData()->time > 0.0, "MJB-backed world should advance time");
}

void TestUnsupportedExtensionIsRejected()
{
    auto world = std::make_shared<WorldImpl>();
    try {
        world->loadModel("model.urdf");
    } catch (const std::runtime_error& error) {
        const std::string message = error.what();
        Expect(
            message.find("Unsupported MuJoCo model extension") != std::string::npos,
            "unsupported extension error should be actionable");
        return;
    }
    throw std::runtime_error("unsupported model extension should fail");
}
}

int main()
{
    try {
        TestXmlAndMjbLoading();
        TestUnsupportedExtensionIsRejected();
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "world_model_loader_test passed" << std::endl;
    return EXIT_SUCCESS;
}
