#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace sa = hakoniwa::sensor_adapter;
using hako::robots::physics::RayCasterSpec;
using hako::robots::physics::impl::MujocoRayCaster;

// The floor is z = 0. The robot base is at z = 1 (its own geom off to the
// side) with a link 0.5 m ahead (near face x = 0.45) and a sensor body with a
// housing at the base origin. A wall stands ahead (near face x = 2.9).
// A second housing stands on the world beside the robot (y = -0.5).
const char* kXml = R"(
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="wall" pos="3 0 1">
      <geom name="wall" type="box" size="0.1 1 1"/>
    </body>
    <body name="floor_housing" pos="0 -0.5 1">
      <geom name="floor_housing" type="cylinder" size="0.035 0.02"/>
    </body>
    <body name="base" pos="0 0 1">
      <geom name="base" type="sphere" size="0.05" pos="0 0.3 0"/>
      <body name="link" pos="0.5 0 0">
        <geom name="link" type="box" size="0.05 0.05 0.05"/>
      </body>
      <body name="sensor" pos="0 0 0">
        <geom name="housing" type="cylinder" size="0.02 0.02"/>
      </body>
    </body>
  </worldbody>
</mujoco>
)";

void Expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string& message)
{
    Expect(std::fabs(actual - expected) <= tolerance,
        message + ": " + std::to_string(actual) + " != " + std::to_string(expected));
}

struct Model
{
    Model()
    {
        char message[1000] = "";
        spec = mj_parseXMLString(kXml, nullptr, message, sizeof(message));
        Expect(spec != nullptr, std::string("model: ") + message);
        model = mj_compile(spec, nullptr);
        Expect(model != nullptr, "model compiles");
        data = mj_makeData(model);
        mj_forward(model, data);
    }
    ~Model()
    {
        mj_deleteData(data);
        mj_deleteModel(model);
        mj_deleteSpec(spec);
    }
    int geom(const char* name) const { return mj_name2id(model, mjOBJ_GEOM, name); }

    mjSpec* spec {nullptr};
    mjModel* model {nullptr};
    mjData* data {nullptr};
};

sa::RayHit Cast(MujocoRayCaster& caster, double x, double y, double z, double max_distance)
{
    sa::Ray ray {};
    ray.direction = {x, y, z};
    sa::RayHit hit {};
    std::string error;
    Expect(caster.cast(&ray, 1, max_distance, &hit, error), "cast failed: " + error);
    return hit;
}

RayCasterSpec Spec(const char* exclude_root)
{
    RayCasterSpec spec;
    spec.mount_body = "sensor";
    spec.exclude_root_body = exclude_root;
    return spec;
}

void DistanceToPlane(const Model& m)
{
    MujocoRayCaster caster(m.model, m.data, Spec(""));
    const auto hit = Cast(caster, 0, 0, -1, 10.0);
    Expect(hit.hit, "the floor is hit");
    Expect(hit.object_id == m.geom("floor"), "the floor geom id is returned");
    ExpectNear(hit.distance, 1.0, 1e-9, "distance to the floor");
    ExpectNear(hit.normal.z, 1.0, 1e-9, "floor normal points up in the sensor axes");
}

void DistanceToBoxAndSelfExclusion(const Model& m)
{
    // Only the sensor body is excluded: the arm's own link is hit.
    MujocoRayCaster sensor_only(m.model, m.data, Spec(""));
    auto hit = Cast(sensor_only, 1, 0, 0, 10.0);
    Expect(hit.hit && hit.object_id == m.geom("link"), "the link is hit when not excluded");
    ExpectNear(hit.distance, 0.45, 1e-9, "distance to the link box");

    // The base is the exclusion root: the link and the base are skipped.
    MujocoRayCaster whole_robot(m.model, m.data, Spec("base"));
    hit = Cast(whole_robot, 1, 0, 0, 10.0);
    Expect(hit.hit && hit.object_id == m.geom("wall"), "the wall is hit past the excluded link");
    ExpectNear(hit.distance, 2.9, 1e-5, "distance to the wall box");
    ExpectNear(hit.normal.x, -1.0, 1e-9, "wall normal faces the sensor");
}

void HousingOnWorldStaysExcludedWithRoot(const Model& m)
{
    // The housing is not under the base: hiding the robot must not unhide it.
    RayCasterSpec spec;
    spec.mount_body = "floor_housing";
    spec.exclude_root_body = "base";
    MujocoRayCaster caster(m.model, m.data, spec);
    auto hit = Cast(caster, 1, 0, 0, 10.0);
    Expect(hit.hit && hit.object_id == m.geom("wall"), "the own housing is skipped, the wall is hit");
    ExpectNear(hit.distance, 2.9, 1e-5, "distance from the world housing to the wall");
    // Towards the robot (+y): the link is excluded with the base; nothing else there.
    hit = Cast(caster, 0.5, 0.5, 0, 10.0);
    Expect(!hit.hit || (hit.object_id != m.geom("link") && hit.object_id != m.geom("base")
               && hit.object_id != m.geom("floor_housing")),
        "neither the robot nor the own housing is hit");

    // Without a root, the robot is visible from the world housing.
    RayCasterSpec visible;
    visible.mount_body = "floor_housing";
    MujocoRayCaster sees_robot(m.model, m.data, visible);
    hit = Cast(sees_robot, 0.5, 0.5, 0, 10.0);
    Expect(hit.hit && hit.object_id == m.geom("link"), "the robot's link is visible by default");
}

void MissBeyondMaxDistance(const Model& m)
{
    MujocoRayCaster caster(m.model, m.data, Spec("base"));
    Expect(!Cast(caster, 1, 0, 0, 2.0).hit, "the wall beyond max distance is no hit");
    Expect(!Cast(caster, 0, 0, 1, 10.0).hit, "nothing above is no hit");
}

void MountPoseAndNormalInSensorAxes(const Model& m)
{
    // Yaw +90 deg: sensor x = world y, sensor y = world -x.
    auto spec = Spec("base");
    spec.orientation[0] = std::cos(M_PI / 4.0);
    spec.orientation[3] = std::sin(M_PI / 4.0);
    MujocoRayCaster caster(m.model, m.data, spec);
    const auto hit = Cast(caster, 0, -1, 0, 10.0);  // world +x
    Expect(hit.hit && hit.object_id == m.geom("wall"), "the rotated ray hits the wall");
    ExpectNear(hit.normal.x, 0.0, 1e-9, "normal x in the sensor axes");
    ExpectNear(hit.normal.y, 1.0, 1e-9, "normal y in the sensor axes");

    // Mount position 0.1 m up in the sensor body frame.
    auto raised = Spec("");
    raised.position[2] = 0.1;
    MujocoRayCaster up(m.model, m.data, raised);
    ExpectNear(Cast(up, 0, 0, -1, 10.0).distance, 1.1, 1e-9, "distance from the raised mount");
}

void ReadsOnly(const Model& m)
{
    const std::vector<mjtNum> before(m.data->xpos, m.data->xpos + 3 * m.model->nbody);
    MujocoRayCaster caster(m.model, m.data, Spec("base"));
    Cast(caster, 1, 0, 0, 10.0);
    Expect(std::memcmp(before.data(), m.data->xpos, before.size() * sizeof(mjtNum)) == 0,
        "the caster does not change the data");
}

void RejectsUnknownBodies(const Model& m)
{
    auto expect_throw = [&](const RayCasterSpec& spec, const std::string& what) {
        try {
            MujocoRayCaster caster(m.model, m.data, spec);
        } catch (const std::invalid_argument&) {
            return;
        }
        throw std::runtime_error(what + " was accepted");
    };
    RayCasterSpec unknown_mount;
    unknown_mount.mount_body = "nothing";
    expect_throw(unknown_mount, "an unknown mount body");
    RayCasterSpec world_mount;
    world_mount.mount_body = "world";
    expect_throw(world_mount, "the world body as mount");
    expect_throw(Spec("nothing"), "an unknown exclusion root");
    auto zero = Spec("");
    zero.orientation[0] = 0.0;
    expect_throw(zero, "a zero quaternion");
}

void WorldFactory()
{
    hako::robots::physics::impl::WorldImpl world;
    // The world loads from a file; reuse the XML through a temporary file.
    const std::string path = "ray_caster_test_model.xml";
    {
        FILE* f = std::fopen(path.c_str(), "w");
        Expect(f != nullptr, "temporary model file");
        std::fputs(kXml, f);
        std::fclose(f);
    }
    world.loadModel(path);
    std::remove(path.c_str());
    auto caster = world.createRayCaster(Spec("base"));
    Expect(caster != nullptr, "the MuJoCo world creates a ray caster");
    sa::Ray ray {};
    ray.direction = {1, 0, 0};
    sa::RayHit hit {};
    std::string error;
    Expect(caster->cast(&ray, 1, 10.0, &hit, error) && hit.hit, "the world's caster hits the wall");
    ExpectNear(hit.distance, 2.9, 1e-5, "world caster distance");
}

}  // namespace

int main()
{
    try {
        Model m;
        DistanceToPlane(m);
        DistanceToBoxAndSelfExclusion(m);
        MissBeyondMaxDistance(m);
        HousingOnWorldStaysExcludedWithRoot(m);
        MountPoseAndNormalInSensorAxes(m);
        ReadsOnly(m);
        RejectsUnknownBodies(m);
        WorldFactory();
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "ray_caster_test: OK" << std::endl;
    return 0;
}
