#pragma once

#include "primitive_types.hpp"
#include "actuator.hpp"
#include <memory>
#include <string>

namespace hakoniwa::sensor_adapter
{
    class IRayCaster;
    class IImageRenderer;
}

namespace hako::robots::physics
{
    /**
     * Where a ray caster is mounted and what it never hits.
     *
     * The sensor axes are x forward, y left, z up. position [m] and
     * orientation (unit quaternion w, x, y, z) are given in the frame of
     * mount_body. Geoms of the mount body (the sensor housing) and of the
     * bodies under it are never hit. exclude_root_body, when set, adds a
     * second subtree that is never hit, for example an arm's base to hide
     * the whole arm; by default the arm stays visible, as with a real sensor.
     */
    struct RayCasterSpec
    {
        std::string mount_body;
        double position[3] {0.0, 0.0, 0.0};
        double orientation[4] {1.0, 0.0, 0.0, 0.0};
        std::string exclude_root_body;
    };

    class IRigidBody
    {
    public:
        virtual ~IRigidBody() {}
        virtual hako::robots::types::Position GetPosition() = 0;
        virtual hako::robots::types::Euler GetEuler() = 0;
        virtual hako::robots::types::Velocity GetVelocity() = 0;
        virtual hako::robots::types::EulerRate GetEulerRate() = 0;
        virtual hako::robots::types::BodyVelocity GetBodyVelocity() = 0;
        virtual hako::robots::types::BodyAngularVelocity GetBodyAngularVelocity() = 0;

        virtual void SetTorque(const std::string& joint_name, double torque) = 0;
        virtual void SetForce(const hako::robots::types::Vector3& force) = 0;

    };
    class IWorld
    {
    protected:
        mjModel* model = nullptr;
        mjData* data = nullptr;
    public:
        virtual ~IWorld()
        {
            if (data) {
                mj_deleteData(data);
                data = nullptr;
            }
            if (model) {
                mj_deleteModel(model);
                model = nullptr;
            }
        }
        virtual mjModel *getModel() const { return model; }
        virtual mjData *getData() const { return data; }
        virtual void loadModel(const std::string& model_file) = 0;
        virtual void advanceTimeStep() = 0;
        virtual std::shared_ptr<IRigidBody> getRigidBody(const std::string& model_name) = 0;
        virtual std::shared_ptr<actuator::ITorqueActuator> getTorqueActuator(const std::string& name) = 0;
        virtual std::shared_ptr<actuator::IJointActuator> createJointActuator() { return nullptr; }
        virtual std::shared_ptr<actuator::INamedActuator> createNamedActuator() { return nullptr; }
        virtual std::shared_ptr<actuator::IJointTrajectoryActuator> createJointTrajectoryActuator() { return nullptr; }
        /**
         * Creates a ray caster for range sensors (hakoniwa-sensor-adapter
         * IRayCaster). Rays are cast at the state of the last mj_forward; the
         * caller recomputes after mj_step. Returns nullptr when the world has
         * no ray caster. Throws std::invalid_argument for an unknown body.
         */
        virtual std::shared_ptr<hakoniwa::sensor_adapter::IRayCaster> createRayCaster(const RayCasterSpec&) { return nullptr; }
        /**
         * Creates an image renderer for camera sensors (hakoniwa-sensor-adapter
         * IImageRenderer). Each call hands the requests to the process's render
         * server and waits: the server draws on the thread that runs it (the
         * main thread, see RenderServer), at the world state the caller froze by
         * waiting. Returns nullptr when the world has no renderer. The world
         * must live for the whole run and not load another model; the
         * renderer must not be used after that.
         */
        virtual std::shared_ptr<hakoniwa::sensor_adapter::IImageRenderer> createImageRenderer() { return nullptr; }
    };

}
