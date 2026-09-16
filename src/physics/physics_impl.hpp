#pragma once

#include "physics.hpp"
#include "actuator/actuator_impl.hpp"
#include "actuator/joint_actuator_impl.hpp"
#include "actuator/named_actuator_impl.hpp"
#include "actuator/joint_trajectory_actuator_impl.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace hako {
namespace robots {
namespace physics {
namespace impl {

    class RigidBodyImpl : public IRigidBody
    {
    private:
        mjModel* model;
        mjData* data;
        int body_id;
        std::unordered_map<std::string, int> joint_id_map;
        int getJointId(const std::string& joint_name) {
            auto it = joint_id_map.find(joint_name);
            if (it != joint_id_map.end()) {
                return it->second;
            }
            int id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
            if (id < 0) {
                throw std::runtime_error("Joint not found: " + joint_name);
            }
            joint_id_map[joint_name] = id;
            return id;
        }

        void mat2euler(const mjtNum* R, hako::robots::types::Euler& euler) {
            // R is 3x3 row-major
            euler.y = std::asin(R[6]);  // pitch = asin(R20)
        
            double cos_pitch = std::cos(euler.y);
            if (std::fabs(cos_pitch) > 1e-6) {
                euler.x = std::atan2(-R[7], R[8]);  // roll = atan2(-R21, R22)
                euler.z = std::atan2(R[3], R[0]);   // yaw = atan2(R10, R00)
            }
            else {
                // gimbal lock fallback
                euler.x = 0;
                euler.z = std::atan2(-R[1], R[4]);  // yaw fallback = atan2(-R01, R11)
            }
        }
        
        
        static hako::robots::types::Vector3 transformWorldToBody(
            const hako::robots::types::Vector3& vel,
            const hako::robots::types::Euler& angle)
        {
            double c_phi   = std::cos(angle.x), s_phi = std::sin(angle.x);
            double c_theta = std::cos(angle.y), s_theta = std::sin(angle.y);
            double c_psi   = std::cos(angle.z), s_psi = std::sin(angle.z);
    
            double x_e = vel.x;
            double y_e = vel.y;
            double z_e = vel.z;
    
            hako::robots::types::BodyVelocity body_vel;
            body_vel.x =   (c_theta * c_psi)                         * x_e
                         + (c_theta * s_psi)                         * y_e
                         - (s_theta)                                 * z_e;
    
            body_vel.y =   (s_phi * s_theta * c_psi - c_phi * s_psi) * x_e
                         + (s_phi * s_theta * s_psi + c_phi * c_psi) * y_e
                         + (s_phi * c_theta)                         * z_e;
    
            body_vel.z =   (c_phi * s_theta * c_psi + s_phi * s_psi) * x_e
                         + (c_phi * s_theta * s_psi - s_phi * c_psi) * y_e
                         + (c_phi * c_theta)                         * z_e;
    
            return body_vel;
        }
    public:
        RigidBodyImpl(mjModel* model, mjData* data, const std::string& model_name)
            : model(model), data(data)
        {
            body_id = mj_name2id(model, mjOBJ_BODY, model_name.c_str());
            if (body_id < 0) {
                throw std::runtime_error("Body not found: " + model_name);
            }
        }
        virtual ~RigidBodyImpl() override {}

        hako::robots::types::Position GetPosition() override
        {
            hako::robots::types::Position pos;
            pos.x = data->xpos[3 * body_id];
            pos.y = data->xpos[3 * body_id + 1];
            pos.z = data->xpos[3 * body_id + 2];
            return pos;
        }
        hako::robots::types::Euler GetEuler() override
        {
            hako::robots::types::Euler euler;
            const mjtNum* mat = &(data->xmat[9 * body_id]);
            mat2euler(mat, euler);
            return euler;
        }
        hako::robots::types::Velocity GetVelocity() override
        {
            mjtNum object_velocity[6] = {};
            mj_objectVelocity(model, data, mjOBJ_BODY, body_id, object_velocity, 0);
            hako::robots::types::Velocity vel;
            vel.x = object_velocity[3];
            vel.y = object_velocity[4];
            vel.z = object_velocity[5];
            return vel;
        }
        hako::robots::types::EulerRate GetEulerRate() override
        {
            hako::robots::types::EulerRate euler_rate = {};
            //not supported
            return euler_rate;
        }
        hako::robots::types::BodyVelocity GetBodyVelocity() override
        {
            mjtNum object_velocity[6] = {};
            // mjOBJ_XBODY uses the regular body frame (xpos/xmat). mjOBJ_BODY
            // would express local velocity in the body's inertial frame
            // (xipos/ximat), which can be rotated independently by <inertial>.
            mj_objectVelocity(model, data, mjOBJ_XBODY, body_id, object_velocity, 1);
            hako::robots::types::BodyVelocity body_vel;
            body_vel.x = object_velocity[3];
            body_vel.y = object_velocity[4];
            body_vel.z = object_velocity[5];
            return body_vel;
        }
        hako::robots::types::BodyAngularVelocity GetBodyAngularVelocity() override
        {
            mjtNum object_velocity[6] = {};
            // Keep angular velocity in the same regular body frame used by
            // xquat and by sensor/body frame conventions.
            mj_objectVelocity(model, data, mjOBJ_XBODY, body_id, object_velocity, 1);
            hako::robots::types::BodyAngularVelocity angular_vel;
            angular_vel.x = object_velocity[0];
            angular_vel.y = object_velocity[1];
            angular_vel.z = object_velocity[2];
            return angular_vel;
        }
        void SetTorque(const std::string& joint_name, double torque) override
        {
            int joint_id = getJointId(joint_name);
            data->ctrl[joint_id] = torque;
        }
        void SetForce(const hako::robots::types::Vector3& force) override
        {
            data->xfrc_applied[6 * body_id] = force.x;
            data->xfrc_applied[6 * body_id + 1] = force.y;
            data->xfrc_applied[6 * body_id + 2] = force.z;
        }
    };
    class WorldImpl : public IWorld
    {
    private:
        static std::string ModelExtension(const std::string& model_file)
        {
            std::string extension = std::filesystem::path(model_file).extension().string();
            std::transform(
                extension.begin(), extension.end(), extension.begin(),
                [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            return extension;
        }

    public:
        WorldImpl() {}
        virtual ~WorldImpl() {}
        void loadModel(const std::string& model_file) override
        {
            const std::string extension = ModelExtension(model_file);
            mjModel* loaded_model = nullptr;
            if (extension == ".xml") {
                std::array<char, 4096> error {};
                loaded_model = mj_loadXML(
                    model_file.c_str(), nullptr, error.data(),
                    static_cast<int>(error.size()));
                if (loaded_model == nullptr) {
                    const std::string detail = error.data()[0] != '\0'
                        ? std::string(": ") + error.data()
                        : std::string();
                    throw std::runtime_error(
                        "MuJoCo XML model loading failed: " + model_file + detail);
                }
            } else if (extension == ".mjb") {
                loaded_model = mj_loadModel(model_file.c_str(), nullptr);
                if (loaded_model == nullptr) {
                    throw std::runtime_error(
                        "MuJoCo MJB model loading failed: " + model_file
                        + ". MJB artifacts must be generated with a compatible MuJoCo version.");
                }
            } else {
                throw std::runtime_error(
                    "Unsupported MuJoCo model extension '" + extension
                    + "': " + model_file + ". Expected .xml or .mjb.");
            }

            mjData* loaded_data = mj_makeData(loaded_model);
            if (loaded_data == nullptr) {
                mj_deleteModel(loaded_model);
                throw std::runtime_error("MuJoCo data allocation failed: " + model_file);
            }

            if (data != nullptr) {
                mj_deleteData(data);
            }
            if (model != nullptr) {
                mj_deleteModel(model);
            }
            model = loaded_model;
            data = loaded_data;
            mj_forward(model, data);
        }
        void advanceTimeStep() override
        {
            mj_step(model, data);
        }
        std::shared_ptr<IRigidBody> getRigidBody(const std::string& model_name) override
        {
            return std::make_shared<RigidBodyImpl>(model, data, model_name);
        }
        std::shared_ptr<actuator::ITorqueActuator> getTorqueActuator(const std::string& name) override {
            return std::make_shared<actuator::impl::TorqueActuatorImpl>(model, data, name);
        }
        std::shared_ptr<actuator::IJointActuator> createJointActuator() override {
            return std::make_shared<actuator::impl::JointActuatorImpl>(model, data);
        }
        std::shared_ptr<actuator::INamedActuator> createNamedActuator() override {
            return std::make_shared<actuator::impl::NamedActuatorImpl>(model, data);
        }
        std::shared_ptr<actuator::IJointTrajectoryActuator> createJointTrajectoryActuator() override {
            return std::make_shared<actuator::impl::JointTrajectoryActuatorImpl>(model, data);
        }
        
    };
}  // namespace impl
}  // namespace physics
}  // namespace robots
}  // namespace hako
