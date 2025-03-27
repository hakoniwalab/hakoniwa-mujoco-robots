#pragma once

#include "physics.hpp"

namespace hako::robots::physics::impl
{

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
            // R: 3x3 row-major
            euler.y = asin(-R[2]);  // R[2] = -sin(pitch)
            double cos_pitch = cos(euler.y);
        
            if (fabs(cos_pitch) > 1e-6) {
                euler.x = atan2(R[5], R[8]);  // R[5]=R[1][2], R[8]=R[2][2]
                euler.z  = atan2(R[1], R[0]);  // R[1]=R[0][1], R[0]=R[0][0]
            } else {
                // gimbal lock
                euler.x = 0;
                euler.z  = atan2(-R[3], R[4]);  // fallback
            }
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
            hako::robots::types::Velocity vel;
            vel.x = data->cvel[6 * body_id];
            vel.y = data->cvel[6 * body_id + 1];
            vel.z = data->cvel[6 * body_id + 2];
            return vel;
        }
        hako::robots::types::EulerRate GetEulerRate() override
        {
            hako::robots::types::EulerRate euler_rate;
            euler_rate.x = data->cvel[6 * body_id + 3];
            euler_rate.y = data->cvel[6 * body_id + 4];
            euler_rate.z = data->cvel[6 * body_id + 5];
            return euler_rate;
        }
        hako::robots::types::BodyVelocity GetBodyVelocity() override
        {
            hako::robots::types::BodyVelocity body_vel;
            body_vel.x = data->cvel[6 * body_id];
            body_vel.y = data->cvel[6 * body_id + 1];
            body_vel.z = data->cvel[6 * body_id + 2];
            return body_vel;
        }
        hako::robots::types::BodyAngularVelocity GetBodyAngularVelocity() override
        {
            hako::robots::types::BodyAngularVelocity angular_vel;
            angular_vel.x = data->cvel[6 * body_id + 3];
            angular_vel.y = data->cvel[6 * body_id + 4];
            angular_vel.z = data->cvel[6 * body_id + 5];
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
    public:
        WorldImpl() {}
        virtual ~WorldImpl() {}
        void loadModel(const std::string& model_file) override
        {
            model = mj_loadXML(model_file.c_str(), nullptr, nullptr, 0);
            if (!model) {
                throw std::runtime_error("Model loading failed");
            }
            data = mj_makeData(model);
        }
        void advanceTimeStep() override
        {
            mj_step(model, data);
        }
        std::shared_ptr<IRigidBody> getRigidBody(const std::string& model_name) override
        {
            return std::make_shared<RigidBodyImpl>(model, data, model_name);
        }
    };
}
