#pragma once

#include "physics.hpp"
#include "actuator/actuator_impl.hpp"

namespace hako::robots::physics::impl
{

    class RigidBodyImpl : public IRigidBody
    {
    private:
        mjModel* model;
        mjData* data;
        int body_id;
        int jnt_id;
        int dof_index;
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
            euler.y = asin(R[6]);  // pitch = asin(R20)
        
            double cos_pitch = cos(euler.y);
            if (fabs(cos_pitch) > 1e-6) {
                euler.x = atan2(-R[7], R[8]);  // roll = atan2(-R21, R22)
                euler.z = atan2(R[3], R[0]);   // yaw = atan2(R10, R00) ← 符号修正！
            }
            else {
                // gimbal lock fallback
                euler.x = 0;
                euler.z = atan2(-R[1], R[4]);  // yaw fallback = atan2(-R01, R11)
            }
        }
        
        
        static hako::robots::types::Vector3 transformWorldToBody(
            const hako::robots::types::Vector3& vel,
            const hako::robots::types::Euler& angle)
        {
            using std::cos; using std::sin;
    
            double c_phi   = cos(angle.x), s_phi = sin(angle.x);
            double c_theta = cos(angle.y), s_theta = sin(angle.y);
            double c_psi   = cos(angle.z), s_psi = sin(angle.z);
    
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
            jnt_id = model->body_jntadr[body_id];
            if (jnt_id < 0) {
                throw std::runtime_error("jnt_id not found: " + model_name);
            }
            dof_index = model->jnt_dofadr[jnt_id];
            if (dof_index < 0) {
                throw std::runtime_error("dof_index not found: " + model_name);
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
            vel.x = data->qvel[dof_index + 0];
            vel.y = data->qvel[dof_index + 1];
            vel.z = data->qvel[dof_index + 2];
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
            auto vel = GetVelocity();
            auto euler = GetEuler();
            return transformWorldToBody(vel, euler);
        }
        hako::robots::types::BodyAngularVelocity GetBodyAngularVelocity() override
        {
            hako::robots::types::BodyAngularVelocity angular_vel;
            angular_vel.x = data->qvel[dof_index + 3];
            angular_vel.y = data->qvel[dof_index + 4];
            angular_vel.z = data->qvel[dof_index + 5];
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
        
    };
}
