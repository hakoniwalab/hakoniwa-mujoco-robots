#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace hako::examples::sensors
{
    inline int FindFreejointQposAddr(const mjModel* model, const char* joint_name)
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

    inline void MoveFreejointPlanar(
        mjModel* model,
        mjData* data,
        int qpos_addr,
        double dx,
        double dy)
    {
        data->qpos[qpos_addr + 0] += dx;
        data->qpos[qpos_addr + 1] += dy;
        data->qpos[qpos_addr + 3] = 1.0;
        data->qpos[qpos_addr + 4] = 0.0;
        data->qpos[qpos_addr + 5] = 0.0;
        data->qpos[qpos_addr + 6] = 0.0;
        mj_forward(model, data);
    }

    inline void MoveFreejointPlanarSteps(
        mjModel* model,
        mjData* data,
        int qpos_addr,
        int x_steps,
        int y_steps,
        double step_size)
    {
        MoveFreejointPlanar(
            model,
            data,
            qpos_addr,
            static_cast<double>(x_steps) * step_size,
            static_cast<double>(y_steps) * step_size);
    }

    inline void PrintFreejointPosition(
        const mjData* data,
        int qpos_addr,
        const char* label)
    {
        std::cout
            << label << "=("
            << std::fixed << std::setprecision(3)
            << data->qpos[qpos_addr + 0] << ", "
            << data->qpos[qpos_addr + 1] << ", "
            << data->qpos[qpos_addr + 2] << ")"
            << std::defaultfloat
            << std::endl;
    }

    inline void PrintPlanarStepMove(
        const char* label,
        int x_steps,
        int y_steps,
        double step_size)
    {
        std::cout
            << "moved " << label << ": "
            << "x " << (x_steps >= 0 ? "+= " : "-= ")
            << std::abs(x_steps) * step_size
            << ", y " << (y_steps >= 0 ? "+= " : "-= ")
            << std::abs(y_steps) * step_size
            << ", ";
    }
}
