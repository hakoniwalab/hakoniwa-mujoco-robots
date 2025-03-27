#include <mujoco/mujoco.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <mutex>
#include "mujoco_debug.hpp"
#include "mujoco_viewer.hpp"

// MuJoCoのモデルとデータ
static mjData* mujoco_data;
static mjModel* mujoco_model;
static const std::string model_path = "models/forklift/forklift.xml";

#include <math.h>

// 回転行列からZ-Y-Xオイラー角（roll, pitch, yaw）を取り出す
void mat2euler(const mjtNum* R, double* roll, double* pitch, double* yaw) {
    // R: 3x3 row-major
    *pitch = asin(-R[2]);  // R[2] = -sin(pitch)
    double cos_pitch = cos(*pitch);

    if (fabs(cos_pitch) > 1e-6) {
        *roll = atan2(R[5], R[8]);  // R[5]=R[1][2], R[8]=R[2][2]
        *yaw  = atan2(R[1], R[0]);  // R[1]=R[0][1], R[0]=R[0][0]
    } else {
        // gimbal lock
        *roll = 0;
        *yaw  = atan2(-R[3], R[4]);  // fallback
    }

    // ラジアン → 度
    *roll  *= 180.0 / M_PI;
    *pitch *= 180.0 / M_PI;
    *yaw   *= 180.0 / M_PI;
}

#include <stdio.h>

void print_qvel(const char* body_name, const mjModel* m, const mjData* d) {
    // body ID を取得
    int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
    if (body_id == -1) {
        printf("[ERROR] body '%s' not found!\n", body_name);
        return;
    }

    // qvel の開始インデックス
    int dof_start = m->body_dofadr[body_id];

    // 線速度だけ取得（並進3自由度）
    double vx = d->qvel[dof_start + 0];
    double vy = d->qvel[dof_start + 1];
    double vz = d->qvel[dof_start + 2];

    // 出力（m/s）
    printf("[QVEL] %s linear velocity: [%.3f, %.3f, %.3f] m/s\n",
           body_name, vx, vy, vz);
}



void print_position(const char* parts_name, const mjModel* model, const mjData* data) {
    int body_id = mj_name2id(model, mjOBJ_BODY, parts_name);
    if (body_id == -1) {
        printf("[ERROR] body '%s' not found!\n", parts_name);
        return;
    }

    const mjtNum* pos = &(data->xpos[3 * body_id]);
    const mjtNum* mat = &(data->xmat[9 * body_id]);

    double roll, pitch, yaw;
    mat2euler(mat, &roll, &pitch, &yaw);

    printf("[INFO] %s pos: x=%.3f, y=%.3f, z=%.3f | r=%.1f°, p=%.1f°, y=%.1f°\n",
           parts_name, pos[0], pos[1], pos[2], roll, pitch, yaw);
}


// シミュレーションスレッド
void simulation_thread(mjModel* model, mjData* data, bool& running_flag, std::mutex& mutex) {
    double simulation_timestep = model->opt.timestep;  // **XMLから `timestep` を取得**
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        //get simulation time
        double sim_time = data->time;
        //std::cout << "sim_time: " << sim_time << std::endl;
        {
            std::lock_guard<std::mutex> lock(mutex);
            data->ctrl[0] = 0.2;  // 左モーター
            data->ctrl[1] = 0.2;  // 右モーター
            if (sim_time > 10.0) {
                data->ctrl[2] = 0.23; // フォークリフトのリフト
            } else {
                data->ctrl[2] = 0.0; // フォークリフトのリフト
            }
            mj_step(model, data);
            print_position("forklift_base", model, data);
            print_position("lift_arm", model, data);
            print_position("pallet", model, data);
            print_qvel("forklift_base", model, data);

            //print_body_state_by_name(model, data, "tb3_base");
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double sleep_time = simulation_timestep - elapsed.count();

        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }
}


int main(int argc, const char* argv[])
{
    // **MuJoCoモデルの読み込み**
    char error[1000];
    std::cout << "[INFO] Loading model: " << model_path << std::endl;
    mujoco_model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!mujoco_model) {
        std::cerr << "[ERROR] Failed to load model: " << model_path << "\n" << error << std::endl;
        return 1;
    }
    std::cout << "[INFO] Model loaded successfully." << std::endl;

    // **データの作成**
    std::cout << "[INFO] Creating simulation data." << std::endl;
    mujoco_data = mj_makeData(mujoco_model);

    // **初期状態を正しく計算する**
    mj_forward(mujoco_model, mujoco_data);
    
    // **シミュレーションの実行**
    const double dt = mujoco_model->opt.timestep;
    std::cout << "[INFO] Starting simulation." << std::endl;
    
    std::mutex data_mutex;
    bool running_flag = true;
    std::thread sim_thread(simulation_thread, mujoco_model, mujoco_data, std::ref(running_flag), std::ref(data_mutex));
    viewer_thread(mujoco_model, mujoco_data, std::ref(running_flag), std::ref(data_mutex));
    running_flag = false;
    sim_thread.join();
    // **リソース解放**
    std::cout << "[INFO] Cleaning up resources." << std::endl;
    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);

    std::cout << "[INFO] Simulation completed successfully." << std::endl;
    return 0;
}
