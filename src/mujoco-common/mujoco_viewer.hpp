#pragma once

#include <mujoco/mujoco.h>
#include <mutex>

/**
 * @brief MuJoCoの3Dビューアスレッド
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param running_flag シミュレーションの実行フラグ
 * @param mutex MuJoCoデータの排他制御用ミューテックス
 */
void viewer_thread(mjModel* model, mjData* data, bool& running_flag, std::mutex& mutex);