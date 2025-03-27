#pragma once

#include <mujoco/mujoco.h>
#include <iostream>
#include <string>

/**
 * @file mujoco_debug.h
 * @brief MuJoCoのデバッグ出力関数群
 *
 * このヘッダーファイルでは、MuJoCoの状態を出力するための関数を提供する。
 * - 関節（joint）の状態 (`qpos`, `qvel`)
 * - 剛体（body）のワールド座標 (`xpos`)
 * - アクチュエータ（actuator）の制御入力 (`ctrl`)
 * - 関節の種類 (`joint type`)
 * - 剛体の慣性 (`mass`, `inertia`)
 * - アクチュエータの制御範囲 (`ctrlrange`)
 * - シミュレーション全体の状態
 *
 * @author Takashi Mori
 * @date 2025-02-04
 */

/**
 * @brief 指定した関節の種類を取得する
 * @param model MuJoCoのモデルデータ
 * @param joint_name 取得する関節の名前
 * @return 関節の種類（hinge, slide, ball, free など）
 */
std::string get_joint_type_by_name(const mjModel* model, const std::string& joint_name);

/**
 * @brief 指定した関節の状態を取得し、出力する
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param joint_name 取得する関節の名前
 */
void print_joint_state_by_name(const mjModel* model, const mjData* data, const std::string& joint_name);

/**
 * @brief 指定した剛体のワールド座標を取得し、出力する
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param body_name 取得するボディの名前
 */
void print_body_state_by_name(const mjModel* model, const mjData* data, const std::string& body_name);

/**
 * @brief 指定した剛体の慣性情報（質量、慣性テンソル）を取得し、出力する
 * @param model MuJoCoのモデルデータ
 * @param body_name 取得するボディの名前
 */
void print_body_inertia_by_name(const mjModel* model, const std::string& body_name);

/**
 * @brief 指定した剛体の姿勢（オイラー角）を取得し、ラジアンで出力
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param body_name 取得するボディの名前
 */
void print_body_orientation_by_name_rad(const mjModel* model, const mjData* data, const std::string& body_name);

/**
 * @brief 指定した剛体の姿勢（オイラー角）を取得し、度数法で出力
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param body_name 取得するボディの名前
 */
void print_body_orientation_by_name_deg(const mjModel* model, const mjData* data, const std::string& body_name);

/**
 * @brief 指定したアクチュエータの制御入力を取得し、出力する
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 * @param actuator_name 取得するアクチュエータの名前
 */
void print_actuator_by_name(const mjModel* model, const mjData* data, const std::string& actuator_name);

/**
 * @brief 指定したアクチュエータの制御範囲 (`ctrlrange`) を取得し、出力する
 * @param model MuJoCoのモデルデータ
 * @param actuator_name 取得するアクチュエータの名前
 */
void print_actuator_range_by_name(const mjModel* model, const std::string& actuator_name);

/**
 * @brief シミュレーション全体の状態を出力する
 * @param model MuJoCoのモデルデータ
 * @param data MuJoCoのシミュレーションデータ
 */
void print_all_states(const mjModel* model, const mjData* data);

/**
 * @brief MuJoCoのデータ構造概要
 *
 * - **mjModel** (`model`): MuJoCoのシミュレーションモデル
 *   - `model->nq`: 位置変数（qpos）の数
 *   - `model->nv`: 速度変数（qvel）の数
 *   - `model->nu`: 制御入力（ctrl）の数
 *   - `model->nbody`: ボディの数
 *   - `model->njnt`: 関節の数
 *
 * - **mjData** (`data`): シミュレーションの動的データ
 *   - `data->qpos[nq]`: 関節の位置（関節座標系）
 *   - `data->qvel[nv]`: 関節の速度
 *   - `data->xpos[3 * nbody]`: 剛体のワールド座標位置
 *   - `data->ctrl[nu]`: アクチュエータ制御入力
 */
