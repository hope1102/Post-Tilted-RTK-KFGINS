/*
 * Tilt-RTK: Tilted RTK-GNSS/INS Integrated Navigation System
 *
 * 基于KF-GINS的倾斜RTK扩展，支持ZUPT（零速更新）和快速初始化
 *
 * Copyright (C) 2024
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TILT_RTK_TYPES_H
#define TILT_RTK_TYPES_H

#include <Eigen/Dense>
#include <vector>

#include "../common/types.h"
#include "../kf-gins/kf_gins_types.h"

/**
 * @brief 倾斜RTK工作流程状态枚举
 *
 * 系统分为4个主要阶段：
 * 1. INIT: 初始化阶段，等待GNSS数据
 * 2. GYRO_BIAS_EST: 陀螺零偏估计阶段（静止2秒）
 * 3. HEADING_ALIGN: 航向对准阶段（杆尖固定，杆身摆动3秒）
 * 4. MEASURE: 测量模式（ZUPT + RTK）
 */
enum class TiltState {
    INIT = 0,           // 初始化阶段
    GYRO_BIAS_EST,      // 陀螺零偏估计阶段
    HEADING_ALIGN,      // 航向对准阶段
    MEASURE,            // 测量模式
    ERROR               // 错误状态
};

/**
 * @brief ZUPT检测器配置参数
 */
struct ZUPTConfig {
    // IMU静止检测（加速度方差）
    double acc_threshold = 0.05;      // [m/s²]
    int acc_window = 10;              // 滑动窗口大小 [样本数]

    // IMU静止检测（角速度阈值）
    double gyro_threshold = 0.1;     // [deg/s]
    int gyro_window = 10;             // 滑动窗口大小 [样本数]

    // GNSS静止检测
    double gnss_vel_threshold = 0.02;    // [m/s]
    int gnss_window = 5;                 // 滑动窗口大小 [样本数]

    // 双重条件同时满足才触发ZUPT
    bool require_both_conditions = true;

    // ZUPT启用后的最小持续时间 [秒]
    double min_zupt_duration = 0.5;

    // ZUPT量测噪声 [m/s]
    double measurement_noise = 0.01;
};

/**
 * @brief 工作流程配置参数
 */
struct WorkflowConfig {
    // 零偏估计阶段：静止时长 [秒]
    double gyro_bias_est_time = 2.0;

    // 航向对准阶段：摆动对准时长 [秒]
    double heading_align_time = 3.0;

    // 航向对准有效性的GNSS轨迹长度阈值 [m]
    double gnss_traj_length_threshold = 0.1;

    // 航向对准INS与RTK轨迹最小相关系数
    double min_traj_correlation = 0.8;
};

/**
 * @brief 杆臂配置参数
 */
struct LeverArmConfig {
    // IMU到杆尖的杆臂 (IMU坐标系前右下方向) [m]
    Eigen::Vector3d imu_to_tip = Eigen::Vector3d(0.0, 0.0, 1.8);
};

/**
 * @brief 调试输出配置
 */
struct DebugConfig {
    bool output_state_transitions = true;
    bool output_zupt_status = true;
    bool output_heading_align = true;
};

/**
 * @brief 倾斜RTK完整配置
 */
struct TiltOptions {
    ZUPTConfig zupt;
    WorkflowConfig workflow;
    LeverArmConfig leverarm;
    DebugConfig debug;
};

/**
 * @brief 轨迹点结构（用于航向对准）
 */
struct TrajectoryPoint {
    double time;
    Eigen::Vector3d pos_enu;    // ENU坐标系下的位置
};

/**
 * @brief ZUPT检测结果
 */
struct ZUPTResult {
    bool zupt_detected = false;     // ZUPT是否被触发
    bool imu_static = false;        // IMU是否静止
    bool gnss_static = false;       // GNSS是否静止
    double zupt_duration = 0.0;      // ZUPT持续时间
};

/**
 * @brief 航向对准结果
 */
struct HeadingAlignResult {
    bool success = false;           // 对准是否成功
    double heading_correction = 0.0; // 航向校正量 [rad]
    double correlation = 0.0;        // INS与RTK轨迹相关系数
    double gnss_traj_length = 0.0;  // GNSS轨迹长度 [m]
};

#endif // TILT_RTK_TYPES_H
