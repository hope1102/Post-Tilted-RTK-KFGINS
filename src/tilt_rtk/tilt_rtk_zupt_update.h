/*
 * Tilt-RTK: ZUPT量测更新模块
 *
 * 实现基于杆尖速度为零的ZUPT量测更新
 *
 * 原理：
 * 当ZUPT触发时（杆尖静止），杆尖速度为0
 * 通过IMU速度、角速度和杆臂向量计算杆尖速度
 * 将杆尖速度作为虚拟观测引入Kalman滤波器
 *
 * 观测方程: z = H * x + v
 * 其中 z = 0 - v_tip = -v_tip (杆尖速度)
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_ZUPT_UPDATE_H
#define TILT_RTK_ZUPT_UPDATE_H

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

#include "tilt_rtk_types.h"
#include "../common/rotation.h"

/**
 * @brief ZUPT量测更新器
 *
 * 实现ZUPT约束的Kalman滤波器量测更新
 *
 * 观测模型（论文公式18-21）：
 * z_v = v^n_T - v^m_T
 *
 * 其中：
 * - v^n_T: 杆尖在n系中的速度
 * - v^m_T: 测量得到的杆尖速度（ZUPT时为0）
 *
 * 观测矩阵H的结构：
 * H_zupt = [H_pos, H_vel, H_att, H_bias, H_scale]
 *
 * 其中包含：
 * - H_pos: 位置误差对杆尖速度的偏导数
 * - H_vel: 速度误差对杆尖速度的偏导数 (I_3)
 * - H_att: 姿态误差对杆尖速度的偏导数
 * - H_bias: 陀螺零偏对杆尖速度的偏导数
 * - H_scale: 陀螺比例因子对杆尖速度的偏导数
 */
class ZUPTUpdater {
public:
    /**
     * @brief 构造函数
     *
     * @param options ZUPT配置参数
     * @param leverarm 杆臂配置参数
     */
    ZUPTUpdater(const ZUPTConfig& options, const LeverArmConfig& leverarm)
        : options_(options),
          leverarm_(leverarm) {}

    /**
     * @brief 计算ZUPT观测矩阵和残差
     *
     * 基于论文的杆尖速度观测模型
     *
     * @param imu_vel IMU速度（导航系）[m/s]
     * @param imu_gyro 陀螺角速度（载体系）[rad/s]
     * @param cbn 载体到导航系旋转矩阵
     * @param pos 当前位置 [BLH]
     * @param lever_arm 杆臂向量（IMU到杆尖）[m]
     *
     * @return 包含观测信息的结构体
     */
    struct ZUPTObservation {
        Eigen::Vector3d zupt_residual;    // 观测残差（杆尖速度）
        Eigen::MatrixXd H;                 // 观测矩阵 (3 x 21)
        Eigen::Matrix3d R;                 // 观测噪声矩阵
    };

    /**
     * @brief 计算ZUPT观测
     *
     * @param imu_vel IMU速度（导航系）[m/s]
     * @param gyro_rate 陀螺角速度（载体系）[rad/s]
     * @param cbn 载体到导航系旋转矩阵
     * @param lever_arm 杆臂向量 [m]
     *
     * @return ZUPT观测结构
     */
    ZUPTObservation computeObservation(
        const Eigen::Vector3d& imu_vel,
        const Eigen::Vector3d& gyro_rate,
        const Eigen::Matrix3d& cbn,
        const Eigen::Vector3d& lever_arm) {

        ZUPTObservation obs;

        // 设置观测噪声
        double noise = options_.measurement_noise;
        obs.R = Eigen::Matrix3d::Identity() * noise * noise;

        // 初始化观测矩阵 (3 x 21)
        obs.H.resize(3, 21);
        obs.H.setZero();

        // ========== 计算杆尖速度 ==========
        // 论文公式(14): v^n_T = v^n_I - [(ω^n_ie×) + (ω^n_en×)] * C^b_n * l^b - C^b_n * (l^b×) * ω^b_ib
        //
        // 简化版本（忽略地球自转和导航系旋转）：
        // v^n_T ≈ v^n_I - C^b_n * (l^b×) * ω^b_ib
        //
        // 其中 (l^b×) 是杆臂向量的叉乘矩阵

        // 计算杆臂叉乘矩阵
        Eigen::Matrix3d lever_skew = Rotation::skewSymmetric(lever_arm);

        // 计算杆尖速度
        Eigen::Vector3d v_tip = imu_vel - cbn * lever_skew * gyro_rate;

        // 观测残差：ZUPT时杆尖速度应为0
        // z = v_tip - 0 = v_tip
        obs.zupt_residual = v_tip;

        // ========== 构造观测矩阵H ==========
        // H矩阵对各个误差状态的偏导数

        // 1. 速度误差对观测的偏导数 (I_3)
        // ∂v_tip/∂δv = I_3
        obs.H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        // 2. 姿态误差对观测的偏导数
        // ∂v_tip/∂φ = -C^b_n * (l^b×)
        // 这是论文公式中姿态误差传播的影响
        obs.H.block<3, 3>(0, 6) = -cbn * lever_skew;

        // 3. 陀螺零偏对观测的偏导数
        // ∂v_tip/∂ε_g = -C^b_n * (l^b×)
        // 陀螺零偏产生的角速度误差直接贡献到杆尖速度
        obs.H.block<3, 3>(0, 9) = -cbn * lever_skew;

        // 4. 陀螺比例因子误差对观测的偏导数
        // ∂v_tip/∂δs_g = -C^b_n * (l^b×) * diag(ω)
        // 比例因子误差与角速度的乘积
        Eigen::Matrix3d omega_skew = Rotation::skewSymmetric(gyro_rate);
        obs.H.block<3, 3>(0, 15) = -cbn * lever_skew * omega_skew;

        return obs;
    }

    /**
     * @brief 简化版ZUPT观测（仅使用速度和姿态）
     *
     * 当只关注位置和姿态误差时使用的简化观测模型
     *
     * @param imu_vel IMU速度（导航系）[m/s]
     * @param gyro_rate 陀螺角速度（载体系）[rad/s]
     * @param cbn 载体到导航系旋转矩阵
     *
     * @return ZUPT观测结构
     */
    ZUPTObservation computeSimpleObservation(
        const Eigen::Vector3d& imu_vel,
        const Eigen::Vector3d& gyro_rate,
        const Eigen::Matrix3d& cbn) {

        ZUPTObservation obs;

        // 设置观测噪声
        double noise = options_.measurement_noise;
        obs.R = Eigen::Matrix3d::Identity() * noise * noise;

        // 初始化观测矩阵 (3 x 21)
        obs.H.resize(3, 21);
        obs.H.setZero();

        // 计算杆臂叉乘矩阵
        Eigen::Matrix3d lever_skew = Rotation::skewSymmetric(leverarm_.imu_to_tip);

        // 计算杆尖速度
        Eigen::Vector3d v_tip = imu_vel - cbn * lever_skew * gyro_rate;

        // 观测残差
        obs.zupt_residual = v_tip;

        // ========== 简化观测矩阵H ==========
        // 仅考虑速度和姿态的影响

        // 1. 速度误差偏导数 (I_3)
        obs.H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        // 2. 姿态误差偏导数
        obs.H.block<3, 3>(0, 6) = -cbn * lever_skew;

        // 3. 陀螺零偏偏导数
        obs.H.block<3, 3>(0, 9) = -cbn * lever_skew;

        return obs;
    }

    /**
     * @brief 获取观测噪声矩阵
     */
    Eigen::Matrix3d getMeasurementNoise() const {
        double noise = options_.measurement_noise;
        return Eigen::Matrix3d::Identity() * noise * noise;
    }

    /**
     * @brief 获取杆臂向量
     */
    const Eigen::Vector3d& getLeverArm() const {
        return leverarm_.imu_to_tip;
    }

private:
    const ZUPTConfig& options_;
    const LeverArmConfig& leverarm_;
};

#endif // TILT_RTK_ZUPT_UPDATE_H
