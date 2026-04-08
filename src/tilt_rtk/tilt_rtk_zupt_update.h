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
 * 其中 z = v_tip (杆尖速度，ZUPT时应为0)
 *
 * Copyright (C) 2024
 *
 * [修复记录]
 * - 2024: 严格推导姿态偏导数H1，使用分步计算增强可读性
 *          H1 = skew(v_l) 其中 v_l = C^b_n * (ω_ib × l^b)
 */

#ifndef TILT_RTK_ZUPT_UPDATE_H
#define TILT_RTK_ZUPT_UPDATE_H

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>

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
 * 观测矩阵H的结构（论文公式19）：
 * H_zupt = | 0_3x3    I_3     H1     0_3      H2  |
 *
 * 其中：
 * - H_pos: 位置误差对杆尖速度的偏导数 (0_3x3)
 * - H_vel: 速度误差对杆尖速度的偏导数 (I_3)
 * - H1: 姿态角误差对杆尖速度的偏导数
 * - H_bias: 陀螺零偏对杆尖速度的偏导数
 * - H2: 陀螺比例因子误差对杆尖速度的偏导数
 *
 * 杆尖速度公式（论文公式14）：
 * v^n_T = v^n_I - [(ω^n_ie×) + (ω^n_en×)] * C^b_n * l^b - C^b_n * (l^b×) * ω^b_ib
 */
class ZUPTUpdater {
public:
    // 地球自转角速度 [rad/s]
    static constexpr double WGS84_WIE = 7.2921151467e-5;

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
     * @brief 计算ZUPT观测矩阵和残差（完整版，包含地球自转）
     *
     * 论文公式(14): v^n_T = v^n_I - [(ω^n_ie×) + (ω^n_en×)] * C^b_n * l^b - C^b_n * (l^b×) * ω^b_ib
     *
     * 观测矩阵H（论文公式19-21）：
     * H_zupt = | H_pos, H_vel, H_att, H_bias, H_scale |
     *          | 0_3x3    I_3     H1     0_3       H2  |
     *
     * @param imu_vel IMU速度（导航系）[m/s]
     * @param gyro_rate 陀螺角速度（载体系）[rad/s]
     * @param cbn 载体到导航系旋转矩阵
     * @param lever_arm 杆臂向量 [m]
     * @param latitude 纬度 [rad]（用于计算地球自转ω_ie）
     *
     * @return ZUPT观测结构
     */
    struct ZUPTObservation {
        Eigen::Vector3d zupt_residual;    // 观测残差（杆尖速度）
        Eigen::MatrixXd H;                 // 观测矩阵 (3 x 21)
        Eigen::Matrix3d R;                 // 观测噪声矩阵
    };

    ZUPTObservation computeObservation(
        const Eigen::Vector3d& imu_vel,
        const Eigen::Vector3d& gyro_rate,
        const Eigen::Matrix3d& cbn,
        const Eigen::Vector3d& lever_arm,
        double latitude = 0.0) {

        ZUPTObservation obs;

        // 设置观测噪声
        double noise = options_.measurement_noise;
        obs.R = Eigen::Matrix3d::Identity() * noise * noise;

        // 初始化观测矩阵 (3 x 21)
        obs.H.resize(3, 21);
        obs.H.setZero();

        // ========== 计算杆尖速度（完整公式，论文公式14）==========
        // v^n_T = v^n_I - [(ω^n_ie×) + (ω^n_en×)] * C^b_n * l^b - C^b_n * (l^b×) * ω^b_ib
        //
        // 其中：
        // - ω^n_ie: 地球自转角速度在n系中的投影
        // - ω^n_en: 导航系相对于地球的角速度
        // - ω^b_ib: 陀螺测量的机体角速度

        Eigen::Matrix3d lever_skew = Rotation::skewSymmetric(lever_arm);

        // 计算地球自转角速度在n系的投影: ω^n_ie = [wie*cos(L), 0, -wie*sin(L)]^T
        Eigen::Vector3d wie_n;
        wie_n << WGS84_WIE * std::cos(latitude), 0.0, -WGS84_WIE * std::sin(latitude);
        Eigen::Matrix3d wie_n_skew = Rotation::skewSymmetric(wie_n);

        // ω^n_en 的近似：在静止/低速条件下可以忽略
        // 如果需要精确计算，可以用：wen_n = [v_E/(R_N+h), -v_N/(R_M+h), -v_E*tan(L)/(R_N+h)]^T
        Eigen::Vector3d wen_n = Eigen::Vector3d::Zero();
        Eigen::Matrix3d wen_n_skew = Rotation::skewSymmetric(wen_n);

        // 合并n系角速度: ω_total = ω_ie + ω_en
        Eigen::Matrix3d omega_total_skew = wie_n_skew + wen_n_skew;

        // ========== [修复] 严格推导H1矩阵 ==========
        //
        // 论文公式(14)的物理意义：
        // v^n_T = v^n_I - v^n_omega - v^n_rot
        //
        // 其中：
        // - v^n_omega = [(ω^n_ie×) + (ω^n_en×)] * C^b_n * l^b  （地球自转/导航系旋转导致的杆臂端点速度）
        // - v^n_rot = C^b_n * (l^b×) * ω^b_ib  （杆臂旋转产生的线速度）
        //
        // 对于姿态误差φ对v^n_T的偏导数：
        // 姿态误差φ会导致C^b_n变为(I - φ×)C^b_n
        // 主要影响v^n_rot项中的C^b_n
        //
        // [修复] H1的严格推导：
        // H1 = ∂v^n_T / ∂φ = ∂[C^b_n * (l^b×) * ω^b_ib] / ∂φ
        //     = -(φ×) * C^b_n * (l^b×) * ω^b_ib
        //     = skew(C^b_n * (l^b×) * ω^b_ib)    // 因为 -(φ×)v = v×φ = skew(v)φ
        //
        // 即：H1 = skew(v_l)  其中 v_l = C^b_n * (l^b×) * ω^b_ib
        //
        // 这与代码中的实现一致，但这里分步计算更清晰

        // Step 1: 计算杆臂旋转产生的线速度 v_l = C^b_n * (l^b×) * ω^b_ib
        Eigen::Vector3d v_lever_rot = cbn * lever_skew * gyro_rate;

        // Step 2: 计算地球自转/导航旋转导致的线速度
        // v_omega = (ω_ie + ω_en) × * C_bn * l^b
        Eigen::Vector3d v_omega = omega_total_skew * cbn * lever_arm;

        // Step 3: 计算总杆尖速度
        Eigen::Vector3d v_tip = imu_vel - v_omega - v_lever_rot;

        // 观测残差：ZUPT时杆尖速度应为0
        // z = v_tip - 0 = v_tip
        obs.zupt_residual = v_tip;

        // ========== 构造观测矩阵H（论文公式19-21）==========
        // H_zupt = | H_pos, H_vel, H_att, H_bias, H_scale |
        //          | 0_3x3    I_3     H1     0_3       H2  |

        // 1. 速度误差对观测的偏导数 (I_3)
        // ∂v_tip/∂δv = I_3
        obs.H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        // 2. [修复] 姿态误差对观测的偏导数 (H1)
        // H1 = skew(v_l) = skew(C^b_n * (l^b×) * ω^b_ib)
        // 这是因为 ∂[(I-φ×)v_l]/∂φ = -v_l× = skew(v_l)
        Eigen::Matrix3d H1 = Rotation::skewSymmetric(v_lever_rot);
        obs.H.block<3, 3>(0, 6) = H1;

        // 3. 陀螺零偏对观测的偏导数
        // ∂v_tip/∂ε_g = -C^b_n * (l^b×)
        // 因为 v_l = C^b_n * (l^b×) * ω^b_ib
        // 而 ω^b_ib = ω_true + ε_g
        // 所以 ∂v_l/∂ε_g = C^b_n * (l^b×)
        // 取负号因为残差是 v_tip
        obs.H.block<3, 3>(0, 9) = -cbn * lever_skew;

        // 4. 陀螺比例因子误差对观测的偏导数 (H2, 论文公式21)
        // ∂v_tip/∂δs_g = -C^b_n * (l^b×) * diag(ω_ib)
        // H2 = -C^b_n * (l^b×) * diag(ω_ib)
        Eigen::Matrix3d omega_skew = Rotation::skewSymmetric(gyro_rate);
        obs.H.block<3, 3>(0, 15) = -cbn * lever_skew * omega_skew;

        return obs;
    }

    /**
     * @brief 简化版ZUPT观测（忽略地球自转）
     *
     * 当位置精度要求不高时使用的简化观测模型
     *
     * @param imu_vel IMU速度（导航系）[m/s]
     * @param gyro_rate 陀螺角速度（载体系）[rad/s]
     * @param cbn 载体到导航系旋转矩阵
     * @param lever_arm 杆臂向量 [m]
     *
     * @return ZUPT观测结构
     */
    ZUPTObservation computeSimpleObservation(
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

        // 计算杆臂叉乘矩阵
        Eigen::Matrix3d lever_skew = Rotation::skewSymmetric(lever_arm);

        // 计算杆尖速度（简化版，忽略地球自转）
        Eigen::Vector3d v_tip = imu_vel - cbn * lever_skew * gyro_rate;

        // 观测残差
        obs.zupt_residual = v_tip;

        // ========== 简化观测矩阵H ==========
        // 1. 速度误差偏导数 (I_3)
        obs.H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        // 2. 姿态误差偏导数
        obs.H.block<3, 3>(0, 6) = -cbn * lever_skew;

        // 3. 陀螺零偏偏导数
        obs.H.block<3, 3>(0, 9) = -cbn * lever_skew;

        // 4. 陀螺比例因子误差偏导数
        Eigen::Matrix3d omega_skew = Rotation::skewSymmetric(gyro_rate);
        obs.H.block<3, 3>(0, 15) = -cbn * lever_skew * omega_skew;

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