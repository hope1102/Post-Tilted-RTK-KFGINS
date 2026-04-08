/*
 * Tilt-RTK: 核心引擎
 *
 * 整合状态机、ZUPT检测器、航向对准器和ZUPT更新器
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_ENGINE_H
#define TILT_RTK_ENGINE_H

#include <iostream>
#include <iomanip>

#include "tilt_rtk_types.h"
#include "tilt_rtk_config.h"
#include "tilt_rtk_state_machine.h"
#include "tilt_rtk_zupt_detector.h"
#include "tilt_rtk_heading_aligner.h"
#include "tilt_rtk_zupt_update.h"

#include "../kf-gins/gi_engine.h"
#include "../kf-gins/kf_gins_types.h"
#include "../common/rotation.h"

/**
 * @brief 倾斜RTK引擎
 *
 * 整合KFGINS和倾斜RTK扩展功能
 *
 * 工作流程：
 * 1. INIT: 等待GNSS数据
 * 2. GYRO_BIAS_EST: 静止2秒估计陀螺零偏
 * 3. HEADING_ALIGN: 杆尖固定、杆身摆动3秒估计航向误差
 * 4. MEASURE: 测量模式（ZUPT + RTK）
 */
class TiltRTKEngine {
public:
    /**
     * @brief 构造函数
     *
     * @param gins_options KFGINS配置
     * @param tilt_options 倾斜RTK配置
     */
    TiltRTKEngine(const GINSOptions& gins_options, const TiltOptions& tilt_options)
        : gins_options_(gins_options),
          tilt_options_(tilt_options),
          gi_engine_(const_cast<GINSOptions&>(gins_options)),
          state_machine_(tilt_options),
          zupt_detector_(tilt_options.zupt),
          heading_aligner_(tilt_options.workflow),
          zupt_updater_(tilt_options.zupt, tilt_options.leverarm),
          initialized_(false),
          heading_aligned_(false),
          process_start_time_(-1.0) {

        printTiltOptions(tilt_options);
    }

    /**
     * @brief 获取KFGINS引擎
     */
    GIEngine& getGIEngine() {
        return gi_engine_;
    }

    /**
     * @brief 获取倾斜RTK配置
     */
    const TiltOptions& getOptions() const {
        return tilt_options_;
    }

    /**
     * @brief 获取当前状态机
     */
    const TiltRTKStateMachine& getStateMachine() const {
        return state_machine_;
    }

    /**
     * @brief 检查是否完成初始化
     */
    bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief 处理IMU数据
     *
     * @param imu IMU原始数据
     */
    void addImuData(const IMU& imu) {
        double timestamp = imu.time;

        // 记录处理开始时间
        if (process_start_time_ < 0) {
            process_start_time_ = timestamp;
        }

        // 更新状态机
        state_machine_.update(timestamp);

        // 根据当前状态执行不同处理
        TiltState state = state_machine_.getCurrentState();

        switch (state) {
            case TiltState::INIT:
                // 等待GNSS数据，暂不处理IMU
                break;

            case TiltState::GYRO_BIAS_EST:
                // 陀螺零偏估计阶段
                processGyroBiasEst(imu);
                // INS机械编排（不补偿零偏）
                gi_engine_.addImuData(imu, false);
                gi_engine_.newImuProcess();
                break;

            case TiltState::HEADING_ALIGN:
                // 航向对准阶段
                processHeadingAlign(imu);
                // INS机械编排（使用估计的零偏）
                gi_engine_.addImuData(imu, true);
                gi_engine_.newImuProcess();
                break;

            case TiltState::MEASURE:
                // 测量模式
                processMeasureMode(imu);
                // INS机械编排
                gi_engine_.addImuData(imu, true);
                gi_engine_.newImuProcess();
                break;

            case TiltState::ERROR:
            default:
                std::cerr << "[Tilt-RTK] Error state, processing stopped!" << std::endl;
                break;
        }
    }

    /**
     * @brief 处理GNSS数据
     *
     * @param gnss GNSS数据
     */
    void addGnssData(const GNSS& gnss) {
        // 添加到GIEngine
        gi_engine_.addGnssData(gnss);

        // 检查是否应该进入零偏估计阶段
        if (!state_machine_.hasGnssReceived()) {
            state_machine_.setGnssReceived(gnss.time);
        }

        // 在航向对准阶段收集RTK轨迹
        if (state_machine_.getCurrentState() == TiltState::HEADING_ALIGN) {
            heading_aligner_.addRtkTrajectory(gnss.time, gnss.blh);
        }

        // ZUPT检测器处理GNSS数据
        ZUPTResult gnss_result = zupt_detector_.addGnssData(gnss.time, gnss.blh);
        last_gnss_static_ = gnss_result.gnss_static;
    }

    /**
     * @brief 获取当前导航状态
     */
    NavState getNavState() {
        return gi_engine_.getNavState();
    }

    /**
     * @brief 获取当前时间戳
     */
    double getTimestamp() {
        return gi_engine_.timestamp();
    }

    /**
     * @brief 获取状态协方差
     */
    Eigen::MatrixXd getCovariance() {
        return gi_engine_.getCovariance();
    }

    /**
     * @brief 检查ZUPT是否正在触发
     */
    bool isZUPTActive() const {
        return zupt_detector_.isZUPTActive();
    }

private:
    /**
     * @brief 处理陀螺零偏估计阶段
     */
    void processGyroBiasEst(const IMU& imu) {
        // 累积陀螺数据用于零偏估计
        Eigen::Vector3d dvel(imu.dvel);
        Eigen::Vector3d dtheta(imu.dtheta);
        zupt_detector_.addImuData(imu.time, dvel, dtheta, imu.dt);

        // 检查是否完成零偏估计
        double elapsed = state_machine_.getCurrentPhaseElapsed(imu.time);
        if (elapsed >= tilt_options_.workflow.gyro_bias_est_time) {
            // 计算陀螺零偏：零偏 = 累积角度增量 / 累积时间
            double dt_sum = zupt_detector_.getDtSum();
            int sample_count = zupt_detector_.getGyroSampleCount();
            Eigen::Vector3d gyro_bias_acc = zupt_detector_.getGyroBiasAccumulator();

            if (dt_sum > 0 && sample_count > 0) {
                Eigen::Vector3d estimated_gyro_bias = gyro_bias_acc / dt_sum;
                // 应用估计的陀螺零偏到滤波器状态
                gi_engine_.applyGyroBias(estimated_gyro_bias);

                if (tilt_options_.debug.output_state_transitions) {
                    std::cout << "[Tilt-RTK] Gyro bias estimated: "
                              << estimated_gyro_bias.transpose() * R2D * 3600
                              << " [deg/h] (from " << sample_count << " samples, "
                              << dt_sum << "s)" << std::endl;
                }
            }

            // 重置累积器
            zupt_detector_.resetBiasAccumulator();

            // 状态转换到航向对准
            state_machine_.setGyroBiasEstimated(imu.time);

            // 启动航向对准器
            heading_aligner_.start(imu.time);

            std::cout << "[Tilt-RTK] Gyro bias estimation completed. Starting heading alignment." << std::endl;
        }
    }

    /**
     * @brief 处理航向对准阶段
     */
    void processHeadingAlign(const IMU& imu) {
        // 添加INS轨迹
        NavState nav = gi_engine_.getNavState();
        heading_aligner_.addInsTrajectory(imu.time, nav.pos);

        // 检查航向对准是否完成
        double elapsed = state_machine_.getCurrentPhaseElapsed(imu.time);
        if (elapsed >= tilt_options_.workflow.heading_align_time) {
            // 执行航向对准
            HeadingAlignResult result = heading_aligner_.compute();

            if (result.success) {
                // 将航向校正量反馈到INS导航状态
                gi_engine_.applyHeadingCorrection(result.heading_correction);
                heading_aligned_ = true;

                if (tilt_options_.debug.output_heading_align) {
                    std::cout << "[Tilt-RTK] Heading alignment completed:" << std::endl;
                    std::cout << "\t - Heading correction: " << result.heading_correction * 180.0 / M_PI << " [deg]" << std::endl;
                    std::cout << "\t - GNSS trajectory length: " << result.gnss_traj_length << " [m]" << std::endl;
                    std::cout << "\t - Trajectory correlation: " << result.correlation << std::endl;
                }
            } else {
                std::cout << "[Tilt-RTK] Heading alignment failed. Continuing without correction." << std::endl;
            }

            // 停止航向对准
            heading_aligner_.stop();

            // 状态转换到测量模式
            state_machine_.setHeadingAligned(imu.time);

            std::cout << "[Tilt-RTK] Heading alignment completed. Entering MEASURE mode." << std::endl;
        }
    }

    /**
     * @brief 处理测量模式
     */
    void processMeasureMode(const IMU& imu) {
        // ZUPT检测 - IMU静止检测
        ZUPTResult imu_result = zupt_detector_.addImuData(imu.time, imu.dvel, imu.dtheta, imu.dt);

        // 综合IMU和GNSS的检测结果
        ZUPTResult zupt_result = zupt_detector_.updateZUPT(imu_result, ZUPTResult(), imu.time);
        zupt_result.gnss_static = last_gnss_static_;

        // 检查ZUPT触发条件
        if (tilt_options_.zupt.require_both_conditions) {
            zupt_result.zupt_detected = zupt_result.imu_static && zupt_result.gnss_static;
        } else {
            zupt_result.zupt_detected = zupt_result.imu_static || zupt_result.gnss_static;
        }

        // ZUPT持续时间检查
        if (zupt_result.zupt_detected && zupt_result.zupt_duration < tilt_options_.zupt.min_zupt_duration) {
            zupt_result.zupt_detected = false;
        }

        // 如果ZUPT触发，执行量测更新
        if (zupt_result.zupt_detected) {
            performZUPTUpdate(imu);
        }

        // 输出ZUPT状态（可选）
        if (zupt_result.zupt_detected && tilt_options_.debug.output_zupt_status) {
            std::cout << "[Tilt-RTK] ZUPT triggered: duration=" << zupt_result.zupt_duration
                      << "s, IMU_static=" << zupt_result.imu_static
                      << ", GNSS_static=" << zupt_result.gnss_static << std::endl;
        }
    }

    /**
     * @brief 执行ZUPT量测更新
     *
     * 根据论文公式，计算ZUPT观测并执行Kalman滤波器更新
     */
    void performZUPTUpdate(const IMU& imu) {
        // 获取当前状态
        PVA pva = gi_engine_.getPVA();
        ImuError imu_err = gi_engine_.getImuError();

        // 计算当前角速度（从IMU数据）
        Eigen::Vector3d gyro_rate;
        if (imu.dt > 0) {
            gyro_rate = imu.dtheta / imu.dt;
        } else {
            gyro_rate.setZero();
        }

        // 计算ZUPT观测（完整版，包含地球自转）
        // 传递纬度用于计算地球自转角速度
        ZUPTUpdater::ZUPTObservation obs = zupt_updater_.computeObservation(
            pva.vel,
            gyro_rate,
            pva.att.cbn,
            tilt_options_.leverarm.imu_to_tip,
            pva.pos[0]  // 纬度 [rad]
        );

        // 执行Kalman滤波器更新
        gi_engine_.zuptUpdate(obs.zupt_residual, obs.H, obs.R);
    }

private:
    const GINSOptions& gins_options_;
    const TiltOptions& tilt_options_;

    // KFGINS核心引擎
    GIEngine gi_engine_;

    // 倾斜RTK组件
    TiltRTKStateMachine state_machine_;
    ZUPTDetector zupt_detector_;
    HeadingAligner heading_aligner_;
    ZUPTUpdater zupt_updater_;

    // 状态标志
    bool initialized_;
    bool heading_aligned_;
    bool last_gnss_static_;
    double process_start_time_;
};

#endif // TILT_RTK_ENGINE_H
