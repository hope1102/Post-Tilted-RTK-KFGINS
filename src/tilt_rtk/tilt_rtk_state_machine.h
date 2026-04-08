/*
 * Tilt-RTK: 状态机模块
 *
 * 管理倾斜RTK工作流程的各个阶段状态转换
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_STATE_MACHINE_H
#define TILT_RTK_STATE_MACHINE_H

#include <iostream>
#include <iomanip>

#include "tilt_rtk_types.h"
#include "tilt_rtk_config.h"

/**
 * @brief 倾斜RTK状态机
 *
 * 管理整个倾斜RTK系统的工作流程，包括：
 * - 初始化阶段 (INIT)
 * - 陀螺零偏估计阶段 (GYRO_BIAS_EST)
 * - 航向对准阶段 (HEADING_ALIGN)
 * - 测量模式 (MEASURE)
 *
 * 状态转换规则：
 * INIT -> GYRO_BIAS_EST: 收到第一个有效GNSS数据后
 * GYRO_BIAS_EST -> HEADING_ALIGN: 静止满gyro_bias_est_time秒后
 * HEADING_ALIGN -> MEASURE: 摆动满heading_align_time秒后
 * 任何状态 -> ERROR: 检测到错误时
 */
class TiltRTKStateMachine {
public:
    /**
     * @brief 构造函数
     *
     * @param options 倾斜RTK配置参数
     */
    explicit TiltRTKStateMachine(const TiltOptions& options)
        : options_(options),
          current_state_(TiltState::INIT),
          state_start_time_(-1.0),
          gnss_received_(false),
          gyro_bias_estimated_(false),
          heading_aligned_(false) {}

    /**
     * @brief 重置状态机
     */
    void reset() {
        current_state_ = TiltState::INIT;
        state_start_time_ = -1.0;
        gnss_received_ = false;
        gyro_bias_estimated_ = false;
        heading_aligned_ = false;
        last_state_ = TiltState::INIT;
    }

    /**
     * @brief 获取当前状态
     */
    TiltState getCurrentState() const {
        return current_state_;
    }

    /**
     * @brief 获取当前状态名称
     */
    static std::string stateToString(TiltState state) {
        switch (state) {
            case TiltState::INIT:           return "INIT";
            case TiltState::GYRO_BIAS_EST:  return "GYRO_BIAS_EST";
            case TiltState::HEADING_ALIGN:  return "HEADING_ALIGN";
            case TiltState::MEASURE:        return "MEASURE";
            case TiltState::ERROR:          return "ERROR";
            default:                        return "UNKNOWN";
        }
    }

    /**
     * @brief 获取当前状态名称
     */
    std::string getCurrentStateName() const {
        return stateToString(current_state_);
    }

    /**
     * @brief 检查是否已经收到GNSS数据
     */
    bool hasGnssReceived() const {
        return gnss_received_;
    }

    /**
     * @brief 检查是否完成陀螺零偏估计
     */
    bool isGyroBiasEstimated() const {
        return gyro_bias_estimated_;
    }

    /**
     * @brief 检查是否完成航向对准
     */
    bool isHeadingAligned() const {
        return heading_aligned_;
    }

    /**
     * @brief 检查是否处于测量模式
     */
    bool isInMeasureMode() const {
        return current_state_ == TiltState::MEASURE;
    }

    /**
     * @brief 标记已收到GNSS数据并进入零偏估计阶段
     *
     * @param timestamp 当前时间戳
     */
    void setGnssReceived(double timestamp) {
        if (!gnss_received_) {
            gnss_received_ = true;
            transitionTo(TiltState::GYRO_BIAS_EST, timestamp);
        }
    }

    /**
     * @brief 完成陀螺零偏估计，进入航向对准阶段
     *
     * @param timestamp 当前时间戳
     */
    void setGyroBiasEstimated(double timestamp) {
        if (current_state_ == TiltState::GYRO_BIAS_EST) {
            gyro_bias_estimated_ = true;
            transitionTo(TiltState::HEADING_ALIGN, timestamp);
        }
    }

    /**
     * @brief 完成航向对准，进入测量模式
     *
     * @param timestamp 当前时间戳
     */
    void setHeadingAligned(double timestamp) {
        if (current_state_ == TiltState::HEADING_ALIGN) {
            heading_aligned_ = true;
            transitionTo(TiltState::MEASURE, timestamp);
        }
    }

    /**
     * @brief 进入错误状态
     *
     * @param timestamp 当前时间戳
     */
    void setError(double timestamp) {
        transitionTo(TiltState::ERROR, timestamp);
    }

    /**
     * @brief 检查并更新状态（基于时间）
     *
     * 自动检测是否应该进入下一个阶段
     *
     * @param timestamp 当前时间戳
     */
    void update(double timestamp) {
        if (current_state_ == TiltState::INIT) {
            // 等待GNSS数据
            return;
        }

        if (current_state_ == TiltState::GYRO_BIAS_EST) {
            // 检查是否完成零偏估计
            double elapsed = timestamp - state_start_time_;
            if (elapsed >= options_.workflow.gyro_bias_est_time) {
                setGyroBiasEstimated(timestamp);
            }
        }

        if (current_state_ == TiltState::HEADING_ALIGN) {
            // 检查是否完成航向对准
            double elapsed = timestamp - state_start_time_;
            if (elapsed >= options_.workflow.heading_align_time) {
                setHeadingAligned(timestamp);
            }
        }
    }

    /**
     * @brief 获取当前阶段的已运行时间
     *
     * @param timestamp 当前时间戳
     * @return 当前阶段已运行的时间 [秒]
     */
    double getCurrentPhaseElapsed(double timestamp) const {
        if (state_start_time_ < 0) {
            return 0.0;
        }
        return timestamp - state_start_time_;
    }

    /**
     * @brief 获取当前阶段的剩余时间
     *
     * @param timestamp 当前时间戳
     * @return 当前阶段剩余的时间 [秒]
     */
    double getCurrentPhaseRemaining(double timestamp) const {
        double elapsed = getCurrentPhaseElapsed(timestamp);
        double total = 0.0;

        switch (current_state_) {
            case TiltState::GYRO_BIAS_EST:
                total = options_.workflow.gyro_bias_est_time;
                break;
            case TiltState::HEADING_ALIGN:
                total = options_.workflow.heading_align_time;
                break;
            default:
                return 0.0;
        }

        return std::max(0.0, total - elapsed);
    }

    /**
     * @brief 获取工作流程配置
     */
    const TiltOptions& getOptions() const {
        return options_;
    }

private:
    /**
     * @brief 状态转换
     *
     * @param new_state 新状态
     * @param timestamp 当前时间戳
     */
    void transitionTo(TiltState new_state, double timestamp) {
        if (current_state_ != new_state) {
            last_state_ = current_state_;
            current_state_ = new_state;
            state_start_time_ = timestamp;

            if (options_.debug.output_state_transitions) {
                std::cout << "[Tilt-RTK] State transition: "
                          << stateToString(last_state_) << " -> "
                          << stateToString(new_state)
                          << " at t = " << std::fixed << std::setprecision(3)
                          << timestamp << " [s]" << std::endl;
            }
        }
    }

private:
    const TiltOptions& options_;

    TiltState current_state_;
    TiltState last_state_;

    double state_start_time_;    // 当前状态开始时间

    bool gnss_received_;         // 是否已收到GNSS数据
    bool gyro_bias_estimated_;   // 是否已完成陀螺零偏估计
    bool heading_aligned_;       // 是否已完成航向对准
};

#endif // TILT_RTK_STATE_MACHINE_H
