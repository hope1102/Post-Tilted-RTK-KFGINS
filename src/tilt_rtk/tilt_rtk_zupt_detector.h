/*
 * Tilt-RTK: ZUPT检测器模块
 *
 * 实现IMU+GNSS双重检测的零速更新触发机制
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_ZUPT_DETECTOR_H
#define TILT_RTK_ZUPT_DETECTOR_H

#include <vector>
#include <iostream>
#include <iomanip>

#include "tilt_rtk_types.h"
#include "../common/angle.h"

/**
 * @brief ZUPT检测器
 *
 * 使用IMU加速度方差和角速度阈值 + GNSS速度阈值
 * 双重条件同时满足时才触发ZUPT
 */
class ZUPTDetector {
public:
    /**
     * @brief 构造函数
     *
     * @param options ZUPT配置参数
     */
    explicit ZUPTDetector(const ZUPTConfig& options)
        : options_(options),
          zupt_triggered_(false),
          zupt_start_time_(-1.0),
          total_zupt_time_(0.0) {
        reset();
    }

    /**
     * @brief 重置检测器
     */
    void reset() {
        // 清空IMU滑动窗口
        acc_buffer_.clear();
        gyro_buffer_.clear();

        // 清空GNSS位置缓冲区
        gnss_pos_buffer_.clear();
        gnss_time_buffer_.clear();
        last_gnss_time_ = -1.0;

        // 重置ZUPT状态
        zupt_triggered_ = false;
        zupt_start_time_ = -1.0;
        total_zupt_time_ = 0.0;

        // 清空累积的加速度和角速度（用于零偏估计）
        acc_sum_.setZero();
        gyro_sum_.setZero();
        gyro_sample_count_ = 0;
        dt_sum_ = 0.0;
    }

    /**
     * @brief 添加IMU数据并检测静止
     *
     * @param timestamp 当前时间戳 [秒]
     * @param dvel IMU速度增量 [m/s]
     * @param dtheta IMU角度增量 [rad]
     * @param dt IMU采样间隔 [秒]（用于零偏估计）
     *
     * @return ZUPT检测结果
     */
    ZUPTResult addImuData(double timestamp, const Eigen::Vector3d& dvel, const Eigen::Vector3d& dtheta, double dt) {
        ZUPTResult result;

        // 转换为double并添加到缓冲区
        Eigen::Vector3d acc = dvel;  // 使用dvel作为"加速度"的代理
        acc_buffer_.push_back(acc);
        gyro_buffer_.push_back(dtheta);

        // 保持窗口大小
        if (acc_buffer_.size() > static_cast<size_t>(options_.acc_window)) {
            acc_buffer_.erase(acc_buffer_.begin());
        }
        if (gyro_buffer_.size() > static_cast<size_t>(options_.gyro_window)) {
            gyro_buffer_.erase(gyro_buffer_.begin());
        }

        // IMU静止检测
        result.imu_static = checkImuStatic();

        // 累积用于零偏估计（仅在零偏估计阶段使用）
        if (gyro_sample_count_ >= 0) {
            gyro_sum_ += dtheta;
            dt_sum_ += dt;
            gyro_sample_count_++;
        }

        return result;
    }

    /**
     * @brief 添加IMU原始数据（原始加速度和角速度）
     *
     * @param timestamp 当前时间戳 [秒]
     * @param acc 原始加速度 [m/s²]
     * @param gyro 原始角速度 [rad/s]
     * @param dt IMU采样间隔 [秒]（用于零偏估计）
     *
     * @return ZUPT检测结果
     */
    ZUPTResult addImuRawData(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) {
        ZUPTResult result;

        // 添加到缓冲区
        acc_buffer_.push_back(acc);
        gyro_buffer_.push_back(gyro);

        // 保持窗口大小
        if (acc_buffer_.size() > static_cast<size_t>(options_.acc_window)) {
            acc_buffer_.erase(acc_buffer_.begin());
        }
        if (gyro_buffer_.size() > static_cast<size_t>(options_.gyro_window)) {
            gyro_buffer_.erase(gyro_buffer_.begin());
        }

        // IMU静止检测
        result.imu_static = checkImuStatic();

        // 累积用于零偏估计
        gyro_sum_ += gyro * dt;
        dt_sum_ += dt;
        gyro_sample_count_++;

        return result;
    }

    /**
     * @brief 添加GNSS数据并检测静止
     *
     * @param timestamp GNSS时间戳 [秒]
     * @param pos GNSS位置 [BLH: rad, rad, m]
     *
     * @return 更新后的静止检测结果
     */
    ZUPTResult addGnssData(double timestamp, const Eigen::Vector3d& pos) {
        ZUPTResult result;

        // 先添加到缓冲区（确保back()能取到上一帧位置）
        gnss_pos_buffer_.push_back(pos);
        gnss_time_buffer_.push_back(timestamp);
        last_gnss_time_ = timestamp;

        // 保持窗口大小（添加后再裁剪，保证至少有一帧历史数据）
        if (gnss_pos_buffer_.size() > static_cast<size_t>(options_.gnss_window + 1)) {
            gnss_pos_buffer_.erase(gnss_pos_buffer_.begin());
            gnss_time_buffer_.erase(gnss_time_buffer_.begin());
        }

        // 计算GNSS速度（位置差分）：使用前一个位置和当前位置
        if (gnss_pos_buffer_.size() >= 2) {
            double dt = gnss_time_buffer_.back() - gnss_time_buffer_.at(gnss_time_buffer_.size() - 2);
            if (dt > 0) {
                Eigen::Vector3d vel = (gnss_pos_buffer_.back() - gnss_pos_buffer_.at(gnss_pos_buffer_.size() - 2)) / dt;
                gnss_vel_buffer_.push_back(vel);
            }
        }

        // 保持GNSS速度缓冲区大小
        if (gnss_vel_buffer_.size() > static_cast<size_t>(options_.gnss_window)) {
            gnss_vel_buffer_.erase(gnss_vel_buffer_.begin());
        }

        // GNSS静止检测
        result.gnss_static = checkGnssStatic();

        return result;
    }

    /**
     * @brief 更新ZUPT状态（基于IMU和GNSS的检测结果）
     *
     * @param imu_result IMU静止检测结果
     * @param gnss_result GNSS静止检测结果
     * @param timestamp 当前时间戳 [秒]
     *
     * @return 综合后的ZUPT检测结果
     */
    ZUPTResult updateZUPT(const ZUPTResult& imu_result, const ZUPTResult& gnss_result, double timestamp) {
        ZUPTResult result;

        result.imu_static = imu_result.imu_static;
        result.gnss_static = gnss_result.gnss_static;

        // 双重条件判断
        if (options_.require_both_conditions) {
            result.zupt_detected = result.imu_static && result.gnss_static;
        } else {
            result.zupt_detected = result.imu_static || result.gnss_static;
        }

        // 跟踪ZUPT持续时间
        if (result.zupt_detected) {
            if (!zupt_triggered_) {
                zupt_start_time_ = timestamp;
                zupt_triggered_ = true;
            }
            result.zupt_duration = timestamp - zupt_start_time_;
        } else {
            if (zupt_triggered_) {
                total_zupt_time_ += timestamp - zupt_start_time_;
            }
            zupt_triggered_ = false;
            zupt_start_time_ = -1.0;
            result.zupt_duration = 0.0;
        }

        return result;
    }

    /**
     * @brief 检查ZUPT是否正在触发
     */
    bool isZUPTActive() const {
        return zupt_triggered_;
    }

    /**
     * @brief 获取总ZUPT时间
     */
    double getTotalZUPTTime() const {
        return total_zupt_time_;
    }

    /**
     * @brief 获取累积的陀螺零偏估计
     *
     * @return 累积的陀螺角度增量（需要除以时间得到角速度零偏）
     */
    Eigen::Vector3d getGyroBiasAccumulator() const {
        return gyro_sum_;
    }

    /**
     * @brief 获取累积的陀螺样本数
     */
    int getGyroSampleCount() const {
        return gyro_sample_count_;
    }

    /**
     * @brief 获取累积的时间间隔（用于零偏估计）
     */
    double getDtSum() const {
        return dt_sum_;
    }

    /**
     * @brief 重置零偏估计累积值（零偏估计阶段完成后调用）
     */
    void resetBiasAccumulator() {
        gyro_sum_.setZero();
        gyro_sample_count_ = 0;
        dt_sum_ = 0.0;
    }

private:
    /**
     * @brief 检查IMU是否静止（加速度方差 + 角速度阈值）
     */
    bool checkImuStatic() {
        // 需要足够的样本
        if (acc_buffer_.size() < 3 || gyro_buffer_.size() < 3) {
            return false;
        }

        // 计算加速度方差
        double acc_variance = calculateVariance(acc_buffer_);

        // 计算角速度幅值的平均值
        double gyro_mag_mean = 0.0;
        for (const auto& gyro : gyro_buffer_) {
            gyro_mag_mean += gyro.norm();
        }
        gyro_mag_mean /= gyro_buffer_.size();

        // 双重条件：加速度方差小 AND 角速度小
        bool acc_static = (acc_variance < options_.acc_threshold * options_.acc_threshold);
        bool gyro_static = (gyro_mag_mean < options_.gyro_threshold * D2R);  // 转换为rad/s

        return acc_static && gyro_static;
    }

    /**
     * @brief 检查GNSS是否静止（速度阈值）
     */
    bool checkGnssStatic() {
        // 需要足够的样本
        if (gnss_vel_buffer_.size() < 3) {
            return false;
        }

        // 计算平均GNSS速度
        double vel_mag_mean = 0.0;
        for (const auto& vel : gnss_vel_buffer_) {
            vel_mag_mean += vel.norm();
        }
        vel_mag_mean /= gnss_vel_buffer_.size();

        return (vel_mag_mean < options_.gnss_vel_threshold);
    }

    /**
     * @brief 计算向量的方差
     */
    double calculateVariance(const std::vector<Eigen::Vector3d>& data) {
        if (data.empty()) return 0.0;

        // 计算均值
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& v : data) {
            mean += v;
        }
        mean /= data.size();

        // 计算方差
        double variance = 0.0;
        for (const auto& v : data) {
            Eigen::Vector3d diff = v - mean;
            variance += diff.dot(diff);
        }
        variance /= data.size();

        return variance;
    }

private:
    const ZUPTConfig& options_;

    // IMU数据缓冲区
    std::vector<Eigen::Vector3d> acc_buffer_;
    std::vector<Eigen::Vector3d> gyro_buffer_;

    // GNSS数据缓冲区
    std::vector<Eigen::Vector3d> gnss_pos_buffer_;
    std::vector<double> gnss_time_buffer_;
    std::vector<Eigen::Vector3d> gnss_vel_buffer_;
    double last_gnss_time_;

    // ZUPT状态
    bool zupt_triggered_;
    double zupt_start_time_;
    double total_zupt_time_;

    // 零偏估计累积值
    Eigen::Vector3d acc_sum_;
    Eigen::Vector3d gyro_sum_;
    int gyro_sample_count_;
    double dt_sum_;  // 累积的时间间隔（用于零偏估计）
};

#endif // TILT_RTK_ZUPT_DETECTOR_H
