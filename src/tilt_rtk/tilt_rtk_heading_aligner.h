/*
 * Tilt-RTK: 航向对准模块
 *
 * 实现INS-RTK轨迹夹角法估计初始航向误差
 *
 * 原理：
 * 1. 航向对准阶段，杆尖固定不动，杆身摆动
 * 2. INS通过积分推算的轨迹和RTK的轨迹应该反映相同的运动
 * 3. 两者在水平面上的航向夹角即为初始航向误差
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_HEADING_ALIGNER_H
#define TILT_RTK_HEADING_ALIGNER_H

#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>

#include "tilt_rtk_types.h"
#include "tilt_rtk_coord.h"

/**
 * @brief 航向对准器
 *
 * 使用INS轨迹和RTK轨迹的夹角来估计初始航向误差
 *
 * 算法步骤：
 * 1. 收集INS推算的位置轨迹和RTK位置轨迹（使用正确的BLH到ENU转换）
 * 2. 计算各轨迹的航向角
 * 3. 计算两个航向角的夹角
 * 4. 检查轨迹有效性和相关性
 * 5. 返回航向校正量
 */
class HeadingAligner {
public:
    /**
     * @brief 构造函数
     *
     * @param options 工作流程配置参数
     */
    explicit HeadingAligner(const WorkflowConfig& options)
        : options_(options),
          is_active_(false),
          phase_start_time_(-1.0) {
        reset();
    }

    /**
     * @brief 重置对准器
     */
    void reset() {
        ins_traj_.clear();
        rtk_traj_.clear();
        is_active_ = false;
        phase_start_time_ = -1.0;
        last_result_ = HeadingAlignResult();
        ref_pos_ = Eigen::Vector3d::Zero();
        has_ref_ = false;
    }

    /**
     * @brief 启动航向对准
     *
     * @param timestamp 当前时间戳
     */
    void start(double timestamp) {
        reset();
        is_active_ = true;
        phase_start_time_ = timestamp;
    }

    /**
     * @brief 检查是否处于活动状态
     */
    bool isActive() const {
        return is_active_;
    }

    /**
     * @brief 获取阶段开始时间
     */
    double getPhaseStartTime() const {
        return phase_start_time_;
    }

    /**
     * @brief 添加INS轨迹点
     *
     * @param timestamp 时间戳
     * @param pos_ins INS推算的位置 [BLH: rad, rad, m]
     */
    void addInsTrajectory(double timestamp, const Eigen::Vector3d& pos_ins) {
        if (!is_active_) return;

        // 设置参考点（第一个点）
        if (!has_ref_) {
            ref_pos_ = pos_ins;
            has_ref_ = true;
        }

        TrajectoryPoint point;
        point.time = timestamp;
        point.pos_enu = CoordUtils::blhToEnu(pos_ins, ref_pos_);
        ins_traj_.push_back(point);
    }

    /**
     * @brief 添加RTK轨迹点
     *
     * @param timestamp 时间戳
     * @param pos_rtk RTK位置 [BLH: rad, rad, m]
     */
    void addRtkTrajectory(double timestamp, const Eigen::Vector3d& pos_rtk) {
        if (!is_active_) return;

        TrajectoryPoint point;
        point.time = timestamp;
        point.pos_enu = CoordUtils::blhToEnu(pos_rtk, ref_pos_);
        rtk_traj_.push_back(point);
    }

    /**
     * @brief 执行航向对准
     *
     * 计算INS轨迹和RTK轨迹的夹角，返回航向校正量
     *
     * 论文方法：
     * 1. 使用线性回归计算轨迹航向角
     * 2. 计算INS和RTK轨迹的相关系数（论文要求 > 0.8）
     * 3. 计算航向夹角
     *
     * @return 航向对准结果
     */
    HeadingAlignResult compute() {
        HeadingAlignResult result;

        if (!is_active_) {
            return result;
        }

        // 需要足够的轨迹点
        if (ins_traj_.size() < 10 || rtk_traj_.size() < 10) {
            std::cout << "[HeadingAligner] Not enough trajectory points: "
                      << "INS=" << ins_traj_.size()
                      << " RTK=" << rtk_traj_.size() << std::endl;
            return result;
        }

        // 计算轨迹长度（以米为单位）
        double ins_traj_length = computeTrajectoryLength(ins_traj_);
        double rtk_traj_length = computeTrajectoryLength(rtk_traj_);
        result.gnss_traj_length = rtk_traj_length;

        // 检查轨迹长度是否足够（论文要求 > 0.1m）
        if (rtk_traj_length < options_.gnss_traj_length_threshold) {
            std::cout << "[HeadingAligner] RTK trajectory too short: "
                      << rtk_traj_length << " < " << options_.gnss_traj_length_threshold << " m" << std::endl;
            return result;
        }

        if (ins_traj_length < options_.gnss_traj_length_threshold) {
            std::cout << "[HeadingAligner] INS trajectory too short: "
                      << ins_traj_length << " m" << std::endl;
            return result;
        }

        // ========== 计算轨迹相关系数（论文要求 > 0.8） ==========
        result.correlation = computeTrajectoryCorrelation(ins_traj_, rtk_traj_);
        if (result.correlation < options_.min_traj_correlation) {
            std::cout << "[HeadingAligner] Trajectory correlation too low: "
                      << result.correlation << " < " << options_.min_traj_correlation << std::endl;
            return result;
        }

        // ========== 使用线性回归计算轨迹航向角（论文方法） ==========
        double ins_heading = computeHeadingByLinearRegression(ins_traj_);
        double rtk_heading = computeHeadingByLinearRegression(rtk_traj_);

        // 计算航向夹角（INS航向 - RTK航向）
        double heading_diff = ins_heading - rtk_heading;

        // 归一化到 [-PI, PI]
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;

        result.heading_correction = heading_diff;
        result.success = true;
        last_result_ = result;

        return result;
    }

    /**
     * @brief 获取最后计算的对准结果
     */
    const HeadingAlignResult& getLastResult() const {
        return last_result_;
    }

    /**
     * @brief 获取INS轨迹点数
     */
    size_t getInsTrajSize() const {
        return ins_traj_.size();
    }

    /**
     * @brief 获取RTK轨迹点数
     */
    size_t getRtkTrajSize() const {
        return rtk_traj_.size();
    }

    /**
     * @brief 停止航向对准
     */
    void stop() {
        is_active_ = false;
    }

private:
    /**
     * @brief 计算轨迹方向向量（使用首尾点法）
     *
     * @param traj 轨迹点
     * @return 2D方向向量 [E, N]
     */
    Eigen::Vector2d computeTrajectoryDirection(const std::vector<TrajectoryPoint>& traj) {
        if (traj.size() < 2) {
            return Eigen::Vector2d::Zero();
        }

        // 使用起点和终点计算方向
        Eigen::Vector2d start(traj.front().pos_enu[0], traj.front().pos_enu[1]);  // [E, N]
        Eigen::Vector2d end(traj.back().pos_enu[0], traj.back().pos_enu[1]);

        Eigen::Vector2d dir = end - start;

        return dir;
    }

    /**
     * @brief 计算轨迹长度
     *
     * @param traj 轨迹点
     * @return 轨迹总长度 [m]
     */
    double computeTrajectoryLength(const std::vector<TrajectoryPoint>& traj) {
        if (traj.size() < 2) {
            return 0.0;
        }

        double length = 0.0;
        for (size_t i = 1; i < traj.size(); i++) {
            double dx = traj[i].pos_enu[0] - traj[i-1].pos_enu[0]; // E
            double dy = traj[i].pos_enu[1] - traj[i-1].pos_enu[1]; // N
            length += sqrt(dx * dx + dy * dy);
        }

        return length;
    }

    /**
     * @brief 计算方向向量的航向角
     *
     * @param dir 方向向量 [E, N]
     * @return 航向角 [rad], 以北为0, 顺时针增加
     */
    double computeHeadingAngle(const Eigen::Vector2d& dir) {
        double norm = dir.norm();
        if (norm < 1e-6) {
            return 0.0;
        }

        // atan2(x, y) 给出从y轴（北向）顺时针测量的角度
        double heading = atan2(dir[0], dir[1]); // [E, N] -> heading
        return heading;
    }

    /**
     * @brief 使用线性回归计算轨迹航向角（论文方法）
     *
     * 论文公式(5)使用方向余弦法计算航向：
     * cos(Δψ) = (A'B' · A'B'') / (||A'B'|| · ||A'B''||)
     *
     * 这里使用线性回归计算斜率来得到航向角
     *
     * @param traj 轨迹点
     * @return 航向角 [rad]
     */
    double computeHeadingByLinearRegression(const std::vector<TrajectoryPoint>& traj) {
        if (traj.size() < 2) {
            return 0.0;
        }

        // 计算均值
        double x_mean = 0.0, y_mean = 0.0;
        for (const auto& p : traj) {
            x_mean += p.pos_enu[0];  // E
            y_mean += p.pos_enu[1];  // N
        }
        x_mean /= traj.size();
        y_mean /= traj.size();

        // 计算线性回归斜率
        // y = k * x + b
        double numerator = 0.0;
        double denominator = 0.0;
        for (const auto& p : traj) {
            double dx = p.pos_enu[0] - x_mean;
            double dy = p.pos_enu[1] - y_mean;
            numerator += dx * dy;
            denominator += dx * dx;
        }

        if (std::abs(denominator) < 1e-10) {
            return 0.0;
        }

        double slope = numerator / denominator;

        // 将斜率转换为航向角（以北为0，顺时针增加）
        // E = r * sin(heading), N = r * cos(heading)
        // 所以 slope = tan(heading) = E/N
        double heading = atan2(slope, 1.0);
        return heading;
    }

    /**
     * @brief 计算INS和RTK轨迹的相关系数（论文要求 > 0.8）
     *
     * 使用Pearson相关系数评估两条轨迹的线性相关性
     *
     * @param ins_traj INS轨迹
     * @param rtk_traj RTK轨迹
     * @return 相关系数 [-1, 1]
     */
    double computeTrajectoryCorrelation(
        const std::vector<TrajectoryPoint>& ins_traj,
        const std::vector<TrajectoryPoint>& rtk_traj) {

        // 使用时间对齐的轨迹点计算相关系数
        // 将轨迹投影到水平面上，计算E方向和N方向的相关系数
        if (ins_traj.size() != rtk_traj.size() || ins_traj.size() < 3) {
            return 0.0;
        }

        // 计算E方向相关系数
        double e_mean_ins = 0.0, e_mean_rtk = 0.0;
        double n_mean_ins = 0.0, n_mean_rtk = 0.0;

        for (size_t i = 0; i < ins_traj.size(); i++) {
            e_mean_ins += ins_traj[i].pos_enu[0];
            e_mean_rtk += rtk_traj[i].pos_enu[0];
            n_mean_ins += ins_traj[i].pos_enu[1];
            n_mean_rtk += rtk_traj[i].pos_enu[1];
        }
        e_mean_ins /= ins_traj.size();
        e_mean_rtk /= rtk_traj.size();
        n_mean_ins /= ins_traj.size();
        n_mean_rtk /= rtk_traj.size();

        // Pearson相关系数
        double sum_e = 0.0, sum_e_ins_sq = 0.0, sum_e_rtk_sq = 0.0;
        double sum_n = 0.0, sum_n_ins_sq = 0.0, sum_n_rtk_sq = 0.0;

        for (size_t i = 0; i < ins_traj.size(); i++) {
            double e_ins_diff = ins_traj[i].pos_enu[0] - e_mean_ins;
            double e_rtk_diff = rtk_traj[i].pos_enu[0] - e_mean_rtk;
            sum_e += e_ins_diff * e_rtk_diff;
            sum_e_ins_sq += e_ins_diff * e_ins_diff;
            sum_e_rtk_sq += e_rtk_diff * e_rtk_diff;

            double n_ins_diff = ins_traj[i].pos_enu[1] - n_mean_ins;
            double n_rtk_diff = rtk_traj[i].pos_enu[1] - n_mean_rtk;
            sum_n += n_ins_diff * n_rtk_diff;
            sum_n_ins_sq += n_ins_diff * n_ins_diff;
            sum_n_rtk_sq += n_rtk_diff * n_rtk_diff;
        }

        // 避免除零
        if (sum_e_ins_sq < 1e-10 || sum_e_rtk_sq < 1e-10 ||
            sum_n_ins_sq < 1e-10 || sum_n_rtk_sq < 1e-10) {
            return 0.0;
        }

        double corr_e = sum_e / std::sqrt(sum_e_ins_sq * sum_e_rtk_sq);
        double corr_n = sum_n / std::sqrt(sum_n_ins_sq * sum_n_rtk_sq);

        // 取两个方向的平均相关系数
        return (corr_e + corr_n) / 2.0;
    }

private:
    const WorkflowConfig& options_;

    bool is_active_;
    double phase_start_time_;

    std::vector<TrajectoryPoint> ins_traj_;
    std::vector<TrajectoryPoint> rtk_traj_;

    HeadingAlignResult last_result_;

    // 参考位置（轨迹的原点）
    Eigen::Vector3d ref_pos_;
    bool has_ref_;
};

#endif // TILT_RTK_HEADING_ALIGNER_H
