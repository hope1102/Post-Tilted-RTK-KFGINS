/*
 * Tilt-RTK: 配置解析模块
 *
 * 解析tilt-rtk.yaml配置文件
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_CONFIG_H
#define TILT_RTK_CONFIG_H

#include <yaml-cpp/yaml.h>

#include "tilt_rtk_types.h"

/**
 * @brief 解析倾斜RTK专用配置
 *
 * @param config YAML配置节点
 * @param options 输出：倾斜RTK配置参数
 */
inline void loadTiltConfig(const YAML::Node& config, TiltOptions& options) {
    // 解析工作流程配置
    if (config["workflow"]) {
        auto& wf = config["workflow"];

        if (wf["gyro_bias_est_time"]) {
            options.workflow.gyro_bias_est_time = wf["gyro_bias_est_time"].as<double>();
        }
        if (wf["heading_align_time"]) {
            options.workflow.heading_align_time = wf["heading_align_time"].as<double>();
        }
        if (wf["gnss_traj_length_threshold"]) {
            options.workflow.gnss_traj_length_threshold = wf["gnss_traj_length_threshold"].as<double>();
        }
        if (wf["min_traj_correlation"]) {
            options.workflow.min_traj_correlation = wf["min_traj_correlation"].as<double>();
        }
    }

    // 解析ZUPT检测器配置
    if (config["zupt"]) {
        auto& zupt = config["zupt"];

        if (zupt["acc_threshold"]) {
            options.zupt.acc_threshold = zupt["acc_threshold"].as<double>();
        }
        if (zupt["acc_window"]) {
            options.zupt.acc_window = zupt["acc_window"].as<int>();
        }
        if (zupt["gyro_threshold"]) {
            options.zupt.gyro_threshold = zupt["gyro_threshold"].as<double>();
        }
        if (zupt["gyro_window"]) {
            options.zupt.gyro_window = zupt["gyro_window"].as<int>();
        }
        if (zupt["gnss_vel_threshold"]) {
            options.zupt.gnss_vel_threshold = zupt["gnss_vel_threshold"].as<double>();
        }
        if (zupt["gnss_window"]) {
            options.zupt.gnss_window = zupt["gnss_window"].as<int>();
        }
        if (zupt["require_both_conditions"]) {
            options.zupt.require_both_conditions = zupt["require_both_conditions"].as<bool>();
        }
        if (zupt["min_zupt_duration"]) {
            options.zupt.min_zupt_duration = zupt["min_zupt_duration"].as<double>();
        }
        if (zupt["measurement_noise"]) {
            options.zupt.measurement_noise = zupt["measurement_noise"].as<double>();
        }
    }

    // 解析杆臂配置
    if (config["leverarm"]) {
        auto& lever = config["leverarm"];

        if (lever["imu_to_tip"]) {
            auto lever_vec = lever["imu_to_tip"].as<std::vector<double>>();
            options.leverarm.imu_to_tip = Eigen::Vector3d(lever_vec[0], lever_vec[1], lever_vec[2]);
        }
    }

    // 解析调试输出配置
    if (config["debug"]) {
        auto& dbg = config["debug"];

        if (dbg["output_state_transitions"]) {
            options.debug.output_state_transitions = dbg["output_state_transitions"].as<bool>();
        }
        if (dbg["output_zupt_status"]) {
            options.debug.output_zupt_status = dbg["output_zupt_status"].as<bool>();
        }
        if (dbg["output_heading_align"]) {
            options.debug.output_heading_align = dbg["output_heading_align"].as<bool>();
        }
    }
}

/**
 * @brief 打印倾斜RTK配置参数
 */
inline void printTiltOptions(const TiltOptions& options) {
    std::cout << "---------------Tilt-RTK Options:---------------" << std::endl;

    // 工作流程配置
    std::cout << " - Workflow: " << std::endl;
    std::cout << "\t - Gyro bias estimation time: " << options.workflow.gyro_bias_est_time << " [s]" << std::endl;
    std::cout << "\t - Heading alignment time: " << options.workflow.heading_align_time << " [s]" << std::endl;
    std::cout << "\t - GNSS trajectory threshold: " << options.workflow.gnss_traj_length_threshold << " [m]" << std::endl;
    std::cout << "\t - Min trajectory correlation: " << options.workflow.min_traj_correlation << std::endl;

    // ZUPT配置
    std::cout << " - ZUPT Detector: " << std::endl;
    std::cout << "\t - Acceleration threshold: " << options.zupt.acc_threshold << " [m/s²]" << std::endl;
    std::cout << "\t - Acceleration window: " << options.zupt.acc_window << " [samples]" << std::endl;
    std::cout << "\t - Gyroscope threshold: " << options.zupt.gyro_threshold << " [deg/s]" << std::endl;
    std::cout << "\t - Gyroscope window: " << options.zupt.gyro_window << " [samples]" << std::endl;
    std::cout << "\t - GNSS velocity threshold: " << options.zupt.gnss_vel_threshold << " [m/s]" << std::endl;
    std::cout << "\t - GNSS window: " << options.zupt.gnss_window << " [samples]" << std::endl;
    std::cout << "\t - Require both conditions: " << (options.zupt.require_both_conditions ? "Yes" : "No") << std::endl;
    std::cout << "\t - Min ZUPT duration: " << options.zupt.min_zupt_duration << " [s]" << std::endl;
    std::cout << "\t - Measurement noise: " << options.zupt.measurement_noise << " [m/s]" << std::endl;

    // 杆臂配置
    std::cout << " - Lever Arm: " << std::endl;
    std::cout << "\t - IMU to tip: " << options.leverarm.imu_to_tip.transpose() << " [m]" << std::endl;

    // 调试配置
    std::cout << " - Debug: " << std::endl;
    std::cout << "\t - Output state transitions: " << (options.debug.output_state_transitions ? "Yes" : "No") << std::endl;
    std::cout << "\t - Output ZUPT status: " << (options.debug.output_zupt_status ? "Yes" : "No") << std::endl;
    std::cout << "\t - Output heading align: " << (options.debug.output_heading_align ? "Yes" : "No") << std::endl;

    std::cout << std::endl;
}

#endif // TILT_RTK_CONFIG_H
