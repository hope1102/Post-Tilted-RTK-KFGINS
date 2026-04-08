/*
 * Tilt-RTK: 主程序入口
 *
 * 基于KF-GINS的倾斜RTK扩展程序
 * 支持ZUPT（零速更新）和快速初始化
 *
 * Copyright (C) 2024
 */

#include <Eigen/Dense>
#include <absl/time/clock.h>
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "common/angle.h"
#include "fileio/filesaver.h"
#include "fileio/gnssnmealoader.h"
#include "fileio/imunmealoader.h"

#include "kf-gins/gi_engine.h"
#include "kf-gins/kf_gins_types.h"

#include "tilt_rtk/tilt_rtk_config.h"
#include "tilt_rtk/tilt_rtk_engine.h"

bool loadGINSConfig(YAML::Node& config, GINSOptions& options);
void writeNavResult(double time, NavState& navstate, FileSaver& navfile, FileSaver& imuerrfile);
void writeSTD(double time, Eigen::MatrixXd& cov, FileSaver& stdfile);
void writeTiltResult(double time, TiltState state, ZUPTResult& zupt_result, FileSaver& tiltfile);

int main(int argc, char* argv[]) {

    if (argc != 2) {
        std::cout << "Usage: Tilt-RTK tilt-rtk.yaml" << std::endl;
        std::cout << "Example: ./Tilt-RTK ../config/tilt-rtk.yaml" << std::endl;
        return -1;
    }

    std::cout << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "     Tilt-RTK: Tilted RTK-GNSS/INS Navigation System    " << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << std::endl;

    auto ts = absl::Now();

    // 加载配置文件
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception& exception) {
        std::cout << "Failed to read configuration file. Please check the path and format!" << std::endl;
        return -1;
    }

    // 读取KFGINS配置参数
    GINSOptions options;
    if (!loadGINSConfig(config, options)) {
        std::cout << "Error occurs in the KFGINS configuration!" << std::endl;
        return -1;
    }

    // 读取Tilt-RTK专用配置
    TiltOptions tilt_options;
    loadTiltConfig(config, tilt_options);

    // 读取文件路径配置
    std::string imupath, gnsspath, outputpath;
    try {
        imupath    = config["imupath"].as<std::string>();
        gnsspath   = config["gnsspath"].as<std::string>();
        outputpath = config["outputpath"].as<std::string>();
    } catch (YAML::Exception& exception) {
        std::cout << "Failed when loading file paths!" << std::endl;
        return -1;
    }

    // IMU数据配置
    int imudatalen, imudatarate;
    double starttime, endtime;
    try {
        imudatalen  = config["imudatalen"].as<int>();
        imudatarate = config["imudatarate"].as<int>();
        starttime   = config["starttime"].as<double>();
        endtime     = config["endtime"].as<double>();
    } catch (YAML::Exception& exception) {
        std::cout << "Failed when loading data configuration!" << std::endl;
        return -1;
    }

    // 加载数据文件
    GnssNmeaLoader gnssfile(gnsspath);
    ImuNmeaLoader imufile(imupath);

    // 构造Tilt-RTK引擎
    TiltRTKEngine tilt_engine(options, tilt_options);

    // 构造输出文件
    int nav_columns = 11, imuerr_columns = 13, std_columns = 22, tilt_columns = 6;
    FileSaver navfile(outputpath + "/Tilt_RTK_Navresult.nav", nav_columns, FileSaver::TEXT);
    FileSaver imuerrfile(outputpath + "/Tilt_RTK_IMU_ERR.txt", imuerr_columns, FileSaver::TEXT);
    FileSaver stdfile(outputpath + "/Tilt_RTK_STD.txt", std_columns, FileSaver::TEXT);
    FileSaver tiltfile(outputpath + "/Tilt_RTK_State.txt", tilt_columns, FileSaver::TEXT);

    // 检查文件是否正确打开
    if (!gnssfile.isOpen() || !imufile.isOpen() || !navfile.isOpen() ||
        !imuerrfile.isOpen() || !stdfile.isOpen() || !tiltfile.isOpen()) {
        std::cout << "Failed to open data file!" << std::endl;
        return -1;
    }

    // 检查处理时间
    if (endtime < 0) {
        endtime = imufile.endtime();
    }
    // 首先确保 starttime >= 0
    if (starttime < 0) {
        starttime = 0;
    }
    // 然后检查其他条件
    if (endtime > 604800 || starttime < imufile.starttime() || starttime > endtime) {
        std::cout << "Process time ERROR!" << std::endl;
        std::cout << "  - endtime: " << endtime << std::endl;
        std::cout << "  - imufile.starttime(): " << imufile.starttime() << std::endl;
        return -1;
    }

    // 数据对齐
    IMU imu_cur;
    do {
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time <= starttime);

    // 添加初始数据
    tilt_engine.addGnssData(gnss);

    // 用于保存处理结果
    double timestamp;
    NavState navstate;
    Eigen::MatrixXd cov;
    ZUPTResult zupt_result;
    zupt_result.zupt_detected = false;
    zupt_result.imu_static = false;
    zupt_result.gnss_static = false;
    zupt_result.zupt_duration = 0.0;

    // 用于显示处理进程
    int percent = 0, lastpercent = 0;
    double interval = endtime - starttime;

    std::cout << "Processing starts..." << std::endl;
    std::cout << "======================================================" << std::endl;

    while (true) {
        // 加载新的GNSS数据
        if (gnss.time < imu_cur.time && !gnssfile.isEof()) {
            gnss = gnssfile.next();
            tilt_engine.addGnssData(gnss);
        }

        // 加载新的IMU数据
        imu_cur = imufile.next();
        if (imu_cur.time > endtime || imufile.isEof()) {
            break;
        }
        tilt_engine.addImuData(imu_cur);

        // 获取当前状态
        timestamp = tilt_engine.getTimestamp();
        navstate = tilt_engine.getNavState();
        cov = tilt_engine.getCovariance();

        // 获取当前工作状态
        TiltState state = tilt_engine.getStateMachine().getCurrentState();

        // 保存处理结果
        writeNavResult(timestamp, navstate, navfile, imuerrfile);
        writeSTD(timestamp, cov, stdfile);
        writeTiltResult(timestamp, state, zupt_result, tiltfile);

        // 显示处理进展
        percent = int((imu_cur.time - starttime) / interval * 100);
        if (percent - lastpercent >= 1) {
            std::cout << "\r - Processing: " << std::setw(3) << percent
                      << "%  |  State: " << TiltRTKStateMachine::stateToString(state)
                      << "  |  ZUPT: " << (zupt_result.zupt_detected ? "ON" : "OFF")
                      << "     " << std::flush;
            lastpercent = percent;
        }
    }

    std::cout << std::endl;
    std::cout << "======================================================" << std::endl;

    // 关闭文件
    imufile.close();
    gnssfile.close();
    navfile.close();
    imuerrfile.close();
    stdfile.close();
    tiltfile.close();

    // 处理完毕
    auto te = absl::Now();
    std::cout << std::endl << "Tilt-RTK Process Finish!" << std::endl;
    std::cout << "From " << starttime << " s to " << endtime << " s, total " << interval << " s!" << std::endl;
    std::cout << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;
    std::cout << std::endl;

    return 0;
}

/**
 * @brief 加载KFGINS配置
 */
bool loadGINSConfig(YAML::Node& config, GINSOptions& options) {

    // 读取初始状态
    std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;
    try {
        vec1 = config["initpos"].as<std::vector<double>>();
        vec2 = config["initvel"].as<std::vector<double>>();
        vec3 = config["initatt"].as<std::vector<double>>();
    } catch (YAML::Exception& exception) {
        std::cout << "Failed when loading initial state!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.pos[i]    = vec1[i] * D2R;
        options.initstate.vel[i]    = vec2[i];
        options.initstate.euler[i] = vec3[i] * D2R;
    }
    options.initstate.pos[2] *= R2D;

    // 读取IMU误差初始值
    vec1 = {0.0, 0.0, 0.0};
    vec2 = {0.0, 0.0, 0.0};
    vec3 = {0.0, 0.0, 0.0};
    vec4 = {0.0, 0.0, 0.0};
    try {
        vec1 = config["initgyrbias"].as<std::vector<double>>();
        vec2 = config["initaccbias"].as<std::vector<double>>();
        vec3 = config["initgyrscale"].as<std::vector<double>>();
        vec4 = config["initaccscale"].as<std::vector<double>>();
    } catch (YAML::Exception& exception) {
        // 使用默认值（全零）
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.imuerror.gyrbias[i]  = vec1[i] * D2R / 3600.0;
        options.initstate.imuerror.accbias[i]  = vec2[i] * 1e-5;
        options.initstate.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    // 读取初始状态标准差
    vec1 = {0.1, 0.1, 0.2};
    vec2 = {0.05, 0.05, 0.05};
    vec3 = {0.5, 0.5, 1.0};
    try {
        vec1 = config["initposstd"].as<std::vector<double>>();
        vec2 = config["initvelstd"].as<std::vector<double>>();
        vec3 = config["initattstd"].as<std::vector<double>>();
    } catch (YAML::Exception& exception) {
        // 使用默认值
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.pos[i]    = vec1[i];
        options.initstate_std.vel[i]    = vec2[i];
        options.initstate_std.euler[i] = vec3[i] * D2R;
    }

    // 读取IMU噪声参数
    vec1 = {0.24, 0.24, 0.24};
    vec2 = {0.24, 0.24, 0.24};
    vec3 = {50.0, 50.0, 50.0};
    vec4 = {250.0, 250.0, 250.0};
    vec5 = {1000.0, 1000.0, 1000.0};
    vec6 = {1000.0, 1000.0, 1000.0};
    try {
        vec1 = config["imunoise"]["arw"].as<std::vector<double>>();
        vec2 = config["imunoise"]["vrw"].as<std::vector<double>>();
        vec3 = config["imunoise"]["gbstd"].as<std::vector<double>>();
        vec4 = config["imunoise"]["abstd"].as<std::vector<double>>();
        vec5 = config["imunoise"]["gsstd"].as<std::vector<double>>();
        vec6 = config["imunoise"]["asstd"].as<std::vector<double>>();
        options.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    } catch (YAML::Exception& exception) {
        // 使用默认值
        options.imunoise.corr_time = 3600.0;
    }
    for (int i = 0; i < 3; i++) {
        options.imunoise.gyr_arw[i]     = vec1[i];
        options.imunoise.acc_vrw[i]     = vec2[i];
        options.imunoise.gyrbias_std[i]  = vec3[i];
        options.imunoise.accbias_std[i]  = vec4[i];
        options.imunoise.gyrscale_std[i] = vec5[i];
        options.imunoise.accscale_std[i] = vec6[i];
    }

    // IMU噪声参数转换
    options.imunoise.gyr_arw *= (D2R / 60.0);
    options.imunoise.acc_vrw /= 60.0;
    options.imunoise.gyrbias_std *= (D2R / 3600.0);
    options.imunoise.accbias_std *= 1e-5;
    options.imunoise.gyrscale_std *= 1e-6;
    options.imunoise.accscale_std *= 1e-6;
    options.imunoise.corr_time *= 3600;

    // 读取IMU误差初始标准差
    // 使用IMU噪声参数中的零偏和比例因子的标准差作为默认值
    vec1 = {options.imunoise.gyrbias_std.x() / (D2R / 3600.0),
            options.imunoise.gyrbias_std.y() / (D2R / 3600.0),
            options.imunoise.gyrbias_std.z() / (D2R / 3600.0)};
    vec2 = {options.imunoise.accbias_std.x() / 1e-5,
            options.imunoise.accbias_std.y() / 1e-5,
            options.imunoise.accbias_std.z() / 1e-5};
    vec3 = {options.imunoise.gyrscale_std.x() / 1e-6,
            options.imunoise.gyrscale_std.y() / 1e-6,
            options.imunoise.gyrscale_std.z() / 1e-6};
    vec4 = {options.imunoise.accscale_std.x() / 1e-6,
            options.imunoise.accscale_std.y() / 1e-6,
            options.imunoise.accscale_std.z() / 1e-6};
    try {
        vec1 = config["initbgstd"].as<std::vector<double>>();
        vec2 = config["initbastd"].as<std::vector<double>>();
        vec3 = config["initsgstd"].as<std::vector<double>>();
        vec4 = config["initsastd"].as<std::vector<double>>();
    } catch (YAML::Exception& exception) {
        // 使用默认值
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.imuerror.gyrbias[i]  = vec1[i] * D2R / 3600.0;
        options.initstate_std.imuerror.accbias[i]  = vec2[i] * 1e-5;
        options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    // GNSS天线杆臂
    try {
        vec1 = config["antlever"].as<std::vector<double>>();
    } catch (YAML::Exception& exception) {
        vec1 = {0.0, 0.0, 0.0};
    }
    options.antlever = Eigen::Vector3d(vec1.data());

    return true;
}

/**
 * @brief 保存导航结果
 */
void writeNavResult(double time, NavState& navstate, FileSaver& navfile, FileSaver& imuerrfile) {
    std::vector<double> result;

    result.clear();
    result.push_back(0);
    result.push_back(time);
    result.push_back(navstate.pos[0] * R2D);
    result.push_back(navstate.pos[1] * R2D);
    result.push_back(navstate.pos[2]);
    result.push_back(navstate.vel[0]);
    result.push_back(navstate.vel[1]);
    result.push_back(navstate.vel[2]);
    result.push_back(navstate.euler[0] * R2D);
    result.push_back(navstate.euler[1] * R2D);
    result.push_back(navstate.euler[2] * R2D);
    navfile.dump(result);

    auto imuerr = navstate.imuerror;
    result.clear();
    result.push_back(time);
    result.push_back(imuerr.gyrbias[0] * R2D * 3600);
    result.push_back(imuerr.gyrbias[1] * R2D * 3600);
    result.push_back(imuerr.gyrbias[2] * R2D * 3600);
    result.push_back(imuerr.accbias[0] * 1e5);
    result.push_back(imuerr.accbias[1] * 1e5);
    result.push_back(imuerr.accbias[2] * 1e5);
    result.push_back(imuerr.gyrscale[0] * 1e6);
    result.push_back(imuerr.gyrscale[1] * 1e6);
    result.push_back(imuerr.gyrscale[2] * 1e6);
    result.push_back(imuerr.accscale[0] * 1e6);
    result.push_back(imuerr.accscale[1] * 1e6);
    result.push_back(imuerr.accscale[2] * 1e6);
    imuerrfile.dump(result);
}

/**
 * @brief 保存标准差
 */
void writeSTD(double time, Eigen::MatrixXd& cov, FileSaver& stdfile) {
    std::vector<double> result;

    result.clear();
    result.push_back(time);

    for (int i = 0; i < 6; i++) {
        result.push_back(sqrt(cov(i, i)));
    }
    for (int i = 6; i < 9; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D);
    }

    for (int i = 9; i < 12; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D * 3600);
    }
    for (int i = 12; i < 15; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e5);
    }
    for (int i = 15; i < 21; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e6);
    }

    stdfile.dump(result);
}

/**
 * @brief 保存倾斜RTK状态信息
 */
void writeTiltResult(double time, TiltState state, ZUPTResult& zupt_result, FileSaver& tiltfile) {
    std::vector<double> result;

    result.clear();
    result.push_back(time);
    result.push_back(static_cast<double>(state));  // 状态ID
    result.push_back(zupt_result.zupt_detected ? 1.0 : 0.0);  // ZUPT触发
    result.push_back(zupt_result.imu_static ? 1.0 : 0.0);      // IMU静止
    result.push_back(zupt_result.gnss_static ? 1.0 : 0.0);     // GNSS静止
    result.push_back(zupt_result.zupt_duration);               // ZUPT持续时间

    tiltfile.dump(result);
}
