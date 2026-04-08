/*
 * KFGINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 * Extended for NMEA IMU data format by AlgoTech
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

#ifndef IMUNMEALOADER_H
#define IMUNMEALOADER_H

#include <fstream>
#include <string>

#include "common/types.h"

/**
 * ImuNmeaLoader: 解析 $AIMU 格式的 NMEA IMU 数据文件
 *
 * $AIMU 格式:
 * $AIMU,<time_sync>,<time>,<yaw>,<pitch>,<roll>,<yaw_gyro>,<pitch_gyro>,<roll_gyro>,
 *       <forward_accel>,<right_accel>,<down_accel>,<temperature>*<checksum>
 *
 * 字段说明:
 *   time_sync: 时间同步状态 (2 = 与GNSS时间同步)
 *   time: IMU系统时间 (GPS时间秒数)
 *   yaw/pitch/roll: 方位角 (deg)
 *   yaw_gyro/pitch_gyro/roll_gyro: 角速度 (deg/s), 对应 FRD 坐标系的 F/R/D 轴
 *   forward/right/down_accel: 比力 (m/s^2), 对应 FRD 坐标系
 *
 * 输出 IMU 结构体:
 *   time: GPS 时间秒数
 *   dt: 采样间隔 (s)
 *   dtheta: 角度增量 (rad), FRD -> [yaw_gyro, pitch_gyro, roll_gyro] * dt
 *   dvel: 速度增量 (m/s), FRD -> [forward, right, down]_accel
 *
 * 注意: 只保留 time_sync == 2 的数据行
 */
class ImuNmeaLoader {

public:
    ImuNmeaLoader() = default;
    explicit ImuNmeaLoader(const std::string &filename);

    bool open(const std::string &filename);
    bool isOpen() const;
    bool isEof() const;
    void close();

    const IMU &next();

    double starttime();
    double endtime();

private:
    std::ifstream filefp_;
    IMU imu_, imu_pre_;
};

#endif // IMUNMEALOADER_H
