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

#include "imunmealoader.h"

#include <absl/strings/str_split.h>
#include <iomanip>
#include <sstream>

ImuNmeaLoader::ImuNmeaLoader(const std::string &filename) {
    open(filename);
}

bool ImuNmeaLoader::open(const std::string &filename) {
    filefp_.open(filename, std::ios::in);
    imu_.time = 0;
    return filefp_.is_open();
}

bool ImuNmeaLoader::isOpen() const {
    return filefp_.is_open();
}

bool ImuNmeaLoader::isEof() const {
    return filefp_.eof();
}

void ImuNmeaLoader::close() {
    filefp_.close();
}

const IMU &ImuNmeaLoader::next() {
    imu_pre_ = imu_;

    std::string line;
    while (std::getline(filefp_, line)) {
        // 跳过空行
        if (line.empty()) {
            continue;
        }

        // 跳过不是 $AIMU 开头的行
        if (line.rfind("$AIMU", 0) != 0) {
            continue;
        }

        // 去除帧尾的校验和部分 (*xx)
        auto asterisk_pos = line.find('*');
        if (asterisk_pos != std::string::npos) {
            line = line.substr(0, asterisk_pos);
        }

        // 按逗号分割字段
        std::vector<std::string> fields = absl::StrSplit(line, ',');

        // $AIMU 格式应有 13 个字段: $AIMU,time_sync,time,yaw,pitch,roll,
        //                            yaw_gyro,pitch_gyro,roll_gyro,
        //                            forward_accel,right_accel,down_accel,temp
        if (fields.size() < 13) {
            continue;
        }

        // 解析 time_sync，只处理与 GNSS 时间同步的数据
        unsigned int time_sync = 0;
        std::istringstream(fields[1]) >> time_sync;
        if (time_sync != 2) {
            continue;
        }

        // 解析 IMU 时间 (GPS 时间秒数)
        std::istringstream(fields[2]) >> imu_.time;

        // 解析陀螺仪角速度 (deg/s) -> FRD 坐标系前右下
        // fields[6] = yaw_gyro (F轴)
        // fields[7] = pitch_gyro (R轴)
        // fields[8] = roll_gyro (D轴)
        double yaw_gyro = 0, pitch_gyro = 0, roll_gyro = 0;
        std::istringstream(fields[6]) >> yaw_gyro;
        std::istringstream(fields[7]) >> pitch_gyro;
        std::istringstream(fields[8]) >> roll_gyro;

        // 解析加速度计比力 (m/s^2) -> FRD 坐标系前右下
        // fields[9]  = forward_accel (F轴)
        // fields[10] = right_accel  (R轴)
        // fields[11] = down_accel    (D轴)
        double forward_accel = 0, right_accel = 0, down_accel = 0;
        std::istringstream(fields[9]) >> forward_accel;
        std::istringstream(fields[10]) >> right_accel;
        std::istringstream(fields[11]) >> down_accel;

        // 计算采样间隔 dt
        double dt = imu_.time - imu_pre_.time;
        if (dt <= 0.0 || dt > 0.1) {
            // 异常时间间隔，使用默认值 0.01s (假设 100Hz)
            dt = 0.01;
        }
        imu_.dt = dt;

        // 将角速率转换为角度增量 (deg/s -> rad, 并乘以采样间隔)
        const double D2R = M_PI / 180.0;
        imu_.dtheta[0] = yaw_gyro * D2R * dt;
        imu_.dtheta[1] = pitch_gyro * D2R * dt;
        imu_.dtheta[2] = roll_gyro * D2R * dt;

        // 加速度已经是 m/s^2，直接赋值
        imu_.dvel[0] = forward_accel;
        imu_.dvel[1] = right_accel;
        imu_.dvel[2] = down_accel;

        // odovel 未使用，置零
        imu_.odovel = 0.0;

        return imu_;
    }

    // 文件结束或出错，返回最后一帧
    imu_.dt = 0.0;
    return imu_;
}

double ImuNmeaLoader::starttime() {
    std::streampos sp = filefp_.tellg();
    filefp_.seekg(0, std::ios::beg);

    double start_time = 0.0;
    std::string line;

    while (std::getline(filefp_, line)) {
        if (line.empty() || line.rfind("$AIMU", 0) != 0) {
            continue;
        }

        auto asterisk_pos = line.find('*');
        if (asterisk_pos != std::string::npos) {
            line = line.substr(0, asterisk_pos);
        }

        std::vector<std::string> fields = absl::StrSplit(line, ',');
        if (fields.size() < 13) {
            continue;
        }

        unsigned int time_sync = 0;
        std::istringstream(fields[1]) >> time_sync;
        if (time_sync == 2) {
            std::istringstream(fields[2]) >> start_time;
            break;
        }
    }

    filefp_.seekg(sp, std::ios::beg);
    return start_time;
}

double ImuNmeaLoader::endtime() {
    std::streampos sp = filefp_.tellg();
    filefp_.seekg(0, std::ios::end);

    double end_time = -1.0;
    std::streamoff pos = filefp_.tellg();
    std::string line;

    // 从文件末尾向前搜索，找到最后一个有效的 $AIMU 行
    while (pos > 0) {
        pos -= 1;
        filefp_.seekg(pos);
        char ch = filefp_.get();
        if (ch == '\n') {
            std::getline(filefp_, line);
            if (!line.empty() && line.rfind("$AIMU", 0) == 0) {
                auto asterisk_pos = line.find('*');
                if (asterisk_pos != std::string::npos) {
                    line = line.substr(0, asterisk_pos);
                }

                std::vector<std::string> fields = absl::StrSplit(line, ',');
                if (fields.size() >= 13) {
                    unsigned int time_sync = 0;
                    std::istringstream(fields[1]) >> time_sync;
                    if (time_sync == 2) {
                        std::istringstream(fields[2]) >> end_time;
                        break;
                    }
                }
            }
        }
    }

    filefp_.seekg(sp, std::ios::beg);
    return end_time;
}
