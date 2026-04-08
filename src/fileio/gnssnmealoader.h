/*
 * KFGINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 * Extended for NMEA GNSS data format by AlgoTech
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

#ifndef GNSSNMEALOADER_H
#define GNSSNMEALOADER_H

#include <fstream>
#include <string>

#include "common/types.h"

/**
 * GnssNmeaLoader: 解析 NMEA0183 GNSS 数据文件 ($GNGGA, $GNRMC, $GNGST)
 *
 * 输入文件混合包含以下语句:
 *   $GNGGA: 定位数据 (时间、位置、定位质量、高度)
 *   $GNRMC: 推荐最小定位数据 (时间、状态、速度、航向)
 *   $GNGST: GNSS 伪距误差统计 (时间、精度估计、标准差)
 *
 * KFGINS GNSS 结构体字段:
 *   time: GPS 时间 (s)
 *   blh: 纬度(rad), 经度(rad), 椭球高(m)
 *   std: 纬度标准差(m), 经度标准差(m), 高度标准差(m)
 *   isvalid: 定位是否有效 (fixQuality >= 4, RTK固定解)
 *
 * 输出策略:
 *   每调用一次 next()，读取并组合一整组同时间的 GGA+RMC+GST 语句
 *   解析时间 -> GPS 时间转换 (使用 RMC 中的 UTC 时间 + 日期换算 GPS 周秒)
 *   从 GGA 获取 blh，从 GST 获取 std
 */
class GnssNmeaLoader {

public:
    GnssNmeaLoader() = default;
    explicit GnssNmeaLoader(const std::string &filename);

    bool open(const std::string &filename);
    bool isOpen() const;
    bool isEof() const;
    void close();

    const GNSS &next();

private:
    // 从 UTC 时间 + 日期换算 GPS 时间
    // RMC: time = hhmmss.ss, date = ddmmyy
    // 返回 GPS 时间秒数 (GPS 周内秒)
    // week: 输出参数，传出 GPS Week 编号
    double utcToGpsTime(const std::string &utc_time, const std::string &date, int *week);

    // NMEA 度分格式 (ddmm.mmmm) 转十进制度
    double nmeaAngleToDeg(const std::string &nmea_angle);

    std::ifstream filefp_;
    GNSS gnss_;
    int gnss_week_ = 0; // 当前 GNSS 数据的 GPS Week 编号
};

#endif // GNSSNMEALOADER_H
