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

#include "gnssnmealoader.h"

#include <absl/strings/str_split.h>
#include <cmath>

namespace {

// GPS 起始时间: 1980年1月6日 00:00:00 UTC
// 与 RTKLIB rtkcmn.c 中的 gpst0[] 保持一致
constexpr double GPST0[6] = {1980, 1, 6, 0, 0, 0};

// 闰秒表 (y, m, d, h, min, s, utc-gpst)，与 RTKLIB rtkcmn.c 中的 leaps[][] 保持一致
// 截至 2026 年 1 月: UTC - GPST = -18s
constexpr double LEAPS[][7] = {
    {2017, 1, 1, 0, 0, 0, -18},
    {2015, 7, 1, 0, 0, 0, -17},
    {2012, 7, 1, 0, 0, 0, -16},
    {2009, 1, 1, 0, 0, 0, -15},
    {2006, 1, 1, 0, 0, 0, -14},
    {1999, 1, 1, 0, 0, 0, -13},
    {1997, 7, 1, 0, 0, 0, -12},
    {1996, 1, 1, 0, 0, 0, -11},
    {1994, 7, 1, 0, 0, 0, -10},
    {1993, 7, 1, 0, 0, 0,  -9},
    {1992, 7, 1, 0, 0, 0,  -8},
    {1991, 1, 1, 0, 0, 0,  -7},
    {1990, 1, 1, 0, 0, 0,  -6},
    {1988, 1, 1, 0, 0, 0,  -5},
    {1985, 7, 1, 0, 0, 0,  -4},
    {1983, 7, 1, 0, 0, 0,  -3},
    {1982, 7, 1, 0, 0, 0,  -2},
    {1981, 7, 1, 0, 0, 0,  -1},
    {0}
};

// --------------------------------------------------------------------------
// epoch2time: 将日历时间 {year,month,day,hour,min,sec} 转换为 time_t (UTC 秒数)
// 源自 RTKLIB rtkcmn.c epoch2time()
// 有效范围: 1970-2099
// --------------------------------------------------------------------------
time_t epoch2time(const double ep[6]) {
    static const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    int days;
    int sec;
    int year  = static_cast<int>(ep[0]);
    int month = static_cast<int>(ep[1]);
    int day   = static_cast<int>(ep[2]);

    if (year < 1970 || 2099 < year || month < 1 || 12 < month) {
        return 0;
    }

    // 1970-2099 期间: 闰年判定 year%4==0 (2000年是世纪年但也是闰年)
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[month - 1] + day - 2 +
           ((year % 4 == 0 && month >= 3) ? 1 : 0);
    sec = static_cast<int>(floor(ep[5]));
    return static_cast<time_t>(days) * 86400 +
           static_cast<int>(ep[3]) * 3600 +
           static_cast<int>(ep[4]) * 60 + sec;
}

// --------------------------------------------------------------------------
// time2gpst: 将 UTC time_t 转换为 GPS 周内秒 (从 1980/1/6 起算)
// 源自 RTKLIB rtkcmn.c time2gpst()
// 注意: 返回值仅为 TOW (0~604799.999s)，GPS Week 通过 out 参数 week 传出
// --------------------------------------------------------------------------
double time2gpst(time_t utc_time, double utc_sec_frac, int *week) {
    double ep0[6] = {GPST0[0], GPST0[1], GPST0[2], GPST0[3], GPST0[4], GPST0[5]};
    time_t t0 = epoch2time(ep0);

    double total_sec = static_cast<double>(utc_time - t0) + utc_sec_frac;
    int w = static_cast<int>(total_sec / (86400.0 * 7.0));

    // C++ static_cast<int> 对负数向零取整，与 floor 行为不同
    // 例如: -0.1 / 7 = -0.014...，向零取整 = 0，但实际应为 -1
    // RTKLIB 源码中对这种情况做 week-- 补偿
    if (total_sec < 0.0) {
        w--;
    }

    if (week) {
        *week = w;
    }
    return total_sec - static_cast<double>(w) * 86400.0 * 7.0;
}

// --------------------------------------------------------------------------
// timeadd: 给 gtime_t 增加指定秒数
// 源自 RTKLIB rtkcmn.c timeadd()
// --------------------------------------------------------------------------
void timeadd(time_t &t, double &sec_frac, double add_sec) {
    sec_frac += add_sec;
    double tt = floor(sec_frac);
    t += static_cast<time_t>(tt);
    sec_frac -= tt;
}

// --------------------------------------------------------------------------
// timediff: 计算两个 time_t 之间的时间差 (秒)
// 源自 RTKLIB rtkcmn.c timediff()
// --------------------------------------------------------------------------
double timediff(time_t t1, double sec1, time_t t2, double sec2) {
    return static_cast<double>(t1 - t2) + sec1 - sec2;
}

// --------------------------------------------------------------------------
// utc2gpst: 将 UTC 日历时间转换为 GPS 周内秒
// 源自 RTKLIB rtkcmn.c utc2gpst()
// 逻辑: 先构建 UTC gtime_t，再通过闰秒表查表减去对应闰秒得到 GPST
// 注意: 返回值仅为 TOW (0~604799.999s)，GPS Week 通过 out 参数 week 传出
// --------------------------------------------------------------------------
double utc2gpst(int year, int month, int day, int hour, int min, double sec, int *week) {
    double ep_utc[6] = {
        static_cast<double>(year),
        static_cast<double>(month),
        static_cast<double>(day),
        static_cast<double>(hour),
        static_cast<double>(min),
        sec
    };

    time_t t_utc = epoch2time(ep_utc);
    double sec_frac = ep_utc[5] - floor(ep_utc[5]);

    // 遍历闰秒表，找到适用的闰秒值
    for (int i = 0; LEAPS[i][0] > 0; i++) {
        double ep_leap[6] = {LEAPS[i][0], LEAPS[i][1], LEAPS[i][2],
                             LEAPS[i][3], LEAPS[i][4], LEAPS[i][5]};
        time_t t_leap = epoch2time(ep_leap);

        // 如果当前 UTC 时间 >= 闰秒生效时间，则 UTC - 闰秒 = GPST
        // 关键: 在连续 time_t 上补偿闰秒，再调用 time2gpst()，避免周跨边界 TOW 溢出
        if (timediff(t_utc, sec_frac, t_leap, 0.0) >= 0.0) {
            double leap_sec = LEAPS[i][6]; // 负值，如 -18
            time_t t_gpst = t_utc;
            double frac_gpst = sec_frac;
            timeadd(t_gpst, frac_gpst, -leap_sec); // UTC - 闰秒 = GPST
            return time2gpst(t_gpst, frac_gpst, week); // week 通过 out 参数传出
        }
    }
    return 0.0;
}

// --------------------------------------------------------------------------
// ascii2dbl: 将 ASCII 字符串转换为 double，不依赖 Locale
// 源自 RTKLIB rtkcmn.c str2num()
// 使用 ASCII 直接解析，兼容所有语言环境的 NMEA 小数点格式
// --------------------------------------------------------------------------
double ascii2dbl(const char *s) {
    double value = 0.0;
    int sign = 1;
    double scale = 1.0;
    bool after_dot = false;

    if (*s == '-') {
        sign = -1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while (*s) {
        if (*s == '.') {
            after_dot = true;
            s++;
            continue;
        }
        if (*s >= '0' && *s <= '9') {
            value = value * 10.0 + (*s - '0');
            if (after_dot) {
                scale *= 10.0;
            }
        } else {
            break;
        }
        s++;
    }

    return sign * value / scale;
}

// --------------------------------------------------------------------------
// ascii2int: 将 ASCII 字符串转换为 int，不依赖 Locale
// --------------------------------------------------------------------------
int ascii2int(const char *s) {
    int value = 0;
    int sign = 1;

    if (*s == '-') {
        sign = -1;
        s++;
    }

    while (*s && *s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        s++;
    }

    return sign * value;
}

// --------------------------------------------------------------------------
// parseNmeaTime: 解析 hhmmss.ss 格式时间，返回 (hour, min, sec)
// --------------------------------------------------------------------------
void parseNmeaTime(const std::string &time_str, int &hour, int &min, double &sec) {
    hour = 0;
    min  = 0;
    sec  = 0.0;

    if (time_str.size() < 6) {
        return;
    }

    hour = ascii2int(time_str.substr(0, 2).c_str());
    min  = ascii2int(time_str.substr(2, 2).c_str());
    sec  = ascii2dbl(time_str.substr(4).c_str());
}

// --------------------------------------------------------------------------
// parseNmeaDate: 解析 ddmmyy 格式日期
// 参照 RTKLIB 惯例: yy < 80 -> 2000+yy, 否则 -> 1900+yy
// 这样可以正确处理 1980 年以前的数据（虽然 NMEA 不太可能出现）
// --------------------------------------------------------------------------
bool parseNmeaDate(const std::string &date_str, int &year, int &month, int &day) {
    if (date_str.size() < 6) {
        return false;
    }

    day   = ascii2int(date_str.substr(0, 2).c_str());
    month = ascii2int(date_str.substr(2, 2).c_str());

    int yy = ascii2int(date_str.substr(4, 2).c_str());
    year = (yy < 80) ? (2000 + yy) : (1900 + yy);

    return true;
}

} // anonymous namespace

GnssNmeaLoader::GnssNmeaLoader(const std::string &filename) {
    open(filename);
}

bool GnssNmeaLoader::open(const std::string &filename) {
    filefp_.open(filename, std::ios::in);
    return filefp_.is_open();
}

bool GnssNmeaLoader::isOpen() const {
    return filefp_.is_open();
}

bool GnssNmeaLoader::isEof() const {
    return filefp_.eof();
}

void GnssNmeaLoader::close() {
    filefp_.close();
}

double GnssNmeaLoader::utcToGpsTime(const std::string &utc_time_str,
                                     const std::string &date_str, int *week) {
    int year = 0, month = 0, day = 0;
    if (!parseNmeaDate(date_str, year, month, day)) {
        return 0.0;
    }

    int hour = 0;
    int min  = 0;
    double sec = 0.0;
    parseNmeaTime(utc_time_str, hour, min, sec);

    // 调用 RTKLIB 算法计算 GPS 周内秒
    return utc2gpst(year, month, day, hour, min, sec, week);
}

double GnssNmeaLoader::nmeaAngleToDeg(const std::string &nmea_angle) {
    if (nmea_angle.empty()) {
        return 0.0;
    }

    double val = ascii2dbl(nmea_angle.c_str()); // 不依赖 Locale
    int deg = static_cast<int>(val / 100.0);
    double min_part = val - deg * 100.0;
    return static_cast<double>(deg) + min_part / 60.0;
}

const GNSS &GnssNmeaLoader::next() {
    gnss_.isvalid = false;
    gnss_.time = 0.0;
    gnss_.std.setZero(); // 每次读取新历元前清空

    std::string line;
    int mask = 0;

    // 工业级流式解析法：发现同类型语句重现即为跨历元，使用 seekg 回退指针
    while (true) {
        // 记录当前行首的文件指针位置，发现跨历元时用于退回
        std::streampos current_pos = filefp_.tellg();

        if (!std::getline(filefp_, line)) {
            break; // 到达文件末尾
        }

        if (line.empty()) continue;

        auto asterisk_pos = line.find('*');
        if (asterisk_pos != std::string::npos) {
            line = line.substr(0, asterisk_pos);
        }

        if (line.rfind("$GNGGA", 0) == 0 || line.rfind("$GPGGA", 0) == 0) {
            // 如果当前历元已经解析过 GGA，说明这行属于下一秒，回退并结束
            if (mask & 1) {
                filefp_.seekg(current_pos);
                break;
            }

            std::vector<std::string> fields = absl::StrSplit(line, ',');
            if (fields.size() >= 10) {
                double lat = nmeaAngleToDeg(fields[2]);
                if (fields[3] == "S") lat = -lat;
                gnss_.blh[0] = lat * M_PI / 180.0;

                double lon = nmeaAngleToDeg(fields[4]);
                if (fields[5] == "W") lon = -lon;
                gnss_.blh[1] = lon * M_PI / 180.0;

                unsigned int fix_quality = static_cast<unsigned int>(ascii2dbl(fields[6].c_str()));
                gnss_.isvalid = (fix_quality >= 4);

                gnss_.blh[2] = ascii2dbl(fields[9].c_str());
                mask |= 1;
            }
        } else if (line.rfind("$GNRMC", 0) == 0 || line.rfind("$GPRMC", 0) == 0) {
            // 如果当前历元已经解析过 RMC，说明这行属于下一秒，回退并结束
            if (mask & 2) {
                filefp_.seekg(current_pos);
                break;
            }

            std::vector<std::string> fields = absl::StrSplit(line, ',');
            if (fields.size() >= 10) {
                gnss_.time = utcToGpsTime(fields[1], fields[9], &gnss_week_);

                // 注意: gnss_week_ 保存了当前历元的 GPS Week 编号
                // 如果后端组合导航算法仅使用时间差 dt（相邻历元间的时间间隔），
                // 则此变量暂未使用。如需计算完整 GPS 时间（week * 604800 + tow），
                // 请使用: full_gps_time = gnss_week_ * 604800.0 + gnss_.time;

                mask |= 2;
            }
        } else if (line.rfind("$GNGST", 0) == 0 || line.rfind("$GPGST", 0) == 0) {
            // 如果当前历元已经解析过 GST，说明这行属于下一秒，回退并结束
            if (mask & 4) {
                filefp_.seekg(current_pos);
                break;
            }

            std::vector<std::string> fields = absl::StrSplit(line, ',');
            if (fields.size() >= 9) {
                gnss_.std[0] = ascii2dbl(fields[6].c_str());
                gnss_.std[1] = ascii2dbl(fields[7].c_str());
                gnss_.std[2] = ascii2dbl(fields[8].c_str());
                mask |= 4;
            }
        }

        // GGA、RMC、GST 全部集齐，提前安全跳出
        if (mask == 7) {
            break;
        }
    }

    return gnss_;
}
