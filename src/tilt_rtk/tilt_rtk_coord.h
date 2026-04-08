/*
 * Tilt-RTK: 坐标转换辅助函数
 *
 * 从RTKLIB移植的基础坐标转换函数
 * 参考: RTKLIB 2.4.3 rtkcmn.c
 *
 * Copyright (C) 2024
 */

#ifndef TILT_RTK_COORD_H
#define TILT_RTK_COORD_H

#include <Eigen/Dense>

// RTKLIB常数定义
#define PI 3.1415926535897932
#define D2R (PI / 180.0)
#define R2D (180.0 / PI)
#define RE_WGS84 6378137.0            // WGS84地球长半轴
#define FE_WGS84 (1.0 / 298.257223563) // WGS84地球扁率

namespace CoordUtils {

/**
 * @brief WGS84椭球参数计算
 *
 * @param lat 纬度 [rad]
 * @return Eigen::Vector2d [RN, RM] 子午圈半径和卯酉圈半径
 */
inline Eigen::Vector2d wgs84Radius(double lat) {
    double sinp = sin(lat);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
    return Eigen::Vector2d(v, v); // [RN, RM] - 卯酉圈和子午圈半径
}

/**
 * @brief BLH坐标转换为ENU局部切平面坐标
 *
 * 将BLH坐标（纬度、经度、高程）转换为以给定原点为准的ENU坐标
 * 这是航向对准中轨迹投影的核心函数
 *
 * @param blh BLH坐标 [lat, lon, h] 单位 [rad, rad, m]
 * @param origin 参考原点BLH坐标 [lat, lon, h] 单位 [rad, rad, m]
 * @return Eigen::Vector3d ENU坐标 [E, N, U] 单位 [m, m, m]
 */
inline Eigen::Vector3d blhToEnu(const Eigen::Vector3d& blh, const Eigen::Vector3d& origin) {
    // 计算参考点的XYZ坐标
    double sinp = sin(origin[0]);
    double cosp = cos(origin[0]);
    double sinl = sin(origin[1]);
    double cosl = cos(origin[1]);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    // 原点在ECEF坐标系下的位置
    double x0 = (v + origin[2]) * cosp * cosl;
    double y0 = (v + origin[2]) * cosp * sinl;
    double z0 = (v * (1.0 - e2) + origin[2]) * sinp;

    // 当前点在ECEF坐标系下的位置
    sinp = sin(blh[0]);
    cosp = cos(blh[0]);
    sinl = sin(blh[1]);
    cosl = cos(blh[1]);
    v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    double x = (v + blh[2]) * cosp * cosl;
    double y = (v + blh[2]) * cosp * sinl;
    double z = (v * (1.0 - e2) + blh[2]) * sinp;

    // 计算ENU坐标
    // E = -sin(l0) * (x - x0) + cos(l0) * (y - y0)
    // N = -sin(p0) * cos(l0) * (x - x0) - sin(p0) * sin(l0) * (y - y0) + cos(p0) * (z - z0)
    // U =  cos(p0) * cos(l0) * (x - x0) + cos(p0) * sin(l0) * (y - y0) + sin(p0) * (z - z0)
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    sinp = sin(origin[0]);
    cosp = cos(origin[0]);

    double e = -sinl * dx + cosl * dy;
    double n = -sinp * cosl * dx - sinp * sinl * dy + cosp * dz;
    double u =  cosp * cosl * dx + cosp * sinl * dy + sinp * dz;

    return Eigen::Vector3d(e, n, u);
}

/**
 * @brief 计算两点间的水平距离
 *
 * @param blh1 BLH坐标1 [lat, lon, h]
 * @param blh2 BLH坐标2 [lat, lon, h]
 * @return double 水平距离 [m]
 */
inline double horizontalDistance(const Eigen::Vector3d& blh1, const Eigen::Vector3d& blh2) {
    Eigen::Vector3d enu = blhToEnu(blh2, blh1);
    return sqrt(enu[0] * enu[0] + enu[1] * enu[1]);
}

/**
 * @brief 计算水平面航向角
 *
 * 从点1到点2的水平航向角，以北为0，顺时针为正
 *
 * @param blh1 起点BLH坐标
 * @param blh2 终点BLH坐标
 * @return double 水平航向角 [rad]
 */
inline double horizontalBearing(const Eigen::Vector3d& blh1, const Eigen::Vector3d& blh2) {
    Eigen::Vector3d enu = blhToEnu(blh2, blh1);
    double bearing = atan2(enu[0], enu[1]); // E = x, N = y
    if (bearing < 0) {
        bearing += 2 * PI;
    }
    return bearing;
}

} // namespace CoordUtils

#endif // TILT_RTK_COORD_H
