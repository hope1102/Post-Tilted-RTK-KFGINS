#ifndef ALGO_LASER_H_
#define ALGO_LASER_H_

#include "setting.h"

/// @brief 初始化激光器的参数，主要是和设备的相对位姿关系
/// @param laser_setting 
void laser_init(laser_setting_t *laser_setting);

/// @brief 输入设位置姿态，根据laser_setting参数，计算激光器的位置姿态
/// @param pose 返回激光器的位置姿态
void laser_get_pose(float *dev, float *laser);

/// @brief 输入测距，根据激光器的位置姿态，计算目标点的坐标
/// @param dis 输入测距
/// @param target 返回目标点坐标
void laser_get_target(float dis, float *target);

#endif
