#ifndef ALGO_CAM_H
#define ALGO_CAM_H

#include "setting.h"

/// @brief 初始化相机内参，外参，畸变参数
/// @param cam_setting
void cam_init(cam_setting_t *cam_setting);

/// @brief 计算相机的位置姿态
/// @param dev 输入设备的位置姿态
/// @param cam 返回相机的位置姿态
void cam1_get_pose(float *dev, float *cam);

/// @brief 计算相机的位置姿态
/// @param b 输入相机的宽度
/// @param l 输入相机的长度
/// @param h 输入相机的高度
/// @param ar_x 返回相机的x轴方向的单位向量
/// @param ar_y 返回相机的y轴方向的单位向量
void cam1_get_ar(float b, float l, float h, float *ar_x, float *ar_y);

/// @brief 计算相机的位置姿态
/// @param dev 输入设备的位置姿态
/// @param cam 返回相机的位置姿态
void cam2_get_pose(float *dev, float *cam);

/// @brief 计算相机的位置姿态
/// @param b 输入相机的宽度
/// @param l 输入相机的长度
/// @param h 输入相机的高度
/// @param ar_x 返回相机的x轴方向的单位向量
/// @param ar_y 返回相机的y轴方向的单位向量
void cam2_get_ar(float b, float l, float h, float *ar_x, float *ar_y);

#endif
