/**
 * @file algo_ahrs.h
 * @author your name (you@domain.com)
 * @brief algo_ahrs模块主要功能是进行位置姿态估计，ahrs_add_imu输入100HZIMU数据，
 *        ahrs_add_gnss输入10HZ位置数据，模块内部进行GNSS和IMU的松组合处理，得到实时的状态。
 * @version 0.1
 * @date 2025-07-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef ALGO_ARHS_H_
#define ALGO_ARHS_H_

#include "setting.h" //setting struct
#include "algo_data.h" //algo imu data struct
#include "nmea_data.h" //nmea gnss data struct

#include <stdint.h>

// 状态向量定义（四元数+速度+位置+零偏）
typedef struct
{
	uint8_t time_sync;
	uint32_t time_sec_ms;//时间戳，单位：毫秒
	uint32_t time_sec_s; //时间戳，单位：秒
	float sys_time;      // 系统时间，单位：秒
	float q[4];          // 姿态四元数 (w,x,y,z)
	float rpy[3];        // 欧拉角 (roll,pitch,yaw)
	float vel[3];        // 速度 (NED系 m/s)
	float pos[3];        // 位置 (纬度,经度,高度)
	float gyro_bias[3];  // 陀螺仪零偏估计
	float accel_bias[3]; // 加速度计零偏估计
} ahrs_state_t;

void ahrs_init();

void ahrs_add_imu(imu_data_t* imu_data);

void ahrs_add_gnss(gnss_data_t* gnss_data);

void ahrs_get_imu_pose(ahrs_state_t* pose_data);

void ahrs_get_gnss_pose(ahrs_state_t* pose_data);

void ahrs_uninit();

#endif //!ALGO_ARHS_H_