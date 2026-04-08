#ifndef __ALGO_DATA_H__
#define __ALGO_DATA_H__

#include <stdint.h>

// IMU数据结构体
//$ALGOIMU, 2381,  113234.689,  13327.227,-0.063242, -0.084905, -0.035306,  0.008235,  0.001446, -1.001354,11100*54
typedef struct
{
	uint32_t index;		  // 自增计数
	uint8_t time_sync;	  // 时间同步状态 (0:未同步，MCU系统时间；1:NMEA的UTC0时间；2:处理跳秒后的GPS时间)
	uint32_t time_week;	  // 时间GPS周(秒)
	uint32_t time_sec_s;  // 时间整数部分(秒)
	uint32_t time_sec_ms; // 时间小数部分(毫秒)
	uint32_t time_stamp;  // MCU systemtime，单位(毫秒)
	float acc_x;		  // 加速度计x轴数据
	float acc_y;		  // 加速度计y轴数据
	float acc_z;		  // 加速度计z轴数据
	float gyro_x;		  // 陀螺仪x轴数据
	float gyro_y;		  // 陀螺仪y轴数据
	float gyro_z;		  // 陀螺仪z轴数据
	float temperature;	  // IMU温度(℃)
} imu_data_t;

// POSE数据结构体
// $POSE,1,2,130089,4,4,31.2312,121.4732,52.3,1.8,3.1,123.5,45.2 * 6A
typedef struct
{
	uint32_t index;		  // IMU自增计数
	uint8_t time_sync;	  // 时间同步状态 (0:未同步，MCU系统时间；1:NMEA的UTC0时间；2:处理跳秒后的GPS时间)
	uint32_t time_week;	  // 时间GPS周(秒)
	uint32_t time_sec_s;  // 时间整数部分(秒)
	uint32_t time_sec_ms; // 时间小数部分(毫秒)
	uint32_t time_stamp;  // MCU systemtime，单位(毫秒)

	// 元数据
	uint8_t system_status;	// 系统状态 (0-不可用, 1-可用)
	uint8_t algorithm_type; // 算法类型
	uint8_t fusion_status;	// 组合定位状态 (0-失效,1-初始化,2-低精度,4-高精度)

	// 定位数据
	float precision_factor;	 // 精度因子
	uint8_t rtk_status;		 // RTK定位状态 (0-未定位,1-单点,2-伪距差分,4-固定解,5-浮点解)
	uint8_t rtk_delay;		 // RTK差分延时
	uint8_t satellite_count; // 参与解算的卫星数

	// 地理坐标
	double gnss_latitude;  // GNSS纬度 (度)
	double gnss_longitude; // GNSS经度 (度)
	float gnss_altitude;   // GNSS高程 (米)

	// 杆体位置
	double pole_latitude;  // 杆底纬度 (度)
	double pole_longitude; // 杆底经度 (度)
	float pole_altitude;   // 杆底高程 (米)

	// 姿态数据
	float pole_length; // 杆长 (米)
	float roll;		   // 横滚角 (度)
	float pitch;	   // 俯仰角 (度)
	float yaw;		   // 航向角 (度)
	float azimuth;	   // 方位角 (度)
} pose_data_t;

// IMU数据解码函数
int32_t imu_data_decode(const uint8_t *data,uint32_t len, imu_data_t *imu_data);

// POSE数据解码函数
int32_t pose_data_decode(const uint8_t *data,uint32_t len, pose_data_t *pose_data);

// IMU数据编码函数
int32_t imu_data_encode(uint8_t *data, uint32_t len, const imu_data_t *imu_data);

// POSE数据编码函数
int32_t pose_data_encode(uint8_t *data,uint32_t len, const pose_data_t *pose_data);

#endif
