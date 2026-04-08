#ifndef MAIN_SETTING_H
#define MAIN_SETTING_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// 固件版本定义 (4段式版本号)
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 0
#define FIRMWARE_VERSION_PATCH 1
#define FIRMWARE_VERSION_BUILD 20250902

// 版本字符串宏
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_VERSION_STRING TOSTRING(FIRMWARE_VERSION_MAJOR) "." TOSTRING(FIRMWARE_VERSION_MINOR) "." TOSTRING(FIRMWARE_VERSION_PATCH) "." TOSTRING(FIRMWARE_VERSION_BUILD)

// 参数魔数定义
#define PARAM_MAGIC_NUMBER 0x12345678


// Camera setting structure
typedef struct
{
	uint32_t size[2]; // Camera 1 image size (width, height)
	float arm[3];	  // Camera 1 lever arm, in meters
	float rot[3];	  // Camera 1 installation angles (RPY), in degrees
	float ffxy[4];	  // Camera 1 intrinsic parameters (fx, fy, x0, y0)
	float dist[5];	  // Camera 1 distortion parameters (m, k1, k2, p1, p2)
} cam_setting_t;

// Laser setting structure
typedef struct
{
	float arm[3]; // Laser lever arm, in meters
	float ah;	  // Laser horizontal installation angle, in degrees
	float av;	  // Laser vertical installation angle, in degrees
	float ar;	  // Laser roll installation angle, in degrees
} laser_setting_t;

typedef struct
{
	float arm[3]; // Laser lever arm, in meters
	float rot[3]; // Lidar rotation angle
} lidar_setting_t;

enum GNSS_TYPE_t
{
	GNSS_TYPE_NMEA = 0,
	GNSS_TYPE_UNICORE = 1,
};

enum GNSS_Mode_t
{
	GNSS_MODE_UNKNOW = 0,
	GNSS_MODE_AHRS = 1,	 // 姿态估计
	GNSS_MODE_IM1 = 2,	 // 倾斜测量-摇一摇
	GNSS_MODE_IM2 = 3,	 // 倾斜测量-免初始化
	GNSS_MODE_GINS1 = 4, // 组合导航-松耦合
	GNSS_MODE_GINS2 = 5	 // 组合导航-紧耦合
};

// User parameter structure
typedef struct
{
	// validation magic number - must be first field
	uint32_t magic_number;

	// registration
	uint8_t sn[20];			   // device sn,using 1633 sn
	uint8_t reg_flag;		   // Register flag, default 0
	uint8_t reg_date_start[8]; // Register date, default 0
	uint8_t reg_date_end[8];   // Register date, default 0

	// debug
	uint8_t debug_enable; // Debug mode, default 0
	uint8_t echo_enable;  // 回显控制, default 0
	uint8_t print_gnss;	  // 打印GNSS数据, default 0
	uint8_t print_imu;	  // 打印IMU数据, default 0

	// basic_settings
	uint32_t baud1;	  // Serial port 1 baud rate, default 460800
	uint32_t baud2;	  // Serial port 2 baud rate, default 460800
	uint32_t baud3;	  // Serial port 3 baud rate, default 460800
	uint8_t gps_leap; // GPS leap seconds, default 18
	float gravity;	  // Gravity constant, default 9.80665 m/s^2

	// algo_setting
	uint8_t mode; // Algorithm mode, default depends on product/firmware

	// uint8_t func_arhs;	// ARHS function support: 0-supported, 1-not supported
	// uint8_t func_im1;	// IM1 function support: 0-supported, 1-not supported
	// uint8_t func_im2;	// IM2 function support: 0-supported, 1-not supported
	// uint8_t func_gins1; // GINS1 function support: 0-supported, 1-not supported
	// uint8_t func_gins2; // GINS2 function support: 0-supported, 1-not supported

	// uint8_t func_imu;	  // IMU function support: 0-supported, 1-not supported
	// uint8_t func_gnss;	  // GNSS function support: 0-supported, 1-not supported
	// uint8_t func_magnet;  // Magnetometer function support: 0-supported, 1-not supported
	// uint8_t func_laser;	  // Laser function support: 0-supported, 1-not supported
	// uint8_t func_cam1_ar; // Camera 1 AR function support: 0-supported, 1-not supported
	// uint8_t func_cam2_ar; // Camera 2 AR function support: 0-supported, 1-not supported

	// imu_setting
	uint8_t imu_data; // Is Send $AIMU data to UART1
	uint8_t imu_pose; // Is Send $AIMU with rpy data to UART1
	uint8_t imu_freq; // IMU data output frequency, default 100Hz,200HZ MAX
	uint8_t imu_axis; // IMU axis definition, default "FRD"
	float imu_rot[3]; // IMU installation angles (roll, pitch, yaw), in degrees
	float imu_arm[3]; // IMU arm definition, default "FRD"

	// gnss_setting
	uint8_t gnss_type; // GNSS input data format, e.g., default 1-"NMEA" 2-"UNICORE" 0-Unknown
	uint8_t gnss_last; // NMEA last sentence, default NMEA_GPGST= 3, using enum NMEA_Type_t
	uint8_t gnss_data; // Is Send NMEA0183 data to UART1
	uint8_t gnss_pose; // Is Send $POSE data to UART1
	uint8_t gnss_pps; // Is Send $APPS data to UART1
	float gnss_arm[3]; // GNSS lever arm, in meters
	float club_arm[3]; // Output point lever arm, in meters

	// laser_setting
	laser_setting_t laser; // Laser setting

	// lidar_setting
	laser_setting_t lidar; // Lidar setting

	// cam_setting
	cam_setting_t cam1; // Camera 1 setting
	cam_setting_t cam2; // Camera 2 setting
	cam_setting_t cam3; // Camera 1 setting
	cam_setting_t cam4; // Camera 2 setting

	// solve_setting
	uint8_t carrier_type;// 0-wsrobot;1-car;2-pedestrian;3-uav;4-wsrobotdiesel;5-marine;6-airborne
	uint8_t imutype;
	uint8_t estmisv;
	uint8_t estlever;
	uint8_t estmemsscale;
	uint8_t estmemsort;
	uint8_t backfeedtype;
	uint8_t outsmoothtype;
} user_params_t;

void init_user_params();

user_params_t *get_user_params();

// 统一的保存函数接口
uint8_t save_current_params();

// 统一的加载函数接口
uint8_t read_current_params(const char *config_file);

// 调试打印所有用户参数
void debug_print_user_params(const char* prefix);

#endif // MAIN_SETTING_H