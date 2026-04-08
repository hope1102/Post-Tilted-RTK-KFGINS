#include "algo_ahrs.h"
#include "setting.h"
#include "time_sync.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

// 定义圆周率和数学常数
#define M_PI 3.14159265358979323846f
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

// IMU坐标系定义：前右下（FRD）
// X轴：向前（Forward）
// Y轴：向右（Right）
// Z轴：向下（Down）

// 滤波器参数
#define MADGWICK_BETA 0.1f      // Madgwick滤波器增益参数
#define MAHONY_KP 2.0f          // Mahony滤波器比例增益
#define MAHONY_KI 0.01f         // Mahony滤波器积分增益

// AHRS配置参数结构体（使用float适配MCU）
typedef struct
{
	// 惯性传感器参数
	float gyro_bias[3];	  // 陀螺仪零偏 (rad/s)
	float accel_bias[3];  // 加速度计零偏 (m/s²)
	float gyro_noise[3];  // 陀螺仪噪声标准差 (rad/s/√Hz)
	float accel_noise[3]; // 加速度计噪声标准差 (m/s²/√Hz)

	// GNSS接收机参数
	float gnss_lever_arm[3]; // GNSS天线杆臂补偿 (m, [x,y,z])
	float pos_noise[3];		 // GNSS位置观测噪声 (m, [E,N,U])
	float vel_noise[3];		 // GNSS速度观测噪声 (m/s, [E,N,U])

	// 滤波参数
	float init_att_noise;  // 初始姿态角不确定度 (rad)
	float init_vel_noise;  // 初始速度不确定度 (m/s)
	float init_pos_noise;  // 初始位置不确定度 (m)
	float init_bias_noise; // 初始零偏不确定度 (rad/s或m/s²)

	// 过程噪声
	float gyro_bias_noise;	// 陀螺零偏过程噪声 (rad/s/√s)
	float accel_bias_noise; // 加计零偏过程噪声 (m/s²/√s)

	// 时间参数
	float imu_sample_rate;	// IMU采样频率 (Hz)
	float gnss_update_rate; // GNSS更新频率 (Hz)

	// 环境参数
	float gravity;			// 当地重力加速度 (m/s²)
	uint8_t use_earth_corr; // 使能地球自转补偿 (0/1)
} ahrs_setting_t;

// 全局配置和状态
static ahrs_setting_t g_config;
static ahrs_state_t g_state;

// IMU和GNSS数据缓存
static imu_data_t g_last_imu;
static gnss_data_t g_last_gnss;
static uint8_t g_imu_initialized = 0;
static uint8_t g_gnss_initialized = 0;
static float g_last_imu_time = 0.0f;
static float g_last_gnss_time = 0.0f;

// AHRS滤波器状态
static float g_integralFB[3] = {0.0f, 0.0f, 0.0f}; // Mahony积分反馈项
static uint32_t g_alignment_count = 0;               // 初始对齐计数

/**
 * @brief 快速平方根倒数计算（提高MCU性能）
 */
static float fast_inv_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * @brief 四元数归一化（优化版）
 */
static void quaternion_normalize(float q[4])
{
	float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	if (norm_sq > 1e-12f)
	{
		float inv_norm = fast_inv_sqrt(norm_sq);
		q[0] *= inv_norm;
		q[1] *= inv_norm;
		q[2] *= inv_norm;
		q[3] *= inv_norm;
	}
}

/**
 * @brief 四元数转欧拉角（FRD坐标系）
 */
static void quaternion_to_euler(const float q[4], float rpy[3])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];

	// Roll (绕X轴旋转，前右下坐标系)
	float sinr_cosp = 2.0f * (w * x + y * z);
	float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	rpy[0] = atan2f(sinr_cosp, cosr_cosp);

	// Pitch (绕Y轴旋转，前右下坐标系)
	float sinp = 2.0f * (w * y - z * x);
	if (fabsf(sinp) >= 1.0f)
		rpy[1] = copysignf(M_PI / 2.0f, sinp);
	else
		rpy[1] = asinf(sinp);

	// Yaw (绕Z轴旋转，前右下坐标系，初始方位角为0)
	float siny_cosp = 2.0f * (w * z + x * y);
	float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	rpy[2] = atan2f(siny_cosp, cosy_cosp);

	// 转换为度并处理角度范围
	rpy[0] = RAD_TO_DEG(rpy[0]);
	rpy[1] = RAD_TO_DEG(rpy[1]);
	rpy[2] = RAD_TO_DEG(rpy[2]);

	// 保持Yaw角在0-360度范围内
	if (rpy[2] < 0.0f)
		rpy[2] += 360.0f;
}

/**
 * @brief 初始姿态对齐（FRD坐标系，多采样平均）
 */
static void initial_alignment(const imu_data_t *imu)
{
	static float acc_sum[3] = {0.0f, 0.0f, 0.0f};

	// 累积加速度数据用于平均
	acc_sum[0] += imu->acc_x;
	acc_sum[1] += imu->acc_y;
	acc_sum[2] += imu->acc_z;
	g_alignment_count++;

	// 采集100个样本后进行对齐
	if (g_alignment_count >= 100)
	{
		// 计算平均加速度
		float ax = acc_sum[0] / g_alignment_count;
		float ay = acc_sum[1] / g_alignment_count;
		float az = acc_sum[2] / g_alignment_count;

		float norm_acc = sqrtf(ax * ax + ay * ay + az * az);

		// 检查是否接近重力加速度值（静止状态）
		if (norm_acc > 8.0f && norm_acc < 12.0f)
		{
			// 归一化加速度矢量
			ax /= norm_acc;
			ay /= norm_acc;
			az /= norm_acc;

			// FRD坐标系：前右下
			// 静止时重力指向下方（正Z方向）
			// Roll: 绕前轴（X）旋转，基于右轴（Y）和下轴（Z）
			// Pitch: 绕右轴（Y）旋转，基于前轴（X）和下轴（Z）
			float roll = atan2f(ay, az);
			float pitch = -asinf(ax);  // 负号适应FRD坐标系
			float yaw = 0.0f;          // 初始偏航角设为0

			// 欧拉角转四元数（ZYX顺序）
			float cr = cosf(roll * 0.5f);
			float sr = sinf(roll * 0.5f);
			float cp = cosf(pitch * 0.5f);
			float sp = sinf(pitch * 0.5f);
			float cy = cosf(yaw * 0.5f);
			float sy = sinf(yaw * 0.5f);

			g_state.q[0] = cr * cp * cy + sr * sp * sy; // w
			g_state.q[1] = sr * cp * cy - cr * sp * sy; // x
			g_state.q[2] = cr * sp * cy + sr * cp * sy; // y
			g_state.q[3] = cr * cp * sy - sr * sp * cy; // z

			quaternion_normalize(g_state.q);
			quaternion_to_euler(g_state.q, g_state.rpy);
		}

		// 重置累积器
		acc_sum[0] = acc_sum[1] = acc_sum[2] = 0.0f;
		g_alignment_count = 0;
	}
}

/**
 * @brief Mahony AHRS算法更新（FRD坐标系优化版）
 */
static void mahony_ahrs_update(const imu_data_t *imu, float dt)
{
	if (dt <= 0.0f || dt > 0.1f)
		return;

	// 读取传感器数据并补偿零偏
	float gx = DEG_TO_RAD(imu->gyro_x - g_state.gyro_bias[0]);
	float gy = DEG_TO_RAD(imu->gyro_y - g_state.gyro_bias[1]);
	float gz = DEG_TO_RAD(imu->gyro_z - g_state.gyro_bias[2]);

	float ax = imu->acc_x - g_state.accel_bias[0];
	float ay = imu->acc_y - g_state.accel_bias[1];
	float az = imu->acc_z - g_state.accel_bias[2];

	// 归一化加速度计数据
	float norm_acc = fast_inv_sqrt(ax * ax + ay * ay + az * az);
	if (norm_acc < 1e-6f) return; // 避免除零

	ax *= norm_acc;
	ay *= norm_acc;
	az *= norm_acc;

	// 提取四元数
	float q0 = g_state.q[0], q1 = g_state.q[1], q2 = g_state.q[2], q3 = g_state.q[3];

	// 计算重力在机体坐标系的预测值（从四元数）
	float vx = 2.0f * (q1 * q3 - q0 * q2);
	float vy = 2.0f * (q0 * q1 + q2 * q3);
	float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	// 计算加速度计测量值与重力预测值的误差
	float ex = ay * vz - az * vy;
	float ey = az * vx - ax * vz;
	float ez = ax * vy - ay * vx;

	// 积分误差（仅在静止或低加速度时）
	float acc_magnitude = sqrtf(ax * ax + ay * ay + az * az);
	if (fabsf(acc_magnitude - 1.0f) < 0.2f) // 接近重力加速度时才积分
	{
		g_integralFB[0] += ex * MAHONY_KI * dt;
		g_integralFB[1] += ey * MAHONY_KI * dt;
		g_integralFB[2] += ez * MAHONY_KI * dt;
	}

	// 应用比例和积分反馈
	gx += MAHONY_KP * ex + g_integralFB[0];
	gy += MAHONY_KP * ey + g_integralFB[1];
	gz += MAHONY_KP * ez + g_integralFB[2];

	// 四元数微分方程
	float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	// 积分更新四元数
	g_state.q[0] += dq0 * dt;
	g_state.q[1] += dq1 * dt;
	g_state.q[2] += dq2 * dt;
	g_state.q[3] += dq3 * dt;

	// 归一化四元数
	quaternion_normalize(g_state.q);

	// 转换为欧拉角
	quaternion_to_euler(g_state.q, g_state.rpy);
}

/**
 * @brief AHRS模块初始化
 */
void ahrs_init()
{
	// 获取用户参数
	user_params_t *params = get_user_params();

	// 初始化配置参数（使用float适配MCU）
	g_config = (ahrs_setting_t){
		.gyro_bias = {0.001f, -0.0005f, 0.002f},
		.accel_bias = {0.02f, -0.03f, 0.12f},
		.gyro_noise = {0.005f, 0.004f, 0.006f},
		.accel_noise = {0.01f, 0.01f, 0.01f},
		.gnss_lever_arm = {params->gnss_arm[0], params->gnss_arm[1], params->gnss_arm[2]},
		.pos_noise = {0.5f, 0.5f, 1.0f},
		.vel_noise = {0.05f, 0.05f, 0.1f},
		.init_att_noise = 0.1f,
		.init_vel_noise = 0.5f,
		.init_pos_noise = 10.0f,
		.init_bias_noise = 0.01f,
		.gyro_bias_noise = 1e-5f,
		.accel_bias_noise = 1e-4f,
		.imu_sample_rate = (float)params->imu_freq,
		.gnss_update_rate = 10.0f,
		.gravity = params->gravity,
		.use_earth_corr = 1};

	// 初始化状态
	memset(&g_state, 0, sizeof(ahrs_state_t));
	g_state.q[0] = 1.0f; // 初始四元数为单位四元数
	memcpy(g_state.gyro_bias, g_config.gyro_bias, sizeof(g_config.gyro_bias));
	memcpy(g_state.accel_bias, g_config.accel_bias, sizeof(g_config.accel_bias));

	// 重置初始化标志和滤波器状态
	g_imu_initialized = 0;
	g_gnss_initialized = 0;
	g_last_imu_time = 0.0f;
	g_last_gnss_time = 0.0f;
	g_alignment_count = 0;

	// 重置Mahony滤波器积分项
	g_integralFB[0] = g_integralFB[1] = g_integralFB[2] = 0.0f;
}

/**
 * @brief 添加IMU数据进行处理（100Hz）
 */
void ahrs_add_imu(imu_data_t *data)
{
	if (data == NULL)
		return;

	// 计算时间戳
	double current_time = (double)data->time_sec_s + (double)data->time_sec_ms * 0.001f;

	// 首次IMU数据进行初始对齐
	if (!g_imu_initialized)
	{
		initial_alignment(data);

		// 检查是否完成对齐（需要采集100个样本）
		if (g_alignment_count == 0) // 对齐完成后计数器被重置为0
		{
			g_imu_initialized = 1;
		}

		g_last_imu_time = current_time;
		memcpy(&g_last_imu, data, sizeof(imu_data_t));
		return;
	}

	// 计算时间间隔
	double dt = current_time - g_last_imu_time;
	if (dt > 0.001f && dt < 0.1f) // 有效时间间隔（1ms - 100ms）
	{
		// 执行Mahony AHRS算法更新
		mahony_ahrs_update(data, (float)dt);

		// 更新系统时间戳
		g_state.time_sync = data->time_sync;
		g_state.time_sec_s = data->time_sec_s;
		g_state.time_sec_ms = data->time_sec_ms;
		g_state.sys_time = current_time;
	}

	// 保存当前数据
	g_last_imu_time = current_time;
	memcpy(&g_last_imu, data, sizeof(imu_data_t));
}

/**
 * @brief 计算两点间的方位角（真北方向）
 */
static float calculate_azimuth(double lat1, double lon1, double lat2, double lon2)
{
	// 转换为弧度
	double lat1_rad = DEG_TO_RAD(lat1);
	double lat2_rad = DEG_TO_RAD(lat2);
	double dlon_rad = DEG_TO_RAD(lon2 - lon1);

	// 计算方位角
	double y = sin(dlon_rad) * cos(lat2_rad);
	double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad);

	double azimuth_rad = atan2(y, x);
	double azimuth_deg = RAD_TO_DEG(azimuth_rad);

	// 转换为0-360度范围
	if (azimuth_deg < 0.0)
		azimuth_deg += 360.0;

	return (float)azimuth_deg;
}

/**
 * @brief GNSS+IMU松组合算法（计算真实方位角）
 */
static void gnss_imu_loose_coupling(gnss_data_t *gnss_data, float dt)
{
	static double prev_lat = 0.0, prev_lon = 0.0;
	static float prev_course = 0.0f;
	static uint32_t velocity_count = 0;

	// 位置更新（加权融合）
	if (!g_gnss_initialized)
	{
		// 首次GNSS数据：初始化位置
		g_state.pos[0] = gnss_data->gga.latitude;
		g_state.pos[1] = gnss_data->gga.longitude;
		g_state.pos[2] = gnss_data->gga.altitude;

		prev_lat = gnss_data->gga.latitude;
		prev_lon = gnss_data->gga.longitude;
		g_gnss_initialized = 1;
		return;
	}

	// 位置加权融合
	float pos_weight = 0.7f; // GNSS位置权重
	g_state.pos[0] = pos_weight * gnss_data->gga.latitude + (1.0f - pos_weight) * g_state.pos[0];
	g_state.pos[1] = pos_weight * gnss_data->gga.longitude + (1.0f - pos_weight) * g_state.pos[1];
	g_state.pos[2] = pos_weight * gnss_data->gga.altitude + (1.0f - pos_weight) * g_state.pos[2];

	// 速度计算（基于位置差分）
	if (dt > 0.01f && dt < 2.0f)
	{
		// 计算位置变化
		double dlat = gnss_data->gga.latitude - prev_lat;
		double dlon = gnss_data->gga.longitude - prev_lon;
		double dalt = gnss_data->gga.altitude - g_last_gnss.gga.altitude;

		// 转换为米/秒（简化处理）
		float vel_n = (float)(dlat * 111000.0 / dt);  // 北向速度
		float vel_e = (float)(dlon * 111000.0 * cos(DEG_TO_RAD(gnss_data->gga.latitude)) / dt); // 东向速度
		float vel_d = (float)(dalt / dt);             // 垂向速度

		// 速度滤波
		float vel_weight = 0.6f;
		g_state.vel[0] = vel_weight * vel_n + (1.0f - vel_weight) * g_state.vel[0];
		g_state.vel[1] = vel_weight * vel_e + (1.0f - vel_weight) * g_state.vel[1];
		g_state.vel[2] = vel_weight * vel_d + (1.0f - vel_weight) * g_state.vel[2];

		// 计算地面速度和航向
		float ground_speed = sqrtf(vel_n * vel_n + vel_e * vel_e);

		// 仅在有足够运动时更新航向（避免噪声）
		if (ground_speed > 0.5f) // 速度大于0.5m/s时
		{
			// 计算运动方向的真航向
			float course_true = atan2f(vel_e, vel_n);
			course_true = RAD_TO_DEG(course_true);
			if (course_true < 0.0f)
				course_true += 360.0f;

			// 航向滤波
			float heading_weight = 0.3f; // 较低的权重保持稳定性
			float heading_diff = course_true - prev_course;

			// 处理角度跳变（360/0度边界）
			if (heading_diff > 180.0f)
				heading_diff -= 360.0f;
			else if (heading_diff < -180.0f)
				heading_diff += 360.0f;

			float new_heading = prev_course + heading_weight * heading_diff;
			if (new_heading < 0.0f)
				new_heading += 360.0f;
			else if (new_heading >= 360.0f)
				new_heading -= 360.0f;

			// 更新IMU航向角（松组合核心：用GNSS真航向修正IMU姿态）
			g_state.rpy[2] = new_heading;

			// 更新四元数以匹配新的航向角
			float roll_rad = DEG_TO_RAD(g_state.rpy[0]);
			float pitch_rad = DEG_TO_RAD(g_state.rpy[1]);
			float yaw_rad = DEG_TO_RAD(g_state.rpy[2]);

			float cr = cosf(roll_rad * 0.5f);
			float sr = sinf(roll_rad * 0.5f);
			float cp = cosf(pitch_rad * 0.5f);
			float sp = sinf(pitch_rad * 0.5f);
			float cy = cosf(yaw_rad * 0.5f);
			float sy = sinf(yaw_rad * 0.5f);

			g_state.q[0] = cr * cp * cy + sr * sp * sy; // w
			g_state.q[1] = sr * cp * cy - cr * sp * sy; // x
			g_state.q[2] = cr * sp * cy + sr * cp * sy; // y
			g_state.q[3] = cr * cp * sy - sr * sp * cy; // z

			quaternion_normalize(g_state.q);

			prev_course = new_heading;
			velocity_count++;
		}
	}

	// 保存当前位置
	prev_lat = gnss_data->gga.latitude;
	prev_lon = gnss_data->gga.longitude;
}

/**
 * @brief 添加GNSS数据进行松组合处理（10Hz）
 */
void ahrs_add_gnss(gnss_data_t *data)
{
	if (data == NULL)
		return;

	// 计算时间戳
	float current_time = (float)data->time_sec_s + (float)data->time_sec_ms * 0.001f;

	// 调试输出
	user_params_t *p = get_user_params();
	if (p->debug_enable && p->print_gnss && data->index % 10 == 0)
	{
		printf("[AHRS]-[ADD-GNSS-10]: %d,%d.%03d, %.9f, %.9f, %.3f,%d,%d\n",
			   data->index, data->time_sec_s, data->time_sec_ms,
			   data->gga.latitude, data->gga.longitude, data->gga.altitude,
			   data->gga.satellitesUsed, data->gga.fixQuality);
	}

	// 检查GNSS数据有效性
	if (data->gga.fixQuality >= 1 && data->gga.satellitesUsed >= 4)
	{
		// 计算时间间隔
		float dt = current_time - g_last_gnss_time;

		// 执行GNSS+IMU松组合算法
		gnss_imu_loose_coupling(data, dt);

		// 更新时间戳
		g_state.time_sec_s = data->time_sec_s;
		g_state.time_sec_ms = data->time_sec_ms;
	}

	// 保存当前数据
	g_last_gnss_time = current_time;
	memcpy(&g_last_gnss, data, sizeof(gnss_data_t));
}

/**
 * @brief 获取当前IMU数据和姿态角
 */
void ahrs_get_imu_pose(ahrs_state_t *pose_data)
{
	pose_data = &g_state;

	// 构建NMEA格式的IMU+姿态数据字符串（不包括$和*）
	char nmea_data[256];
	snprintf(nmea_data, sizeof(nmea_data), "AIMU,%d,%d.%03d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
			 pose_data->time_sync,
			 pose_data->time_sec_s,
			 pose_data->time_sec_ms,
			 pose_data->rpy[0], // roll
			 pose_data->rpy[1], // pitch
			 pose_data->rpy[2], // yaw
			 g_last_imu.gyro_x,
			 g_last_imu.gyro_y,
			 g_last_imu.gyro_z,
			 g_last_imu.acc_x,
			 g_last_imu.acc_y,
			 g_last_imu.acc_z);

	// 计算NMEA0183校验和（8位异或）
	uint8_t checksum = 0;
	for (int i = 0; nmea_data[i] != '\0'; i++)
	{
		checksum ^= (uint8_t)nmea_data[i];
	}

	// 输出当前IMU数据和姿态角（带校验和）
	printf("$%s*%02X\r\n", nmea_data, checksum);
}

/**
 * @brief 获取当前位置姿态估计结果
 */
void ahrs_get_gnss_pose(ahrs_state_t *pose_data)
{
	if (pose_data == NULL)
		return;

	// 复制当前状态
	memcpy(pose_data, &g_state, sizeof(ahrs_state_t));

	// 构建NMEA格式的数据字符串（不包括$和*）
	char nmea_data[256];
	snprintf(nmea_data, sizeof(nmea_data), "POSE,1,%d.%03d,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f",
			 pose_data->time_sec_s,
			 pose_data->time_sec_ms,
			 pose_data->pos[0],
			 pose_data->pos[1],
			 pose_data->pos[2],
			 pose_data->rpy[0],
			 pose_data->rpy[1],
			 pose_data->rpy[2]);

	// 计算NMEA0183校验和（8位异或）
	uint8_t checksum = 0;
	for (int i = 0; nmea_data[i] != '\0'; i++)
	{
		checksum ^= (uint8_t)nmea_data[i];
	}

	// 输出当前位置姿态（带校验和）
	printf("$%s*%02X\r\n", nmea_data, checksum);
}

/**
 * @brief 清理AHRS模块资源
 */
void ahrs_uninit(void)
{
	// 重置所有状态和标志
	memset(&g_state, 0, sizeof(ahrs_state_t));
	memset(&g_config, 0, sizeof(ahrs_setting_t));
	g_imu_initialized = 0;
	g_gnss_initialized = 0;
	g_last_imu_time = 0.0f;
	g_last_gnss_time = 0.0f;
	g_alignment_count = 0;

	// 重置Mahony滤波器积分项
	g_integralFB[0] = g_integralFB[1] = g_integralFB[2] = 0.0f;
}
