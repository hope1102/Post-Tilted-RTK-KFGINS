#include "algo_laser.h"

#include <math.h>
#include <string.h>

// 定义圆周率
#define M_PI 3.14159265358979323846f
// 角度转弧度
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
// 弧度转角度
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

// 静态全局变量存储激光器配置
static laser_setting_t g_laser_config;

// 静态变量存储激光器当前位姿
static float g_laser_pose[6] = {0}; // x, y, z, roll, pitch, yaw

/**
 * @brief 初始化激光器的参数，主要是和设备的相对位姿关系
 * @param laser_setting 激光器配置参数
 */
void laser_init(laser_setting_t *laser_setting)
{
    if (laser_setting != NULL)
    {
        // 复制激光器配置参数
        memcpy(&g_laser_config, laser_setting, sizeof(laser_setting_t));
    }
}

/**
 * @brief 根据设备位置姿态和laser_setting参数，计算激光器的位置姿态
 * @param dev 输入设备的位置姿态 [x, y, z, roll, pitch, yaw]
 * @param laser 输出激光器的位置姿态 [x, y, z, roll, pitch, yaw]
 */
void laser_get_pose(float *dev, float *laser)
{
    if (dev == NULL || laser == NULL)
    {
        return;
    }

    // 设备位置和姿态
    float dev_x = dev[0], dev_y = dev[1], dev_z = dev[2];
    float dev_roll = DEG_TO_RAD(dev[3]);
    float dev_pitch = DEG_TO_RAD(dev[4]);
    float dev_yaw = DEG_TO_RAD(dev[5]);

    // 激光器相对于设备的杆臂和姿态角
    float arm_x = g_laser_config.arm[0];
    float arm_y = g_laser_config.arm[1];
    float arm_z = g_laser_config.arm[2];
    float laser_roll = DEG_TO_RAD(g_laser_config.ar);
    float laser_pitch = DEG_TO_RAD(g_laser_config.av);
    float laser_yaw = DEG_TO_RAD(g_laser_config.ah);

    // 计算旋转矩阵
    float cos_roll = cosf(dev_roll), sin_roll = sinf(dev_roll);
    float cos_pitch = cosf(dev_pitch), sin_pitch = sinf(dev_pitch);
    float cos_yaw = cosf(dev_yaw), sin_yaw = sinf(dev_yaw);

    // 旋转矩阵 (ZYX欧拉角顺序)
    float R[3][3];
    R[0][0] = cos_yaw * cos_pitch;
    R[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    R[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    R[1][0] = sin_yaw * cos_pitch;
    R[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    R[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    R[2][0] = -sin_pitch;
    R[2][1] = cos_pitch * sin_roll;
    R[2][2] = cos_pitch * cos_roll;

    // 计算激光器在全局坐标系下的位置
    laser[0] = dev_x + R[0][0] * arm_x + R[0][1] * arm_y + R[0][2] * arm_z;
    laser[1] = dev_y + R[1][0] * arm_x + R[1][1] * arm_y + R[1][2] * arm_z;
    laser[2] = dev_z + R[2][0] * arm_x + R[2][1] * arm_y + R[2][2] * arm_z;

    // 计算激光器在全局坐标系下的姿态 (简化处理：设备姿态 + 激光器安装姿态)
    laser[3] = RAD_TO_DEG(dev_roll + laser_roll);
    laser[4] = RAD_TO_DEG(dev_pitch + laser_pitch);
    laser[5] = RAD_TO_DEG(dev_yaw + laser_yaw);

    // 存储激光器当前位姿
    memcpy(g_laser_pose, laser, sizeof(g_laser_pose));
}

/**
 * @brief 根据激光器的位置姿态和测距，计算目标点的坐标
 * @param dis 输入测距值 (米)
 * @param target 输出目标点坐标 [x, y, z]
 */
void laser_get_target(float dis, float *target)
{
    if (target == NULL || dis <= 0.0f)
    {
        return;
    }

    // 激光器位置和姿态
    float laser_x = g_laser_pose[0];
    float laser_y = g_laser_pose[1];
    float laser_z = g_laser_pose[2];
    float laser_roll = DEG_TO_RAD(g_laser_pose[3]);
    float laser_pitch = DEG_TO_RAD(g_laser_pose[4]);
    float laser_yaw = DEG_TO_RAD(g_laser_pose[5]);

    // 激光器指向方向的单位向量 (激光器坐标系的X轴正方向)
    // 考虑激光器的姿态角影响
    float cos_pitch = cosf(laser_pitch);
    float sin_pitch = sinf(laser_pitch);
    float cos_yaw = cosf(laser_yaw);
    float sin_yaw = sinf(laser_yaw);

    // 激光束方向向量 (在全局坐标系下)
    float dx = cos_yaw * cos_pitch;
    float dy = sin_yaw * cos_pitch;
    float dz = -sin_pitch; // 负号表示向下

    // 计算目标点坐标
    target[0] = laser_x + dis * dx;
    target[1] = laser_y + dis * dy;
    target[2] = laser_z + dis * dz;
}
