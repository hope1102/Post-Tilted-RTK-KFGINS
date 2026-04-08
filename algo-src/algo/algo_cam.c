#include "algo_cam.h"

#include <math.h>
#include <string.h>

// 定义圆周率
#define M_PI 3.14159265358979323846f
// 角度转弧度
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
// 弧度转角度
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

// 静态全局变量存储相机配置
static cam_setting_t g_cam1_config;
static cam_setting_t g_cam2_config;

// 静态变量存储相机当前位姿
static float g_cam1_pose[6] = {0}; // x, y, z, roll, pitch, yaw
static float g_cam2_pose[6] = {0}; // x, y, z, roll, pitch, yaw

/**
 * @brief 初始化相机内参，外参，畸变参数
 * @param cam_setting 相机配置参数
 */
void cam_init(cam_setting_t *cam_setting)
{
    if (cam_setting != NULL)
    {
        // 复制相机配置参数到cam1
        memcpy(&g_cam1_config, cam_setting, sizeof(cam_setting_t));
        // 默认cam2使用相同配置
        memcpy(&g_cam2_config, cam_setting, sizeof(cam_setting_t));
    }
}

/**
 * @brief 计算相机1的位置姿态
 * @param dev 输入设备的位置姿态 [x, y, z, roll, pitch, yaw]
 * @param cam 返回相机的位置姿态 [x, y, z, roll, pitch, yaw]
 */
void cam1_get_pose(float *dev, float *cam)
{
    if (dev == NULL || cam == NULL)
    {
        return;
    }

    // 设备位置和姿态
    float dev_x = dev[0], dev_y = dev[1], dev_z = dev[2];
    float dev_roll = DEG_TO_RAD(dev[3]);
    float dev_pitch = DEG_TO_RAD(dev[4]);
    float dev_yaw = DEG_TO_RAD(dev[5]);

    // 相机1相对于设备的杆臂和姿态角
    float arm_x = g_cam1_config.arm[0];
    float arm_y = g_cam1_config.arm[1];
    float arm_z = g_cam1_config.arm[2];
    float cam_roll = DEG_TO_RAD(g_cam1_config.rot[0]);
    float cam_pitch = DEG_TO_RAD(g_cam1_config.rot[1]);
    float cam_yaw = DEG_TO_RAD(g_cam1_config.rot[2]);

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

    // 计算相机1在全局坐标系下的位置
    cam[0] = dev_x + R[0][0] * arm_x + R[0][1] * arm_y + R[0][2] * arm_z;
    cam[1] = dev_y + R[1][0] * arm_x + R[1][1] * arm_y + R[1][2] * arm_z;
    cam[2] = dev_z + R[2][0] * arm_x + R[2][1] * arm_y + R[2][2] * arm_z;

    // 计算相机1在全局坐标系下的姿态 (简化处理：设备姿态 + 相机安装姿态)
    cam[3] = RAD_TO_DEG(dev_roll + cam_roll);
    cam[4] = RAD_TO_DEG(dev_pitch + cam_pitch);
    cam[5] = RAD_TO_DEG(dev_yaw + cam_yaw);

    // 存储相机1当前位姿
    memcpy(g_cam1_pose, cam, sizeof(g_cam1_pose));
}

/**
 * @brief 计算相机1的AR方向向量
 * @param b 输入相机的宽度 (米)
 * @param l 输入相机的长度 (米)  
 * @param h 输入相机的高度 (米)
 * @param ar_x 返回相机的x轴方向的单位向量 [x, y, z]
 * @param ar_y 返回相机的y轴方向的单位向量 [x, y, z]
 */
void cam1_get_ar(float b, float l, float h, float *ar_x, float *ar_y)
{
    if (ar_x == NULL || ar_y == NULL)
    {
        return;
    }

    // 相机1位姿
    float cam_roll = DEG_TO_RAD(g_cam1_pose[3]);
    float cam_pitch = DEG_TO_RAD(g_cam1_pose[4]);
    float cam_yaw = DEG_TO_RAD(g_cam1_pose[5]);

    // 计算三角函数值
    float cos_roll = cosf(cam_roll), sin_roll = sinf(cam_roll);
    float cos_pitch = cosf(cam_pitch), sin_pitch = sinf(cam_pitch);
    float cos_yaw = cosf(cam_yaw), sin_yaw = sinf(cam_yaw);

    // 相机坐标系的X轴方向（在全局坐标系下）
    ar_x[0] = cos_yaw * cos_pitch;
    ar_x[1] = sin_yaw * cos_pitch;
    ar_x[2] = -sin_pitch;

    // 相机坐标系的Y轴方向（在全局坐标系下）
    ar_y[0] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    ar_y[1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    ar_y[2] = cos_pitch * sin_roll;

    // 考虑相机尺寸的影响（归一化处理）
    float scale_x = sqrtf(b * b + l * l + h * h);
    if (scale_x > 0.0f)
    {
        ar_x[0] /= scale_x;
        ar_x[1] /= scale_x;
        ar_x[2] /= scale_x;
        
        ar_y[0] /= scale_x;
        ar_y[1] /= scale_x;
        ar_y[2] /= scale_x;
    }
}

/**
 * @brief 计算相机2的位置姿态
 * @param dev 输入设备的位置姿态 [x, y, z, roll, pitch, yaw]
 * @param cam 返回相机的位置姿态 [x, y, z, roll, pitch, yaw]
 */
void cam2_get_pose(float *dev, float *cam)
{
    if (dev == NULL || cam == NULL)
    {
        return;
    }

    // 设备位置和姿态
    float dev_x = dev[0], dev_y = dev[1], dev_z = dev[2];
    float dev_roll = DEG_TO_RAD(dev[3]);
    float dev_pitch = DEG_TO_RAD(dev[4]);
    float dev_yaw = DEG_TO_RAD(dev[5]);

    // 相机2相对于设备的杆臂和姿态角
    float arm_x = g_cam2_config.arm[0];
    float arm_y = g_cam2_config.arm[1];
    float arm_z = g_cam2_config.arm[2];
    float cam_roll = DEG_TO_RAD(g_cam2_config.rot[0]);
    float cam_pitch = DEG_TO_RAD(g_cam2_config.rot[1]);
    float cam_yaw = DEG_TO_RAD(g_cam2_config.rot[2]);

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

    // 计算相机2在全局坐标系下的位置
    cam[0] = dev_x + R[0][0] * arm_x + R[0][1] * arm_y + R[0][2] * arm_z;
    cam[1] = dev_y + R[1][0] * arm_x + R[1][1] * arm_y + R[1][2] * arm_z;
    cam[2] = dev_z + R[2][0] * arm_x + R[2][1] * arm_y + R[2][2] * arm_z;

    // 计算相机2在全局坐标系下的姿态 (简化处理：设备姿态 + 相机安装姿态)
    cam[3] = RAD_TO_DEG(dev_roll + cam_roll);
    cam[4] = RAD_TO_DEG(dev_pitch + cam_pitch);
    cam[5] = RAD_TO_DEG(dev_yaw + cam_yaw);

    // 存储相机2当前位姿
    memcpy(g_cam2_pose, cam, sizeof(g_cam2_pose));
}

/**
 * @brief 计算相机2的AR方向向量
 * @param b 输入相机的宽度 (米)
 * @param l 输入相机的长度 (米)
 * @param h 输入相机的高度 (米)
 * @param ar_x 返回相机的x轴方向的单位向量 [x, y, z]
 * @param ar_y 返回相机的y轴方向的单位向量 [x, y, z]
 */
void cam2_get_ar(float b, float l, float h, float *ar_x, float *ar_y)
{
    if (ar_x == NULL || ar_y == NULL)
    {
        return;
    }

    // 相机2位姿
    float cam_roll = DEG_TO_RAD(g_cam2_pose[3]);
    float cam_pitch = DEG_TO_RAD(g_cam2_pose[4]);
    float cam_yaw = DEG_TO_RAD(g_cam2_pose[5]);

    // 计算三角函数值
    float cos_roll = cosf(cam_roll), sin_roll = sinf(cam_roll);
    float cos_pitch = cosf(cam_pitch), sin_pitch = sinf(cam_pitch);
    float cos_yaw = cosf(cam_yaw), sin_yaw = sinf(cam_yaw);

    // 相机坐标系的X轴方向（在全局坐标系下）
    ar_x[0] = cos_yaw * cos_pitch;
    ar_x[1] = sin_yaw * cos_pitch;
    ar_x[2] = -sin_pitch;

    // 相机坐标系的Y轴方向（在全局坐标系下）
    ar_y[0] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    ar_y[1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    ar_y[2] = cos_pitch * sin_roll;

    // 考虑相机尺寸的影响（归一化处理）
    float scale_y = sqrtf(b * b + l * l + h * h);
    if (scale_y > 0.0f)
    {
        ar_x[0] /= scale_y;
        ar_x[1] /= scale_y;
        ar_x[2] /= scale_y;
        
        ar_y[0] /= scale_y;
        ar_y[1] /= scale_y;
        ar_y[2] /= scale_y;
    }
}
