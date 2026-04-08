#ifndef UART1_IMU_H
#define UART1_IMU_H


// 初始化IMU
void imu_init();

// IMU采集任务
void imu_read_loop();

// IMU采集任务
void imu_read_task();

#endif
