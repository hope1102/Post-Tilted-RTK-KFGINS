#ifndef UART3_GNSS_H
#define UART3_GNSS_H

// 初始化UART3用于GNSS模块
void uart3_init(void);

// 释放资源
void uart3_uninit();

// GNSS任务

void gnss_pps_loop();

void gnss_read_loop();

void gnss_read_task();

#endif // UART3_GNSS_H
