#ifndef UART2_UPDATE_H
#define UART2_UPDATE_H

// 用户固件升级

// 初始化Uart2
void uart2_init();

// 释放资源
void uart2_uninit();

// 更新任务循环
void update_read_loop();

// 更新任务现成
void update_read_task();

#endif // UART2_UPDATE_H
