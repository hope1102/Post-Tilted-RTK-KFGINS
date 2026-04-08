#ifndef UART1_ATCMD_H
#define UART1_ATCMD_H

// 初始化uart1
void uart1_init(void);

// 反初始化uart1
void uart1_uninit();

// AT指令循环任务
void atcmd_read_loop();

// AT指令任务线程
void atcmd_read_task();

#endif
