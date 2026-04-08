#ifndef WIN_UART_H
#define WIN_UART_H

#ifdef PLATFORM_WINDOWS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

// 串口配置结构体
typedef struct
{
	char port_str[64];
	int port_num;
	int baud_rate;
	int data_bits; // 5, 6, 7, 8
	int stop_bits; // 1, 2
	char parity;   // 'n', 'o', 'e' for none/odd/even
} UartConfig;

// 串口设备句柄
typedef struct
{
	HANDLE hCom;
	UartConfig config;
} UartHandle;

// ========== 通用接口 ==========
UartHandle* uart_init(UartConfig config);
int uart_open(UartHandle* handle);
int uart_close(UartHandle* handle);
int uart_read(UartHandle* handle, char* buffer, size_t size);
int uart_write(UartHandle* handle, const char* data, size_t length);
void uart_release(UartHandle* handle);

#endif

#endif //WIN_UART_H
