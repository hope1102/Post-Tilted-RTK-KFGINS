#ifndef __ALGO_AT_H__
#define __ALGO_AT_H__

#include <stdint.h>

#define AT_MAX_PARAMS 6 // 最多参数个数
#define MAX_CMD_LEN 128 // 最大命令长度

// 定义AT指令结构体
typedef struct
{
	char prefix[3];	  // "AT"
	char command[32]; // 命令部分，如"IMU"
	char operator;	  // 操作符，如'='或'?'
	char value1[16];  // 值部分，如"1"
	char value2[16];  // 值部分，如"1"
	char value3[16];  // 值部分，如"1"
	char value4[16];  // 值部分，如"1"
	char value5[16];  // 值部分，如"1"
	char value6[16];  // 值部分，如"1"
	char value7[16];  // 值部分，如"1"
	char value8[16];  // 值部分，如"1"
} at_command_t;

// 指令回调函数类型
typedef void (*AT_CmdHandler)(const char *cmd, const char *params[], uint8_t param_count, uint8_t is_query);

// 指令注册结构
typedef struct
{
	const char *cmd;
	AT_CmdHandler handler;
} AT_CmdRegistry_t;

// 单挑指令处理
uint8_t algo_at_command(char *cmd, uint32_t len);

// 批量指令处理
uint8_t algo_at_process(char *text, uint32_t len);

#endif
