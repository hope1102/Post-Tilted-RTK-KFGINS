#include "algo_atcmd.h"

#include "algo_string.h"
#include "setting.h"

#ifdef PLATFORM_MCU
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#include "mcu_flash.h"
#include "mcu_uart.h"
#endif // PLATFORM_MCU

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 全局变量
static user_params_t *sysConfig = NULL;

// 全局缓存，用于存储未完成的AT指令
static char cmdBuffer[MAX_CMD_LEN];
static uint32_t cmdIndex = 0;

/**
 * @brief 解析AT指令
 * @param input 输入的AT指令字符串
 * @param cmd 输出参数，存储解析后的指令信息
 * @return 成功返回0，失败返回-1
 */
int32_t parse_at_command(const char *input, at_command_t *cmd)
{
	if (input == NULL || cmd == NULL)
	{
		return -1;
	}

	// 初始化结构体
	memset(cmd, 0, sizeof(at_command_t));

	// 检查基本格式
	if (strlen(input) < 2 || strncmp(input, "AT", 2) != 0)
	{
		printf("ERROR: No AT command found\n");
		return -2;
	}

	// 提取前缀"AT"
	strncpy(cmd->prefix, input, 2);
	cmd->prefix[2] = '\0';

	const char *ptr = input + 2; // 跳过"AT"

	// 检查是否有'+'符号
	if (*ptr == '+')
	{
		ptr++; // 跳过'+'
	}
	else if (*ptr == '\0' || *ptr == '\r' || *ptr == '\n' || *ptr == '?')
	{
		// 只有"AT"指令，没有其他内容
		cmd->command[0] = '\0';
		cmd->operator = '\0';
		return 0;
	}
	else
	{
		return -3;
	}

	// 提取命令部分
	int i = 0;
	while (*ptr != '\0' && *ptr != '=' && *ptr != '?' && *ptr != '\r' && *ptr != '\n')
	{
		if (i < sizeof(cmd->command) - 1)
		{
			cmd->command[i++] = *ptr;
		}
		ptr++;
	}
	cmd->command[i] = '\0';

	// 检查操作符
	if (*ptr == '=' || *ptr == '?')
	{
		cmd->operator = *ptr;
		ptr++;

		// 创建参数字符串数组指针
		char *value_ptrs[AT_MAX_PARAMS] = {
			cmd->value1, cmd->value2, cmd->value3,
			cmd->value4, cmd->value5, cmd->value6};

		// 解析参数，用逗号分隔
		int param_index = 0;
		int char_index = 0;

		while (*ptr != '\0' && *ptr != '\r' && *ptr != '\n' && param_index < AT_MAX_PARAMS)
		{
			if (*ptr == ',')
			{
				// 结束当前参数
				value_ptrs[param_index][char_index] = '\0';
				param_index++;
				char_index = 0;
			}
			else
			{
				// 添加字符到当前参数
				if (char_index < 15) // value字段大小为16，保留一个位置给'\0'
				{
					value_ptrs[param_index][char_index++] = *ptr;
				}
			}
			ptr++;
		}

		// 结束最后一个参数
		if (param_index < AT_MAX_PARAMS)
		{
			value_ptrs[param_index][char_index] = '\0';
		}
	}
	else if (*ptr == '\r' || *ptr == '\n' || *ptr == '\0')
	{
		// 没有操作符和值，只有命令
		cmd->operator = '\0';
		cmd->value1[0] = '\0';
	}
	else
	{
		printf("ERROR: AT command error\n");
		return -4;
	}

	return 0;
}

/**
 * @brief 打印解析后的AT指令信息
 * @param cmd AT指令结构体
 */
void print_at_command(const at_command_t *cmd)
{
	printf("$ACMD,%s+%s", cmd->prefix, cmd->command);
	if (cmd->operator == '=')
	{
		printf("%c", cmd->operator ? cmd->operator : ' ');
		if (cmd->value1[0])
			printf("%s", cmd->value1[0] ? cmd->value1 : " ");
		if (cmd->value2[0])
			printf(",%s", cmd->value2[0] ? cmd->value2 : " ");
		if (cmd->value3[0])
			printf(",%s", cmd->value3[0] ? cmd->value3 : " ");
		if (cmd->value4[0])
			printf(",%s", cmd->value4[0] ? cmd->value4 : " ");
		if (cmd->value5[0])
			printf(",%s", cmd->value5[0] ? cmd->value5 : " ");
		if (cmd->value6[0])
			printf(",%s", cmd->value6[0] ? cmd->value6 : " ");
	}
	printf("\r\n");
}

/**************************** 基础函数实现 ******************************/

// 整数解析函数
static int parse_int_param(const char *param)
{
	return (param != NULL) ? atoi(param) : -1;
}

// 浮点数解析函数
static float parse_float_param(const char *param)
{
	return (param != NULL) ? atof(param) : 0.0f;
}

// 发送响应函数
void AT_SendResponse(const char *response)
{
	printf("%s\r\n", response);
}

/**************************** 指令处理函数实现 ******************************/

// 系统重启
void AT_REBOOT_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
#ifdef PLATFORM_MCU
	__disable_irq();
	NVIC_SystemReset();
#endif // PLATFORM_MCU
	AT_SendResponse("OK: Resetting system");
}

// 恢复出厂设置
void AT_RESET_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	init_user_params();
	int result = save_current_params();
	if (result == 0)
	{
		AT_SendResponse("OK: Parameters reset done.");
	}
	else
	{
		AT_SendResponse("ERROR: Failed to read parameters");
	}
}

// 固件升级
void AT_UPGRADE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
#ifdef PLATFORM_MCU
	uint32_t i = APP_UPGRADE_FLAG;
	FLASH_WriteData(APP_UPGRADE_FLAG_ADDR, (uint8_t *)&i, 4U);

	__disable_irq();
	NVIC_SystemReset();
#endif // PLATFORM_MCU
	AT_SendResponse("OK: Upgrading...");
}

// 保存参数
void AT_SAVE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	int result = save_current_params();
	if (result == 0)
	{
		AT_SendResponse("OK");
	}
	else
	{
		AT_SendResponse("ERROR: Failed to save parameters");
	}
}

// 查询所有参数
void AT_CONFIG_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (param_count > 0)
	{
		AT_SendResponse("ERROR: Command does not take parameters");
		return;
	}

	// 输出所有参数
	char buf[256];
	AT_SendResponse("******** Qurey Config Start ********");

	// debug
	snprintf(buf, sizeof(buf), "DEBUG=%u,%u,%u", sysConfig->debug_enable,
			 sysConfig->print_imu, sysConfig->print_gnss);
	AT_SendResponse(buf);

	// basic_settings
	snprintf(buf, sizeof(buf), "BAUD1=%u", sysConfig->baud1);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "BAUD2=%u", sysConfig->baud2);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "BAUD3=%u", sysConfig->baud3);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "GPS_LEAP=%u", sysConfig->gps_leap);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "GRAVITY=%.5f", sysConfig->gravity);
	AT_SendResponse(buf);

	// algo_setting
	snprintf(buf, sizeof(buf), "MODE=%u", sysConfig->mode);
	AT_SendResponse(buf);

	// imu_setting
	snprintf(buf, sizeof(buf), "IMU_DATA=%u", sysConfig->imu_data);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "IMU_POSE=%u", sysConfig->imu_pose);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "IMU_FREQ=%u", sysConfig->imu_freq);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "IMU_AXIS=%u", sysConfig->imu_axis);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "IMU_ARM=%.3f,%.3f,%.3f",
			 sysConfig->imu_arm[0], sysConfig->imu_arm[1], sysConfig->imu_arm[2]);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "IMU_ROT=%.3f,%.3f,%.3f",
			 sysConfig->imu_rot[0], sysConfig->imu_rot[1], sysConfig->imu_rot[2]);
	AT_SendResponse(buf);

	// gnss_setting
	if (sysConfig->gnss_type == 0)
	{
		snprintf(buf, sizeof(buf), "GNSS_TYPE=%s", "NMEA");
	}
	else
	{
		snprintf(buf, sizeof(buf), "GNSS_TYPE=%s", "UNICORE");
	}
	AT_SendResponse(buf);

	snprintf(buf, sizeof(buf), "GNSS_ARM=%.3f,%.3f,%.3f",
			 sysConfig->gnss_arm[0], sysConfig->gnss_arm[1], sysConfig->gnss_arm[2]);
	AT_SendResponse(buf);

	snprintf(buf, sizeof(buf), "CLUB_ARM=%.3f,%.3f,%.3f",
			 sysConfig->club_arm[0], sysConfig->club_arm[1], sysConfig->club_arm[2]);
	AT_SendResponse(buf);

	AT_SendResponse("******** Qurey Config End ********");
}

// 设置波特率
void AT_BAUD_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		// 查询当前波特率
		char buf[64];
		if (strcmp(cmd, "BAUD1") == 0)
		{
			snprintf(buf, sizeof(buf), "BAUD1=%u", sysConfig->baud1);
		}
		else if (strcmp(cmd, "BAUD2") == 0)
		{
			snprintf(buf, sizeof(buf), "BAUD2=%u", sysConfig->baud2);
		}
		else if (strcmp(cmd, "BAUD3") == 0)
		{
			snprintf(buf, sizeof(buf), "BAUD3=%u", sysConfig->baud3);
		}
		AT_SendResponse(buf);
	}
	else
	{
		// 设置波特率
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing baud rate");
			return;
		}

		uint32_t baud = atoi(params[0]);
		if (baud < 4800 || baud > 921600)
		{
			AT_SendResponse("ERROR: Baud rate out of range (4800-921600)");
			return;
		}
		if (strcmp(cmd, "BAUD1") == 0)
		{
			sysConfig->baud1 = baud;
		}
		else if (strcmp(cmd, "BAUD2") == 0)
		{
			sysConfig->baud2 = baud;
		}
		else if (strcmp(cmd, "BAUD3") == 0)
		{
			sysConfig->baud3 = baud;
		}
		else
		{
			AT_SendResponse("ERROR: Invalid BAUD command");
			return;
		}

		AT_SendResponse("OK");
	}
}

// 设置算法模式
void AT_MODE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "MODE=%u", sysConfig->mode);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing mode parameter");
			return;
		}
		uint8_t mode = atoi(params[0]);
		if (mode > 4)
		{ // 最大模式为4
			AT_SendResponse("ERROR: Invalid mode (0-4)");
			return;
		}
		sysConfig->mode = mode;
		AT_SendResponse("OK");
	}
}

// 设置IMU频率
void AT_IMUFREQ_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "IMU_FREQ=%u", sysConfig->imu_freq);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing frequency parameter");
			return;
		}
		int freq_val = atoi(params[0]);
		if (freq_val < 1 || freq_val > 255)
		{ // 最大255Hz due to uint8_t limit
			AT_SendResponse("ERROR: Frequency out of range (1-255)");
			return;
		}
		sysConfig->imu_freq = (uint8_t)freq_val;
		AT_SendResponse("OK");
	}
}

// 设置坐标轴定义
void AT_AXIS_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "IMU_AXIS=%u", sysConfig->imu_axis);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing axis definition");
			return;
		}
		uint8_t axis = atoi(params[0]);
		// 验证轴定义是否有效
		if (axis > 2)
		{ // 假设有3种定义
			AT_SendResponse("ERROR: Invalid axis definition");
			return;
		}
		sysConfig->imu_axis = axis;
		AT_SendResponse("OK");
	}
}

// 设置IMU安装角
void AT_IMU_ROT_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "IMU_ROT=%.3f,%.3f,%.3f",
				 sysConfig->imu_rot[0], sysConfig->imu_rot[1], sysConfig->imu_rot[2]);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 2)
		{
			AT_SendResponse("ERROR: Need roll, pitch, yaw angles");
			return;
		}
		float roll = atof(params[0]);  // X轴旋转角 (横滚角)
		float pitch = atof(params[1]); // Y轴旋转角 (俯仰角)
		float yaw = 0.0f;
		if (param_count >= 3)
		{
			yaw = atof(params[2]); // Z轴旋转角 (偏航角)
		}

		// 角度范围检查 (-180° 到 +180°)
		if (roll < -180.0f || roll > 180.0f ||
			pitch < -180.0f || pitch > 180.0f ||
			yaw < -180.0f || yaw > 180.0f)
		{
			AT_SendResponse("ERROR: Angles out of range (-180 to +180)");
			return;
		}

		sysConfig->imu_rot[0] = roll;
		sysConfig->imu_rot[1] = pitch;
		sysConfig->imu_rot[2] = yaw;
		AT_SendResponse("OK");
	}
}

// 设置GNSS杆臂
void AT_GNSSARM_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "GNSS_ARM=%.3f,%.3f,%.3f",
				 sysConfig->gnss_arm[0], sysConfig->gnss_arm[1], sysConfig->gnss_arm[2]);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 3)
		{
			AT_SendResponse("ERROR: Need x,y,z coordinates");
			return;
		}
		float x = atof(params[0]);
		float y = atof(params[1]);
		float z = atof(params[2]);
		sysConfig->gnss_arm[0] = x;
		sysConfig->gnss_arm[1] = y;
		sysConfig->gnss_arm[2] = z;
		AT_SendResponse("OK");
	}
}

// 设置输出点杆臂
void AT_CLUB_ARM_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "CLUB_ARM=%.3f,%.3f,%.3f",
				 sysConfig->club_arm[0], sysConfig->club_arm[1], sysConfig->club_arm[2]);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 3)
		{
			AT_SendResponse("ERROR: Need x,y,z coordinates");
			return;
		}
		float x = atof(params[0]);
		float y = atof(params[1]);
		float z = atof(params[2]);
		sysConfig->club_arm[0] = x;
		sysConfig->club_arm[1] = y;
		sysConfig->club_arm[2] = z;
		AT_SendResponse("OK");
	}
}

// 设置重力加速度
void AT_GRAVITY_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "GRAVITY=%.5f", sysConfig->gravity);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing gravity value");
			return;
		}
		float gravity = atof(params[0]);
		if (gravity < 9.0f || gravity > 10.0f)
		{
			AT_SendResponse("ERROR: Gravity out of range (9.0-10.0)");
			return;
		}
		sysConfig->gravity = gravity;
		AT_SendResponse("OK");
	}
}

// 设置GNSS跳秒参数
void AT_GNSS_LEAP_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "GNSS_LEAP=%u", sysConfig->gps_leap);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count < 1)
		{
			AT_SendResponse("ERROR: Missing leap seconds value");
			return;
		}
		int leap_seconds = atoi(params[0]);
		if (leap_seconds < 0 || leap_seconds > 50)
		{
			AT_SendResponse("ERROR: Leap seconds out of range (0-50)");
			return;
		}
		sysConfig->gps_leap = (uint8_t)leap_seconds;
		AT_SendResponse("OK");
	}
}

// 查询固件版本
void AT_VER_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	char buf[64];
	snprintf(buf, sizeof(buf), "VERSION=%s", FIRMWARE_VERSION_STRING);
	AT_SendResponse(buf);
}

// 查询设备状态
void AT_STATUS_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	char buf[128];

	// 基本设备状态
	AT_SendResponse("STATUS=RUNING");

#ifdef PLATFORM_MCU
// MCU内存和消耗信息 (不依赖FreeRTOS)

// 获取系统时钟频率 (HC32F460)
#ifdef SystemCoreClock
	snprintf(buf, sizeof(buf), "CPU_FREQ=%u", (unsigned int)SystemCoreClock);
	AT_SendResponse(buf);
#endif

	// 获取系统运行时间 (使用系统时钟计数器)
	extern volatile uint32_t g_system_tick_ms; // 假设有全局毫秒计数器
#ifdef g_system_tick_ms
	snprintf(buf, sizeof(buf), "UPTIME_MS=%u", g_system_tick_ms);
	AT_SendResponse(buf);
	snprintf(buf, sizeof(buf), "UPTIME_SEC=%u", g_system_tick_ms / 1000);
	AT_SendResponse(buf);
#endif

// HC32F460 Flash和SRAM规格信息
#ifdef HC32F460JEUA
	// HC32F460JEUA: 512KB Flash, 188KB SRAM
	AT_SendResponse("FLASH_SIZE=512KB");
	AT_SendResponse("SRAM_SIZE=188KB");
#elif defined(HC32F460KETA)
	// HC32F460KETA: 256KB Flash, 188KB SRAM
	AT_SendResponse("FLASH_SIZE=256KB");
	AT_SendResponse("SRAM_SIZE=188KB");
#elif defined(HC32F460PETA)
	// HC32F460PETA: 512KB Flash, 188KB SRAM
	AT_SendResponse("FLASH_SIZE=512KB");
	AT_SendResponse("SRAM_SIZE=188KB");
#else
	// 默认HC32F460规格
	AT_SendResponse("FLASH_SIZE=512KB");
	AT_SendResponse("SRAM_SIZE=188KB");
#endif

	// 简单的堆栈信息 (基于链接器符号)
	extern char _heap_start[];	// 堆起始地址
	extern char _heap_end[];	// 堆结束地址
	extern char _stack_start[]; // 栈起始地址
	extern char _stack_end[];	// 栈结束地址

#ifdef _heap_start
	uint32_t heapSize = (uint32_t)_heap_end - (uint32_t)_heap_start;
	snprintf(buf, sizeof(buf), "HEAP_SIZE=%u", heapSize);
	AT_SendResponse(buf);
#endif

#ifdef _stack_start
	uint32_t stackSize = (uint32_t)_stack_end - (uint32_t)_stack_start;
	snprintf(buf, sizeof(buf), "STACK_SIZE=%u", stackSize);
	AT_SendResponse(buf);
#endif
#else
	// 非MCU平台的模拟信息
#endif
}

// 调试设置
void AT_DEBUG_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "DEBUG=%u,%u,%u",
				 sysConfig->debug_enable, sysConfig->print_imu, sysConfig->print_gnss);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->debug_enable = 1;
				printf("DEBUG: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->debug_enable = 0;
				printf("DEBUG: 0\n");
			}
		}

		if (param_count >= 2 && params[1])
		{
			if (strcmp(params[1], "1") == 0)
			{
				sysConfig->print_imu = 1;
				printf("PRINT_IMU: 1\n");
			}
			else if (strcmp(params[1], "0") == 0)
			{
				sysConfig->print_imu = 0;
				printf("PRINT_IMU: 0\n");
			}
		}

		if (param_count >= 3 && params[2])
		{
			if (strcmp(params[2], "1") == 0)
			{
				sysConfig->print_gnss = 1;
				printf("PRINT_GNSS: 1\n");
			}
			else if (strcmp(params[2], "0") == 0)
			{
				sysConfig->print_gnss = 0;
				printf("PRINT_GNSS: 0\n");
			}
		}

		AT_SendResponse("OK");
	}
}

// IMU数据输出控制
void AT_IMU_DATA_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "IMU_DATA=%u", sysConfig->imu_data);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->imu_data = 1;
				printf("IMU_DATA: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->imu_data = 0;
				printf("IMU_DATA: 0\n");
			}
		}
		AT_SendResponse("OK");
	}
}

// IMU姿态输出控制
void AT_IMU_POSE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "IMU_POSE=%u", sysConfig->imu_pose);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->imu_pose = 1;
				printf("IMU_POSE: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->imu_pose = 0;
				printf("IMU_POSE: 0\n");
			}
		}
		AT_SendResponse("OK");
	}
}

// GNSS类型控制
void AT_GNSS_TYPE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		// 查询指令：返回当前状态或帮助信息
		char buf[32];
		if (sysConfig->gnss_type == GNSS_TYPE_NMEA)
		{
			/* code */
			snprintf(buf, sizeof(buf), "GNSS_TYPE=%s", "NMEA");
		}
		else if (sysConfig->gnss_type == GNSS_TYPE_UNICORE)
		{
			/* code */
			snprintf(buf, sizeof(buf), "GNSS_TYPE=%s", "UNICORE");
		}
		else
		{
			snprintf(buf, sizeof(buf), "GNSS_TYPE=%s", "UNKNOWN");
		}
		AT_SendResponse(buf);
	}
	else
	{
		uint8_t *pu8Buff = (uint8_t *)"\r\nlog gpgga ontime 0.1\r\n log gprmc ontime 0.1\r\n log gpgst ontime 0.1\r\n";
		uint16_t u16Len = strlen((char *)pu8Buff);

		// 设置指令：向UART3发送GNSS日志命令
#ifdef PLATFORM_MCU
		USART_UART_Trans(CM_USART3, pu8Buff, u16Len, 0xFFFFFFFFUL);
#endif
		AT_SendResponse("OK");
	}
}

void AT_GNSS_DATA_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "GNSS_DATA=%u", sysConfig->gnss_data);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->gnss_data = 1;
				printf("GNSS_DATA: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->gnss_data = 0;
				printf("GNSS_DATA: 0\n");
			}
			else
			{
				AT_SendResponse("ERROR");
				return;
			}
			AT_SendResponse("OK");
		}
		else
		{
			AT_SendResponse("ERROR");
		}
	}
}

void AT_GNSS_POSE_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "GNSS_POSE=%u", sysConfig->gnss_pose);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->gnss_pose = 1;
				printf("GNSS_POSE: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->gnss_pose = 0;
				printf("GNSS_POSE: 0\n");
			}
			else
			{
				AT_SendResponse("ERROR");
				return;
			}

			AT_SendResponse("OK");
		}
		else
		{
			AT_SendResponse("ERROR");
		}
	}
}

void AT_GNSS_PPS_Handler(const char *cmd, const char **params, uint8_t param_count, uint8_t is_query)
{
	if (is_query)
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "GNSS_PPS=%u", sysConfig->gnss_pps);
		AT_SendResponse(buf);
	}
	else
	{
		if (param_count >= 1 && params[0])
		{
			if (strcmp(params[0], "1") == 0)
			{
				sysConfig->gnss_pps = 1;
				printf("GNSS_PPS: 1\n");
			}
			else if (strcmp(params[0], "0") == 0)
			{
				sysConfig->gnss_pps = 0;
				printf("GNSS_PPS: 0\n");
			}
			else
			{
				AT_SendResponse("ERROR");
				return;
			}

			AT_SendResponse("OK");
		}
		else
		{
			AT_SendResponse("ERROR");
		}
	}
}

/**************************** 外部调用接口 ******************************/

// 命令注册表
static const AT_CmdRegistry_t cmdRegistry[] = {

	// 查询调试
	{"DEBUG", AT_DEBUG_Handler},   // AT+DEBUG
	{"CONFIG", AT_CONFIG_Handler}, // AT+CONFIG
	{"VERSION", AT_VER_Handler},   // AT+VERSION
	{"STATUS", AT_STATUS_Handler}, // AT+STATUS

	// 系统指令
	{"RESET", AT_RESET_Handler},	 // AT+RESET
	{"REBOOT", AT_REBOOT_Handler},	 // AT+REBOOT
	{"SAVE", AT_SAVE_Handler},		 // AT+SAVE
	{"UPGRADE", AT_UPGRADE_Handler}, // AT+UPGRADE

	// 配置指令
	{"MODE", AT_MODE_Handler},		   // AT+MODE
	{"BAUD1", AT_BAUD_Handler},		   // AT+BAUD1
	{"BAUD2", AT_BAUD_Handler},		   // AT+BAUD2
	{"BAUD3", AT_BAUD_Handler},		   // AT+BAUD3
	{"IMU_DATA", AT_IMU_DATA_Handler}, // AT+IMU_DATA
	{"IMU_POSE", AT_IMU_POSE_Handler}, // AT+IMU_POSE
	{"IMU_FREQ", AT_IMUFREQ_Handler},  // AT+IMU_FREQ
	{"IMU_AXIS", AT_AXIS_Handler},	   // AT+IMU_AXIS
	{"IMU_ROT", AT_IMU_ROT_Handler},   // AT+IMU_ROT

	{"GNSS_ARM", AT_GNSSARM_Handler},	 // AT+GNSS_ARM
	{"GNSS_LEAP", AT_GNSS_LEAP_Handler}, // AT+GNSS_LEAP
	{"GNSS_TYPE", AT_GNSS_TYPE_Handler}, // AT+GNSS_TYPE
	{"GNSS_DATA", AT_GNSS_DATA_Handler}, // AT+GNSS_DATA
	{"GNSS_POSE", AT_GNSS_POSE_Handler}, // AT+GNSS_POSE
	{"GNSS_PPS", AT_GNSS_PPS_Handler},	 // AT+GNSS_PPS
	{"CLUB_ARM", AT_CLUB_ARM_Handler},	 // AT+CLUB_ARM
	{"GRAVITY", AT_GRAVITY_Handler},	 // AT+GRAVITY
};

/**
 * @brief 解析并处理AT指令
 * @param cmd 输入的AT指令字符串
 * @param len 输入的AT指令字符串长度
 * @return uint8_t 处理结果
 */
uint8_t algo_at_command(char *cmd, uint32_t len)
{
	sysConfig = get_user_params();

	uint32_t trm_len = trim_all(cmd);
	if (!(trm_len >= 2 && cmd[0] == 'A' && cmd[1] == 'T'))
	{
		printf("error cmd(%d): %s\n", trm_len, cmd);
		return 1;
	}

	at_command_t cmd_t;
	int32_t result = parse_at_command(cmd, &cmd_t);

	if (result != 0)
	{
		printf("parse_at_command failed\n");
		return 2;
	}
	else
	{
		if (sysConfig->debug_enable)
		{
			print_at_command(&cmd_t);
		}
	}

	// 特殊处理：单独的AT指令
	if (strlen(cmd_t.command) == 0)
	{
		AT_SendResponse("OK");
		return 0;
	}

	// 使用注册表查找命令处理器
	// 准备参数数组
	const char *params[AT_MAX_PARAMS] = {
		cmd_t.value1[0] ? cmd_t.value1 : NULL,
		cmd_t.value2[0] ? cmd_t.value2 : NULL,
		cmd_t.value3[0] ? cmd_t.value3 : NULL,
		cmd_t.value4[0] ? cmd_t.value4 : NULL,
		cmd_t.value5[0] ? cmd_t.value5 : NULL,
		cmd_t.value6[0] ? cmd_t.value6 : NULL};

	// 计算参数数量
	uint8_t param_count = 0;
	for (int i = 0; i < AT_MAX_PARAMS; i++)
	{
		if (params[i] != NULL)
			param_count++;
		else
			break;
	}

	// 判断是查询还是设置
	uint8_t is_query = (cmd_t.operator != '=');

	// 查找命令处理器
	uint8_t registrySize = sizeof(cmdRegistry) / sizeof(cmdRegistry[0]);
	uint8_t handled = 0;
	for (int i = 0; i < registrySize; i++)
	{
		if (strcmp(cmd_t.command, cmdRegistry[i].cmd) == 0)
		{
			if (cmdRegistry[i].handler != NULL)
			{
				cmdRegistry[i].handler(cmd_t.command, params, param_count, is_query);
				handled = 1;
				break;
			}
		}
	}

	if (!handled)
	{
		char response[64];
		snprintf(response, sizeof(response), "ERROR: Unknown command 'AT+%s'", cmd_t.command);
		AT_SendResponse(response);
		return 3;
	}

	return 0;
}

/**
 * @brief 处理AT指令
 * @param text 输入的AT指令字符串
 * @param len 输入的AT指令字符串长度
 * @return uint8_t 处理结果
 */
uint8_t algo_at_process(char *text, uint32_t len)
{
	sysConfig = get_user_params();

	// 遍历输入数据
	for (uint32_t i = 0; i < len; i++)
	{
		char c = text[i];

		// 检查缓存区是否已满
		if (cmdIndex >= MAX_CMD_LEN - 1)
		{
			// 缓存区溢出，清空重新开始
			cmdIndex = 0;
			printf("Error: AT command buffer overflow!\n");
		}

		// 添加字符到缓存
		cmdBuffer[cmdIndex++] = c;

		// 检查是否收到 \r\n 结束符
		if (cmdIndex >= 2 && cmdBuffer[cmdIndex - 2] == '\r' && cmdBuffer[cmdIndex - 1] == '\n')
		{
			// 找到完整的AT指令，去掉 \r\n 并添加字符串结束符
			cmdBuffer[cmdIndex - 2] = '\0';

			// 调用 algo_at_command 处理这条指令
			if (cmdIndex - 2 > 0)
			{
				algo_at_command(cmdBuffer, cmdIndex - 2);
			}

			// 重置缓存索引
			cmdIndex = 0;
		}
	}

	return 0;
}

/*
********************************************************************************
*                           AT指令测试命令集                                    *
********************************************************************************
以下为所有AT指令的测试命令示例，可直接复制到串口工具中进行测试

// 基础测试
AT
AT?

// 查询调试指令
AT+DEBUG?
AT+CONFIG?
AT+VERSION?
AT+STATUS?

// 系统指令
AT+RESET
AT+REBOOT
AT+SAVE
AT+UPGRADE

// 调试设置指令
AT+DEBUG=1,1,1
AT+DEBUG=0,0,0

// 配置指令测试
AT+MODE?
AT+MODE=1
AT+MODE=2

AT+BAUD1?
AT+BAUD1=460800
AT+BAUD1=9600

AT+BAUD2?
AT+BAUD2=460800
AT+BAUD2=57600

AT+BAUD3?
AT+BAUD3=460800
AT+BAUD3=921600

// IMU相关指令
AT+IMU_DATA?
AT+IMU_DATA=1
AT+IMU_DATA=0

AT+IMU_POSE?
AT+IMU_POSE=1
AT+IMU_POSE=0

AT+IMU_FREQ?
AT+IMU_FREQ=100
AT+IMU_FREQ=200

AT+IMU_AXIS?
AT+IMU_AXIS=0
AT+IMU_AXIS=1

AT+IMU_ROT?
AT+IMU_ROT=0.0,0.0,0.0
AT+IMU_ROT=1.5,-2.3,0.8
AT+IMU_ROT=90.0,45.0,-30.0

// GNSS相关指令
AT+GNSS_ARM?
AT+GNSS_ARM=0.023,0.045,0.876
AT+GNSS_ARM=0.0,0.0,0.0

AT+GNSS_LEAP?
AT+GNSS_LEAP=18
AT+GNSS_LEAP?
AT+GNSS_LEAP=19
AT+GNSS_LEAP?

// 杆臂设置指令
AT+CLUB_ARM?
AT+CLUB_ARM=0.000,0.000,2.098
AT+CLUB_ARM?
AT+CLUB_ARM=0.123,0.456,1.789
AT+CLUB_ARM?

// 重力加速度指令
AT+GRAVITY?
AT+GRAVITY=9.80665
AT+GRAVITY?
AT+GRAVITY=9.81234
AT+GRAVITY?
AT+GRAVITY=9.78033
AT+GRAVITY?

// 错误测试示例（应返回错误）
AT+GRAVITY=11.0              // 超出范围
AT+GNSS_LEAP=60              // 超出范围
AT+IMU_ROT=200.0,0.0,0.0     // 角度超出范围
AT+BAUD1=1200                // 波特率超出范围
AT+UNKNOWN_CMD               // 未知指令
AT+IMU_ROT=1.0,2.0           // 参数不足

********************************************************************************
*/
