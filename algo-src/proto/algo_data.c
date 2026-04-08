#include "algo_data.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// 十六进制字符数组
const char hex_chars[] = "0123456789ABCDEF";

/**
 * 计算语句的校验和
 * @param sentence NMEA语句（以$开头，可能包含*和校验和）
 * @return 计算得到的校验和值（0-255）
 */
uint8_t algo_checksum(const uint8_t *sentence)
{
	unsigned char checksum = 0;

	// 跳过开头的'$'（如果存在）
	if (*sentence == '$')
	{
		sentence++;
	}

	// 计算直到'*'或字符串结束的所有字符的异或
	while (*sentence && *sentence != '*')
	{
		checksum ^= *sentence;
		sentence++;
	}

	return checksum;
}

/**
 * 计算语句的校验和
 * @param sentence NMEA语句（以$开头，可能包含*和校验和）
 * @return 计算得到的校验和值（0-255）
 */
uint8_t algo_add_checksum(uint8_t *sentence)
{
	// 查找语句中的'*'字符
	char *asterisk = strchr(sentence, '*');
	if (!asterisk)
	{
		return 0; // 没有找到*或*后没有足够的字符
	}

	// 计算校验和
	uint8_t calculated_checksum = algo_checksum(sentence);

	*(asterisk + 1) = hex_chars[calculated_checksum / 16];
	*(asterisk + 2) = hex_chars[calculated_checksum % 16];
	*(asterisk + 3) = 0x0D;
	*(asterisk + 4) = 0x0A;
	*(asterisk + 5) = '\0';

	return 1;
}

/**
 * 验证语句的校验和
 * @param sentence 完整的NMEA语句（包含$、*和校验和）
 * @return 校验和是否正确（1=正确，0=错误）
 */
uint8_t algo_verify_checksum(const char *sentence)
{
	// 查找语句中的'*'字符
	const char *asterisk = strchr(sentence, '*');
	if (!asterisk || strlen(asterisk) < 3)
	{
		return 0; // 没有找到*或*后没有足够的字符
	}

	// 提取语句中的校验和（*后的两个十六进制字符）
	unsigned char received_checksum;
	if (sscanf(asterisk + 1, "%2hhx", &received_checksum) != 1)
	{
		return 0; // 无法解析校验和
	}

	// 计算校验和
	unsigned char calculated_checksum = algo_checksum(sentence);

	// 比较校验和
	return calculated_checksum == received_checksum;
}

/**
 * 优化的字符串分割函数，用于处理包含小数点的字符串
 * @param str 输入字符串
 * @param int_part 指向整数部分的指针
 * @param frac_part 指向小数部分的指针
 */
void split_decimal_string_optimized(const uint8_t *str, uint32_t *int_part, uint32_t *frac_part)
{
	*int_part = 0;
	*frac_part = 0;

	int is_fraction = 0;
	int fraction_digits = 0;

	for (int i = 0; str[i] != '\0'; i++)
	{
		if (str[i] == '.')
		{
			is_fraction = 1;
			continue;
		}

		if (isdigit(str[i]))
		{
			if (is_fraction)
			{
				*frac_part = *frac_part * 10 + (str[i] - '0');
				fraction_digits++;
			}
			else
			{
				*int_part = *int_part * 10 + (str[i] - '0');
			}
		}
	}
}

// IMU数据解码函数
int imu_data_decode(const uint8_t *data, uint32_t len, imu_data_t *imu_data)
{
	if (!data || !imu_data)
	{
		return 0;
	}

	uint8_t ck = algo_verify_checksum(data);
	if (!ck)
	{
		return 0;
	}

	int result = 0;

	// 检查数据头是否为ALGOIMU,使用Algomini调试用
	if (strncmp(data, "$ALGOIMU", 8) == 0)
	{

		uint32_t gps_week;
		char gps_week_sec[16];
		char sys_sec[16];
		memset(gps_week_sec, 0, sizeof(gps_week_sec));
		memset(sys_sec, 0, sizeof(sys_sec));

		result = sscanf(data, "$ALGOIMU,%u,%15[^,],%15[^,],%f,%f,%f,%f,%f,%f,%f",
						&gps_week,
						gps_week_sec,
						sys_sec,
						&imu_data->gyro_x,
						&imu_data->gyro_y,
						&imu_data->gyro_z,
						&imu_data->acc_x,
						&imu_data->acc_y,
						&imu_data->acc_z,
						&imu_data->temperature);

		uint32_t sec_s = 0, sec_ms = 0;
		split_decimal_string_optimized(gps_week_sec, &sec_s, &sec_ms);
		imu_data->time_week = gps_week;
		imu_data->time_sec_s = sec_s;
		imu_data->time_sec_ms = sec_ms;
		imu_data->temperature = 0.0;
		imu_data->time_sync = 1;
	}

	// 检查数据头是否为1010IMU,使用1010调试用
	if (strncmp(data, "$1010IMU", 8) == 0)
	{

		uint32_t gps_week;
		char gps_week_sec[16];
		char sys_sec[16];
		memset(gps_week_sec, 0, sizeof(gps_week_sec));
		memset(sys_sec, 0, sizeof(sys_sec));

		result = sscanf(data, "$1010IMU,%u,%15[^,],%15[^,],%f,%f,%f,%f,%f,%f,%f",
						&gps_week,
						gps_week_sec,
						sys_sec,
						&imu_data->gyro_x,
						&imu_data->gyro_y,
						&imu_data->gyro_z,
						&imu_data->acc_x,
						&imu_data->acc_y,
						&imu_data->acc_z,
						&imu_data->temperature);

		uint32_t sec_s = 0, sec_ms = 0;
		split_decimal_string_optimized(gps_week_sec, &sec_s, &sec_ms);
		imu_data->time_week = 0;
		imu_data->time_sec_s = sec_s;
		imu_data->time_sec_ms = sec_ms;
		imu_data->temperature = 0.0;
		imu_data->time_sync = 0;
	}

	// 检查数据头是否为AIMU
	if (strncmp(data, "$AIMU", 5) == 0)
	{

		uint32_t gps_week;
		char gps_week_sec[16];
		char sys_sec[16];
		memset(gps_week_sec, 0, sizeof(gps_week_sec));
		memset(sys_sec, 0, sizeof(sys_sec));

		result = sscanf(data, "$AIMU,%hhu,%15[^,],%f,%f,%f,%f,%f,%f,%f",
						&imu_data->time_sync,
						sys_sec,
						&imu_data->gyro_x,
						&imu_data->gyro_y,
						&imu_data->gyro_z,
						&imu_data->acc_x,
						&imu_data->acc_y,
						&imu_data->acc_z,
						&imu_data->temperature);

		uint32_t sec_s = 0, sec_ms = 0;
		split_decimal_string_optimized(sys_sec, &sec_s, &sec_ms);
		imu_data->time_week = 0;
		imu_data->time_sec_s = sec_s;
		imu_data->time_sec_ms = sec_ms;
	}

	return (result >= 9) ? 1 : 0;
}

// POSE数据解码函数
int pose_data_decode(const uint8_t *data, uint32_t len, pose_data_t *pose_data)
{
	return 0;
}

// IMU数据编码函数
int32_t imu_data_encode(uint8_t *data, uint32_t len, const imu_data_t *imu_data)
{
	if (!data || !imu_data)
		return 0;

	int32_t result = sprintf(data, "$AIMU,%hhu,%d.%03d,0,0,0,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%.1f*",
							 imu_data->time_sync,
							 imu_data->time_sec_s,
							 imu_data->time_sec_ms,
							 imu_data->gyro_x,
							 imu_data->gyro_y,
							 imu_data->gyro_z,
							 imu_data->acc_x,
							 imu_data->acc_y,
							 imu_data->acc_z,
							 imu_data->temperature);

	if (result <= 0 || result > len - 5)
		return 0;

	uint8_t res = algo_add_checksum(data);

	return res ? result + 5 : 0;
}

// POSE数据编码函数
int32_t pose_data_encode(uint8_t *data, uint32_t len, const pose_data_t *pose_data)
{
	return 0;
}