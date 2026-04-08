#include "nmea_data.h"
#include "algo_datetime.h"
#include "setting.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#define NMEA_BUFFER_MAX 512

// NMEA解析器上下文
typedef struct
{
	char buffer[NMEA_BUFFER_MAX]; // 接收缓冲区
	size_t index;				  // 当前缓冲区位置
	// size_t start_index;			  // 当前消息起始位置
	uint8_t in_message;

	char msg_gga[256]; // gga接收缓冲区
	char msg_rmc[256]; // rmc接收缓冲区
	char msg_gst[256]; // gst接收缓冲区
	uint8_t has_gga;   // 是否在消息中
	uint8_t has_gst;   // 是否在消息中
	uint8_t has_rmc;   // 是否在消息中
} NMEA_Parser_t;

// 全局解析器实例
static NMEA_Parser_t nmea_parser = {0};

// 初始化NMEA解析器
void nmea_data_init(void)
{
	memset(&nmea_parser, 0, sizeof(nmea_parser));
}

// 获取NMEA消息类型
static NMEA_Type_t get_nmea_type(const char *message)
{

	if (strncmp(message + 1, "GPGGA", 5) == 0)
		return NMEA_GPGGA;
	if (strncmp(message + 1, "GPRMC", 5) == 0)
		return NMEA_GPRMC;
	if (strncmp(message + 1, "GPGST", 5) == 0)
		return NMEA_GPGST;

	if (strncmp(message + 1, "GNGGA", 5) == 0)
		return NMEA_GPGGA;
	if (strncmp(message + 1, "GNRMC", 5) == 0)
		return NMEA_GPRMC;
	if (strncmp(message + 1, "GNGST", 5) == 0)
		return NMEA_GPGST;

	return NMEA_UNKNOWN;
}

// 获取缓存中剩余的数据
const char *nmea_get_remaining_data(void)
{
	if (nmea_parser.index > 0)
	{
		nmea_parser.buffer[nmea_parser.index] = '\0';
		return nmea_parser.buffer;
	}
	return NULL;
}

// 清除缓存
void nmea_clear_buffer(void)
{
	nmea_parser.index = 0;
	nmea_parser.in_message = 0;
}

/**
 * 计算NMEA语句的校验和
 * @param sentence NMEA语句（以$开头，可能包含*和校验和）
 * @return 计算得到的校验和值（0-255）
 */
unsigned char nmea_checksum(const char *sentence)
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
 * 验证NMEA语句的校验和
 * @param sentence 完整的NMEA语句（包含$、*和校验和）
 * @return 校验和是否正确（1=正确，0=错误）
 */
int nmea_verify_checksum(const char *sentence)
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
	unsigned char calculated_checksum = nmea_checksum(sentence);

	// 比较校验和
	return calculated_checksum == received_checksum;
}

void nmea_split_decimal_string(const uint8_t *str, uint32_t *int_part, uint32_t *frac_part)
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

// ======================== 解码函数 ========================

// 辅助函数：解析度分格式为十进制
static double nmea_parse_degrees(const char *str)
{
	double deg = atof(str);
	double minutes = fmod(deg, 100.0);
	double degrees = trunc(deg / 100.0);
	return degrees + minutes / 60.0;
}

// GPGGA解码
bool gpgga_decode(const char *nmea, GPGGA_t *out)
{
	char fields[15][16] = {0}; // 15个字段，每字段最长15字符
	int field_count = sscanf(nmea, "$%6[^,],%9[^,],%15[^,],%c,%15[^,],%c,%hhu,%hhu,%f,%f,%c,%f,%c,%f,%hu",
							 out->type, fields[1], fields[2], &out->latDirection,
							 fields[4], &out->lonDirection, &out->fixQuality, &out->satellitesUsed,
							 &out->hdop, &out->altitude, &out->altUnits, &out->geoidalSeparation, &out->geoSepUnits,
							 &out->diffCorrAge, &out->refStationID);

	// 最少需要13个有效字段
	// if (field_count < 13)	return false;

	// 解析经纬度
	out->latitude = nmea_parse_degrees(fields[2]);
	out->longitude = nmea_parse_degrees(fields[4]);

	// 复制时间
	strncpy(out->time, fields[1], sizeof(out->time) - 1);
	out->time[sizeof(out->time) - 1] = '\0';

	// 方向有效性检查
	if (out->latDirection != 'N' && out->latDirection != 'S')
		return false;
	if (out->lonDirection != 'E' && out->lonDirection != 'W')
		return false;

	return true;
}

// GPRMC解码
bool gprmc_decode(const char *nmea, GPRMC_t *out)
{
	char fields[13][16] = {0};
	int field_count = sscanf(nmea, "$%6[^,],%9[^,],%c,%15[^,],%c,%15[^,],%c,%f,%f,%6[^,],%f,%c,%c",
							 out->type, out->time, &out->status,
							 fields[3], &out->latDirection, fields[5], &out->lonDirection,
							 &out->speedKnots, &out->courseTrue, out->date,
							 &out->magneticVariation, &out->magVarDirection, &out->modeIndicator);

	if (field_count < 12)
		return false;

	out->latitude = nmea_parse_degrees(fields[3]);
	out->longitude = nmea_parse_degrees(fields[5]);

	// 有效性检查
	if (out->status != 'A' && out->status != 'V')
		return false;
	if (out->magVarDirection != 'E' && out->magVarDirection != 'W')
		return false;

	return true;
}

// GPGST解码
bool gpgst_decode(const char *nmea, GPGST_t *out)
{
	int field_count = sscanf(nmea, "$%6[^,],%9[^,],%f,%f,%f,%f,%f,%f,%f",
							 out->type, out->time, &out->rangeRms,
							 &out->stdMajor, &out->stdMinor, &out->orientation,
							 &out->stdLat, &out->stdLong, &out->stdAlt);
	return (field_count == 9);
}

// ======================== 编码函数 ========================

// 辅助函数：十进制转度分格式
static void nmea_format_degrees(char *buffer, size_t size, double decimalDeg, char direction)
{
	double degrees = fabs(decimalDeg);
	int degInt = (int)degrees;
	double minutes = (degrees - degInt) * 60.0;
	double value = degInt * 100 + minutes;
	snprintf(buffer, size, "%09.4f,%c", value, direction);
}

// GPGGA编码
size_t gpgga_encode(const GPGGA_t *data, char *buffer, size_t size)
{
	char latStr[16], lonStr[16];
	nmea_format_degrees(latStr, sizeof(latStr), data->latitude, data->latDirection);
	nmea_format_degrees(lonStr, sizeof(lonStr), data->longitude, data->lonDirection);

	int len = snprintf(buffer, size, "$%s,%s,%s,%s,%hhu,%hhu,%.1f,%.1f,%c,%.1f,%c,%.1f,%hu",
					   data->type, data->time, latStr, lonStr,
					   data->fixQuality, data->satellitesUsed, data->hdop,
					   data->altitude, data->altUnits, data->geoidalSeparation,
					   data->geoSepUnits, data->diffCorrAge, data->refStationID);

	// 计算并添加校验和
	if (len > 0 && len < size)
	{
		uint8_t checksum = 0;
		for (int i = 1; i < len; i++)
		{ // 跳过'$'
			checksum ^= buffer[i];
		}
		snprintf(buffer + len, size - len, "*%02X\r\n", checksum);
		return strlen(buffer);
	}
	return 0;
}

// GPRMC编码
size_t gprmc_encode(const GPRMC_t *data, char *buffer, size_t size)
{
	char latStr[16], lonStr[16];
	nmea_format_degrees(latStr, sizeof(latStr), data->latitude, data->latDirection);
	nmea_format_degrees(lonStr, sizeof(lonStr), data->longitude, data->lonDirection);

	int len = snprintf(buffer, size, "$%s,%s,%c,%s,%s,%c,%.1f,%.1f,%s,%.1f,%c,%c",
					   data->type, data->time, data->status,
					   latStr, lonStr, data->lonDirection, data->speedKnots,
					   data->courseTrue, data->date, data->magneticVariation,
					   data->magVarDirection, data->modeIndicator);

	if (len > 0 && len < size)
	{
		uint8_t checksum = 0;
		for (int i = 1; i < len; i++)
		{
			checksum ^= buffer[i];
		}
		snprintf(buffer + len, size - len, "*%02X\r\n", checksum);
		return strlen(buffer);
	}
	return 0;
}

// GPGST编码
size_t gpgst_encode(const GPGST_t *data, char *buffer, size_t size)
{
	int len = snprintf(buffer, size, "$%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
					   data->type, data->time, data->rangeRms,
					   data->stdMajor, data->stdMinor, data->orientation,
					   data->stdLat, data->stdLong, data->stdAlt);

	if (len > 0 && len < size)
	{
		uint8_t checksum = 0;
		for (int i = 1; i < len; i++)
		{
			checksum ^= buffer[i];
		}
		snprintf(buffer + len, size - len, "*%02X\r\n", checksum);
		return strlen(buffer);
	}
	return 0;
}

// NMEA数据拆包函数
uint32_t nmea_parser_loop(const uint8_t *data, uint32_t len)
{
	// printf("nmea_parser_loop: %s, %d\r\n", data, len);
	const uint8_t *message = data;
	NMEA_Type_t type = get_nmea_type(message);
	switch (type)
	{
	case NMEA_GPGGA:
	{
		nmea_parser.has_gga = 1;
		memset(nmea_parser.msg_gga, 0, sizeof(nmea_parser.msg_gga));
		memcpy(nmea_parser.msg_gga, message, len);
		// printf("nmea_parser_loop(%d): %s\r\n", len,nmea_parser.msg_gga);
	}
	break;
	case NMEA_GPRMC:
	{
		nmea_parser.has_rmc = 1;
		memset(nmea_parser.msg_rmc, 0, sizeof(nmea_parser.msg_rmc));
		memcpy(nmea_parser.msg_rmc, message, len);
		// printf("nmea_parser_loop(%d): %s\r\n", len,nmea_parser.msg_rmc);
	}
	break;
	case NMEA_GPGST:
	{
		nmea_parser.has_gst = 1;
		memset(nmea_parser.msg_gst, 0, sizeof(nmea_parser.msg_gst));
		memcpy(nmea_parser.msg_gst, message, len);
		// printf("nmea_parser_loop(%d): %s\r\n", len,nmea_parser.msg_gst);
	}
	break;
	default:
	{
		printf("nmea_parser_loop: unknown type(%s)\r\n", message);
	}
	break;
	}

	user_params_t *p = get_user_params();
	uint32_t flag = 0;
	if (type == p->gnss_last)
	{
		flag = 1;
	}
	else
	{
		flag = 0;
	}

	return flag;
}

uint32_t gnss_index = 0;
uint32_t nmea_parser_get(gnss_data_t *gnss)
{

	if (!nmea_parser.has_gga || !nmea_parser.has_rmc)
	{
		return 1;
	}

	GPGGA_t *gga = &gnss->gga;
	GPRMC_t *rmc = &gnss->rmc;
	GPGST_t *gst = &gnss->gst;

	if (nmea_parser.has_gga)
	{
		char *message = nmea_parser.msg_gga;

		// 解析GPGGA消息
		// GPGGA_t gga;
		memset(gga, 0, sizeof(*gga));
		gpgga_decode(message, gga);

		// 打印GPGGA数据
		if (0)
			printf("********: %s, %s, %f, %c, %f, %c, %hhu, %hhu, %.1f, %.1f, %c, %.1f, %hu\n",
				   gga->type, gga->time, gga->latitude, gga->latDirection,
				   gga->longitude, gga->lonDirection, gga->fixQuality,
				   gga->satellitesUsed, gga->hdop, gga->altitude,
				   gga->altUnits, gga->geoidalSeparation, gga->geoSepUnits,
				   gga->diffCorrAge, gga->refStationID);
	}

	if (nmea_parser.has_rmc)
	{
		char *message = nmea_parser.msg_rmc;

		// 解析GPRMC消息
		// GPRMC_t rmc;
		memset(rmc, 0, sizeof(*rmc));
		gprmc_decode(message, rmc);

		// 打印GPRMC数据
		if (0)
			printf("********: %s, %s, %c, %f, %c, %f, %c, %.1f, %.1f, %s, %.1f, %c, %c\n",
				   rmc->type, rmc->time, rmc->status,
				   rmc->latitude, rmc->latDirection,
				   rmc->longitude, rmc->lonDirection,
				   rmc->speedKnots, rmc->courseTrue,
				   rmc->date, rmc->magneticVariation,
				   rmc->magVarDirection, rmc->modeIndicator);
	}

	if (nmea_parser.has_gst)
	{
		char *message = nmea_parser.msg_gst;

		// 解析GPGST消息
		// GPGST_t gst;
		memset(gst, 0, sizeof(*gst));
		gpgst_decode(message, gst);

		// 打印GPGST数据
		if (0)
			printf("********: %s, %s, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n",
				   gst->type, gst->time, gst->rangeRms,
				   gst->stdMajor, gst->stdMinor, gst->orientation,
				   gst->stdLat, gst->stdLong, gst->stdAlt);
	}

	gnss->index = gnss_index++;
	gnss->time_sync = 1;

	datetime_t dt;
	int result = parse_datetime(gnss->rmc.time, gnss->rmc.date, &dt);
	if (result == 0)
	{
		gtime_t time = epoch2time(&dt);
		gnss->time_sec_s = time.sec_s;
		gnss->time_sec_ms = time.sec_ms;
	}
	else
	{
		printf("parse_datetime failed!\n");
	}

	nmea_parser.has_gga = 0;
	nmea_parser.has_rmc = 0;
	nmea_parser.has_gst = 0;

	return 0;
}

// 测试语句
//$GPGGA,044744.00,3122.4658,N,12025.2791,E,1,10,3.00,12.575,M,7.100,M,00,0000*5F
//$GPRMC,044838.00,A,3122.4658,N,12025.2799,E,0.257,261.7,180921,0.0,E,A*3B
//$GPGST,123519,0.7,0.8,0.9,1.2,4.5,6,0.9,3.8,M,4*4E