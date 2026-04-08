#ifndef NMEA_DATA_H
#define NMEA_DATA_H

#include <stdint.h>

// NMEA0183语句类型枚举
typedef enum
{
	NMEA_UNKNOWN = 0,
	NMEA_GPGGA = 1,
	NMEA_GPRMC = 2,
	NMEA_GPGST = 3
} NMEA_Type_t;

// GPGGA数据结构
typedef struct
{
	char type[6];
	char time[10];			  // hhmmss.ss格式，固定9字节
	double latitude;		  // 十进制纬度
	char latDirection;		  // 'N'或'S'
	double longitude;		  // 十进制经度
	char lonDirection;		  // 'E'或'W'
	uint8_t fixQuality;		  // 0-5
	uint8_t satellitesUsed;	  // 0-60
	float hdop;				  // 水平精度因子
	float altitude;		  // 高度(米)
	char altUnits;			  // 'M'
	float geoidalSeparation; // 大地水准面高度(米)
	char geoSepUnits;		  // 'M'
	float diffCorrAge;		  // 差分年龄(秒)
	uint16_t refStationID;	  // 参考站ID
} GPGGA_t;

// GPRMC数据结构
typedef struct
{
	char type[6];
	char time[10]; // hhmmss.ss格式
	char status;   // 'A'或'V'
	double latitude;
	char latDirection;
	double longitude;
	char lonDirection;
	float speedKnots;		 // 地面速度(节)
	float courseTrue;		 // 地面航向(真北度)
	char date[7];			 // ddmmyy格式
	float magneticVariation; // 磁偏角
	char magVarDirection;	 // 'E'或'W'
	char modeIndicator;		 // 'A'/'D'/'E'/'N'
} GPRMC_t;

// GPGST数据结构
typedef struct
{
	char type[6];
	char time[10];	   // hhmmss.ss格式
	float rangeRms;	   // 伪距RMS误差(米)
	float stdMajor;	   // 误差椭圆长轴(米)
	float stdMinor;	   // 误差椭圆短轴(米)
	float orientation; // 椭圆方向(度)
	float stdLat;	   // 纬度标准差(米)
	float stdLong;	   // 经度标准差(米)
	float stdAlt;	   // 高度标准差(米)
} GPGST_t;

typedef struct
{
	uint32_t index;
	uint8_t time_sync;	  // 时间同步状态 (0:未同步，MCU系统时间；1:NMEA的UTC0时间；2:处理跳秒后的GPS时间)
	uint32_t time_sec_s;  // 时间整数部分(秒)
	uint32_t time_sec_ms; // 时间小数部分(毫秒)
	uint32_t time_week; // 时间周数
	uint32_t time_stamp; // 时间戳

	GPGGA_t gga;
	GPRMC_t rmc;
	GPGST_t gst;

} gnss_data_t;

uint32_t nmea_parser_loop(const uint8_t *data, uint32_t len);

uint32_t nmea_parser_get(gnss_data_t *gnss);

#endif
