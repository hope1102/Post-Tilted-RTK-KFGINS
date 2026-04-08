#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef PLATFORM_MCU
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#endif

// 时间同步模块:输入pps和gnss数据，内部做好时间同步计算和状态维护，请根据头文件定义重新实现该time_sync模块。

// MCU兼容性说明:
// - 系统时钟函数支持多种MCU平台的时钟源

// 时间同步误差阈值 (毫秒)
#define TIME_SYNC_ERROR_MS 99	  // gnss数据时间毫秒值小于TIME_SYNC_ERROR_MS 或者 大于1000-TIME_SYNC_ERROR_MS 才进行同步处理，这样10hz数据只有整数秒数据会同步处理
#define TIME_SYNC_TIMEOUT_MS 5000 // 上次同步超时时间5秒,超时标记为TIME_SYNC_LOST
#define TIME_SYNC_COUNT 5		  // 连续TIME_SYNC_COUNT个pps信号同步成功，标记g_time_sync.sync_valid为1

// 时间同步状态定义
enum time_sync_status
{
	TIME_SYNC_NONE = 0,	  // 未同步
	TIME_SYNC_SYNCED = 1, // 已完成时间同步
	TIME_SYNC_LOST = 2	  // 时间同步丢失
};

// 时间同步信息结构体
typedef struct
{
	// PPS参数
	uint8_t sync_pps_valid;		 // PPS信号是否有效
	uint32_t pps_system_ms;		 // 最新一次PPS时间戳 (系统时钟毫秒)
	uint32_t pps_system_ms_last; // 上次PPS系统时间(系统时钟毫秒)

	// GNSS参数
	uint8_t sync_gnss_valid;   // GNSS信号是否有效
	uint32_t gnss_time_s;	   // GNSS UTC时间戳 (秒)
	uint32_t gnss_time_ms;	   // GNSS UTC毫秒部分
	uint32_t gnss_system_time; // GNSS接收时的系统时间

	// 同步结果参数
	uint8_t sync_valid;				   // 同步是否有效，UTC和PPS都有效时为1，否则为0
	uint32_t sync_count;			   // 同步成功次数
	enum time_sync_status sync_status; // 同步状态 (TIME_SYNC_*)

	// TIME_SYNC_COUNT同步成功，标记sync_valid为1
	uint32_t last_sync_systime_ms; // 上次同步成功系统时间（pps时刻的系统时间）
	uint32_t last_sync_utctime_s;  // 上次同步成功UTC时间戳，整秒

	double time_offset[TIME_SYNC_COUNT]; // 时间偏移量 (毫秒)，单位：毫秒 time_offset[i] = -last_sync_utctime_s[i] -last_sync_systime_ms[i]；
										 // 每次同步成功，计算一次时间偏移量，循环存储TIME_SYNC_COUNT个时间偏移量
} time_sync_t;

// 核心函数声明

/**
 * @brief 初始化时间同步模块
 */
void time_sync_init(void);

/**
 * @brief 输入PPS时间戳
 * @param pps_timestamp_ms PPS信号对应的系统时间戳 (毫秒)
 * @return true 处理成功, false 处理失败
 */
bool time_sync_input_pps(uint32_t pps_timestamp_ms);

/**
 * @brief 输入GNSS时间
 * @param gnss_timestamp_s GNSS时间戳秒部分
 * @param gnss_timestamp_ms GNSS毫秒部分
 * @param system_timestamp_ms 系统时间戳毫秒
 * @return true 处理成功, false 处理失败
 */
bool time_sync_input_gnss(uint32_t gnss_timestamp_s, uint32_t gnss_timestamp_ms, uint32_t system_timestamp_ms);

/**
 * @brief 检查时间同步是否有效
 * @return true 有效, false 无效
 */
bool time_sync_is_valid(void);

/**
 * @brief 获取同步数据结构
 * @param sync_data 输出时间同步数据结构
 * @return true 获取成功, false 获取失败
 */
bool time_sync_get_data(time_sync_t *sync_data);

/**
 * @brief 获取上次PPS同步数据
 * @param sync_data 输出时间同步数据结构
 * @return true 获取成功, false 获取失败
 */
bool time_sync_get_pps(time_sync_t *sync_data);

/**
 * @brief 根据系统时间戳获取对应的GNSS时间戳
 * @param system_timestamp_ms 系统时间戳 (毫秒)
 * @param gnss_timestamp_s 输出的GNSS时间戳 (秒)
 * @param gnss_timestamp_ms 输出的GNSS毫秒部分
 * @return true 转换成功, false 转换失败
 */
bool time_sync_system_to_gnss(uint32_t system_timestamp_ms, uint32_t *gnss_timestamp_s, uint32_t *gnss_timestamp_ms);

#endif