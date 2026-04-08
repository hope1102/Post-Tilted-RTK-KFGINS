#ifndef __DATETIME_H__
#define __DATETIME_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

// 基本日期时间结构体
typedef struct
{
    int16_t year;        // 年份 (如 2023)
    int8_t month;        // 月份 (1-12)
    int8_t day;          // 日 (1-31)
    int8_t hour;         // 时 (0-23)
    int8_t minute;       // 分 (0-59)
    int8_t second;       // 秒 (0-59)
    int16_t millisecond; // 毫秒 (0-999)
} datetime_t;

// 定义 gtime_t 类型
typedef struct
{
    uint32_t sec_s;  // 整秒部分
    uint32_t sec_ms; // 小数部分
} gtime_t;

// 判断是否为闰年
int is_leap_year(int year);

// 获取月份的天数
int days_in_month(int month, int year);

// 解析 hhmmss.ss 格式的时间字符串
int parse_time(const char *time_str, datetime_t *dt);

// 解析 ddmmyy 格式的日期字符串
int parse_date(const char *date_str, datetime_t *dt);

// 组合函数：解析时间和日期字符串为datetime_t
int parse_datetime(const char *time_str, const char *date_str, datetime_t *dt);

// 格式化输出datetime_t
void datetime_format(const datetime_t *dt, char *buffer, size_t size);

/* convert calendar day/time to time -------------------------------------------
 * convert calendar day/time to gtime_t struct
 * args   : datetime_t *ep       I   day/time {year,month,day,hour,min,sec}
 * return : gtime_t struct
 * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
 *-----------------------------------------------------------------------------*/
gtime_t epoch2time(const datetime_t *dt);


#endif