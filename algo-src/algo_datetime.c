#include "algo_datetime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

// 判断是否为闰年
int is_leap_year(int year)
{
    if (year % 4 != 0)
        return 0;
    if (year % 100 != 0)
        return 1;
    if (year % 400 != 0)
        return 0;
    return 1;
}

// 获取月份的天数
int days_in_month(int month, int year)
{
    static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if (month == 2 && is_leap_year(year))
    {
        return 29;
    }

    if (month < 1 || month > 12)
    {
        return 0;
    }

    return days[month - 1];
}

// 解析 hhmmss.ss 格式的时间字符串
int parse_time(const char *time_str, datetime_t *dt)
{
    if (!time_str || !dt)
        return -1;

    size_t len = strlen(time_str);
    if (len < 6)
        return -1; // 至少需要6个字符 (hhmmss)

    // 提取小时、分钟、秒
    char hour_str[3] = {0};
    char min_str[3] = {0};
    char sec_str[10] = {0}; // 足够容纳秒和小数部分

    // 复制小时部分（前2位）
    strncpy(hour_str, time_str, 2);
    hour_str[2] = '\0';

    // 复制分钟部分（接下来的2位）
    strncpy(min_str, time_str + 2, 2);
    min_str[2] = '\0';

    // 复制秒部分（剩余部分）
    if (len > 6)
    {
        // 有小數點
        strncpy(sec_str, time_str + 4, len - 4);
        sec_str[len - 4] = '\0';
    }
    else
    {
        // 没有小数点
        strncpy(sec_str, time_str + 4, 2);
        sec_str[2] = '\0';
    }

    // 转换为数值
    dt->hour = atoi(hour_str);
    dt->minute = atoi(min_str);

    // 处理秒和小数秒
    double seconds = atof(sec_str);
    dt->second = (int8_t)floor(seconds);
    double fractional = seconds - dt->second;
    dt->millisecond = (int16_t)round(fractional * 1000);

    // 验证时间范围
    if (dt->hour < 0 || dt->hour > 23 ||
        dt->minute < 0 || dt->minute > 59 ||
        dt->second < 0 || dt->second > 59 ||
        dt->millisecond < 0 || dt->millisecond > 999)
    {
        return -1;
    }

    return 0;
}

// 解析 ddmmyy 格式的日期字符串
int parse_date(const char *date_str, datetime_t *dt)
{
    if (!date_str || !dt)
        return -1;

    size_t len = strlen(date_str);
    if (len != 6)
        return -1; // 必须是6个字符 (ddmmyy)

    // 提取日、月、年
    char day_str[3] = {0};
    char month_str[3] = {0};
    char year_str[3] = {0};

    // 复制日部分（前2位）
    strncpy(day_str, date_str, 2);
    day_str[2] = '\0';

    // 复制月部分（接下来的2位）
    strncpy(month_str, date_str + 2, 2);
    month_str[2] = '\0';

    // 复制年部分（最后2位）
    strncpy(year_str, date_str + 4, 2);
    year_str[2] = '\0';

    // 转换为数值
    dt->day = atoi(day_str);
    dt->month = atoi(month_str);
    int year = atoi(year_str);

    // 处理两位年份（00-99）
    if (year < 50)
    {
        dt->year = 2000 + year; // 00-49 -> 2000-2049
    }
    else
    {
        dt->year = 1900 + year; // 50-99 -> 1950-1999
    }

    // 验证日期范围
    if (dt->month < 1 || dt->month > 12 ||
        dt->day < 1 || dt->day > days_in_month(dt->month, dt->year))
    {
        return -1;
    }

    return 0;
}

// 组合函数：解析时间和日期字符串为datetime_t
int parse_datetime(const char *time_str, const char *date_str, datetime_t *dt)
{
    if (!time_str || !date_str || !dt)
        return -1;

    // 先解析日期
    if (parse_date(date_str, dt) != 0)
    {
        return -1;
    }

    // 再解析时间
    if (parse_time(time_str, dt) != 0)
    {
        return -1;
    }

    return 0;
}

// 格式化输出datetime_t
void datetime_format(const datetime_t *dt, char *buffer, size_t size)
{
    if (!dt || !buffer)
        return;

    snprintf(buffer, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             dt->year, dt->month, dt->day,
             dt->hour, dt->minute, dt->second, dt->millisecond);
}

/* convert calendar day/time to time -------------------------------------------
 * convert calendar day/time to gtime_t struct
 * args   : datetime_t *ep       I   day/time {year,month,day,hour,min,sec}
 * return : gtime_t struct
 * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
 *-----------------------------------------------------------------------------*/
gtime_t epoch2time(const datetime_t *dt)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = {0};
    uint32_t days, year = (uint32_t)dt->year, mon = (uint32_t)dt->month, day = (uint32_t)dt->day;

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon)
        return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    time.sec_s = (uint32_t)days * 86400 + (uint32_t)dt->hour * 3600 + (uint32_t)dt->minute * 60 + dt->second;
    time.sec_ms = dt->millisecond;
    return time;
}

// 测试函数
int utest_datetime()
{
    // 测试用例
    const char *test_times[] = {
        "123456.78", // 12:34:56.780
        "000000.00", // 00:00:00.000
        "235959.99", // 23:59:59.990
        "120000",    // 12:00:00.000
        "012345",    // 01:23:45.000
        "123456.",   // 12:34:56.000
        "123456.7",  // 12:34:56.700
        "246060",    // 无效（小时、分钟或秒超出范围）
        "12a456.78"  // 无效（包含非数字字符）
    };

    const char *test_dates[] = {
        "151223", // 2023年12月15日
        "010100", // 2000年1月1日
        "290220", // 2020年2月29日（闰年）
        "311299", // 1999年12月31日
        "010150", // 1950年1月1日
        "000000", // 无效（日、月、年不能为0）
        "321213", // 无效（月份超出范围）
        "1512a3"  // 无效（包含非数字字符）
    };

    int num_tests = sizeof(test_times) / sizeof(test_times[0]);

    printf("时间日期解析测试:\n\n");

    for (int i = 0; i < num_tests; i++)
    {
        datetime_t dt;
        int result = parse_datetime(test_times[i], test_dates[i], &dt);

        printf("时间: %s, 日期: %s -> ", test_times[i], test_dates[i]);

        if (result == 0)
        {
            char buffer[64];
            datetime_format(&dt, buffer, sizeof(buffer));
            printf("%s\n", buffer);
        }
        else
        {
            printf("解析失败\n");
        }
    }

    return 0;
}