#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

// 日志级别枚举
typedef enum {
    LOG_LEVEL_DEBUG,   // 调试信息
    LOG_LEVEL_INFO,    // 一般信息
    LOG_LEVEL_WARNING, // 警告信息
    LOG_LEVEL_ERROR,   // 错误信息
    LOG_LEVEL_FATAL,   // 严重错误
    LOG_LEVEL_NONE     // 关闭所有日志
} log_level_t;

// 日志输出目标
typedef enum {
    LOG_TARGET_UART,   // 串口输出
    LOG_TARGET_RTT,    // Segger RTT输出
    LOG_TARGET_SWO,    // SWO输出
    LOG_TARGET_MEMORY, // 内存缓冲区
    LOG_TARGET_NONE    // 无输出
} log_target_t;

// 日志配置结构体
typedef struct {
    log_level_t level;                 // 当前日志级别
    log_target_t target;               // 输出目标
    void (*output_func)(const char *); // 自定义输出函数
    char buffer[512];                  // 格式化缓冲区（增大以支持长路径）
    uint8_t use_timestamp;             // 是否添加时间戳
    uint8_t use_level_prefix;          // 是否添加级别前缀
    uint8_t use_thread_id;             // 是否添加线程ID
    uint8_t flush_immediately;         // 是否立即刷新缓冲区
} log_config_t;

// 全局日志配置
static log_config_t global_log_config = {
    .level = LOG_LEVEL_INFO,
    .target = LOG_TARGET_UART,
    .output_func = 0,
    .use_timestamp = 1,
    .use_level_prefix = 1,
    .use_thread_id = 0,
    .flush_immediately = 1
};

// 日志级别字符串
static const char *level_strings[] = {
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR",
    "FATAL"
};

// 初始化日志系统
void log_init(log_level_t level, log_target_t target);

// 设置日志级别
void log_set_level(log_level_t level);

// 设置日志目标
void log_set_target(log_target_t target);

// 设置自定义输出函数
void log_set_output_func(void (*func)(const char *));

// 设置是否添加时间戳
void log_set_timestamp(uint8_t enable);

// 设置是否添加级别前缀
void log_set_level_prefix(uint8_t enable);

// 设置是否添加线程ID
void log_set_thread_id(uint8_t enable);

// 设置是否立即刷新缓冲区
void log_set_flush(uint8_t enable);

// 获取线程ID（用户需实现）
uint32_t log_get_thread_id(void);

// 获取当前时间字符串（用户需实现）
void log_get_timestamp(char *buffer, uint32_t size);

// 输出函数（用户需实现）
void log_output(const char *str);

// 核心日志函数
void log_message(log_level_t level, const char *format, va_list args);

// 实际日志打印函数
void log_print(log_level_t level, const char *format, ...);

// 日志宏定义
#define LOG_DEBUG(format, ...) log_print(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) log_print(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) log_print(LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) log_print(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) log_print(LOG_LEVEL_FATAL, format, ##__VA_ARGS__)

#endif // __LOG_H__