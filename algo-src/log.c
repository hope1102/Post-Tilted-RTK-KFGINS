// log.c
#include "log.h"
#include <stdio.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif

#ifdef PLATFORM_LINUX
#include <unistd.h>
#endif

#ifdef __GNUC__
#define WEAK __attribute__((weak))
#elif defined(__ICCARM__)
#define WEAK __weak
#else
#define WEAK
#endif


// 初始化日志系统
void log_init(log_level_t level, log_target_t target)
{
    global_log_config.level = level;
    global_log_config.target = target;
}

// 设置日志级别
void log_set_level(log_level_t level)
{
    global_log_config.level = level;
}

// 设置日志目标
void log_set_target(log_target_t target)
{
    global_log_config.target = target;
}

// 设置自定义输出函数
void log_set_output_func(void (*func)(const char *))
{
    global_log_config.output_func = func;
}

// 设置是否添加时间戳
void log_set_timestamp(uint8_t enable)
{
    global_log_config.use_timestamp = enable;
}

// 设置是否添加级别前缀
void log_set_level_prefix(uint8_t enable)
{
    global_log_config.use_level_prefix = enable;
}

// 设置是否添加线程ID
void log_set_thread_id(uint8_t enable)
{
    global_log_config.use_thread_id = enable;
}

// 设置是否立即刷新缓冲区
void log_set_flush(uint8_t enable)
{
    global_log_config.flush_immediately = enable;
}

// 整数转字符串（替代itoa）
static uint32_t log_itoa(uint32_t value, char *str, int base, int min_width)
{
    char *ptr = str, *ptr1 = str, tmp_char;
    uint32_t tmp_value;
    int width = 0;

    // 生成数字字符串（逆序）
    do
    {
        tmp_value = value;
        value /= base;
        *ptr++ = "0123456789ABCDEF"[(tmp_value - value * base)];
        width++;
    } while (value);

    // 填充前导零
    while (width < min_width)
    {
        *ptr++ = '0';
        width++;
    }

    // 添加结束符
    *ptr-- = '\0';

    // 反转字符串
    while (ptr1 < ptr)
    {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }

    return width;
}

// 获取线程ID（默认实现）
WEAK uint32_t log_get_thread_id(void)
{
    return 0;
}

// 获取当前时间字符串（默认实现）
WEAK void log_get_timestamp(char *buffer, uint32_t size)
{
    // 简单时间戳实现：毫秒计数
    static uint32_t counter = 0;
    counter += 10; // 假设每次调用增加10ms
    uint32_t ms = counter;
    uint32_t s = ms / 1000;
    ms %= 1000;
    uint32_t m = s / 60;
    s %= 60;
    uint32_t h = m / 60;
    m %= 60;

    // 格式: [HH:MM:SS.mmm]
    uint32_t pos = 0;
    buffer[pos++] = '[';
    pos += log_itoa(h, buffer + pos, 10, 2);
    buffer[pos++] = ':';
    pos += log_itoa(m, buffer + pos, 10, 2);
    buffer[pos++] = ':';
    pos += log_itoa(s, buffer + pos, 10, 2);
    buffer[pos++] = '.';
    pos += log_itoa(ms, buffer + pos, 10, 3);
    buffer[pos++] = ']';
    buffer[pos++] = ' ';
    buffer[pos] = '\0';
}

// 输出函数（默认实现）
WEAK void log_output(const char *str)
{
#if defined(PLATFORM_WINDOWS) || defined(PLATFORM_LINUX)
    // Windows 和 Linux 平台直接使用 printf
    printf("%s", str);
    fflush(stdout);
#else
    // 嵌入式平台需要用户实现此函数
    // 这里留空，用户需要在自己的代码中提供非弱定义的实现
#endif
}

// 核心日志函数
WEAK void log_message(log_level_t level, const char *format, va_list args)
{
    // 检查日志级别
    if (level < global_log_config.level)
    {
        return;
    }

    char *buffer = global_log_config.buffer;
    uint32_t pos = 0;
    uint32_t buf_size = sizeof(global_log_config.buffer);

    // 添加时间戳
    if (global_log_config.use_timestamp)
    {
        char timestamp[32];
        log_get_timestamp(timestamp, sizeof(timestamp));
        uint32_t ts_len = strlen(timestamp);
        if (pos + ts_len < buf_size)
        {
            strcpy(buffer + pos, timestamp);
            pos += ts_len;
        }
    }

    // 添加线程ID
    if (global_log_config.use_thread_id)
    {
        uint32_t tid = log_get_thread_id();
        buffer[pos++] = '[';
        pos += log_itoa(tid, buffer + pos, 10, 0);
        buffer[pos++] = ']';
        buffer[pos++] = ' ';
    }

    // 添加日志级别前缀
    if (global_log_config.use_level_prefix)
    {
        const char *level_str = level_strings[level];
        uint32_t level_len = strlen(level_str);

        buffer[pos++] = '[';
        if (pos + level_len < buf_size)
        {
            strcpy(buffer + pos, level_str);
            pos += level_len;
        }
        buffer[pos++] = ']';
        buffer[pos++] = ' ';
    }

#if defined(PLATFORM_WINDOWS) || defined(PLATFORM_LINUX)
    // 在 Windows/Linux 平台使用标准库的 vsnprintf 进行格式化
    // 这样可以支持所有标准格式，包括 %.3f 等
    int remaining = buf_size - pos - 1;
    if (remaining > 0)
    {
        int written = vsnprintf(buffer + pos, remaining, format, args);
        if (written > 0)
        {
            pos += written;
        }
    }
#else
    // 嵌入式平台使用简化的格式化（保留原来的实现）
    char *fmt_ptr = (char *)format;
    while (*fmt_ptr && pos < buf_size - 1)
    {
        if (*fmt_ptr == '%')
        {
            fmt_ptr++;
            switch (*fmt_ptr)
            {
            case 'd':
            {
                int num = va_arg(args, int);
                pos += log_itoa(num, buffer + pos, 10, 0);
                break;
            }
            case 'u':
            {
                unsigned int num = va_arg(args, unsigned int);
                pos += log_itoa(num, buffer + pos, 10, 0);
                break;
            }
            case 'x':
            {
                unsigned int num = va_arg(args, unsigned int);
                pos += log_itoa(num, buffer + pos, 16, 0);
                break;
            }
            case 'c':
            {
                char c = (char)va_arg(args, int);
                buffer[pos++] = c;
                break;
            }
            case 's':
            {
                char *str = va_arg(args, char *);
                if (str == NULL)
                {
                    str = "(null)";
                }
                while (*str && pos < buf_size - 1)
                {
                    buffer[pos++] = *str++;
                }
                break;
            }
            case 'f':
            {
                // 嵌入式平台简单浮点数转换
                double num = va_arg(args, double);
                int integer_part = (int)num;
                int decimal_part = (int)((num - integer_part) * 1000000);
                if (decimal_part < 0) decimal_part = -decimal_part;
                pos += log_itoa(integer_part, buffer + pos, 10, 0);
                buffer[pos++] = '.';
                pos += log_itoa(decimal_part, buffer + pos, 10, 6);
                break;
            }
            case '%':
            {
                buffer[pos++] = '%';
                break;
            }
            default:
            {
                buffer[pos++] = '%';
                buffer[pos++] = *fmt_ptr;
                break;
            }
            }
            fmt_ptr++;
        }
        else
        {
            buffer[pos++] = *fmt_ptr++;
        }
    }
#endif

    // 确保字符串以空字符结尾
    buffer[pos] = '\0';

    // 输出日志
    if (global_log_config.output_func)
    {
        global_log_config.output_func(buffer);
    }
    else
    {
        log_output(buffer);
    }

    // 添加换行符
    if (global_log_config.target != LOG_TARGET_MEMORY)
    {
        if (global_log_config.output_func)
        {
            global_log_config.output_func("\n");
        }
        else
        {
            log_output("\n");
        }
    }
}

// 实际日志打印函数
WEAK void log_print(log_level_t level, const char *format, ...)
{
    if (level < global_log_config.level)
    {
        return;
    }

    va_list args;
    va_start(args, format);
    log_message(level, format, args);
    va_end(args);
}

// 单元测试
#ifdef LOG_UNIT_TEST
#include <stdio.h>

// 测试输出函数
void test_output(const char *str)
{
    printf("%s", str);
}

int main()
{
    // 初始化日志系统
    log_init(LOG_LEVEL_DEBUG, LOG_TARGET_UART);
    log_set_output_func(test_output);

    // 测试日志输出
    LOG_DEBUG("系统启动");
    LOG_INFO("初始化完成");
    LOG_WARNING("内存使用率: %d%%", 75);
    LOG_ERROR("传感器 %s 读取失败", "SENSOR1");
    LOG_FATAL("系统崩溃，错误代码: %x", 0xDEADBEEF);

    // 测试日志级别过滤
    log_set_level(LOG_LEVEL_WARNING);
    LOG_DEBUG("这条调试信息不会被显示");
    LOG_WARNING("警告信息仍然显示");

    return 0;
}
#endif