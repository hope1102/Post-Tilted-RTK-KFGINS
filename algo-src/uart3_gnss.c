#include "uart3_gnss.h"

#include "setting.h"
#include "nmea_data.h"
#include "algo_ahrs.h"
#include "time_sync.h"

#ifdef PLATFORM_MCU
#include "mcu_uart.h"
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#endif
#ifdef PLATFORM_WINDOWS
#include "win_uart.h"
#include "proj_file.h"
#endif

void COM3_SendData(uint8_t *pu8Buff, uint16_t u16Len);

void COM3_SendString(uint8_t *pu8Str);

UartConfig config3 = {
    .port_str = "COM26",
    .port_num = 0,
    .baud_rate = 460800,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = 'n'};

UartHandle *handle3 = NULL;

#ifdef PLATFORM_MCU
uint8_t str_buf[UART3_RING_BUF_SIZE];
#else
#define UART3_RING_BUF_SIZE 2048
uint8_t str_buf[UART3_RING_BUF_SIZE];
#endif

void uart3_init(void)
{
#ifdef PLATFORM_MCU
    APP_USART3Cfg();
    COM3_SendString((uint8_t *)"\r\nlog gpgga ontime 0.1\r\n log gprmc ontime 0.1\r\n log gpgst ontime 0.1\r\n");
#else
    handle3 = uart_init(config3);

    if (uart_open(handle3) != 0)
    {
        printf("UART3 open failed\n");
        return;
    }

    char data[] = "log gpgga ontime 0.1\r\nlog gprmc ontime 0.1\r\nlog gpgst ontime 0.1\r\n";
    uart_write(handle3, data, strlen(data));

    // char buffer[128];
    // int received = uart_read(handle3, buffer, sizeof(buffer));

#endif
}

void uart3_uninit()
{
    if (handle3 != NULL)
    {
        uart_close(handle3);
        uart_release(handle3);
        handle3 = NULL;
    }
}

void gnss_pps_loop()
{
    time_sync_t pps_data;
    user_params_t *params = get_user_params();

    // Check if GNSS PPS output is enabled
    if (!params->gnss_pps)
        return;

    if (time_sync_get_pps(&pps_data))
    {
        if (time_sync_is_valid())
        {
            time_sync_get_data(&pps_data);
            uint32_t idx = (pps_data.sync_count > 0) ? ((pps_data.sync_count - 1) % TIME_SYNC_COUNT) : 0;

            // Build NMEA format data string (without $ and *)
            char nmea_data[256];
            snprintf(nmea_data, sizeof(nmea_data), "APPS,%u,%u,%.3f,%.3f,%u",
                     pps_data.sync_valid, pps_data.last_sync_utctime_s, pps_data.last_sync_systime_ms / 1000.0, pps_data.time_offset[0], pps_data.sync_count);

            // Calculate NMEA0183 checksum (8-bit XOR)
            uint8_t checksum = 0;
            for (int i = 0; nmea_data[i] != '\0'; i++)
            {
                checksum ^= (uint8_t)nmea_data[i];
            }

            // Output with calculated checksum
            printf("$%s*%02X\r\n", nmea_data, checksum);
        }
        else
        {
            uint32_t idx = (pps_data.sync_count > 0) ? ((pps_data.sync_count - 1) % TIME_SYNC_COUNT) : 0;

            // Build NMEA format data string (without $ and *)
            char nmea_data[256];
            snprintf(nmea_data, sizeof(nmea_data), "APPS,%u,%u,%.3f,%.3f,%u",
                     pps_data.sync_valid, pps_data.last_sync_utctime_s, pps_data.pps_system_ms / 1000.0, pps_data.time_offset[0], pps_data.sync_count);

            // Calculate NMEA0183 checksum (8-bit XOR)
            uint8_t checksum = 0;
            for (int i = 0; nmea_data[i] != '\0'; i++)
            {
                checksum ^= (uint8_t)nmea_data[i];
            }

            // Output with calculated checksum
            printf("$%s*%02X\r\n", nmea_data, checksum);
        }
    }
}

// NMEA数据缓冲区
static uint8_t g_nmea_buffer[2048];
static uint32_t g_buffer_len = 0;

// 临时数据包缓冲区
static uint8_t g_packet_buffer[512];

/**
 * @brief 处理完整的NMEA数据包
 * @param packet NMEA数据包
 * @param len 数据包长度
 */
static void process_nmea_packet(uint8_t *packet, uint32_t len)
{
    if (packet == NULL || len == 0)
        return;

    user_params_t *params = get_user_params();

    // GNSS_DATA: 实时转发NMEA数据
    if (params->gnss_data)
    {
        printf("%.*s", (int)len, packet);
    }

#ifdef PLATFORM_WINDOWS
    // Windows平台：写入文件记录
    proj_gnss_write(packet, len);
#endif

    // 解析NMEA数据
    uint8_t result = nmea_parser_loop(packet, len);

    if (result)
    {
        gnss_data_t gnss;
        ahrs_state_t state;
        uint32_t res_gnss = nmea_parser_get(&gnss);
        if (res_gnss == 0)
        {

#ifdef PLATFORM_MCU
            gnss.time_stamp = SysTick_GetTick();
#endif
            time_sync_input_gnss(gnss.time_sec_s, gnss.time_sec_ms, gnss.time_stamp);

            ahrs_add_gnss(&gnss);

            // GNSS_POSE: 实时调用ahrs_get_gnss_pose发送pose数据
            if (params->gnss_pose)
            {
                ahrs_get_gnss_pose(&state);
            }
        }
    }
}

/**
 * @brief 从缓冲区中提取完整的NMEA数据包
 * @param buffer 数据缓冲区
 * @param buffer_len 缓冲区长度指针
 */
static void extract_nmea_packets(uint8_t *buffer, uint32_t *buffer_len)
{
    if (buffer == NULL || buffer_len == NULL || *buffer_len == 0)
        return;

    uint32_t start = 0;
    uint32_t processed = 0;

    for (uint32_t i = 0; i < *buffer_len - 1; i++)
    {
        // 查找NMEA消息开始标识
        if (buffer[i] == '$')
        {
            start = i;
        }

        // 查找\r\n结束符
        if (buffer[i] == '\r' && buffer[i + 1] == '\n')
        {
            uint32_t packet_len = i + 2 - start;

            // 检查是否是完整的NMEA包（以$开头）
            if (start < i && buffer[start] == '$' && packet_len <= sizeof(g_packet_buffer))
            {
                memcpy(g_packet_buffer, buffer + start, packet_len);
                g_packet_buffer[packet_len] = '\0';

                // 处理这个完整的NMEA包
                process_nmea_packet(g_packet_buffer, packet_len);
            }

            processed = i + 2;
        }
    }

    // 移除已处理的数据，保留未完成的数据包
    if (processed > 0 && processed < *buffer_len)
    {
        uint32_t remaining = *buffer_len - processed;
        memmove(buffer, buffer + processed, remaining);
        *buffer_len = remaining;
        memset(buffer + remaining, 0, sizeof(g_nmea_buffer) - remaining);
    }
    else if (processed >= *buffer_len)
    {
        // 所有数据都已处理完
        *buffer_len = 0;
        memset(buffer, 0, sizeof(g_nmea_buffer));
    }
}

/**
 * @brief GNSS数据读取循环主函数
 */
void gnss_read_loop()
{
#ifdef PLATFORM_MCU

    // MCU平台：从环形缓冲区读取数据
    if (Uart3_Rx_Complete)
    {
        if (g_buffer_len >= sizeof(g_nmea_buffer) - 1)
        {
            // 缓冲区已满，保留最后的数据并重置
            printf("GNSS buffer overflow, preserving recent data and resetting...\n");

            // 保留最后1/3的数据，避免完全丢失重要信息
            uint32_t preserve_size = sizeof(g_nmea_buffer) / 3;
            if (preserve_size > 0 && g_buffer_len > preserve_size)
            {
                memmove(g_nmea_buffer, g_nmea_buffer + g_buffer_len - preserve_size, preserve_size);
                g_buffer_len = preserve_size;
                g_nmea_buffer[g_buffer_len] = '\0';
            }
            else
            {
                g_buffer_len = 0;
            }
            return;
        }

        uint32_t max_len = sizeof(g_nmea_buffer) - g_buffer_len - 1;
        uint32_t used_size = Uart3_RingBuf.u32LFSize;
        uint32_t read_len = (max_len > used_size) ? used_size : max_len;

        uint32_t received = BUF_Read(&Uart3_RingBuf, (char *)(g_nmea_buffer + g_buffer_len), read_len);
        if (received > 0)
        {
            g_buffer_len += received;
            g_nmea_buffer[g_buffer_len] = '\0';

            if (get_user_params()->print_gnss)
            {
                // printf("%4d:%s\r\n", g_buffer_len, g_nmea_buffer);
            }

            // 提取并处理完整的NMEA数据包
            extract_nmea_packets(g_nmea_buffer, &g_buffer_len);
            // 处理这个完整的NMEA包
            // process_nmea_packet(g_nmea_buffer, g_buffer_len);
        }

        Uart3_Rx_Complete = 0;
        Uart3_RingBuf.u8HasLF = 0;
        Uart3_RingBuf.u32LFSize = 0;
    }

#endif

#ifdef PLATFORM_WINDOWS
    // Windows平台：从串口读取数据
    if (g_buffer_len >= sizeof(g_nmea_buffer) - 1)
    {
        // 缓冲区已满，保留最后的数据并重置
        printf("GNSS buffer overflow (Windows), preserving recent data and resetting...\n");

        // 保留最后1/3的数据，避免完全丢失重要信息
        uint32_t preserve_size = sizeof(g_nmea_buffer) / 3;
        if (preserve_size > 0 && g_buffer_len > preserve_size)
        {
            memmove(g_nmea_buffer, g_nmea_buffer + g_buffer_len - preserve_size, preserve_size);
            g_buffer_len = preserve_size;
            g_nmea_buffer[g_buffer_len] = '\0';
        }
        else
        {
            g_buffer_len = 0;
        }
        return;
    }

    // 非阻塞读取串口数据
    int32_t received = uart_read(handle3, (char *)(g_nmea_buffer + g_buffer_len),
                                 sizeof(g_nmea_buffer) - g_buffer_len - 1);

    if (received > 0)
    {
        g_buffer_len += received;
        g_nmea_buffer[g_buffer_len] = '\0';

        // 提取并处理完整的NMEA数据包
        extract_nmea_packets(g_nmea_buffer, &g_buffer_len);
    }
    // received <= 0 表示暂无数据，直接返回不做处理
#endif
}

void gnss_read_task()
{
    // TODO,using thread
}

void COM3_SendData(uint8_t *pu8Buff, uint16_t u16Len)
{
#ifdef PLATFORM_MCU
    USART_UART_Trans(CM_USART3, pu8Buff, u16Len, 0xFFFFFFFFUL);
#endif
}

void COM3_SendString(uint8_t *pu8Str)
{
    uint32_t u32Len = 0;

    while (pu8Str[u32Len] != '\0')
    {
        u32Len++;
    }
    COM3_SendData(pu8Str, u32Len);
}
