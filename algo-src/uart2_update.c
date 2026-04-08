#include "uart2_update.h"
#include "algo_atcmd.h"

#ifdef PLATFORM_MCU
#include "mcu_uart.h"
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#endif
#ifdef PLATFORM_WINDOWS
#include "win_uart.h"
#include "proj_file.h"
#endif

#define UART2_RING_BUF_SIZE (100UL)

UartConfig config2 = {
	.port_str = "COM17",
	.port_num = 0,
	.baud_rate = 460800,
	.data_bits = 8,
	.stop_bits = 1,
	.parity = 'n' };

UartHandle* handle2 = NULL;

void COM2_SendData(uint8_t *pu8Buff, uint16_t u16Len)
{
#ifdef PLATFORM_MCU
    USART_UART_Trans(CM_USART2, pu8Buff, u16Len, 0xFFFFFFFFUL);
#endif
}

void COM2_SendString(uint8_t *pu8Str)
{
    uint32_t u32Len = 0;

    while (pu8Str[u32Len] != '\0')
    {
        u32Len++;
    }
    COM2_SendData(pu8Str, u32Len);
}

void uart2_init(void)
{
#ifdef PLATFORM_MCU
	APP_USART2Cfg();
	COM2_SendString((uint8_t *)"Hello UART2");
#else
	handle2 = uart_init(config2);

	if (uart_open(handle2) != 0)
	{
		printf("UART2 open failed\n");
		return;
	}

	char data[] = "Hello UART2";
	uart_write(handle2, data, strlen(data));
#endif
}

void uart2_uninit()
{
	if (handle2 != NULL)
	{
		uart_close(handle2);
		uart_release(handle2);
		handle2 = NULL;
	}
}

void update_read_loop()
{
#ifdef PLATFORM_MCU
	if (Uart2_Rx_Complete)
	{
		uint8_t str_buf[UART2_RING_BUF_SIZE];
		uint32_t used_size = BUF_UsedSize(&Uart2_RingBuf);

		if (used_size > UART2_RING_BUF_SIZE - 1)
		{
			used_size = UART2_RING_BUF_SIZE - 1;
		}

		uint32_t read_len = BUF_Read(&Uart2_RingBuf, str_buf, used_size);
		str_buf[read_len] = '\0';

		algo_at_command((char*)str_buf, read_len);

		Uart2_Rx_Complete = 0;
		BUF_Clear(&Uart2_RingBuf);
	}
#else
	char buffer[128];
	int received = uart_read(handle2, buffer, sizeof(buffer));
	if (received > 0)
	{
		buffer[received] = '\0';
		printf("Received: %s\n", buffer);
	}
#endif
}

void update_read_task()
{
	// start thread
}