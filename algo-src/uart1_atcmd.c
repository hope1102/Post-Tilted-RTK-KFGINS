#include "uart1_atcmd.h"
#include "algo_atcmd.h"
#include "algo_string.h"

#ifdef PLATFORM_MCU
#include "mcu_uart.h"
#include "mcu_flash.h"
#include "ring_buf.h"
#endif

#ifdef PLATFORM_WINDOWS
#include "win_uart.h"
#endif

#include <stdio.h>
#include <string.h>

UartConfig config1 = {
	.port_str = "COM",
	.port_num = 0,
	.baud_rate = 460800,
	.data_bits = 8,
	.stop_bits = 1,
	.parity = 'n' };

UartHandle* handle1 = NULL;

void uart1_init(void)
{

	handle1 = uart_init(config1);
	if (uart_open(handle1) != 0)
	{
		printf("UART1 open failed\n");
		return;
	}

	char data[] = "Hello UART1";
	uart_write(handle1, data, strlen(data));
}

void uart1_uninit()
{
	if (handle1 != NULL)
	{
		uart_close(handle1);
		uart_release(handle1);
		handle1 = NULL;
	}
}

void atcmd_read_loop()
{

#ifdef PLATFORM_MCU
	if (Uart1_Rx_Complete)
	{
		uint8_t str_buf[UART1_RING_BUF_SIZE];
		uint32_t used_size = BUF_UsedSize(&Uart1_RingBuf);

		if (used_size > UART1_RING_BUF_SIZE - 1)
		{
			used_size = UART1_RING_BUF_SIZE - 1;
		}

		uint32_t read_len = BUF_Read(&Uart1_RingBuf, str_buf, used_size);
		str_buf[read_len] = '\0';

		algo_at_command((char *)str_buf, read_len);

		Uart1_Rx_Complete = 0;
		BUF_Clear(&Uart1_RingBuf);
	}
#endif

#ifdef PLATFORM_WINDOWS
	char buffer[128];
	memset(buffer, 0, sizeof(buffer));
	int received = uart_read(handle1, buffer, sizeof(buffer) - 1);
	if (received > 0)
	{
		// printf("UART1 TEXT: %s\n", buffer);
		algo_at_process(buffer, received);	
	}

#endif // PLATFORM_WINDOWS
}


void atcmd_read_task()
{
	// start thread
}