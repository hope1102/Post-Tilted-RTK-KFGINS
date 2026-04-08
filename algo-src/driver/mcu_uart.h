#ifndef MCU_UART_H
#define MCU_UART_H

#ifdef PLATFORM_MCU

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ring_buf.h"

#define XTAL_OUT_PORT   GPIO_PORT_H
#define XTAL_OUT_PIN    GPIO_PIN_00

#define XTAL_IN_PORT   GPIO_PORT_H
#define XTAL_IN_PIN    GPIO_PIN_01

#define IMU_RESET_PORT   GPIO_PORT_B
#define IMU_RESET_PIN    GPIO_PIN_12

#define IMU_SYNC_PORT   GPIO_PORT_B
#define IMU_SYNC_PIN    GPIO_PIN_13

#define PPS_PORT   GPIO_PORT_A
#define PPS_PIN    GPIO_PIN_08

extern void App_PortCfg(void);
extern int32_t App_USART1Cfg(void *vpDevice, uint32_t u32Param);
extern void APP_USART2Cfg(void);
extern void APP_USART3Cfg(void);

/* INT_SRC_USART1_RI_IrqCallback. */
static void INT_SRC_USART1_RI_IrqCallback(void);

#define UART1_RING_BUF_SIZE (128UL)
extern volatile uint8_t Uart1_Rx_Complete;
extern uint8_t Uart1_DataBuf[UART1_RING_BUF_SIZE];
extern stc_ring_buf_t Uart1_RingBuf;

/* INT_SRC_USART2_RI_IrqCallback. */
static void INT_SRC_USART2_RI_IrqCallback(void);

#define UART2_RING_BUF_SIZE (256UL)
extern volatile uint8_t Uart2_Rx_Complete;
extern uint8_t Uart2_DataBuf[UART2_RING_BUF_SIZE];
extern stc_ring_buf_t Uart2_RingBuf;

/* INT_SRC_USART3_RI_IrqCallback. */
static void INT_SRC_USART3_RI_IrqCallback(void);

#define UART3_RING_BUF_SIZE (2048UL)
extern volatile uint8_t Uart3_Rx_Complete;
extern uint8_t Uart3_DataBuf[UART3_RING_BUF_SIZE];
extern stc_ring_buf_t Uart3_RingBuf;

// 串口配置结构体
typedef struct
{
	  char port_str[64];
    int port_num;
    int baud_rate;
    int data_bits; // 5, 6, 7, 8
    int stop_bits; // 1, 2
    char parity;   // 'n', 'o', 'e' for none/odd/even
} UartConfig;

// 串口设备句柄
typedef struct
{
    void *usart;
    UartConfig config;
} UartHandle;

// ========== 通用接口 ==========
UartHandle *uart_init( UartConfig config);
int uart_open(UartHandle *handle);
int uart_close(UartHandle *handle);
int uart_read(UartHandle *handle, char *buffer, size_t size);
int uart_write(UartHandle *handle, const char *data, size_t length);
void uart_release(UartHandle *handle);

#endif

#endif
