#include "mcu_uart.h"

// ===================== MCU实现 =====================
#ifdef PLATFORM_MCU

#include "hc32f4xx_conf.h"
#include "hc32_ll.h"

volatile uint8_t Uart1_Rx_Complete = 0;
uint8_t Uart1_DataBuf[UART1_RING_BUF_SIZE];
stc_ring_buf_t Uart1_RingBuf;

volatile uint8_t Uart2_Rx_Complete = 0;
uint8_t Uart2_DataBuf[UART2_RING_BUF_SIZE];
stc_ring_buf_t Uart2_RingBuf;

volatile uint8_t Uart3_Rx_Complete = 0;
uint8_t Uart3_DataBuf[UART3_RING_BUF_SIZE];
stc_ring_buf_t Uart3_RingBuf;

void App_PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;

    /* PH0 set to XTAL-EXT/XTAL-OUT */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(XTAL_OUT_PORT, XTAL_OUT_PIN, &stcGpioInit);

    /* PH1 set to XTAL-IN */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(XTAL_IN_PORT, XTAL_IN_PIN, &stcGpioInit);

    /* PB12 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinDrv = PIN_MID_DRV;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(IMU_RESET_PORT, IMU_RESET_PIN, &stcGpioInit);

    /* PB13 set to EIRQ13 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    (void)GPIO_Init(IMU_SYNC_PORT, IMU_SYNC_PIN, &stcGpioInit);

    /* PA8 set to EIRQ8 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    (void)GPIO_Init(PPS_PORT, PPS_PIN, &stcGpioInit);

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_03, GPIO_FUNC_40); // SPI1-MOSI

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_04, GPIO_FUNC_41); // SPI1-MISO

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_05, GPIO_FUNC_42); // SPI1-SS0

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_06, GPIO_FUNC_43); // SPI1-SCK

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_07, GPIO_FUNC_44); // SPI2-MOSI

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_00, GPIO_FUNC_45); // SPI2-MISO

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_01, GPIO_FUNC_46); // SPI2-SS0

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_02, GPIO_FUNC_47); // SPI2-SCK
}

// USART1 Config
int32_t App_USART1Cfg(void *vpDevice, uint32_t u32Param)
{
    stc_usart_uart_init_t stcUartInit;

    /* Enable USART3 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE);
    // 1.uart config
    USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = u32Param;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    USART_UART_Init(CM_USART1, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX function */
    USART_FuncCmd(CM_USART1, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    // 2.port config
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_00, GPIO_FUNC_32); // USART1-TX

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_01, GPIO_FUNC_33); // USART1-RX
    // 3.INT config
    stc_irq_signin_config_t stcIrq;
    /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_USART1_RI;
    stcIrq.enIRQn = INT080_IRQn;
    stcIrq.pfnCallback = &INT_SRC_USART1_RI_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT080_IRQn);
    NVIC_SetPriority(INT080_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(INT080_IRQn);

    // 4.ring buf init
    BUF_Init(&Uart1_RingBuf, Uart1_DataBuf, sizeof(Uart1_DataBuf));
    return LL_OK;
}

void APP_USART2Cfg(void)
{
    stc_usart_uart_init_t stcUartInit;

    // 1.uart config
    /* Enable USART3 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART2, ENABLE);
    USART_DeInit(CM_USART2);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 460800UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    USART_UART_Init(CM_USART2, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX | USART_INT_RX function */
    USART_FuncCmd(CM_USART2, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    // 2.port config
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_10, GPIO_FUNC_36); // USART2-TX

    GPIO_SetDebugPort(GPIO_PIN_TDI, DISABLE);             // important!
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_15, GPIO_FUNC_37); // USART2-RX

    // 3.INT config
    stc_irq_signin_config_t stcIrq;
    /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_USART2_RI;
    stcIrq.enIRQn = INT081_IRQn;
    stcIrq.pfnCallback = &INT_SRC_USART2_RI_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT081_IRQn);
    NVIC_SetPriority(INT081_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(INT081_IRQn);

    // 4.ring buf init
    BUF_Init(&Uart2_RingBuf, Uart2_DataBuf, sizeof(Uart2_DataBuf));
}

void APP_USART3Cfg(void)
{
    stc_usart_uart_init_t stcUartInit;

    // 1.uart config
    /* Enable USART3 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);
    USART_DeInit(CM_USART3);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 460800UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    USART_UART_Init(CM_USART3, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX function */
    USART_FuncCmd(CM_USART3, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    // 2.port config
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_15, GPIO_FUNC_32); // USART3-TX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_14, GPIO_FUNC_33); // USART3-RX

    // 3.INT config
    stc_irq_signin_config_t stcIrq;
    /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_USART3_RI;
    stcIrq.enIRQn = INT086_IRQn;
    stcIrq.pfnCallback = &INT_SRC_USART3_RI_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT086_IRQn);
    NVIC_SetPriority(INT086_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(INT086_IRQn);

    // 4.ring buf init
    BUF_Init(&Uart3_RingBuf, Uart3_DataBuf, sizeof(Uart3_DataBuf));
}

static void INT_SRC_USART1_RI_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(CM_USART1);
    if (u8Data == '\n')
    {
        Uart1_Rx_Complete = 1;
        BUF_Write(&Uart1_RingBuf, &u8Data, 1UL);
    }
    else
    {
        BUF_Write(&Uart1_RingBuf, &u8Data, 1UL);
    }
}

static void INT_SRC_USART2_RI_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(CM_USART2);
    if (u8Data == '\n')
    {
        Uart2_Rx_Complete = 1;
        BUF_Write(&Uart2_RingBuf, &u8Data, 1UL);
    }
    else
    {
        BUF_Write(&Uart2_RingBuf, &u8Data, 1UL);
    }
}

static void INT_SRC_USART3_RI_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(CM_USART3);
    BUF_Write(&Uart3_RingBuf, &u8Data, 1UL);
    if (u8Data == '\n')
    {
        Uart3_Rx_Complete = 1;
        Uart3_RingBuf.u8HasLF = 1;
        Uart3_RingBuf.u32LFSize = BUF_UsedSize(&Uart3_RingBuf);
    }
}

UartHandle *uart_init(UartConfig config)
{
    UartHandle *handle = (UartHandle *)malloc(sizeof(UartHandle));
    if (!handle)
    {
        return NULL;
    }

    return LL_OK;
}

int uart_open(UartHandle *handle)
{
    return 0;
}

int uart_close(UartHandle *handle)
{
    return 0;
}

int uart_read(UartHandle *handle, char *buffer, size_t size)
{

    size_t bytes_read = 0;
    return bytes_read;
}

int uart_write(UartHandle *handle, const char *data, size_t length)
{
    size_t bytes_written = 0;
    return bytes_written;
}

void uart_release(UartHandle *handle)
{
    if (handle)
    {
        free(handle);
    }
}

#endif //! PLATFORM_MCU