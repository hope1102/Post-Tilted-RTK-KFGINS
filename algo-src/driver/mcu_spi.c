#include "mcu_spi.h"

//SPIx Config
void App_SPIxCfg(void)
{
    stc_spi_init_t stcSpiInit;
    stc_spi_delay_t stcSpiDelay;
	
	/* Enable SPI1 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_SPI1, ENABLE);
    /************************* Configure SPI1***************************/
    SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode = SPI_4_WIRE;
    stcSpiInit.u32TransMode = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave = SPI_SLAVE;
    stcSpiInit.u32ModeFaultDetect = SPI_MD_FAULT_DETECT_DISABLE;
    stcSpiInit.u32Parity = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode = SPI_MD_1;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV8;
    stcSpiInit.u32DataBits = SPI_DATA_SIZE_32BIT;
    stcSpiInit.u32FirstBit = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel = SPI_1_FRAME;
    (void)SPI_Init(CM_SPI1, &stcSpiInit);

    /* SPI loopback function configuration */
    SPI_SetLoopbackMode(CM_SPI1, SPI_LOOPBACK_INVD);
    /* SPI parity check error self diagnosis configuration */
    SPI_ParityCheckCmd(CM_SPI1, DISABLE);
    /* SPI SS signal valid level configuration */
    SPI_SetSSValidLevel(CM_SPI1, SPI_PIN_SS0, SPI_SS_VALID_LVL_LOW);
    /* Enable SPI1 */
    SPI_Cmd(CM_SPI1, ENABLE);
	
    /* Enable SPI2 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_SPI2, ENABLE);
    /************************* Configure SPI2***************************/
    SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode = SPI_4_WIRE;
    stcSpiInit.u32TransMode = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave = SPI_MASTER;
    stcSpiInit.u32Parity = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode = SPI_MD_0;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV16;//change SPI_BR_CLK_DIV8 to SPI_BR_CLK_DIV16
    stcSpiInit.u32DataBits = SPI_DATA_SIZE_32BIT;
    stcSpiInit.u32FirstBit = SPI_FIRST_MSB;
    stcSpiInit.u32SuspendMode = SPI_COM_SUSP_FUNC_OFF;
    stcSpiInit.u32FrameLevel = SPI_1_FRAME;
    (void)SPI_Init(CM_SPI2, &stcSpiInit);

    SPI_DelayStructInit(&stcSpiDelay);
    stcSpiDelay.u32IntervalDelay = SPI_INTERVAL_TIME_1SCK;
    stcSpiDelay.u32ReleaseDelay = SPI_RELEASE_TIME_1SCK;
    stcSpiDelay.u32SetupDelay = SPI_SETUP_TIME_1SCK;
    (void)SPI_DelayTimeConfig(CM_SPI2, &stcSpiDelay);

    /* SPI loopback function configuration */
    SPI_SetLoopbackMode(CM_SPI2, SPI_LOOPBACK_INVD);
    /* SPI parity check error self diagnosis configuration */
    SPI_ParityCheckCmd(CM_SPI2, DISABLE);
    /* SPI valid SS signal configuration */
    SPI_SSPinSelect(CM_SPI2, SPI_PIN_SS0);
    /* SPI SS signal valid level configuration */
    SPI_SetSSValidLevel(CM_SPI2, SPI_PIN_SS0, SPI_SS_VALID_LVL_LOW);
    /* Enable SPI2 */
    SPI_Cmd(CM_SPI2, ENABLE);
}
