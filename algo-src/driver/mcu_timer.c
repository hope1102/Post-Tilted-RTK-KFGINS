#include "mcu_timer.h"

/* SysTick Callback. */
void SysTick_Handler(void)
{
	SysTick_IncTick();
	
	__DSB();  /* Arm Errata 838869 */
}

void App_Timer0Cfg(void)
{
    stc_tmr0_init_t stcTmr0Init;

    /* Enable timer0_1 clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE);

    /************************* Configure TMR0_1_B***************************/
    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV2;
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = 0xC34FU;//change 0x9C3F to 0xC34F
    (void)TMR0_Init(CM_TMR0_1, TMR0_CH_B, &stcTmr0Init);
    TMR0_IntCmd(CM_TMR0_1, TMR0_INT_CMP_B, ENABLE);
    /* TMR0 start counting */
    //TMR0_Start(CM_TMR0_1, TMR0_CH_B);
}