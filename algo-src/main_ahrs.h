#ifndef __MAIN_AHRS_H
#define __MAIN_AHRS_H

#include "log.h"
#include "setting.h"
#include "uart1_imu.h"
#include "uart1_atcmd.h"
#include "uart2_update.h"
#include "uart3_gnss.h"
#include "proj_file.h"
#include "time_sync.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifdef PLATFORM_MCU
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#define APP_FLASH_BASE 0x00000000
#define VECT_TAB_OFFSET 0x00010000
#endif

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif

void algo_sleep_ms(uint32_t ms)
{
#ifdef PLATFORM_WINDOWS
	Sleep(ms);
#elif defined(PLATFORM_LINUX)
	usleep(ms * 1000);
#elif defined(PLATFORM_FREERTOS)
	vTaskDelay(pdMS_TO_TICKS(ms));
#elif defined(PLATFORM_MCU)
	SysTick_Delay(ms);
#endif
}

void hal_Init()
{
	imu_init();

	uart1_init();

	uart2_init();

	uart3_init();
}

void task_Init()
{
	time_sync_init();
}

void task_start()
{
	init_user_params();

	init_proj_file();

	read_current_params("config.txt");

	for (;;)
	{
		atcmd_read_loop();

		imu_read_loop();

		gnss_pps_loop();

		gnss_read_loop();

		algo_sleep_ms(1);
	}
}

#endif // MAIN_AHRS_H
