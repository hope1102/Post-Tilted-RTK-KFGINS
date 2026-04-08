#include "uart1_imu.h"

#include "setting.h"
#include "time_sync.h"
#include "algo_ahrs.h"
#include "algo_data.h"

#ifdef PLATFORM_MCU
#include "hc32f4xx_conf.h"
#include "hc32_ll.h"
#include "SCH1600.h"
#include "mcu_clock.h"
#include "mcu_pps.h"
#include "mcu_spi.h"
#include "mcu_uart.h"
#include "mcu_timer.h"
#include "mcu_flash.h"
#include "ring_buf.h"
#include "uart1_atcmd.h"

#define RING_BUFFER_NUM_VALUES 4
static sch1600_raw_data_summed sch1600_data_summed_raw[RING_BUFFER_NUM_VALUES];
static volatile uint32_t ring_buffer_idx_wr = 0;
static volatile uint32_t ring_buffer_idx_rd = 0;
static volatile bool sch1600_error_available = false;
static volatile uint32_t sample_count = 0;

/* INT_SRC_PORT_EIRQ8 Callback. */
static void PPS_IrqCallback(void);

/* INT_SRC_TMR0_1_CMP_B Callback. */
static void INT_SRC_TMR0_1_CMP_B_IrqCallback(void);

/* INT_SRC_PORT_EIRQ13 Callback. */
static void IMU_SYNC_IrqCallback(void);

/* Check if there is new data available for main() to process. */
static bool new_summed_data_available(void);

// static bool inc_ring_buffer_wr_idx(void);  // Unused function

static void inc_ring_buffer_rd_idx(void);

/* send data to client using UART1 */
static void send_data_to_client(imu_data_t *sch1600_data);

// Int Config
static void App_IntCfg(void)
{
	stc_irq_signin_config_t stcIrq;

	/* IRQ sign-in */
	stcIrq.enIntSrc = INT_SRC_PORT_EIRQ13;
	stcIrq.enIRQn = INT032_IRQn;
	stcIrq.pfnCallback = &IMU_SYNC_IrqCallback;
	(void)INTC_IrqSignIn(&stcIrq);
	/* NVIC config */
	NVIC_ClearPendingIRQ(INT032_IRQn);
	NVIC_SetPriority(INT032_IRQn, DDL_IRQ_PRIO_15);
	NVIC_EnableIRQ(INT032_IRQn);

	/* IRQ sign-in */
	stcIrq.enIntSrc = INT_SRC_PORT_EIRQ8;
	stcIrq.enIRQn = INT033_IRQn;
	stcIrq.pfnCallback = &PPS_IrqCallback;
	(void)INTC_IrqSignIn(&stcIrq);
	/* NVIC config */
	NVIC_ClearPendingIRQ(INT033_IRQn);
	NVIC_SetPriority(INT033_IRQn, DDL_IRQ_PRIO_14);
	NVIC_EnableIRQ(INT033_IRQn);

	/* IRQ sign-in */
	stcIrq.enIntSrc = INT_SRC_TMR0_1_CMP_B;
	stcIrq.enIRQn = INT044_IRQn;
	stcIrq.pfnCallback = &INT_SRC_TMR0_1_CMP_B_IrqCallback;
	(void)INTC_IrqSignIn(&stcIrq);
	/* NVIC config */
	NVIC_ClearPendingIRQ(INT044_IRQn);
	NVIC_SetPriority(INT044_IRQn, DDL_IRQ_PRIO_15);
	NVIC_EnableIRQ(INT044_IRQn);
}

// EIRQ Config
static void App_EIRQCfg()
{
	stc_extint_init_t stcExtIntInit;

	/* EXTINT_CH13 config */
	// PA13 烧写引脚 可能是多余的
	(void)EXTINT_StructInit(&stcExtIntInit);
	stcExtIntInit.u32Filter = EXTINT_FILTER_ON;
	stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV8;
	stcExtIntInit.u32Edge = EXTINT_TRIG_RISING;
	(void)EXTINT_Init(EXTINT_CH13, &stcExtIntInit);

	/* EXTINT_CH08 config */
	// PPS PA8 外部中断
	(void)EXTINT_StructInit(&stcExtIntInit);
	stcExtIntInit.u32Filter = EXTINT_FILTER_ON;
	stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV32;
	stcExtIntInit.u32Edge = EXTINT_TRIG_RISING;
	(void)EXTINT_Init(EXTINT_CH08, &stcExtIntInit);
}

/* INT_SRC_PORT_EIRQ13 Callback. */
static void IMU_SYNC_IrqCallback(void)
{
	// add your codes here
}

/* INT_SRC_PORT_EIRQ8 Callback. */
static void PPS_IrqCallback(void)
{
	// add your codes here
	uint32_t total_ms = SysTick_GetTick();
	time_sync_input_pps(total_ms);
}

/* INT_SRC_TMR0_1_CMP_B Callback. */
static void INT_SRC_TMR0_1_CMP_B_IrqCallback(void)
{
	sch1600_raw_data raw_data;

	// 1.read SCH1633 data
	sch1600_read_data(&raw_data);

	// 2.data accumulation
	sch1600_data_summed_raw[ring_buffer_idx_wr].acc_x_lsb += raw_data.acc_x_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].acc_y_lsb += raw_data.acc_y_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].acc_z_lsb += raw_data.acc_z_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].gyro_x_lsb += raw_data.gyro_x_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].gyro_y_lsb += raw_data.gyro_y_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].gyro_z_lsb += raw_data.gyro_z_lsb;
	sch1600_data_summed_raw[ring_buffer_idx_wr].temp_lsb += raw_data.temp_lsb;

	// 3.error detect
	if (raw_data.frame_error)
	{
		sch1600_error_available = true;
	}

	// 4.check if up to AVG_FACTOR
	if (++sample_count >= AVG_FACTOR)
	{
		sch1600_data_summed_raw[ring_buffer_idx_wr].time_stamp = SysTick_GetTick();

		uint32_t new_idx = (ring_buffer_idx_wr + 1) % RING_BUFFER_NUM_VALUES;
		if (new_idx != ring_buffer_idx_rd)
		{
			ring_buffer_idx_wr = new_idx;
			sch1600_data_summed_raw[ring_buffer_idx_wr] = (sch1600_raw_data_summed){0};
		}
		sample_count = 0;
	}
}

static bool new_summed_data_available(void)
{
	volatile uint32_t wr_idx_tmp = ring_buffer_idx_wr;
	if (wr_idx_tmp != ring_buffer_idx_rd)
		return true;

	return false;
}

static bool inc_ring_buffer_wr_idx(void)
{
	volatile uint32_t new_idx = ring_buffer_idx_wr;

	new_idx++;
	if (new_idx >= RING_BUFFER_NUM_VALUES)
		new_idx = 0;

	// Don't add data to ring buffer if there is no free space
	if (new_idx == ring_buffer_idx_rd)
	{
		return false;
	}

	ring_buffer_idx_wr = new_idx;
	return true;
}

static void inc_ring_buffer_rd_idx(void)
{
	volatile uint32_t new_idx = ring_buffer_idx_rd;

	new_idx++;
	if (new_idx >= RING_BUFFER_NUM_VALUES)
		new_idx = 0;

	ring_buffer_idx_rd = new_idx;
}

static void send_data_to_client(imu_data_t *imu_data)
{
	uint8_t aimu_txt[128];
	memset(aimu_txt, 0, 128);
	uint32_t aimu_len = 0;
	aimu_len = imu_data_encode(aimu_txt, 128, imu_data);
	if (aimu_len > 0)
	{
		printf("%s", aimu_txt);
	}
}

void imu_init()
{
	/* Register write unprotected for some required peripherals. */
	LL_PERIPH_WE(LL_PERIPH_ALL);
	// Clock Config
	App_ClkCfg();
	// Port Config
	App_PortCfg();
	// Int Config
	App_IntCfg();
	// EIRQ Config
	App_EIRQCfg();
	// Timer0 Config
	App_Timer0Cfg();
	// USARTx Config & Initialize ring buffer function.
	int32_t ret = LL_PrintfInit(CM_USART1, 460800UL, App_USART1Cfg);
	(void)ret; // Suppress unused warning
	// SPIx Config
	App_SPIxCfg();
	// EFM Init
	APP_EFM_Init();
	// SysTick Init
	SysTick_Init(1000U);
	// Wait for power supply to stabilize
	DDL_DelayMS(100);
	// Initialize the sensor
	int status = sch1600_init();
	if (status != SCH1600_OK)
	{
		printf("ERROR: sch1600_init failed with code: %d\r\nApplication halted", status);
		while (true)
			;
	}
	else
	{
		printf("\nIMU init success!\r\n");
	}

	// Read serial number from sensor
	char serial_num[14];
	// sch1600_raw_data sch1600_raw_data_last;  // Unused variable
	sch1600_get_serial(serial_num);
	printf("IMU SN: %s\r\n\r\n", serial_num);

	// Clear summed sensor raw data
	memset(&sch1600_data_summed_raw, 0, sizeof(sch1600_data_summed_raw));

	// start Timer0
	TMR0_Start(CM_TMR0_1, TMR0_CH_B);
}

uint32_t imu_index = 0;
void imu_read_loop()
{
	if (new_summed_data_available())
	{
		sch1600_real_data sch1600_data;
		sch1600_convert_data(&sch1600_data_summed_raw[ring_buffer_idx_rd], &sch1600_data);
		inc_ring_buffer_rd_idx();

		imu_data_t imu; // convert  to FRD

		imu.index = imu_index++;
		imu.time_sync = 0;
		imu.time_week = 0;
		imu.time_sec_ms = sch1600_data.time_stamp % 1000;
		imu.time_sec_s = sch1600_data.time_stamp / 1000;
		imu.time_stamp = sch1600_data.time_stamp;
		imu.acc_x = sch1600_data.acc_x;
		imu.acc_y = -sch1600_data.acc_y;
		imu.acc_z = -sch1600_data.acc_z;
		imu.gyro_x = sch1600_data.gyro_x;
		imu.gyro_y = -sch1600_data.gyro_y;
		imu.gyro_z = -sch1600_data.gyro_z;
		imu.temperature = sch1600_data.temp;

		// 检查时间同步是否成功，如果成功则计算GPS时间
		if (time_sync_is_valid())
		{
			uint32_t gnss_timestamp_s, gnss_timestamp_ms;
			if (time_sync_system_to_gnss(sch1600_data.time_stamp, &gnss_timestamp_s, &gnss_timestamp_ms))
			{
				imu.time_sync = 1;
				imu.time_sec_s = gnss_timestamp_s;
				imu.time_sec_ms = gnss_timestamp_ms;
			}
		}

		// 发送数据到客户端
		if (get_user_params()->imu_data)
		{	
			// 发送数据到客户端
			if (get_user_params()->imu_pose)
			{
				ahrs_add_imu(&imu);
				ahrs_state_t *state;
				ahrs_get_imu_pose(state);
			}else{
				send_data_to_client(&imu);
			}
		}
	}
	
	// Handle possible sensor errors
	if (sch1600_error_available)
	{
		// Error from ASIC. Stop sample timer to avoid race condition when reading sensor status.
		sch1600_error_available = false;

		sch1600_sensor_status status;
		sch1600_read_sensor_status(&status);

		// Restart sampling timer
		printf("STATUS:SUM:%4x,SAT:%4x,COM:%4x,RATE:%4x,GX:%4x,GY:%4x,GZ:%4x,AX:%4x,AY:%4x,AZ:%4x\r\n",
			   status.summary_status, status.saturation_status, status.common_status,
			   status.rate_common_status, status.rate_x_status, status.rate_y_status, status.rate_z_status,
			   status.acc_x_status, status.acc_y_status, status.acc_z_status);
	}
}
void imu_read_task()
{
}
#endif //  PLATFORM_MCU

#ifdef PLATFORM_WINDOWS

#include "win_uart.h"
#include "proj_file.h"

UartConfig configIMU = {
	.port_str = "COM5",
	.port_num = 0,
	.baud_rate = 460800,
	.data_bits = 8,
	.stop_bits = 1,
	.parity = 'n'};

UartHandle *handleIMU = NULL;

void imu_init()
{
	handleIMU = uart_init(configIMU);

	if (uart_open(handleIMU) != 0)
	{
		printf("UART_IMU open failed\n");
		return;
	}

	char data[] = "Hello UART_IMU_COM7\r\n";
	uart_write(handleIMU, data, strlen(data));
}

uint8_t uart_buffer[1024];
int buffer_index = 0;
uint8_t buffer_in_message = 0;
uint32_t imu_index = 0;

void imu_read_loop()
{
	char buffer[512];
	int received = uart_read(handleIMU, buffer, sizeof(buffer) - 1);
	if (received > 0)
	{
		buffer[received] = '\0';
		// printf("Received %d: %s\n", received, buffer);
		for (size_t i = 0; i < received; i++)
		{
			if (buffer_in_message)
			{
				uart_buffer[buffer_index++] = buffer[i];
			}

			if (buffer[i] == '$')
			{
				buffer_index = 0;
				buffer_in_message = 1;
				uart_buffer[buffer_index++] = buffer[i];
			}

			if (buffer_index >= 2 &&
				uart_buffer[buffer_index - 2] == '\r' &&
				uart_buffer[buffer_index - 1] == '\n')
			{

				uart_buffer[buffer_index] = '\0';
				// printf("Message: %s\n", uart_buffer);

				proj_imu_write(uart_buffer, buffer_index);

				imu_data_t imu;
				uint8_t result = imu_data_decode(uart_buffer, sizeof(uart_buffer), &imu);
				if (result)
				{

					imu.index = imu_index++;
					ahrs_add_imu(&imu);
				}
				else
				{
					printf("imu_data_decode: error\n");
				}

				buffer_index = 0;
				buffer_in_message = 0;
				memset(uart_buffer, 0, sizeof(uart_buffer));
			}
		}
	}
}

void imu_read_task()
{
	// start thread
}

#endif // PLATFORM_WINDOWS
