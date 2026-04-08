#include "time_sync.h"
#include <string.h>
#include <stdio.h>

// Global variable definitions
static time_sync_t g_time_sync = {0};
static bool s_initialized = false;
static bool s_pps_flag = false; // PPS data update flag

// Private function declarations
static uint32_t get_system_time_ms(void);
static int32_t calculate_time_offset(uint32_t pps_system_ms, uint32_t gnss_utc_s, uint32_t gnss_system_ms);
static bool try_synchronize(void);

/**
 * @brief Initialize time synchronization module
 */
void time_sync_init(void)
{
	// Clear global variables
	memset(&g_time_sync, 0, sizeof(time_sync_t));

	// Initialize state
	g_time_sync.sync_status = TIME_SYNC_NONE;
	g_time_sync.sync_valid = 0;
	g_time_sync.sync_count = 0;
	g_time_sync.sync_pps_valid = 0;
	g_time_sync.sync_gnss_valid = 0;

	// Reset PPS flag
	s_pps_flag = false;
	s_initialized = true;
}

/**
 * @brief Input PPS timestamp
 */
bool time_sync_input_pps(uint32_t pps_timestamp_ms)
{
	if (!s_initialized)
	{
		time_sync_init();
	}

	// Update PPS parameters
	g_time_sync.pps_system_ms_last = g_time_sync.pps_system_ms;
	g_time_sync.pps_system_ms = pps_timestamp_ms;
	g_time_sync.sync_pps_valid = 1;

	// Set PPS flag to mark new data
	s_pps_flag = true;

	if (0)
	{
		printf("[PPS] ts=%u, status=%d, pps_valid=%d, gnss_valid=%d\r\n",
			   pps_timestamp_ms, g_time_sync.sync_status, g_time_sync.sync_pps_valid, g_time_sync.sync_gnss_valid);
	}

	// Try synchronization if both PPS and GNSS are valid
	// if (g_time_sync.sync_pps_valid && g_time_sync.sync_gnss_valid)
	// {
	// 	if (try_synchronize())
	// 	{
	// 		printf("[PPS] Sync attempt successful\r\n");
	// 	}
	// }
	// else
	// {
	// 	printf("[PPS] Waiting for GNSS data (gnss_valid=%d)\r\n", g_time_sync.sync_gnss_valid);
	// }

	return true;
}

/**
 * @brief Input GNSS time
 */
bool time_sync_input_gnss(uint32_t gnss_timestamp_s, uint32_t gnss_timestamp_ms, uint32_t system_timestamp_ms)
{
	if (!s_initialized)
	{
		time_sync_init();
	}

	// Basic parameter validation
	if (gnss_timestamp_s == 0 || gnss_timestamp_ms >= 1000)
	{
		printf("[GNSS] INVALID: ts_s=%u, ts_ms=%u\r\n", gnss_timestamp_s, gnss_timestamp_ms);
		return false;
	}

	// Update GNSS parameters
	g_time_sync.gnss_time_s = gnss_timestamp_s;
	g_time_sync.gnss_time_ms = gnss_timestamp_ms;
	g_time_sync.gnss_system_time = system_timestamp_ms;
	g_time_sync.sync_gnss_valid = 1;

	if (0)
	{
		printf("[GNSS] ts_s=%u, ts_ms=%u, sys_ts_ms=%u, status=%d, pps_valid=%d, gnss_valid=%d\r\n",
			   gnss_timestamp_s, gnss_timestamp_ms, system_timestamp_ms,
			   g_time_sync.sync_status, g_time_sync.sync_pps_valid, g_time_sync.sync_gnss_valid);
	}

	// Check if GNSS time milliseconds are near second boundary (0-99ms or 901-999ms)
	bool near_boundary = (g_time_sync.gnss_time_ms <= TIME_SYNC_ERROR_MS) ||
						 (g_time_sync.gnss_time_ms >= (1000 - TIME_SYNC_ERROR_MS));
	if (!near_boundary)
	{
		// printf("[SYNC] FAILED: gnss_ms=%u not near boundary, ts_s=%u, ts_ms=%u\r\n", g_time_sync.gnss_time_ms, gnss_timestamp_s, gnss_timestamp_ms);
		return false;
	}

	// Try synchronization if both PPS and GNSS are valid
	if (g_time_sync.sync_pps_valid && g_time_sync.sync_gnss_valid)
	{
		if (try_synchronize())
		{
			// printf("[GNSS] Sync attempt successful\r\n");
		}
	}
	else
	{
		printf("[GNSS] Waiting for PPS data (pps_valid=%d)\r\n", g_time_sync.sync_pps_valid);
	}

	return true;
}

/**
 * @brief Attempt time synchronization
 * @return true sync success, false sync failed
 */
static bool try_synchronize(void)
{
	// Calculate time offset
	int32_t time_diff_ms = (int32_t)(g_time_sync.pps_system_ms - g_time_sync.gnss_system_time);

	// Check if time difference is within reasonable range (±TIME_SYNC_ERROR_MS milliseconds)
	if (time_diff_ms < -1000 || time_diff_ms > 1000)
	{
		printf("[SYNC] FAILED: time_diff=%d out of range [-%u,%u]ms\r\n", time_diff_ms, 1000, 1000);
		return false;
	}

	// Calculate time offset - implement according to header file formula
	double offset = g_time_sync.gnss_time_s - g_time_sync.pps_system_ms / 1000.0;
	if (g_time_sync.gnss_time_ms > 500)
	{
		offset += 1.0;
	}

	// Record sync data
	uint32_t idx = g_time_sync.sync_count % TIME_SYNC_COUNT;
	g_time_sync.last_sync_systime_ms = g_time_sync.pps_system_ms;
	g_time_sync.last_sync_utctime_s = g_time_sync.gnss_time_s;
	g_time_sync.time_offset[idx] = (double)offset;
	g_time_sync.sync_count++;
	g_time_sync.sync_status = TIME_SYNC_SYNCED;

	// After TIME_SYNC_COUNT consecutive successful syncs, mark as valid
	if (g_time_sync.sync_count >= TIME_SYNC_COUNT)
	{
		g_time_sync.sync_valid = 1;
	}
	else
	{
		g_time_sync.sync_valid = 0;
	}

	if (0)
	{
		printf("[SYNC] SUCCESS: time_diff=%dms, offset=%.3f, count=%u\r\n",
			   time_diff_ms, offset, g_time_sync.sync_count + 1);
	}

	return true;
}

/**
 * @brief Check if time synchronization is valid
 */
bool time_sync_is_valid(void)
{
	if (!s_initialized)
	{
		return false;
	}

	// Check sync validity
	if (g_time_sync.sync_valid == 0 || g_time_sync.sync_status != TIME_SYNC_SYNCED)
	{
		return false;
	}

	// Check timeout
	uint32_t current_time = get_system_time_ms();

	if ((current_time - g_time_sync.last_sync_systime_ms) > TIME_SYNC_TIMEOUT_MS)
	{
		g_time_sync.sync_status = TIME_SYNC_LOST;
		g_time_sync.sync_valid = 0;
		g_time_sync.sync_pps_valid = 0;
		g_time_sync.sync_gnss_valid = 0;
		printf("[SYNC] LOST: timeout detected, last_sync_systime_ms=%u, current_time=%u, timeout=%u\r\n",
			   g_time_sync.last_sync_systime_ms, current_time, TIME_SYNC_TIMEOUT_MS);
		return false;
	}

	return true;
}

/**
 * @brief Get sync data structure
 */
bool time_sync_get_data(time_sync_t *sync_data)
{
	if (!s_initialized || !sync_data)
	{
		return false;
	}

	*sync_data = g_time_sync;
	return g_time_sync.sync_valid;
}

/**
 * @brief Get last PPS sync data
 */
bool time_sync_get_pps(time_sync_t *sync_data)
{
	if (!s_initialized || !sync_data)
	{
		return false;
	}

	// Check if there is new PPS data
	if (!s_pps_flag)
	{
		return false; // No new PPS data
	}

	// Return current sync data
	*sync_data = g_time_sync;

	// Mark PPS as read, clear flag
	s_pps_flag = false;

	return true; // Successfully got new PPS data
}

/**
 * @brief Convert system timestamp to corresponding GNSS timestamp
 */
bool time_sync_system_to_gnss(uint32_t system_timestamp_ms, uint32_t *gnss_timestamp_s, uint32_t *gnss_timestamp_ms)
{
	if (!time_sync_is_valid())
	{
		return false;
	}

	// Use most recent time offset
	uint32_t last_idx = (g_time_sync.sync_count - 1) % TIME_SYNC_COUNT;
	double offset = system_timestamp_ms/1000.0 + g_time_sync.time_offset[last_idx];

	uint32_t target_s = (uint32_t)offset;
	int32_t target_ms = (uint32_t)((offset - target_s) * 1000);

	*gnss_timestamp_s = (uint32_t)target_s;
	*gnss_timestamp_ms = (uint32_t)target_ms;

	return true;
}

// Private function implementations

/**
 * @brief Get system time (milliseconds)
 */
static uint32_t get_system_time_ms(void)
{
#ifdef PLATFORM_MCU
	return SysTick_GetTick();
#else
	// Non-MCU platform simulation implementation
	return 0;
#endif
}