#include "setting.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 全局用户参数变量
user_params_t g_user_params;

#ifdef PLATFORM_MCU

#include "mcu_flash.h"

// Define the flash memory address for storing user_params_t
// HC32F460 Flash layout:
// 0x00000000-0x0000E000: Boot loader (56KB = 7 sectors)
// 0x0000E000-0x00010000: IAP param area (8KB = 1 sector)
// 0x00010000-0x0007FFFF: Application area (448KB)
// Sector size = 0x2000 (8KB), address must be sector-aligned
// Use a sector near end of application area: 0x0007E000 (sector 63)
#define FLASH_USER_PARAMS_ADDRESS 0x0007E000

// Add parameter validation magic number
#define PARAM_MAGIC_NUMBER 0x12345678

// Function to read parameters from non-volatile memory
int32_t read_user_params(user_params_t *params)
{
	int32_t result = 0;
	uint32_t param_size = sizeof(user_params_t);
	user_params_t temp_params;

	printf("Flash read: addr=0x%08X, size=%u bytes\r\n", FLASH_USER_PARAMS_ADDRESS, param_size);

	// Read parameters from flash memory into temp buffer first
	result = FLASH_ReadData(FLASH_USER_PARAMS_ADDRESS, (uint8_t *)&temp_params, param_size);
	if (result != 0) {
		printf("Flash read failed: result=%d\r\n", result);
		return result;
	}

	// Validate magic number to ensure data integrity
	if (temp_params.magic_number != PARAM_MAGIC_NUMBER) {
		printf("Flash validation failed (magic=0x%08X, expected=0x%08X), using default parameters\r\n",
		       temp_params.magic_number, PARAM_MAGIC_NUMBER);
		init_user_params();
		return -1; // Indicate that defaults were loaded
	}

	// Validation passed, copy temp_params to output
	memcpy(params, &temp_params, param_size);
	printf("Flash read successful\r\n");
	return 0;
}

// Function to write parameters to non-volatile memory
int32_t write_user_params(const user_params_t *params)
{
	int32_t result = 0;
	uint32_t param_size = sizeof(user_params_t);

	printf("Flash write: addr=0x%08X, size=%u bytes\r\n", FLASH_USER_PARAMS_ADDRESS, param_size);
	printf("Flash range: 0x%08X - 0x%08X\r\n", 0, 0x0007FFFF);
	printf("Sector size: 0x%08X (%u bytes)\r\n", 0x2000, 0x2000);

	// Check address alignment
	result = FLASH_CheckAddrAlign(FLASH_USER_PARAMS_ADDRESS);
	if (result != 0) {
		printf("Flash write failed: address not aligned (0x%08X)\r\n", FLASH_USER_PARAMS_ADDRESS);
		return result;
	}

	// Erase flash sector before writing
	printf("Erasing flash sector...\r\n");
	result = FLASH_EraseSector(FLASH_USER_PARAMS_ADDRESS, param_size);
	if (result != 0) {
		printf("Flash erase failed: result=%d\r\n", result);
		return result;
	}

	// Write parameters to flash memory
	printf("Writing parameters to flash...\r\n");
	result = FLASH_WriteData(FLASH_USER_PARAMS_ADDRESS, (uint8_t *)params, param_size);
	if (result != 0) {
		printf("Flash write failed: result=%d\r\n", result);
		return result;
	}

	printf("Flash write successful\r\n");
	return 0;
}

#else
#include <stdio.h>
#include <string.h>

uint8_t save_user_params(const char *filename, const user_params_t *params)
{
	FILE *file = fopen(filename, "w");
	if (!file)
	{
		return -1; // Failed to open file
	}

	// Write header
	fprintf(file, "# IMU Firmware Configuration File\n");
	fprintf(file, "# Auto-generated - Edit with caution\n");
	fprintf(file, "\n");

	// Validation
	fprintf(file, "[validation]\n");
	fprintf(file, "magic_number=0x%08X\n", params->magic_number);
	fprintf(file, "\n");

	// Registration
	fprintf(file, "[registration]\n");
	fprintf(file, "sn=%s\n", params->sn);
	fprintf(file, "reg_flag=%u\n", params->reg_flag);
	fprintf(file, "reg_date_start=%s\n", params->reg_date_start);
	fprintf(file, "reg_date_end=%s\n", params->reg_date_end);
	fprintf(file, "\n");

	// Debug
	fprintf(file, "[debug]\n");
	fprintf(file, "debug_enable=%u\n", params->debug_enable);
	fprintf(file, "echo_enable=%u\n", params->echo_enable);
	fprintf(file, "print_gnss=%u\n", params->print_gnss);
	fprintf(file, "print_imu=%u\n", params->print_imu);
	fprintf(file, "\n");

	// Basic Settings
	fprintf(file, "[basic_settings]\n");
	fprintf(file, "baud1=%u\n", params->baud1);
	fprintf(file, "baud2=%u\n", params->baud2);
	fprintf(file, "baud3=%u\n", params->baud3);
	fprintf(file, "gps_leap=%u\n", params->gps_leap);
	fprintf(file, "gravity=%.6f\n", params->gravity);
	fprintf(file, "\n");

	// Algorithm Settings
	fprintf(file, "[algo_setting]\n");
	fprintf(file, "mode=%u\n", params->mode);
	fprintf(file, "\n");

	// IMU Settings
	fprintf(file, "[imu_setting]\n");
	fprintf(file, "imu_data=%u\n", params->imu_data);
	fprintf(file, "imu_pose=%u\n", params->imu_pose);
	fprintf(file, "imu_freq=%u\n", params->imu_freq);
	fprintf(file, "imu_axis=%c\n", params->imu_axis);
	fprintf(file, "imu_rot=%.6f,%.6f,%.6f\n", params->imu_rot[0], params->imu_rot[1], params->imu_rot[2]);
	fprintf(file, "imu_arm=%.6f,%.6f,%.6f\n", params->imu_arm[0], params->imu_arm[1], params->imu_arm[2]);
	fprintf(file, "\n");

	// GNSS Settings
	fprintf(file, "[gnss_setting]\n");
	fprintf(file, "gnss_type=%u\n", params->gnss_type);
	fprintf(file, "gnss_last=%u\n", params->gnss_last);
	fprintf(file, "gnss_data=%u\n", params->gnss_data);
	fprintf(file, "gnss_pose=%u\n", params->gnss_pose);
	fprintf(file, "gnss_pps=%u\n", params->gnss_pps);
	fprintf(file, "gnss_arm=%.6f,%.6f,%.6f\n", params->gnss_arm[0], params->gnss_arm[1], params->gnss_arm[2]);
	fprintf(file, "club_arm=%.6f,%.6f,%.6f\n", params->club_arm[0], params->club_arm[1], params->club_arm[2]);
	fprintf(file, "\n");

	// Laser Settings
	fprintf(file, "[laser_setting]\n");
	fprintf(file, "laser_arm=%.6f,%.6f,%.6f\n", params->laser.arm[0], params->laser.arm[1], params->laser.arm[2]);
	fprintf(file, "laser_ah=%.6f\n", params->laser.ah);
	fprintf(file, "laser_av=%.6f\n", params->laser.av);
	fprintf(file, "laser_ar=%.6f\n", params->laser.ar);
	fprintf(file, "\n");

	// Lidar Settings
	fprintf(file, "[lidar_setting]\n");
	fprintf(file, "lidar_arm=%.6f,%.6f,%.6f\n", params->lidar.arm[0], params->lidar.arm[1], params->lidar.arm[2]);
	fprintf(file, "lidar_ah=%.6f\n", params->lidar.ah);
	fprintf(file, "lidar_av=%.6f\n", params->lidar.av);
	fprintf(file, "lidar_ar=%.6f\n", params->lidar.ar);
	fprintf(file, "\n");

	// Camera 1 Settings
	fprintf(file, "[cam1_setting]\n");
	fprintf(file, "cam1_size=%u,%u\n", params->cam1.size[0], params->cam1.size[1]);
	fprintf(file, "cam1_arm=%.6f,%.6f,%.6f\n", params->cam1.arm[0], params->cam1.arm[1], params->cam1.arm[2]);
	fprintf(file, "cam1_rot=%.6f,%.6f,%.6f\n", params->cam1.rot[0], params->cam1.rot[1], params->cam1.rot[2]);
	fprintf(file, "cam1_ffxy=%.6f,%.6f,%.6f,%.6f\n", params->cam1.ffxy[0], params->cam1.ffxy[1], params->cam1.ffxy[2], params->cam1.ffxy[3]);
	fprintf(file, "cam1_dist=%.6f,%.6f,%.6f,%.6f,%.6f\n", params->cam1.dist[0], params->cam1.dist[1], params->cam1.dist[2], params->cam1.dist[3], params->cam1.dist[4]);
	fprintf(file, "\n");

	// Camera 2 Settings
	fprintf(file, "[cam2_setting]\n");
	fprintf(file, "cam2_size=%u,%u\n", params->cam2.size[0], params->cam2.size[1]);
	fprintf(file, "cam2_arm=%.6f,%.6f,%.6f\n", params->cam2.arm[0], params->cam2.arm[1], params->cam2.arm[2]);
	fprintf(file, "cam2_rot=%.6f,%.6f,%.6f\n", params->cam2.rot[0], params->cam2.rot[1], params->cam2.rot[2]);
	fprintf(file, "cam2_ffxy=%.6f,%.6f,%.6f,%.6f\n", params->cam2.ffxy[0], params->cam2.ffxy[1], params->cam2.ffxy[2], params->cam2.ffxy[3]);
	fprintf(file, "cam2_dist=%.6f,%.6f,%.6f,%.6f,%.6f\n", params->cam2.dist[0], params->cam2.dist[1], params->cam2.dist[2], params->cam2.dist[3], params->cam2.dist[4]);
	fprintf(file, "\n");

	// Camera 3 Settings
	fprintf(file, "[cam3_setting]\n");
	fprintf(file, "cam3_size=%u,%u\n", params->cam3.size[0], params->cam3.size[1]);
	fprintf(file, "cam3_arm=%.6f,%.6f,%.6f\n", params->cam3.arm[0], params->cam3.arm[1], params->cam3.arm[2]);
	fprintf(file, "cam3_rot=%.6f,%.6f,%.6f\n", params->cam3.rot[0], params->cam3.rot[1], params->cam3.rot[2]);
	fprintf(file, "cam3_ffxy=%.6f,%.6f,%.6f,%.6f\n", params->cam3.ffxy[0], params->cam3.ffxy[1], params->cam3.ffxy[2], params->cam3.ffxy[3]);
	fprintf(file, "cam3_dist=%.6f,%.6f,%.6f,%.6f,%.6f\n", params->cam3.dist[0], params->cam3.dist[1], params->cam3.dist[2], params->cam3.dist[3], params->cam3.dist[4]);
	fprintf(file, "\n");

	// Camera 4 Settings
	fprintf(file, "[cam4_setting]\n");
	fprintf(file, "cam4_size=%u,%u\n", params->cam4.size[0], params->cam4.size[1]);
	fprintf(file, "cam4_arm=%.6f,%.6f,%.6f\n", params->cam4.arm[0], params->cam4.arm[1], params->cam4.arm[2]);
	fprintf(file, "cam4_rot=%.6f,%.6f,%.6f\n", params->cam4.rot[0], params->cam4.rot[1], params->cam4.rot[2]);
	fprintf(file, "cam4_ffxy=%.6f,%.6f,%.6f,%.6f\n", params->cam4.ffxy[0], params->cam4.ffxy[1], params->cam4.ffxy[2], params->cam4.ffxy[3]);
	fprintf(file, "cam4_dist=%.6f,%.6f,%.6f,%.6f,%.6f\n", params->cam4.dist[0], params->cam4.dist[1], params->cam4.dist[2], params->cam4.dist[3], params->cam4.dist[4]);
	fprintf(file, "\n");

	// Solve Settings
	fprintf(file, "[solve_setting]\n");
	fprintf(file, "carrier_type=%u\n", params->carrier_type);
	fprintf(file, "imutype=%u\n", params->imutype);
	fprintf(file, "estmisv=%u\n", params->estmisv);
	fprintf(file, "estlever=%u\n", params->estlever);
	fprintf(file, "estmemsscale=%u\n", params->estmemsscale);
	fprintf(file, "estmemsort=%u\n", params->estmemsort);
	fprintf(file, "backfeedtype=%u\n", params->backfeedtype);
	fprintf(file, "outsmoothtype=%u\n", params->outsmoothtype);
	fprintf(file, "\n");

	fclose(file);
	return 0;
}

uint8_t load_user_params(const char *filename, user_params_t *params)
{
	FILE *file = fopen(filename, "r");
	if (!file)
	{
		return -1; // Failed to open file
	}

	char line[256];
	char key[64];
	char value[192];

	// Initialize params with defaults first
	init_user_params();

	while (fgets(line, sizeof(line), file))
	{
		// Skip comments and empty lines
		if (line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == '[')
			continue;

		// Parse key=value pairs
		if (sscanf(line, "%63[^=]=%191[^\r\n]", key, value) == 2)
		{
			// Validation
			if (strcmp(key, "magic_number") == 0)
			{
				sscanf(value, "0x%X", &params->magic_number);
			}
			// Registration
			else if (strcmp(key, "sn") == 0)
			{
				strncpy((char *)params->sn, value, sizeof(params->sn) - 1);
			}
			else if (strcmp(key, "reg_flag") == 0)
			{
				sscanf(value, "%hhu", &params->reg_flag);
			}
			else if (strcmp(key, "reg_date_start") == 0)
			{
				strncpy((char *)params->reg_date_start, value, sizeof(params->reg_date_start) - 1);
			}
			else if (strcmp(key, "reg_date_end") == 0)
			{
				strncpy((char *)params->reg_date_end, value, sizeof(params->reg_date_end) - 1);
			}
			// Debug
			else if (strcmp(key, "debug_enable") == 0)
			{
				sscanf(value, "%hhu", &params->debug_enable);
			}
			else if (strcmp(key, "echo_enable") == 0)
			{
				sscanf(value, "%hhu", &params->echo_enable);
			}
			else if (strcmp(key, "print_gnss") == 0)
			{
				sscanf(value, "%hhu", &params->print_gnss);
			}
			else if (strcmp(key, "print_imu") == 0)
			{
				sscanf(value, "%hhu", &params->print_imu);
			}
			// Basic Settings
			else if (strcmp(key, "baud1") == 0)
			{
				sscanf(value, "%u", &params->baud1);
			}
			else if (strcmp(key, "baud2") == 0)
			{
				sscanf(value, "%u", &params->baud2);
			}
			else if (strcmp(key, "baud3") == 0)
			{
				sscanf(value, "%u", &params->baud3);
			}
			else if (strcmp(key, "gps_leap") == 0)
			{
				sscanf(value, "%hhu", &params->gps_leap);
			}
			else if (strcmp(key, "gravity") == 0)
			{
				sscanf(value, "%f", &params->gravity);
			}
			// Algorithm Settings
			else if (strcmp(key, "mode") == 0)
			{
				sscanf(value, "%hhu", &params->mode);
			}
			// IMU Settings
			else if (strcmp(key, "imu_data") == 0)
			{
				sscanf(value, "%hhu", &params->imu_data);
			}
			else if (strcmp(key, "imu_pose") == 0)
			{
				sscanf(value, "%hhu", &params->imu_pose);
			}
			else if (strcmp(key, "imu_freq") == 0)
			{
				sscanf(value, "%hhu", &params->imu_freq);
			}
			else if (strcmp(key, "imu_axis") == 0)
			{
				sscanf(value, "%c", &params->imu_axis);
			}
			else if (strcmp(key, "imu_rot") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->imu_rot[0], &params->imu_rot[1], &params->imu_rot[2]);
			}
			else if (strcmp(key, "imu_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->imu_arm[0], &params->imu_arm[1], &params->imu_arm[2]);
			}
			// GNSS Settings
			else if (strcmp(key, "gnss_type") == 0)
			{
				sscanf(value, "%hhu", &params->gnss_type);
			}
			else if (strcmp(key, "gnss_last") == 0)
			{
				sscanf(value, "%hhu", &params->gnss_last);
			}
			else if (strcmp(key, "gnss_data") == 0)
			{
				sscanf(value, "%hhu", &params->gnss_data);
			}
			else if (strcmp(key, "gnss_pose") == 0)
			{
				sscanf(value, "%hhu", &params->gnss_pose);
			}
			else if (strcmp(key, "gnss_pps") == 0)
			{
				sscanf(value, "%hhu", &params->gnss_pps);
			}
			else if (strcmp(key, "gnss_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->gnss_arm[0], &params->gnss_arm[1], &params->gnss_arm[2]);
			}
			else if (strcmp(key, "club_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->club_arm[0], &params->club_arm[1], &params->club_arm[2]);
			}
			// Laser Settings
			else if (strcmp(key, "laser_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->laser.arm[0], &params->laser.arm[1], &params->laser.arm[2]);
			}
			else if (strcmp(key, "laser_ah") == 0)
			{
				sscanf(value, "%f", &params->laser.ah);
			}
			else if (strcmp(key, "laser_av") == 0)
			{
				sscanf(value, "%f", &params->laser.av);
			}
			else if (strcmp(key, "laser_ar") == 0)
			{
				sscanf(value, "%f", &params->laser.ar);
			}
			// Lidar Settings
			else if (strcmp(key, "lidar_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->lidar.arm[0], &params->lidar.arm[1], &params->lidar.arm[2]);
			}
			else if (strcmp(key, "lidar_ah") == 0)
			{
				sscanf(value, "%f", &params->lidar.ah);
			}
			else if (strcmp(key, "lidar_av") == 0)
			{
				sscanf(value, "%f", &params->lidar.av);
			}
			else if (strcmp(key, "lidar_ar") == 0)
			{
				sscanf(value, "%f", &params->lidar.ar);
			}
			// Camera 1 Settings
			else if (strcmp(key, "cam1_size") == 0)
			{
				sscanf(value, "%u,%u", &params->cam1.size[0], &params->cam1.size[1]);
			}
			else if (strcmp(key, "cam1_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam1.arm[0], &params->cam1.arm[1], &params->cam1.arm[2]);
			}
			else if (strcmp(key, "cam1_rot") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam1.rot[0], &params->cam1.rot[1], &params->cam1.rot[2]);
			}
			else if (strcmp(key, "cam1_ffxy") == 0)
			{
				sscanf(value, "%f,%f,%f,%f", &params->cam1.ffxy[0], &params->cam1.ffxy[1], &params->cam1.ffxy[2], &params->cam1.ffxy[3]);
			}
			else if (strcmp(key, "cam1_dist") == 0)
			{
				sscanf(value, "%f,%f,%f,%f,%f", &params->cam1.dist[0], &params->cam1.dist[1], &params->cam1.dist[2], &params->cam1.dist[3], &params->cam1.dist[4]);
			}
			// Camera 2 Settings
			else if (strcmp(key, "cam2_size") == 0)
			{
				sscanf(value, "%u,%u", &params->cam2.size[0], &params->cam2.size[1]);
			}
			else if (strcmp(key, "cam2_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam2.arm[0], &params->cam2.arm[1], &params->cam2.arm[2]);
			}
			else if (strcmp(key, "cam2_rot") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam2.rot[0], &params->cam2.rot[1], &params->cam2.rot[2]);
			}
			else if (strcmp(key, "cam2_ffxy") == 0)
			{
				sscanf(value, "%f,%f,%f,%f", &params->cam2.ffxy[0], &params->cam2.ffxy[1], &params->cam2.ffxy[2], &params->cam2.ffxy[3]);
			}
			else if (strcmp(key, "cam2_dist") == 0)
			{
				sscanf(value, "%f,%f,%f,%f,%f", &params->cam2.dist[0], &params->cam2.dist[1], &params->cam2.dist[2], &params->cam2.dist[3], &params->cam2.dist[4]);
			}
			// Camera 3 Settings
			else if (strcmp(key, "cam3_size") == 0)
			{
				sscanf(value, "%u,%u", &params->cam3.size[0], &params->cam3.size[1]);
			}
			else if (strcmp(key, "cam3_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam3.arm[0], &params->cam3.arm[1], &params->cam3.arm[2]);
			}
			else if (strcmp(key, "cam3_rot") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam3.rot[0], &params->cam3.rot[1], &params->cam3.rot[2]);
			}
			else if (strcmp(key, "cam3_ffxy") == 0)
			{
				sscanf(value, "%f,%f,%f,%f", &params->cam3.ffxy[0], &params->cam3.ffxy[1], &params->cam3.ffxy[2], &params->cam3.ffxy[3]);
			}
			else if (strcmp(key, "cam3_dist") == 0)
			{
				sscanf(value, "%f,%f,%f,%f,%f", &params->cam3.dist[0], &params->cam3.dist[1], &params->cam3.dist[2], &params->cam3.dist[3], &params->cam3.dist[4]);
			}
			// Camera 4 Settings
			else if (strcmp(key, "cam4_size") == 0)
			{
				sscanf(value, "%u,%u", &params->cam4.size[0], &params->cam4.size[1]);
			}
			else if (strcmp(key, "cam4_arm") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam4.arm[0], &params->cam4.arm[1], &params->cam4.arm[2]);
			}
			else if (strcmp(key, "cam4_rot") == 0)
			{
				sscanf(value, "%f,%f,%f", &params->cam4.rot[0], &params->cam4.rot[1], &params->cam4.rot[2]);
			}
			else if (strcmp(key, "cam4_ffxy") == 0)
			{
				sscanf(value, "%f,%f,%f,%f", &params->cam4.ffxy[0], &params->cam4.ffxy[1], &params->cam4.ffxy[2], &params->cam4.ffxy[3]);
			}
			else if (strcmp(key, "cam4_dist") == 0)
			{
				sscanf(value, "%f,%f,%f,%f,%f", &params->cam4.dist[0], &params->cam4.dist[1], &params->cam4.dist[2], &params->cam4.dist[3], &params->cam4.dist[4]);
			}
			// Solve Settings
			else if (strcmp(key, "carrier_type") == 0)
			{
				sscanf(value, "%hhu", &params->carrier_type);
			}
			else if (strcmp(key, "imutype") == 0)
			{
				sscanf(value, "%hhu", &params->imutype);
			}
			else if (strcmp(key, "estmisv") == 0)
			{
				sscanf(value, "%hhu", &params->estmisv);
			}
			else if (strcmp(key, "estlever") == 0)
			{
				sscanf(value, "%hhu", &params->estlever);
			}
			else if (strcmp(key, "estmemsscale") == 0)
			{
				sscanf(value, "%hhu", &params->estmemsscale);
			}
			else if (strcmp(key, "estmemsort") == 0)
			{
				sscanf(value, "%hhu", &params->estmemsort);
			}
			else if (strcmp(key, "backfeedtype") == 0)
			{
				sscanf(value, "%hhu", &params->backfeedtype);
			}
			else if (strcmp(key, "outsmoothtype") == 0)
			{
				sscanf(value, "%hhu", &params->outsmoothtype);
			}
		}
	}

	fclose(file);
	return 0;
}

#endif // PLATFORM_MCU

// 获取当前用户参数
user_params_t *get_user_params()
{
	return &g_user_params;
}

//  初始化用户参数为默认值
void init_user_params()
{
	user_params_t *params = &g_user_params;

	// Set magic number for validation
	params->magic_number = PARAM_MAGIC_NUMBER;

	// registration
	params->sn[0] = 0;
	params->reg_flag = 0;
	params->reg_date_start[0] = 0;
	params->reg_date_end[0] = 0;

	// debug
	params->debug_enable = 1;
	params->echo_enable = 1;
	params->print_gnss = 1;
	params->print_imu = 1;

	// basic_settings
	params->baud1 = 460800;
	params->baud2 = 460800;
	params->baud3 = 460800;
	params->gps_leap = 18;
	params->gravity = 9.80665f;

	// algo_setting
	params->mode = GNSS_MODE_GINS1; // 默认模式

	// gnss_setting
	params->gnss_type = 0;
	params->gnss_last = 3; // NMEA_GPGST
	params->gnss_data = 0; // 不发送NMEA0183数据到UART1
	params->gnss_pose = 1; // 发送$POSE数据到UART1
	params->gnss_pps = 0;  // 不发送$APPS数据到UART1
	for (int i = 0; i < 3; i++)
	{
		params->gnss_arm[i] = 0.0f;
		params->club_arm[i] = 0.0f;
	}

	// imu_setting
	params->imu_data = 1;	// 发送$AIMU数据
	params->imu_pose = 0;   // 不发送$AIMU with rpy数据
	params->imu_freq = 100; // 100Hz
	params->imu_axis = 'F'; // FRD坐标系
	for (int i = 0; i < 3; i++)
	{
		params->imu_rot[i] = 0.0f; // IMU installation angles (roll, pitch, yaw)
		params->imu_arm[i] = 0.0f;
	}

	// laser_setting
	for (int i = 0; i < 3; i++)
	{
		params->laser.arm[i] = 0.0f;
	}
	params->laser.ah = 0.0f;
	params->laser.av = 0.0f;

	// cam_setting
	for (int i = 0; i < 3; i++)
	{
		params->cam1.arm[i] = 0.0f;
		params->cam2.arm[i] = 0.0f;
		params->cam1.rot[i] = 0.0f;
		params->cam2.rot[i] = 0.0f;
	}

	// 相机内参默认值
	params->cam1.ffxy[0] = 1000.0f; // fx
	params->cam1.ffxy[1] = 1000.0f; // fy
	params->cam1.ffxy[2] = 640.0f;	// x0
	params->cam1.ffxy[3] = 480.0f;	// y0

	params->cam2.ffxy[0] = 1000.0f;
	params->cam2.ffxy[1] = 1000.0f;
	params->cam2.ffxy[2] = 640.0f;
	params->cam2.ffxy[3] = 480.0f;

	// 畸变参数默认值
	for (int i = 0; i < 5; i++)
	{
		params->cam1.dist[i] = 0.0f;
		params->cam2.dist[i] = 0.0f;
	}

	// 图像尺寸默认值
	params->cam1.size[0] = 1280; // width
	params->cam1.size[1] = 720;	 // height
	params->cam2.size[0] = 1280;
	params->cam2.size[1] = 720;

	// cam3 和 cam4 默认值
	for (int i = 0; i < 3; i++)
	{
		params->cam3.arm[i] = 0.0f;
		params->cam4.arm[i] = 0.0f;
		params->cam3.rot[i] = 0.0f;
		params->cam4.rot[i] = 0.0f;
	}

	params->cam3.ffxy[0] = 1000.0f;
	params->cam3.ffxy[1] = 1000.0f;
	params->cam3.ffxy[2] = 640.0f;
	params->cam3.ffxy[3] = 480.0f;

	params->cam4.ffxy[0] = 1000.0f;
	params->cam4.ffxy[1] = 1000.0f;
	params->cam4.ffxy[2] = 640.0f;
	params->cam4.ffxy[3] = 480.0f;

	for (int i = 0; i < 5; i++)
	{
		params->cam3.dist[i] = 0.0f;
		params->cam4.dist[i] = 0.0f;
	}

	params->cam3.size[0] = 1280;
	params->cam3.size[1] = 720;
	params->cam4.size[0] = 1280;
	params->cam4.size[1] = 720;

	// solve_setting 默认值
	params->carrier_type = 2;
	params->imutype = 0;
	params->estmisv = 1;
	params->estlever = 1;
	params->estmemsscale = 1;
	params->estmemsort = 1;
	params->backfeedtype = 0;
	params->outsmoothtype = 1;
}

// Debug print all user parameters
void debug_print_user_params(const char* prefix)
{
	user_params_t *params = &g_user_params;

	printf("=== %s: User Parameters Debug Print ===\r\n", prefix);

	// validation
	printf("Validation:\r\n");
	printf("  Magic Number: 0x%08X (expected: 0x%08X)\r\n", params->magic_number, PARAM_MAGIC_NUMBER);

	// registration
	printf("Registration Info:\r\n");
	printf("  SN: %.20s\r\n", params->sn);
	printf("  Reg Flag: %d\r\n", params->reg_flag);
	printf("  Reg Start Date: %.8s\r\n", params->reg_date_start);
	printf("  Reg End Date: %.8s\r\n", params->reg_date_end);

	// debug
	printf("Debug Settings:\r\n");
	printf("  Debug Enable: %d\r\n", params->debug_enable);
	printf("  Echo Enable: %d\r\n", params->echo_enable);
	printf("  Print GNSS: %d\r\n", params->print_gnss);
	printf("  Print IMU: %d\r\n", params->print_imu);

	// basic_settings
	printf("Basic Settings:\r\n");
	printf("  UART1 Baud: %u\r\n", params->baud1);
	printf("  UART2 Baud: %u\r\n", params->baud2);
	printf("  UART3 Baud: %u\r\n", params->baud3);
	printf("  GPS Leap Seconds: %d\r\n", params->gps_leap);
	printf("  Gravity: %.6f\r\n", params->gravity);

	// algo_setting
	printf("Algorithm Settings:\r\n");
	printf("  Work Mode: %d\r\n", params->mode);

	// imu_setting
	printf("IMU Settings:\r\n");
	printf("  IMU Data Output: %d\r\n", params->imu_data);
	printf("  IMU Pose Output: %d\r\n", params->imu_pose);
	printf("  IMU Frequency: %d Hz\r\n", params->imu_freq);
	printf("  IMU Axis: %c\r\n", params->imu_axis);
	printf("  IMU Install Angles (RPY): [%.3f, %.3f, %.3f]\r\n",
		params->imu_rot[0], params->imu_rot[1], params->imu_rot[2]);
	printf("  IMU Lever Arm: [%.3f, %.3f, %.3f]\r\n",
		params->imu_arm[0], params->imu_arm[1], params->imu_arm[2]);

	// gnss_setting
	printf("GNSS Settings:\r\n");
	printf("  GNSS Type: %d\r\n", params->gnss_type);
	printf("  GNSS Last Sentence: %d\r\n", params->gnss_last);
	printf("  GNSS Data Output: %d\r\n", params->gnss_data);
	printf("  GNSS Pose Output: %d\r\n", params->gnss_pose);
	printf("  GNSS Lever Arm: [%.3f, %.3f, %.3f]\r\n",
		params->gnss_arm[0], params->gnss_arm[1], params->gnss_arm[2]);
	printf("  Output Point Lever Arm: [%.3f, %.3f, %.3f]\r\n",
		params->club_arm[0], params->club_arm[1], params->club_arm[2]);

	// laser_setting
	printf("Laser Settings:\r\n");
	printf("  Laser Lever Arm: [%.3f, %.3f, %.3f]\r\n",
		params->laser.arm[0], params->laser.arm[1], params->laser.arm[2]);
	printf("  Laser Horizontal Angle: %.3f\r\n", params->laser.ah);
	printf("  Laser Vertical Angle: %.3f\r\n", params->laser.av);


	printf("=== %s: Parameter Print Complete ===\r\n", prefix);
}

// 统一的保存函数接口
uint8_t save_current_params()
{
	// 打印调试信息
	debug_print_user_params("SAVE_PARAMS");

#ifdef PLATFORM_MCU
	int32_t result = write_user_params(&g_user_params);
	if (result == 0) {
		printf("Parameters saved to MCU Flash successfully\r\n");
		return 0;
	} else {
		printf("Parameters save to MCU Flash failed: %d\r\n", result);
		return 1;
	}
#else
	// Windows and Linux platforms save to config.txt file
	uint8_t result = save_user_params("config.txt", &g_user_params);
	if (result == 0) {
		printf("Parameters saved to config.txt\r\n");
	} else {
		printf("Parameters save failed\r\n");
	}
	return result;
#endif
}

// 统一的加载函数接口
uint8_t read_current_params(const char* config_file)
{
#ifdef PLATFORM_MCU
	int32_t result = read_user_params(&g_user_params);
	if (result == 0) {
		printf("Parameters loaded from MCU Flash successfully\r\n");
		// Print debug information
		debug_print_user_params("READ_PARAMS");
		return 0;
	} else {
		printf("Parameters load from MCU Flash failed: %d\r\n", result);
		// If flash read failed, ensure we have valid default parameters
		init_user_params();
		debug_print_user_params("READ_PARAMS_DEFAULT");
		return 1;
	}
#else
	// Windows and Linux platforms load from config.txt file
	uint8_t result = load_user_params(config_file, &g_user_params);
	if (result == 0) {
		printf("Parameters loaded from config.txt\r\n");
		// Print debug information
		debug_print_user_params("READ_PARAMS");
	} else {
		printf("Parameters load failed\r\n");
	}
	return result;
#endif
}
