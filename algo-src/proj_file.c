#include "proj_file.h"

#include <stdio.h>
#include <stdint.h>

#ifdef PLATFORM_WINDOWS
static uint8_t g_is_save = 1;
#else
static uint8_t g_is_save = 0;
#endif

FILE* fileIMU;
FILE* fileGNSS;
FILE* filePOSE;

void init_proj_file()
{
	if (!g_is_save)
		return;

	fileIMU = fopen("algo-imu.txt", "wb");
	fileGNSS = fopen("algo-gnss.txt", "wb");
	filePOSE = fopen("algo-pose.txt", "wb");

	if (fileIMU == NULL || fileGNSS == NULL || filePOSE == NULL)
	{
		printf("Create project file failed\n");
	}
	printf("Create project file:\n");
	printf("imu file: %s\n", "algo-imu.txt");
	printf("gnss file: %s\n", "algo-gnss.txt");
	printf("pose file: %s\n", "algo-pose.txt");
}

void close_proj_file()
{
	if (!g_is_save)
		return;

	fclose(fileIMU);
	fclose(fileGNSS);
	fclose(filePOSE);

	printf("close project file success\n");
}

void proj_imu_write(uint8_t* data, uint32_t len)
{
	if (!g_is_save)
		return;

	fwrite(data, 1, len, fileIMU);
}

void proj_gnss_write(uint8_t* data, uint32_t len)
{
	if (!g_is_save)
		return;

	fwrite(data, 1, len, fileGNSS);
}

void proj_pose_write(uint8_t* data, uint32_t len)
{
	if (!g_is_save)
		return;

	fwrite(data, 1, len, filePOSE);
}
