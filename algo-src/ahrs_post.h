#ifndef __AHRS_POST_H
#define __AHRS_POST_H

#include <string.h>
#include <stdint.h>
#include "setting.h"

typedef struct {
	user_params_t user_params;
	char *rover_file;
	char *imu_file;
	char *result_file;
	uint8_t fileformat;
	uint8_t use_start_time;
	uint8_t use_end_time;
	int16_t start_week;
	float start_sec;
	int16_t end_week;
	float end_sec;
	uint8_t enable;
}ahrs_post_config_t;
typedef struct {
	ahrs_post_config_t* config;
	int n;
	int nmax;
}post_configs_t;

void task_start_post(char* post_config_file);

#endif // __AHRS_POST_H

