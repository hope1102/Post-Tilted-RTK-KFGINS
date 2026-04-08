#ifndef __PROJ_FILE_H__
#define __PROJ_FILE_H__
#include <stdio.h>
#include <stdint.h>

void init_proj_file();

void close_proj_file();

void proj_imu_write(uint8_t *data, uint32_t len);

void proj_gnss_write(uint8_t *data, uint32_t len);

#endif
