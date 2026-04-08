#ifndef SCH1600_H
#define SCH1600_H

#include <stdint.h>
#include <stdbool.h>

#define SCH1633_A05

// Raw data values from sensor
typedef struct _sch1600_raw_data
{

    int16_t acc_x_lsb;
    int16_t acc_y_lsb;
    int16_t acc_z_lsb;

    int16_t gyro_x_lsb;
    int16_t gyro_y_lsb;
    int16_t gyro_z_lsb;

    int16_t temp_lsb;

    //    uint32_t freq_cnt_lsb;

    bool frame_error;

    uint32_t time_stamp;

} sch1600_raw_data;

// Summed raw data values from sensor
typedef struct _sch1600_raw_data_summed
{

    int32_t acc_x_lsb;
    int32_t acc_y_lsb;
    int32_t acc_z_lsb;

    int32_t gyro_x_lsb;
    int32_t gyro_y_lsb;
    int32_t gyro_z_lsb;

    int32_t temp_lsb;

    //    uint32_t freq_cnt_lsb;
    uint32_t time_stamp;

} sch1600_raw_data_summed;

// Converted (real) data from sensor
typedef struct _sch1600_real_data
{

    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float temp;

    uint32_t time_stamp;

} sch1600_real_data;

// Sensor statuses
typedef struct _sch1600_sensor_status
{

    uint16_t summary_status;
    uint16_t saturation_status;
    uint16_t common_status;
    uint16_t rate_common_status;
    uint16_t rate_x_status;
    uint16_t rate_y_status;
    uint16_t rate_z_status;
    uint16_t acc_x_status;
    uint16_t acc_y_status;
    uint16_t acc_z_status;

} sch1600_sensor_status;

// SCH1600 library return codes
// Negative values = errors, positive values = warnings
// NOTE: Error code always overrides warning when returning from function
#define SCH1600_OK 0
#define SCH1600_NOK -1 // Component status not OK after all init steps

#define AVG_FACTOR 10.0f // Averaging factor outside the sensor
#define GRAVITY_CORRECT_FACTOR 0.026f // Averaging factor outside the sensor

extern void sch1600_reset(void);
extern int sch1600_init(void);
extern void sch1600_get_serial(char *serial_num);
extern void sch1600_read_data(sch1600_raw_data *data);
extern void sch1600_convert_data(sch1600_raw_data_summed *data_in, sch1600_real_data *data_out);
extern void sch1600_read_sensor_status(sch1600_sensor_status *status);
extern bool sch1600_verify_sensor_status(sch1600_sensor_status *status);

#endif // SCH1600_H
