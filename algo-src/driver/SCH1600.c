#include "SCH1600.h"

#ifdef PLATFORM_MCU
#include "hc32_ll.h"
#endif // P

#include <stdio.h>

/*----------------------------------------------------------------------------
    Define sensitivities for selected product type
 *----------------------------------------------------------------------------*/

// One of these has to be defined in config.h

#if !defined SCH1633_A05
#error Invalid product type or no product type defined
#endif

#define SENSITIVITY_GYRO 100.0f // LSB/dps, DYN2 Nominal Sensitivity for 16bit data.
#define SENSITIVITY_ACC 200.0f  // LSB/m/s2, DYN1 Nominal Sensitivity for 16bit data. SENSITIVITY_ACC=200 in user mannual

#define FREQ_CNT_NOM_FREQ ((1024.0f * 23600.0f) / 16.0f) // ASIC master clock MCLK = 1024 * F_PRIM
                                                         // Clock for FREQ_CNT = MCLK / 16

// Macros for parsing values from sensor MISO words
#define SPI32_DATA_INT16(a) ((int16_t)(((a) >> 4) & 0xffff))
#define SPI32_DATA_UINT16(a) ((uint16_t)(((a) >> 4) & 0xffff))
#define SPI48_DATA_INT32(a) (((int32_t)(((a) << 4) & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a) ((uint32_t)(((a) >> 8) & 0x000fffffUL))
#define SPI48_DATA_UINT16(a) ((uint16_t)(((a) >> 8) & 0x0000ffffUL))
#define SPI48_CNTR_UINT8(a) ((uint8_t)(((a) >> 29) & 0x0000000fUL))
#define GET_TEMPERATURE(a) ((a) / 100.0f)

// Pre-calculated 32 bit SPI frames (FT bit = 0) for various operations.
// NOTE: these have TA9 and TA8 bits set to zero, modify if needed.

// RATE filter settings
#define REQ32_WRITE_FILTER_13HZ_RATE 0xC9600495  // Set 13 Hz filter for rate (LPF2)
#define REQ32_WRITE_FILTER_30HZ_RATE 0xC960024B  // Set 30 Hz filter for rate (LPF1)
#define REQ32_WRITE_FILTER_68HZ_RATE 0xC9600001  // Set 68 Hz filter for rate (LPF0)
#define REQ32_WRITE_FILTER_280HZ_RATE 0xC96006DF // Set 280 Hz filter for rate (LPF3)
#define REQ32_WRITE_FILTER_370HZ_RATE 0xC9600922 // Set 370 Hz filter for rate (LPF4)

// ACC filter settings
#define REQ32_WRITE_FILTER_13HZ_ACC12 0xC9A00493  // Set 13 Hz filter for acceleration (LPF2)
#define REQ32_WRITE_FILTER_30Hz_ACC12 0xC9A0024D  // Set 30 Hz filter for acceleration (LPF1)
#define REQ32_WRITE_FILTER_68Hz_ACC12 0xC9A00007  // Set 68 Hz filter for acceleration (LPF0)
#define REQ32_WRITE_FILTER_280Hz_ACC12 0xC9A006D9 // Set 280 Hz filter for acceleration (LPF3)
#define REQ32_WRITE_FILTER_370Hz_ACC12 0xC9A00924 // Set 370 Hz filter for acceleration (LPF4)

// RATE and ACC ranges and data rate reduction (decimation) factors
#define REQ32_WRITE_CTRL_RATE_DEC1 0xCA212007  // Set RATE range to 315 dps, decimation factor 1 (DYN2, DEC1)
#define REQ32_WRITE_CTRL_RATE_DEC3 0xCA212493  // Set RATE range to 315 dps, decimation factor 4 (DYN2, DEC3)
#define REQ32_WRITE_CTRL_RATE_DEC5 0xCA212924  // Set RATE range to 315 dps, decimation factor 16 (DYN2, DEC5)
#define REQ32_WRITE_CTRL_ACC12_DEC1 0xCA609001 // Set ACC range to 80 m/s2, decimation factor 1 (DYN1, DEC1)
#define REQ32_WRITE_CTRL_ACC12_DEC3 0xCA609495 // Set ACC range to 80 m/s2, decimation factor 4 (DYN1, DEC3)
#define REQ32_WRITE_CTRL_ACC12_DEC5 0xCA609922 // Set ACC range to 80 m/s2, decimation factor 16 (DYN1, DEC5)

// Status registers
#define REQ32_READ_STAT_SUM 0xC5000000 // Read status summary for non-saturation related flags
#define REQ32_READ_STAT_SUM_SAT 0xC5400002
#define REQ32_READ_STAT_COM 0xC5800004
#define REQ32_READ_STAT_RATE_COM 0xC5C00006
#define REQ32_READ_STAT_RATE_X 0xC6000005
#define REQ32_READ_STAT_RATE_Y 0xC6400007
#define REQ32_READ_STAT_RATE_Z 0xC6800001
#define REQ32_READ_STAT_ACC_X 0xC6C00003
#define REQ32_READ_STAT_ACC_Y 0xC7000006
#define REQ32_READ_STAT_ACC_Z 0xC7400004

// Sensor mode control
#define REQ32_WRITE_EN_SENSOR 0xCD60000D // Enable RATE and ACC measurement
#define REQ32_WRITE_EOI 0xCD60001B       // Write EOI bit and keep EN_SENSORs on as well

// Component serial number registers
#define REQ32_READ_SN_ID1 0xCF400001
#define REQ32_READ_SN_ID2 0xCF800007
#define REQ32_READ_SN_ID3 0xCFC00005

// 32 bit frame masks
#define MASK32_ERROR 0x00100008
#define MASK32_FT 0x00080000
#define MASK32_TA9 0x80000000
#define MASK32_TA8 0x40000000

// Pre-calculated 48 bit SPI frames (FT bit = 1) for various operations.
// NOTE: these have TA9 and TA8 bits set to zero, modify if needed.

// RATE filter settings
#define REQ48_WRITE_FILTER_13HZ_RATE 0x096800009205UL  // Set 13 Hz filter for rate (LPF2)
#define REQ48_WRITE_FILTER_30HZ_RATE 0x096800004988UL  // Set 30 Hz filter for rate (LPF1)
#define REQ48_WRITE_FILTER_68HZ_RATE 0x096800000016UL  // Set 68 Hz filter for rate (LPF0)
#define REQ48_WRITE_FILTER_280HZ_RATE 0x09680000DB9BUL // Set 280 Hz filter for rate (LPF3)

// ACC filter settings
#define REQ48_WRITE_FILTER_13HZ_ACC12 0x09A800009233UL  // Set 13 Hz filter for acceleration (LPF2)
#define REQ48_WRITE_FILTER_30Hz_ACC12 0x09A8000049BEUL  // Set 30 Hz filter for acceleration (LPF1)
#define REQ48_WRITE_FILTER_68Hz_ACC12 0x09A800000020UL  // Set 68 Hz filter for acceleration (LPF0)
#define REQ48_WRITE_FILTER_280Hz_ACC12 0x09A80000DBADUL // Set 280 Hz filter for acceleration (LPF3)

// RATE and ACC ranges and data rate reduction (decimation) factors
#define REQ48_WRITE_CTRL_RATE_DEC1 0x0A28002400A6UL  // Set RATE range to 315 dps, decimation factor 1 (DYN2, DEC1)
#define REQ48_WRITE_CTRL_RATE_DEC3 0x0A28002492B5UL  // Set RATE range to 315 dps, decimation factor 4 (DYN2, DEC3)
#define REQ48_WRITE_CTRL_RATE_DEC5 0x0A2800252480UL  // Set RATE range to 315 dps, decimation factor 16 (DYN2, DEC5)
#define REQ48_WRITE_CTRL_ACC12_DEC1 0x0A6800120016UL // Set ACC range to 80 m/s2, decimation factor 1 (DYN1, DEC1)
#define REQ48_WRITE_CTRL_ACC12_DEC3 0x0A6800129205UL // Set ACC range to 80 m/s2, decimation factor 4 (DYN1, DEC3)
#define REQ48_WRITE_CTRL_ACC12_DEC5 0x0A6800132430UL // Set ACC range to 80 m/s2, decimation factor 16 (DYN1, DEC5)

// Status registers
#define REQ48_READ_STAT_SUM 0x05080000001CUL // Read status summary for non-saturation related flags
#define REQ48_READ_STAT_SUM_SAT 0x0548000000EBUL
#define REQ48_READ_STAT_COM 0x0588000000DDUL
#define REQ48_READ_STAT_RATE_COM 0x05C80000002AUL
#define REQ48_READ_STAT_RATE_X 0x0608000000C4UL
#define REQ48_READ_STAT_RATE_Y 0x064800000033UL
#define REQ48_READ_STAT_RATE_Z 0x068800000005UL
#define REQ48_READ_STAT_ACC_X 0x06C8000000F2UL
#define REQ48_READ_STAT_ACC_Y 0x070800000069UL
#define REQ48_READ_STAT_ACC_Z 0x07480000009EUL

// Sensor mode control
#define REQ48_WRITE_EN_SENSOR 0x0D68000001D3UL // Enable RATE and ACC measurement
#define REQ48_WRITE_EOI 0x0D680000038DUL       // Write EOI bit and keep EN_SENSORs on as well

// Interpolated RATE and ACC outputs
#define REQ32_READ_GYRO_X1 0xC0400006
#define REQ32_READ_GYRO_Y1 0xC0800000
#define REQ32_READ_GYRO_Z1 0xC0C00002
#define REQ32_READ_ACC_X1 0xC1000007
#define REQ32_READ_ACC_Y1 0xC1400005
#define REQ32_READ_ACC_Z1 0xC1800003
// Decimated RATE and ACC outputs
#define REQ32_READ_GYRO_X2 0xC2800006
#define REQ32_READ_GYRO_Y2 0xC2C00004
#define REQ32_READ_GYRO_Z2 0xC3000001
#define REQ32_READ_ACC_X2 0xC3400003
#define REQ32_READ_ACC_Y2 0xC3800005
#define REQ32_READ_ACC_Z2 0xC3C00007

#define REQ32_READ_TEMP 0xC4000003            // Read temperature
#define REQ48_READ_FREQ_CNTR 0x04C800000087UL // Read Frequency Counter

// Component serial number registers
#define REQ48_READ_SN_ID1 0x0F4800000065UL
#define REQ48_READ_SN_ID2 0x0F8800000053UL
#define REQ48_READ_SN_ID3 0x0FC8000000A4UL

// 48 bit frame masks
#define MASK48_ERROR 0x001E00000000UL
#define MASK48_FT 0x000800000000UL
#define MASK48_TA9 0x800000000000UL
#define MASK48_TA8 0x400000000000UL

/**

    Static function prototypes

**/

static bool check_32bit_frame_error(uint32_t *data, int size);
static bool check_48bit_frame_error(uint64_t *data, int size);
static uint32_t insert_TA32(uint32_t frame_to_be_modified, bool TA9, bool TA8);
static uint64_t insert_TA48(uint64_t frame_to_be_modified, bool TA9, bool TA8);
static uint8_t CRC8(uint64_t input);
static uint8_t CRC3(uint32_t input);
static uint32_t spi32_send_request(uint32_t txbuf);
static void wait_ms(int time);

/**

    SPI communication

**/
uint32_t spi32_send_request(uint32_t tx_buf)
{
    uint32_t rx_buf = 0;
    int32_t ret;

    GPIO_ResetPins(GPIO_PORT_B, GPIO_PIN_01);
    ret = SPI_TransReceive(CM_SPI2, &tx_buf, &rx_buf, 1, 100);
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_01);

    if (ret != LL_OK)
    {
        if (ret == LL_ERR_TIMEOUT)
        {
            return 0xDDDDDDDD;
        }
        if (ret == LL_ERR_INVD_PARAM)
        {
            return 0xEEEEEEEE;
        }
    }
    return rx_buf;
}
/**

    Internal data structures

**/
void wait_ms(int time)
{
    DDL_DelayMS(time);
}

/**

    Perform SCH1600 reset

**/
void sch1600_reset(void)
{
    GPIO_ResetPins(GPIO_PORT_B, GPIO_PIN_12); // Set EXTRESN active
    wait_ms(2);
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_12); // Set EXTRESN inactive
}

/**

    Initialize SCH1600 sensor

    Parameters:     None


    Return value:   SCH1600_OK       = success, SCH1600_ERR_* = failure.
                                       See header file for definitions.

**/
int sch1600_init(void)
{
    int attempt;
    bool sensor_status = false;
    sch1600_sensor_status status;
    // Init sequence specified in section "5 Component Operation, Reset and Power Up" in
    // the data sheet

    // Power on.
    // Wait until voltage supplies are inside the spec (3...3.6V). Wait done in hw_init().

    sch1600_reset(); // Reset sensor

    for (attempt = 0; attempt < 2; attempt++)
    {

        // Wait 32 ms for the non-volatile memory (NVM) Read
        wait_ms(32);

        // Set user controls
        spi32_send_request(REQ32_WRITE_FILTER_13HZ_RATE);  // Set 13Hz filter for RATE
        spi32_send_request(REQ32_WRITE_FILTER_13HZ_ACC12); // Set 13Hz filter for ACC
        spi32_send_request(REQ32_WRITE_CTRL_RATE_DEC3);    // Set 315 dps range and decimation factor 4 for RATE
        spi32_send_request(REQ32_WRITE_CTRL_ACC12_DEC3);   // Set 80 m/s2 range and decimation factor 4 for ACC

        // Write EN_SENSOR = 1
        spi32_send_request(REQ32_WRITE_EN_SENSOR);

        // Wait 250 ms
        wait_ms(250);

        // Read all status registers once
        sch1600_read_sensor_status(&status);

        // Set EOI=1 (End of Initialization command)
        spi32_send_request(REQ32_WRITE_EOI);
        wait_ms(3); // 5.2 Write EOI=1��Wait 3ms
        // Read all status registers twice
        sch1600_read_sensor_status(&status);
        sch1600_read_sensor_status(&status);

        // Read all user control registers and verify content. Add verification here if needed for FuSa.

        // Read summary status to clear various error flags in sensor status
        // registers, which are normal during internal startup tests.
        spi32_send_request(REQ32_READ_STAT_SUM);

        // Check that all status register have OK status
        if (!sch1600_verify_sensor_status(&status))
        {
            sensor_status = false;
            sch1600_reset(); // Reset sensor
        }
        else
        {
            sensor_status = true;
            break;
        }

    } // for (attempt = 0; attempt < 2; attempt++)

    // System in operation mode and data ready to be read
    if (sensor_status == true)
    {
        return SCH1600_OK;
    }

    // System in FAILURE mode
    return SCH1600_NOK;
}

/**

    Read SCH1600 sensor serial number

    Parameters:       *serial_num       - pointer to a string containing the serial number

    Return value:     None

**/
void sch1600_get_serial(char *serial_num)
{
    uint16_t sn_id1;
    uint16_t sn_id2;
    uint16_t sn_id3;

    spi32_send_request(REQ32_READ_SN_ID1);
    sn_id1 = SPI32_DATA_UINT16(spi32_send_request(REQ32_READ_SN_ID2));
    sn_id2 = SPI32_DATA_UINT16(spi32_send_request(REQ32_READ_SN_ID3));
    sn_id3 = SPI32_DATA_UINT16(spi32_send_request(REQ32_READ_SN_ID3));

    // Build serial number string
    snprintf(serial_num, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
}

/**

    Read acceleration, rate and temperature data from sensor. Called by sampling_callback()

    Parameters:       *data     - pointer to "raw" data from sensor

    Return value:     None

**/
void sch1600_read_data(sch1600_raw_data *data)
{
    spi32_send_request(REQ32_READ_GYRO_X2);
    uint32_t gyro_x_lsb = spi32_send_request(REQ32_READ_GYRO_Y2);
    uint32_t gyro_y_lsb = spi32_send_request(REQ32_READ_GYRO_Z2);
    uint32_t gyro_z_lsb = spi32_send_request(REQ32_READ_ACC_X2);
    uint32_t acc_x_lsb = spi32_send_request(REQ32_READ_ACC_Y2);
    uint32_t acc_y_lsb = spi32_send_request(REQ32_READ_ACC_Z2);
    uint32_t acc_z_lsb = spi32_send_request(REQ32_READ_TEMP);
    uint32_t temp_lsb = spi32_send_request(REQ32_READ_TEMP);

    // Get possible frame errors
    uint32_t miso_words[] = {gyro_x_lsb, gyro_y_lsb, gyro_z_lsb, acc_x_lsb, acc_y_lsb, acc_z_lsb, temp_lsb};

    data->frame_error = check_32bit_frame_error(miso_words, (sizeof(miso_words) / sizeof(uint32_t)));

    // Parse MISO data to structure
    data->acc_x_lsb = SPI32_DATA_INT16(acc_x_lsb);
    data->acc_y_lsb = SPI32_DATA_INT16(acc_y_lsb);
    data->acc_z_lsb = SPI32_DATA_INT16(acc_z_lsb);

    data->gyro_x_lsb = SPI32_DATA_INT16(gyro_x_lsb);
    data->gyro_y_lsb = SPI32_DATA_INT16(gyro_y_lsb);
    data->gyro_z_lsb = SPI32_DATA_INT16(gyro_z_lsb);

    // Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
    data->temp_lsb = SPI32_DATA_INT16(temp_lsb);
}

/**

    Check if 32-bit MISO frames have any error bits set. Return true on the 1st error encountered.

    Parameters:           *data         - pointer to 32-bit MISO frames from sensor
                          size          - number of frames to check

    Return value:         true          = any error bit set
                          false         = no error

**/
static bool check_32bit_frame_error(uint32_t *data, int size)
{
    for (int i = 0; i < size; i++)
    {
        uint32_t value = data[i];
        if (value & MASK32_ERROR)
        {
            return true;
        }
    }

    return false;
}

/**

    Check if 48-bit MISO frames have any error bits set. Return true on the 1st error encountered.

    Parameters:          *data         - pointer to 48-bit MISO frames from sensor
                         size          - number of frames to check

    Return value:        true          = any error bit set
                         false         = no error

**/
static bool check_48bit_frame_error(uint64_t *data, int size)
{
    for (int i = 0; i < size; i++)
    {
        uint64_t value = data[i];
        if (value & MASK48_ERROR)
        {
            return true;
        }
    }

    return false;
}

/**

      Convert summed raw binary data from sensor to real values. Also calculate averages values.

      Parameters:         *data_in      - pointer to summed "raw" data from sensor
                          *data_out     - pointer to converted values

      Return value:       None

*/
void sch1600_convert_data(sch1600_raw_data_summed *data_in, sch1600_real_data *data_out)
{
    data_out->acc_x = data_in->acc_x_lsb;
    data_out->acc_y = data_in->acc_y_lsb;
    data_out->acc_z = data_in->acc_z_lsb;
    data_out->gyro_x = data_in->gyro_x_lsb;
    data_out->gyro_y = data_in->gyro_y_lsb;
    data_out->gyro_z = data_in->gyro_z_lsb;

    // Convert from LSB to sensitivity and calculate averages here for faster execution
    data_out->acc_x = data_out->acc_x / (SENSITIVITY_ACC * AVG_FACTOR);
    data_out->acc_y = data_out->acc_y / (SENSITIVITY_ACC * AVG_FACTOR);
    data_out->acc_z = data_out->acc_z / (SENSITIVITY_ACC * AVG_FACTOR);
    data_out->gyro_x = data_out->gyro_x / (SENSITIVITY_GYRO * AVG_FACTOR);
    data_out->gyro_y = data_out->gyro_y / (SENSITIVITY_GYRO * AVG_FACTOR);
    data_out->gyro_z = data_out->gyro_z / (SENSITIVITY_GYRO * AVG_FACTOR);

    // Convert temperature and calculate average
    data_out->temp = GET_TEMPERATURE((float)data_in->temp_lsb / (1.0 * AVG_FACTOR));
    data_out->time_stamp = data_in->time_stamp;
}

/**

    Read sensor status

    Parameters:         *status       - pointer to sensor status structure where the data is read to

    Return value:       None

**/
void sch1600_read_sensor_status(sch1600_sensor_status *status)
{

    spi32_send_request(REQ32_READ_STAT_SUM);
    uint32_t stat_sum = spi32_send_request(REQ32_READ_STAT_SUM_SAT);
    uint32_t stat_sum_sat = spi32_send_request(REQ32_READ_STAT_COM);
    uint32_t stat_com = spi32_send_request(REQ32_READ_STAT_RATE_COM);
    uint32_t stat_rate_com = spi32_send_request(REQ32_READ_STAT_RATE_X);
    uint32_t stat_rate_x = spi32_send_request(REQ32_READ_STAT_RATE_Y);
    uint32_t stat_rate_y = spi32_send_request(REQ32_READ_STAT_RATE_Z);
    uint32_t stat_rate_z = spi32_send_request(REQ32_READ_STAT_ACC_X);
    uint32_t stat_acc_x = spi32_send_request(REQ32_READ_STAT_ACC_Y);
    uint32_t stat_acc_y = spi32_send_request(REQ32_READ_STAT_ACC_Z);
    uint32_t stat_acc_z = spi32_send_request(REQ32_READ_STAT_ACC_Z);

    status->summary_status = SPI32_DATA_UINT16(stat_sum);
    status->saturation_status = SPI32_DATA_UINT16(stat_sum_sat);
    status->common_status = SPI32_DATA_UINT16(stat_com);
    status->rate_common_status = SPI32_DATA_UINT16(stat_rate_com);
    status->rate_x_status = SPI32_DATA_UINT16(stat_rate_x);
    status->rate_y_status = SPI32_DATA_UINT16(stat_rate_y);
    status->rate_z_status = SPI32_DATA_UINT16(stat_rate_z);
    status->acc_x_status = SPI32_DATA_UINT16(stat_acc_x);
    status->acc_y_status = SPI32_DATA_UINT16(stat_acc_y);
    status->acc_z_status = SPI32_DATA_UINT16(stat_acc_z);
}

/**

    Verify sensor status

    Parameters:         *status       - pointer to sensor status structure

    Return value:       true          = all status registers show OK status
                        false         = one or more status registers had a NOT OK status

**/
bool sch1600_verify_sensor_status(sch1600_sensor_status *status)
{

    if (status->summary_status != 0xffff)
        return false;
    if (status->saturation_status != 0xffff)
        return false;
    if (status->common_status != 0xffff)
        return false;
    if (status->rate_common_status != 0xffff)
        return false;
    if (status->rate_x_status != 0xffff)
        return false;
    if (status->rate_y_status != 0xffff)
        return false;
    if (status->rate_z_status != 0xffff)
        return false;
    if (status->acc_x_status != 0xffff)
        return false;
    if (status->acc_y_status != 0xffff)
        return false;
    if (status->acc_z_status != 0xffff)
        return false;

    return true;
}

/**

    Set/reset TA9 and TA8 bits in a 32-bit frame. If using only one DUT, this function can be removed.

    Parameters:             frame_to_be_modified        - 32-bit frame for modification
                            TA9                         - State of TA9 pin
                            TA8                         - State of TA8 pin

    Return value:           modified frame

**/
static uint32_t insert_TA32(uint32_t frame_to_be_modified, bool TA9, bool TA8)
{
    uint32_t output_frame;
    uint32_t new_crc;

    if (TA9)
    {
        output_frame = frame_to_be_modified | MASK32_TA9;
    }
    else
    {
        output_frame = frame_to_be_modified & (~MASK32_TA9);
    }

    if (TA8)
    {
        output_frame = frame_to_be_modified | MASK32_TA8;
    }
    else
    {
        output_frame = frame_to_be_modified & (~MASK32_TA8);
    }

    // Reset crc bits
    output_frame = output_frame & 0xFFFFFFF8;

    // Calculate new crc
    new_crc = CRC3(output_frame);

    // Replace crc bits
    output_frame = output_frame | new_crc;

    return output_frame;
}

/**

    Set/reset TA9 and TA8 bits in a 48-bit frame. If using only one DUT, this function can be removed.

    Parameters:               frame_to_be_modified        - 48-bit frame for modification
                              TA9                         - State of TA9 pin
                              TA8                         - State of TA8 pin

    Return value:             modified frame

**/
static uint64_t insert_TA48(uint64_t frame_to_be_modified, bool TA9, bool TA8)
{
    uint64_t output_frame;
    uint64_t new_crc;

    if (TA9)
    {
        output_frame = frame_to_be_modified | MASK48_TA9;
    }
    else
    {
        output_frame = frame_to_be_modified & (~MASK48_TA9);
    }

    if (TA8)
    {
        output_frame = frame_to_be_modified | MASK48_TA8;
    }
    else
    {
        output_frame = frame_to_be_modified & (~MASK48_TA8);
    }

    // Reset crc bits
    output_frame = output_frame & 0xFFFFFFFFFF00;

    // Calculate new crc
    new_crc = CRC8(output_frame);

    // Replace crc bits
    output_frame = output_frame | new_crc;

    return output_frame;
}

/**

    Calculate crc3 for a 32-bit frame. Needed if insert_TA32() or FT32() functions are used.

    Parameters:           input               - 32-bit frame for calculation

    Return value:         crc of the frame

**/
uint8_t CRC3(uint32_t input)
{
    uint32_t data = input & 0xFFFFFFF8;
    uint8_t t[] = {1, 0, 1};
    uint8_t temp[3];
    uint8_t crc = 0;

    for (int i = 31; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;

        temp[2] = t[1];
        temp[1] = t[0] ^ t[2];
        temp[0] = data_bit ^ t[2];

        for (int j = 0; j < 3; j++)
        {
            t[j] = temp[j];
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (t[i])
        {
            crc = crc | (0x01 << i);
        }
    }

    return crc;
}

/**

    Calculate crc8 for a 48-bit frame. Needed if insert_TA48() function is used.

    Parameters:           input                - 48-bit frame for calculation

    Return value:         crc of the frame

**/
uint8_t CRC8(uint64_t input)
{
    uint64_t data = input & 0xFFFFFFFFFF00;
    uint8_t t[] = {1, 1, 1, 1, 1, 1, 1, 1};
    uint8_t temp[8];
    uint8_t crc = 0;

    for (int i = 47; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;

        temp[7] = t[6];
        temp[6] = t[5];
        temp[5] = t[4] ^ t[7];
        temp[4] = t[3];
        temp[3] = t[2] ^ t[7];
        temp[2] = t[1] ^ t[7];
        temp[1] = t[0] ^ t[7];
        temp[0] = data_bit ^ t[7];

        for (int j = 0; j < 8; j++)
        {
            t[j] = temp[j];
        }
    }

    for (int i = 0; i < 8; i++)
    {
        if (t[i])
        {
            crc = crc | (0x01 << i);
        }
    }

    return crc;
}
