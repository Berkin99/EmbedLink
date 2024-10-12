/*
 *  mpu6500.h
 *
 *  Created on: Jan 12, 2024
 *  Author: BerkN
 *
 *  TDK IvenSense MPU6500 sensor driver.
 *  Platform independent C API.
 *  Calibration and raw data transformation.
 *  SPI or I2C based communication.
 *  Adaptive to changed settings.
 *
 *  Updates and bug reports :  @ https://github.com/Berkin99/MPU6500
 *
 *  12.01.2024 : Created MPU6500 module driver.
 *  03.10.2024 : Constructor updated, comments added.
 *
 *  References:
 *  [0] PS-MPU-6500A-01-v1.3.pdf (Datasheet)
 *  [1] MPU-6500-Register-Map2.pdf
 *
 */

#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdint.h>

#define MPU6500_OK       0
#define MPU6500_ERROR    1

typedef enum {
	MPU6500_ACCEL_RANGE_2G = 0,
	MPU6500_ACCEL_RANGE_4G,
	MPU6500_ACCEL_RANGE_8G,
	MPU6500_ACCEL_RANGE_16G
}MPU6500_AccelRange_e;

typedef enum {
	MPU6500_GYRO_RANGE_250DPS = 0,
	MPU6500_GYRO_RANGE_500DPS,
	MPU6500_GYRO_RANGE_1000DPS,
	MPU6500_GYRO_RANGE_2000DPS
}MPU6500_GyroRange_e;

typedef enum {
	MPU6500_DLPF_BAND_WIDTH_250HZ = 0,
	MPU6500_DLPF_BAND_WIDTH_184HZ,
	MPU6500_DLPF_BAND_WIDTH_92HZ,
	MPU6500_DLPF_BAND_WIDTH_41HZ,
	MPU6500_DLPF_BAND_WIDTH_20HZ,
	MPU6500_DLPF_BAND_WIDTH_10HZ,
	MPU6500_DLPF_BAND_WIDTH_5HZ
}MPU6500_DLPFBandWidth_e;

typedef enum {
	MPU6500_SAMPLE_RATE_DIVIDER_1000HZ = 0,
	MPU6500_SAMPLE_RATE_DIVIDER_500HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_333HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_250HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_200HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_167HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_143HZ,
	MPU6500_SAMPLE_RATE_DIVIDER_125HZ,
}MPU6500_SampleRateDivider_e;

typedef enum{
	MPU6500_CLOCK_INTERNAL = 0,
	MPU6500_CLOCK_PLL_XGYRO,
	MPU6500_CLOCK_PLL_YGYRO,
	MPU6500_CLOCK_PLL_ZGYRO,
	MPU6500_CLOCK_PLL_EXT32K,
	MPU6500_CLOCK_PLL_EXT19M,
	MPU6500_CLOCK_KEEP_RESET,
}MPU6500_ClockSource_e;

typedef enum {
    MPU6500_INTF_SPI,
    MPU6500_INTF_I2C
}MPU6500_Intf_e;

typedef int8_t (*MPU6500_Read_t)(void* intf, uint8_t reg, uint8_t *pRxData, uint8_t len);
typedef int8_t (*MPU6500_Write_t)(void* intf, uint8_t reg, const uint8_t *pTxData, uint8_t len);
typedef void   (*MPU6500_Delay_t)(uint32_t ms); /* Delay Microseconds function pointer */

struct MPU6500_Settings{
    MPU6500_AccelRange_e acc_range;
    MPU6500_GyroRange_e gyr_range;
    MPU6500_DLPFBandWidth_e dlpf;
    MPU6500_SampleRateDivider_e srd;
    MPU6500_ClockSource_e clock_source;
};

struct MPU6500_CalibData{
    int16_t acc_cal[3];
    float acc_scale[3][2]; /* 3 axis positive and negative */
    int16_t gyr_cal[3];
    float acc_coefficient; /* Coefficient for transform raw data to Gs */
    float gyr_coefficient; /* Coefficient for transform raw data to deg/s */
};

typedef struct MPU6500_Device_s{
    uint8_t chip_id;
    void* intf;
    MPU6500_Intf_e intf_type;
    MPU6500_Read_t read;      /* Read function pointer */
    MPU6500_Write_t write;    /* Write function pointer */
    MPU6500_Delay_t delay;    /* Delay Microseconds function pointer */
    struct MPU6500_CalibData calib_data;
    struct MPU6500_Settings settings;
}MPU6500_Device_t;

/*
 * @brief Creates a new MPU6500 device object.
 *
 * @param[in]  intf        : Pointer to the interface (SPI or I2C handle).
 * @param[in]  intf_type   : Interface type (I2C or SPI).
 * @param[in]  readf       : Function pointer to the read operation.
 * @param[in]  writef      : Function pointer to the write operation.
 * @param[in]  delayf      : Function pointer to the delay operation.
 *
 * @return -> MPU6500_Device_t construct.
 */
MPU6500_Device_t MPU6500_NewDevice(void* intf, MPU6500_Intf_e intf_type, MPU6500_Read_t readf, MPU6500_Write_t writef, MPU6500_Delay_t delayf);

/*
 * @brief Initializes the MPU6500 device and calibrates the sensor.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
int8_t MPU6500_Init(MPU6500_Device_t *dev);

/*
 * @brief Tests the connection to the MPU6500 device.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
int8_t MPU6500_Test(MPU6500_Device_t *dev);

/*
 * @brief Applies the settings to the MPU6500 device.
 *
 * @param[in]  dev      : Pointer to the MPU6500 device object.
 * @param[in]  settings : Structure containing the desired settings.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_ApplySettings(MPU6500_Device_t *dev, struct MPU6500_Settings settings);

/*
 * @brief Retrieves the current settings from the MPU6500 device.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_GetSettings(MPU6500_Device_t *dev);

/*
 * @brief Performs accelerometer calibration. Set device parallel to the surface.
 *  Do not move the device during the calibration.
 *
 * @param[in]  dev     : Pointer to the MPU6500 device object.
 * @param[in]  time_ms : Calibration duration in milliseconds.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_AccCalibration(MPU6500_Device_t *dev, uint32_t time_ms);

/*
 * @brief Performs gyroscope calibration.
 *  Do not move the device during the calibration.
 *
 * @param[in]  dev     : Pointer to the MPU6500 device object.
 * @param[in]  time_ms : Calibration duration in milliseconds.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_GyrCalibration(MPU6500_Device_t *dev, uint32_t time_ms);

/*
 * @brief Reads a register from the MPU6500 device.
 *
 * @param[in]  dev      : Pointer to the MPU6500 device object.
 * @param[in]  reg      : Register address to be read.
 * @param[out] pRxBuffer : Pointer to the receive buffer.
 * @param[in]  len      : Number of bytes to be read.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
int8_t MPU6500_ReadRegister(MPU6500_Device_t *dev, uint8_t reg, uint8_t* pRxBuffer, uint8_t len);

/*
 * @brief Writes to a register of the MPU6500 device.
 *
 * @param[in]  dev      : Pointer to the MPU6500 device object.
 * @param[in]  reg      : Register address to be written to.
 * @param[in]  pTxBuffer : Pointer to the transmit buffer.
 * @param[in]  len      : Number of bytes to be written.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
int8_t MPU6500_WriteRegister(MPU6500_Device_t *dev, uint8_t reg, const uint8_t* pTxBuffer, uint8_t len);

/*
 * @brief Retrieves the device ID of the MPU6500.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval Device ID
 */
uint8_t MPU6500_GetDeviceID(MPU6500_Device_t *dev);

/*
 * @brief Sets the sleep mode of the MPU6500 device.
 *
 * @param[in]  dev     : Pointer to the MPU6500 device object.
 * @param[in]  isSleep : 0 to wake, 1 to sleep.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetSleepMode(MPU6500_Device_t *dev, uint8_t isSleep);

/*
 * @brief Reads raw accelerometer and gyroscope data.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 * @param[out] acc  : Pointer to the accelerometer data array.
 * @param[out] gyr  : Pointer to the gyroscope data array.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
int8_t MPU6500_GetDataRaw(MPU6500_Device_t *dev, int16_t* acc, int16_t* gyr);

/*
 * @brief Retrieves processed accelerometer and gyroscope data.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 * @param[out] acc  : Pointer to the accelerometer data array (in Gs).
 * @param[out] gyr  : Pointer to the gyroscope data array (in degrees/second).
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_GetData(MPU6500_Device_t *dev, float* acc, float* gyr);

/*
 * @brief Sets the accelerometer range.
 *
 * @param[in]  dev   : Pointer to the MPU6500 device object.
 * @param[in]  range : Accelerometer range setting.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetAccelRange(MPU6500_Device_t *dev, MPU6500_AccelRange_e range);

/*
 * @brief Sets the gyroscope range.
 *
 * @param[in]  dev   : Pointer to the MPU6500 device object.
 * @param[in]  range : Gyroscope range setting.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetGyroRange(MPU6500_Device_t *dev, MPU6500_GyroRange_e range);

/*
 * @brief Sets the DLPF (Digital Low Pass Filter) bandwidth for the MPU6500 device.
 *
 * @param[in]  dev   : Pointer to the MPU6500 device object.
 * @param[in]  dlpf  : DLPF bandwidth setting.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetDLPFBandWidth(MPU6500_Device_t *dev, MPU6500_DLPFBandWidth_e dlpf);

/*
 * @brief  Sets the sample rate divider for the MPU6500 device.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 * @param[in]  srd  : Sample rate divider setting.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetSampleRateDivider(MPU6500_Device_t *dev, MPU6500_SampleRateDivider_e srd);

/*
 * @brief Sets the clock source for the MPU6500 device.
 *
 * @param[in]  dev          : Pointer to the MPU6500 device object.
 * @param[in]  clock_source : Clock source setting.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetClockSource(MPU6500_Device_t *dev, MPU6500_ClockSource_e clock_source);

/*
 * @brief Sets the accelerometer's DLPF bandwidth.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval 0  -> Success
 * @retval > 0 -> Warning or Error
 */
void MPU6500_SetAccelDLPFBandwidth(MPU6500_Device_t *dev);

/*
 * @brief Retrieves the accelerometer range setting.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval Accelerometer range setting.
 */
uint8_t MPU6500_GetAccelRange(MPU6500_Device_t *dev);

/*
 * @brief Retrieves the gyroscope range setting.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval Gyroscope range setting.
 */
uint8_t MPU6500_GetGyroRange(MPU6500_Device_t *dev);

/*
 * @brief Retrieves the DLPF bandwidth setting.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval DLPF bandwidth setting.
 */
uint8_t MPU6500_GetDLPFBandwidth(MPU6500_Device_t *dev);

/*
 * @brief Retrieves the sample rate divider setting.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval Sample rate divider setting.
 */
uint8_t MPU6500_GetSampleRateDivider(MPU6500_Device_t *dev);

/*
 * @brief Retrieves the clock source setting.
 *
 * @param[in]  dev  : Pointer to the MPU6500 device object.
 *
 * @retval Clock source setting.
 */
uint8_t MPU6500_GetClockSource(MPU6500_Device_t *dev);

#endif /* MPU6500_H_ */
