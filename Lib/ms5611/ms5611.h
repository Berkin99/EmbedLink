/*
 *  ms5611.h
 *
 *  Created on: Oct 3, 2023
 *  Author: BerkN
 *
 *  TE Connectivity MS5611 sensor driver.
 *  Platform independent C API.
 *  Calibration and raw data transformation.
 *  SPI or I2C based communication.
 *  Adaptive to changed settings.
 *
 *  Updates and bug reports :  @ https://github.com/Berkin99/MS5611
 *
 *  03.11.2023 : Created for STM32 HAL library.
 *  05.01.2024 : Test Connection Routine Change.
 *  07.04.2024 : Generic I2C functions used.
 *  04.10.2024 : Object system.
 *
 *  References:
 *  [0] ENG_DS_MS5611-01BA03_B3.pdf (Datasheet)
 *
 */

#ifndef MS5611_H_
#define MS5611_H_

#include <stdint.h>

#define MS5611_OK             0
#define MS5611_ERROR          1

#define MS5611_I2C_ADDRESS    0x77

typedef enum{
    MS5611_INTF_SPI,
    MS5611_INTF_I2C
}MS5611_Intf_e;

typedef enum
{
    MS5611_ULTRA_LOW_POWER  = 0,        /* 1 ms conversion time. */
    MS5611_LOW_POWER,                   /* 2 ms conversion time. */
    MS5611_STANDARD,                    /* 3 ms conversion time. */
    MS5611_HIGH_RES,                    /* 5 ms conversion time. */
    MS5611_ULTRA_HIGH_RES,              /* 10 ms conversion time.*/
} MS5611_OSRate_t;                      /* Output Sampling Rate  */

typedef struct MS5611_Data_s{
    int32_t temperature;  /* celcius * 10^2 */
    int32_t pressure;     /* mbar * 10^2 */
}MS5611_Data_t;

typedef int8_t (*MS5611_Read_t)(void* intf, uint8_t reg, uint8_t *pRxData, uint8_t len);
typedef int8_t (*MS5611_Write_t)(void* intf, uint8_t reg, const uint8_t *pTxData, uint8_t len);
typedef void   (*MS5611_Delay_t)(uint32_t ms); /* Delay Microseconds function pointer */

typedef struct MS5611_Config_s
{
    MS5611_OSRate_t osRate; /* Output Sampling Rate */
    uint8_t ct;             /* Conversion Time */
    float C[7];             /* Coefficients */
}MS5611_Config_t;

typedef struct MS5611_Device_s
{
    void* intf;
    MS5611_Intf_e intf_type;
    MS5611_Read_t read;
    MS5611_Write_t write;
    MS5611_Delay_t delay;
    MS5611_Config_t config;
}MS5611_Device_t;

/*
 * @brief Creates and initializes a new MS5611 device instance.
 *
 * @param[in] intf       : Interface instance (SPI/I2C).
 * @param[in] intf_type  : Interface type (SPI or I2C).
 * @param[in] readf      : Pointer to the read function.
 * @param[in] writef     : Pointer to the write function.
 * @param[in] delayf     : Pointer to the delay function.
 *
 * @return MS5611_Device_t  : Initialized MS5611 device structure.
 */
MS5611_Device_t MS5611_NewDevice(void* intf, MS5611_Intf_e intf_type, MS5611_Read_t readf, MS5611_Write_t writef, MS5611_Delay_t delayf);

/*
 * @brief Initializes the MS5611 device by resetting it and loading the default settings.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 *
 * @retval 0 -> Success
 * @retval > 0 -> Failure
 */
int8_t MS5611_Init(MS5611_Device_t* dev);

/*
 * @brief Performs a self-test on the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 *
 * @retval 0 -> Success
 * @retval > 0 -> Failure
 */
int8_t MS5611_Test(MS5611_Device_t* dev);

/*
 * @brief Sends a reset command to the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 *
 * @return void
 */
void MS5611_Reset(MS5611_Device_t* dev);

/*
 * @brief Initializes calibration constants for the MS5611 device.
 *
 * @param[in] dev       : Pointer to the MS5611 device structure.
 * @param[in] mathMode  : Specifies whether to use alternative math mode.
 *
 * @return void
 */
void MS5611_InitConstants(MS5611_Device_t* dev, int8_t mathMode);

/*
 * @brief Reads and stores PROM calibration data for the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 *
 * @retval 0 -> Success
 * @retval > 0 -> Failure
 */
int8_t MS5611_PROM(MS5611_Device_t* dev);

/*
 * @brief Reads calibration data from the MS5611 PROM for a given register.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 * @param[in] reg  : Register address to read from.
 *
 * @return uint16_t  : Calibration data from the specified register.
 */
uint16_t MS5611_ReadPROM(MS5611_Device_t* dev, uint8_t reg);

/*
 * @brief Sets the output sampling rate (OSR) for the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 * @param[in] osr  : Desired output sampling rate.
 *
 * @return void
 */
void MS5611_SetOSRate(MS5611_Device_t* dev, MS5611_OSRate_t osr);

/*
 * @brief Gets the current output sampling rate (OSR) from the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 *
 * @return MS5611_OSRate_t  : Current OSR value.
 */
MS5611_OSRate_t MS5611_GetOSRate(MS5611_Device_t* dev);

/*
 * @brief Retrieves the processed temperature and pressure data from the MS5611 device.
 *
 * @param[in] dev     : Pointer to the MS5611 device structure.
 * @param[out] pTemp  : Pointer to store the temperature value (in °C).
 * @param[out] pPress : Pointer to store the pressure value (in mbar).
 *
 * @retval 0 -> Success
 * @retval > 0 -> Failure
 */
int8_t MS5611_GetData(MS5611_Device_t* dev, float* pTemp, float* pPress);

/*
 * @brief Initiates a conversion process on the MS5611 device.
 *
 * @param[in] dev  : Pointer to the MS5611 device structure.
 * @param[in] addr : Command to initiate conversion.
 *
 * @return void
 */
void MS5611_Convert(MS5611_Device_t* dev, const uint8_t addr);

/*
 * @brief Reads the ADC result from the MS5611 device after a conversion.
 *
 * @param[in] dev    : Pointer to the MS5611 device structure.
 * @param[out] buffer: Pointer to store the ADC result.
 *
 * @retval 0 -> Success
 * @retval > 0 -> Failure
 */
int8_t MS5611_AdcRead(MS5611_Device_t* dev, uint32_t* buffer);

/*
 * @brief Processes raw ADC data to calculate temperature and pressure values.
 *
 * @param[in] dev          : Pointer to the MS5611 device structure.
 * @param[in] D1           : Raw pressure data.
 * @param[in] D2           : Raw temperature data.
 * @param[in] compensation : Flag to apply temperature compensation.
 *
 * @return MS5611_Data_t  : Processed temperature and pressure data.
 */
MS5611_Data_t MS5611_RawDataProcess(MS5611_Device_t* dev, uint32_t D1, uint32_t D2, int8_t compensation);

#endif /* MS5611_H_ */
