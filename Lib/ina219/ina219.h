/*
 * ina219.h
 *
 *
 *
 *  Created on: Jan 27, 2025
 *      Author: yunus
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include <stdint.h>
#include <math.h>


typedef int8_t (*INA219_Read_t)(void* intf, uint8_t reg, uint8_t *pRxData, uint8_t len);
typedef int8_t (*INA219_Write_t)(void* intf, uint8_t reg, uint8_t *pTxData, uint8_t len);

typedef enum {
    INA219_BUSVOLTAGERANGE_16V = 0, 
    INA219_BUSVOLTAGERANGE_32V = 1  
} INA219_BusVoltageRange_e;

typedef enum {
    INA219_GAIN_40MV = 0,
    INA219_GAIN_80MV = 1,
    INA219_GAIN_160MV = 2,
    INA219_GAIN_320MV = 3
} INA219_Gain_e;

typedef enum {
    INA219_ADC_9BIT = 0,
    INA219_ADC_10BIT = 1,
    INA219_ADC_11BIT = 2,
    INA219_ADC_12BIT = 3
} INA219_ADCResolution_e;

typedef enum {
    INA219_MODE_POWER_DOWN = 0,
    INA219_MODE_SHUNT_VOLTAGE_TRIGGERED = 1,
    INA219_MODE_BUS_VOLTAGE_TRIGGERED = 2,
    INA219_MODE_SHUNT_BUS_TRIGGERED = 3,
    INA219_MODE_ADC_OFF = 4,
    INA219_MODE_SHUNT_VOLTAGE_CONTINUOUS = 5,
    INA219_MODE_BUS_VOLTAGE_CONTINUOUS = 6,
    INA219_MODE_SHUNT_BUS_CONTINUOUS = 7
} INA219_OperatingMode_e;

typedef struct INA219_Settings {
    INA219_BusVoltageRange_e bus_voltage_range;   
    INA219_Gain_e gain;                           
    INA219_ADCResolution_e shunt_adc_resolution;  
    INA219_ADCResolution_e bus_adc_resolution;    
    INA219_OperatingMode_e mode;				  
} INA219_Settings;

typedef struct INA219_Device_s {
    uint8_t chip_id;
    float r_shunt;
    float max_current;
    float current_lsb;
    float power_lsb;
    void* intf;         
    INA219_Read_t read; 
    INA219_Write_t write; 
    INA219_Settings settings;
} INA219_Device_t;


INA219_Device_t INA219_NewDevice(uint8_t chip_id, float r_shunt, float max_current,  void *intf, INA219_Read_t read, INA219_Write_t write);

/**
 * @brief Creates a new structure for the INA219 device.
 *
 * @param[in] chip_id      : Chip ID of the device.
 * @param[in] r_shunt      : Shunt resistance.
 * @param[in] max_current  : Maximum current.
 * @param[in] intf         : Interface (I2C).
 * @param[in] read         : Pointer to the read function.
 * @param[in] write        : Pointer to the write function.
 *
 * @return INA219_Device_t : New device structure.
 */

int8_t INA219_ReadRegister(INA219_Device_t *dev, uint8_t reg, uint16_t *pRxBuffer);

/**
 * @brief Reads a register from the INA219 device.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[in] reg       : Address of the register to read.
 * @param[out] pRxBuffer: Buffer to store the read data.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

int8_t INA219_WriteRegister(INA219_Device_t *dev, uint8_t reg, uint16_t *pTxBuffer);

/**
 * @brief Writes a register to the INA219 device.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[in] reg       : Address of the register to write.
 * @param[in] pTxBuffer : Buffer containing the data to write.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

void INA219_SetBusVoltage(INA219_Device_t *dev, INA219_BusVoltageRange_e range);

/**
 * @brief Sets the bus voltage range.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 * @param[in] range : Voltage range (16V or 32V).
 */

void INA219_SetGain(INA219_Device_t *dev, INA219_Gain_e range);

/**
 * @brief Sets the gain.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 * @param[in] range : Gain (40mV, 80mV, 160mV, 320mV).
 */

void INA219_SetBusAdcResolution(INA219_Device_t *dev, INA219_ADCResolution_e range);

/**
 * @brief Sets the bus ADC resolution.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 * @param[in] range : Bus ADC resolution (9,10,11,12 BIT)
 */

void INA219_SetShuntAdcResolution(INA219_Device_t *dev, INA219_ADCResolution_e range);

/**
 * @brief Sets the shunt ADC resolution.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 * @param[in] range : Shunt ADC resolution (9,10,11,12 BIT)
 */

void INA219_SetMode(INA219_Device_t *dev, INA219_OperatingMode_e range);

/**
 * @brief Sets the operating mode.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 * @param[in] range : Operating mode.
 */

void INA219_SetLsb(INA219_Device_t *dev);

/**
 * @brief Configures the current and power LSB settings.
 *
 * @param[in] dev   : Pointer to the INA219 device structure.
 */

void INA219_CalibReg(INA219_Device_t *dev);

/**
 * @brief Writes the calibration register.
 *
 * @param[in] dev : Pointer to the INA219 device structure.
 */

int8_t INA219_ReadBusVoltage(INA219_Device_t *dev, float *pRxBuffer);

/**
 * @brief Reads the bus voltage.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[out] pRxBuffer: Buffer to store the read voltage.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

int8_t INA219_ReadShuntVoltage(INA219_Device_t *dev, float *pRxBuffer);

/**
 * @brief Reads the shunt voltage.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[out] pRxBuffer: Buffer to store the read voltage.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

int8_t INA219_ReadCurrentAmper(INA219_Device_t *dev, float *pRxBuffer);

/**
 * @brief Reads the current in amperes.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[out] pRxBuffer: Buffer to store the read current.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

int8_t INA219_ReadPowerWatt(INA219_Device_t *dev, float *pRxBuffer);

/**
 * @brief Reads the power in watts.
 *
 * @param[in] dev       : Pointer to the INA219 device structure.
 * @param[out] pRxBuffer: Buffer to store the read power.
 *
 * @return int8_t       : Result of the operation. 0 for success, non-zero error code otherwise.
 */

void INA219_Reset(INA219_Device_t *dev);

/**
 * @brief Resets the INA219 device.
 *
 * @param[in] dev : Pointer to the INA219 device structure.
 */


#endif
