/*
 * ina219.c
 *
 *  Created on: Jan 27, 2025
 *      Author: yunus
 */

#include "ina219.h"
#include <math.h>
#include "main.h"

#define REG_CONFIG 0x00
#define REG_SHUNTVOLTAGE 0x01
#define REG_BUSVOLTAGE 0x02
#define REG_POWER 0x03
#define REG_CURRENT 0x04
#define REG_CALIBRATION 0x05

INA219_Device_t INA219_NewDevice(uint8_t chip_id, float r_shunt, float max_current,  void *intf, INA219_Read_t read, INA219_Write_t write)
{
    INA219_Device_t dev;
    dev.chip_id = chip_id;
    dev.r_shunt = r_shunt;
    dev.max_current = max_current;
    dev.intf = intf;
    dev.read = read;
    dev.write = write;

    INA219_SetLsb(&dev);

    return dev;
}

int8_t INA219_ReadRegister(INA219_Device_t *dev, uint8_t reg, uint16_t *pRxBuffer)
{
	uint8_t buffer[2];
    int8_t status = dev->read(dev->intf, reg, buffer, 2);
    *pRxBuffer = buffer[0] << 8 | buffer[1];
    return status;
}

int8_t INA219_WriteRegister(INA219_Device_t *dev, uint8_t reg, uint16_t *pTxBuffer)
{
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(*pTxBuffer >> 8);
	buffer[1] = (uint8_t) *pTxBuffer;
    int8_t status = dev->write(dev->intf, reg, buffer, 2);
	return status;
}

void INA219_SetBusVoltage(INA219_Device_t *dev, INA219_BusVoltageRange_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);

    buffer &= 0b1101111111111111;
    buffer |= (uint16_t)range << 13;
    dev->settings.bus_voltage_range = range;
    INA219_WriteRegister(dev, REG_CONFIG, &buffer);
}

void INA219_SetGain(INA219_Device_t *dev, INA219_Gain_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);

    buffer &= 0b1110011111111111;
    buffer |= (uint16_t)range << 11;
    dev->settings.gain = range;

    INA219_WriteRegister(dev, REG_CONFIG, &buffer);
}

void INA219_SetBusAdcResolution(INA219_Device_t *dev, INA219_ADCResolution_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);

    buffer &= 0b1111100001111111;
    buffer |= (uint16_t)range << 7;
    dev->settings.bus_adc_resolution = range;

    INA219_WriteRegister(dev, REG_CONFIG, &buffer);

}

void INA219_SetShuntAdcResolution(INA219_Device_t *dev, INA219_ADCResolution_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);

    buffer &= 0b1111111110000111;
    buffer |= (uint16_t)range << 3;
    dev->settings.shunt_adc_resolution = range;

    INA219_WriteRegister(dev, REG_CONFIG, &buffer);
}

void INA219_SetMode(INA219_Device_t *dev, INA219_OperatingMode_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);

    buffer &= 0b1111111111111000;
    buffer |= (uint16_t)range;
    dev->settings.mode = range;

    INA219_WriteRegister(dev, REG_CONFIG, &buffer);
}

void INA219_SetLsb(INA219_Device_t *dev)
{
    dev->current_lsb = 0.0001;
    dev->power_lsb = 20 * dev->current_lsb;
}

void INA219_CalibReg(INA219_Device_t *dev)
{
    uint16_t calib = (uint16_t)round(0.04096 / (dev->current_lsb * dev->r_shunt));
    INA219_WriteRegister(dev, REG_CALIBRATION, &calib);
}

int8_t INA219_ReadBusVoltage(INA219_Device_t *dev, float *pRxBuffer)
{
    uint16_t raw_voltage = 0;
    int8_t status = INA219_ReadRegister(dev, REG_BUSVOLTAGE, &raw_voltage);
    raw_voltage >>= 3;
    *pRxBuffer = raw_voltage * 4.0 * 0.001;
    return status;
}


int8_t INA219_ReadShuntVoltage(INA219_Device_t *dev, float *pRxBuffer)
{
	uint16_t raw_voltage = 0;
	int8_t status = INA219_ReadRegister(dev, REG_SHUNTVOLTAGE, &raw_voltage);
	*pRxBuffer = raw_voltage * 0.01;
	return status;  //in milivolts
}


int8_t INA219_ReadCurrentAmper(INA219_Device_t *dev, float *pRxBuffer)
{
    uint16_t current_register = 0;
    int8_t status = INA219_ReadRegister(dev, REG_CURRENT, &current_register);
    *pRxBuffer = ((float)current_register * dev->current_lsb) * 1000;  //miliamper
    return status;
}

int8_t INA219_ReadPowerWatt(INA219_Device_t *dev, float *pRxBuffer)
{
    uint16_t power_register = 0;
    int8_t status = INA219_ReadRegister(dev, REG_POWER, &power_register);
    *pRxBuffer = (float)power_register * dev->power_lsb;
    return status;
}

void INA219_Reset(INA219_Device_t *dev)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, &buffer);
    buffer |= 0x8000;
    INA219_WriteRegister(dev, REG_CONFIG, &buffer);
}





