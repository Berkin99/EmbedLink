/*
 *  ms5611.c
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

#include <stddef.h>
#include "ms5611.h"

#define MS5611_DEFAULT_OSR          (MS5611_ULTRA_HIGH_RES)

/* MS5611 Has only 5 basic commands: */

#define MS5611_CMD_RESET          	0x1E    /* Reset */
#define MS5611_CMD_READ_PROM       	0xA0    /* PROM (128 bit of calibration words) */
#define MS5611_CMD_CONV_D1         	0x40    /* D1 Conversion */
#define MS5611_CMD_CONV_D2        	0x50    /* D2 Conversion */
#define MS5611_CMD_ADC_READ      	0x00    /* Read ADC Result of the conversion (24 bit pressure / temperature) */

MS5611_Device_t MS5611_NewDevice(void* intf, MS5611_Intf_e intf_type, MS5611_Read_t readf, MS5611_Write_t writef, MS5611_Delay_t delayf)
{
    MS5611_Device_t dev = {
        .intf = intf,
        .intf_type = intf_type,
        .read = readf,
        .write = writef,
        .delay = delayf
    };
    return dev;
}

int8_t MS5611_Init(MS5611_Device_t* dev){
    MS5611_Reset(dev);
    MS5611_SetOSRate(dev, MS5611_DEFAULT_OSR);
    MS5611_InitConstants(dev, 0);
    dev->delay(20);
    return MS5611_PROM(dev);
}

int8_t MS5611_Test(MS5611_Device_t* dev){
	uint8_t temp;
    return dev->read(dev->intf, MS5611_CMD_READ_PROM, &temp, 1);
}

void MS5611_Reset(MS5611_Device_t* dev){
    dev->write(dev->intf, MS5611_CMD_RESET, NULL, 0);
}

void   MS5611_InitConstants(MS5611_Device_t* dev, int8_t mathMode)
{
	dev->config.C[0] = 1;

	dev->config.C[1] = 32768L;          	/* Pressure sensitivity    : SENSt1     = C[1] * 2^15 */
	dev->config.C[2] = 65536L;          	/* Pressure offset         : OFFt1      = C[2] * 2^16 */
	dev->config.C[3] = 3.90625E-3;      	/* Temperature coef. of C1 : TCS        = C[3] / 2^8  */
	dev->config.C[4] = 7.8125E-3;       	/* Temperature coef. of C2 : TCO        = C[4] / 2^7  */
	dev->config.C[5] = 256;             	/* Reference temperature   : Tref       = C[5] * 2^8  */
	dev->config.C[6] = 1.1920928955E-7; 	/* Temperature coefficient : TEMPSENS   = C[6] / 2^23 */

	if (mathMode)
	{
		dev->config.C[1] = 65536L;
		dev->config.C[2] = 131072L;
		dev->config.C[3] = 7.8125E-3;
		dev->config.C[4] = 1.5625e-2;
	}
}

int8_t MS5611_PROM(MS5611_Device_t* dev){

    int8_t rslt = MS5611_OK;

    for (uint8_t reg = 0; reg < 7; reg++)
    {
      uint16_t tmp = MS5611_ReadPROM(dev, reg);
      dev->config.C[reg] *= tmp;

      if ((reg > 0) && (tmp == 0)) rslt = MS5611_ERROR;
    }
    return rslt;
}

uint16_t MS5611_ReadPROM(MS5611_Device_t* dev, uint8_t reg){
	uint8_t temp[2];
	uint8_t mem = (MS5611_CMD_READ_PROM + (reg * 2)); /* 0xA0 to 0xAE 6 coefficient */
    dev->read(dev->intf, mem, temp, 2);
    return (temp[0] << 8) | temp[1];
}

void MS5611_SetOSRate(MS5611_Device_t* dev, MS5611_OSRate_t osr){

    const uint8_t osrToConversitonTime [] = {
        [MS5611_ULTRA_LOW_POWER] = 1,
        [MS5611_LOW_POWER]  = 2,
        [MS5611_STANDARD]   = 3,
        [MS5611_HIGH_RES]   = 5,
        [MS5611_ULTRA_HIGH_RES] = 10,
    };
	dev->config.ct =  osrToConversitonTime[osr];
    dev->config.osRate = osr;
}

MS5611_OSRate_t MS5611_GetOSRate(MS5611_Device_t* dev){
	return dev->config.osRate;
}

int8_t MS5611_GetData(MS5611_Device_t* dev, float* pTemp, float* pPress){

	int8_t rslt = MS5611_OK;
    uint32_t D1, D2;

    MS5611_Convert(dev, MS5611_CMD_CONV_D1);
    dev->delay(dev->config.ct);
    rslt |= MS5611_AdcRead(dev, &D1);

    MS5611_Convert(dev, MS5611_CMD_CONV_D2);
    dev->delay(dev->config.ct);
    rslt |= MS5611_AdcRead(dev, &D2);

    MS5611_Data_t data = MS5611_RawDataProcess(dev, D1, D2, 1);
	*pTemp = (float)data.temperature / 100.0f;
	*pPress = (float)data.pressure / 100.0f;

	return rslt;
}

void MS5611_Convert(MS5611_Device_t* dev, const uint8_t addr){
	uint8_t cmd_convert = addr + (dev->config.osRate * 2);
    dev->write(dev->intf, cmd_convert, NULL, 0);
}

int8_t MS5611_AdcRead(MS5611_Device_t* dev, uint32_t* buffer){
	int8_t rslt;
    uint8_t temp [3];

    rslt = dev->read(dev->intf, MS5611_CMD_ADC_READ, temp, 3);
    *buffer = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];
    return rslt;
}

MS5611_Data_t MS5611_RawDataProcess(MS5611_Device_t* dev, uint32_t D1 , uint32_t D2, int8_t compensation){
	MS5611_Data_t data;

	float dT = D2 - dev->config.C[5];
	data.temperature = (int32_t)(2000 + (dT * dev->config.C[6]));

	float offset =  dev->config.C[2] + (dT * dev->config.C[4]);
	float sens = dev->config.C[1] + (dT * dev->config.C[3]);

	if (compensation)
	{
		if (data.temperature < 2000)
		{
			float T2 = dT * dT * 4.6566128731E-10;
			float t = (data.temperature - 2000) * (data.temperature - 2000);
			float offset2 = 2.5 * t;
			float sens2 = 1.25 * t;
			if (data.temperature < -1500)
			{
				t = (float)((data.temperature + 1500) * (data.temperature + 1500));
				offset2 += 7 * t;
				sens2 += 5.5 * t;
			}
			data.temperature -= T2;
			offset -= offset2;
			sens -= sens2;
		}
	}

	data.pressure = (D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;
	return data;
}
