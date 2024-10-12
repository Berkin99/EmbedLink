/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "sysconfig.h"
#include "systime.h"

#ifdef BMP388_I2C
#include "rtos.h"
#include "sensor_bmp388.h"
#include "i2c.h"
#include "uart.h"
#include "bmp3.h"
#include "estimator.h"
#include "mem.h"
#include "memory.h"

/* BMP3.h Variables */
static struct bmp3_dev device;
static struct bmp3_data data;
static uint8_t dev_addr = BMP3_ADDR_I2C_PRIM;

int8_t _sensorReadBMP388(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
int8_t _sensorWriteBMP388(uint8_t reg_addr, uint8_t *write_data, uint32_t len, void *intf_ptr);
void   _sensorDelayUsBMP388(uint32_t period, void *intf_ptr);
int8_t _sensorErrorConvertBMP388(int8_t err);

/* Sensor Object Variables */

#define BMI088_CAL_INTERVAL_MS 2000
#define BMI088_STABILIZE_MS    3000

static float barMean;
static float barVariance;
static measurement_t mbar;
static measurement_t mtmp;
static uint32_t idptr;
static uint8_t  isReady;
static uint8_t  isInit;
static uint8_t newbar;
static uint8_t newtmp;
static int8_t  status;

taskAllocateStatic(BMP388, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskBMP388(void* argv);

int8_t sensorInitBMP388(void){

    if(isInit == 1) return E_OVERWRITE;

    struct bmp3_settings settings;
    uint16_t settings_sel;
    int8_t result;

    /* Device Object */
    device.intf_ptr = &dev_addr;
    device.intf     = BMP3_I2C_INTF;
    device.read     = &_sensorReadBMP388;
    device.write    = &_sensorWriteBMP388;
    device.delay_us = &_sensorDelayUsBMP388;

    delay(100); /* bmp388 initialize delay */

    result = bmp3_init(&device);
    if(result != BMP3_OK) return _sensorErrorConvertBMP388(result);

    /* Device Settings */
    settings.int_settings.drdy_en = BMP3_ENABLE; /* IRQ Pin not connected :'( */
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

    status = bmp3_set_sensor_settings(settings_sel, &settings, &device);
    if(result != BMP3_OK) return _sensorErrorConvertBMP388(result);

    settings.op_mode = BMP3_MODE_NORMAL;
    status = bmp3_set_op_mode(&settings, &device);
    if(result != BMP3_OK) return _sensorErrorConvertBMP388(result);

    taskCreateStatic(BMP388, sensorTaskBMP388, NULL);
    isInit = 1;
    return OK;
}

int8_t sensorTestBMP388(void){
    uint8_t temp = 0;
    int8_t result = bmp3_get_regs(BMP3_REG_CHIP_ID, &temp, 1, &device);
    return _sensorErrorConvertBMP388(result);
}

void sensorTaskBMP388(void* argv){

    uint32_t caltim = millis() + BMI088_STABILIZE_MS;
    /* Need to pull the data about 3 seconds for BMP388 internal IIR filter stabilization */
    while(caltim > millis()){bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device);    	delay(100);}

    if(!sensorIsCalibratedBMP388()){
    	serialPrint("[!] BMP388 is not calibrated\n");
        sensorCalibrateBMP388();
    }

    mbar.type = STATE_PRESSURE;
    mtmp.type = STATE_TEMPERATURE;
    mbar.pressure.stdDev.x = sqrtf(barVariance);
    mtmp.temperature.stdDev.x = 0.1f;

    isReady = 1;

    while(1){
        status = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device);

        if(status == BMP3_OK){
            mbar.pressure.value    = (float) data.pressure / 100.0f;
            mtmp.temperature.value = (float) data.temperature;

            estimatorEnqueue(&mbar, FALSE);
            estimatorEnqueue(&mtmp, FALSE);

            newbar = 1;
            newtmp = 1;
        }

        delay(pdMS_TO_TICKS(1000 / sensorFreqBMP388));
    }
}

void sensorCalibrateBMP388(void){
	/* Calculate the mean and variance */
	float i = 0;
	barMean = 0;
	uint32_t intervalMs = millis() + BMI088_CAL_INTERVAL_MS;
	while(millis() < intervalMs){
        status = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device);
        if (status == BMP3_OK) {
    		barMean = (barMean * (i / (i+1))) + (data.pressure / 100.0f) * (1.0f / (i+1));
    		i++;
		}
        delay(10);
	}

	i = 0;
	barVariance = 0;
	intervalMs  = millis() + BMI088_CAL_INTERVAL_MS;
	while(millis() < intervalMs){
        status = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device);
        if (status == BMP3_OK) {
        	float dif = (data.pressure / 100.0f) - barMean;
        	barVariance = (barVariance * (i / (i + 1))) + (dif*dif) * (1.0f / (i + 1));
    		i++;
		}
        delay(10);
	}

	/* TODO: Memory Write */
	idptr = (uint32_t)SYS_ID(sensorNameBMP388);
	memoryMemUpload("BMP388", NULL);
}

int8_t sensorIsCalibratedBMP388(void){
	if(idptr == (uint32_t)SYS_ID(sensorNameBMP388))return TRUE;
	return FALSE;
}

int8_t sensorAcquireBMP388(measurement_t* plist, uint8_t n){
    if(status != BMP3_OK) return _sensorErrorConvertBMP388(status);
    int8_t i = 0;
    if(newbar){
        plist[i] = mbar;
        newbar = 0;
        i++;
    }
    if(newtmp){
        plist[i] = mtmp;
        newtmp = 0;
        i++;
    }
    return i;
}

int8_t sensorIsReadyBMP388(void){
    return isReady;
}

void sensorWaitDataReadyBMP388(void){
    while(!newbar || !newtmp) delay(2);
}

int8_t _sensorReadBMP388(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr){

    uint8_t status;

    status =  i2cTransmit(&BMP388_I2C, BMP3_ADDR_I2C_PRIM, &reg_addr, 1);
    status |= i2cReceive (&BMP388_I2C, BMP3_ADDR_I2C_PRIM, read_data, len);

    if(status == HAL_ERROR) return BMP3_E_COMM_FAIL; /*I2C dev error is -1 & BMP Error is 1*/
    return 0; /*BMP3_OK*/
}

int8_t _sensorWriteBMP388(uint8_t reg_addr, uint8_t *write_data, uint32_t len, void *intf_ptr){
    uint8_t status;
    status = i2cMemWrite(&BMP388_I2C, BMP3_ADDR_I2C_PRIM, reg_addr, write_data, len);

    if(status != HAL_OK) return BMP3_E_COMM_FAIL;    /* BMP ERROR */
    return 0;    /* BMP3_OK */
}

void _sensorDelayUsBMP388(uint32_t period, void *intf_ptr){
    delayUs(period);
}

int8_t _sensorErrorConvertBMP388(int8_t err){
    switch (err) {
        case BMP3_OK:                  return OK;
        case BMP3_E_NULL_PTR:          return E_NULL_PTR;
        case BMP3_E_INVALID_ODR_OSR_SETTINGS: return E_CONF_FAIL;
        case BMP3_E_CMD_EXEC_FAILED:   return E_COMM_FAIL;
        case BMP3_E_CONFIGURATION_ERR: return E_CONF_FAIL;
        case BMP3_E_INVALID_LEN:       return E_OVERFLOW;
        case BMP3_E_DEV_NOT_FOUND:     return E_NOT_FOUND;
        default:break;
    }
    return E_ERROR;
}

//MEM_GROUP_START(BMP388)
//MEM_ADD(MEM_UINT32, idptr,    &idptr)
//MEM_ADD(MEM_FLOAT,  mean,     &barMean)
//MEM_ADD(MEM_FLOAT,  variance, &barVariance)
//MEM_GROUP_STOP(BMP388)

#endif /* BMP388_I2C */
